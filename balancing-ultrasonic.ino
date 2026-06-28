#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

// ===================== PIN =====================
#define LED_PIN    2

// Motor
#define PIN_ENA 25
#define PIN_IN1 26
#define PIN_IN2 27
#define PIN_IN3 12
#define PIN_IN4 14
#define PIN_ENB 33

#define I2C_SDA    21
#define I2C_SCL    22

// Ultrasonik
#define TRIG_FRONT 23
#define ECHO_FRONT 35
#define TRIG_RIGHT 18
#define ECHO_RIGHT 19
#define TRIG_LEFT  16
#define ECHO_LEFT  17

// ===================== PWM =====================
#define PWM_FREQ    1000   // Sweet spot untuk L298N
#define PWM_RES      8
#define PWM_SLEW    15     // Lebih halus dari 20, kurangi hunting

// ===================== MPU6050 =====================
uint8_t MPU_ADDR = 0x68;
#define REG_PWR_MGMT_1  0x6B
#define ACCEL_SCALE     16384.0f
#define GYRO_SCALE      131.0f

// ===================== CONTROL & PID =====================
#define CONTROL_PERIOD_US  4000   // PID: 4ms (250Hz) — jantung robot
#define NAV_PERIOD_MS      50     // Navigasi: 50ms (20Hz) — otak robot
#define DEBUG_PRINT_MS     100

volatile float Kp = 35.0f;
volatile float Ki = 300.0f;
volatile float Kd = 2.0f;

float baseSetpoint = -2.3f;

float pidI             = 0.0f;
float pitchDeg         = 0.0f;
float gyroOffsetY      = 0.0f;
float compAngleY       = 0.0f;
float accAngleYFiltered = 0.0f;

int      lastPWMLeft   = 0;
int      lastPWMRight  = 0;
uint32_t lastControlUs = 0;
uint32_t lastNavMs     = 0;
uint32_t lastPrintMs   = 0;

// ===================== NAVIGATION STATE =====================
enum NavMode { MODE_MANUAL, MODE_AUTO_WALL };
volatile NavMode currentMode = MODE_MANUAL;

// [FIX HUNTING] targetMoveOffset default = 0, di mode manual SELALU 0
volatile float targetMoveOffset  = 0.0f;
volatile float currentMoveOffset = 0.0f;

// [FIX HUNTING] Slew rate diperhalus agar tidak overshooting
float moveSlewRate = 0.003f;

volatile int   turnOffset     = 0;
volatile int   autoTurnOffset = 0;
volatile unsigned long lastCommandMs  = 0;
const    unsigned long commandTimeout = 180;

// ===================== ULTRASONIC =====================
// [FIX MANUAL] Flag aktif/nonaktif ultrasonik
volatile bool  ultrasonicEnabled = false; // default MATI saat mode manual
volatile float distFront = 999.0f;
volatile float distRight = 999.0f;
volatile float distLeft  = 999.0f;
uint32_t lastUSMs    = 0;
uint8_t  usSensorStep = 0;
const uint32_t US_INTERVAL_MS = 25;

// ===================== WIFI & WEB =====================
const char* apSSID     = "Balance_Pro";
const char* apPassword = "12345678";
WebServer server(80);
TaskHandle_t TaskCore0;

// ===================== HTML WEB UI =====================
String makePage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Robot Balancing Control</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; text-align: center; background: #e0e5ec; margin: 0; padding: 20px; color: #333; }
    .wrap { max-width: 400px; margin: auto; background: white; padding: 25px; border-radius: 20px; box-shadow: 0 10px 30px rgba(0,0,0,0.1); }
    h2 { margin-top: 0; color: #2c3e50; font-size: 26px; font-weight: 800; }
    .section { margin-bottom: 25px; padding: 15px; background: #f8f9fa; border-radius: 15px; border: 1px solid #eee; }
    .row { display: flex; justify-content: center; align-items: center; margin: 8px 0; }
    .btn-dir { width: 75px; height: 65px; margin: 5px; font-size: 15px; font-weight: bold; border: none; border-radius: 12px; background: #3498db; color: white; touch-action: manipulation; cursor: pointer; box-shadow: 0 5px 0 #2980b9; }
    .btn-dir:active { transform: translateY(5px); box-shadow: 0 0 0 #2980b9; }
    .btn-stop { background: #e74c3c; box-shadow: 0 5px 0 #c0392b; }
    .btn-mode { width: 45%; height: 50px; background: #f39c12; margin: 5px; box-shadow: 0 5px 0 #e67e22; }
    .btn-mode-manual { background: #8e44ad; box-shadow: 0 5px 0 #732d91; }
    .input-group { display: flex; align-items: center; justify-content: space-between; margin: 12px 0; }
    .input-group input { width: 55%; padding: 10px; border: 2px solid #bdc3c7; border-radius: 8px; font-size: 16px; text-align: center; font-weight: bold; }
    .btn-save { width: 100%; padding: 14px; margin-top: 15px; font-size: 16px; font-weight: bold; border: none; border-radius: 10px; background: #2ecc71; color: white; cursor: pointer; box-shadow: 0 5px 0 #27ae60; }
  </style>
</head>
<body>
  <div class="wrap">
    <h2>Robot Balancing</h2>

    <div class="section">
      <h3>Pemilihan Mode</h3>
      <div class="row">
        <button class="btn-dir btn-mode" onclick="send('/modeAuto')">Mode Auto</button>
        <button class="btn-dir btn-mode btn-mode-manual" onclick="send('/modeManual')">Manual</button>
      </div>
    </div>

    <div class="section">
      <h3>Tuning PID</h3>
      <div class="input-group"><label>Kp</label><input type="number" id="kp" step="0.1" value=")rawliteral";
  html += String(Kp); html += R"rawliteral("></div>
      <div class="input-group"><label>Ki</label><input type="number" id="ki" step="0.1" value=")rawliteral";
  html += String(Ki); html += R"rawliteral("></div>
      <div class="input-group"><label>Kd</label><input type="number" id="kd" step="0.1" value=")rawliteral";
  html += String(Kd); html += R"rawliteral("></div>
      <button class="btn-save" onclick="updatePID()">Simpan PID</button>
    </div>

    <div class="section">
      <h3>Kontrol Gerak (Manual)</h3>
      <div class="row"><button class="btn-dir" onpointerdown="send('/forward')" onpointerup="send('/stop')">Maju</button></div>
      <div class="row">
        <button class="btn-dir" onpointerdown="send('/left')" onpointerup="send('/stop')">Kiri</button>
        <button class="btn-dir btn-stop" onclick="send('/stop')">Stop</button>
        <button class="btn-dir" onpointerdown="send('/right')" onpointerup="send('/stop')">Kanan</button>
      </div>
      <div class="row"><button class="btn-dir" onpointerdown="send('/backward')" onpointerup="send('/stop')">Mundur</button></div>
    </div>
  </div>
  <script>
    function send(path) { fetch(path).catch(()=>{}); }
    function updatePID() {
      fetch(`/updatePID?kp=${document.getElementById('kp').value}&ki=${document.getElementById('ki').value}&kd=${document.getElementById('kd').value}`)
        .then(r => r.ok ? alert("PID Diperbarui!") : alert("Gagal"));
    }
  </script>
</body>
</html>)rawliteral";
  return html;
}

// ===================== WEB HANDLERS =====================
void handleRoot()      { server.send(200, "text/html", makePage()); }
void handleUpdatePID() {
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
  server.send(200, "text/plain", "OK");
}

// [FIX MANUAL] Saat masuk mode AUTO -> ultrasonik aktif
void handleModeAuto() {
  currentMode       = MODE_AUTO_WALL;
  ultrasonicEnabled = true;   // Hidupkan ultrasonik
  server.send(200);
}

// [FIX MANUAL] Saat masuk mode MANUAL -> matikan ultrasonik + reset semua offset
void handleModeManual() {
  currentMode         = MODE_MANUAL;
  ultrasonicEnabled   = false;  // Matikan ultrasonik
  targetMoveOffset    = 0.0f;   // [FIX HUNTING] Paksa offset ke 0
  currentMoveOffset   = 0.0f;   // [FIX HUNTING] Langsung reset, tidak di-slew
  autoTurnOffset      = 0;
  turnOffset          = 0;
  pidI                = 0.0f;   // [FIX HUNTING] Reset integral agar tidak ada bias
  // Reset jarak ke nilai aman agar tidak ada logika navigasi sisa
  distFront = 999.0f;
  distRight = 999.0f;
  distLeft  = 999.0f;
  server.send(200);
}

void setManualCommand(float move, int turn) {
  targetMoveOffset = move;
  turnOffset       = turn;
  lastCommandMs    = millis();
}
void handleForward()  { setManualCommand( 0.8f,   0); server.send(200); }
void handleBackward() { setManualCommand(-0.8f,   0); server.send(200); }
void handleLeft()     { setManualCommand( 0.0f,  15); server.send(200); }
void handleRight()    { setManualCommand( 0.0f, -15); server.send(200); }

// [FIX HUNTING] handleStop reset semua ke 0 dan reset integral
void handleStop() {
  targetMoveOffset  = 0.0f;
  currentMoveOffset = 0.0f;  // Langsung snap ke 0, tidak di-slew
  turnOffset        = 0;
  autoTurnOffset    = 0;
  pidI              = 0.0f;  // Reset integral biar tidak ada sisa dorongan
  lastCommandMs     = millis();
  server.send(200);
}

// ===================== I2C & MPU6050 =====================
void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(value); Wire.endTransmission();
}
bool readBytes(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)len) != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}
bool mpu6050Begin() {
  delay(150); Wire.beginTransmission(0x68);
  MPU_ADDR = (Wire.endTransmission() == 0) ? 0x68 : 0x69;
  writeReg(REG_PWR_MGMT_1, 0x00); delay(100);
  return true;
}
bool readMPU6050(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t raw[14]; if (!readBytes(0x3B, raw, 14)) return false;
  ax = (int16_t)((raw[0]<<8)|raw[1])  / ACCEL_SCALE;
  ay = (int16_t)((raw[2]<<8)|raw[3])  / ACCEL_SCALE;
  az = (int16_t)((raw[4]<<8)|raw[5])  / ACCEL_SCALE;
  gx = (int16_t)((raw[8]<<8)|raw[9])  / GYRO_SCALE;
  gy = (int16_t)((raw[10]<<8)|raw[11])/ GYRO_SCALE;
  gz = (int16_t)((raw[12]<<8)|raw[13])/ GYRO_SCALE;
  return true;
}

// ===================== SENSOR & CORE 0 =====================
float getDistanceNB(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10); digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 12000);
  return (dur == 0) ? 999.0f : dur * 0.01715f;
}

void core0Loop(void * parameter) {
  for (;;) {
    server.handleClient();

    // [FIX MANUAL] Ultrasonik hanya dibaca jika flag aktif (mode AUTO)
    if (ultrasonicEnabled && (millis() - lastUSMs >= US_INTERVAL_MS)) {
      lastUSMs = millis();
      if      (usSensorStep == 0) { distFront = getDistanceNB(TRIG_FRONT, ECHO_FRONT); usSensorStep = 1; }
      else if (usSensorStep == 1) { distRight = getDistanceNB(TRIG_RIGHT, ECHO_RIGHT); usSensorStep = 2; }
      else if (usSensorStep == 2) { distLeft  = getDistanceNB(TRIG_LEFT,  ECHO_LEFT);  usSensorStep = 0; }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ===================== MOTOR CONTROL =====================
int applySlew(int target, int prev) {
  if ((target > 0 && prev < 0) || (target < 0 && prev > 0)) return target;
  if (target - prev >  PWM_SLEW) return prev + PWM_SLEW;
  if (target - prev < -PWM_SLEW) return prev - PWM_SLEW;
  return target;
}
void setMotorsLR(int leftPWM, int rightPWM) {
  lastPWMLeft  = applySlew(leftPWM,  lastPWMLeft);
  lastPWMRight = applySlew(rightPWM, lastPWMRight);

  digitalWrite(PIN_IN1, lastPWMLeft > 0 ? LOW  : HIGH);
  digitalWrite(PIN_IN2, lastPWMLeft > 0 ? HIGH : LOW);
  if (lastPWMLeft == 0) { digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW); }
  ledcWrite(PIN_ENA, abs(lastPWMLeft));

  digitalWrite(PIN_IN3, lastPWMRight > 0 ? HIGH : LOW);
  digitalWrite(PIN_IN4, lastPWMRight > 0 ? LOW  : HIGH);
  if (lastPWMRight == 0) { digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW); }
  ledcWrite(PIN_ENB, abs(lastPWMRight));
}

// ===================== ARSITEKTUR NAVIGASI =====================
// Dipanggil 20Hz (50ms) — tidak ganggu PID
void updateNavigation(float dt) {

  if (currentMode == MODE_MANUAL) {
    // [FIX MANUAL] Mode manual: ultrasonik dimatikan, offset selalu 0
    autoTurnOffset = 0;

    // Timeout tombol -> kembali diam di tempat
    if (millis() - lastCommandMs > commandTimeout) {
      targetMoveOffset = 0.0f;
      turnOffset       = 0;
    }
    // [FIX MANUAL] TIDAK ada pembacaan distFront di sini sama sekali

  } else if (currentMode == MODE_AUTO_WALL) {

    float batasDepan   = 25.0f;
    float batasSamping = 18.0f;

    // --- PRIORITAS 1: PROTEKSI MENTOK FISIK ---
    if (distFront < 10.0f) {
      targetMoveOffset = 0.8f;
      autoTurnOffset   = (distRight > distLeft) ? 200 : -200;
    }
    else if (distRight < 5.0f) {
      targetMoveOffset = -0.3f; autoTurnOffset = -30;
    }
    else if (distLeft < 5.0f) {
      targetMoveOffset = -0.3f; autoTurnOffset =  30;
    }

    // --- PRIORITAS 2: DEPAN MENTOK, PILIH JALAN ---
    else if (distFront <= batasDepan) {
      targetMoveOffset = 0.3f;
      if      (distRight > batasSamping && distLeft <= batasSamping)  { autoTurnOffset =  190; }
      else if (distLeft  > batasSamping && distRight <= batasSamping) { autoTurnOffset = -190; }
      else if (distRight > batasSamping && distLeft  > batasSamping)  { autoTurnOffset = (distRight > distLeft) ? 190 : -190; }
      else { targetMoveOffset = 0.6f; autoTurnOffset = 190; }
    }

    // --- PRIORITAS 3: LURUS ---
    else {
      targetMoveOffset = -0.6f;
      autoTurnOffset   = 0;
    }
  }

  // --- SLEW RATE currentMoveOffset ---
  float scaledSlewRate = moveSlewRate * (dt / 0.004f);
  if (currentMoveOffset < targetMoveOffset) {
    currentMoveOffset += scaledSlewRate;
    if (currentMoveOffset > targetMoveOffset) currentMoveOffset = targetMoveOffset;
  } else if (currentMoveOffset > targetMoveOffset) {
    currentMoveOffset -= scaledSlewRate;
    if (currentMoveOffset < targetMoveOffset) currentMoveOffset = targetMoveOffset;
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);

  ledcAttach(PIN_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_ENB, PWM_FREQ, PWM_RES);
  Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(400000);
  mpu6050Begin();

  // Kalibrasi Gyro
  Serial.println("Kalibrasi gyro, jangan gerakkan robot...");
  float sumGy = 0, sumPitch = 0;
  for (int i = 0; i < 300; i++) {
    float ax, ay, az, gx, gy, gz;
    if (readMPU6050(ax, ay, az, gx, gy, gz)) {
      sumGy    += gy;
      sumPitch += atan2f(-ax, az) * 180.0f / PI;
    }
    delay(2);
  }
  gyroOffsetY       = sumGy / 300.0f;
  compAngleY        = sumPitch / 300.0f;
  accAngleYFiltered = compAngleY;
  Serial.print("Kalibrasi selesai. Offset gyro: "); Serial.println(gyroOffsetY);

  WiFi.softAP(apSSID, apPassword);
  server.on("/",           handleRoot);
  server.on("/updatePID",  handleUpdatePID);
  server.on("/modeAuto",   handleModeAuto);
  server.on("/modeManual", handleModeManual);
  server.on("/forward",    handleForward);
  server.on("/backward",   handleBackward);
  server.on("/left",       handleLeft);
  server.on("/right",      handleRight);
  server.on("/stop",       handleStop);
  server.begin();

  xTaskCreatePinnedToCore(core0Loop, "Core0", 10000, NULL, 1, &TaskCore0, 0);
  lastControlUs = micros();
  lastNavMs     = millis();
}

// ===================== MAIN LOOP =====================
void loop() {
  uint32_t nowUs = micros();
  if (nowUs - lastControlUs < CONTROL_PERIOD_US) return;
  float dt = constrain((nowUs - lastControlUs) * 1e-6f, 0.001f, 0.02f);
  lastControlUs = nowUs;

  // --- NAVIGASI: 20Hz, tidak ganggu PID ---
  if (millis() - lastNavMs >= NAV_PERIOD_MS) {
    lastNavMs = millis();
    updateNavigation(dt);
  }

  // --- PID: 250Hz, selalu tepat waktu ---

  // 1. Baca sudut
  float ax, ay, az, gx, gy, gz;
  if (!readMPU6050(ax, ay, az, gx, gy, gz)) return;

  float gyUse     = gy - gyroOffsetY;
  float accAngleY = atan2f(-ax, az) * 180.0f / PI;

  accAngleYFiltered = 0.85f * accAngleYFiltered + 0.15f * accAngleY;
  compAngleY = 0.99f * (compAngleY + gyUse * dt) + 0.01f * accAngleYFiltered;
  pitchDeg   = compAngleY;

  // 2. Sensor jatuh
  if (fabs(pitchDeg) > 45.0f) {
    setMotorsLR(0, 0);
    pidI              = 0.0f;
    currentMoveOffset = 0.0f;
    targetMoveOffset  = 0.0f;
    return;
  }

  // 3. Hitung PID
  float err = (baseSetpoint + currentMoveOffset) - pitchDeg;

  // [FIX HUNTING] Anti-windup lebih ketat: integral dibatasi kecil di mode manual
  float iLimit = (currentMode == MODE_MANUAL) ? 1.5f : 3.0f;
  pidI = constrain(pidI + (err * dt), -iLimit, iLimit);

  float pwm_f = Kp * err + Ki * pidI - Kd * gyUse;
  int   pwm   = (int)pwm_f;

  // Dead-band
  if (pwm > 0  && pwm <  80) pwm =  80;
  if (pwm < 0  && pwm > -80) pwm = -80;

  // [FIX HUNTING] Dead-zone lebih lebar di mode manual agar robot tenang
  float errThreshold = (currentMode == MODE_MANUAL) ? 0.15f : 0.08f;
  if (fabs(err) < errThreshold) pwm = 0;

  // 4. Eksekusi roda
  int steerTrim = 0; // Isi + atau - jika maju masih penceng
  int finalTurn = constrain(turnOffset + autoTurnOffset + steerTrim, -220, 220);

  setMotorsLR(
    constrain(pwm + finalTurn, -255, 255),
    constrain(pwm - finalTurn, -255, 255)
  );

  // 5. Monitor serial
  if (millis() - lastPrintMs >= DEBUG_PRINT_MS) {
    lastPrintMs = millis();
    Serial.print("Mode:");   Serial.print(currentMode == MODE_AUTO_WALL ? "AUTO" : "MANUAL");
    Serial.print(" Pitch:");  Serial.print(pitchDeg, 2);
    Serial.print(" Err:");    Serial.print(err, 2);
    Serial.print(" PWM:");    Serial.print(pwm);
    Serial.print(" US:");     Serial.print(ultrasonicEnabled ? "ON" : "OFF");
    if (ultrasonicEnabled) {
      Serial.print(" F:"); Serial.print(distFront);
      Serial.print(" R:"); Serial.print(distRight);
      Serial.print(" L:"); Serial.print(distLeft);
    }
    Serial.println();
  }
}
