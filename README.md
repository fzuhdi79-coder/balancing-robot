# 🤖 Balance Pro — Self-Balancing Autonomous Robot

Robot dua roda yang mampu menyeimbangkan diri secara real-time menggunakan PID controller, sekaligus dapat bernavigasi secara otonom menghindari rintangan dengan sensor ultrasonik. Dikendalikan via Wi-Fi melalui web interface bawaan.

---

## 📋 Daftar Isi

- [Fitur Utama](#-fitur-utama)
- [Hardware](#-hardware)
- [Diagram Pin](#-diagram-pin)
- [Dependensi](#-dependensi)
- [Cara Upload](#-cara-upload)
- [Cara Penggunaan](#-cara-penggunaan)
- [Tuning PID](#-tuning-pid)
- [Arsitektur Software](#-arsitektur-software)
- [Logika Navigasi Otonom](#-logika-navigasi-otonom)
- [Troubleshooting](#-troubleshooting)
- [Konfigurasi Cepat](#-konfigurasi-cepat)

---

## ✨ Fitur Utama

- **Self-balancing** dengan complementary filter (Accelerometer + Gyroscope MPU6050)
- **PID Controller** yang dapat di-tune secara real-time via web browser
- **Mode Manual** — kontrol arah via tombol di web UI
- **Mode Otonom** — navigasi otomatis menghindari rintangan 3 arah (depan, kanan, kiri)
- **Dual-core ESP32** — PID berjalan di Core 1 (250Hz), sensor & web server di Core 0
- **Sensor aman** — ultrasonik hanya aktif saat mode AUTO, tidak mengganggu keseimbangan saat manual

---

## 🔧 Hardware

| Komponen | Jumlah | Keterangan |
|---|---|---|
| ESP32 DevKit | 1 | Otak utama (dual-core 240MHz) |
| MPU6050 | 1 | IMU 6-axis (I2C) |
| L298N Motor Driver | 1 | Driver motor DC |
| Motor DC + Roda | 2 | Motor kiri & kanan |
| HC-SR04 | 3 | Sensor ultrasonik (depan, kanan, kiri) |
| Baterai LiPo / Li-ion | 1 | Disarankan 7.4V 2S |
| Regulator 5V | 1 | Untuk supply ESP32 & sensor |

---

## 📌 Diagram Pin

```
ESP32               Perangkat
─────────────────────────────────────────
GPIO 21 (SDA)  →    MPU6050 SDA
GPIO 22 (SCL)  →    MPU6050 SCL

GPIO 25 (ENA)  →    L298N ENA  (PWM Motor Kiri)
GPIO 26 (IN1)  →    L298N IN1
GPIO 27 (IN2)  →    L298N IN2
GPIO 12 (IN3)  →    L298N IN3
GPIO 14 (IN4)  →    L298N IN4
GPIO 33 (ENB)  →    L298N ENB  (PWM Motor Kanan)

GPIO 23 (TRIG) →    HC-SR04 Depan  TRIG
GPIO 35 (ECHO) ←    HC-SR04 Depan  ECHO
GPIO 18 (TRIG) →    HC-SR04 Kanan  TRIG
GPIO 19 (ECHO) ←    HC-SR04 Kanan  ECHO
GPIO 16 (TRIG) →    HC-SR04 Kiri   TRIG
GPIO 17 (ECHO) ←    HC-SR04 Kiri   ECHO

GPIO 2         →    LED indikator (onboard)
```

> ⚠️ **Catatan:** GPIO 35 adalah input-only pada ESP32, sudah sesuai untuk ECHO.

---

## 📦 Dependensi

Library yang dibutuhkan (dapat diinstall via Arduino Library Manager atau PlatformIO):

| Library | Keterangan |
|---|---|
| `Arduino.h` | Core Arduino / ESP32 |
| `Wire.h` | Komunikasi I2C untuk MPU6050 |
| `WiFi.h` | Built-in ESP32 |
| `WebServer.h` | Built-in ESP32 |
| `math.h` | Fungsi `atan2f`, `fabs` |

Tidak ada library eksternal tambahan yang diperlukan.

---

## 🚀 Cara Upload

### Arduino IDE
1. Install **ESP32 board package** via Board Manager (`https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`)
2. Pilih board: **ESP32 Dev Module**
3. Set **CPU Frequency: 240MHz**
4. Set **Partition Scheme: Default 4MB with spiffs** (atau lebih besar)
5. Upload kode

### PlatformIO
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
```

---

## 📱 Cara Penggunaan

1. Nyalakan robot dan **letakkan di posisi tegak**
2. Tunggu kalibrasi gyro selesai (~3 detik, LED akan menyala)
3. Hubungkan perangkat (HP/laptop) ke Wi-Fi:
   - **SSID:** `Balance_Pro`
   - **Password:** `12345678`
4. Buka browser, akses: `http://192.168.4.1`
5. Pilih mode operasi:
   - **Manual** — kontrol tombol arah
   - **Mode Auto** — robot jelajah otomatis

### Tombol Kontrol (Mode Manual)

| Tombol | Fungsi |
|---|---|
| **Maju** | Robot condong ke depan (hold) |
| **Mundur** | Robot condong ke belakang (hold) |
| **Kiri** | Putar ke kiri (hold) |
| **Kanan** | Putar ke kanan (hold) |
| **Stop** | Berhenti & reset semua offset |

> Semua tombol arah bekerja secara **hold** (tekan dan tahan). Robot berhenti otomatis saat tombol dilepas, atau setelah 180ms timeout.

---

## 🎛️ Tuning PID

Parameter PID dapat diubah **real-time** tanpa perlu upload ulang melalui form di web UI.

| Parameter | Default | Fungsi |
|---|---|---|
| **Kp** | 35.0 | Respons proporsional — naikkan jika robot lambat merespons |
| **Ki** | 300.0 | Koreksi steady-state — naikkan jika robot terus-terusan miring |
| **Kd** | 2.0 | Redaman — naikkan jika robot bergoyang/osilasi |

### Panduan Tuning Bertahap

```
1. Set Ki=0, Kd=0. Naikkan Kp pelan-pelan sampai robot berdiri tapi masih osilasi.
2. Naikkan Kd sampai osilasi berkurang (robot lebih "tenang").
3. Naikkan Ki pelan-pelan sampai robot tidak lagi miring permanen ke satu sisi.
4. Fine-tune baseSetpoint di kode jika robot cenderung maju/mundur terus.
```

### Parameter Penting di Kode

```cpp
float baseSetpoint = -2.3f;  // Titik keseimbangan dalam derajat
                              // Negatif = robot sedikit condong ke depan
                              // Sesuaikan ±0.5° sampai robot diam di tempat
```

---

## 🏗️ Arsitektur Software

```
┌─────────────────────────────────────────────────────────┐
│                        ESP32                            │
│                                                         │
│   CORE 1 (loop)              CORE 0 (core0Loop)         │
│   ─────────────────          ─────────────────          │
│   PID Control (250Hz)        Web Server                 │
│   Angle Calculation          Ultrasonic Sensor          │
│   Motor Output               (25ms per sensor)          │
│   Navigation Update          Flag: ultrasonicEnabled    │
│   (20Hz via timer)                                      │
└─────────────────────────────────────────────────────────┘
```

### Penjelasan Dual-Core

- **Core 1** menjalankan `loop()` — PID dan kalkulasi sudut berjalan setiap 4ms (250Hz). Ini adalah tugas kritis yang tidak boleh terganggu.
- **Core 0** menjalankan `core0Loop()` — melayani HTTP request dan membaca sensor ultrasonik secara bergantian. Ultrasonik **hanya aktif saat Mode AUTO** (`ultrasonicEnabled = true`) untuk menghindari interferensi saat mode manual.

---

## 🧭 Logika Navigasi Otonom

Saat **Mode Auto** aktif, navigasi berjalan dengan prioritas bertingkat:

```
PRIORITAS 1 — Benturan Fisik (kritis)
  distFront < 10cm  → mundur + belok keras
  distRight < 5cm   → mundur + geser kiri
  distLeft  < 5cm   → mundur + geser kanan

PRIORITAS 2 — Depan Terhalang (25cm)
  Pilih arah ke sisi yang lebih lapang (kanan/kiri > 18cm)
  Jika keduanya sempit → tetap maju sambil belok

PRIORITAS 3 — Jalan Bebas
  Maju lurus dengan autoTurnOffset = 0
```

### Threshold Jarak (dapat disesuaikan di kode)

| Variabel | Nilai | Arti |
|---|---|---|
| `batasDepan` | 25 cm | Mulai manuver depan |
| `batasSamping` | 18 cm | Sisi dianggap "terbuka" |
| Darurat depan | 10 cm | Mundur paksa |
| Darurat samping | 5 cm | Hindari mepet |

---

## 🔍 Troubleshooting

| Gejala | Kemungkinan Penyebab | Solusi |
|---|---|---|
| Robot langsung jatuh | `baseSetpoint` tidak tepat | Sesuaikan ±0.5° per coba |
| Robot osilasi/bergoyang | Kd terlalu kecil | Naikkan Kd bertahap |
| Robot miring terus ke satu sisi | Ki terlalu kecil | Naikkan Ki, atau koreksi `baseSetpoint` |
| Motor tidak berputar | Dead-band PWM terlalu tinggi | Turunkan threshold 80 jika motor butuh lebih |
| Web UI tidak muncul | Belum terhubung ke WiFi `Balance_Pro` | Pastikan password `12345678` |
| Sensor ultrasonik selalu 999 | Mode masih MANUAL | Klik "Mode Auto" di web UI |
| Robot tidak mau belok di mode auto | `autoTurnOffset` tidak cukup besar | Naikkan nilai turn (190 → 220) di kode |
| MPU6050 tidak terdeteksi | Alamat I2C salah | Cek apakah pin AD0 HIGH (→ 0x69) atau LOW (→ 0x68) |

---

## ⚙️ Konfigurasi Cepat

Semua parameter yang sering diubah dikumpulkan di bagian atas kode:

```cpp
// ── PID ──────────────────────────────
volatile float Kp = 35.0f;
volatile float Ki = 300.0f;
volatile float Kd = 2.0f;
float baseSetpoint = -2.3f;   // titik keseimbangan (°)

// ── NAVIGASI ─────────────────────────
float batasDepan   = 25.0f;   // cm: trigger manuver depan
float batasSamping = 18.0f;   // cm: threshold sisi terbuka

// ── WIFI ─────────────────────────────
const char* apSSID     = "Balance_Pro";
const char* apPassword = "12345678";

// ── MOTOR ────────────────────────────
#define PWM_FREQ  1000   // Hz: sweet spot L298N
#define PWM_SLEW  15     // maks perubahan PWM per siklus
```

---

## 📄 Lisensi

Proyek ini bersifat open source untuk keperluan edukasi dan riset robotika.  
Bebas dimodifikasi dan didistribusikan dengan menyertakan atribusi.

---

*Developed with ❤️ by [Zudi Jago/ fzuhdi79-coder]*
