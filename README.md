# PETSA - Cattle Health Monitoring System

[![Arduino](https://img.shields.io/badge/Arduino-ESP8266-blue.svg)](https://www.arduino.cc/)
[![Azure IoT](https://img.shields.io/badge/Azure-IoT%20Hub-0078d4.svg)](https://azure.microsoft.com/en-us/services/iot-hub/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

## 📋 Deskripsi

PETSA (Personal Health Monitoring System) adalah sistem pemantauan kesehatan berbasis IoT yang menggunakan ESP8266 untuk mengumpulkan data vital seperti suhu tubuh, detak jantung, dan saturasi oksigen (SpO2). Data kemudian dikirim ke Azure IoT Hub untuk pemantauan real-time dan analisis.

## 🎯 Fitur Utama

- **Pemantauan Suhu Tubuh**: Menggunakan sensor MLX90614 untuk pengukuran suhu non-kontak
- **Detak Jantung**: Monitoring BPM menggunakan sensor MAX30105
- **Saturasi Oksigen (SpO2)**: Pengukuran kadar oksigen dalam darah
- **Konektivitas WiFi**: Otomatis terhubung ke jaringan WiFi
- **Azure IoT Integration**: Pengiriman data ke Azure IoT Hub
- **Data Averaging**: Sistem rata-rata untuk akurasi data yang lebih baik
- **Auto-Reconnection**: Otomatis reconnect jika koneksi terputus
- **Real-time Monitoring**: Pemantauan status koneksi dan sensor

## 🛠️ Komponen Hardware

### Mikrokontroler
- **ESP8266** (NodeMCU/Wemos D1 Mini)

### Sensor
- **MLX90614**: Sensor suhu inframerah non-kontak
- **MAX30105**: Sensor detak jantung dan SpO2

### Koneksi I2C
- SDA: Pin D2 (GPIO4)
- SCL: Pin D1 (GPIO5)
- VCC: 3.3V
- GND: Ground

## 📚 Library Dependencies

```cpp
// Sensor Libraries
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

// IoT Libraries
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <az_core.h>
#include <az_iot.h>
#include <azure_ca.h>
#include <base64.h>
#include <ArduinoJson.h>
#include <Ticker.h>
```

### Instalasi Library
1. Buka Arduino IDE
2. Go to **Sketch** → **Include Library** → **Manage Libraries**
3. Install library berikut:
   - `Adafruit MLX90614 Library`
   - `MAX30105 Library`
   - `ESP8266WiFi`
   - `PubSubClient`
   - `ArduinoJson`
   - `Azure SDK for C - Arduino`

## ⚙️ Konfigurasi

### 1. WiFi Configuration
Edit file `iot_configs.h`:
```cpp
#define IOT_CONFIG_WIFI_SSID      "Your_WiFi_SSID"
#define IOT_CONFIG_WIFI_PASSWORD  "Your_WiFi_Password"
```

### 2. Azure IoT Hub Configuration
```cpp
#define IOT_CONFIG_IOTHUB_FQDN    "your-iot-hub.azure-devices.net"
#define IOT_CONFIG_DEVICE_ID      "your-device-id"
#define IOT_CONFIG_DEVICE_KEY     "your-device-key"
```

### 3. Sensor Settings
```cpp
#define DATA_POINTS 30                    // Jumlah data point sebelum averaging
#define COLLECTION_INTERVAL_MS 2000       // Interval pengumpulan data (2 detik)
#define SEND_INTERVAL_MS 60000           // Interval pengiriman data (1 menit)
#define SPO2_INTERVAL_MS 10000           // Interval pembacaan SpO2 (10 detik)
```

## 🚀 Cara Penggunaan

### 1. Setup Hardware
1. Hubungkan sensor MLX90614 dan MAX30105 ke ESP8266 via I2C
2. Power ESP8266 menggunakan USB atau power supply 5V

### 2. Upload Code
1. Buka `main.ino` di Arduino IDE
2. Pilih board **ESP8266** dan port yang sesuai
3. Upload code ke ESP8266

### 3. Monitor Serial
1. Buka Serial Monitor (115200 baud)
2. Perhatikan proses inisialisasi sensor dan koneksi
3. Monitor data yang dikirim ke Azure IoT Hub

## 📊 Format Data Output

Data dikirim ke Azure IoT Hub dalam format JSON:
```json
{
  "deviceId": "1",
  "pulseRate": 75.5,
  "temperature": 36.8,
  "sp02": 98.2,
  "timestamp": "2025-07-30T10:30:00Z"
}
```

## 🔧 Troubleshooting

### Sensor Issues
- **MLX90614 tidak terdeteksi**: Periksa koneksi I2C dan alamat sensor (0x5A)
- **MAX30105 tidak terdeteksi**: Pastikan koneksi I2C benar dan sensor mendapat power yang cukup
- **Pembacaan tidak stabil**: Pastikan jari ditempatkan dengan benar pada sensor MAX30105

### Koneksi Issues
- **WiFi tidak terhubung**: Periksa SSID dan password di `iot_configs.h`
- **Azure IoT Hub gagal**: Verifikasi connection string dan device key
- **MQTT disconnect**: Sistem akan otomatis reconnect setiap 30 detik

### Memory Issues
- **Heap overflow**: Monitor free heap di serial output
- **Watchdog reset**: Pastikan `yield()` dipanggil dalam loop yang panjang

## 📈 Monitoring & Analytics

### Serial Output Example
```
PETSA | ========================
Temperature = 36.75°C
BPM = 78.50
SpO2 = 98.1%
Free heap: 45632 bytes
Data point 15/30 collected
WiFi Status: Connected, MQTT Status: Connected
```

### Azure IoT Hub
- Data dapat dipantau melalui Azure IoT Explorer
- Setup alerts untuk nilai abnormal
- Integrasikan dengan Power BI untuk dashboard

## 🔒 Security Features

- **TLS/SSL**: Koneksi aman ke Azure IoT Hub
- **SAS Token**: Autentikasi otomatis dengan regenerasi setiap jam
- **Certificate Pinning**: Validasi sertifikat Azure

## 🎛️ Advanced Configuration

### Sensor Calibration
```cpp
// MLX90614 Emissivity (default: 0.98)
mlx.writeEmissivity(0.98);

// MAX30105 Settings
particleSensor.setup(60, 4, 2, 100, 411, 4096);
```

### Timing Adjustments
- Kurangi `COLLECTION_INTERVAL_MS` untuk data lebih frequent
- Tingkatkan `DATA_POINTS` untuk averaging yang lebih smooth
- Sesuaikan `SPO2_INTERVAL_MS` berdasarkan kebutuhan

## 🤝 Contributing

1. Fork repository ini
2. Buat feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add some AmazingFeature'`)
4. Push ke branch (`git push origin feature/AmazingFeature`)
5. Buat Pull Request

## 📝 License

Distributed under the MIT License. See `LICENSE` for more information.

## 🙏 Acknowledgments

- Adafruit untuk library MLX90614
- Maxim Integrated untuk MAX30105 library
- Microsoft Azure untuk IoT Hub platform
- Arduino community untuk ESP8266 support

## 📞 Support

Jika mengalami issues atau memiliki pertanyaan:
1. Check troubleshooting section di atas
2. Create issue di GitHub repository
3. Contact author melalui LinkedIn

---

**PETSA** - Making health monitoring accessible and smart! 🏥✨
