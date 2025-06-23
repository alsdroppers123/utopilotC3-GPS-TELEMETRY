# ✈️ ESP32-C3 Aircraft Flight Controller

This project implements a robust, web-enabled aircraft flight controller using an **ESP32-C3**, supporting **gyro stabilization**, **altitude hold**, **differential thrust**, and **real-time telemetry via MQTT**. It is designed to work **independently of Wi-Fi or MQTT availability**, ensuring safety and reliability in flight.

---

## 📦 Features

- ✅ PPM-based RC input handling
- ✅ MPU6050 for pitch/roll estimation (complementary filter)
- ✅ BMP280 for altitude sensing
- ✅ Altitude Hold Mode (auto throttle & pitch control)
- ✅ Gyro Stabilization for smoother flight
- ✅ Differential Thrust Support (for yaw control via ESCs)
- ✅ Live GPS telemetry with TinyGPS++
- ✅ Onboard Web UI for:
  - PID tuning (roll/pitch/altitude)
  - Calibration
  - Live GPS display
- ✅ MQTT telemetry publishing (`esp32/telemetry`)
- ✅ Safe fallback when Wi-Fi or MQTT is unavailable

---

## 🔧 Hardware Requirements

| Component            | Description                       |
|---------------------|-----------------------------------|
| ESP32-C3 (Super Mini recommended) | Microcontroller |
| MPU6050              | IMU Sensor                        |
| BMP280               | Barometric pressure sensor        |
| TinyGPS Module       | GPS module (e.g., NEO-6M)         |
| 2x ESCs + Brushless Motors | Main propulsion             |
| 3x Servos            | Right Aileron, Left Aileron, Elevator |
| RC Receiver          | PPM-summing capable               |
| Power Supply         | LiPo Battery or BEC               |

---

## 📶 Pin Configuration

| Function              | Pin |
|-----------------------|-----|
| PPM Signal Input      | GPIO2 |
| ESC1 (Throttle Left)  | GPIO0 |
| ESC2 (Throttle Right) | GPIO1 |
| Servo Right Aileron   | GPIO8 |
| Servo Left Aileron    | GPIO9 |
| Servo Elevator        | GPIO10 |
| GPS RX                | GPIO20 |
| GPS TX                | GPIO21 |
| I2C SDA               | GPIO6 |
| I2C SCL               | GPIO7 |

---

## 🌐 Web Interface

Once connected to Wi-Fi, visit the ESP32 IP in a browser (shown in Serial Monitor) to:

- View live GPS coordinates
- Adjust PID values (kP, kI, kD for pitch/roll; altitude KP/KD)
- Trigger manual calibration

---

## ☁️ MQTT Telemetry

- Broker: `test.mosquitto.org` (can be changed)
- Topic: `esp32/telemetry`
- Payload format: JSON

Example:
```json
{
  "altitude": 125.30,
  "pitch": 2.15,
  "roll": -3.40,
  "latitude": 27.712345,
  "longitude": 85.315678,
  "gyroStabilizationEnabled": true,
  "differentialThrustEnabled": false,
  "altitudeHoldEnabled": true,
  "altitudeHoldEngaged": true,
  "throttleSignal": 1345
}
