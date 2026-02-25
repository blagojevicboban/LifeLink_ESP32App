# LifeLink Smartwatch - ESP32-S3

[🇷🇸 Srpski verzija dokumentacije nalazi se u README_RS.md](README_RS.md)

LifeLink is an advanced smartwatch prototype built on the **ESP32-S3** platform, utilizing **ESP-IDF** alongside **LVGL** for a rich graphical interface (466x466 AMOLED display). It focuses on elderly care, health tracking, and robust emergency response functionality.

## Features

- **Advanced Fall Detection**: Utilizes the QMI8658 IMU (Accelerometer + Gyroscope) to detect sudden drops (Free Fall) and hard impacts. It requires extended stillness after an impact coupled with an orientation shift to confirm a real fall and avoid false alarms.
- **Fall Simulation & Override**: Users can simulate a fall via the interactive UI for testing purposes. Real falls trigger an immediate 5-second on-screen countdown; if it's a false alarm, users can tap to cancel before an alert is dispatched.
- **Automated GSM Emergency SMS**: Communicates with a SIM800L GSM Module to send background SMS alerts containing:
  - Precise GPS coordinates formatted as a direct Google Maps link.
  - Heart rate at the time of the event.
  - Contextual warnings stating whether the fall was real or simulated.
- **Live Health Monitoring**: Reads heart rate (BPM) and blood oxygen saturation (SpO2) using a MAX30102 sensor. Values are constantly updated on the primary watch face.
- **Interactive UI (LVGL)**: 
  - Dynamic top status bar indicating GPS lock, Cellular Network registration, Battery Levels, and BLE connectivity.
  - Intuitive gesture-based navigation (swipe left/right) between screens.
  - Dedicated "Settings" view featuring an on-screen numpad allowing users to register emergency SMS phone numbers without requiring an external mobile app.
- **Sensor Debug View**: Accessible "DEBUG" toggle available in the UI to visualize live X, Y, Z, and G-force readings for rapid testing and threshold calibration.

## Companion Mobile App (Flutter)

A cross-platform **Flutter** companion app extends LifeLink's capabilities via Bluetooth Low Energy (BLE):

- **Real-Time Dashboard**: Live Heart Rate (BPM), SpO2, G-Force, and GPS location mirrored from the watch.
- **BLE Connectivity**: Automatic or manual pairing with the LifeLink ESP32 wearable via BLE SPP.
- **Emergency Response**: Configurable fall response actions — direct phone **Call**, **SMS** with GPS coordinates, or system-wide **SOS** intent.
- **Fall Mirroring**: App mirrors the watch's 3-stage fall detection (Safe → Warning → Alarm) with a 5-second countdown and haptic/audio alerts.
- **Settings**: Configure emergency contact, fall action type, countdown duration, and default device MAC address.
- **Interactive Map**: Displays the user's location on an OpenStreetMap view for responder assistance.

## Hardware Stack

- **MCU**: ESP32-S3
- **Display**: Round AMOLED (466 x 466)
- **Cellular**: SIM800L GSM Module (Communicating via AT Commands over UART, powered directly from 3.7V Li-Ion battery)
- **IMU Sensor**: QMI8658 (Accelerometer & Gyroscope)
- **Health Sensor**: MAX30102 (HR & SpO2)
- **Power Management**: AXP2101

## Setup & Building

This project is built using the official [Espressif ESP-IDF framework](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/) (v5.x recommended).

### 1. Configure the Project
Make sure your target is properly mapped to the ESP32-S3:
```bash
idf.py set-target esp32s3
idf.py menuconfig
```
### 2. Build and Flash
Compile the source code and flash it directly to your connected device:
```bash
idf.py build
idf.py -p COM_PORT flash monitor
```

## Application Flow

1. **Dashboard (Screen 1)**: Primary clock face, health vitals, and connectivity status icons.
2. **Fall Trigger/Debug (Screen 2)**: Simulate a fall, or engage the raw hardware debug sensors stream.
3. **Settings (Screen 3)**: Utilize the customized numpad to input or replace your dedicated emergency contact's GSM phone number.
4. **Emergency Countdown (Screen 4)**: Activated upon Fall Confirmation. Provides an unmissable red-alert interface allowing users to abort the alert before it attempts to connect via the SIM800L modem to send an SMS.
