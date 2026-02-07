# LifeLink

## Opis (Description)

LifeLink is a smart device that combines technology and humanity, designed for people on dialysis, diabetics and patients undergoing cancer treatments. In the form of a ring, bracelet or pendant, it makes it possible to immediately send a signal for help with a single touch or voice, without the need for a person to search for a phone, write a message or make a call. This is crucial because in moments of weakness, many patients are exhausted, disoriented, or physically unable to respond, and that's when LifeLink responds for them.
Equipped with sensors to monitor heart rate, movement and oxygen saturation, as well as a GPS system, LifeLink accurately sends real-time location and vital data. It is connected to a mobile application that connects family, doctors, psychological support and emergency services, allowing help to arrive immediately – anywhere, anytime. With the constant availability of psychological support 24/7, users receive not only medical, but also emotional security.
LifeLink not only makes life easier for patients, but also for their families – it gives them a sense of peace and freedom to go about their daily chores knowing that their loved one is safe and under supervision. This is not just a device – it is a system of trust, safety and dignity that allows patients to live freely, to travel and breathe with a sense of security, because they know that help is always coming – even when they can no longer ask for it themselves.

LifeLink je pametan uređaj koji spaja tehnologiju i humanost, namenjen osobama na dijalizi, dijabetičarima i pacijentima koji prolaze kroz terapije raka. U obliku prstena, narukvice ili priveska, on omogućava da se jednim dodirom ili glasom, bez potrebe da osoba traži telefon, piše poruku ili poziva, odmah pošalje signal za pomoć. To je presudno jer su u trenucima slabosti mnogi pacijenti iscrpljeni, dezorijentisani ili fizički nesposobni da reaguju, a upravo tada LifeLink reaguje umesto njih.

Opremljen senzorima za praćenje pulsa, pokreta i zasićenosti kiseonikom, kao i GPS sistemom, LifeLink precizno šalje lokaciju i vitalne podatke u realnom vremenu. Povezan je sa mobilnom aplikacijom koja umrežava porodicu, lekare, psihološku podršku i službe hitne pomoći, omogućavajući da pomoć stigne odmah – bilo gde i bilo kada. Uz stalnu dostupnost psihološke podrške 24/7, korisnici dobijaju ne samo medicinsku, već i emocionalnu sigurnost.

LifeLink ne olakšava život samo pacijentima, već i njihovim porodicama – pruža im osećaj mira i slobodu da obavljaju svakodnevne obaveze znajući da je njihova voljena osoba bezbedna i pod nadzorom. Ovo nije samo uređaj – to je sistem poverenja, sigurnosti i dostojanstva koji omogućava bolesnicima da žive slobodno, da putuju i dišu s osećajem sigurnosti, jer znaju da pomoć uvek stiže – čak i kada oni više ne mogu da je zatraže sami.

<p align="center">
  <img src="lifelink_interface.png" alt="LifeLink Interface" width="400"/>
</p>

---

## Hardware Specifications

This project is designed to run on the **Waveshare ESP32-S3-Touch-AMOLED-1.75** development board.

*   **SoC**: ESP32-S3R8 (Xtensa® 32-bit LX7 dual-core, up to 240 MHz)
*   **Memory**: 512KB SRAM, 384KB ROM, 8MB PSRAM, 16MB External Flash
*   **Display**: 1.75-inch AMOLED, 466x466 resolution, 16.7M colors
*   **Touch**: Capacitive touch (CST9217)
*   **Sensors**:
    *   QMI8658 6-axis IMU (3-axis accelerometer and 3-axis gyroscope)
    *   **GPS**: LC76G (GNSS Module) via I2C
    *   **Heart Rate/SpO2**: MAX30102 via I2C
*   **RTC**: PCF85063
*   **Connectivity**: 2.4GHz Wi-Fi (802.11 b/g/n) & Bluetooth® 5 (LE)
*   **Other**: Microphone, Speaker, TF Card Slot, Battery Manager (AXP2101)

## Hardware Connection

The connection between the ESP32-S3 and the AMOLED panel is pre-wired on the board. The internal connections for reference are:

| ESP32-S3 Pin | AMOLED/Peripheral Function |
| :--- | :--- |
| **QCA (QSPI)** | **Display Interface** |
| GPIO 0... | *See schematic/driver for exact mapping* |
| **I2C** | **Touch & Sensors (Shared Bus)** |
| SDA | GPIO 15 (TP_SDA, QMI8658, MAX30102, LC76G) |
| SCL | GPIO 14 (TP_SCL, QMI8658, MAX30102, LC76G) |

## Features (Current Status)
- **Vital Signs**: Real-time Heart Rate and SpO2 monitoring using MAX30102.
- **Motion Tracking**: 6-axis IMU (QMI8658) with Advanced Fall Detection (Impact + Stillness + Angle Check).
- **Location**: GPS positioning using LC76G module (NMEA parsing for Lat/Lon/Time).
- **Power Management**: 
    - Battery monitoring via AXP2101 (Voltage & %).
    - Screen timeout (15s) with touch-to-wake.
- **Interface**: 
    - **Watch Face**: Digital Clock (GPS Time), Status Icons (GPS, BLE, Bat), Health Data.
    - **Debug Screen**: Swipe to see raw sensor data.
- **Connectivity**: Bluetooth Low Energy (BLE) reporting for fall events and diagnostics.

## Optimization Notes
- **Power Efficiency**: Screen sleeps after inactivity; Sensors rate-limited when idle.
- **Stability**: Custom I2C drivers ensure robust communication on the shared bus.

*Note: This project uses a custom QSPI panel driver compatible with the Waveshare hardware.*

## Build and Flash

1.  **Setup Environment**: Ensure you have ESP-IDF v5.x installed.
2.  **Build**:
    ```bash
    idf.py build
    ```
3.  **Flash and Monitor**:
    ```bash
    idf.py -p PORT flash monitor
    ```
    *(Replace `PORT` with your device's serial port name, e.g., `COM3` on Windows or `/dev/ttyUSB0` on Linux)*

### Troubleshooting

If the display does not turn on:
*   Check the backlight level macro in `main/lifelink.c`.
*   Ensure the battery is charged if running without USB.

For detailed hardware docs, visit the [Waveshare Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.75).
