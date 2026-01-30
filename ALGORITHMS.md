# LifeLink Algorithms Documentation

This document describes the signal processing algorithms used in the LifeLink project for Vital Signs monitoring and Fall Detection.

## 1. Heart Rate & SpO2 (FFT-Based)

We use the **Fast Fourier Transform (FFT)** to analyze the Photoplethysmogram (PPG) signal from the MAX30102 sensor. This method is more robust against motion artifacts compared to simple peak detection w. time-domain analysis.

### Implementation Details:
- **Library**: `espressif/esp-dsp` (Hardware accelerated on ESP32-S3).
- **Sampling Rate**: 100 Hz.
- **Buffer Size**: 512 samples (~5.12 seconds of data).
- **Window**: Sliding window with 50-sample overlap updates (~0.5s refresh).

### Steps:
1.  **DC Removal**: Subtract the mean value of the signal (0Hz component).
2.  **Windowing**: Apply a **Hann Window** to reduce spectral leakage.
3.  **FFT**: Compute the complex FFT using `dsps_fft2r_fc32`.
4.  **Power Spectrum**: Calculate magnitudes for both Red and IR channels.
5.  **Peak Detection**: Find the dominant frequency bin in the 0.5Hz - 4.0Hz range (30 - 240 BPM).
6.  **SpO2 Calculation**:
    - Extract AC components (Peak Magnitude) and DC components (Mean) for Red and IR.
    - Calculate Ratio $R = \frac{AC_{red}/DC_{red}}{AC_{ir}/DC_{ir}}$.
    - Apply formula: $SpO_2 = 110 - 25 \times R$.

---

## 2. Advanced Fall Detection (3-Phase Model)

The fall detection uses data from the QMI8658 IMU (Accelerometer + Gyroscope) and implements a state machine to distinguish real falls from daily activities.

### Core Concept: SVM (Signal Vector Magnitude)
The total G-force is calculated as:
$$SVM = \sqrt{a_x^2 + a_y^2 + a_z^2}$$
In rest, $SVM \approx 1.0g$.

### The 3-Phase Logic:

#### Phase 1: Free Fall (Slobodan Pad)
- **Trigger**: SVM drops below `0.6g` (`FALL_THRESHOLD_LOW`).
- **Action**:
    - The system enters `FREE_FALL` state.
    - Captures the **Release Orientation** (Reference Gravity Vector $\vec{v}_{ref}$) to compare later.

#### Phase 2: Impact (Udarac)
- **Window**: Must occur within 500ms of Free Fall.
- **Trigger**: SVM spikes above `3.5g` (`FALL_THRESHOLD_HIGH`).
- **Action**: System enters `WAITING_FOR_STILLNESS`.

#### Phase 3: Stillness & Orientation Check (Mirovanje)
- **Duration**: System monitors stability for **5 seconds** (`STILLNESS_DURATION_MS`).
- **Action**:
    - If the user moves significantly (SVM deviates > 0.2g from 1g), the fall is rejected.
    - After 5 seconds, if the user is still, we check **Orientation Change**.

### Orientation Verification
To prevent false positives (e.g., clapping hands, hitting a desk), we compare the device orientation before and after the "fall".

- **Formula**:
$\theta = \arccos(\frac{v_{ref} \cdot v_{curr}}{|v_{ref}| \cdot |v_{curr}|})$
- **Condition**: If $\theta > 60^\circ$ AND User is Still $\rightarrow$ **CONFIRMED FALL**.

## 3. Sensor Fusion & Rate Limiting

- **GPS (LC76G)**: Polled every 100ms, retry logic for robust I2C reading.
- **Rate Limiting**:
    - Sensor Polling: 100Hz (Real-time).
    - UI/Log Reporting: 10Hz (Every 0.1s) to save CPU cycles for DSP.
