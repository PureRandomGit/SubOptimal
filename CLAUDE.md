# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SubOptimal is firmware for an autonomous underwater vehicle (submarine) built for EGN 1007C. It runs on an **ESP32-S3** using the **Arduino framework** via **PlatformIO**. The sub uses a BNO08x IMU for orientation, four brushless motors with ESCs, PID controllers for yaw/pitch/roll stabilization, and a reed switch to arm/trigger runs.

## Build & Upload

```bash
# Build
pio run

# Upload via OTA (default, configured to 10.53.227.126)
pio run -t upload

# Upload via USB (change upload_port in platformio.ini to /dev/ttyACM0 and comment out espota lines)
pio run -t upload

# Serial monitor
pio device monitor
```

The board target is `esp32-s3-devkitc-1`. Monitor baud rate is 115200.

## Architecture

### Firmware (src/)
- **main.cpp** — State machine controlling the sub's lifecycle: `Idle → Armed → Running → Recovery → Finished`. Handles WiFi (non-blocking connect), OTA updates, IMU reads (rotation vector + gyroscope + linear acceleration at 50Hz), PID computation, motor mixing, and in-memory CSV logging (~350 samples).
- **Motor.h/Motor.cpp** — ESC driver wrapping ESP-IDF LEDC. Maps a 0.0–1.0 speed float to 1000–2000µs PWM pulses at 50Hz. Each motor gets its own LEDC channel.

### Motor Mixing
PID outputs (pitch, roll, yaw corrections) are combined per-motor with opposite signs for front/back and left/right, then normalized so the fastest motor hits 1.0 and all others stay ≥ 0. During ramp-up after launch, `maxSpeed` scales linearly from `stabilizeSpeed` to 1.0.

### Data Pipeline
After a run finishes and WiFi reconnects, the sub serves logged CSV data at `http://<sub-ip>/logs`. Two Python tools consume this:
- **plot_logs.py** — Tkinter + matplotlib GUI that fetches and plots pitch/roll/yaw/motor data. `pip install requests matplotlib`
- **prediction.py** — Propeller optimization using physics curve-fitting and polynomial regression against thrust/amp test data from a Google Sheet. `pip install pandas numpy plotly scikit-learn scipy`
- **propGen.py** — Simpler propeller optimizer using scipy griddata interpolation.

### Calibration Sketches
- **esc_calibrate/** — Arduino sketch for ESC calibration
- **imu_calibrate/** — Arduino sketch for IMU calibration

## Key Constants to Tune
All in `main.cpp`: PID gains (`yawkp/ki/kd`, `pitchkp/kd`, `rollkp/ki/kd`), `stabilizeSpeed`, `BASE_PITCH_DEG`, `RAMP_UP_MS`, `pathTime`, `MID_TURN_DEG`, `MID_TURN_MS`, `RECOVERY_DEGREES`, `RECOVERY_TIME_MS`.
