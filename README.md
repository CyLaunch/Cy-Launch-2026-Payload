# CyLaunch 2026 Payload

## Overview

The **CyLaunch 2026 Payload** is a two-system avionics package designed for a high-power rocket with an expected apogee of approximately 4800 ft. The payload consists of a **Nosecone System** that detects flight events and deploys the rover, and a **Rover System** that performs autonomous soil collection after landing.

Both systems run **CircuitPython** on an Adafruit Feather M4 Express.

---

## System 1 — Nosecone

### Purpose

Detects launch and landing events, then actuates a servo to release the rover from the nosecone on landing confirmation.

### Hardware

| Component | Role |
|---|---|
| Adafruit Feather M4 Express | Main microcontroller |
| Adafruit ICM-20948 9-DOF IMU | Accelerometer / gyro / magnetometer |
| Adafruit MPL3115A2 Altimeter | Barometric altitude |
| Servo (D5) | Nosecone pin retraction on landing |
| 5V 5200mAh battery | System power |

### Flight Detection Logic

Launch and landing are detected using a threshold-based state machine — no Kalman filter is required given the ~4800ft apogee, which provides a 30:1 signal-to-noise margin over the 50m detection threshold.

**State machine:** `IDLE → LAUNCHED → LANDED`

**Launch detection (`IDLE → LAUNCHED`):**
- Requires 5 consecutive readings where:
  - IMU acceleration magnitude > 20 m/s² (motor ignition signature, ~2G)
  - Altimeter AGL > 50m (confirms vehicle is airborne, not just bumped on the pad)
- A 10-second lockout after launch prevents landing from being evaluated during boost

**Landing detection (`LAUNCHED → LANDED`):**
- Requires 10 consecutive readings where:
  - Altimeter AGL < 50m
  - Standard deviation of IMU acceleration magnitude < 0.5 m/s² (vehicle is stationary)
- On confirmation: servo rotates to retract nosecone pins and release the rover

**Calibration (run at startup, vehicle stationary):**
- Averages raw pressure readings to derive local sea-level pressure, correcting for weather and launch site elevation
- Sets altimeter ground baseline so all altitude readings are AGL
- IMU gravity offset on the Z axis is measured and removed from acceleration readings

### Scripts

| File | Description |
|---|---|
| `feather/nosecone/simpleFlightDetection.py` | Primary flight detection script |
| `feather/nosecone/landingDetection.py` | Earlier Kalman filter-based flight detection (reference) |
| `feather/nosecone/imuAltimeterServoTest.py` | Bench diagnostic — servo sweep + sensor readout |

---

## System 2 — Rover

### Purpose

Deployed from the nosecone after landing. Autonomously navigates to a soil collection site, collects a sample, and returns.

### Hardware

| Component | Role |
|---|---|
| Adafruit Feather M4 Express | Main microcontroller |
| Adafruit ICM-20948 9-DOF IMU | Accelerometer / gyro / magnetometer (flight detection) |
| Adafruit MPL3115A2 Altimeter | Barometric altitude (flight detection) |
| Adafruit BNO055 Absolute Orientation IMU | Heading / orientation for motor 3 rotation control |
| Adafruit PCA9685 PWM Driver | Supplies PWM signals to all DC motors via I2C |
| DC Motor 1 — Drive (encoder) | Primary locomotion |
| DC Motor 2 — Soil Collection (encoder) | Soil collection mechanism |
| DC Motor 3 — Rotation (no encoder) | Rover rotation, controlled via BNO055 heading |

### Motor Configuration

| Motor | Encoder | Control Basis | Function |
|---|---|---|---|
| Drive | Yes | Encoder RPM | Forward/reverse locomotion, obstacle navigation |
| Soil Collection | Yes | Encoder RPM / voltage | Soil collection mechanism |
| Rotation | No | BNO055 heading | Rotates rover to correct heading |

The two encoder-equipped motors allow RPM measurement for obstacle navigation and voltage requirement tuning. The rotation motor uses BNO055 absolute orientation data to determine how far and in which direction to rotate.

### Flight Detection

The rover runs a landing detection script closely mirroring the nosecone's `simpleFlightDetection.py`, using the same ICM-20948 + MPL3115A2 hardware stack and the same launch/landing logic. The only modification is that on landing confirmation, the rover executes the soil collection algorithm rather than actuating a servo.

### Scripts

| File | Description |
|---|---|
| `feather/rover/landingDetection.py` | Kalman filter-based flight detection (reference) |
| `feather/rover/DCMotorTest.py` | Basic DC motor actuation test |
| `feather/rover/DCMotorEncoderTest.py` | Motor RPM measurement via encoder |
| `feather/rover/motorTest.py` | ESC calibration and motor test sequence |
| `feather/rover/orientationTest.py` | BNO055 orientation-driven motor control test |

---

## Repository Structure

```
Cy-Launch-2026-Payload/
├── feather/
│   ├── nosecone/
│   │   ├── simpleFlightDetection.py     # Primary nosecone flight script
│   │   ├── landingDetection.py          # Kalman-based flight detection (reference)
│   │   └── imuAltimeterServoTest.py     # Bench diagnostic
│   ├── rover/
│   │   ├── landingDetection.py          # Rover flight detection
│   │   ├── DCMotorTest.py               # DC motor test
│   │   ├── DCMotorEncoderTest.py        # Encoder RPM test
│   │   ├── motorTest.py                 # ESC calibration
│   │   └── orientationTest.py           # BNO055 orientation test
│   └── feather_libraries/
│       └── lib/                         # CircuitPython libraries (.mpy)
└── py/
    └── testing/                         # Host-side Python test scripts
        ├── imuTesting/
        ├── lightSensorTesting/
        ├── motorTesting/
        ├── soilProbeTesting/
        └── transmissionTesting/
```

---

## I2C Address Reference

| Device | Default Address |
|---|---|
| ICM-20948 | 0x69 (alt: 0x68) |
| MPL3115A2 | 0x60 |
| BNO055 | 0x28 |
| PCA9685 | 0x40 |

---

## Power

The nosecone system is powered by a dedicated 5V 5200mAh battery. Motor power on the rover must use a dedicated supply — do not draw motor current from the Feather's 3.3V or 5V rails. All grounds must be shared.

---

## Contributors

CyLaunch 2026 Payload Team
* Noah Wons
