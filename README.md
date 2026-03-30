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

### Scripts

| File | Description |
|---|---|
| `feather/nosecone/flightDetection.py` | Primary flight detection script |
| `feather/nosecone/landingDetection.py` | Earlier Kalman filter-based flight detection (reference) |
| `feather/nosecone/testing/imuAltimeterServoTest.py` | Bench diagnostic — servo sweep + sensor readout |
| `feather/nosecone/testing/testLandingDetection.py` | Landing detection bench test |

---

## System 2 — Rover

### Purpose

Deployed from the nosecone after landing. Autonomously navigates to a soil collection site, collects a sample, and returns.

### Hardware

| Component | Role |
|---|---|
| Adafruit Feather M4 Express | Main microcontroller |
| Adafruit BNO055 Absolute Orientation IMU | Accelerometer / gyro / orientation (flight detection + leveling) |
| Adafruit MPL3115A2 Altimeter | Barometric altitude (flight detection) |
| Adafruit PCA9685 PWM Driver | Supplies PWM signals to H-bridge motors via I2C |
| Motor 1 — Drive, brushed ESC (encoder A=D11, B=D12) | Primary locomotion |
| Motor 2 — Soil Collection, TB9051FTG H-bridge CH0/CH1 (encoder A=D9, B=D10, OCC=D13) | Soil collection mechanism |
| Motor 3 — Orientation, TB9051FTG H-bridge CH2/CH3 (no encoder, OCC=D6) | Rover leveling, controlled via BNO055 roll |

### Motor Configuration

| Motor | Channel | Encoder | OCC Pin | Function |
|---|---|---|---|---|
| M1 — Drive (brushed ESC) | D5 PWM | A=D11, B=D12 | — | Forward/reverse locomotion |
| M2 — Soil Collection (TB9051FTG) | PCA9685 CH0/CH1 | A=D9, B=D10 | D13 | Soil collection mechanism |
| M3 — Orientation (TB9051FTG) | PCA9685 CH2/CH3 | None | D6 | Levels rover using BNO055 roll |

The BNO055 replaces a dedicated 9-DOF IMU — its onboard sensor fusion provides both raw accelerometer/gyro data for flight detection and fused Euler angles for orientation-based motor control, reducing I2C bus load to two sensors (BNO055 + MPL3115A2).

### Flight Detection

`feather/rover/flightDetection.py` mirrors the nosecone state machine using BNO055 acceleration + MPL3115A2 altitude. On landing confirmation, it runs a pre-captured roll offset calibration and drives the orientation motor (M3) to level the rover before beginning autonomous operations.

**Pre-launch calibration sequence:**
1. Altimeter pressure baseline (50 samples, ~2.5s)
2. BNO055 roll offset baseline (3s sample, rover on level ground)

### Scripts

| File | Description |
|---|---|
| `feather/rover/flightDetection.py` | Primary rover flight detection + post-landing leveling |
| `feather/rover/testing/motorTest.py` | All-motor test — ESC + two H-bridge motors with OCC monitoring |
| `feather/rover/testing/orientationTest.py` | BNO055-driven orientation motor control with level calibration |
| `feather/rover/testing/componentCheck.py` | I2C component detection check (BNO055, MPL3115A2, PCA9685) |
| `feather/rover/testing/brushedESCTest.py` | Single brushed ESC test with encoder RPM |
| `feather/rover/testing/dualMotorDriveTest.py` | Dual H-bridge motor test with encoder RPM |
| `feather/rover/testing/singleMotorTest.py` | Single H-bridge motor test |
| `feather/rover/testing/imuTest.py` | ICM-20948 bench diagnostic (reference) |
| `feather/rover/testing/DCMotorTest.py` | Basic DC motor actuation test |
| `feather/rover/testing/DCMotorEncoderTest.py` | Encoder RPM measurement test |
| `feather/rover/testing/testSoilTester.py` | 7-in-1 soil sensor RS-485/Modbus readout |
| `feather/rover/testing/landingDetection.py` | Kalman filter-based flight detection (reference) |

---

## Repository Structure

```
Cy-Launch-2026-Payload/
├── feather/
│   ├── nosecone/
│   │   ├── flightDetection.py           # Primary nosecone flight script
│   │   ├── landingDetection.py          # Kalman-based flight detection (reference)
│   │   └── testing/
│   │       ├── imuAltimeterServoTest.py # Bench diagnostic — servo + sensor readout
│   │       └── testLandingDetection.py  # Landing detection bench test
│   ├── rover/
│   │   ├── flightDetection.py           # Primary rover flight detection + leveling
│   │   └── testing/
│   │       ├── motorTest.py             # All-motor test (ESC + 2x H-bridge)
│   │       ├── orientationTest.py       # BNO055 orientation motor control
│   │       ├── componentCheck.py        # I2C component detection check
│   │       ├── brushedESCTest.py        # Single brushed ESC test
│   │       ├── dualMotorDriveTest.py    # Dual H-bridge motor test
│   │       ├── singleMotorTest.py       # Single motor test
│   │       ├── imuTest.py               # ICM-20948 bench diagnostic (reference)
│   │       ├── DCMotorTest.py           # DC motor actuation test
│   │       ├── DCMotorEncoderTest.py    # Encoder RPM test
│   │       ├── testSoilTester.py        # Soil sensor RS-485/Modbus readout
│   │       └── landingDetection.py      # Kalman-based flight detection (reference)
│   └── feather_libraries/
│       └── lib/                         # CircuitPython libraries (.mpy)
└── pi/
    └── testing/                         # Old test scripts for Raspberry Pi flight computer
        ├── imuTesting/
        ├── lightSensorTesting/
        ├── motorTesting/
        ├── soilProbeTesting/
        └── transmissionTesting/
```

---

## I2C Address Reference

| Device | Default Address | Used By |
|---|---|---|
| ICM-20948 | 0x69 (alt: 0x68) | Nosecone |
| MPL3115A2 | 0x60 | Nosecone, Rover |
| BNO055 | 0x28 (alt: 0x29) | Rover |
| PCA9685 | 0x40 | Rover |

> **Note:** The rover runs only two I2C devices (BNO055 + MPL3115A2 + PCA9685). Each breakout board ships with onboard pull-up resistors — cut the `I2C PU` jumper on all but one board to avoid excessive bus loading.

---

## Power

The nosecone system is powered by a dedicated 5V 5200mAh battery. Motor power on the rover must use a dedicated supply — do not draw motor current from the Feather's 3.3V or 5V rails. All grounds must be shared.

---

## Contributors

CyLaunch 2026 Payload Team
* Noah Wons
