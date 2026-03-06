# 🚀 CyLaunch 2026 Payload Device

## Overview

The **CyLaunch 2026 Payload Device** is a hybrid avionics and telemetry system designed for high-reliability data collection and motor actuation during rocket flight operations. The system is split across two subsystems — the **Nosecone** and the **Rover (Payload)** — each built around an Adafruit Feather M4 Express with a stacked RFM9x LoRa FeatherWing for communication.

This system supports:

* Real-time telemetry transmission
* Flight detection (launch, apogee, landing) via sensor fusion
* Absolute orientation sensing
* Remote command capability
* Multi-motor actuation
* Redundant communication pathways

---

## System Architecture

The system is composed of two independent but communicating units:

### Nosecone Unit

Responsible for flight detection (launch, apogee, landing) and telemetry relay.

| Component | Role |
|---|---|
| Adafruit Feather M4 Express | Main microcontroller |
| RFM9x LoRa FeatherWing (433 MHz) | Wireless telemetry (stacked) |
| Adafruit ICM-20948 9-DOF IMU | Accelerometer / gyro for flight detection |
| Adafruit MPL3115A2 Altimeter | Barometric altitude |

### Rover (Payload) Unit

Responsible for post-landing orientation, drive motor control, and sensor acquisition.

| Component | Role |
|---|---|
| Adafruit Feather M4 Express | Main microcontroller |
| RFM9x LoRa FeatherWing (433 MHz) | Wireless telemetry (stacked) |
| Adafruit ICM-20948 9-DOF IMU | Accelerometer / gyro for flight detection |
| Adafruit MPL3115A2 Altimeter | Barometric altitude |
| Adafruit BNO055 Absolute Orientation IMU | Post-landing orientation correction |
| Adafruit PCA9685 16-Channel PWM Driver | ESC/motor control via I2C |
| 3× Bigfoot25 Brushless Motors + ESCs | 1× orientation, 2× drive wheels |

---

## Communication Architecture

Both units use RFM9x LoRa FeatherWings stacked directly on their respective Feather M4s, operating at 433 MHz for bidirectional communication.

```
[Nosecone]                        [Rover]
Feather M4                        Feather M4
  + RFM9x FeatherWing  <──LoRa──>   + RFM9x FeatherWing
  + ICM-20948 (I2C)                 + ICM-20948 (I2C)
  + MPL3115A2 (I2C)                 + MPL3115A2 (I2C)
                                    + BNO055 (I2C)
                                    + PCA9685 (I2C)
                                         ├── Motor 0: Orientation
                                         ├── Motor 1: Drive Left
                                         └── Motor 2: Drive Right
```

---

## Flight Logic Overview

### Nosecone

Uses a Kalman filter fusing ICM-20948 accelerometer data with MPL3115A2 barometric altitude to drive a state machine:

```
IDLE → LAUNCHED → ASCENDING → APOGEE → DESCENDING → LANDED
```

Each state transition requires multiple consecutive confirmations to reject noise-triggered false positives.

### Rover

Runs an identical flight detection stack. On landing confirmation:

1. Drive motors stop
2. BNO055 orientation is read
3. PID controller drives the orientation motor to level the payload (Y axis vertical)
4. Motor stops once level is held within tolerance
5. Post-landing rover logic executes (TBD)

---

## Hardware Layout

### Feather M4 Connections (Both Units)

| Interface | Connected To |
|---|---|
| FeatherWing (stacked) | RFM9x LoRa radio |
| I2C (SDA/SCL) | ICM-20948, MPL3115A2 |
| I2C (SDA/SCL) | BNO055, PCA9685 *(Rover only)* |
| PWM (via PCA9685) | Motor ESCs *(Rover only)* |

### I2C Address Reference

| Device | Default I2C Address |
|---|---|
| ICM-20948 | 0x69 |
| MPL3115A2 | 0x60 |
| BNO055 | 0x28 |
| PCA9685 | 0x40 |

> ⚠️ Verify no address conflicts before powering both units. Use `i2cdetect` during bench testing.

---

## Motor Configuration (Rover)

| Channel | Motor | Function |
|---|---|---|
| PCA9685 Ch 0 | Bigfoot25 | Orientation (rotate payload to level) |
| PCA9685 Ch 1 | Bigfoot25 | Drive wheel left |
| PCA9685 Ch 2 | Bigfoot25 | Drive wheel right |

**ESC Protocol:** Standard hobby PWM (1000–2000µs), 50 Hz

---

## Power Requirements

> ⚠️ **Motors must use a dedicated power source. Do NOT power motors from the Feather 3.3V or 5V rail.**

All grounds must be shared across systems.

```
Battery Pack
  ├── ESC Power rails (motor voltage)
  ├── 5V Regulator → Feather M4 (USB/BAT input)
  └── Common GND across all boards
```

---

## Software Stack

Both units run **CircuitPython** on the Feather M4 Express.

### Nosecone

* `adafruit_icm20x` — ICM-20948 IMU driver
* `adafruit_mpl3115a2` — Altimeter driver
* `adafruit_rfm9x` — LoRa radio
* Custom Kalman filter + flight state machine

### Rover

* `adafruit_icm20x` — ICM-20948 IMU driver
* `adafruit_mpl3115a2` — Altimeter driver
* `adafruit_bno055` — Absolute orientation
* `adafruit_pca9685` — PWM motor driver
* `adafruit_rfm9x` — LoRa radio
* Custom Kalman filter + flight state machine + PID orientation controller

### Dependencies

```bash
circup install adafruit_icm20x adafruit_mpl3115a2 adafruit_bno055 adafruit_pca9685 adafruit_rfm9x
```

---

## Repository Structure

```
TODO
```

---

## Flight Safety Notes

* Verify Li-Ion battery storage and transport compliance with launch range rules.
* Secure all connectors and FeatherWing stacks against vibration before flight.
* Use strain relief on all I2C wiring runs.
* Confirm 433 MHz LoRa operation complies with launch range RF regulations.
* Perform ESC arming sequence verification on the bench before integration.

---

## Future Improvements

* Hardware watchdog between nosecone and rover units
* Redundant IMU integration on nosecone
* Custom PCB replacing breadboard/FeatherWing prototype
* UART or CAN backup communication channel
* Rover drive logic (post-landing locomotion)

---

## Contributors

CyLaunch 2026 Payload Team
* Noah Wons