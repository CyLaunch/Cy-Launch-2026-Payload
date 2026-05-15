# ======================================================================================
# Developer: Noah Wons
# Program: Test script for orientation-based motor control using BNO055 IMU and PCA9685 on FeatherWing M4 Express
# Additional Notes:
# - Level is determined by comparing the Y and Z components of the gravity vector to a
#   calibrated reference, ignoring the X axis entirely.
# - The motor will attempt to correct any tilt by running in the appropriate direction
#   at a constant power until the tilt falls within the dead zone.
# - A dead zone is implemented to prevent constant small corrections when nearly level.
# - Tuning parameters (CORRECTION_SPEED, LEVEL_THRESHOLD) may need adjustment based on
#   your specific motor and mechanical setup.
# - NOTE: Orientation motor is on PWM channels 0 and 1. Encoder A on D9, B on D10.
#
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import math
import board
import digitalio
from adafruit_pca9685 import PCA9685
import adafruit_bno055

# --- Setup I2C, PCA9685, and BNO055 ---
i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 1000
bno = adafruit_bno055.BNO055_I2C(i2c)

# --- Orientation motor is on CH0 (forward) and CH1 (reverse) ---
MOTOR_PWM1 = 2
MOTOR_PWM2 = 3

# --- OCC pins (watching all candidates to identify which motor this is) ---
occ_d6 = digitalio.DigitalInOut(board.D6)
occ_d6.direction = digitalio.Direction.INPUT
occ_d6.pull = digitalio.Pull.UP

occ_d9 = digitalio.DigitalInOut(board.D9)
occ_d9.direction = digitalio.Direction.INPUT
occ_d9.pull = digitalio.Pull.UP

occ_d10 = digitalio.DigitalInOut(board.D10)
occ_d10.direction = digitalio.Direction.INPUT
occ_d10.pull = digitalio.Pull.UP

# --- Tuning Parameters ---
LEVEL_THRESHOLD   = 10   # degrees — dead zone around calibrated position
CORRECTION_SPEED  = 90  # % power applied whenever tilt exceeds LEVEL_THRESHOLD

# --- Calibration ---
CALIBRATION_DURATION = 3.0
CALIBRATION_DT       = 0.1


def yz_angle(v1, v2):
    """Unsigned angle in degrees between two gravity vectors using only Y and Z.
    Ignores X so rotation around the X axis does not affect the level reading."""
    y1, z1 = v1[1], v1[2]
    y2, z2 = v2[1], v2[2]
    dot  = y1 * y2 + z1 * z2
    mag1 = math.sqrt(y1 * y1 + z1 * z1)
    mag2 = math.sqrt(y2 * y2 + z2 * z2)
    if mag1 == 0 or mag2 == 0:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / (mag1 * mag2)))))


def signed_yz_tilt(ref, g):
    """Signed angle in degrees in the Y-Z plane relative to the reference.
    Positive/negative sign determines motor correction direction."""
    ref_angle = math.atan2(ref[2], ref[1])
    cur_angle = math.atan2(g[2],   g[1])
    return math.degrees(cur_angle - ref_angle)


def calibrate_level():
    print("=" * 45)
    print("  CALIBRATION")
    print("  Place the rover on level ground and")
    print("  keep it still.")
    print("  Starting in 3 seconds...")
    print("=" * 45)
    time.sleep(3)

    samples = []
    end_time = time.monotonic() + CALIBRATION_DURATION
    print(f"Collecting {CALIBRATION_DURATION:.0f}s of data...", end="")

    while time.monotonic() < end_time:
        g = bno.gravity
        if g is not None and all(v is not None for v in g):
            samples.append(g)
        time.sleep(CALIBRATION_DT)

    if not samples:
        print(" FAILED (no IMU data). Defaulting to (0, 0, -9.8)")
        return (0.0, 0.0, -9.8)

    ref = tuple(sum(s[i] for s in samples) / len(samples) for i in range(3))
    print(f" done ({len(samples)} samples)")
    print(f"  Gravity reference: X={ref[0]:+.3f}  Y={ref[1]:+.3f}  Z={ref[2]:+.3f} m/s²")
    print()
    return ref


# --- Motor Control ---
def set_motor(speed):
    speed = max(-100, min(100, speed))
    duty  = int(abs(speed) / 100 * 65535)
    if speed > 0:
        pca.channels[MOTOR_PWM1].duty_cycle = duty
        pca.channels[MOTOR_PWM2].duty_cycle = 0
    elif speed < 0:
        pca.channels[MOTOR_PWM1].duty_cycle = 0
        pca.channels[MOTOR_PWM2].duty_cycle = duty
    else:
        pca.channels[MOTOR_PWM1].duty_cycle = 0
        pca.channels[MOTOR_PWM2].duty_cycle = 0

def stop_motor():
    set_motor(0)


time.sleep(30)
ref_gravity = calibrate_level()

print("Starting leveling loop. Press CTRL+C to stop.")
print(f"Dead zone: +/- {LEVEL_THRESHOLD} degrees")
print(f"Correction speed: {CORRECTION_SPEED}%")
print()

try:
    while True:
        g = bno.gravity

        if g is None or any(v is None for v in g):
            print("No IMU data — check wiring!")
            stop_motor()
            time.sleep(0.5)
            continue

        tilt      = signed_yz_tilt(ref_gravity, g)
        angle     = yz_angle(ref_gravity, g)
        speed_out = 0

        if angle <= LEVEL_THRESHOLD:
            stop_motor()
            speed_out = 0
            status = "LEVEL"
        else:
            speed_out = CORRECTION_SPEED if tilt > 0 else -CORRECTION_SPEED
            set_motor(speed_out)
            status = "CORRECTING"

        occ_str = (
            f"D6={'OCC' if not occ_d6.value else 'ok'} | "
            f"D9={'OCC' if not occ_d9.value else 'ok'} | "
            f"D10={'OCC' if not occ_d10.value else 'ok'}"
        )
        print(f"Angle: {angle:+.1f} deg | Speed: {speed_out:+.0f}% | {occ_str} | Status: {status}")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped by user.")
    stop_motor()
