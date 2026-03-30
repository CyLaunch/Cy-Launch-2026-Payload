# ======================================================================================
# Developer: Noah Wons
# Program: Test script for orientation-based motor control using BNO055 IMU and PCA9685 on FeatherWing M4 Express
# Additional Notes:
# - This script reads the roll angle from the BNO055 IMU and uses it to control a single motor for leveling.
# - The motor will attempt to correct any tilt by running in the appropriate direction and speed proportional to the angle of tilt.
# - A dead zone is implemented to prevent constant small corrections when the system is nearly level.
# - Tuning parameters (KP, MAX_SPEED, MIN_SPEED) may need adjustment based on your specific motor and mechanical setup.
# - NOTE: Orientation motor is on PWM channels 2 and 3. Encoder A on D9, B on D10.
# 
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import countio
import digitalio
from adafruit_pca9685 import PCA9685
import adafruit_bno055

# --- Setup I2C, PCA9685, and BNO055 ---
i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 1000  # 1kHz PWM for motor drivers
bno = adafruit_bno055.BNO055_I2C(i2c)

# --- Orientation motor is on CH2 (forward) and CH3 (reverse) ---
MOTOR_PWM1 = 0
MOTOR_PWM2 = 1

# --- Encoder (D9 = A, D10 = B) ---
enc_a = countio.Counter(board.D9, edge=countio.Edge.RISE)
enc_b = digitalio.DigitalInOut(board.D10)
enc_b.direction = digitalio.Direction.INPUT

# --- Tuning Parameters ---
LEVEL_THRESHOLD = 0.5    # degrees — if tilt is within this range, do nothing (dead zone)
MAX_SPEED = 60           # max motor speed % (keep below 100 to avoid violent corrections)
MIN_SPEED = 15           # min speed % to overcome motor stiction
KP = 1.5                 # proportional gain — increase if corrections are too slow,
                         # decrease if motor overshoots/oscillates

# --- Motor Control ---
def set_motor(speed):
    """
    speed: -100 to 100
    positive = forward, negative = reverse, 0 = stop
    """
    speed = max(-100, min(100, speed))  # clamp to safe range
    duty = int(abs(speed) / 100 * 65535)
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

# --- Calibration ---
CALIBRATION_DURATION = 3.0   # seconds to collect roll baseline
CALIBRATION_DT       = 0.05  # 20 Hz sample rate during calibration

def calibrate_level():
    """
    Collect roll readings for CALIBRATION_DURATION seconds with the rover
    sitting level on the ground. Returns the average roll as the zero offset.
    Only roll is used — pitch and yaw are ignored.
    """
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
        euler = bno.euler
        if euler is not None and euler[1] is not None:
            samples.append(euler[1])  # roll only
        time.sleep(CALIBRATION_DT)

    if not samples:
        print(" FAILED (no IMU data). Defaulting offset to 0.0")
        return 0.0

    offset = sum(samples) / len(samples)
    print(f" done ({len(samples)} samples)")
    print(f"  Roll offset: {offset:+.2f} deg")
    print()
    return offset

roll_offset = calibrate_level()

print("Starting leveling loop. Press CTRL+C to stop.")
print(f"Dead zone: +/- {LEVEL_THRESHOLD} degrees")
print(f"Max speed: {MAX_SPEED}%")
print()

try:
    while True:
        euler = bno.euler

        if euler is None or euler[1] is None:
            print("No IMU data — check wiring!")
            stop_motor()
            time.sleep(0.5)
            continue

        # Only read roll — pitch and yaw are not used
        roll = euler[1]

        # Tilt relative to calibrated ground-level roll
        tilt = roll - roll_offset

        # --- Proportional Control ---
        if abs(tilt) <= LEVEL_THRESHOLD:
            # Within dead zone — hold still
            stop_motor()
            status = "LEVEL"
            speed_out = 0
        else:
            # Calculate correction speed proportional to tilt angle
            raw_speed = KP * tilt
            # Enforce minimum speed so motor actually moves
            if raw_speed > 0:
                speed_out = max(MIN_SPEED, min(MAX_SPEED, raw_speed))
            else:
                speed_out = min(-MIN_SPEED, max(-MAX_SPEED, raw_speed))

            set_motor(speed_out)
            status = "CORRECTING"

        direction = 1 if enc_b.value else -1
        print(f"Tilt: {tilt:+.1f} deg | Speed: {speed_out:+.0f}% | Enc: {enc_a.count * direction} | Status: {status}")
        time.sleep(0.05)  # 20Hz control loop

except KeyboardInterrupt:
    print("\nStopped by user.")
    stop_motor()