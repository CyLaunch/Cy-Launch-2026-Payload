# ======================================================================================
# Developer: Noah Wons
# Program: Test script for orientation-based motor control using BNO055 IMU and PCA9685 on FeatherWing M4 Express
# Additional Notes:
# - This script reads the roll angle from the BNO055 IMU and uses it to control a single motor for leveling.
# - The motor will attempt to correct any tilt by running in the appropriate direction and speed proportional to the angle of tilt.
# - A dead zone is implemented to prevent constant small corrections when the system is nearly level.
# - Tuning parameters (KP, MAX_SPEED, MIN_SPEED) may need adjustment based on your specific motor and mechanical setup.
# - NOTE: Ensure the orientation motor is on PWM channels 0 and 1.
# 
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import busio
from adafruit_pca9685 import PCA9685
import adafruit_bno055

# --- Setup I2C, PCA9685, and BNO055 ---
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # 1kHz PWM for motor drivers
bno = adafruit_bno055.BNO055_I2C(i2c)

# --- Motor 1 (Leveling) is on CH0 (PWM1) and CH1 (PWM2) ---
MOTOR_PWM1 = 0
MOTOR_PWM2 = 1

# --- Tuning Parameters ---
LEVEL_THRESHOLD = 2.0    # degrees — if tilt is within this range, do nothing (dead zone)
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

# --- Wait for BNO055 to calibrate ---
print("Waiting for BNO055 to be ready...")
time.sleep(1)

print("Starting leveling loop. Press CTRL+C to stop.")
print(f"Dead zone: +/- {LEVEL_THRESHOLD} degrees")
print(f"Max speed: {MAX_SPEED}%")
print()

try:
    while True:
        # Read Euler angles from BNO055
        # euler returns (heading, roll, pitch) in degrees
        euler = bno.euler

        if euler is None or euler[1] is None:
            print("No IMU data — check wiring!")
            stop_motor()
            time.sleep(0.5)
            continue

        heading, roll, pitch = euler

        # Use ROLL to determine tilt (side-to-side leveling)
        # Swap to 'pitch' if your motor corrects front-to-back tilt instead
        tilt = roll

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

        print(f"Tilt: {tilt:+.1f} deg | Speed: {speed_out:+.0f}% | Status: {status}")
        time.sleep(0.05)  # 20Hz control loop

except KeyboardInterrupt:
    print("\nStopped by user.")
    stop_motor()