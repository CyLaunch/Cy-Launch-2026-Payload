# ======================================================================================
# Developer: Noah Wons
# Program: Shake-triggered servo test for nosecone system
#          Moves servo when ICM-20948 detects a shake (accel spike above threshold)
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import math
import board
import pwmio
import adafruit_icm20x
from adafruit_motor import servo

# ---- Servo setup ----
SERVO_REST  = 0    # degrees — resting position
SERVO_FIRE  = 90   # degrees — deployed position
HOLD_TIME   = 1.0  # seconds to hold deployed position before returning

pwm = pwmio.PWMOut(board.D5, duty_cycle=0, frequency=50)
my_servo = servo.Servo(pwm, min_pulse=500, max_pulse=2500)
my_servo.angle = SERVO_REST

# ---- Shake detection ----
SHAKE_THRESHOLD = 15.0  # m/s² — resting is ~9.8; a firm shake exceeds this
COOLDOWN        = 2.0   # seconds before another trigger is allowed

# ---- IMU setup ----
i2c = board.I2C()
icm = None
for addr in (0x69, 0x68):
    try:
        icm = adafruit_icm20x.ICM20948(i2c, address=addr)
        print(f"ICM-20948 found at 0x{addr:02X}")
        break
    except Exception as e:
        print(f"Not at 0x{addr:02X}: {e}")

if icm is None:
    print("ERROR: ICM-20948 not detected. Check wiring.")
    while True:
        time.sleep(1)

print("Ready — shake the Feather to trigger the servo.")
print(f"Shake threshold: {SHAKE_THRESHOLD} m/s²  |  Servo: {SERVO_REST}° → {SERVO_FIRE}°")

last_trigger = -COOLDOWN  # allow immediate first trigger

while True:
    ax, ay, az = icm.acceleration
    a_mag = math.sqrt(ax * ax + ay * ay + az * az)

    now = time.monotonic()
    if a_mag > SHAKE_THRESHOLD and (now - last_trigger) > COOLDOWN:
        print(f"Shake detected! |a| = {a_mag:.2f} m/s²  → firing servo")
        my_servo.angle = SERVO_FIRE
        time.sleep(HOLD_TIME)
        my_servo.angle = SERVO_REST
        print("Servo returned to rest.")
        last_trigger = time.monotonic()

    time.sleep(0.02)  # 50 Hz polling
