# ======================================================================================
# Developer: Noah Wons
# Program: Component-gated servo test for nosecone system
#          Rotates servo to 45 degrees only if both the ICM-20948 IMU
#          and MPL3115A2 altimeter are successfully detected on I2C.
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import pwmio
import adafruit_icm20x
import adafruit_mpl3115a2
from adafruit_motor import servo

# ---- Servo setup ----
SERVO_PIN  = board.D5
SERVO_REST = 0    # degrees — resting position
SERVO_TARGET = 45 # degrees — target position if both sensors detected

pwm = pwmio.PWMOut(SERVO_PIN, duty_cycle=0, frequency=50)
my_servo = servo.Servo(pwm, min_pulse=500, max_pulse=2500)
my_servo.angle = SERVO_REST

# ---- I2C setup ----
i2c = board.I2C()

print("=" * 50)
print("  NOSECONE COMPONENT + SERVO TEST")
print("=" * 50)

# ---- Detect ICM-20948 IMU ----
print("\n[ ICM-20948 — IMU ]")
icm = None
for addr in (0x69, 0x68):
    try:
        icm = adafruit_icm20x.ICM20948(i2c, address=addr)
        print(f"  PASS  ICM-20948 found at 0x{addr:02X}")
        break
    except Exception as e:
        print(f"  --    Not at 0x{addr:02X}: {e}")

if icm is None:
    print("  FAIL  ICM-20948 not detected — check wiring.")

# ---- Detect MPL3115A2 altimeter ----
print("\n[ MPL3115A2 — Altimeter ]")
mpl = None
try:
    mpl = adafruit_mpl3115a2.MPL3115A2(i2c)
    print("  PASS  MPL3115A2 found at 0x60")
except Exception as e:
    print(f"  FAIL  MPL3115A2 not detected: {e}")

# ---- Summary and servo action ----
print()
print("=" * 50)
both_detected = (icm is not None) and (mpl is not None)

if both_detected:
    print("  Both sensors detected — rotating servo to 45 degrees.")
    my_servo.angle = SERVO_TARGET
    time.sleep(1.0)
    print(f"  Servo at {SERVO_TARGET} degrees. Returning to rest.")
    time.sleep(1.0)
    my_servo.angle = SERVO_REST
    print("  Servo returned to rest.")
else:
    missing = []
    if icm is None:
        missing.append("ICM-20948")
    if mpl is None:
        missing.append("MPL3115A2")
    print(f"  Sensor(s) missing: {', '.join(missing)}")
    print("  Servo will not move until all sensors are detected.")

print("=" * 50)
