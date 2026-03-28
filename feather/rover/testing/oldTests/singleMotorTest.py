# ======================================================================================
# Developer: Noah Wons
# Program: Simple single-motor test using PCA9685 CH0 (PWM1) and CH1 (PWM2)
# Additional Notes:
# - Motor is wired to a TB9051FTG driver with PWM1 on PCA9685 channel 0 and PWM2 on channel 1.
# - CH0 > 0, CH1 = 0 → forward
# - CH0 = 0, CH1 > 0 → reverse
# - CH0 = 0, CH1 = 0 → stop
#
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import busio
import countio
import digitalio
from adafruit_pca9685 import PCA9685

# --- Setup I2C and PCA9685 ---
i2c = busio.I2C(board.SCL, board.SDA)

print("Scanning I2C bus...")
while not i2c.try_lock():
    pass
found = [hex(a) for a in i2c.scan()]
i2c.unlock()
print(f"  Devices found: {found}")
if "0x40" not in found:
    print("  WARNING: PCA9685 not found at 0x40 — check SDA/SCL wiring and power.")

pca = None
try:
    pca = PCA9685(i2c)
    pca.frequency = 1000  # 1kHz PWM for TB9051FTG motor driver
    print("  PCA9685 initialized OK")
except Exception as e:
    print(f"  ERROR: PCA9685 init failed: {e}")
    while True:
        time.sleep(1)

# --- Encoder setup ---
enc_a = countio.Counter(board.D11, edge=countio.Edge.RISE)  # pulse count
enc_b = digitalio.DigitalInOut(board.D12)                   # direction
enc_b.direction = digitalio.Direction.INPUT

CPR = 48        # counts per revolution at motor shaft
GEAR_RATIO = 34
COUNTS_PER_REV = CPR * GEAR_RATIO  # 1632 counts per output shaft revolution

_last_count = 0
_last_time = time.monotonic()

def read_rpm():
    """Returns current output shaft RPM (signed: + forward, - reverse)."""
    global _last_count, _last_time
    now = time.monotonic()
    dt = now - _last_time
    if dt < 0.05:
        return None  # not enough time elapsed
    current_count = enc_a.count
    delta = current_count - _last_count
    rpm = (delta / COUNTS_PER_REV) / dt * 60
    direction = 1 if enc_b.value else -1
    _last_count = current_count
    _last_time = now
    return rpm * direction

# --- Motor channel assignments ---
MOTOR_PWM1 = 0  # CH0 — forward drive
MOTOR_PWM2 = 1  # CH1 — reverse drive

def set_motor(speed):
    """
    speed: -100 to 100
    positive = forward, negative = reverse, 0 = stop
    """
    speed = max(-100, min(100, speed))
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

def stop():
    set_motor(0)

def run_with_rpm(duration_s):
    """Run for duration_s seconds, printing RPM at each encoder update."""
    end_time = time.monotonic() + duration_s
    while time.monotonic() < end_time:
        rpm = read_rpm()
        if rpm is not None:
            print(f"  RPM: {rpm:+.1f}")
        time.sleep(0.05)

# --- Test Sequence ---
print("Single Motor Test Starting (CH0/CH1)...")
enc_a.reset()
stop()
time.sleep(1)

try:
    print("FORWARD 50%")
    set_motor(50)
    run_with_rpm(2)

    print("STOP")
    stop()
    time.sleep(1)

    print("REVERSE 50%")
    set_motor(-50)
    run_with_rpm(2)

    print("STOP")
    stop()
    time.sleep(1)

    print("Ramp up forward: 0 → 75%")
    for speed in range(0, 78, 5):
        set_motor(speed)
        rpm = read_rpm()
        rpm_str = f"{rpm:+.1f}" if rpm is not None else "---"
        print(f"  {speed:3d}%  RPM: {rpm_str}")
        time.sleep(0.2)
    run_with_rpm(1)

    print("Ramp down: 75% → 0")
    for speed in range(75, -1, -5):
        set_motor(speed)
        rpm = read_rpm()
        rpm_str = f"{rpm:+.1f}" if rpm is not None else "---"
        print(f"  {speed:3d}%  RPM: {rpm_str}")
        time.sleep(0.2)

    print("Test complete.")
    stop()

except KeyboardInterrupt:
    print("\nStopped by user.")
    stop()

finally:
    pca.deinit()
