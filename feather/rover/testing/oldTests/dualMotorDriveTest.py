# ======================================================================================
# Developer: Noah Wons
# Program: Double motor test using PCA9685 CH0/CH1 for motor 1 and CH2/CH3 for motor 2
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

class Motor:
    def __init__(self, pwm_forward_ch, pwm_reverse_ch, enc_a_pin, enc_b_pin):
        self.pwm_forward_ch = pwm_forward_ch
        self.pwm_reverse_ch = pwm_reverse_ch
        self.enc_a = countio.Counter(enc_a_pin, edge=countio.Edge.RISE)  # pulse count
        self.enc_b = digitalio.DigitalInOut(enc_b_pin)                   # direction
        self.enc_b.direction = digitalio.Direction.INPUT
        self._last_count = 0
        self._last_time = time.monotonic()

    def stop(self):
        set_motor(self, 0)

# --- Motor Initialization ---
motor1 = Motor(pwm_forward_ch=0, pwm_reverse_ch=1, enc_a_pin=board.D11, enc_b_pin=board.D12)
motor2 = Motor(pwm_forward_ch=2, pwm_reverse_ch=3, enc_a_pin=board.D9, enc_b_pin=board.D10)


CPR = 48        # counts per revolution at motor shaft
GEAR_RATIO = 34
COUNTS_PER_REV = CPR * GEAR_RATIO  # 1632 counts per output shaft revolution

def read_rpm(motor):
    """Returns current output shaft RPM (signed: + forward, - reverse)."""
    now = time.monotonic()
    dt = now - motor._last_time
    if dt < 0.05:
        return None  # not enough time elapsed
    current_count = motor.enc_a.count
    delta = current_count - motor._last_count
    rpm = (delta / COUNTS_PER_REV) / dt * 60
    direction = 1 if motor.enc_b.value else -1
    motor._last_count = current_count
    motor._last_time = now
    return rpm * direction

def set_motor(motor, speed):
    """
    speed: -100 to 100
    positive = forward, negative = reverse, 0 = stop
    """
    speed = max(-100, min(100, speed))
    duty = int(abs(speed) / 100 * 65535)
    if speed > 0:
        pca.channels[motor.pwm_forward_ch].duty_cycle = duty
        pca.channels[motor.pwm_reverse_ch].duty_cycle = 0
    elif speed < 0:
        pca.channels[motor.pwm_forward_ch].duty_cycle = 0
        pca.channels[motor.pwm_reverse_ch].duty_cycle = duty
    else:
        pca.channels[motor.pwm_forward_ch].duty_cycle = 0
        pca.channels[motor.pwm_reverse_ch].duty_cycle = 0

def stop():
    set_motor(motor1, 0)
    set_motor(motor2, 0)

def run_with_rpm(duration_s):
    """Run for duration_s seconds, printing RPM at each encoder update."""
    end_time = time.monotonic() + duration_s
    while time.monotonic() < end_time:
        rpm1 = read_rpm(motor1)
        rpm2 = read_rpm(motor2)
        if rpm1 is not None:
            print(f"  M1 RPM: {rpm1:+.1f}")
        if rpm2 is not None:
            print(f"  M2 RPM: {rpm2:+.1f}")

        time.sleep(0.05)

# --- Test Sequence ---
print("Single Motor Test Starting (CH0/CH1)...")
motor1.enc_a.reset()
motor1._last_count = 0
motor2.enc_a.reset()
motor2._last_count = 0
stop()
time.sleep(1)

try:
    print("FORWARD 50%")
    set_motor(motor1, 50)
    set_motor(motor2, 50)
    run_with_rpm(2)

    print("STOP")
    stop()
    time.sleep(1)

    print("REVERSE 50%")
    set_motor(motor1, -50)
    set_motor(motor2, -50)
    run_with_rpm(2)

    print("STOP")
    stop()
    time.sleep(1)

    print("Ramp up forward: 0 → 75%")
    for speed in range(0, 78, 5):
        set_motor(motor1, speed)
        set_motor(motor2, speed)
        rpm1 = read_rpm(motor1)
        rpm2 = read_rpm(motor2)
        rpm_str1 = f"{rpm1:+.1f}" if rpm1 is not None else "---"
        rpm_str2 = f"{rpm2:+.1f}" if rpm2 is not None else "---"
        print(f"  {speed:3d}%  RPM: {rpm_str1}, {rpm_str2}")
        time.sleep(0.2)
    run_with_rpm(1)

    print("Ramp down: 75% → 0")
    for speed in range(75, -1, -5):
        set_motor(motor1, speed)
        set_motor(motor2, speed)
        rpm1 = read_rpm(motor1)
        rpm2 = read_rpm(motor2)
        rpm_str1 = f"{rpm1:+.1f}" if rpm1 is not None else "---"
        rpm_str2 = f"{rpm2:+.1f}" if rpm2 is not None else "---"
        print(f"  {speed:3d}%  RPM: {rpm_str1}, {rpm_str2}")
        time.sleep(0.2)

    print("Test complete.")
    stop()

except KeyboardInterrupt:
    print("\nStopped by user.")
    stop()

finally:
    pca.deinit()
