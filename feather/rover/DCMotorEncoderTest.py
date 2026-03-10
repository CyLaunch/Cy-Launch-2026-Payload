# ======================================================================================
# Developer: Noah Wons
# Program: Test script for closed-loop RPM control of DC motors with encoders using PCA9685 on FeatherWing M4 Express
# Additional Notes:
# - In this script we use the GPIO pins on the feather to count encoder pulses to determine direction and RPM of the motors.
# - We can use this information to deliver more or less voltage to the motors to maintain a target RPM.
# 
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import busio
import countio
import digitalio
from adafruit_pca9685 import PCA9685

# =============================================================================
# SETUP
# =============================================================================

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000

# --- Encoder A channels (pulse counting) ---
enc_drive = countio.Counter(board.D9,  edge=countio.Edge.RISE)  # Motor 2 - drive belt
enc_blade = countio.Counter(board.D11, edge=countio.Edge.RISE)  # Motor 3 - blades

# --- Encoder B channels (direction detection) ---
enc_drive_b = digitalio.DigitalInOut(board.D10)
enc_blade_b = digitalio.DigitalInOut(board.D12)
enc_drive_b.direction = digitalio.Direction.INPUT
enc_blade_b.direction = digitalio.Direction.INPUT

# --- Gearbox Constants ---
CPR = 48        # counts per revolution at motor shaft
GEAR_RATIO = 34
COUNTS_PER_REV = CPR * GEAR_RATIO  # 1632 counts per output shaft revolution

# =============================================================================
# MOTOR CONTROL
# =============================================================================

def set_motor(pwm1_ch, pwm2_ch, speed):
    """
    speed: -100 to 100
    positive = forward, negative = reverse, 0 = stop
    """
    speed = max(-100, min(100, speed))
    duty = int(abs(speed) / 100 * 65535)
    if speed > 0:
        pca.channels[pwm1_ch].duty_cycle = duty
        pca.channels[pwm2_ch].duty_cycle = 0
    elif speed < 0:
        pca.channels[pwm1_ch].duty_cycle = 0
        pca.channels[pwm2_ch].duty_cycle = duty
    else:
        pca.channels[pwm1_ch].duty_cycle = 0
        pca.channels[pwm2_ch].duty_cycle = 0

def stop_all():
    set_motor(2, 3, 0)
    set_motor(4, 5, 0)

# =============================================================================
# CLOSED LOOP RPM CONTROLLER
# =============================================================================

class MotorController:
    def __init__(self, pwm1_ch, pwm2_ch, encoder, b_pin, target_rpm):
        self.pwm1_ch = pwm1_ch
        self.pwm2_ch = pwm2_ch
        self.encoder = encoder
        self.b_pin = b_pin
        self.target_rpm = target_rpm

        self.power = 0          # current power output %
        self.last_count = 0     # last encoder count
        self.last_time = time.monotonic()

        # Tuning
        self.KP = 0.8           # proportional gain
        self.MAX_POWER = 80     # cap power to avoid overcurrent
        self.MIN_POWER = 15     # minimum power to overcome stiction

    def update(self):
        now = time.monotonic()
        dt = now - self.last_time

        if dt < 0.05:           # only update at ~20Hz
            return

        # --- Calculate actual RPM ---
        current_count = self.encoder.count
        delta_counts = current_count - self.last_count
        actual_rpm = (delta_counts / COUNTS_PER_REV) / dt * 60

        # --- Determine direction from B pin ---
        direction = 1 if self.b_pin.value else -1
        actual_rpm *= direction

        # --- Proportional correction ---
        error = self.target_rpm - actual_rpm
        correction = self.KP * error
        self.power += correction

        # --- Clamp power ---
        if abs(self.power) < self.MIN_POWER and abs(self.target_rpm) > 0:
            self.power = self.MIN_POWER if self.target_rpm > 0 else -self.MIN_POWER
        self.power = max(-self.MAX_POWER, min(self.MAX_POWER, self.power))

        set_motor(self.pwm1_ch, self.pwm2_ch, self.power)

        # --- Debug output ---
        print(f"  Target: {self.target_rpm:.0f} RPM | "
              f"Actual: {actual_rpm:.1f} RPM | "
              f"Power: {self.power:.1f}%")

        # --- Store state ---
        self.last_count = current_count
        self.last_time = now

# =============================================================================
# TEST SEQUENCE
# =============================================================================

# Target RPMs — adjust these to match your mission needs
DRIVE_TARGET_RPM  = 60   # output shaft RPM for drive belt
BLADE_TARGET_RPM  = 80   # output shaft RPM for collection blades

drive_ctrl = MotorController(2, 3, enc_drive, enc_drive_b, DRIVE_TARGET_RPM)
blade_ctrl = MotorController(4, 5, enc_blade, enc_blade_b, BLADE_TARGET_RPM)

print("Closed Loop Motor Test Starting...")
print(f"Drive target:  {DRIVE_TARGET_RPM} RPM")
print(f"Blade target:  {BLADE_TARGET_RPM} RPM")
print()
time.sleep(1)

try:
    # --- Test Drive Motor alone ---
    print("--- Testing Drive Motor ---")
    drive_ctrl.power = 0
    enc_drive.reset()
    end_time = time.monotonic() + 5  # run for 5 seconds
    while time.monotonic() < end_time:
        print("DRIVE:", end=" ")
        drive_ctrl.update()
        time.sleep(0.05)

    set_motor(2, 3, 0)
    print("Drive motor stopped.\n")
    time.sleep(1)

    # --- Test Blade Motor alone ---
    print("--- Testing Blade Motor ---")
    blade_ctrl.power = 0
    enc_blade.reset()
    end_time = time.monotonic() + 5  # run for 5 seconds
    while time.monotonic() < end_time:
        print("BLADE:", end=" ")
        blade_ctrl.update()
        time.sleep(0.05)

    set_motor(4, 5, 0)
    print("Blade motor stopped.\n")
    time.sleep(1)

    # --- Test Both Together ---
    print("--- Testing Both Motors Simultaneously ---")
    drive_ctrl.power = 0
    blade_ctrl.power = 0
    enc_drive.reset()
    enc_blade.reset()
    end_time = time.monotonic() + 5
    while time.monotonic() < end_time:
        print("DRIVE:", end=" ")
        drive_ctrl.update()
        print("BLADE:", end=" ")
        blade_ctrl.update()
        time.sleep(0.05)

    stop_all()
    print("\nAll tests complete!")

except KeyboardInterrupt:
    print("\nStopped by user.")
    stop_all()