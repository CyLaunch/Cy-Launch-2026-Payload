import time
import board
import busio
from adafruit_pca9685 import PCA9685

# ── MODE SELECT ───────────────────────────────────────────────────────────────
CALIBRATE_MODE = False   # Set True for first run, False after calibration

# ── I2C + PCA9685 Setup ───────────────────────────────────────────────────────
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # ESCs require 50Hz PWM

# ── ESC Pulse Width Settings (µs) ─────────────────────────────────────────────
# Tune these after calibration if motors still feel too fast/slow
ESC_MIN     = 1400   # Minimum throttle (raised floor for Bigfoot 25)
ESC_NEUTRAL = 1500   # Neutral / stopped
ESC_MAX     = 1600   # Maximum throttle (lowered ceiling for Bigfoot 25)

# ── Channel Assignment ────────────────────────────────────────────────────────
MOTOR_1 = pca.channels[1]   # First terminal
MOTOR_2 = pca.channels[2]   # Second terminal

# ── Core PWM Helpers ──────────────────────────────────────────────────────────
def us_to_duty(pulse_us):
    """Convert microsecond pulse width to PCA9685 16-bit duty cycle."""
    return int((pulse_us / 20000) * 65535)

def set_motor(channel, pulse_us):
    """Set a motor to a raw pulse width in microseconds (clamped to safe range)."""
    pulse_us = max(1000, min(2000, pulse_us))
    channel.duty_cycle = us_to_duty(pulse_us)

def set_speed(channel, speed):
    """
    Set motor speed on a -100 to +100 scale.
      -100 = full reverse
         0 = neutral / stopped
      +100 = full forward
    Note: negative values only work if your ESC supports reverse.
    """
    speed = max(-100, min(100, speed))  # clamp input
    if speed >= 0:
        pulse_us = ESC_NEUTRAL + (speed / 100) * (ESC_MAX - ESC_NEUTRAL)
    else:
        pulse_us = ESC_NEUTRAL + (speed / 100) * (ESC_NEUTRAL - ESC_MIN)
    set_motor(channel, pulse_us)

def stop_all():
    """Send neutral signal to all motors."""
    set_motor(MOTOR_1, ESC_NEUTRAL)
    set_motor(MOTOR_2, ESC_NEUTRAL)


# ── CALIBRATION ROUTINE ───────────────────────────────────────────────────────
def calibrate():
    print("=" * 50)
    print("  ESC CALIBRATION MODE")
    print("=" * 50)
    print()
    print("!! SAFETY WARNING !!")
    print("  Disconnect props/wheels or secure your vehicle.")
    print("  Motors WILL spin during the test phase.")
    print()
    print("Starting in 5 seconds — disconnect anything unsafe now!")
    for i in range(5, 0, -1):
        print(f"  {i}...")
        time.sleep(1)

    # Step 1: Send maximum signal
    print()
    print("Step 1/3 — Sending MAX throttle signal (2000us)...")
    print("  Wait for ESC startup beeps...")
    set_motor(MOTOR_1, 2000)
    set_motor(MOTOR_2, 2000)
    time.sleep(4)

    # Step 2: Send minimum signal
    print("Step 2/3 — Sending MIN throttle signal (1000us)...")
    print("  Wait for confirmation beeps (usually 2-3 beeps)...")
    set_motor(MOTOR_1, 1000)
    set_motor(MOTOR_2, 1000)
    time.sleep(4)

    # Step 3: Arm at neutral
    print("Step 3/3 — Sending NEUTRAL signal to arm ESCs...")
    stop_all()
    time.sleep(2)

    print()
    print("Calibration complete!")
    print()
    print("Running a brief motor test at LOW speed (15%)...")
    print("If motors spin, calibration was successful.")
    print()
    time.sleep(1)

    # Brief low-speed confirmation test
    for speed in range(0, 18, 3):
        set_speed(MOTOR_1, speed)
        set_speed(MOTOR_2, speed)
        print(f"  Speed: {speed}%")
        time.sleep(0.5)

    time.sleep(1)
    stop_all()
    print()
    print("Test complete. Motors stopped.")
    print()
    print("-" * 50)
    print("Next step:")
    print("  Set CALIBRATE_MODE = False at the top of this")
    print("  file and save to run normal operation.")
    print("-" * 50)


# ── NORMAL OPERATION ──────────────────────────────────────────────────────────
def run():
    print("=" * 50)
    print("  NORMAL OPERATION MODE")
    print("=" * 50)

    # Arm ESCs
    print("\nArming ESCs — sending neutral for 3 seconds...")
    stop_all()
    time.sleep(3)
    print("Armed! Starting test sequence...\n")

    try:
        while True:

            # Ramp Motor 1 forward, Motor 2 stopped
            print("Motor 1: ramping up | Motor 2: stopped")
            for speed in range(0, 28, 3):   # 0% to ~25%
                set_speed(MOTOR_1, speed)
                set_speed(MOTOR_2, 0)
                print(f"  M1={speed:+4d}%  M2={0:+4d}%")
                time.sleep(0.4)
            time.sleep(1)

            # Both motors at 25%
            print("Both motors: 25% forward")
            set_speed(MOTOR_1, 25)
            set_speed(MOTOR_2, 25)
            time.sleep(2)

            # Ramp both down
            print("Ramping both motors down...")
            for speed in range(25, -1, -3):
                set_speed(MOTOR_1, speed)
                set_speed(MOTOR_2, speed)
                print(f"  M1={speed:+4d}%  M2={speed:+4d}%")
                time.sleep(0.4)
            time.sleep(1)

            # Ramp Motor 2 forward, Motor 1 stopped
            print("Motor 1: stopped | Motor 2: ramping up")
            for speed in range(0, 28, 3):
                set_speed(MOTOR_1, 0)
                set_speed(MOTOR_2, speed)
                print(f"  M1={0:+4d}%  M2={speed:+4d}%")
                time.sleep(0.4)
            time.sleep(1)

            # Stop all and repeat
            print("Stopping all motors...\n")
            stop_all()
            time.sleep(2)

    except KeyboardInterrupt:
        print("\nStopping all motors and exiting.")
        stop_all()
        pca.deinit()


# ── Entry Point ───────────────────────────────────────────────────────────────
if CALIBRATE_MODE:
    calibrate()
else:
    run()