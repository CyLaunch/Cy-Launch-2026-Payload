# ======================================================================================
# Developer: Noah Wons
# Program: Slow rotation test for TB9051FTG H-bridge motor on PCA9685 channels 2 & 3.
#          Runs the motor at a low duty cycle to verify slow, controlled rotation.
# Wiring:
#   Motor (H-bridge): PCA9685 CH2 (forward) / CH3 (reverse)
#   OCC pin:          D6  (active LOW — False = overcurrent)
#   Encoder A:        D9
#   Encoder B:        D10
#
# Tuning:
#   SLOW_SPEED — % duty cycle (1–100). Start low (~15) and increase if motor stalls.
#   RUN_TIME   — seconds to run in each direction before stopping.
#
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import busio
import countio
import digitalio
from adafruit_pca9685 import PCA9685

# --- Tuning ---
SLOW_SPEED = 30    # % duty cycle — raise if motor doesn't start; lower once it spins
RUN_TIME   = 3.0   # seconds per direction

# --- PCA9685 channel assignments ---
CH_FWD = 2
CH_REV = 3

# --- I2C + PCA9685 ---
i2c = busio.I2C(board.SCL, board.SDA)

print("Scanning I2C bus...")
while not i2c.try_lock():
    pass
found = [hex(a) for a in i2c.scan()]
i2c.unlock()
print(f"  Devices found: {found}")
if "0x40" not in found:
    print("  ERROR: PCA9685 not found at 0x40 — check wiring/power and restart.")
    while True:
        time.sleep(1)

pca = PCA9685(i2c)
pca.frequency = 1000   # 1kHz — matches TB9051FTG setup
print("  PCA9685 OK")

# --- OCC (overcurrent, active LOW) ---
occ = digitalio.DigitalInOut(board.D6)
occ.direction = digitalio.Direction.INPUT
occ.pull = digitalio.Pull.UP

# --- Encoder ---
enc_a = countio.Counter(board.D9, edge=countio.Edge.RISE)
enc_b = digitalio.DigitalInOut(board.D10)
enc_b.direction = digitalio.Direction.INPUT


def set_motor(speed):
    """speed: -100 to 100. Positive = forward, negative = reverse, 0 = stop."""
    speed = max(-100, min(100, speed))
    duty = int(abs(speed) / 100 * 65535)
    if speed > 0:
        pca.channels[CH_FWD].duty_cycle = duty
        pca.channels[CH_REV].duty_cycle = 0
    elif speed < 0:
        pca.channels[CH_FWD].duty_cycle = 0
        pca.channels[CH_REV].duty_cycle = duty
    else:
        pca.channels[CH_FWD].duty_cycle = 0
        pca.channels[CH_REV].duty_cycle = 0


def stop():
    set_motor(0)


def run_and_report(speed, duration):
    """Run motor at speed% for duration seconds, printing encoder count + OCC every 0.25s."""
    set_motor(speed)
    end = time.monotonic() + duration
    last_print = 0
    while time.monotonic() < end:
        now = time.monotonic()
        if now - last_print >= 0.25:
            direction = 1 if enc_b.value else -1
            count = enc_a.count * direction
            overcurrent = not occ.value
            occ_str = "  *** OVERCURRENT ***" if overcurrent else ""
            print(f"  Speed: {speed:+3d}%  |  Encoder: {count:6d}{occ_str}")
            last_print = now
            if overcurrent:
                print("Stopping due to overcurrent.")
                stop()
                return


# =============================================================================
# Test sequence
# =============================================================================

print("=" * 50)
print("  SLOW MOTOR TEST  (CH2/CH3)")
print(f"  Speed: {SLOW_SPEED}%  |  Run time: {RUN_TIME}s per direction")
print("=" * 50)

enc_a.reset()

try:
    print(f"\nFORWARD {SLOW_SPEED}%")
    run_and_report(SLOW_SPEED, RUN_TIME)
    stop()
    time.sleep(1)

    print(f"\nREVERSE {SLOW_SPEED}%")
    run_and_report(-SLOW_SPEED, RUN_TIME)
    stop()
    time.sleep(1)

    print("\nTest complete.")

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    stop()
    pca.deinit()
