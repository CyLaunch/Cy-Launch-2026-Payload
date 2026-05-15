# ======================================================================================
# Developer: Noah Wons
# Program: Slow rotation test for three TB9051FTG H-bridges on PCA9685.
#          Runs all three motors simultaneously at a low duty cycle to verify
#          slow, controlled rotation and encoder/OCC feedback.
#
# Wiring (update pin assignments below to match actual harness):
#   Motor 1: PCA9685 CH2 (fwd) / CH3 (rev) | OCC: D6  | Enc A: D9  | Enc B: D10
#   Motor 2: PCA9685 CH4 (fwd) / CH5 (rev) | OCC: D11 | Enc A: D12 | Enc B: D13
#   Motor 3: PCA9685 CH6 (fwd) / CH7 (rev) | OCC: A0  | Enc A: A1  | Enc B: A2
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
SLOW_SPEED  = 100    # % duty cycle (target speed)
RUN_TIME    = 3.0   # seconds per direction
RAMP_TIME   = 1.0   # seconds to ramp from 0 to target speed
RAMP_STEPS  = 20    # number of increments during ramp


# =============================================================================
# Motor class — abstracts PWM, OCC, and encoder for one TB9051FTG channel pair
# =============================================================================
class Motor:
    def __init__(self, pca, ch_fwd, ch_rev, occ_pin, enc_a_pin=None, enc_b_pin=None, name="Motor"):
        self.name = name
        self._pca = pca
        self._ch_fwd = ch_fwd
        self._ch_rev = ch_rev

        self.occ = digitalio.DigitalInOut(occ_pin)
        self.occ.direction = digitalio.Direction.INPUT
        self.occ.pull = digitalio.Pull.UP

        if enc_a_pin is not None:
            self.enc_a = countio.Counter(enc_a_pin, edge=countio.Edge.RISE)
        else:
            self.enc_a = None

        if enc_b_pin is not None:
            self.enc_b = digitalio.DigitalInOut(enc_b_pin)
            self.enc_b.direction = digitalio.Direction.INPUT
        else:
            self.enc_b = None

    def set_speed(self, speed):
        """speed: -100 to 100. Positive = forward, negative = reverse, 0 = stop."""
        speed = max(-100, min(100, speed))
        duty = int(abs(speed) / 100 * 65535)
        if speed > 0:
            self._pca.channels[self._ch_fwd].duty_cycle = duty
            self._pca.channels[self._ch_rev].duty_cycle = 0
        elif speed < 0:
            self._pca.channels[self._ch_fwd].duty_cycle = 0
            self._pca.channels[self._ch_rev].duty_cycle = duty
        else:
            self._pca.channels[self._ch_fwd].duty_cycle = 0
            self._pca.channels[self._ch_rev].duty_cycle = 0

    def stop(self):
        self.set_speed(0)

    @property
    def encoder_count(self):
        if self.enc_a is None:
            return None
        direction = 1 if (self.enc_b.value if self.enc_b else True) else -1
        return self.enc_a.count * direction

    @property
    def overcurrent(self):
        return not self.occ.value

    def reset_encoder(self):
        if self.enc_a is not None:
            self.enc_a.reset()

    def deinit(self):
        self.stop()
        self.occ.deinit()
        if self.enc_a is not None:
            self.enc_a.deinit()
        if self.enc_b is not None:
            self.enc_b.deinit()


# =============================================================================
# Hardware setup
# =============================================================================
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
pca.frequency = 1000
print("  PCA9685 OK")

motors = [
    Motor(pca, ch_fwd=0, ch_rev=1, occ_pin=board.D6, name="Drive"),
    Motor(pca, ch_fwd=11, ch_rev=10, occ_pin=board.D10, name="Orientation"),
]


# =============================================================================
# Helpers
# =============================================================================
def stop_all():
    for m in motors:
        m.stop()


def ramp_all(target_speed):
    """Gradually ramp all motors from 0 to target_speed over RAMP_TIME seconds."""
    step_delay = RAMP_TIME / RAMP_STEPS
    sign = 1 if target_speed >= 0 else -1
    for i in range(1, RAMP_STEPS + 1):
        current = sign * (abs(target_speed) * i / RAMP_STEPS)
        for m in motors:
            m.set_speed(current)
        time.sleep(step_delay)


def run_all_and_report(speed, duration):
    """Run all motors at speed% simultaneously, printing status every 0.25 s."""
    for m in motors:
        m.set_speed(speed)

    end = time.monotonic() + duration
    last_print = 0
    faulted = []

    while time.monotonic() < end:
        now = time.monotonic()
        if now - last_print >= 0.25:
            parts = []
            for m in motors:
                occ_str = " OCC!" if m.overcurrent else ""
                enc = m.encoder_count
                enc_str = f"{enc:6d}" if enc is not None else "   N/A"
                parts.append(f"{m.name}: {enc_str}{occ_str}")
                if m.overcurrent and m.name not in faulted:
                    faulted.append(m.name)
            print("  " + "  |  ".join(parts))
            last_print = now

        if faulted:
            print(f"Overcurrent on {faulted} — stopping all.")
            stop_all()
            return


# =============================================================================
# Test sequence
# =============================================================================
print("=" * 60)
print("  TRIPLE SLOW MOTOR TEST  (M1: CH2/3 | M2: CH4/5 | M3: CH6/7)")
print(f"  Speed: {SLOW_SPEED}%  |  Run time: {RUN_TIME}s per direction")
print("=" * 60)

for m in motors:
    m.reset_encoder()

try:
    print(f"\nFORWARD {SLOW_SPEED}%  (ramping over {RAMP_TIME}s)")
    ramp_all(SLOW_SPEED)
    run_all_and_report(SLOW_SPEED, RUN_TIME)
    stop_all()
    time.sleep(1)



    print("\nTest complete.")

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    stop_all()
    for m in motors:
        m.deinit()
    pca.deinit()
