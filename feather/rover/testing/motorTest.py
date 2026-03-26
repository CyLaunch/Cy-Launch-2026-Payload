# ======================================================================================
# Developer: Noah Wons
# Program: All-motor test — brushed ESC (motor 1) + two TB9051FTG H-bridge motors (2 & 3)
# Wiring:
#   Motor 1 (ESC):      PWM signal → D5  | Encoder A → D6,  B → D7
#   Motor 2 (H-bridge): PCA9685 CH0/CH1  | Encoder A → D11, B → D12
#   Motor 3 (H-bridge): PCA9685 CH2/CH3  | Encoder A → D9,  B → D10
#
#   TB9051FTG wiring: CH_fwd > 0, CH_rev = 0 → forward
#                     CH_fwd = 0, CH_rev > 0 → reverse
#                     CH_fwd = 0, CH_rev = 0 → stop
#
#   Assumed wiring:                                                                                                                                                                                                          
                                                                                                                                                                                                                         
#   ┌───────────────┬─────────────────┬───────┬───────┐                                                                                                                                                                      
#   │     Motor     │     Control     │ Enc A │ Enc B │                                                                                                                                                                    
#   ├───────────────┼─────────────────┼───────┼───────┤                                                                                                                                                                      
#   │ M1 (ESC)      │ D5 (50Hz PWM)   │ D6    │ D7    │                                                                                                                                                                    
#   ├───────────────┼─────────────────┼───────┼───────┤                                                                                                                                                                    
#   │ M2 (H-bridge) │ PCA9685 CH0/CH1 │ D11   │ D12   │                                                                                                                                                                      
#   ├───────────────┼─────────────────┼───────┼───────┤                                                                                                                                                                      
#   │ M3 (H-bridge) │ PCA9685 CH2/CH3 │ D9    │ D10   │                                                                                                                                                                      
#   └───────────────┴─────────────────┴───────┴───────┘
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import busio
import pwmio
import countio
import digitalio
from adafruit_pca9685 import PCA9685

# --- Constants ---
CPR = 48
GEAR_RATIO = 34.014
COUNTS_PER_OUTPUT_REV = CPR * GEAR_RATIO   # ~1632 counts per output shaft revolution

# --- I2C + PCA9685 (for H-bridge motors) ---
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
    pca.frequency = 1000   # 1kHz PWM for TB9051FTG
    print("  PCA9685 initialized OK")
except Exception as e:
    print(f"  ERROR: PCA9685 init failed: {e}")
    while True:
        time.sleep(1)


# =============================================================================
# Motor classes
# =============================================================================

class ESCMotor:
    """Brushed ESC motor controlled via a single PWM signal (1000–2000µs at 50Hz).
    Encoder is polled — call poll_encoder() as frequently as possible."""

    def __init__(self, pwm_pin, enc_a_pin, enc_b_pin):
        self.esc = pwmio.PWMOut(pwm_pin, frequency=50)

        self.enc_a = digitalio.DigitalInOut(enc_a_pin)
        self.enc_a.direction = digitalio.Direction.INPUT
        self.enc_a.pull = digitalio.Pull.UP

        self.enc_b = digitalio.DigitalInOut(enc_b_pin)
        self.enc_b.direction = digitalio.Direction.INPUT
        self.enc_b.pull = digitalio.Pull.UP

        self._last_a = self.enc_a.value
        self.encoder_count = 0
        self._last_count = 0
        self._last_time = time.monotonic()

    def set_pulse(self, pulse_us):
        """Set ESC pulse width in microseconds (1000 = stop, 2000 = full forward)."""
        pulse_us = max(1000, min(2000, pulse_us))
        self.esc.duty_cycle = int((pulse_us / 20000) * 65535)

    def stop(self):
        self.set_pulse(1000)

    def poll_encoder(self):
        """Detect encoder edges. Must be called frequently to avoid missing pulses."""
        current_a = self.enc_a.value
        if current_a != self._last_a:
            if self.enc_b.value != current_a:
                self.encoder_count += 1
            else:
                self.encoder_count -= 1
        self._last_a = current_a

    def read_rpm(self):
        """Returns output shaft RPM since last call, or None if < 50ms elapsed."""
        now = time.monotonic()
        dt = now - self._last_time
        if dt < 0.05:
            return None
        delta = self.encoder_count - self._last_count
        rpm = (abs(delta) / COUNTS_PER_OUTPUT_REV) / (dt / 60)
        self._last_count = self.encoder_count
        self._last_time = now
        return rpm


class HBridgeMotor:
    """TB9051FTG motor controlled via two PCA9685 channels (forward + reverse).
    Encoder uses countio for reliable edge counting."""

    def __init__(self, fwd_ch, rev_ch, enc_a_pin, enc_b_pin):
        self.fwd_ch = fwd_ch
        self.rev_ch = rev_ch
        self.enc_a = countio.Counter(enc_a_pin, edge=countio.Edge.RISE)
        self.enc_b = digitalio.DigitalInOut(enc_b_pin)
        self.enc_b.direction = digitalio.Direction.INPUT
        self._last_count = 0
        self._last_time = time.monotonic()

    def set_speed(self, speed):
        """speed: -100 to 100. Positive = forward, negative = reverse, 0 = stop."""
        speed = max(-100, min(100, speed))
        duty = int(abs(speed) / 100 * 65535)
        if speed > 0:
            pca.channels[self.fwd_ch].duty_cycle = duty
            pca.channels[self.rev_ch].duty_cycle = 0
        elif speed < 0:
            pca.channels[self.fwd_ch].duty_cycle = 0
            pca.channels[self.rev_ch].duty_cycle = duty
        else:
            pca.channels[self.fwd_ch].duty_cycle = 0
            pca.channels[self.rev_ch].duty_cycle = 0

    def stop(self):
        self.set_speed(0)

    def read_rpm(self):
        """Returns signed output shaft RPM since last call, or None if < 50ms elapsed."""
        now = time.monotonic()
        dt = now - self._last_time
        if dt < 0.05:
            return None
        current = self.enc_a.count
        delta = current - self._last_count
        rpm = (delta / COUNTS_PER_OUTPUT_REV) / dt * 60
        direction = 1 if self.enc_b.value else -1
        self._last_count = current
        self._last_time = now
        return rpm * direction


# =============================================================================
# Motor Initialization
# =============================================================================

motor1 = ESCMotor(pwm_pin=board.D5, enc_a_pin=board.D6, enc_b_pin=board.D7)
motor2 = HBridgeMotor(fwd_ch=0, rev_ch=1, enc_a_pin=board.D11, enc_b_pin=board.D12)
motor3 = HBridgeMotor(fwd_ch=2, rev_ch=3, enc_a_pin=board.D9,  enc_b_pin=board.D10)


# =============================================================================
# Helpers
# =============================================================================

def stop_all():
    motor1.stop()
    motor2.stop()
    motor3.stop()

def reset_encoders():
    motor1.encoder_count = 0
    motor1._last_count = 0
    motor2.enc_a.reset()
    motor2._last_count = 0
    motor3.enc_a.reset()
    motor3._last_count = 0

def run_all(duration_s):
    """Run for duration_s seconds, printing RPM for all motors every 0.5s.
    Polls the ESC encoder at ~100Hz throughout."""
    end = time.monotonic() + duration_s
    last_print = time.monotonic()
    while time.monotonic() < end:
        motor1.poll_encoder()
        now = time.monotonic()
        if now - last_print >= 0.5:
            _print_rpm()
            last_print = now
        time.sleep(0.01)

def _print_rpm():
    rpm1 = motor1.read_rpm()
    rpm2 = motor2.read_rpm()
    rpm3 = motor3.read_rpm()
    r1 = f"{rpm1:6.1f}" if rpm1 is not None else "   ---"
    r2 = f"{rpm2:+6.1f}" if rpm2 is not None else "   ---"
    r3 = f"{rpm3:+6.1f}" if rpm3 is not None else "   ---"
    print(f"  M1(ESC):{r1} RPM  |  M2:{r2} RPM  |  M3:{r3} RPM")


# =============================================================================
# ESC Arming (H-bridge motors held stopped during arming)
# =============================================================================

print("\nArming ESC (motor 1) — H-bridge motors held at stop...")
motor2.stop()
motor3.stop()
motor1.set_pulse(2000)
time.sleep(2)
motor1.stop()
time.sleep(2)
print("ESC armed!\n")

reset_encoders()

# =============================================================================
# Test Sequence
# =============================================================================

print("=" * 60)
print("  ALL-MOTOR TEST SEQUENCE")
print("=" * 60)

try:
    # --- Forward ---
    print("\nFORWARD 50%  (M1: 1250us | M2/M3: 50%)")
    motor1.set_pulse(1250)
    motor2.set_speed(50)
    motor3.set_speed(50)
    run_all(2)

    print("STOP")
    stop_all()
    time.sleep(1)

    # --- Reverse (H-bridge only; ESC stopped) ---
    print("\nREVERSE 50%  (M1 ESC stopped | M2/M3: -50%)")
    motor1.stop()
    motor2.set_speed(-50)
    motor3.set_speed(-50)
    run_all(2)

    print("STOP")
    stop_all()
    time.sleep(1)

    # --- Ramp up ---
    print("\nRamp up:  M1 1000→1500us | M2/M3 0→75%")
    steps = 15
    for i in range(steps + 1):
        frac = i / steps
        pulse = int(1000 + frac * 500)
        spd = int(frac * 75)
        motor1.set_pulse(pulse)
        motor2.set_speed(spd)
        motor3.set_speed(spd)
        motor1.poll_encoder()
        rpm1 = motor1.read_rpm()
        rpm2 = motor2.read_rpm()
        rpm3 = motor3.read_rpm()
        r1 = f"{rpm1:6.1f}" if rpm1 is not None else "  ---"
        r2 = f"{rpm2:+6.1f}" if rpm2 is not None else "  ---"
        r3 = f"{rpm3:+6.1f}" if rpm3 is not None else "  ---"
        print(f"  M1:{pulse}us  M2/M3:{spd:2d}%  |  RPM  M1:{r1}  M2:{r2}  M3:{r3}")
        time.sleep(0.2)
    run_all(1)

    # --- Ramp down ---
    print("\nRamp down:  M1 1500→1000us | M2/M3 75→0%")
    for i in range(steps + 1):
        frac = 1.0 - i / steps
        pulse = int(1000 + frac * 500)
        spd = int(frac * 75)
        motor1.set_pulse(pulse)
        motor2.set_speed(spd)
        motor3.set_speed(spd)
        motor1.poll_encoder()
        rpm1 = motor1.read_rpm()
        rpm2 = motor2.read_rpm()
        rpm3 = motor3.read_rpm()
        r1 = f"{rpm1:6.1f}" if rpm1 is not None else "  ---"
        r2 = f"{rpm2:+6.1f}" if rpm2 is not None else "  ---"
        r3 = f"{rpm3:+6.1f}" if rpm3 is not None else "  ---"
        print(f"  M1:{pulse}us  M2/M3:{spd:2d}%  |  RPM  M1:{r1}  M2:{r2}  M3:{r3}")
        time.sleep(0.2)

    print("\nTest complete.")
    stop_all()

except KeyboardInterrupt:
    print("\nStopped by user.")
    stop_all()

finally:
    pca.deinit()
