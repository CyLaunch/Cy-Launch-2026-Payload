# ======================================================================================
# Developer: Noah Wons
# Program: Hand-motion test script for simpleFlightDetection.py logic.
#          Runs the same state machine and detection logic as the real flight script
#          but with thresholds scaled for bench testing — no rocket required.
#          Follow the on-screen instructions to simulate launch and landing by hand.
# Contact: wons123@iastate.edu
# ======================================================================================
#
# TEST THRESHOLDS (vs real flight values):
#   LAUNCH_ACCEL_THRESHOLD : 15.0  m/s²  (real: 20.0)  — sharp hand jolt achieves this
#   LAUNCH_ALT_THRESHOLD   : removed     (real: 50m)    — not achievable indoors
#   LANDED_ALT_THRESHOLD   : 0.5   m     (real: 50m)    — just setting sensor on surface
#   LANDING_LOCKOUT        : 3.0   s     (real: 10s)    — shorter for faster testing
#   LANDED_ACCEL_STD_MAX   : 0.5   m/s²  (unchanged)   — same stillness requirement
#   LAUNCH_CONFIRM         : 5     reads  (unchanged)
#   LANDED_CONFIRM         : 10    reads  (unchanged)
# ======================================================================================

import time
import math
import board
import pwmio
import adafruit_mpl3115a2
import adafruit_icm20x
from adafruit_motor import servo

# ======================================================================================
# TEST-SCALED THRESHOLDS
# ======================================================================================
LAUNCH_ACCEL_THRESHOLD = 15.0  # m/s² — achievable with a sharp upward hand jolt
LANDED_ALT_THRESHOLD   = 0.5   # meters AGL — sensor resting on surface after calibration
LANDED_ACCEL_STD_MAX   = 0.5   # m/s² std dev — same stillness requirement as real flight
LANDING_LOCKOUT        = 3.0   # seconds post-launch before landing is evaluated
ACCEL_WINDOW           = 10    # rolling window size for accel std dev

LAUNCH_CONFIRM = 5             # consecutive high-accel readings to confirm launch
LANDED_CONFIRM = 10            # consecutive still + low-alt readings to confirm landing

LOOP_DT = 0.05                 # 20 Hz


# ======================================================================================
# Helpers
# ======================================================================================
def mag3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


# ======================================================================================
# Flight States
# ======================================================================================
class FlightState:
    IDLE     = "IDLE"
    LAUNCHED = "LAUNCHED"
    LANDED   = "LANDED"


# ======================================================================================
# Flight Detector (mirrors simpleFlightDetection.py, test thresholds applied above)
# ======================================================================================
class FlightDetector:
    def __init__(self):
        self.state        = FlightState.IDLE
        self.ground_alt   = None
        self.confirm_count = 0
        self.launch_time  = None
        self.accel_window = []

    def calibrate(self, altimeter, num_samples=50):
        print("  Collecting pressure baseline ({:.1f}s)...".format(num_samples * LOOP_DT))
        pres_readings = []
        for _ in range(num_samples):
            pres_readings.append(altimeter.pressure)
            time.sleep(LOOP_DT)

        local_pressure = sum(pres_readings) / len(pres_readings)
        altimeter.sealevel_pressure = local_pressure

        alt_readings = [altimeter.altitude for _ in range(10)]
        self.ground_alt = sum(alt_readings) / len(alt_readings)

        print(f"  Local pressure:  {local_pressure:.2f} Pa")
        print(f"  Ground altitude: {self.ground_alt:.2f} m (should be near 0)")

    def get_agl(self, altimeter):
        return altimeter.altitude - self.ground_alt

    def _accel_std(self):
        n = len(self.accel_window)
        mean = sum(self.accel_window) / n
        variance = sum((x - mean) ** 2 for x in self.accel_window) / n
        return math.sqrt(variance)

    def update(self, agl, accel_mag):
        prev_state = self.state

        self.accel_window.append(accel_mag)
        if len(self.accel_window) > ACCEL_WINDOW:
            self.accel_window.pop(0)

        if self.state == FlightState.IDLE:
            # Launch: IMU accel jolt only — no alt check (50m not achievable indoors)
            if accel_mag > LAUNCH_ACCEL_THRESHOLD:
                self.confirm_count += 1
                if self.confirm_count >= LAUNCH_CONFIRM:
                    self.state = FlightState.LAUNCHED
                    self.launch_time = time.monotonic()
                    self.confirm_count = 0
            else:
                self.confirm_count = 0

        elif self.state == FlightState.LAUNCHED:
            elapsed = time.monotonic() - self.launch_time
            if elapsed >= LANDING_LOCKOUT:
                window_full = len(self.accel_window) >= ACCEL_WINDOW
                still = window_full and self._accel_std() < LANDED_ACCEL_STD_MAX
                if agl < LANDED_ALT_THRESHOLD and still:
                    self.confirm_count += 1
                    if self.confirm_count >= LANDED_CONFIRM:
                        self.state = FlightState.LANDED
                        self.confirm_count = 0
                else:
                    self.confirm_count = 0

        if self.state != prev_state:
            print(f"\n  *** STATE CHANGE: {prev_state} -> {self.state} ***\n")

        return self.state


# ======================================================================================
# Hardware Setup
# ======================================================================================
print("=" * 60)
print("  CyLaunch Landing Detection — BENCH TEST MODE")
print("=" * 60)

i2c = board.I2C()

print("\nInitializing hardware...")

altimeter = None
try:
    altimeter = adafruit_mpl3115a2.MPL3115A2(i2c)
    altimeter.sealevel_pressure = 101325
    print("  OK  MPL3115A2 altimeter")
except Exception as e:
    print(f"  ERR MPL3115A2: {e}")

imu = None
for addr in (0x69, 0x68):
    try:
        imu = adafruit_icm20x.ICM20948(i2c, address=addr)
        print(f"  OK  ICM-20948 IMU at 0x{addr:02X}")
        break
    except Exception as e:
        print(f"  ERR ICM-20948 at 0x{addr:02X}: {e}")

SERVO_PIN = board.D5
pwm = pwmio.PWMOut(SERVO_PIN, duty_cycle=0, frequency=50)
my_servo = servo.Servo(pwm, min_pulse=500, max_pulse=2500)
print("  OK  Servo on D5")

if altimeter is None or imu is None:
    print("\nERROR: Required sensors missing. Cannot run test.")
    raise SystemExit


# ======================================================================================
# Run Test
# ======================================================================================
detector = FlightDetector()

print("\n" + "=" * 60)
print("  STEP 1 of 3 — CALIBRATION")
print("=" * 60)
print("  Place the sensor flat on a surface and hold it still.")
print("  Calibration will begin in 3 seconds...")
time.sleep(3)
detector.calibrate(altimeter)
print("  Calibration complete.")

print("\n" + "=" * 60)
print("  STEP 2 of 3 — SIMULATE LAUNCH")
print("=" * 60)
print("  Sharply jolt or shake the sensor upward, then hold it")
print(f"  raised at least 30cm above the surface.")
print(f"  Trigger: accel magnitude > {LAUNCH_ACCEL_THRESHOLD} m/s² for {LAUNCH_CONFIRM} consecutive readings.")
print()
print(f"  {'STATE':<12} {'AGL(m)':>7} {'ACCEL(m/s²)':>12} {'CONFIRM':>8}")
print("  " + "-" * 44)

last_instruction_state = FlightState.IDLE

while True:
    agl        = detector.get_agl(altimeter)
    ax, ay, az = imu.acceleration
    accel_mag  = mag3(ax, ay, az)
    state      = detector.update(agl, accel_mag)

    # Compute std dev for display if window is full
    accel_std = detector._accel_std() if len(detector.accel_window) >= ACCEL_WINDOW else float('nan')

    if state == FlightState.IDLE:
        print(f"  {state:<12} {agl:>7.2f} {accel_mag:>12.2f} {detector.confirm_count:>7}/{LAUNCH_CONFIRM}", end="\r")

    elif state == FlightState.LAUNCHED:
        elapsed  = time.monotonic() - detector.launch_time
        lockout_remaining = max(0.0, LANDING_LOCKOUT - elapsed)

        if last_instruction_state != FlightState.LAUNCHED:
            print()
            print("\n" + "=" * 60)
            print("  STEP 3 of 3 — SIMULATE LANDING")
            print("=" * 60)
            if lockout_remaining > 0:
                print(f"  Lockout active — landing evaluation begins in {lockout_remaining:.1f}s.")
            print("  Slowly lower the sensor back to the surface and hold it")
            print("  completely still.")
            print(f"  Trigger: AGL < {LANDED_ALT_THRESHOLD}m AND accel std dev < {LANDED_ACCEL_STD_MAX} m/s²")
            print(f"           for {LANDED_CONFIRM} consecutive readings.")
            print()
            print(f"  {'STATE':<12} {'AGL(m)':>7} {'ACCEL(m/s²)':>12} {'STD DEV':>9} {'LOCKOUT':>9} {'CONFIRM':>8}")
            print("  " + "-" * 62)

        lockout_str = f"{lockout_remaining:.1f}s" if lockout_remaining > 0 else "OPEN"
        print(f"  {state:<12} {agl:>7.2f} {accel_mag:>12.2f} {accel_std:>9.3f} {lockout_str:>9} {detector.confirm_count:>7}/{LANDED_CONFIRM}", end="\r")

    elif state == FlightState.LANDED:
        print()
        print("\n" + "=" * 60)
        print("  LANDING CONFIRMED")
        print("=" * 60)
        print("  In flight, the servo would now deploy to retract nosecone pins.")
        print("  Running servo sweep to confirm servo is functional...\n")
        for angle in range(0, 46, 2):
            my_servo.angle = angle
            time.sleep(0.05)
        time.sleep(0.5)
        for angle in range(45, -1, -2):
            my_servo.angle = angle
            time.sleep(0.05)
        print("  Servo sweep complete.")
        print("\n  Test finished successfully.")
        print("=" * 60)
        break

    last_instruction_state = state
    time.sleep(LOOP_DT)
