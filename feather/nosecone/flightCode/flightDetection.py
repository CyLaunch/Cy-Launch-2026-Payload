# ======================================================================================
# Developer: Noah Wons
# Program: Simplified landing detection using MPL3115A2 altimeter and ICM-20948 IMU
#          on FeatherWing M4 Express.
#          Launch is detected via IMU acceleration threshold (motor ignition signature),
#          confirmed by altimeter reading above 50m AGL. Landing is detected once the
#          altimeter drops back below 50m AGL, with a 10-second post-launch lockout to
#          prevent false triggers during boost. At ~4800ft apogee this approach is
#          reliable without requiring a Kalman filter.
#          Set TEST_MODE = True to run servo and sensor diagnostics instead of flight mode.
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import math
import board
import pwmio
import adafruit_mpl3115a2
import adafruit_icm20x
from adafruit_motor import servo

# ======================================================================================
# GLOBAL CONFIGURATION
# Set TEST_MODE = True  → runs servo + sensor diagnostics
# Set TEST_MODE = False → runs the full flight state machine
# ======================================================================================

LOG_FILE = "flight_log.txt"

LAUNCH_ACCEL_THRESHOLD  = 20.0  # m/s² total — above this indicates motor ignition (~2G)
LAUNCH_ALT_THRESHOLD    = 50.0  # meters AGL — altimeter must confirm we are airborne
LANDED_ALT_THRESHOLD    = 50.0  # meters AGL — below this (after lockout) = landed
LANDED_ACCEL_STD_MAX    = 0.5   # m/s² — std dev of accel magnitude must be below this to confirm stillness
LANDING_LOCKOUT         = 10.0  # seconds after launch before landing is evaluated
ACCEL_WINDOW            = 10    # rolling window size for accel std dev calculation

LAUNCH_CONFIRM = 5    # consecutive readings above accel threshold (+ alt check) to confirm launch
LANDED_CONFIRM = 10   # consecutive readings satisfying both alt and stillness checks to confirm landing

LOOP_DT = 0.05        # 20 Hz


# ======================================================================================
# Logging helper
# ======================================================================================
def log_event(event, altitude=None):
    timestamp = time.monotonic()
    if altitude is not None:
        line = f"[{timestamp:.3f}s] {event} | Alt: {altitude:.2f}m AGL\n"
    else:
        line = f"[{timestamp:.3f}s] {event}\n"
    print(line, end="")
    try:
        with open(LOG_FILE, "a") as f:
            f.write(line)
    except Exception as e:
        print(f"[LOG ERROR] Could not write to {LOG_FILE}: {e}")


# ======================================================================================
# Flight States
# ======================================================================================
class FlightState:
    IDLE     = "IDLE"
    LAUNCHED = "LAUNCHED"
    LANDED   = "LANDED"


# ======================================================================================
# Flight Detector
# ======================================================================================
class FlightDetector:
    def __init__(self):
        self.state         = FlightState.IDLE
        self.ground_alt    = None
        self.confirm_count = 0
        self.launch_time   = None   # monotonic timestamp set at launch confirmation
        self.accel_window  = []     # rolling window of accel_mag readings for std dev

    def calibrate(self, altimeter, num_samples=50):
        """
        Measure local sea-level pressure and ground altitude baseline.
        Keep the vehicle still during calibration.
        """
        pres_readings = []

        print("Calibrating... collecting pressure baseline.")
        for _ in range(num_samples):
            pres_readings.append(altimeter.pressure)
            time.sleep(LOOP_DT)

        # Set sealevel_pressure to the measured local value so the altimeter
        # reports ~0m AGL at the launch site, correcting for weather and elevation.
        local_pressure = sum(pres_readings) / len(pres_readings)
        altimeter.sealevel_pressure = local_pressure

        # Re-sample altitude with the corrected pressure reference.
        alt_readings = [altimeter.altitude for _ in range(10)]
        self.ground_alt = sum(alt_readings) / len(alt_readings)

        print(f"Local pressure:  {local_pressure:.2f} Pa")
        print(f"Ground altitude: {self.ground_alt:.2f} m (should be near 0)")

    def get_agl(self, altimeter):
        return altimeter.altitude - self.ground_alt

    def _accel_std(self):
        """Standard deviation of the rolling accel magnitude window."""
        n = len(self.accel_window)
        mean = sum(self.accel_window) / n
        variance = sum((x - mean) ** 2 for x in self.accel_window) / n
        return math.sqrt(variance)

    def update(self, agl, accel_mag):
        prev_state = self.state

        # Maintain rolling accel window regardless of state.
        self.accel_window.append(accel_mag)
        if len(self.accel_window) > ACCEL_WINDOW:
            self.accel_window.pop(0)

        if self.state == FlightState.IDLE:
            # Require sustained high acceleration AND altimeter confirmation above 50m.
            # High accel catches ignition fast; alt check prevents false triggers from
            # pad handling or accidental bumps.
            if accel_mag > LAUNCH_ACCEL_THRESHOLD and agl > LAUNCH_ALT_THRESHOLD:
                self.confirm_count += 1
                if self.confirm_count >= LAUNCH_CONFIRM:
                    self.state = FlightState.LAUNCHED
                    self.launch_time = time.monotonic()
                    self.confirm_count = 0
            else:
                self.confirm_count = 0

        elif self.state == FlightState.LAUNCHED:
            # Do not evaluate landing until LANDING_LOCKOUT seconds have elapsed.
            # This prevents any transient low-altitude readings during boost from
            # triggering a false landing event.
            elapsed = time.monotonic() - self.launch_time
            if elapsed >= LANDING_LOCKOUT:
                # Require the window to be full before computing std dev, so we
                # always judge stillness on a complete set of readings.
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
            log_event(f"STATE: {prev_state} -> {self.state}", agl)

        return self.state


# ======================================================================================
# Hardware Setup
# ======================================================================================
def mag3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)

i2c = board.I2C()

# --- Altimeter ---
print("Initializing MPL3115A2 altimeter...")
altimeter = None
try:
    altimeter = adafruit_mpl3115a2.MPL3115A2(i2c)
    altimeter.sealevel_pressure = 101325  # Temporary default; overwritten during calibrate()
    print("  OK MPL3115A2 found")
except Exception as e:
    print(f"  ERROR MPL3115A2 not detected: {e}")

# --- IMU (required for launch detection) ---
print("Initializing ICM-20948 IMU...")
imu = None
for addr in (0x69, 0x68):
    try:
        imu = adafruit_icm20x.ICM20948(i2c, address=addr)
        print(f"  OK ICM-20948 found at 0x{addr:02X}")
        break
    except Exception as e:
        print(f"  ERROR Not at 0x{addr:02X}: {e}")

if imu is None:
    print("  ERROR: ICM-20948 not detected. Check wiring.")

# --- Servo ---
SERVO_PIN = board.D5
pwm = pwmio.PWMOut(SERVO_PIN, duty_cycle=0, frequency=50)
my_servo = servo.Servo(pwm, min_pulse=500, max_pulse=2500)


# ======================================================================================
# TEST MODE
# ======================================================================================
def run_test_mode():
    print("\n=== TEST MODE ===")
    print("Running servo and sensor diagnostics.\n")

    # --- Servo test ---
    print("--- Servo Test ---")
    print("Sweeping servo 0 → 45 degrees...")
    for angle in range(0, 46, 2):
        my_servo.angle = angle
        time.sleep(0.05)
    time.sleep(0.5)
    print("Sweeping servo 45 → 0 degrees...")
    for angle in range(45, -1, -2):
        my_servo.angle = angle
        time.sleep(0.05)
    print("Servo test complete.\n")

    # --- Sensor test ---
    print("--- Sensor Test (10 Hz for 5 seconds) ---")
    DT = 0.1
    print(
        f"{'ALT(m)':>8} {'P(hPa)':>8} {'T(C)':>6} | "
        f"{'Ax':>6} {'Ay':>6} {'Az':>6} | "
        f"{'Gx':>6} {'Gy':>6} {'Gz':>6} | "
        f"{'|a|':>6} {'|g|':>6}"
    )
    print("=" * 80)

    end_time = time.monotonic() + 5.0
    while time.monotonic() < end_time:
        if altimeter is not None:
            try:
                alt  = altimeter.altitude
                pres = altimeter.pressure / 100  # Pa -> hPa
                temp = altimeter.temperature
            except Exception as e:
                alt, pres, temp = float('nan'), float('nan'), float('nan')
                print(f"Altimeter read error: {e}")
        else:
            alt, pres, temp = float('nan'), float('nan'), float('nan')

        if imu is not None:
            try:
                ax, ay, az = imu.acceleration
                gx, gy, gz = imu.gyro
                a_mag = mag3(ax, ay, az)
                g_mag = mag3(gx, gy, gz)
            except Exception as e:
                ax = ay = az = gx = gy = gz = a_mag = g_mag = float('nan')
                print(f"IMU read error: {e}")
        else:
            ax = ay = az = gx = gy = gz = a_mag = g_mag = float('nan')

        print(
            f"{alt:8.2f} {pres:8.2f} {temp:6.2f} | "
            f"{ax:6.2f} {ay:6.2f} {az:6.2f} | "
            f"{gx:6.3f} {gy:6.3f} {gz:6.3f} | "
            f"{a_mag:6.2f} {g_mag:6.3f}"
        )
        time.sleep(DT)

    print("\nSensor test complete.")
    print("=== TEST MODE DONE ===\n")


# ======================================================================================
# FLIGHT MODE
# ======================================================================================
def run_flight_mode():
    if altimeter is None or imu is None:
        print("ERROR: Required sensors missing. Cannot run flight mode.")
        return

    detector = FlightDetector()

    log_event("BOOT: Flight computer started")
    print("Calibrating... keep the vehicle still.")
    detector.calibrate(altimeter)
    log_event(f"CALIBRATION: ground_alt={detector.ground_alt:.2f}m")
    print("Ready. Waiting for launch...\n")

    while True:
        agl       = detector.get_agl(altimeter)
        ax, ay, az = imu.acceleration
        accel_mag = mag3(ax, ay, az)
        state     = detector.update(agl, accel_mag)

        accel_std = detector._accel_std() if len(detector.accel_window) >= ACCEL_WINDOW else float('nan')
        print(f"State: {state:10s} | AGL: {agl:6.1f}m | Accel: {accel_mag:5.1f}m/s² | StdDev: {accel_std:.3f}m/s²")

        if state == FlightState.LANDED:
            log_event("EVENT: LANDING CONFIRMED", agl)
            print("\nLanding confirmed. Rotating servo to retract nosecone pins...")
            for angle in range(0, 46, 2):
                my_servo.angle = angle
                time.sleep(0.05)
            log_event("EVENT: SERVO DEPLOYMENT COMPLETE")
            break

        time.sleep(LOOP_DT)


# ======================================================================================
# Entry Point
# ======================================================================================
run_flight_mode()
