# ======================================================================================
# Developer: Noah Wons
# Program: Flight detection for rover deployment using MPL3115A2 altimeter and
#          BNO055 absolute orientation IMU on FeatherWing M4 Express.
#          Launch is detected via BNO055 accelerometer threshold (motor ignition
#          signature), confirmed by altimeter reading above 50m AGL. Landing is
#          detected once the altimeter drops back below 50m AGL with a 10-second
#          post-launch lockout to prevent false triggers during boost.
#          Set TEST_MODE = True to run sensor diagnostics instead of flight mode.
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import math
import board
import countio
import digitalio
import adafruit_mpl3115a2
import adafruit_bno055
from adafruit_pca9685 import PCA9685

# ======================================================================================
# GLOBAL CONFIGURATION
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

print("Initializing PCA9685 PWM driver...")
pca = None
try:    
    pca = PCA9685(i2c)
    pca.frequency = 1000
    print("  OK PCA9685 found at 0x40")
except Exception as e:
    print(f"  ERROR PCA9685 not detected: {e}")


# --- BNO055 (absolute orientation IMU — accel + gyro used for flight detection) ---
print("Initializing BNO055 IMU...")
imu = None
for addr in (0x28, 0x29):
    try:
        imu = adafruit_bno055.BNO055_I2C(i2c, address=addr)
        print(f"  OK BNO055 found at 0x{addr:02X}")
        break
    except Exception as e:
        print(f"  Not at 0x{addr:02X}: {e}")

if imu is None:
    print("  ERROR: BNO055 not detected. Check wiring.")

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
    roll_offset = calibrate_level()   # capture level baseline before launch
    log_event(f"CALIBRATION: ground_alt={detector.ground_alt:.2f}m  roll_offset={roll_offset:+.2f}deg")
    print("Ready. Waiting for launch...\n")

    while True:
        agl            = detector.get_agl(altimeter)
        ax, ay, az     = imu.acceleration
        accel_mag      = mag3(ax, ay, az)
        state          = detector.update(agl, accel_mag)

        accel_std = detector._accel_std() if len(detector.accel_window) >= ACCEL_WINDOW else float('nan')
        print(f"State: {state:10s} | AGL: {agl:6.1f}m | Accel: {accel_mag:5.1f}m/s² | StdDev: {accel_std:.3f}m/s²")

        if state == FlightState.LANDED:
            log_event("EVENT: LANDING CONFIRMED", agl)
            print("\nLanding confirmed.")

            # TODO: add rover post-landing logic here
            # e.g. deploy rover, start motors, begin navigation

            set_device_to_level(roll_offset)

            break

        time.sleep(LOOP_DT)

# ======================================================================================
# Orientation Motor Setup
# ======================================================================================
pca = PCA9685(i2c)
pca.frequency = 1000  # 1kHz PWM for motor drivers

# --- Orientation motor is on CH2 (forward) and CH3 (reverse) ---
MOTOR_PWM1 = 2
MOTOR_PWM2 = 3

# --- Encoder (D9 = A, D10 = B) ---
enc_a = countio.Counter(board.D9, edge=countio.Edge.RISE)
enc_b = digitalio.DigitalInOut(board.D10)
enc_b.direction = digitalio.Direction.INPUT

# --- Tuning Parameters ---
LEVEL_THRESHOLD = 2.0    # degrees — if tilt is within this range, do nothing (dead zone)
MAX_SPEED = 60           # max motor speed % (keep below 100 to avoid violent corrections)
MIN_SPEED = 15           # min speed % to overcome motor stiction
KP = 1.5                 # proportional gain — increase if corrections are too slow,
                         # decrease if motor overshoots/oscillates

# --- Motor Control ---
def set_motor(speed):
    """
    speed: -100 to 100
    positive = forward, negative = reverse, 0 = stop
    """
    speed = max(-100, min(100, speed))  # clamp to safe range
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

def stop_motor():
    set_motor(0)

# --- Calibration ---
CALIBRATION_DURATION = 3.0   # seconds to collect level baseline
CALIBRATION_DT       = 0.05  # 20 Hz sample rate during calibration

def calibrate_level():
    """
    Collect roll readings for CALIBRATION_DURATION seconds with the rover
    sitting level on the ground. Returns the average roll as the zero offset.
    """
    print("=" * 45)
    print("  CALIBRATION")
    print("  Place the rover on level ground and")
    print("  keep it still.")
    print("  Starting in 3 seconds...")
    print("=" * 45)
    time.sleep(3)

    samples = []
    end_time = time.monotonic() + CALIBRATION_DURATION
    print(f"Collecting {CALIBRATION_DURATION:.0f}s of data...", end="")

    while time.monotonic() < end_time:
        euler = imu.euler
        if euler is not None and euler[1] is not None:
            samples.append(euler[1])  # roll
        time.sleep(CALIBRATION_DT)

    if not samples:
        print(" FAILED (no IMU data). Defaulting offset to 0.0")
        return 0.0

    offset = sum(samples) / len(samples)
    print(f" done ({len(samples)} samples)")
    print(f"  Roll offset: {offset:+.2f} deg")
    print()
    return offset

def set_device_to_level(roll_offset):
    """
    Simple proportional controller to drive the motor until the device is level.
    roll_offset is captured before launch via calibrate_level() so the rover
    targets the same orientation it had on the ground, not absolute 0 degrees.
    """

    print("Starting leveling loop. Press CTRL+C to stop.")
    print(f"Dead zone: +/- {LEVEL_THRESHOLD} degrees")
    print(f"Max speed: {MAX_SPEED}%")
    print()

    while True:
        # Read Euler angles from BNO055
        # euler returns (heading, roll, pitch) in degrees
        euler = imu.euler

        if euler is None or euler[1] is None:
            print("No IMU data — check wiring!")
            stop_motor()
            time.sleep(0.5)
            continue

        heading, roll, pitch = euler

        # Subtract calibrated offset so tilt = 0 when rover is at its ground-level position
        # Swap to 'pitch' if your motor corrects front-to-back tilt instead
        tilt = roll - roll_offset

        # --- Proportional Control ---
        if abs(tilt) <= LEVEL_THRESHOLD:
            # Within dead zone — hold still
            stop_motor()
            status = "LEVEL"
            speed_out = 0
            break
        else:
            # Calculate correction speed proportional to tilt angle
            raw_speed = KP * tilt
            # Enforce minimum speed so motor actually moves
            if raw_speed > 0:
                speed_out = max(MIN_SPEED, min(MAX_SPEED, raw_speed))
            else:
                speed_out = min(-MIN_SPEED, max(-MAX_SPEED, raw_speed))

            set_motor(speed_out)
            status = "CORRECTING"

        direction = 1 if enc_b.value else -1
        print(f"Tilt: {tilt:+.1f} deg | Speed: {speed_out:+.0f}% | Enc: {enc_a.count * direction} | Status: {status}")
        time.sleep(0.05)  # 20Hz control loop


# ======================================================================================
# Entry Point
# ======================================================================================
run_flight_mode()
