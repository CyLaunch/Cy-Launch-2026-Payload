# ======================================================================================
# Developer: Noah Wons
# Program: Landing detection state machine using ICM-20948 IMU and MPL3115A2 altimeter on FeatherWing M4 Express
#          Uses a Kalman filter to fuse accel and barometer data for robust altitude/velocity estimates
#          Detects launch, apogee, and landing events with confirmation logic to avoid false triggers.
#          Set TEST_MODE = True to run servo and sensor diagnostics instead of the flight state machine.
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
TEST_MODE = True

LOG_FILE = "flight_log.txt"  # Output file for flight events


# ======================================================================================
# Logging helper — writes timestamped events to LOG_FILE
# ======================================================================================
def log_event(event, altitude=None, velocity=None):
    timestamp = time.monotonic()
    if altitude is not None and velocity is not None:
        line = f"[{timestamp:.3f}s] {event} | Alt: {altitude:.2f}m AGL | Vel: {velocity:.3f}m/s\n"
    else:
        line = f"[{timestamp:.3f}s] {event}\n"
    print(line, end="")
    try:
        with open(LOG_FILE, "a") as f:
            f.write(line)
    except Exception as e:
        print(f"[LOG ERROR] Could not write to {LOG_FILE}: {e}")


# ======================================================================================
# State Machine
# ======================================================================================
class FlightState:
    IDLE = "IDLE"
    LAUNCHED = "LAUNCHED"
    ASCENDING = "ASCENDING"
    APOGEE = "APOGEE"
    DESCENDING = "DESCENDING"
    LANDED = "LANDED"

class FlightDetector:
    def __init__(self):
        self.state = FlightState.IDLE
        self.ground_alt = None

        self.LAUNCH_ALT_THRESHOLD = 50.0  # Ensure we don't trigger on false positive
        self.LANDED_ALT_THRESHOLD = 20.0

        # Kalman filter state
        self.kf_altitude = 0.0
        self.kf_velocity = 0.0
        self.kf_P = [[1, 0], [0, 1]]

        # Noise parameters (tune for your environment)
        self.Q = [[0.1, 0], [0, 0.1]]  # Process noise
        self.R = 2.0                    # Barometer noise (m^2)

        self.confirm_count = 0
        self.CONFIRM_THRESHOLD = 5
        self.dt = 0.05  # 20 Hz

        # Gravity estimate — calibrated on startup
        self.gravity_z = 9.81  # will be refined during calibrate()

    def calibrate(self, altimeter, imu, num_samples=50):
        """Record ground-level altitude baseline, local sea-level pressure, and measure gravity on this IMU's Z axis."""
        alt_readings = []
        gz_readings = []
        pres_readings = []

        for _ in range(num_samples):
            alt_readings.append(altimeter.altitude)
            pres_readings.append(altimeter.pressure)
            az = imu.acceleration[2]  # Z-axis accel while stationary
            gz_readings.append(az)
            time.sleep(0.02)

        # Derive local sea-level pressure from ground-level pressure reading.
        # This corrects for day-to-day weather and launch site elevation,
        # replacing the hardcoded standard atmosphere value.
        local_pressure = sum(pres_readings) / len(pres_readings)
        altimeter.sealevel_pressure = local_pressure

        # Re-sample altitude now that sealevel_pressure is corrected.
        # The previous alt_readings were computed with the old (standard) pressure,
        # so we take a quick fresh average to get an accurate ground_alt baseline.
        alt_readings = [altimeter.altitude for _ in range(10)]

        self.ground_alt = sum(alt_readings) / len(alt_readings)

        # Compute barometer measurement noise (R) from the variance of the re-sampled
        # altitude readings. The rocket is stationary here, so all variation is sensor
        # noise. This replaces the hardcoded R = 2.0 with a value measured on this
        # specific unit under current environmental conditions.
        mean_alt = self.ground_alt
        self.R = sum((a - mean_alt) ** 2 for a in alt_readings) / len(alt_readings)
        self.R = max(self.R, 0.01)  # Floor to avoid R=0 if sensor reads perfectly flat

        # The mean Z accel while stationary = gravity on that axis.
        # This handles the IMU being mounted upside-down or tilted slightly.
        self.gravity_z = sum(gz_readings) / len(gz_readings)

        print(f"Local pressure:    {local_pressure:.2f} Pa")
        print(f"Ground altitude:   {self.ground_alt:.2f} m")
        print(f"Barometer noise R: {self.R:.4f} m^2")
        print(f"Gravity on Z axis: {self.gravity_z:.3f} m/s^2")

    def get_vertical_accel(self, imu):
        """
        Extract the vertical (world Z) acceleration with gravity removed.

        For a rocket/UAV that flies roughly vertically, we can use the
        accelerometer magnitude trick: total_accel - gravity gives net
        acceleration, then project onto the vertical axis using the IMU Z reading.

        For simplicity here we assume Z is approximately vertical.
        Swap or negate axes below if your board is oriented differently.
        """
        ax, ay, az = imu.acceleration

        # Remove gravity component on Z axis (calibrated at startup)
        accel_z_no_gravity = az - self.gravity_z

        return accel_z_no_gravity

    def kalman_update(self, accel_z, agl):
        dt = self.dt

        # Predict
        self.kf_velocity += accel_z * dt
        self.kf_altitude += self.kf_velocity * dt

        self.kf_P[0][0] += dt * (dt * self.kf_P[1][1] - self.kf_P[0][1] - self.kf_P[1][0]) + self.Q[0][0]
        self.kf_P[0][1] -= dt * self.kf_P[1][1]
        self.kf_P[1][0] -= dt * self.kf_P[1][1]
        self.kf_P[1][1] += self.Q[1][1]

        # Update (barometer measurement)
        S = self.kf_P[0][0] + self.R
        K0 = self.kf_P[0][0] / S
        K1 = self.kf_P[1][0] / S

        innovation = agl - self.kf_altitude
        self.kf_altitude += K0 * innovation
        self.kf_velocity += K1 * innovation

        self.kf_P[0][0] *= (1 - K0)
        self.kf_P[1][0] *= (1 - K0)
        self.kf_P[0][1] -= K0 * self.kf_P[0][1]
        self.kf_P[1][1] -= K1 * self.kf_P[1][0]

        return self.kf_altitude, self.kf_velocity

    def update(self, imu, baro_alt):
        if self.ground_alt is None:
            return self.state

        agl = baro_alt - self.ground_alt
        accel_z = self.get_vertical_accel(imu)
        altitude, velocity = self.kalman_update(accel_z, agl)

        prev_state = self.state

        if self.state == FlightState.IDLE:
            if altitude > self.LAUNCH_ALT_THRESHOLD and velocity > 5.0:
                self.confirm_count += 1
                if self.confirm_count >= self.CONFIRM_THRESHOLD:
                    self.state = FlightState.LAUNCHED
                    self.confirm_count = 0
            else:
                self.confirm_count = 0

        elif self.state == FlightState.LAUNCHED:
            self.state = FlightState.ASCENDING

        elif self.state == FlightState.ASCENDING:
            if velocity < 0:
                self.confirm_count += 1
                if self.confirm_count >= self.CONFIRM_THRESHOLD:
                    self.state = FlightState.APOGEE
                    self.confirm_count = 0
            else:
                self.confirm_count = 0

        elif self.state == FlightState.APOGEE:
            self.state = FlightState.DESCENDING

        elif self.state == FlightState.DESCENDING:
            if altitude < self.LANDED_ALT_THRESHOLD and abs(velocity) < 0.5:
                self.confirm_count += 1
                if self.confirm_count >= self.CONFIRM_THRESHOLD:
                    self.state = FlightState.LANDED
                    self.confirm_count = 0
            else:
                self.confirm_count = 0

        if self.state != prev_state:
            log_event(f"STATE: {prev_state} → {self.state}", altitude, velocity)

        return self.state, altitude, velocity


# ======================================================================================
# Hardware Setup (shared by both modes)
# ======================================================================================
def mag3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)

i2c = board.I2C()

# --- Altimeter ---
print("Initializing MPL3115A2 altimeter...")
altimeter = None
try:
    altimeter = adafruit_mpl3115a2.MPL3115A2(i2c)
    print("  OK MPL3115A2 found")
except Exception as e:
    print(f"  ERROR MPL3115A2 not detected: {e}")

# --- IMU ---
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
    print("ERROR: ICM-20948 not detected. Check wiring.")

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
    detector.calibrate(altimeter, imu)
    log_event(f"CALIBRATION: ground_alt={detector.ground_alt:.2f}m gravity_z={detector.gravity_z:.3f}m/s^2")
    print("Ready. Waiting for launch...\n")

    apogee_logged = False

    while True:
        baro_alt = altimeter.altitude
        state, altitude, velocity = detector.update(imu, baro_alt)

        ax, ay, az = imu.acceleration
        print(f"State: {state:12s} | AGL: {altitude:6.1f}m | Vel: {velocity:6.2f}m/s "
              f"| Accel Z: {az - detector.gravity_z:5.2f}m/s^2")

        # Log apogee once when we first enter that state
        if state == FlightState.APOGEE and not apogee_logged:
            log_event("EVENT: APOGEE REACHED", altitude, velocity)
            apogee_logged = True

        if state == FlightState.LANDED:
            log_event("EVENT: LANDING CONFIRMED", altitude, velocity)
            print("\nLanding confirmed. Rotating servo to retract nosecone pins...")
            for angle in range(0, 45, 2):
                my_servo.angle = angle
                time.sleep(0.05)
            log_event("EVENT: SERVO DEPLOYMENT COMPLETE")
            break  # Stop looping after landing

        time.sleep(detector.dt)


# ======================================================================================
# Entry Point
# ======================================================================================
if TEST_MODE:
    run_test_mode()
else:
    run_flight_mode()
