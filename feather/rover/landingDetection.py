import time
import board
import adafruit_mpl3115a2
import adafruit_icm20x
import adafruit_bno055
from adafruit_pca9685 import PCA9685

# ==============================================================================
# CONFIGURATION
# ==============================================================================

# PCA9685 channel assignments
MOTOR_ORIENTATION = 0   # Rotates payload to level
MOTOR_DRIVE_LEFT  = 1   # Drive wheel (TBD)
MOTOR_DRIVE_RIGHT = 2   # Drive wheel (TBD)

# ESC PWM range (standard hobby, 50Hz)
# PCA9685 runs at 12-bit resolution (0–4095) at 50Hz
# 1000µs = 204 counts, 2000µs = 409 counts at 50Hz / 4096 steps
PWM_FREQ        = 50
PWM_MIN         = 204   # 1000µs — ESC minimum / stop
PWM_MAX         = 409   # 2000µs — ESC maximum
PWM_MID         = 307   # 1500µs — neutral/brake for orientation motor
PWM_STOP        = PWM_MIN

# Orientation PID tuning (tune these on the bench)
PID_KP          = 4.0
PID_KI          = 0.05
PID_KD          = 0.8

# Leveling config
LEVEL_TOLERANCE_DEG  = 2.0    # Degrees from level considered "done"
LEVEL_CONFIRM_COUNT  = 10     # Consecutive samples within tolerance to confirm level
LEVEL_TIMEOUT_SEC    = 15.0   # Give up after this many seconds

# BNO055 euler index for Y-axis-up orientation.
# bno.euler returns (heading, roll, pitch).
# With Y vertical, we want to drive ROLL (index 1) and PITCH (index 2) to 0.
# In practice for a single rotation axis motor, we only correct one — choose
# whichever axis your motor physically controls. Default: ROLL (index 1).
CORRECT_EULER_INDEX = 1       # 0=heading, 1=roll, 2=pitch

# ==============================================================================
# FLIGHT STATE MACHINE (same as nosecone, reused here)
# ==============================================================================

class FlightState:
    IDLE        = "IDLE"
    LAUNCHED    = "LAUNCHED"
    ASCENDING   = "ASCENDING"
    APOGEE      = "APOGEE"
    DESCENDING  = "DESCENDING"
    LANDED      = "LANDED"

class FlightDetector:
    def __init__(self):
        self.state          = FlightState.IDLE
        self.ground_alt     = None
        self.gravity_z      = 9.81

        self.kf_altitude    = 0.0
        self.kf_velocity    = 0.0
        self.kf_P           = [[1, 0], [0, 1]]
        self.Q              = [[0.1, 0], [0, 0.1]]
        self.R              = 2.0

        self.confirm_count      = 0
        self.CONFIRM_THRESHOLD  = 5
        self.dt                 = 0.05

    def calibrate(self, altimeter, imu, num_samples=50):
        alt_readings = []
        gz_readings  = []
        for _ in range(num_samples):
            alt_readings.append(altimeter.altitude)
            gz_readings.append(imu.acceleration[2])
            time.sleep(0.02)
        self.ground_alt = sum(alt_readings) / len(alt_readings)
        self.gravity_z  = sum(gz_readings)  / len(gz_readings)
        print(f"[CAL] Ground alt: {self.ground_alt:.2f}m | Gravity Z: {self.gravity_z:.3f}m/s²")

    def _kalman_update(self, accel_z, agl):
        dt = self.dt
        self.kf_velocity += accel_z * dt
        self.kf_altitude += self.kf_velocity * dt

        self.kf_P[0][0] += dt * (dt * self.kf_P[1][1] - self.kf_P[0][1] - self.kf_P[1][0]) + self.Q[0][0]
        self.kf_P[0][1] -= dt * self.kf_P[1][1]
        self.kf_P[1][0] -= dt * self.kf_P[1][1]
        self.kf_P[1][1] += self.Q[1][1]

        S  = self.kf_P[0][0] + self.R
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
            return self.state, 0.0, 0.0

        agl      = baro_alt - self.ground_alt
        accel_z  = imu.acceleration[2] - self.gravity_z
        altitude, velocity = self._kalman_update(accel_z, agl)
        prev     = self.state

        if self.state == FlightState.IDLE:
            if altitude > 5.0 and velocity > 5.0:
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
            if altitude < 10.0 and abs(velocity) < 0.5:
                self.confirm_count += 1
                if self.confirm_count >= self.CONFIRM_THRESHOLD:
                    self.state = FlightState.LANDED
                    self.confirm_count = 0
            else:
                self.confirm_count = 0

        if self.state != prev:
            print(f"[{time.monotonic():.2f}s] {prev} → {self.state} "
                  f"| Alt: {altitude:.1f}m | Vel: {velocity:.2f}m/s")

        return self.state, altitude, velocity


# ==============================================================================
# PCA9685 MOTOR HELPERS
# ==============================================================================

def set_motor_pwm(pca, channel, pwm_counts):
    """Set a PCA9685 channel to a raw 12-bit count value."""
    pwm_counts = max(PWM_MIN, min(PWM_MAX, int(pwm_counts)))
    pca.channels[channel].duty_cycle = int(pwm_counts / 4096 * 65535)

def stop_motor(pca, channel):
    """Send ESC stop/disarm signal."""
    set_motor_pwm(pca, channel, PWM_STOP)

def arm_escs(pca):
    """
    Standard ESC arming sequence:
    Send minimum throttle for 2 seconds so ESCs initialise.
    Do this before any flight logic runs.
    """
    print("[ESC] Arming — sending minimum throttle for 2s...")
    for ch in [MOTOR_ORIENTATION, MOTOR_DRIVE_LEFT, MOTOR_DRIVE_RIGHT]:
        set_motor_pwm(pca, ch, PWM_MIN)
    time.sleep(2.0)
    print("[ESC] Armed.")


# ==============================================================================
# ORIENTATION CONTROLLER
# ==============================================================================

class OrientationController:
    """
    PID controller that drives the orientation motor to level the payload.

    The BNO055 euler tuple is (heading, roll, pitch).
    With Y vertical, we correct the axis set by CORRECT_EULER_INDEX.
    Target is 0 degrees (level).

    Motor output:
      - Error > 0  → spin one direction  (PWM > MID)
      - Error < 0  → spin other direction (PWM < MID)
      - Error ≈ 0  → stop               (PWM = MIN)
    """
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self._integral   = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0

    def compute(self, current_angle_deg):
        """Returns a PWM count to send to the orientation motor."""
        target = 0.0
        error  = target - current_angle_deg

        self._integral   += error * self.dt
        derivative        = (error - self._prev_error) / self.dt
        self._prev_error  = error

        # Clamp integral to prevent windup
        self._integral = max(-50.0, min(50.0, self._integral))

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # Map PID output to PWM range around midpoint (1500µs)
        # Positive output → above mid, negative → below mid
        pwm = PWM_MID + output
        pwm = max(PWM_MIN, min(PWM_MAX, pwm))
        return pwm

    def is_level(self, current_angle_deg):
        return abs(current_angle_deg) <= LEVEL_TOLERANCE_DEG


def wait_for_bno_calibration(bno):
    """Block until the BNO055 reports at least partial calibration."""
    print("[BNO] Waiting for calibration (move sensor figure-8 if needed)...")
    while True:
        sys, gyro, accel, mag = bno.calibration_status
        print(f"[BNO] Cal status — sys:{sys} gyro:{gyro} accel:{accel} mag:{mag}")
        if sys >= 1 and gyro >= 1:
            print("[BNO] Calibration sufficient.")
            break
        time.sleep(0.5)


def run_orientation_leveling(pca, bno, pid):
    """
    After landing: read BNO055 orientation and drive the orientation motor
    until the payload is level, then stop. Times out after LEVEL_TIMEOUT_SEC.
    """
    print("\n[ORIENT] Starting orientation leveling sequence...")
    pid.reset()

    confirm_count = 0
    start_time    = time.monotonic()

    while True:
        elapsed = time.monotonic() - start_time
        if elapsed > LEVEL_TIMEOUT_SEC:
            print(f"[ORIENT] Timeout after {LEVEL_TIMEOUT_SEC}s — stopping motor.")
            stop_motor(pca, MOTOR_ORIENTATION)
            return False

        euler = bno.euler
        if euler is None or euler[CORRECT_EULER_INDEX] is None:
            time.sleep(0.05)
            continue

        current_angle = euler[CORRECT_EULER_INDEX]
        pwm           = pid.compute(current_angle)

        print(f"[ORIENT] Angle: {current_angle:6.1f}° | PWM: {int(pwm)} | "
              f"Elapsed: {elapsed:.1f}s")

        if pid.is_level(current_angle):
            confirm_count += 1
            stop_motor(pca, MOTOR_ORIENTATION)  # Stop while confirming
            if confirm_count >= LEVEL_CONFIRM_COUNT:
                print(f"[ORIENT] Level confirmed at {current_angle:.1f}°. Motor stopped.")
                return True
        else:
            confirm_count = 0
            set_motor_pwm(pca, MOTOR_ORIENTATION, pwm)

        time.sleep(pid.dt)


# ==============================================================================
# DRIVE MOTOR STUBS (TBD)
# ==============================================================================

def drive_forward(pca, speed=0.3):
    """Placeholder — drive both wheels forward. Speed: 0.0–1.0"""
    throttle = int(PWM_MIN + speed * (PWM_MAX - PWM_MIN))
    set_motor_pwm(pca, MOTOR_DRIVE_LEFT,  throttle)
    set_motor_pwm(pca, MOTOR_DRIVE_RIGHT, throttle)

def drive_stop(pca):
    """Placeholder — stop both drive wheels."""
    stop_motor(pca, MOTOR_DRIVE_LEFT)
    stop_motor(pca, MOTOR_DRIVE_RIGHT)


# ==============================================================================
# MAIN
# ==============================================================================

i2c = board.I2C()

# Sensors
altimeter = adafruit_mpl3115a2.MPL3115A2(i2c)
altimeter.sealevel_pressure = 101325  # Update to local QNH in Pascals

flight_imu = adafruit_icm20x.ICM20948(i2c)   # ICM-20948: flight detection
orient_imu = adafruit_bno055.BNO055_I2C(i2c)  # BNO055: orientation

# Motor controller
pca = PCA9685(i2c)
pca.frequency = PWM_FREQ

# Controllers
detector = FlightDetector()
pid      = OrientationController(kp=PID_KP, ki=PID_KI, kd=PID_KD, dt=0.05)

# --- Startup sequence ---
arm_escs(pca)
wait_for_bno_calibration(orient_imu)

print("[CAL] Calibrating flight sensors — keep vehicle still...")
detector.calibrate(altimeter, flight_imu)
print("[SYS] Ready. Waiting for launch...\n")

# --- Main loop ---
while True:
    baro_alt              = altimeter.altitude
    state, altitude, velocity = detector.update(flight_imu, baro_alt)

    if state == FlightState.LANDED:
        drive_stop(pca)
        leveled = run_orientation_leveling(pca, orient_imu, pid)
        if leveled:
            print("[SYS] Payload leveled. Rover ready for next phase.")
        else:
            print("[SYS] Leveling failed or timed out. Manual check needed.")
        # --- Post-landing TBD ---
        # drive_forward(pca, speed=0.5)
        # ...
        break

    time.sleep(detector.dt)