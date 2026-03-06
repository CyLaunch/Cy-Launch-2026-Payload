import time
import math
import board
import adafruit_mpl3115a2
import adafruit_icm20x
from adafruit_motor import servo


# --- State Machine ---
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
        """Record ground-level altitude baseline and measure gravity on this IMU's Z axis."""
        alt_readings = []
        gz_readings = []

        for _ in range(num_samples):
            alt_readings.append(altimeter.altitude)
            az = imu.acceleration[2]  # Z-axis accel while stationary
            gz_readings.append(az)
            time.sleep(0.02)

        self.ground_alt = sum(alt_readings) / len(alt_readings)

        # The mean Z accel while stationary = gravity on that axis.
        # This handles the IMU being mounted upside-down or tilted slightly.
        self.gravity_z = sum(gz_readings) / len(gz_readings)

        print(f"Ground altitude: {self.ground_alt:.2f} m")
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

        if self.state != prev_state:
            print(f"[{time.monotonic():.2f}s] {prev_state} → {self.state} "
                  f"| Alt: {altitude:.1f}m AGL | Vel: {velocity:.2f}m/s")

        return self.state, altitude, velocity


# --- Setup ---
i2c = board.I2C()

altimeter = adafruit_mpl3115a2.MPL3115A2(i2c)
altimeter.sealevel_pressure = 101325  # Set to your local pressure in Pascals

# ICM-20948: 9-DOF (accel + gyro + magnetometer)
imu = adafruit_icm20x.ICM20948(i2c)

SERVO_PIN = board.D5
pwm = pwmio.PWMOut(SERVO_PIN, duty_cycle=0, frequency=50)
my_servo = servo.Servo(pwm, min_pulse=500, max_pulse=2500)

detector = FlightDetector()

print("Calibrating... keep the vehicle still.")
detector.calibrate(altimeter, imu)
print("Ready. Waiting for launch...\n")

# --- Main Loop ---
while True:
    baro_alt = altimeter.altitude
    state, altitude, velocity = detector.update(imu, baro_alt)

    # Optional: print telemetry every loop for debugging
    ax, ay, az = imu.acceleration
    print(f"State: {state:12s} | AGL: {altitude:6.1f}m | Vel: {velocity:6.2f}m/s "
          f"| Accel Z: {az - detector.gravity_z:5.2f}m/s²")

    if state == FlightState.LANDED:
        print("\nLanding confirmed. Rotating servo to retract nosecone pins...")
        my_servo.angle = 0  # Adjust the angle as needed
        for angle in range(0, 45, 2):
            my_servo.angle = angle
            time.sleep(0.05)

    time.sleep(detector.dt)