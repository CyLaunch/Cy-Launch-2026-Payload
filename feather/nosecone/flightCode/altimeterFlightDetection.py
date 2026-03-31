# ======================================================================================
# Developer: Noah Wons
# Program: Altimeter-only flight detection for nosecone system using MPL3115A2.
#          Launch is detected when altitude exceeds 50m AGL.
#          Landing is detected when altitude drops back below 10m AGL and remains
#          there for 30 seconds. Servo holds at 0 degrees throughout flight and
#          rotates to 45 degrees upon landing confirmation.
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import math
import board
import pwmio
import adafruit_mpl3115a2
from adafruit_motor import servo

# ======================================================================================
# CONFIGURATION
# ======================================================================================

LOG_FILE  = "flight_log.txt"
DATA_FILE = "flight_data.txt"

LAUNCH_ALT_THRESHOLD  = 50.0   # meters AGL — above this confirms launch
LANDED_ALT_THRESHOLD  = 10.0   # meters AGL — below this starts landing timer
LANDING_HOLD_TIME     = 30.0   # seconds below LANDED_ALT_THRESHOLD to confirm landing
LANDING_LOCKOUT       = 10.0   # seconds after launch before landing is evaluated

LAUNCH_CONFIRM = 5             # consecutive readings above threshold to confirm launch

LOOP_DT = 0.05                 # 20 Hz


# ======================================================================================
# Logging
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

def log_flight_data(line):
    timestamp = time.monotonic()
    entry = f"[{timestamp:.3f}s] {line}\n"
    try:
        with open(DATA_FILE, "a") as f:
            f.write(entry)
    except Exception as e:
        print(f"[LOG ERROR] Could not write to {DATA_FILE}: {e}")


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
        self.launch_time   = None
        self.below_time    = None   # monotonic time when alt first dropped below threshold

    def calibrate(self, altimeter, num_samples=50):
        """Average pressure over num_samples to zero out local elevation and weather."""
        pres_readings = []
        print("Calibrating altimeter... keep the vehicle still.")
        for _ in range(num_samples):
            pres_readings.append(altimeter.pressure)
            time.sleep(LOOP_DT)

        local_pressure = sum(pres_readings) / len(pres_readings)
        altimeter.sealevel_pressure = local_pressure

        alt_readings = [altimeter.altitude for _ in range(10)]
        self.ground_alt = sum(alt_readings) / len(alt_readings)

        print(f"Local pressure:  {local_pressure:.2f} Pa")
        print(f"Ground altitude: {self.ground_alt:.2f} m (should be near 0)")

    def get_agl(self, altimeter):
        return altimeter.altitude - self.ground_alt

    def update(self, agl):
        prev_state = self.state
        now = time.monotonic()

        if self.state == FlightState.IDLE:
            if agl > LAUNCH_ALT_THRESHOLD:
                self.confirm_count += 1
                if self.confirm_count >= LAUNCH_CONFIRM:
                    self.state = FlightState.LAUNCHED
                    self.launch_time = now
                    self.confirm_count = 0
                    self.below_time = None
            else:
                self.confirm_count = 0

        elif self.state == FlightState.LAUNCHED:
            elapsed = now - self.launch_time
            if elapsed >= LANDING_LOCKOUT:
                if agl < LANDED_ALT_THRESHOLD:
                    if self.below_time is None:
                        self.below_time = now  # start the landing timer
                    elif (now - self.below_time) >= LANDING_HOLD_TIME:
                        self.state = FlightState.LANDED
                else:
                    self.below_time = None  # reset timer if altitude spikes back up

        if self.state != prev_state:
            log_event(f"STATE: {prev_state} -> {self.state}", agl)

        return self.state


# ======================================================================================
# Hardware Setup
# ======================================================================================
i2c = board.I2C()

# --- Altimeter ---
print("Initializing MPL3115A2 altimeter...")
altimeter = None
try:
    altimeter = adafruit_mpl3115a2.MPL3115A2(i2c)
    altimeter.sealevel_pressure = 101325  # temporary default; overwritten during calibrate()
    print("  OK MPL3115A2 found")
except Exception as e:
    print(f"  ERROR MPL3115A2 not detected: {e}")

if altimeter is None:
    print("ERROR: Altimeter required. Halting.")
    while True:
        time.sleep(1)

# --- Servo ---
SERVO_PIN = board.D5
pwm = pwmio.PWMOut(SERVO_PIN, duty_cycle=0, frequency=50)
my_servo = servo.Servo(pwm, min_pulse=500, max_pulse=2500)
my_servo.angle = 45  # hold at rest throughout flight


# ======================================================================================
# Flight Mode
# ======================================================================================
def run_flight_mode():
    detector = FlightDetector()

    log_event("BOOT: Flight computer started")
    detector.calibrate(altimeter)
    log_event(f"CALIBRATION: ground_alt={detector.ground_alt:.2f}m")
    print("Ready. Waiting for launch...\n")

    while True:
        agl   = detector.get_agl(altimeter)
        state = detector.update(agl)

        # Show landing timer countdown while below threshold
        timer_str = ""
        if state == FlightState.LAUNCHED and detector.below_time is not None:
            remaining = LANDING_HOLD_TIME - (time.monotonic() - detector.below_time)
            timer_str = f" | Landing in: {remaining:.1f}s"

        data_line = f"State: {state:10s} | AGL: {agl:6.1f}m{timer_str}"
        print(data_line)
        if state == FlightState.LAUNCHED:
            log_flight_data(data_line)

        if state == FlightState.LANDED:
            log_event("EVENT: LANDING CONFIRMED", agl)
            print("\nLanding confirmed. Rotating servo to 0 degrees.")
            my_servo.angle = 0
            log_event("EVENT: SERVO DEPLOYMENT COMPLETE")
            break

        time.sleep(LOOP_DT)


# ======================================================================================
# Entry Point
# ======================================================================================
run_flight_mode()
