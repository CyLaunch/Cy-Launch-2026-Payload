# ======================================================================================
# Developer: Noah Wons
# Program: Altimeter-only flight detection for rover system using MPL3115A2.
#          Launch is detected when altitude exceeds 50m AGL.
#          Landing is detected when altitude drops back below 10m AGL and remains
#          there for 20 seconds. After a 20s nosecone lockout, the orientation motor
#          runs a forward/reverse sequence, then the soil sensor collects one valid
#          reading.
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import busio
import countio
import digitalio
import struct
import adafruit_mpl3115a2
from adafruit_pca9685 import PCA9685

# ======================================================================================
# CONFIGURATION
# ======================================================================================

LOG_FILE  = "flight_log.txt"
DATA_FILE = "flight_data.txt"

LAUNCH_ALT_THRESHOLD  = 50   # meters AGL — above this confirms launch
LANDED_ALT_THRESHOLD  = 10   # meters AGL — below this starts landing timer
LANDING_HOLD_TIME     = 20   # seconds below LANDED_ALT_THRESHOLD to confirm landing
LANDING_LOCKOUT       = 0    # seconds after launch before landing is evaluated

LAUNCH_CONFIRM = 5           # consecutive readings above threshold to confirm launch

LOOP_DT = 0.05               # 20 Hz

NOSECONE_LOCKOUT = 20        # seconds to wait after landing before rover activates

# --- Motor sequence (matches tripleSlowMotorTest, duration extended to 20s) ---
SLOW_SPEED  = 100             # % duty cycle
RUN_TIME    = 20.0           # seconds per direction
RAMP_TIME   = 1.0            # seconds to ramp from 0 to target speed
RAMP_STEPS  = 20             # increments during ramp

# --- Soil sensor ---
BAUD       = 9600
SLAVE_ADDR = 0x01
DE_RE_PIN  = board.D5
REG_PH       = 0x0006
REG_HUMIDITY = 0x0012
REG_EC       = 0x0015
REG_NPK      = 0x001E


# ======================================================================================
# Logging
# ======================================================================================
def fmt_time(t):
    t = int(t * 1000)
    ms = t % 1000
    t //= 1000
    s = t % 60
    t //= 60
    m = t % 60
    h = t // 60
    return f"{h:02d}:{m:02d}:{s:02d}.{ms:03d}"

def log_event(event, altitude=None):
    timestamp = fmt_time(time.monotonic())
    if altitude is not None:
        line = f"[{timestamp}] {event} | Alt: {altitude:.2f}m AGL\n"
    else:
        line = f"[{timestamp}] {event}\n"
    print(line, end="")
    try:
        with open(LOG_FILE, "a") as f:
            f.write(line)
    except Exception as e:
        print(f"[LOG ERROR] Could not write to {LOG_FILE}: {e}")

def log_flight_data(line):
    timestamp = fmt_time(time.monotonic())
    entry = f"[{timestamp}] {line}\n"
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
        self.below_time    = None

    def calibrate(self, altimeter, num_samples=50):
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
                        self.below_time = now
                    elif (now - self.below_time) >= LANDING_HOLD_TIME:
                        self.state = FlightState.LANDED
                else:
                    self.below_time = None

        if self.state != prev_state:
            log_event(f"STATE: {prev_state} -> {self.state}", agl)

        return self.state


# ======================================================================================
# Motor class — from tripleSlowMotorTest
# ======================================================================================
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


# ======================================================================================
# Motor helpers — from tripleSlowMotorTest
# ======================================================================================
def stop_all(motors):
    for m in motors:
        m.stop()

def ramp_all(motors, target_speed):
    step_delay = RAMP_TIME / RAMP_STEPS
    sign = 1 if target_speed >= 0 else -1
    for i in range(1, RAMP_STEPS + 1):
        current = sign * (abs(target_speed) * i / RAMP_STEPS)
        for m in motors:
            m.set_speed(current)
        time.sleep(step_delay)

def run_all_and_report(motors, speed, duration):
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
            stop_all(motors)
            return


# ======================================================================================
# Soil sensor — from testSoilTester
# ======================================================================================
def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def build_read_request(slave, start, count):
    frame = struct.pack(">BBHH", slave, 0x03, start, count)
    c = crc16(frame)
    frame += struct.pack("<H", c)
    return frame

def modbus_read(uart, de_re, slave, start, count):
    request = build_read_request(slave, start, count)

    while uart.in_waiting:
        uart.read(uart.in_waiting)

    de_re.value = True
    time.sleep(0.005)
    uart.write(request)
    time.sleep(len(request) * 12 / BAUD + 0.002)
    de_re.value = False

    expected = 3 + count * 2 + 2
    max_bytes = len(request) + expected
    deadline = time.monotonic() + 5.0
    buf = bytearray()

    while len(buf) < max_bytes and time.monotonic() < deadline:
        avail = uart.in_waiting
        if avail:
            buf.extend(uart.read(avail))
        else:
            time.sleep(0.005)

    resp_start = None
    expected_byte_count = count * 2
    for i in range(len(buf) - 2):
        if buf[i] == slave and buf[i + 1] == 0x03 and buf[i + 2] == expected_byte_count:
            resp_start = i
            break

    if resp_start is None or len(buf) - resp_start < expected:
        print(f"  !! Timeout — got {len(buf)} bytes: {bytes(buf).hex()}")
        return None

    buf = buf[resp_start:]
    payload = buf[: expected - 2]
    rx_crc = buf[expected - 2] | (buf[expected - 1] << 8)
    if crc16(payload) != rx_crc:
        print("  !! CRC mismatch")
        return None

    if buf[1] & 0x80:
        print(f"  !! Modbus exception code {buf[2]:#04x}")
        return None

    values = []
    for i in range(count):
        val = struct.unpack_from(">h", buf, 3 + i * 2)[0]
        values.append(val)
    return values

def collect_valid_soil_reading(uart, de_re):
    """Poll until all four Modbus reads succeed and return the reading."""
    print("Collecting soil data — retrying until a complete valid reading is received...")
    while True:
        ph_regs  = modbus_read(uart, de_re, SLAVE_ADDR, REG_PH,       1)
        ht_regs  = modbus_read(uart, de_re, SLAVE_ADDR, REG_HUMIDITY,  2)
        ec_regs  = modbus_read(uart, de_re, SLAVE_ADDR, REG_EC,        1)
        npk_regs = modbus_read(uart, de_re, SLAVE_ADDR, REG_NPK,       3)

        if all(r is not None for r in [ph_regs, ht_regs, ec_regs, npk_regs]):
            return ph_regs[0], ht_regs[0], ht_regs[1], ec_regs[0], npk_regs[0], npk_regs[1], npk_regs[2]

        print("  !! Incomplete reading — retrying...")
        time.sleep(1)

def log_soil_reading(ph_r, hum_r, temp_r, ec_r, n_r, p_r, k_r):
    ph       = ph_r / 100.0
    moisture = hum_r / 10.0
    temp_c   = temp_r / 10.0
    temp_f   = temp_c * 9.0 / 5.0 + 32.0
    ec       = ec_r
    n, p, k  = n_r, p_r, k_r

    lines = [
        "── SOIL READING ──────────────────",
        f"  Moisture      : {moisture:.1f} %RH",
        f"  Temperature   : {temp_c:.1f} °C  /  {temp_f:.1f} °F",
        f"  Conductivity  : {ec} µS/cm",
        f"  pH            : {ph:.2f}",
        f"  Nitrogen  (N) : {n} mg/kg",
        f"  Phosphorus(P) : {p} mg/kg",
        f"  Potassium (K) : {k} mg/kg",
        "──────────────────────────────────",
    ]
    for line in lines:
        print(line)
        log_event(line)


# ======================================================================================
# Hardware Setup
# ======================================================================================
i2c = busio.I2C(board.SCL, board.SDA)

# --- Altimeter ---
print("Initializing MPL3115A2 altimeter...")
altimeter = None
try:
    altimeter = adafruit_mpl3115a2.MPL3115A2(i2c)
    altimeter.sealevel_pressure = 101325
    print("  OK MPL3115A2 found")
except Exception as e:
    print(f"  ERROR MPL3115A2 not detected: {e}")

if altimeter is None:
    print("ERROR: Altimeter required. Halting.")
    while True:
        time.sleep(1)

# --- PCA9685 ---
print("Initializing PCA9685...")
while not i2c.try_lock():
    pass
found = [hex(a) for a in i2c.scan()]
i2c.unlock()
print(f"  I2C devices: {found}")
if "0x40" not in found:
    print("  ERROR: PCA9685 not found at 0x40 — check wiring/power and restart.")
    while True:
        time.sleep(1)
pca = PCA9685(i2c)
pca.frequency = 1000
print("  PCA9685 OK")

# --- Soil sensor UART ---
de_re = digitalio.DigitalInOut(DE_RE_PIN)
de_re.direction = digitalio.Direction.OUTPUT
de_re.value = False

uart = busio.UART(
    board.TX,
    board.RX,
    baudrate=BAUD,
    bits=8,
    parity=None,
    stop=1,
    timeout=1,
)


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

            # --- Nosecone lockout ---
            log_event(f"EVENT: Waiting {NOSECONE_LOCKOUT}s for nosecone lockout")
            time.sleep(NOSECONE_LOCKOUT)

            # --- Motor sequence (tripleSlowMotorTest, RUN_TIME = 20s) ---
            motors = [
                Motor(pca, ch_fwd=0, ch_rev=1, occ_pin=board.D6, name="Drive"),
                Motor(pca, ch_fwd=11, ch_rev=10, occ_pin=board.D10, name="Orientation"),

            ]
            for m in motors:
                m.reset_encoder()

            print(f"\nFORWARD {SLOW_SPEED}%  (ramping over {RAMP_TIME}s)")
            log_event(f"EVENT: MOTOR FORWARD START ({SLOW_SPEED}% for {RUN_TIME}s)")
            run_all_and_report(motors, SLOW_SPEED, RUN_TIME)
            stop_all(motors)
            time.sleep(1)


            for m in motors:
                m.deinit()
            pca.deinit()
            log_event("EVENT: MOTOR SEQUENCE COMPLETE")

            # --- Soil sensor ---
            log_event("EVENT: SOIL SAMPLING START")
            reading = collect_valid_soil_reading(uart, de_re)
            log_soil_reading(*reading)
            log_event("EVENT: SOIL SAMPLING COMPLETE")

            while True:
                time.sleep(1)

        time.sleep(LOOP_DT)


# ======================================================================================
# Entry Point
# ======================================================================================
run_flight_mode()
