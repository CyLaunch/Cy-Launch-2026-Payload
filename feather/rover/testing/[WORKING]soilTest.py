"""
7-in-1 Soil Sensor — Modbus RTU Reader
Feather M4 Express + SP3485 RS-485 Breakout

Modbus RTU protocol (from sensor datasheet Ch. 2):
    - Encoding     : 8-bit binary
    - Data bits    : 8
    - Parity       : None
    - Stop bits    : 1
    - Error check  : CRC-16 (poly 0xA001)
    - Baud rate    : 9600 (factory default)
    - Slave address: 0x01 (factory default)
    - Function code: 0x03 (read holding registers)

    Request frame:
        [Address 1B][Function 1B][Start Reg 2B][Reg Count 2B][CRC 2B]
        - Start register and count are big-endian (high byte first)
        - CRC is little-endian (low byte first)
        - Silent gap >= 4 byte-times before and after frame

    Response frame:
        [Address 1B][Function 1B][Byte Count 1B][Data NB][CRC 2B]
        - Data is 2 bytes per register, big-endian, signed

    Register map (datasheet section 2.3):
        0x0006  pH              0.01 pH
        0x0012  Humidity        0.1  %RH
        0x0013  Temperature     0.1  °C   (two's complement when negative)
        0x0015  Conductivity    1    µS/cm
        0x001E  Nitrogen        1    mg/kg
        0x001F  Phosphorus      1    mg/kg
        0x0020  Potassium       1    mg/kg

Wiring:
    Feather M4        RS-485 Breakout        Soil Sensor
    ──────────        ────────────────       ───────────
    TX  → DI          A  ← Yellow (A+)
    RX  ← RO          B  ← Blue   (B-)
    D5  → RTS (RE/DE)
    3V  → VCC                                Brown → 12-24V DC
    GND → GND                                Black → GND
                                             (GND must be shared with Feather)
"""

import board
import busio
import digitalio
import time
import struct

# ── Configuration ────────────────────────────────────────────────────────────

BAUD         = 9600
SLAVE_ADDR   = 0x01
DE_RE_PIN    = board.D6
POLL_INTERVAL = 5       # seconds between readings
STARTUP_DELAY = 0      # seconds for sensor to boot after power-on
RESPONSE_TIMEOUT = 2.0  # seconds to wait for a response
DEBUG        = True      # print raw byte diagnostics

# Minimum silent gap between frames: 4 byte-times.
# At 9600 baud with 10 bits/byte (1 start + 8 data + 1 stop) each byte
# takes ~1.04 ms, so 4 bytes = ~4.2 ms.  We use 5 ms for margin.
SILENT_GAP = 0.005

# ── Register definitions ────────────────────────────────────────────────────

REGISTERS = {
    "pH":            {"addr": 0x0006, "count": 1, "scale": 0.01,  "unit": "pH"},
    "Humidity":      {"addr": 0x0012, "count": 1, "scale": 0.1,   "unit": "%RH"},
    "Temperature":   {"addr": 0x0013, "count": 1, "scale": 0.1,   "unit": "°C"},
    "Conductivity":  {"addr": 0x0015, "count": 1, "scale": 1,     "unit": "µS/cm"},
    "Nitrogen":      {"addr": 0x001E, "count": 1, "scale": 1,     "unit": "mg/kg"},
    "Phosphorus":    {"addr": 0x001F, "count": 1, "scale": 1,     "unit": "mg/kg"},
    "Potassium":     {"addr": 0x0020, "count": 1, "scale": 1,     "unit": "mg/kg"},
}


# ── CRC-16 / Modbus ─────────────────────────────────────────────────────────
#
# The CRC uses polynomial 0xA001 (bit-reversed 0x8005).
# Initial value 0xFFFF.  Each byte is XORed into the low byte of the CRC,
# then 8 right-shifts with conditional XOR.

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


# ── Build request frame ─────────────────────────────────────────────────────
#
# Request format (8 bytes total):
#   Byte 0     : slave address
#   Byte 1     : function code (0x03 = read holding registers)
#   Bytes 2-3  : starting register address (big-endian)
#   Bytes 4-5  : number of registers to read (big-endian)
#   Bytes 6-7  : CRC-16 (little-endian)

def build_request(slave: int, register: int, count: int) -> bytes:
    frame = struct.pack(">BBHH", slave, 0x03, register, count)
    c = crc16(frame)
    frame += struct.pack("<H", c)
    return frame


# ── Send request and receive response ───────────────────────────────────────
#
# Half-duplex RS-485 flow:
#   1. Assert DE/RE HIGH  → driver enabled, receiver disabled
#   2. Wait silent gap    → Modbus requires >= 4 byte-times of silence
#   3. Transmit frame
#   4. Wait for TX to finish on the wire
#   5. De-assert DE/RE LOW → driver disabled, receiver enabled
#   6. Wait for response bytes (slave turnaround is typically 50-200 ms)
#   7. Parse response: scan past any echo bytes to find the real reply
#
# The SP3485 ties RE and DE together on the breakout (active via RTS pin).
# When HIGH: driver on, receiver off.  When LOW: driver off, receiver on.
# Some boards leak echo bytes into RX despite receiver being disabled;
# the parser handles this by scanning for the valid response header.

def modbus_read(uart, de_re, slave: int, register: int, count: int):
    request = build_request(slave, register, count)

    if DEBUG:
        print(f"  [tx] addr=0x{slave:02X} reg=0x{register:04X} "
              f"count={count} → {request.hex()}")

    time.sleep(0.01)
    while uart.in_waiting:
        uart.read(uart.in_waiting)
        time.sleep(0.002)   # let stragglers arrive, then re-check
    while uart.in_waiting:
        uart.read(uart.in_waiting)

    # ── Transmit ─────────────────────────────────────────────────────────
    de_re.value = True                          # enable driver
    time.sleep(SILENT_GAP)                      # pre-frame silent gap
    uart.write(request)
    # write() blocks until TX shift register is empty; just need one bit-time of slack
    time.sleep(11 / BAUD + 0.0005)   # ~1.6 ms instead of ~10 ms
    de_re.value = False

    # Wait for all bytes to leave the UART shift register.
    # Each byte = 10 bits at 9600 baud ≈ 1.04 ms.  Use 11 bits for margin.
    tx_drain = len(request) * 11 / BAUD
    time.sleep(tx_drain + 0.001)

    # ── Receive ──────────────────────────────────────────────────────────
    de_re.value = False                         # enable receiver
    time.sleep(SILENT_GAP)                      # post-frame silent gap

    # Collect bytes: echo (if any) + sensor response.
    # Expected response length: 1 addr + 1 func + 1 bytecount + (count*2) data + 2 CRC
    resp_len = 3 + count * 2 + 2
    max_bytes = len(request) + resp_len         # echo + response worst case
    buf = bytearray()
    deadline = time.monotonic() + RESPONSE_TIMEOUT

    while len(buf) < max_bytes and time.monotonic() < deadline:
        avail = uart.in_waiting
        if avail:
            buf.extend(uart.read(avail))
        else:
            time.sleep(0.005)

    if DEBUG:
        print(f"  [rx] {len(buf)} bytes: {bytes(buf).hex()}")

    # ── Parse: find response header ──────────────────────────────────────
    # Scan for the sequence:  slave | 0x03 | (count*2)
    # This skips past any echo of the request that leaked into RX.
    expected_data_bytes = count * 2
    resp_start = None

    for i in range(len(buf) - 2):
        if (buf[i] == slave
                and buf[i + 1] == 0x03
                and buf[i + 2] == expected_data_bytes):
            # Make sure this isn't the echo (echo has register addr at byte[2],
            # not byte count).  The echo's byte[2] is the high byte of the
            # register address, which won't equal count*2 for our registers.
            resp_start = i
            break

    if resp_start is None:
        # Also check for Modbus exception response: slave | 0x83 | error_code
        for i in range(len(buf) - 2):
            if buf[i] == slave and buf[i + 1] == 0x83:
                print(f"  !! Modbus exception: code 0x{buf[i+2]:02X}")
                return None
        print(f"  !! No valid response found")
        return None

    resp = buf[resp_start:]

    if len(resp) < resp_len:
        print(f"  !! Incomplete response: {len(resp)}/{resp_len} bytes")
        return None

    # ── Verify CRC ───────────────────────────────────────────────────────
    payload = resp[:resp_len - 2]
    rx_crc = resp[resp_len - 2] | (resp[resp_len - 1] << 8)
    calc_crc = crc16(payload)

    if calc_crc != rx_crc:
        print(f"  !! CRC mismatch: received 0x{rx_crc:04X}, "
              f"calculated 0x{calc_crc:04X}")
        return None

    # ── Extract register values ──────────────────────────────────────────
    # Each register is a 16-bit signed integer, big-endian.
    # Temperature uses two's complement for negative values.
    values = []
    for i in range(count):
        val = struct.unpack_from(">h", resp, 3 + i * 2)[0]
        values.append(val)

    return values


# ── Read and display all sensor registers ────────────────────────────────────

def read_all_sensors(uart, de_re):
    results = {}

    for name, reg in REGISTERS.items():
        values = modbus_read(uart, de_re, SLAVE_ADDR,
                             reg["addr"], reg["count"])
        time.sleep(0.1)     # inter-request gap

        if values is not None:
            raw = values[0]
            scaled = raw * reg["scale"]
            results[name] = scaled
        else:
            results[name] = None

    return results


def print_results(results):
    print("──────────────────────────────────")

    for name, reg in REGISTERS.items():
        val = results.get(name)
        if val is None:
            print(f"  {name:14s}: -- error --")
            continue

        unit = reg["unit"]
        if reg["scale"] < 1:
            # fractional scaling → show decimal places
            decimals = len(str(reg["scale"]).split(".")[-1])
            print(f"  {name:14s}: {val:.{decimals}f} {unit}")
        else:
            print(f"  {name:14s}: {int(val)} {unit}")

    # bonus: show temperature in Fahrenheit
    if results.get("Temperature") is not None:
        temp_f = results["Temperature"] * 9.0 / 5.0 + 32.0
        print(f"  {'Temperature':14s}: {temp_f:.1f} °F")

    print("──────────────────────────────────")


# ── Hardware setup ───────────────────────────────────────────────────────────

de_re = digitalio.DigitalInOut(DE_RE_PIN)
de_re.direction = digitalio.Direction.OUTPUT
de_re.value = False                             # start in receive mode

uart = busio.UART(
    board.TX,                                   # Feather TX → DI on breakout
    board.RX,                                   # Feather RX → RO on breakout
    baudrate=BAUD,
    bits=8,
    parity=None,
    stop=1,
    timeout=1,
)

print(f"\n7-in-1 Soil Sensor — Modbus RTU")
print(f"Baud: {BAUD}  Address: 0x{SLAVE_ADDR:02X}  "
      f"Poll: {POLL_INTERVAL}s  Debug: {DEBUG}")
print(f"Waiting {STARTUP_DELAY}s for sensor to initialize...\n")
time.sleep(STARTUP_DELAY)

# ── Main loop ────────────────────────────────────────────────────────────────

while True:
    results = read_all_sensors(uart, de_re)

    if any(v is not None for v in results.values()):
        print_results(results)
    else:
        print("  !! No valid responses — check wiring, power, and ground")

    time.sleep(POLL_INTERVAL)