"""
7-in-1 Soil Sensor Reader — Feather M4 Express + SparkFun RS-485 Breakout (BOB-10124)

Wiring:
    Feather M4          SparkFun RS-485 Breakout
    ─────────           ───────────────────────
    TX  (pin 1)    →    DI   (Driver Input)
    RX  (pin 0)    →    RO   (Receiver Output)
    D5             →    RTS  (RE/DE direction)
    3V             →    VCC
    GND            →    GND

    Soil Sensor          SparkFun RS-485 Breakout
    ───────────          ───────────────────────
    Yellow (A+)    →    A
    Blue   (B-)    →    B

    Soil Sensor power: 12-24 V DC (brown = VCC, black = GND)

Sensor defaults: Modbus RTU, address 0x01, 9600 baud, 8N1
"""

import board
import busio
import digitalio
import time
import struct

# ── Configuration ────────────────────────────────────────────────────────────
BAUD = 9600
SLAVE_ADDR = 0x01
DE_RE_PIN = board.D5   # direction control pin → RTS on breakout
READ_INTERVAL = 5      # seconds between readings

# Register addresses (from datasheet section 2.3)
REG_PH       = 0x0006  # pH              unit: 0.01 pH
REG_HUMIDITY = 0x0012  # humidity        unit: 0.1 %RH  (temperature follows at +1)
REG_EC       = 0x0015  # conductivity    unit: 1 µS/cm
REG_NPK      = 0x001E  # nitrogen        unit: mg/kg  (phosphorus +1, potassium +2)

# ── CRC-16 (Modbus) ─────────────────────────────────────────────────────────
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

# ── Build a Modbus RTU "Read Holding Registers" request ─────────────────────
def build_read_request(slave: int, start: int, count: int) -> bytes:
    frame = struct.pack(">BBHH", slave, 0x03, start, count)
    c = crc16(frame)
    frame += struct.pack("<H", c)          # CRC is little-endian in Modbus
    return frame

# ── Send request and receive response ───────────────────────────────────────
def modbus_read(uart, de_re, slave: int, start: int, count: int):
    request = build_read_request(slave, start, count)

    # flush any stale bytes
    while uart.in_waiting:
        uart.read(uart.in_waiting)

    # switch to transmit mode
    de_re.value = True
    time.sleep(0.005)          # ≥ 4-byte silent gap required before frame
    uart.write(request)
    # wait for bytes to fully leave the UART shift register
    # at 4800 baud each byte ≈ 2.1 ms — wait for full frame + margin
    time.sleep(len(request) * 12 / BAUD + 0.002)
    de_re.value = False                    # back to receive mode

    # Collect bytes until we have enough for echo + response (or just response).
    # The echo (if present) has byte[2] == 0x00 (start-reg high byte); the real
    # response has byte[2] == count*2 (byte count).  We scan for the response.
    expected = 3 + count * 2 + 2          # addr+func+bytecount + data + crc
    max_bytes = len(request) + expected   # worst case: full echo then response
    deadline = time.monotonic() + 5.0
    buf = bytearray()

    while len(buf) < max_bytes and time.monotonic() < deadline:
        avail = uart.in_waiting
        if avail:
            buf.extend(uart.read(avail))
        else:
            time.sleep(0.005)

    # Find the start of the real response: slave / 0x03 / (count*2)
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

    # verify CRC
    payload = buf[: expected - 2]
    rx_crc = buf[expected - 2] | (buf[expected - 1] << 8)
    if crc16(payload) != rx_crc:
        print("  !! CRC mismatch")
        return None

    # check for Modbus exception
    if buf[1] & 0x80:
        print(f"  !! Modbus exception code {buf[2]:#04x}")
        return None

    # unpack register values (big-endian signed 16-bit)
    values = []
    for i in range(count):
        val = struct.unpack_from(">h", buf, 3 + i * 2)[0]
        values.append(val)
    return values

# ── Pretty-print one reading ────────────────────────────────────────────────
def print_reading(ph_r, hum_r, temp_r, ec_r, n_r, p_r, k_r):
    ph       = ph_r / 100.0    # unit: 0.01 pH
    moisture = hum_r / 10.0    # unit: 0.1 %RH
    temp_c   = temp_r / 10.0   # unit: 0.1 °C  (signed — handles sub-zero)
    ec       = ec_r             # unit: 1 µS/cm
    n        = n_r              # unit: mg/kg
    p        = p_r
    k        = k_r

    temp_f = temp_c * 9.0 / 5.0 + 32.0

    print("──────────────────────────────────")
    print(f"  Moisture      : {moisture:.1f} %RH")
    print(f"  Temperature   : {temp_c:.1f} °C  /  {temp_f:.1f} °F")
    print(f"  Conductivity  : {ec} µS/cm")
    print(f"  pH            : {ph:.2f}")
    print(f"  Nitrogen  (N) : {n} mg/kg")
    print(f"  Phosphorus(P) : {p} mg/kg")
    print(f"  Potassium (K) : {k} mg/kg")
    print("──────────────────────────────────")

# ── Hardware setup ───────────────────────────────────────────────────────────
de_re = digitalio.DigitalInOut(DE_RE_PIN)
de_re.direction = digitalio.Direction.OUTPUT
de_re.value = False                        # start in receive mode

uart = busio.UART(
    board.TX,                              # Feather M4 TX → DI on breakout
    board.RX,                              # Feather M4 RX → RO on breakout
    baudrate=BAUD,
    bits=8,
    parity=None,
    stop=1,
    timeout=1,
)

print(f"\n7-in-1 Soil Sensor — polling every {READ_INTERVAL}s  (baud {BAUD})\n")

# ── Main loop ────────────────────────────────────────────────────────────────
while True:
    ph_regs  = modbus_read(uart, de_re, SLAVE_ADDR, REG_PH,       1)
    ht_regs  = modbus_read(uart, de_re, SLAVE_ADDR, REG_HUMIDITY,  2)
    ec_regs  = modbus_read(uart, de_re, SLAVE_ADDR, REG_EC,        1)
    npk_regs = modbus_read(uart, de_re, SLAVE_ADDR, REG_NPK,       3)

    if all(r is not None for r in [ph_regs, ht_regs, ec_regs, npk_regs]):
        print_reading(ph_regs[0], ht_regs[0], ht_regs[1],
                      ec_regs[0], npk_regs[0], npk_regs[1], npk_regs[2])
    else:
        print("  !! No valid response — check wiring, baud rate, and sensor power")
    time.sleep(READ_INTERVAL)