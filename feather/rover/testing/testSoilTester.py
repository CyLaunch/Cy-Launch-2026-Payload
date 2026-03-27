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

    Soil Sensor power: 12–24 V DC (brown = VCC, black = GND)

Sensor defaults: Modbus RTU, address 0x01, 4800 baud, 8N1
    Some sensors ship at 9600 — change BAUD below if you get no response.
"""

import board
import busio
import digitalio
import time
import struct

# ── Configuration ────────────────────────────────────────────────────────────
BAUD = 4800            # try 9600 if you get no response
SLAVE_ADDR = 0x01
START_REG = 0x0000
NUM_REGS = 7           # moisture, temp, EC, pH, N, P, K
DE_RE_PIN = board.D5   # direction control pin → RTS on breakout
READ_INTERVAL = 5      # seconds between readings

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
    frame = struct.pack(">B B H H", slave, 0x03, start, count)
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
    time.sleep(0.001)
    uart.write(request)
    # wait for bytes to fully leave the UART shift register
    # at 4800 baud each byte ≈ 2.1 ms — wait for full frame + margin
    time.sleep(len(request) * 12 / BAUD + 0.005)
    de_re.value = False                    # back to receive mode

    # expected response length: addr(1) + func(1) + bytecount(1) + data(count*2) + crc(2)
    expected = 3 + count * 2 + 2
    deadline = time.monotonic() + 1.0      # 1 s timeout
    buf = bytearray()

    while len(buf) < expected and time.monotonic() < deadline:
        avail = uart.in_waiting
        if avail:
            buf.extend(uart.read(avail))
        else:
            time.sleep(0.01)

    if len(buf) < expected:
        return None

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
def print_reading(regs):
    moisture = regs[0] / 10.0
    temp_c   = regs[1] / 10.0
    ec       = regs[2]
    ph       = regs[3] / 10.0
    n        = regs[4]
    p        = regs[5]
    k        = regs[6]

    temp_f = temp_c * 9.0 / 5.0 + 32.0

    print("──────────────────────────────────")
    print(f"  Moisture      : {moisture:.1f} %")
    print(f"  Temperature   : {temp_c:.1f} °C  /  {temp_f:.1f} °F")
    print(f"  Conductivity  : {ec} µS/cm")
    print(f"  pH            : {ph:.1f}")
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
    regs = modbus_read(uart, de_re, SLAVE_ADDR, START_REG, NUM_REGS)
    if regs is not None:
        print_reading(regs)
    else:
        print("  !! No valid response — check wiring, baud rate, and sensor power")
    time.sleep(READ_INTERVAL)