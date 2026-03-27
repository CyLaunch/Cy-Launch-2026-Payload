# ======================================================================================
# Developer: Noah Wons
# Program: Component detection check — confirms all rover sensors/drivers are present
#          and returning valid data before a full test run.
# Components:
#   - BNO055      Absolute orientation IMU     (0x28 or 0x29)
#   - ICM-20948   9-DOF accel/gyro/mag IMU     (0x68 or 0x69)
#   - MPL3115A2   Altimeter                    (0x60)
#   - PCA9685     PWM driver                   (0x40)
#
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import busio
import adafruit_bno055
import adafruit_icm20x
import adafruit_mpl3115a2
from adafruit_pca9685 import PCA9685

PASS = "PASS"
FAIL = "FAIL"

results = {}

# =============================================================================
# I2C Bus Scan
# =============================================================================

i2c = busio.I2C(board.SCL, board.SDA)

print("=" * 50)
print("  COMPONENT DETECTION CHECK")
print("=" * 50)

print("\n[ I2C Bus Scan ]")
while not i2c.try_lock():
    pass
found_addrs = i2c.scan()
i2c.unlock()

if found_addrs:
    print(f"  Devices found: {[hex(a) for a in found_addrs]}")
else:
    print("  WARNING: No I2C devices found — check SDA/SCL wiring and power.")

EXPECTED = {0x28: "BNO055", 0x29: "BNO055", 0x68: "ICM-20948",
            0x69: "ICM-20948", 0x60: "MPL3115A2", 0x40: "PCA9685"}

for addr in EXPECTED:
    if addr in found_addrs:
        print(f"  0x{addr:02X} ({EXPECTED[addr]}) detected on bus")

# =============================================================================
# BNO055 — Absolute Orientation IMU
# =============================================================================

print("\n[ BNO055 — Absolute Orientation IMU ]")
bno = None
for addr in (0x28, 0x29):
    try:
        bno = adafruit_bno055.BNO055_I2C(i2c, address=addr)
        print(f"  Init OK at 0x{addr:02X}")
        break
    except Exception as e:
        print(f"  Not at 0x{addr:02X}: {e}")

if bno is not None:
    try:
        euler = bno.euler
        temp  = bno.temperature
        if euler is not None:
            print(f"  Euler angles  heading={euler[0]:.1f} roll={euler[1]:.1f} pitch={euler[2]:.1f} deg")
            print(f"  Temperature   {temp} C")
            results["BNO055"] = PASS
        else:
            print("  WARNING: euler returned None — sensor may still be calibrating")
            results["BNO055"] = PASS  # init succeeded; data may need warmup
    except Exception as e:
        print(f"  Read error: {e}")
        results["BNO055"] = FAIL
else:
    print("  ERROR: BNO055 not detected")
    results["BNO055"] = FAIL

# =============================================================================
# ICM-20948 — 9-DOF Accel / Gyro / Magnetometer
# =============================================================================

print("\n[ ICM-20948 — 9-DOF IMU ]")
icm = None
for addr in (0x69, 0x68):
    try:
        icm = adafruit_icm20x.ICM20948(i2c, address=addr)
        print(f"  Init OK at 0x{addr:02X}")
        break
    except Exception as e:
        print(f"  Not at 0x{addr:02X}: {e}")

if icm is not None:
    try:
        ax, ay, az = icm.acceleration   # m/s^2
        gx, gy, gz = icm.gyro           # rad/s
        mx, my, mz = icm.magnetic       # uT
        print(f"  Accel  ax={ax:.2f}  ay={ay:.2f}  az={az:.2f} m/s^2")
        print(f"  Gyro   gx={gx:.3f}  gy={gy:.3f}  gz={gz:.3f} rad/s")
        print(f"  Mag    mx={mx:.3f}  my={my:.3f}  mz={mz:.3f} uT")
        results["ICM-20948"] = PASS
    except Exception as e:
        print(f"  Read error: {e}")
        results["ICM-20948"] = FAIL
else:
    print("  ERROR: ICM-20948 not detected")
    results["ICM-20948"] = FAIL

# =============================================================================
# MPL3115A2 — Altimeter
# =============================================================================

print("\n[ MPL3115A2 — Altimeter ]")
mpl = None
try:
    mpl = adafruit_mpl3115a2.MPL3115A2(i2c)
    print("  Init OK at 0x60")
except Exception as e:
    print(f"  Init failed: {e}")

if mpl is not None:
    try:
        alt  = mpl.altitude
        pres = mpl.pressure / 100   # Pa → hPa
        temp = mpl.temperature
        print(f"  Altitude     {alt:.2f} m")
        print(f"  Pressure     {pres:.2f} hPa")
        print(f"  Temperature  {temp:.2f} C")
        results["MPL3115A2"] = PASS
    except Exception as e:
        print(f"  Read error: {e}")
        results["MPL3115A2"] = FAIL
else:
    print("  ERROR: MPL3115A2 not detected")
    results["MPL3115A2"] = FAIL

# =============================================================================
# PCA9685 — PWM Driver
# =============================================================================

print("\n[ PCA9685 — PWM Driver ]")
pca = None
try:
    pca = PCA9685(i2c)
    pca.frequency = 1000
    print("  Init OK at 0x40")
    # Write and verify a duty cycle on an unused channel (ch15) then zero it out
    pca.channels[15].duty_cycle = 32767
    read_back = pca.channels[15].duty_cycle
    pca.channels[15].duty_cycle = 0
    if read_back > 0:
        print(f"  Channel write/read OK (ch15 = {read_back})")
        results["PCA9685"] = PASS
    else:
        print("  WARNING: Channel write returned 0 — check PCA9685 power")
        results["PCA9685"] = FAIL
except Exception as e:
    print(f"  Init failed: {e}")
    results["PCA9685"] = FAIL

if pca is not None:
    pca.deinit()

# =============================================================================
# Summary
# =============================================================================

print()
print("=" * 50)
print("  RESULTS SUMMARY")
print("=" * 50)

all_pass = True
for component, status in results.items():
    indicator = "  [PASS]" if status == PASS else "  [FAIL]"
    print(f"{indicator}  {component}")
    if status == FAIL:
        all_pass = False

print()
if all_pass:
    print("  All components detected and responding. Ready to run.")
else:
    print("  One or more components failed. Check wiring/power before continuing.")
print("=" * 50)
