import time
import math
import board
import adafruit_bno055

i2c = board.I2C()
bno = adafruit_bno055.BNO055_I2C(i2c)

LEVEL_TOLERANCE  = 5.0   # degrees — within this angle of calibrated orientation = LEVEL
CALIBRATION_DURATION = 3.0
CALIBRATION_DT   = 0.1


def yz_angle(v1, v2):
    """Angle in degrees between two gravity vectors using only the Y and Z components.
    Ignores X so that rotation around the X axis does not affect the level reading."""
    y1, z1 = v1[1], v1[2]
    y2, z2 = v2[1], v2[2]
    dot  = y1 * y2 + z1 * z2
    mag1 = math.sqrt(y1 * y1 + z1 * z1)
    mag2 = math.sqrt(y2 * y2 + z2 * z2)
    if mag1 == 0 or mag2 == 0:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / (mag1 * mag2)))))


def calibrate_orientation():
    print("=" * 40)
    print("  CALIBRATION")
    print("  Place the rover in its level position and keep it still.")
    print("  Starting in 3 seconds...")
    print("=" * 40)
    time.sleep(3)

    samples = []
    end_time = time.monotonic() + CALIBRATION_DURATION
    print("Collecting baseline...", end="")

    while time.monotonic() < end_time:
        g = bno.gravity
        if g is not None and all(v is not None for v in g):
            samples.append(g)
        time.sleep(CALIBRATION_DT)

    if not samples:
        print(" FAILED (no IMU data). Defaulting to (0, 0, -9.8)")
        return (0.0, 0.0, -9.8)

    ref = tuple(sum(s[i] for s in samples) / len(samples) for i in range(3))
    print(f" done ({len(samples)} samples)")
    print(f"  Gravity reference: X={ref[0]:+.3f}  Y={ref[1]:+.3f}  Z={ref[2]:+.3f} m/s²")
    print()
    return ref


ref_gravity = calibrate_orientation()

print("BNO055 Z-Axis Level Test")
print(f"Tolerance: +/- {LEVEL_TOLERANCE} deg")
print(f"{'Gx':>8} {'Gy':>8} {'Gz':>8} {'Angle':>8} {'Status':>12}")
print("=" * 50)

while True:
    g = bno.gravity
    if g is None or any(v is None for v in g):
        print("No IMU data — check wiring!")
        time.sleep(0.5)
        continue

    angle  = yz_angle(ref_gravity, g)
    status = "LEVEL" if angle <= LEVEL_TOLERANCE else "CORRECTING"

    print(f"{g[0]:8.3f} {g[1]:8.3f} {g[2]:8.3f} {angle:8.2f} {status:>12}")
    time.sleep(0.1)
