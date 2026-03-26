# ======================================================================================
# Developer: Noah Wons
# Program: Test script for Adafruit ICM-20948 9-DOF IMU on FeatherWing M4 Express
# Additional Notes:
# - ICM-20948 provides accelerometer (m/s^2), gyroscope (rad/s), and magnetometer (uT).
# - Default I2C addresses: 0x69 or 0x68 (AD0 pin).
# - Requires adafruit_icm20x.mpy in /lib.
#
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import math
import board
import adafruit_icm20x

DT = 0.1  # 10 Hz output rate

def mag3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)

# --- I2C Setup ---
i2c = board.I2C()

# --- ICM-20948 Init ---
print("Initializing ICM-20948...")
imu = None
for addr in (0x69, 0x68):
    try:
        imu = adafruit_icm20x.ICM20948(i2c, address=addr)
        print(f"  ICM-20948 found at 0x{addr:02X}")
        break
    except Exception as e:
        print(f"  Not at 0x{addr:02X}: {e}")

if imu is None:
    print("ERROR: ICM-20948 not detected. Check wiring and that adafruit_icm20x is in /lib.")
    while True:
        time.sleep(1)

print()
print("=" * 80)
print(f"{'Ax':>7} {'Ay':>7} {'Az':>7} {'|a|':>7} | "
      f"{'Gx':>7} {'Gy':>7} {'Gz':>7} {'|g|':>7} | "
      f"{'Mx':>7} {'My':>7} {'Mz':>7} {'|m|':>7}")
print(f"{'m/s2':>7} {'m/s2':>7} {'m/s2':>7} {'m/s2':>7} | "
      f"{'rad/s':>7} {'rad/s':>7} {'rad/s':>7} {'rad/s':>7} | "
      f"{'uT':>7} {'uT':>7} {'uT':>7} {'uT':>7}")
print("=" * 80)

try:
    while True:
        try:
            ax, ay, az = imu.acceleration   # m/s^2
            gx, gy, gz = imu.gyro           # rad/s
            mx, my, mz = imu.magnetic       # gauss
        except Exception as e:
            print(f"Read error: {e}")
            time.sleep(DT)
            continue

        a_mag = mag3(ax, ay, az)
        g_mag = mag3(gx, gy, gz)
        m_mag = mag3(mx, my, mz)

        print(
            f"{ax:7.2f} {ay:7.2f} {az:7.2f} {a_mag:7.2f} | "
            f"{gx:7.3f} {gy:7.3f} {gz:7.3f} {g_mag:7.3f} | "
            f"{mx:7.4f} {my:7.4f} {mz:7.4f} {m_mag:7.4f}"
        )

        time.sleep(DT)

except KeyboardInterrupt:
    print("\nStopped.")
