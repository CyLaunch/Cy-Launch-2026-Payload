import time
import math
import board
import adafruit_icm20x
import adafruit_mpl3115a2
# import pwmio
# from adafruit_motor import servo

# ============================================================
# SERVO TEST (commented out)
# ============================================================
# SERVO_PIN = board.D5
# pwm = pwmio.PWMOut(SERVO_PIN, duty_cycle=0, frequency=50)
# my_servo = servo.Servo(pwm, min_pulse=500, max_pulse=2500)
#
# def test_servo():
#     print("Servo test starting...")
#     for angle in range(0, 45, 2):
#         my_servo.angle = angle
#         time.sleep(0.05)
#     time.sleep(0.5)
# ============================================================

DT = 0.1  # 10 Hz output rate

def mag3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)

# ---------------- I2C Setup ----------------
i2c = board.I2C()

# ---------------- IMU (ICM-20948) ----------------
print("Initializing ICM-20948 IMU...")
icm = None
for addr in (0x69, 0x68):
    try:
        icm = adafruit_icm20x.ICM20948(i2c, address=addr)
        print(f"  ✅ ICM-20948 found at 0x{addr:02X}")
        break
    except Exception as e:
        print(f"  ❌ Not at 0x{addr:02X}: {e}")

if icm is None:
    print("ERROR: ICM-20948 not detected. Check wiring.")

# ---------------- Altimeter (MPL3115A2) ----------------
print("Initializing MPL3115A2 altimeter...")
mpl = None
try:
    mpl = adafruit_mpl3115a2.MPL3115A2(i2c)
    # mpl.sealevel_pressure = 1013.25  # adjust for local conditions (hPa)
    print("  ✅ MPL3115A2 found")
except Exception as e:
    print(f"  ❌ MPL3115A2 not detected: {e}")

if icm is None and mpl is None:
    print("No sensors detected. Halting.")
    while True:
        time.sleep(1)

print()
print("=" * 65)
print(f"{'ALT(m)':>8} {'P(hPa)':>8} {'T(C)':>6} | "
      f"{'Ax':>6} {'Ay':>6} {'Az':>6} | "
      f"{'Gx':>6} {'Gy':>6} {'Gz':>6} | "
      f"{'|a|':>6} {'|g|':>6}")
print("=" * 65)

while True:
    # --- Altimeter ---
    if mpl is not None:
        try:
            alt   = mpl.altitude
            pres  = mpl.pressure / 100  # Pa -> hPa
            temp  = mpl.temperature
        except Exception as e:
            alt, pres, temp = float('nan'), float('nan'), float('nan')
            print(f"Altimeter read error: {e}")
    else:
        alt, pres, temp = float('nan'), float('nan'), float('nan')

    # --- IMU ---
    if icm is not None:
        try:
            ax, ay, az = icm.acceleration  # m/s^2
            gx, gy, gz = icm.gyro          # rad/s
            a_mag  = mag3(ax, ay, az)
            g_mag  = mag3(gx, gy, gz)
        except Exception as e:
            ax = ay = az = gx = gy = gz = a_mag = g_mag = float('nan')
            print(f"IMU read error: {e}")
    else:
        ax = ay = az = gx = gy = gz = a_mag = g_mag = float('nan')

    print(
        f"{alt:8.2f} {pres:8.2f} {temp:6.2f} | "
        f"{ax:6.2f} {ay:6.2f} {az:6.2f} | "
        f"{gx:6.3f} {gy:6.3f} {gz:6.3f} | "
        f"{a_mag:6.2f} {g_mag:6.3f}"
    )

    time.sleep(DT)