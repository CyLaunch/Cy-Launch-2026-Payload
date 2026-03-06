# ======================================================================================
# Developer: Noah Wons
# Program: Print orientation in plain english from imu
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import adafruit_bno055

# ---------- Tunables ----------
LEVEL_TOL_DEG = 2.0          # how close to 0° roll/pitch counts as "perfectly level"
PRINT_RATE_S = 0.2

# Initialize I2C
i2c = board.I2C()

# Your detected I2C address
ADDRESS = 0x39

try:
    sensor = adafruit_bno055.BNO055_I2C(i2c, address=ADDRESS)
    print(f"Successfully connected to sensor at 0x{ADDRESS:02X}")
except Exception as e:
    print(f"Failed to connect: {e}")
    raise SystemExit(1)

def english_orientation(heading, roll, pitch, tol=LEVEL_TOL_DEG):
    # Handle missing values
    if roll is None or pitch is None:
        return "Sensor is initializing/calibrating..."

    # Level check
    if abs(roll) <= tol and abs(pitch) <= tol:
        return f"✅ LEVEL with the ground (roll={roll:.2f}°, pitch={pitch:.2f}°)"

    # Describe pitch (forward/back)
    if pitch > tol:
        pitch_text = f"tilted UP in front (nose up) by {pitch:.2f}°"
    elif pitch < -tol:
        pitch_text = f"tilted DOWN in front (nose down) by {abs(pitch):.2f}°"
    else:
        pitch_text = "pitch ~ level"

    # Describe roll (left/right)
    if roll > tol:
        roll_text = f"rolled RIGHT by {roll:.2f}°"
    elif roll < -tol:
        roll_text = f"rolled LEFT by {abs(roll):.2f}°"
    else:
        roll_text = "roll ~ level"

    # Heading might be None sometimes; include only if present
    if heading is not None:
        return f"Not level: {pitch_text}, {roll_text} | heading {heading:.2f}°"
    return f"Not level: {pitch_text}, {roll_text}"

while True:
    euler = sensor.euler  # (heading, roll, pitch) in degrees, or None values while calibrating

    if euler is None:
        print("Sensor is initializing/calibrating...")
    else:
        heading, roll, pitch = euler
        # Raw numbers (optional but helpful)
        if heading is not None and roll is not None and pitch is not None:
            print(f"Heading: {heading:0.2f}° | Roll: {roll:0.2f}° | Pitch: {pitch:0.2f}°")

        # English description + explicit LEVEL message
        print(english_orientation(heading, roll, pitch))

    time.sleep(PRINT_RATE_S)
