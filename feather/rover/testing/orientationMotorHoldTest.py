# ======================================================================================
# Developer: Noah Wons
# Program: Brakes the orientation motor to hold its current position and monitors OCC pins.
#          Used to identify which OCC pin (D6, D9, D10) belongs to this motor.
#
# Wiring:
#   Motor: PCA9685 CH2 (fwd) / CH3 (rev)
#   OCC candidates: D6, D9, D10
#
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import busio
import digitalio
from adafruit_pca9685 import PCA9685

# --- Hardware setup ---
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000

CH_FWD = 2
CH_REV = 3

# --- OCC pins ---
occ_d6 = digitalio.DigitalInOut(board.D6)
occ_d6.direction = digitalio.Direction.INPUT
occ_d6.pull = digitalio.Pull.UP

occ_d9 = digitalio.DigitalInOut(board.D9)
occ_d9.direction = digitalio.Direction.INPUT
occ_d9.pull = digitalio.Pull.UP

occ_d10 = digitalio.DigitalInOut(board.D10)
occ_d10.direction = digitalio.Direction.INPUT
occ_d10.pull = digitalio.Pull.UP

# --- Brake: both inputs high locks the TB9051FTG in place ---
pca.channels[CH_FWD].duty_cycle = 65535
pca.channels[CH_REV].duty_cycle = 65535

print(f"Motor braked/holding position (CH{CH_FWD} + CH{CH_REV} both high). Press CTRL+C to stop.")
print()

try:
    while True:
        d6  = "OCC" if not occ_d6.value  else "ok"
        d9  = "OCC" if not occ_d9.value  else "ok"
        d10 = "OCC" if not occ_d10.value else "ok"
        print(f"D6={d6} | D9={d9} | D10={d10}")
        time.sleep(0.25)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    pca.channels[CH_FWD].duty_cycle = 0
    pca.channels[CH_REV].duty_cycle = 0
    pca.deinit()
