# ======================================================================================
# Developer: Noah Wons
# Program: Test light sensor data collection (NOT USED)
# Contact: wons123@iastate.edu
# ======================================================================================


import time
import board
from adafruit_apds9960.apds9960 import APDS9960

i2c = board.I2C()
apds = APDS9960(i2c) # Default address is 0x39

# Enable features
apds.enable_proximity = True
apds.enable_color = True

print("Reading APDS-9960 data... Press Ctrl+C to stop.")

while True:
    # Read proximity (0-255, higher is closer)
    print(f"Proximity: {apds.proximity}")
    
    # Read color data (Red, Green, Blue, Clear)
    r, g, b, c = apds.color_data
    print(f"Red: {r}, Green: {g}, Blue: {b}, Light Intensity: {c}")
    
    time.sleep(0.5)
