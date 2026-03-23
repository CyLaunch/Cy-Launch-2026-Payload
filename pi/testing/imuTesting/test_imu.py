# ======================================================================================
# Developer: Noah Wons
# Program: Test imu connection with raspi and output data
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import adafruit_bno055

# Initialize I2C
i2c = board.I2C()

# Update address to 0x39 based on your i2cdetect result
try:
    sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x39)
    print("Successfully connected to sensor at 0x39")
except Exception as e:
    print(f"Failed to connect: {e}")
    exit()

while True:
    # Fetch orientation data
    euler = sensor.euler
    
    if euler[0] is not None:
        print(f"Heading: {euler[0]:0.2f}° | Roll: {euler[1]:0.2f}° | Pitch: {euler[2]:0.2f}°")
    else:
        print("Sensor is initializing/calibrating...")
        
    time.sleep(0.2)