# ======================================================================================
# Developer: Noah Wons
# Program: Test script for PWM motor control using PCA9685 on FeatherWing M4 Express
# Additional Notes:
# - This script is designed to test the basic functionality of the TB9051FTG motor drivers connected to the PCA9685.
# - Each motor is controlled by two PWM channels for direction and speed control.
# - The test sequence runs each motor forward, then reverse, and finally stops it, with
#   a 2-second delay between each action for observation.
# - Note that this script is testing brushed DC motors.
# 
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import busio
from adafruit_pca9685 import PCA9685

# --- Setup I2C and PCA9685 ---
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # 1kHz PWM frequency for motor drivers

# --- Helper to set motor speed/direction ---
# Each TB9051FTG uses 2 PWM channels (PWM1 and PWM2)
# PWM1 > 0, PWM2 = 0 → forward
# PWM1 = 0, PWM2 > 0 → reverse
# PWM1 = 0, PWM2 = 0 → stop
# duty_cycle range: 0 (off) to 65535 (full on)

def set_motor(pwm1_channel, pwm2_channel, speed):
    """
    speed: -100 to 100
    positive = forward, negative = reverse, 0 = stop
    """
    duty = int(abs(speed) / 100 * 65535)
    if speed > 0:
        pca.channels[pwm1_channel].duty_cycle = duty
        pca.channels[pwm2_channel].duty_cycle = 0
    elif speed < 0:
        pca.channels[pwm1_channel].duty_cycle = 0
        pca.channels[pwm2_channel].duty_cycle = duty
    else:
        pca.channels[pwm1_channel].duty_cycle = 0
        pca.channels[pwm2_channel].duty_cycle = 0

def stop_all():
    set_motor(0, 1, 0)  # Motor 1 - leveling
    set_motor(2, 3, 0)  # Motor 2 - drive belt
    set_motor(4, 5, 0)  # Motor 3 - collection blades

# --- Motor channel assignments ---
# Motor 1 (leveling):          CH0=PWM1, CH1=PWM2
# Motor 2 (drive belt):        CH2=PWM1, CH3=PWM2
# Motor 3 (collection blades): CH4=PWM1, CH5=PWM2

print("Motor Test Starting...")
stop_all()
time.sleep(1)

# --- Test Motor 1 (Leveling) ---
print("Motor 1 FORWARD (leveling)")
set_motor(0, 1, 50)  # 50% speed forward
time.sleep(2)

print("Motor 1 REVERSE (leveling)")
set_motor(0, 1, -50)  # 50% speed reverse
time.sleep(2)

print("Motor 1 STOP")
set_motor(0, 1, 0)
time.sleep(1)

# --- Test Motor 2 (Drive Belt) ---
print("Motor 2 FORWARD (drive belt)")
set_motor(2, 3, 50)
time.sleep(2)

print("Motor 2 REVERSE (drive belt)")
set_motor(2, 3, -50)
time.sleep(2)

print("Motor 2 STOP")
set_motor(2, 3, 0)
time.sleep(1)

# --- Test Motor 3 (Collection Blades) ---
print("Motor 3 FORWARD (collection blades)")
set_motor(4, 5, 50)
time.sleep(2)

print("Motor 3 REVERSE (collection blades)")
set_motor(4, 5, -50)
time.sleep(2)

print("Motor 3 STOP")
set_motor(4, 5, 0)
time.sleep(1)

print("All motors tested. Done!")
stop_all()