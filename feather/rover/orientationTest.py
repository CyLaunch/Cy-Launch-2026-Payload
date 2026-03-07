# ======================================================================================
# Developer: Noah Wons
# Program: Test script for PWM motor control using PCA9685 on FeatherWing M4 Express
#          Includes a calibration routine for ESCs and a simple test sequence to verify motor response.
#          NOTE: This test script has NOT been tested with our motors (TACON BIGFOOT25) and PWM driver.
# Contact: wons123@iastate.edu
# ======================================================================================

import time
import board
import adafruit_bno055
from adafruit_pca9685 import PCA9685

# ==============================================================================
# CONFIGURATION — adjust these to match your setup
# ==============================================================================

MOTOR_ORIENTATION   = 0       # PCA9685 channel for orientation motor

PWM_FREQ            = 50      # Hz
PWM_MIN             = 204     # 1000µs — stop
PWM_MAX             = 409     # 2000µs — full speed
PWM_MID             = 307     # 1500µs — neutral

PID_KP              = 2.0     # Start here, tune up if sluggish
PID_KI              = 0.0     # Add small value if steady-state offset persists
PID_KD              = 0.5     # Increase if oscillating

CORRECT_EULER_INDEX = 1       # 1=roll, 2=pitch — confirm on bench by tilting
LEVEL_TOLERANCE_DEG = 2.0     # Degrees considered "level" (motor stops)
DT                  = 0.05    # 20 Hz control loop

# ==============================================================================
# HELPERS
# ==============================================================================

def set_motor_pwm(pca, channel, pwm_counts):
    pwm_counts = max(PWM_MIN, min(PWM_MAX, int(pwm_counts)))
    pca.channels[channel].duty_cycle = int(pwm_counts / 4096 * 65535)

def stop_motor(pca, channel):
    set_motor_pwm(pca, channel, PWM_MIN)

def arm_esc(pca):
    set_motor_pwm(pca, MOTOR_ORIENTATION, PWM_MIN)
    time.sleep(2.0)

def wait_for_calibration(bno):
    while True:
        sys, gyro, accel, mag = bno.calibration_status
        if sys >= 1 and gyro >= 1:
            break
        time.sleep(0.5)

# ==============================================================================
# PID
# ==============================================================================

class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self._integral   = 0.0
        self._prev_error = 0.0

    def compute(self, current_angle):
        error             = 0.0 - current_angle
        self._integral    = max(-50.0, min(50.0, self._integral + error * self.dt))
        derivative        = (error - self._prev_error) / self.dt
        self._prev_error  = error
        output            = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(PWM_MIN, min(PWM_MAX, PWM_MID + output))

# ==============================================================================
# MAIN
# ==============================================================================

i2c        = board.I2C()
bno        = adafruit_bno055.BNO055_I2C(i2c)
pca        = PCA9685(i2c)
pca.frequency = PWM_FREQ
pid        = PID(PID_KP, PID_KI, PID_KD, DT)

arm_esc(pca)
wait_for_calibration(bno)

try:
    while True:
        euler = bno.euler
        if euler is None or euler[CORRECT_EULER_INDEX] is None:
            stop_motor(pca, MOTOR_ORIENTATION)
            time.sleep(DT)
            continue

        angle = euler[CORRECT_EULER_INDEX]

        if abs(angle) <= LEVEL_TOLERANCE_DEG:
            stop_motor(pca, MOTOR_ORIENTATION)
        else:
            set_motor_pwm(pca, MOTOR_ORIENTATION, pid.compute(angle))

        time.sleep(DT)

except KeyboardInterrupt:
    stop_motor(pca, MOTOR_ORIENTATION)
    pca.deinit()