# ======================================================================================
# Developer: Noah Wons
# Program: Sample program to test motors on raspi GPIO ports
# Contact: wons123@iastate.edu
# ======================================================================================


import RPi.GPIO as GPIO
import time
ESC1 = 18
ESC2 = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC1, GPIO.OUT)
GPIO.setup(ESC2, GPIO.OUT)

GPIO.setwarnings(False)

pwm1 = GPIO.PWM(ESC1, 50)
pwm2 = GPIO.PWM(ESC2, 50)

pwm1.start(0)
pwm2.start(0)

try:
	pwm1.ChangeDutyCycle(10)
	pwm2.ChangeDutyCycle(10)
	time.sleep(5)
	
	pwm1.ChangeDutyCycle(5)
	pwm2.ChangeDutyCycle(5)
	time.sleep(5)

finally:
	pwm1.stop()
	pwm2.stop()
	GPIO.cleanup()

