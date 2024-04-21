#!/usr/bin/python3

import time
import sys
sys.path.append("..")
from tang_control.tang_control.config import Pin
from gpiozero import PWMOutputDevice

motor_pin = PWMOutputDevice(Pin.pwm_r)
try:
    while True:
        motor_pin.value = 0.0
        time.sleep(1.0)
        motor_pin.value = 0.5
        time.sleep(1.0)
        motor_pin.value = 0.8
        time.sleep(1.0)
finally:
    motor_pin.close()