#!/usr/bin/python3

import time
import sys
sys.path.append("..")
from gpiozero import PWMOutputDevice, LED
from config import Pin

direction1 = LED(Pin.direction_r_FWD) # FWD_r
direction2 = LED(Pin.direction_r_REV) # REV_r
motor_r = PWMOutputDevice(Pin.pwm_r, frequency=500)
#direction1 = LED(Pin.direction_r_FWD) # FWD_r
#direction2 = LED(Pin.direction_r_REV) # REV_r
#motor_r = PWMOutputDevice(Pin.pwm_r, frequency=500)

direction1.on()
time.sleep(0.5)
direction2.off()
time.sleep(0.5)
motor_r.value=0.2
print(f"right motor run")
time.sleep(3.0)
print(f"right motor stop")
motor_r.value=0.0
time.sleep(1.0)
