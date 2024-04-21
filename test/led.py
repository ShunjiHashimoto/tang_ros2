#!/usr/bin/python3

import time
import sys
sys.path.append("..")
from tang_control.tang_control.config import Pin
from gpiozero import LED

led = LED(Pin.led)
try:
    while True:
        led.on()
        print(f"blink: {led.value}")
        time.sleep(1.0)
        led.off()
        print(f"stop: {led.value}")
        time.sleep(1.0)
finally:
    led.release()