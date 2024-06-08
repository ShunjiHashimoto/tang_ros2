#!/usr/bin/python3

import time
import sys
sys.path.append("..")
from gpiozero import LED

led = LED(26)
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
