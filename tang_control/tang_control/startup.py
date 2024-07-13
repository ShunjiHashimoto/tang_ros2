#!/usr/bin/python3

import time
import sys
#sys.path.append("..")
from gpiozero import LED
from config import Pin 

led = LED(Pin.green_led)
buzzer = LED(Pin.buzzer)
try:
    buzzer.on()
    time.sleep(0.5)
    buzzer.off()
    time.sleep(0.5)
    buzzer.on()
    time.sleep(0.5)
    buzzer.off()
    time.sleep(0.5)
    buzzer.on()
    time.sleep(0.5)
    buzzer.off()
    time.sleep(0.5)
    while True:
        led.on()
        print(f"blink: {led.value}")
        time.sleep(1.0)
        led.off()
        print(f"stop: {led.value}")
        time.sleep(1.0)
finally:
    buzzer.release()
    led.release()
