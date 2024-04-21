#!/usr/bin/python3

import time
import sys
sys.path.append("..")
from tang_control.tang_control.config import Pin
from gpiozero import Button

def do_stuff():
    print("Button Pressed")
    
button = Button(Pin.teleop_mode)
button.when_pressed = do_stuff

while True:
    print("Busy doing other stuff")
    time.sleep(2)