import sys
sys.path.append("..")
from tang_control.tang_control.config import Pin, Control
from gpiozero import Button
import time

encoder_values = {'r': 0, 'l': 0}
last_encoder_pin = Pin.encoder_r_A

def encoder_r_A_callback():
    global last_encoder_pin 
    if last_encoder_pin == Pin.encoder_r_B:
        encoder_values['r'] += 1
        last_encoder_pin = Pin.encoder_r_A

def encoder_r_B_callback():
    global last_encoder_pin 
    if last_encoder_pin == Pin.encoder_r_A:
        encoder_values['r'] += 1
        last_encoder_pin = Pin.encoder_r_B

encoder_r_A = Button(Pin.encoder_r_A)
encoder_r_B = Button(Pin.encoder_r_B)

encoder_r_A.when_pressed = encoder_r_A_callback
encoder_r_B.when_pressed = encoder_r_B_callback

while True:
    print(f"encoder_r = {encoder_values['r']}, encoder_l = {encoder_values['l']}")
    time.sleep(0.1)
