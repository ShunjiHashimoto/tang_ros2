import sys
sys.path.append("..")
from tang_control.config import Pin, Control
from gpiozero import Button, PhaseEnableMotor
import time

encoder_values = {'r': 0, 'l': 0}
last_encoder_pin_r = Pin.encoder_r_A
last_encoder_pin_l = Pin.encoder_l_A
motor_r = PhaseEnableMotor(phase=Pin.direction_r, enable=Pin.pwm_r)
motor_l = PhaseEnableMotor(phase=Pin.direction_l, enable=Pin.pwm_l)

def encoder_r_A_callback():
    global last_encoder_pin_r 
    if last_encoder_pin_r == Pin.encoder_r_B:
        encoder_values['r'] += 1
        last_encoder_pin_r = Pin.encoder_r_A

def encoder_r_B_callback():
    global last_encoder_pin_r 
    if last_encoder_pin_r == Pin.encoder_r_A:
        encoder_values['r'] += 1
        last_encoder_pin_r = Pin.encoder_r_B

def encoder_l_A_callback():
    global last_encoder_pin_l 
    if last_encoder_pin_l == Pin.encoder_l_B:
        encoder_values['l'] += 1
        last_encoder_pin_l = Pin.encoder_l_A

def encoder_l_B_callback():
    global last_encoder_pin_l 
    if last_encoder_pin_l == Pin.encoder_l_A:
        encoder_values['l'] += 1
        last_encoder_pin_l = Pin.encoder_l_B


encoder_r_A = Button(Pin.encoder_r_A)
encoder_r_B = Button(Pin.encoder_r_B)
encoder_r_A.when_pressed = encoder_r_A_callback
encoder_r_B.when_pressed = encoder_r_B_callback
encoder_l_A = Button(Pin.encoder_l_A)
encoder_l_B = Button(Pin.encoder_l_B)
encoder_l_A.when_pressed = encoder_l_A_callback
encoder_l_B.when_pressed = encoder_l_B_callback
motor_l.forward(0.2)
motor_r.forward(0.2)
while True:
    print(f"encoder_r = {encoder_values['r']}, encoder_l = {encoder_values['l']}")
    if(encoder_values['l'] >= Control.encoder_1rotation_l or encoder_values['r'] >= Control.encoder_1rotation_r):
        motor_l.stop()
        motor_r.stop()
        motor_l.close()
        motor_r.close()
        break