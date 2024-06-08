import sys
sys.path.append("..")
from tang_control.tang_control.config import Pin, Control
from gpiozero import Button
import time

encoder_values = {'r': 0}
last_encoder_pin = Pin.encoder_r_A
state_A = False
state_B = False

def encoder_r_A_callback():
    global last_encoder_pin, state_A, state_B, encoder_values
    state_A = not state_A  # Aの状態をトグル
    if state_A:  # AがHighになった時
        if state_B:  # BがHighの場合
            encoder_values['r'] += 1  # 正回転
        else:
            encoder_values['r'] -= 1  # 逆回転

def encoder_r_B_callback():
    global last_encoder_pin, state_A, state_B, encoder_values
    state_B = not state_B  # Bの状態をトグル
    if state_B:  # BがHighになった時
        if state_A:  # AがHighの場合
            encoder_values['r'] -= 1  # 逆回転
        else:
            encoder_values['r'] += 1  # 正回転

encoder_r_A = Button(Pin.encoder_r_A, pull_up=False)
encoder_r_B = Button(Pin.encoder_r_B, pull_up=False)

encoder_r_A.when_pressed = encoder_r_A_callback
encoder_r_A.when_released = encoder_r_A_callback
encoder_r_B.when_pressed = encoder_r_B_callback
encoder_r_B.when_released = encoder_r_B_callback

while True:
    print(f"encoder_r = {encoder_values['r']}")
    time.sleep(0.1)
