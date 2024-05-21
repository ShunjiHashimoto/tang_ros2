#!/usr/bin/python3

import time
import sys
sys.path.append("..")
from gpiozero import PhaseEnableMotor
from tang_control.config import Pin

motor_r = PhaseEnableMotor(phase=Pin.direction_l, enable=Pin.pwm_l)
motor_l = PhaseEnableMotor(phase=Pin.direction_r, enable=Pin.pwm_r)

try:
    motor_r.forward(0.2)
    print(f"right motor run")
    time.sleep(3.0)
    print(f"right motor stop")
    motor_r.stop()
    time.sleep(1.0)

    motor_l.forward(0.2)
    print(f"left motor run")
    time.sleep(3.0)
    print(f"left motor stop")
    motor_l.stop()
    time.sleep(1.0)

    motor_r.backward(0.3)
    print("right back")
    time.sleep(3.0)
    motor_r.stop()
    print("right motor stop")
    time.sleep(1.0)

    motor_l.backward(0.3)
    print("left back")
    time.sleep(3.0)
    motor_l.stop()
    print("left motor stop")
    time.sleep(1.0)

    motor_r.close()
    motor_l.close()
except:
    motor_r.stop()
    motor_l.stop()
    motor_r.close()
    motor_l.close()
