#!/usr/bin/python3

import time
import sys
sys.path.append("..")
from gpiozero import PhaseEnableMotor

motor = PhaseEnableMotor(phase=18, enable=12)
try:
    motor.forward(0.2)
    print(f"motor run")
    time.sleep(10.0)
    print(f"motor stop")
    time.sleep(1.0)
    motor.stop()
    print("stop")
    time.sleep(3.0)
    #motor.backward(0.3)
    #print("back")
    #time.sleep(3.0)
    motor.close()
except:
    motor.stop()
    motor.close()
