#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
from datetime import datetime
from gpiozero import LED, PWMOutputDevice
import sys
import matplotlib.pyplot as plt
from tang_control.config import Pin, PID, PWM, Fig, Control

class Motor:
    def __init__(self):
        self.r_pwm = PWMOutputDevice(Pin.pwm_r, frequency = 500)
        self.l_pwm = PWMOutputDevice(Pin.pwm_l, frequency = 500)
        self.r_FWD = LED(Pin.direction_r_FWD)
        self.r_REV = LED(Pin.direction_r_REV)
        self.l_FWD = LED(Pin.direction_l_FWD)
        self.l_REV = LED(Pin.direction_l_REV)
    
    def calc_motor_speed(self, v_target, w_target):
        vel_r = v_target + w_target * Control.tread_w/2
        vel_l = v_target - w_target * Control.tread_w/2
        rotation_speed_r = (vel_r/Control.wheel_r)*Control.gear_ratio*60/(2*math.pi)
        rotation_speed_l = (vel_l/Control.wheel_r)*Control.gear_ratio*60/(2*math.pi)
        return rotation_speed_r, rotation_speed_l
    
    def cal_duty_brushless(self, rotation_speed_r, rotation_speed_l):
        # CuGo V3のギア比は1
        # 4.5Vで4000[r/min], 0Vで0[r/min]
        # x =4.5/4000
        # y = 0.001125x（y  = V, x = rpm）
        volt_r = 0.001125*rotation_speed_r
        volt_l = 0.001125*rotation_speed_l
        duty_r = volt_r/5.0
        duty_l = volt_l/5.0
        return duty_r, duty_l
        
    def pwm_control_r_FWD(self, duty):
        self.motor.r_FWD.on()
        self.motor.r_REV.off()
        self.motor.r_pwm.value = duty
        return
    def pwm_control_r_REV(self, duty):
        self.motor.r_FWD.off()
        self.motor.r_REV.on()
        self.motor.r_pwm.value = duty
        return
    def pwm_control_l_FWD(self, duty):
        self.motor.l_FWD.on()
        self.motor.l_REV.off()
        self.motor.l_pwm.value = duty
        return
    def pwm_control_l_REV(self, duty):
        self.motor.l_FWD.on()
        self.motor.l_REV.off()
        self.motor.l_pwm.value = duty
        return

    def run(self, duty_r, duty_l):
        if(abs(duty_r) > Control.max_duty or abs(duty_l > Control.max_duty)): 
            print(f"over duty, r,l = {duty_r}, {duty_l}")
            return
        if duty_r > 0:
            self.motor.pwm_control_r_FWD(duty_r)
        else:
            self.motor.pwm_control_r_REV(abs(duty_r))
        if duty_l > 0:
            self.motor.pwm_control_l_FWD(duty_l)
        else:
            self.motor.pwm_control_l_REV(abs(duty_l))
        return
        
    def stop(self):
        self.motor.l_pwm.value = 0.0
        self.motor.r_pwm.value = 0.0

def main():
    # 目標速度
    v_target = 0.1
    # 目標角速度
    w_target = 0.3
    # 目標加速度
    a_target = 0.001
    # 目標角加速度
    alpha_target = 0.0001
    motor = Motor()
    while(1):
        motor.run(v_target, w_target, a_target, alpha_target)
        if(motor.encoder_values['r']>10000):
            motor.stop()
    
if __name__ == "__main__":
    main()