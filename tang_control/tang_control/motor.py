#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
from datetime import datetime
from gpiozero import LED, PWMOutputDevice
import sys
from tang_control.config import Pin, PID, PWM, Fig, Control

class Motor:
    def __init__(self):
        self.r_pwm = PWMOutputDevice(Pin.pwm_r, frequency = 500)
        self.l_pwm = PWMOutputDevice(Pin.pwm_l, frequency = 500)
        self.r_FWD = LED(Pin.direction_r_FWD)
        self.r_REV = LED(Pin.direction_r_REV)
        self.l_FWD = LED(Pin.direction_l_FWD)
        self.l_REV = LED(Pin.direction_l_REV)
    
    def normalize_joystick_input(self, value, max_value=Control.max_duty):
        return value*max_value
    
    def calc_motor_command_pwm(self, joystick_x, joystick_y):
        x = self.normalize_joystick_input(joystick_x)
        y = self.normalize_joystick_input(joystick_y)
        if(y > 0):
            if(x > 0):
                right_motor_pwm = y 
                left_motor_pwm = ((Control.max_duty - x)/Control.max_duty) * y
            if(x <= 0):
                right_motor_pwm = ((Control.max_duty + x)/Control.max_duty) * y
                left_motor_pwm = y
        elif(y <= 0):
            if(x > 0):
                right_motor_pwm = y 
                left_motor_pwm = -abs(((Control.max_duty - x)/Control.max_duty) * y)
            if(x <= 0):
                left_motor_pwm =  y
                right_motor_pwm = -abs(((Control.max_duty + x)/Control.max_duty) * y)
        print(f"right; {right_motor_pwm:.2f} ; left: {left_motor_pwm:.2f}")
        left_motor_pwm = max(min(left_motor_pwm, Control.max_duty), -Control.max_duty)
        right_motor_pwm = max(min(right_motor_pwm, Control.max_duty), -Control.max_duty)
        return right_motor_pwm, left_motor_pwm

    def calc_motor_command_vel(self, joystick_x, joystick_y):
        linear_velocity = self.normalize_joystick_input(joystick_y, Control.max_linear_vel)
        angular_velocity = self.normalize_joystick_input(joystick_x, Control.max_angular_vel)
        return linear_velocity, angular_velocity
    
    def calc_motor_speed(self, v_target, w_target):
        vel_r = v_target + w_target * Control.tread_w/2
        vel_l = v_target - w_target * Control.tread_w/2
        rotation_speed_r = (vel_r/Control.wheel_r)*Control.gear_ratio*60/(2*math.pi)
        rotation_speed_l = (vel_l/Control.wheel_r)*Control.gear_ratio*60/(2*math.pi)
        return rotation_speed_r, rotation_speed_l
    
    def calc_duty_brushless(self, v_target, w_target):
        rotation_speed_r, rotation_speed_l = self.calc_motor_speed(v_target, w_target)
        volt_r = Control.volt_and_rpm_gain*rotation_speed_r
        volt_l = Control.volt_and_rpm_gain*rotation_speed_l
        duty_r = volt_r/Control.src_volt
        duty_l = volt_l/Control.src_volt
        return duty_r, duty_l
        
    def pwm_control_r_FWD(self, duty):
        self.r_FWD.on()
        self.r_REV.off()
        self.r_pwm.value = duty
        return
    def pwm_control_r_REV(self, duty):
        self.r_FWD.off()
        self.r_REV.on()
        self.r_pwm.value = duty
        return
    def pwm_control_l_FWD(self, duty):
        self.l_FWD.on()
        self.l_REV.off()
        self.l_pwm.value = duty
        return
    def pwm_control_l_REV(self, duty):
        self.l_FWD.off()
        self.l_REV.on()
        self.l_pwm.value = duty
        return
    def pwm_control_stop(self):
        self.r_pwm.value = 0.0
        self.r_FWD.off()
        self.r_REV.off()
        self.l_pwm.value = 0.0
        self.l_FWD.off()
        self.l_REV.off()

    def run(self, duty_r, duty_l):
        if(abs(duty_r) > Control.max_duty or abs(duty_l > Control.max_duty)): 
            print(f"over duty, r,l = {duty_r}, {duty_l}")
            return
        if abs(duty_r) < Control.min_duty and abs(duty_l) < Control.min_duty:
            print(f"pwm control stop")
            self.pwm_control_stop()
            return
        if duty_r > 0:
            self.pwm_control_r_FWD(duty_r)
        else:
            self.pwm_control_r_REV(abs(duty_r))
        if duty_l > 0:
            self.pwm_control_l_FWD(duty_l)
        else:
            self.pwm_control_l_REV(abs(duty_l))
        return
        
    def stop(self):
        self.l_pwm.value = 0.0
        self.r_pwm.value = 0.0

def main():
    # 目標速度
    v_target = 0.1
    # 目標角速度
    w_target = 0.3
    motor = Motor()
    while(1):
        duty_r, duty_l = motor.calc_duty_brushless(v_target, w_target)
        motor.run(duty_r, duty_l)
    
if __name__ == "__main__":
    main()