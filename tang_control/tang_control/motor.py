#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
from datetime import datetime
from gpiozero import LED, PWMOutputDevice
from tang_control.config import Pin, PID, PWM, Control

class Motor:
    def __init__(self):
        self.r_pwm = PWMOutputDevice(Pin.pwm_r, frequency=PWM.frequency)
        self.l_pwm = PWMOutputDevice(Pin.pwm_l, frequency=PWM.frequency)
        self.r_FWD = LED(Pin.direction_r_FWD)
        self.r_REV = LED(Pin.direction_r_REV)
        self.l_FWD = LED(Pin.direction_l_FWD)
        self.l_REV = LED(Pin.direction_l_REV)
        # 過去のジョイスティック入力を記憶するための変数を追加
        self.prev_normalized_value_x = 0
        self.prev_normalized_value_y = 0
        self.alpha = 0.1  # EMAの平滑化係数。0に近いほど変化が緩やか
    
    def normalize_joystick_input(self, value, max_value=PWM.max_duty, prev_value=0):
        # ジョイスティック入力がゼロなら、ゼロを返す
        # if abs(value) <= 0.0000001:
        #     return 0
        # ジョイスティック入力を正規化
        normalized_value = value * max_value
        # 平滑化された値を計算
        smoothed_value = self.alpha * normalized_value + (1 - self.alpha) * prev_value
        return smoothed_value
    
    # xが前後、yが左右、左が＋
    def calc_duty_by_joyinput(self, joystick_x, joystick_y):
        normarized_x = self.normalize_joystick_input(joystick_x, prev_value=self.prev_normalized_value_x)
        normarized_y = self.normalize_joystick_input(joystick_y, prev_value=self.prev_normalized_value_y)
        # 現在の値を保存しておく
        self.prev_normalized_value_x = normarized_x
        self.prev_normalized_value_y = normarized_y
        # その場旋回を実現するためのロジック
        # yが前後方向、前進が＋
        # xが左右方向、右が＋
        # 論理はjoystickのx,yの値をそのまま使い、制御にはnormarized_x,yを使う
        print(f"joystick: x={joystick_x:.2f}, y={joystick_y:.2f}, normarized_x={normarized_x:.2f}, normalized_y={normarized_y:.2f}", flush=True)
        if(joystick_y >= -0.15): # 前進（y>0）
            if(joystick_x > 0): # 右回転
                duty_r = PWM.turn_const_duty_r if (abs(joystick_y) < 0.05 and abs(joystick_x) > 0.85) else normarized_y
                duty_l = ((PWM.max_duty - normarized_x)/PWM.max_duty)*normarized_y
            if(joystick_x <= 0): # 左回転
                duty_r = ((PWM.max_duty + normarized_x)/PWM.max_duty)*normarized_y
                duty_l = PWM.turn_const_duty_l if (abs(joystick_y) < 0.05 and abs(joystick_x) > 0.85) else normarized_y
        elif(joystick_y < -0.15): # 後退（y<0）
            if(joystick_x > 0): # 右回転, ただし左車輪をより回し、右車輪はゆっくり回す
                duty_r = -PWM.turn_const_duty_r if (abs(joystick_y) < 0.05 and abs(joystick_x) > 0.85) else normarized_y
                duty_l = -abs(((PWM.max_duty - normarized_x)/PWM.max_duty)*normarized_y)
            if(joystick_y <= 0): # 左回転、ただし右車輪をより回し、左車輪はゆっくり回す
                duty_l = -PWM.turn_const_duty_l if (abs(joystick_y) < 0.05 and abs(joystick_x) > 0.85) else normarized_y
                duty_r = -abs(((PWM.max_duty + normarized_x)/PWM.max_duty)*normarized_y)
            
        print(f"duty_r: {duty_r:.2f}, duty_l: {duty_l:.2f}", flush=True)
        duty_l = max(min(duty_l, PWM.max_duty), -PWM.max_duty)
        duty_r = max(min(duty_r, PWM.max_duty), -PWM.max_duty)
        return duty_r, duty_l

    def calc_robot_vel_command(self, joystick_x, joystick_y):
        target_v = self.normalize_joystick_input(joystick_y, Control.max_target_v)
        target_w = self.normalize_joystick_input(joystick_x, Control.max_target_w)
        return target_v, target_w
    
    def calc_motor_speed(self, target_v, target_w):
        vel_r = target_v + target_w * Control.tread_width/2
        vel_l = target_v - target_w * Control.tread_width/2
        rotation_speed_r = (vel_r/Control.wheel_r)*Control.gear_ratio*60/(2*math.pi)
        rotation_speed_l = (vel_l/Control.wheel_r)*Control.gear_ratio*60/(2*math.pi)
        return rotation_speed_r, rotation_speed_l
    
    def calc_duty_by_vw(self, target_v, target_w):
        rotation_speed_r, rotation_speed_l = self.calc_motor_speed(target_v, target_w)
        volt_r = Control.volt_and_rpm_gain*rotation_speed_r
        volt_l = Control.volt_and_rpm_gain*rotation_speed_l
        duty_r = volt_r/Control.src_volt
        duty_l = volt_l/Control.src_volt
        return duty_r, duty_l
        
    def pwm_control_r_FWD(self, duty):
        self.r_FWD.on()
        #self.r_FWD.off()
        self.r_REV.off()
        self.r_pwm.value = duty
        return
    def pwm_control_r_REV(self, duty):
        #self.r_FWD.on()
        self.r_FWD.off()
        self.r_REV.on()
        self.r_pwm.value = duty
        return
    def pwm_control_l_FWD(self, duty):
        #self.l_FWD.on()
        self.l_FWD.on()
        self.l_REV.off()
        self.l_pwm.value = duty
        return
    def pwm_control_l_REV(self, duty):
        #self.l_FWD.on()
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
        return

    def run(self, duty_r, duty_l):
        if(abs(duty_r) > 0.2 or abs(duty_l) > 0.2): 
            print(f"pwm control skipped, because over duty, r,l = {duty_r}, {duty_l}")
            return
        if abs(duty_r) < PWM.min_duty and abs(duty_l) < PWM.min_duty:
            # print(f"pwm control stop because low duty")
            self.pwm_control_stop()
            return
        if duty_r > 0:
            self.pwm_control_r_FWD(duty_r)
        else:
            self.pwm_control_r_REV(abs(duty_r))
        if duty_l > 0:
            self.pwm_control_l_REV(duty_l)
        else:
            self.pwm_control_l_FWD(abs(duty_l))
        return
        
    def stop(self):
        self.l_pwm.value = 0.0
        self.r_pwm.value = 0.0

def main():
    # 目標速度
    target_v = 0.1
    # 目標角速度
    target_w = 0.3
    motor = Motor()
    while(1):
        duty_r, duty_l = motor.calc_duty_by_vw(target_v, target_w)
        motor.run(duty_r, duty_l)
    
if __name__ == "__main__":
    main()
