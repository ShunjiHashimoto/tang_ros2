#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

class Pin:
    encoder_l_A  = 5
    encoder_l_B  = 6
    direction_l  = 17
    pwm_l        = 12
    encoder_r_A  = 22
    encoder_r_B  = 27
    direction_r  = 18
    pwm_r        = 13
    manual_mode  = 16
    follow_mode  = 21
    vrx_channel  = 0
    vry_channel  = 1
    swt_channel  = 2
    led_follow = 25
    led = 26
    emergency_mode = 11

class FOLLOWPID:
    p_gain = 2.307
    d_gain = 0.0
    #d_gain = 7.0
    dt = 0.1

class PID:
    Kp_v = 1.0
    Ki_v = 0.01
    Kd_v = 0.00
    Kp_w = 0.5
    Ki_w = 0.001
    Kd_w = 0.00
    max_error_sum_w = 80
    max_error_sum_v = 10
    dt = 0.005 # 0.0001がmax

class PWM:
    # PWM周波数をHzで指定
    freq = 1000 # [Hz]
    max_duty = 0.4

class Fig:
    time_data  = []
    vel_data = []
    w_data = []
    target_vel_data = []
    target_w_data = []
    target_a_data = []

class Control:
    # 最大速度
    max_linear_vel = 0.3
    max_angular_vel = 1.0
    max_linear_vel_manual = 0.8
    max_angular_vel_manual = 0.8
    # duty比の最大値
    max_duty = 0.2
    # joystickの最大値
    max_joystick_val = 1000.0
    velocity_thresh = 1e-2
    # 入力電圧
    input_v = 26.1
    # 目標角速度
    w_target = 0.0001
    # 目標速度
    v_target = 0.3
    # 目標加速度
    a_target = 0.1
    # 目標角加速度
    alpha_target = 0.2
    # 目標減速加速度
    d_target = -0.25
    # 逆起電圧定数
    Ke_r = 0.5905
    Ke_l = 0.5905
    # トルク定数
    Kt_r = 0.6666
    Kt_l = 0.6666
    # 巻線抵抗
    R = 3.05931
    # モータ１回転あたりのエンコーダ値
    encoder_1rotation_r = 2030*2
    encoder_1rotation_l = 2020*2
    # エンコーダ値1あたりの回転角度[rad]
    radian_1encoder_r = 2*math.pi/encoder_1rotation_r
    radian_1encoder_l = 2*math.pi/encoder_1rotation_l
    # モータの回転数
    rotation_num = 2
    # トレッド幅[m]
    tread_w = 0.32
    # 車輪半径[m]
    wheel_r = 0.05
    # 車体質量
    M = 24.6
    # 車体慣性モーメント J = ml^2
    # J = 1/3(a^2 + b^2) 44cm, 40cm = 0.58999
    J = M*(0.22*0.22 + 0.2*0.2)/3

class HumanFollowParam:
    depth_min_thresh = 0.5
