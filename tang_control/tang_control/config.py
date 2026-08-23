#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

class Pin:
    ## ブラシレスモータの設定
    pwm_l            = 13 # AN1（コネクタ番号①）, 0.0 ~ 0.9V
    direction_l_FWD  = 20 # DIG1（コネクタ番号①）, 0.0~5.0V
    direction_l_REV  = 18 # encoderピンの黄線, SWB(REV、モータが反対方向に回転)
    direction_l      = 18

    pwm_r            = 12 # AN2（コネクタ番号②）,0.0 ~ 0.9V
    direction_r_FWD  = 17 # DIG2（コネクタ番号②）, 0.0~5.0V
    direction_r_REV  = 19 # encoderピンの青線, SWB(REV、モータが反対方向に回転)
    direction_r      = 17
    
    ## DCモータの設定
    #pwm_l            = 13 # AN1（コネクタ番号①）, 0.0 ~ 0.9V
    #direction_l_FWD  = 18 # DIG1（コネクタ番号①）, 0.0~5.0V
    #direction_l_REV  = 20 # encoderピンの黄線, SWB(REV、モータが反対方向に回転)
    #direction_l      = 18

    #pwm_r            = 12 # AN2（コネクタ番号②）,0.0 ~ 0.9V
    #direction_r_FWD  = 17 # DIG2（コネクタ番号②）, 0.0~5.0V
    #direction_r_REV  = 19 # encoderピンの青線, SWB(REV、モータが反対方向に回転)
    #direction_r      = 17

    encoder_r_A  = 22 # encoderピンの黄線, SWB(REV、モータが反対方向に回転)
    encoder_r_B  = 27 # encoderピンの青線, SWB(REV、モータが反対方向に回転)
    encoder_l_A  = 5
    encoder_l_B  = 6

    follow_mode  = 16
    manual_mode  = 21
    mode_led = 14

    low_speed_button = 3
    high_speed_button = 4
    low_speed_led = 25
    high_speed_led = 26

    vrx_channel  = 0
    vry_channel  = 1
    swt_channel  = 2
    adc_bat      = 3

    green_led = 25
    red_led = 26
    buzzer = 2
    emergency_mode = 11
    
    # Joystickノードで使用するボタン
    emergency_button = 5
    unlock_emergency_button = 4
    followme_start_button = 7
    followme_stop_button = 6
    teleop_start_button = [0, 1, 2, 3]
    speed_mode_button = [9,10]

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
    frequency = 500 # [Hz]
    max_duty = 0.3
    max_turbo_duty = 0.4
    max_duty_follow = 0.4
    min_duty = 0.05
    turn_const_duty_r = 0.1 # 右車輪の定数デューティ比, 超信地旋回時
    turn_const_duty_l = 0.1 # 左車輪の定数デューティ比

class Fig:
    time_data  = []
    vel_data = []
    w_data = []
    target_vel_data = []
    target_w_data = []
    target_a_data = []

class Control:
    # ------------------------------------------------------------------
    # 現行制御：CuGoV4 RS-485用
    # TangControllerとcontroller_core.pyから参照する。
    # ------------------------------------------------------------------
    manual_low_max_v_mps = 0.15
    manual_low_max_w_radps = 0.6
    manual_high_max_v_mps = 0.30
    manual_high_max_w_radps = 1.0
    # ジョイスティックを前後反転して取り付けたため、CH1の符号を反転する。
    # ロボット前進をROSの正方向、後退を負方向に合わせる。
    manual_throttle_sign = -1.0
    # 同じ取付変更でCH0も反転したため、西側への操作をROSの正角速度にする。
    manual_steering_sign = 1.0
    follow_low_max_v_mps = 0.15
    follow_high_max_v_mps = 0.30
    # 旧参照との互換用。FOLLOWの既定・LOW上限を示す。
    follow_max_v_mps = follow_low_max_v_mps
    # TANG追従旋回の調整値はここを唯一の参照元とする。
    follow_normal_max_w_radps = math.radians(15.0)
    follow_extreme_angle_rad = math.radians(45.0)
    follow_extreme_max_w_radps = math.radians(35.0)
    # モータへ渡す最終安全上限は、45度超の旋回上限に合わせる。
    follow_max_w_radps = follow_extreme_max_w_radps
    follow_angular_sign = -1.0
    command_ema_alpha = 0.75
    follow_accel_limit_mps2 = 1.0
    follow_cmd_timeout_sec = 0.5

    wheel_radius_left = 0.03858
    wheel_radius_right = 0.03858
    tread = 0.376
    reduction_ratio = 20.0
    rs485_max_motor_rpm = 2600.0
    rs485_min_motor_rpm = 80.0
    anti_creep_start_rpm = 120.0

    # ------------------------------------------------------------------
    # 旧制御：GPIO PWM・旧teleop互換用
    # tang_control/motor.py、tang_teleop/tang_joy.pyなどが参照する。
    # 現行のTangControllerでは使用しない。
    # ------------------------------------------------------------------
    max_target_v = 0.3
    max_target_w = 1.0
    velocity_thresh = 1e-2
    max_motor_rpm = 4000
    # 入力電圧
    input_v = 26.1
    # 目標角速度
    target_w = 0.0001
    # 目標速度
    target_v = 0.3
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
    # トレッド幅[m], V3は0.356
    tread_width = 0.3
    # 車輪半径[m], CuGoV3は0.05
    wheel_radius = 0.03858
    # CuGoV4のギア比, V3は1.0
    gear_ratio = 20
    # 車体質量
    M = 24.6
    # 車体慣性モーメント J = ml^2
    # J = 1/3(a^2 + b^2) 44cm, 40cm = 0.58999
    J = M*(0.22*0.22 + 0.2*0.2)/3
    # ブラシレスモータの電圧とモータ回転数[rpm]の関係, 4.5Vで4000[r/min]よりy = 4.5/4000x = 0.001125x
    volt_and_rpm_gain = 0.01125
    #volt_and_rpm_gain = 0.02125 # DCモータ設定
    # デューティ比計算に用いる直流電圧
    src_volt = 5.0
    
class HumanFollowParam:
    depth_min_thresh = 0.5

class LiDARParam:
    inverted = -1
    # 後方長と幅はP00-Bと同じ。前方は最前端のLiDARまでを車体外形に含める。
    body_front_length_m = 0.320
    body_rear_length_m = 0.430
    body_half_width_m = 0.250
    # 実機計測値：LiDARは旋回中心から前方0.320m。
    position_x_m = 0.320
    position_y_m = 0.0
    # 車体外形から確保する方向別の近接停止余裕[m]。
    obstacle_front_clearance_m = 0.300
    obstacle_side_clearance_m = 0.100
    # 手動操縦中は操作者が周囲を確認するため、停止余裕を縮小する。
    manual_obstacle_front_clearance_m = 0.050
    manual_obstacle_side_clearance_m = 0.050

class JoyParam:
    # joystickの最大値
    max_joystick_val = 1064.0
    ema_alpha = Control.command_ema_alpha  # 旧GPIO PWM経路でも現行制御と同じEMA係数を使う
