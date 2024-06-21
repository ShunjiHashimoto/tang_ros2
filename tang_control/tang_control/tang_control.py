## モード切り替え機能
# joystickモジュールからモードの切り替え信号を受け取る
# 緊急停止のスイッチが押されれば、緊急停止モードに移行する
# 信号に応じて、下記のモードを切り替える 

## 追跡モード
# leg_tracker_ros2から送られた人物の位置をもとに、追跡対象者の中心位置を計算する
# 追跡対象者の位置に応じて目標角速度、速度を計算する
# それらをPWMに変換して、モータに指令する

## 手動モード
# joystickの現在の操作量を取得する
# 操作量に応じて目標角速度、速度を計算する
# それらをPWMに変換して、モータに指令する

## 緊急停止モード
# モータの制御を停止する

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from leg_tracker_ros2.msg import Person 
from tang_control.config import Pin, PWM, FOLLOWPID, HumanFollowParam, Control
from tang_control.motor import Motor
from gpiozero import Button, LED 
import spidev

try: 
    spi = spidev.SpiDev()
    spi.open(0,0)
    spi.max_speed_hz = 100000 
except:
    print("error: failed to open spi")
    
class TangController(Node):
    def __init__(self):
        super().__init__('tang_control')
        self.logger = get_logger('tang_control_logger')
        self.logger.info('TangController initialized')
        self.button_follow = Button(Pin.follow_mode)
        self.button_follow.when_pressed = self.switch_on_callback_follow
        self.button_manual = Button(Pin.manual_mode)
        self.button_manual.when_pressed = self.switch_on_callback_manual
        self.motor = Motor()
        self.mode = "manual"
        self.led = LED(Pin.led_follow)
        self.follow_target_person = Person()
        
        self.subscription = self.create_subscription(Person, 'follow_target_person', self.follow_target_callback, 10)
        
    def follow_target_callback(self, msg):
        print(f"person x: {msg.pose.position.x}, y: {msg.pose.position.y}", flush=True)
        self.follow_target_person = msg
        # -1.2 ~ 1.2 でyは変わる、またxも0 ~ 1.2で変わる
        # pwmは0.3をmaxとする
       
    # モード切替 
    def switch_on_callback_follow(self):
        self.mode = "follow"
        self.logger.info("追従モード")

    def switch_on_callback_manual(self):
        self.mode = "manual"
        self.logger.info("手動操作")
    
    # joystick信号の取得
    def read_analog_pin(self, channel):
        adc = spi.xfer2([1, (8 + channel)<<4, 0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data
    
    
    # PWM制御を使ったモータ制御
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

    def adjust_pwm_value(self, vel):
        """PWM値が-0.05から0.05の間にある場合、0に調整する"""
        if -0.001 <= vel <= 0.001:
            return 0.05
        return vel
    
    # pwmを使った手動操作
    def manual_control_with_pwm(self):
        # Read the joystick position data
        vrx_pos = self.read_analog_pin(Pin.vrx_channel) / Control.max_joystick_val*2 - 1  # normalize to [-1, 1]
        vry_pos = self.read_analog_pin(Pin.vry_channel) / Control.max_joystick_val*2 - 1   
        # Debugging
        print(f"Normalized X : {vrx_pos:.2f}, Normalized Y : {vry_pos:.2f}")
        duty_r, duty_l = self.calc_motor_command_pwm(vrx_pos, vry_pos)
        duty_l  = self.adjust_pwm_value(duty_l)
        duty_r = self.adjust_pwm_value(duty_r)
        print(f"duty_r : {duty_r:.2f}, duty_l : {duty_l:.2f}")
        self.motor.run(duty_r, duty_l)
        return
    
    
    # 速度制御を使ったモータ制御
    def calc_motor_command_vel(self, joystick_x, joystick_y):
        linear_velocity = self.normalize_joystick_input(joystick_y, Control.max_linear_vel)
        angular_velocity = self.normalize_joystick_input(joystick_x, Control.max_angular_vel)
        return linear_velocity, angular_velocity
    
    # 速度制御を使った手動操作
    def manual_control_with_vel(self):
        # Read the joystick position data
        center =  Control.max_joystick_val/2
        vrx_pos = (self.read_analog_pin(Pin.vrx_channel) - center) / (Control.max_joystick_val - center)  # normalize to [-1, 1]
        vry_pos = (self.read_analog_pin(Pin.vry_channel) - center) / (Control.max_joystick_val - center)  
        # Debugging
        print(f"Normalized X : {vrx_pos:.2f}, Normalized Y : {vry_pos:.2f}")
        v_target, w_target = self.calc_motor_command_vel(vrx_pos, vry_pos)
        print(f"lin_vel : {v_target:.2f}, ang_vel : {w_target:.2f}")
        rotation_speed_r, rotation_speed_l = self.motor.calc_motor_speed(v_target, w_target)
        duty_r, duty_l = Motor.cal_duty_brushless(rotation_speed_r, rotation_speed_l)
        print(f"duty_r : {duty_r:.2f}, duty_l : {duty_l:.2f}")
        self.motor.run(duty_r, duty_l)
        return
    

    # 追従操作
    def calculate_speed(self, x, y, max_vel=Control.max_linear_vel, max_angular_vel=Control.max_angular_vel):
        # x座標を0 ~ 1.2の範囲にクリッピング
        x = max(0, min(x, 1.2))
        # y座標を-1.2 ~ 1.2の範囲にクリッピング
        y = max(-1.2, min(y, 1.2))
        # 前後方向の速度 (x 座標に基づく)
        forward_speed = x / 1.2 * max_vel
        # 左右方向の速度 (y 座標に基づく)
        turn_speed = y / 1.2 * max_angular_vel
        return forward_speed, turn_speed

    def follow_control(self):
        vel, w = self.calculate_speed(self.follow_target_person.pose.position.x-0.15, self.follow_target_person.pose.position.y)
        if hasattr(self, 'follow_target_person') and self.follow_target_person is not None:
            print(f"vel, w = {vel}, {w}", flush=True)
            # モータ制御コード...
            self.motor.run(vel, w, a_target=Control.a_target, alpha_target=Control.alpha_target)
        else:
            self.logger.warning('No follow target person data available.')
        return

    def start(self):
        while(rclpy.ok()):
            if self.mode == "emergency": 
                self.logger.info("緊急停止")
            elif self.mode == "follow":
                self.follow_control()
            else:
                self.manual_control_with_pwm()
            rclpy.spin_once(self, timeout_sec=0.1)

def main():
    rclpy.init()
    node = TangController()
    node.start()
    node.destroy_node()
    rclpy.shutdown()
                
if __name__ == '__main__':
    main()