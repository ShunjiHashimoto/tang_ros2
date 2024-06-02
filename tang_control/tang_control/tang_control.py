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
from gpiozero import Button, PhaseEnableMotor, LED
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
        self.motor_r = PhaseEnableMotor(phase=Pin.direction_r, enable=Pin.pwm_r)
        self.motor_l = PhaseEnableMotor(phase=Pin.direction_l, enable=Pin.pwm_l)
        self.mode = "follow"
        self.led = LED(Pin.led_follow)
        self.follow_target_person = Person()
        
        self.subscription = self.create_subscription(Person, 'follow_target_person', self.follow_target_callback, 10)
        
    def follow_target_callback(self, msg):
        print(f"person x: {msg.pose.position.x}, y: {msg.pose.position.y}", flush=True)
        self.follow_target_person = msg
        # -1.2 ~ 1.2 でyは変わる、またxも0 ~ 1.2で変わる
        # pwmは0.3をmaxとする
        
    def switch_on_callback_follow(self):
        self.mode = "follow"
        self.logger.info("追従モード")

    def switch_on_callback_manual(self):
        self.mode = "manual"
        self.logger.info("手動操作")
    
    def read_analog_pin(self, channel):
        adc = spi.xfer2([1, (8 + channel)<<4, 0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data
    
    def normalize_joystick_input(self, value, max_output=Control.max_duty):
        return value*max_output
    
    def calc_motor_command(self, joystick_x, joystick_y):
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
        # print(f"right; {right_motor_pwm} ; left: {left_motor_pwm}")
        left_motor_pwm = max(min(left_motor_pwm, Control.max_duty), -Control.max_duty)
        right_motor_pwm = max(min(right_motor_pwm, Control.max_duty), -Control.max_duty)
        return left_motor_pwm, right_motor_pwm

    def adjust_pwm_value(self, pwm):
        """PWM値が-0.05から0.05の間にある場合、0に調整する"""
        if -0.05 <= pwm <= 0.05:
            return 0
        return pwm

    def manual_control(self):
        self.led.off()
        # Read the joystick position data
        vrx_pos = self.read_analog_pin(Pin.vrx_channel) / Control.max_joystick_val * 2 - 1  # normalize to [-1, 1]
        vry_pos = self.read_analog_pin(Pin.vry_channel) / Control.max_joystick_val * 2 - 1  
        # Debugging
        # print(f"Normalized X : {vrx_pos:.2f}, Normalized Y : {vry_pos:.2f}")
        left_pwm, right_pwm = self.calc_motor_command(vrx_pos, vry_pos)
        left_pwm = self.adjust_pwm_value(left_pwm)
        right_pwm = self.adjust_pwm_value(right_pwm)
        # print(f"left pwm : {left_pwm:.2f}, right pwm : {right_pwm:.2f}")
        if right_pwm > 0:
            self.motor_r.forward(right_pwm)
        else:
            self.motor_r.backward(abs(right_pwm)) 
        if left_pwm > 0:
            self.motor_l.forward(left_pwm)
        else:
            self.motor_l.backward(abs(left_pwm)) 
        return
    
    def calculate_pwm_duty(self, x, y, max_duty=0.2):
        # x座標を0 ~ 1.2の範囲にクリッピング
        x = max(0, min(x, 1.2))
        
        # y座標を-1.2 ~ 1.2の範囲にクリッピング
        y = max(-1.2, min(y, 1.2))

        # 前後方向の速度 (x 座標に基づく)
        forward_speed = x / 1.2 * max_duty
        
        # 左右方向の速度 (y 座標に基づく)
        turn_speed = y / 1.2 * max_duty

        # 左右モータのPWMデューティ比を計算
        left_motor_pwm = forward_speed - turn_speed
        right_motor_pwm = forward_speed + turn_speed

        # PWMデューティ比を-0.3 ~ 0.3の範囲にクリッピング
        left_motor_pwm = max(0, min(left_motor_pwm, max_duty))
        right_motor_pwm = max(0, min(right_motor_pwm, max_duty))

        return left_motor_pwm, right_motor_pwm

    def follow_control(self):
        duty_l, duty_r = self.calculate_pwm_duty(self.follow_target_person.pose.position.x, self.follow_target_person.pose.position.y)
        if hasattr(self, 'follow_target_person') and self.follow_target_person is not None:
            print(f"duty_l, duty_r = {duty_l}, {duty_r}", flush=True)
            # モータ制御コード...
            self.motor_r.forward(duty_r)
            self.motor_l.forward(duty_l)
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
                self.manual_control()
            rclpy.spin_once(self, timeout_sec=0.1)

def main():
    rclpy.init()
    node = TangController()
    node.start()
    node.destroy_node()
    rclpy.shutdown()
                
if __name__ == '__main__':
    main()