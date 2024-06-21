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
        self.follow_target_person = Person()
        
        self.subscription = self.create_subscription(Person, 'follow_target_person', self.follow_target_callback, 10)
        
    def follow_target_callback(self, msg):
        print(f"person x: {msg.pose.position.x}, y: {msg.pose.position.y}", flush=True)
        self.follow_target_person = msg
        # -1.2 ~ 1.2 でyは変わる、またxも0 ~ 1.2で変わる
       
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
    
    # pwmを使った手動操作
    def manual_pwm_control(self):
        # Read the joystick position data
        vrx_pos = self.read_analog_pin(Pin.vrx_channel) / Control.max_joystick_val*2 - 1  # normalize to [-1, 1]
        vry_pos = self.read_analog_pin(Pin.vry_channel) / Control.max_joystick_val*2 - 1   
        print(f"Normalized joystick position X : {vrx_pos:.2f}, Normalized Y : {vry_pos:.2f}")
        duty_r, duty_l = self.motor.calc_duty_by_joyinput(vrx_pos, vry_pos)
        print(f"duty_r : {duty_r:.2f}, duty_l : {duty_l:.2f}")
        self.motor.run(duty_r, duty_l)
        return
    
    # 速度制御を使った手動操作
    def manual_vel_control(self):
        center =  Control.max_joystick_val/2
        vrx_pos = (self.read_analog_pin(Pin.vrx_channel) - center) / (Control.max_joystick_val - center)  # normalize to [-1, 1]
        vry_pos = (self.read_analog_pin(Pin.vry_channel) - center) / (Control.max_joystick_val - center)  
        print(f"Normalized joystick position X : {vrx_pos:.2f}, Normalized Y : {vry_pos:.2f}")
        target_v, target_w = self.motor.calc_robot_vel_command(vrx_pos, vry_pos)
        print(f"target_v : {target_v:.2f}, target_w : {target_w:.2f}")
        duty_r, duty_l = self.motor.calc_duty_by_vw(target_v, target_w)
        print(f"duty_r : {duty_r:.2f}, duty_l : {duty_l:.2f}")
        self.motor.run(duty_r, duty_l)
        return
    
    # 追従操作
    def calc_vw_by_humanpos(self, x, y, max_target_v=Control.max_target_v, max_target_w=Control.max_target_w):
        # x座標を0 ~ 1.2の範囲にクリッピング
        x = max(0, min(x, 1.2))
        # y座標を-1.2 ~ 1.2の範囲にクリッピング
        y = max(-1.2, min(y, 1.2))
        # 前後方向の速度 (x 座標に基づく)
        target_v = x / 1.2 * max_target_v
        # 左右方向の速度 (y 座標に基づく)
        target_w = y / 1.2 * max_target_w
        return target_v, target_w

    def follow_control(self):
        target_v, target_w = self.calc_vw_by_humanpos(self.follow_target_person.pose.position.x-0.15, self.follow_target_person.pose.position.y)
        print(f"target_v : {target_v:.2f}, target_w : {target_w:.2f}")
        if hasattr(self, 'follow_target_person') and self.follow_target_person is not None:
            duty_r, duty_l = self.motor.calc_duty_by_vw(target_v, target_w)
            print(f"duty_r : {duty_r:.2f}, duty_l : {duty_l:.2f}")
            self.motor.run(duty_r, duty_l)
        else:
            self.logger.warning('No follow target person data available.')
        return

    def start(self):
        while(rclpy.ok()):
            if self.mode == "emergency": 
                self.logger.info("緊急停止")
            elif self.mode == "follow":
                self.follow_control()
            elif self.mode == "manual":
                self.manual_pwm_control()
                # self.manual_vel_control()
            else:
                print("Something wrong, Please check curretn mode")
            rclpy.spin_once(self, timeout_sec=0.1)
        print("Shutdown, Motor stopping ...")
        self.motor.stop()

def main():
    rclpy.init()
    node = TangController()
    node.start()
    node.destroy_node()
    rclpy.shutdown()
                
if __name__ == '__main__':
    main()