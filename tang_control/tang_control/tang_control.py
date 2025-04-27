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

from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist
from tang_control.config import Pin, PWM, FOLLOWPID, HumanFollowParam, Control, LiDARParam
from tang_control.motor import Motor
from gpiozero import Button, LED 
import spidev
import math

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
        self.led = LED(Pin.red_led)
        self.buzzer = LED(Pin.buzzer)
        self.mode = "manual"
        self.obstacle_near = False
        # LiDARデータのサブスクライブ
        self.lidar_subscription = self.create_subscription(LaserScan,'/scan',self.lidar_callback,10)
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.threshold_distance = 0.3
        # joyトピック 
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        
    def lidar_callback(self, msg):
        # LiDARの点群データをチェック
        self.obstacle_near = any(r < self.threshold_distance for r in msg.ranges)

    def cmd_vel_callback(self, cmd_vel):
        duty_l, duty_r = self.convert_cmdvel_to_duty(cmd_vel)
        print(f"Received cmd_vel: linear.x={cmd_vel.linear.x}, angular.z={cmd_vel.angular.z}", flush=True)
        print(f"duty_l : {duty_l:.2f}, duty_r : {duty_r:.2f}", flush=True)
        self.motor.run(duty_r, duty_l)
    
    # joystickのボタンを押したときのコールバック関数
    def publish_joy(self, button_index):
        msg = Joy()
        msg.axes = [0.0] * 8 
        msg.buttons = [0] * 12
        msg.buttons[button_index] = 1
        self.joy_pub.publish(msg)

    def convert_cmdvel_to_duty(self, cmd_vel):
        # cmd_velからモータのデューティ比を計算する
        v_l = cmd_vel.linear.x - (LiDARParam.inverted * cmd_vel.angular.z) * (Control.tread_width/2) 
        v_r = cmd_vel.linear.x + (LiDARParam.inverted * cmd_vel.angular.z)  * (Control.tread_width/2)
        # 車輪の回転数に変換
        wheel_rpm_l = v_l / (2 * math.pi * Control.wheel_radius) * 60
        wheel_rpm_r = v_r / (2 * math.pi * Control.wheel_radius) * 60
        # モータの回転数に変換
        motor_rpm_l = wheel_rpm_l * Control.gear_ratio
        motor_rpm_r = wheel_rpm_r * Control.gear_ratio
        print(f"motor_rpm_l : {motor_rpm_l:.2f}, motor_rpm_r : {motor_rpm_r:.2f}", flush=True)
        # デューティ比に変換
        duty_l = (motor_rpm_l / Control.max_motor_rpm) * PWM.max_duty_follow
        duty_r = (motor_rpm_r / Control.max_motor_rpm) * PWM.max_duty_follow
        return duty_l, duty_r
       
    # モード切替 
    def switch_on_callback_follow(self):
        self.logger.info("追従モード")
        self.buzzer.on()
        self.mode = "follow"
        self.publish_joy(Pin.unlock_emergency_button) 
        self.publish_joy(Pin.followme_start_button) 

    def switch_on_callback_manual(self):
        self.logger.info("手動操作")
        self.buzzer.on()
        self.mode = "manual"
        self.publish_joy(Pin.emergency_button) 
        self.publish_joy(Pin.followme_stop_button) 
    
    # joystick信号の取得
    def read_analog_pin(self, channel):
        adc = spi.xfer2([1, (8 + channel)<<4, 0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data
    
    # pwmを使った手動操作
    def manual_pwm_control(self):
        # xが前後方向、マイナスなら後ろ、プラスなら前
        # yがプラスなら左モータ、マイナスなら右モータを回す
        self.buzzer.off()
        # Read the joystick position data
        # 前後方向
        vry_pos = self.read_analog_pin(Pin.vrx_channel) / Control.max_joystick_val*2 - 1  # normalize to [-1, 1]
        # 左右方向
        vrx_pos = self.read_analog_pin(Pin.vry_channel) / Control.max_joystick_val*2 - 1   
        print(f"Normalized joystick position X : {vrx_pos:.2f}, Normalized Y : {vry_pos:.2f}")
        duty_r, duty_l = self.motor.calc_duty_by_joyinput(vrx_pos, vry_pos)
        #print(f"duty_r : {duty_r:.2f}, duty_l : {duty_l:.2f}")
        self.motor.run(duty_r, duty_l)
        return
    
    # 追従操作
    def calc_vw_by_humanpos(self, x, y, max_target_v=Control.max_target_v, max_target_w=Control.max_target_w):
        # x座標を0 ~ 1.2の範囲にクリッピング
        x = max(0, min(x, 1.2))
        # y座標を-1.2 ~ 1.2の範囲にクリッピング
        y = max(-1.2, min(y, 1.2))
        # TODO: 角度が大きければ先にターンする
        angle_radians = math.atan2(y, x)
        angle_degrees = math.degrees(angle_radians)
        print(f"Angle in degrees: {angle_degrees}")
        if(abs(angle_degrees) > 60): 
            target_v = 0.01
            target_w = max_target_w + 1.0 if angle_degrees>0 else -max_target_w-1.0
            print("max_angle")
        else:
            # 前後方向の速度 (x 座標に基づく)
            target_v = x / 1.2 * max_target_v
            # 左右方向の速度 (y 座標に基づく)
            target_w = y / 1.2 * max_target_w
        return target_v, target_w

    def follow_control(self):
        self.buzzer.off()
        return

    def start(self):
        while(rclpy.ok()):
            if self.mode == "emergency" or self.obstacle_near: 
                self.motor.stop()
                self.logger.info("緊急停止")
                self.mode = "manual"
            elif self.mode == "follow":
                self.follow_control()
            elif self.mode == "manual":
                self.manual_pwm_control()
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
