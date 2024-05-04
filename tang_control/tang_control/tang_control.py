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
        self.motor_l = PhaseEnableMotor(phase=17, enable=12)
        self.motor_r = PhaseEnableMotor(phase=18, enable=13)
        self.mode = None
        self.led = LED(Pin.led_follow)
    
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

    def manual_control(self):
        self.led.off()
        # Read the joystick position data
        vrx_pos = self.read_analog_pin(Pin.vrx_channel) / Control.max_joystick_val * 2 - 1  # normalize to [-1, 1]
        vry_pos = self.read_analog_pin(Pin.vry_channel) / Control.max_joystick_val * 2 - 1  
        # Debugging
        print(f"Normalized X : {vrx_pos:.2f}, Normalized Y : {vry_pos:.2f}")
        self.motor_r.forward(0.2)
        self.motor_l.forward(0.2)
        return
    
    def follow_control(self):
        self.led.blink(on_time=1, off_time=1)
        return

    def start(self):
        while(True):
            if self.mode == "emergency": 
                self.logger.info("緊急停止")
            elif self.mode == "follow":
                self.follow_control()
            else:
                self.manual_control()

def main():
    rclpy.init()
    node = TangController()
    node.start()
    node.destroy_node()
    rclpy.shutdown()
                
if __name__ == '__main__':
    main()