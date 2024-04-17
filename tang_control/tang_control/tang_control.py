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
from config import Pin, PWM, FOLLOWPID, HumanFollowParam, Control
# import pigpio

class TangController(Node):
    def __init__(self):
        super().__init__('tang_control')
    
    def start(self):
        if self.pi.read(Pin.emergency_mode): 
            print(f"緊急停止モード")
            return
        elif self.tang_teleop.mode == 0:
            print(f"遠隔操作")
            self.teleop_control()
        elif self.tang_teleop.mode == 1:
            print(f"追従モード")
            self.follow_control()
        self.prev_mode = self.tang_teleop.mode 
        return 

def main():
    rclpy.init()
    node = TangController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
                
if __name__ == '__main__':
    main()