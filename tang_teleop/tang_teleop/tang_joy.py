# joystickを使ってモータの回転テスト
import rclpy
from rclpy.node import Node

import sys
sys.path.append("..")
from tang_control.tang_control.config import Pin, Control 
import spidev
import gpiozero

class TangJoystickController():
    def __init__(self):
        self.enable_spi = True
        # spi settings
        try: 
            self.spi = spidev.SpiDev()
            self.spi.open(0,0)
            self.spi.max_speed_hz = 100000 
        except:
            self.enable_spi = False
            print("error: failed to open spi")
        
    def read_analog_pin(self, channel):
        adc = self.spi.xfer2([1, (8 + channel)<<4, 0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data
    
    def manual_control(self):
        # Read the joystick position data
        vrx_pos = self.read_analog_pin(Pin.vrx_channel) / Control.max_joystick_val * 2 - 1  # normalize to [-1, 1]
        vry_pos = self.read_analog_pin(Pin.vry_channel) / Control.max_joystick_val * 2 - 1  
        # Debugging
        print(f"Normalized X : {vrx_pos}, Normalized Y : {vry_pos}")
        # Calculate linear and angular velocity
        linear_velocity = Control.max_linear_vel * max(vry_pos, 0)  # vry_pos negative would mean backward, but we restrict that
        angular_velocity = vrx_pos * Control.max_angular_vel
        # Set velocities to zero if they are below the threshold
        if abs(linear_velocity) < Control.velocity_thresh:
            linear_velocity = 0
        if abs(angular_velocity) < Control.velocity_thresh:
            angular_velocity = 0
        print(f"linear_velocity{linear_velocity}, angular_velocity{angular_velocity}")
        # self.motor.run(linear_velocity, angular_velocity)
        return

def main():
    joy_controller = TangJoystickController()
    while(joy_controller.enable_spi):
        joy_controller.manual_control()
        
if __name__ == '__main__':
    main()