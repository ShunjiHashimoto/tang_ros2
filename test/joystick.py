import sys
import time
sys.path.append("..")
from tang_control.config import Control, JoyParam, Pin
import spidev

try: 
    spi = spidev.SpiDev()
    spi.open(0,0)
    spi.max_speed_hz = 100000 
except:
    print("error: failed to open spi")
        
def read_analog_pin(channel):
    adc = spi.xfer2([1, (8 + channel)<<4, 0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data

def manual_control():
    # Read all MCP3004 channels so wiring/channel assignments can be checked.
    raw = [read_analog_pin(channel) for channel in range(4)]
    vrx_pos = raw[Pin.vrx_channel] / JoyParam.max_joystick_val * 2 - 1
    vry_pos = raw[Pin.vry_channel] / JoyParam.max_joystick_val * 2 - 1
    # Calculate linear and angular velocity
    linear_velocity = Control.max_target_v * max(vry_pos, 0)  # vry_pos negative would mean backward, but we restrict that
    angular_velocity = vrx_pos * Control.max_target_w
    # Set velocities to zero if they are below the threshold
    if abs(linear_velocity) < Control.velocity_thresh:
        linear_velocity = 0
    if abs(angular_velocity) < Control.velocity_thresh:
        angular_velocity = 0
    print(
        f"CH0={raw[0]:4d} CH1={raw[1]:4d} "
        f"CH2={raw[2]:4d} CH3={raw[3]:4d} | "
        f"X={vrx_pos:+.3f} Y={vry_pos:+.3f} | "
        f"v={linear_velocity:+.3f} w={angular_velocity:+.3f}"
    )
    return

while(True):
    manual_control()
    time.sleep(0.2)
