import sys
sys.path.append("..")
from tang_control.config import Pin, Control 
from gpiozero import LED
import time

buzzer = LED(Pin.buzzer)
try:
    while True:
        buzzer.on()
        print(f"High: {buzzer.value}")
        time.sleep(1.0)
        buzzer.off()
        print(f"stop: {buzzer.value}")
        time.sleep(1.0)
finally:
    buzzer.release()
