#!/usr/bin/python3

import gpiod
import time

PIN_NO = 12
chip = gpiod.Chip('gpiochip4', gpiod.Chip.OPEN_BY_NAME)
line = chip.get_line(PIN_NO)
line.request(consumer="LED", type=gpiod.LINE_REQ_DIR_OUT)

try:
    while True:
        line.set_value(1)
        time.sleep(1)
        line.set_value(0)
        time.sleep(1)
finally:
    line.release()