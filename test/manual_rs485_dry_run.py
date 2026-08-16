#!/usr/bin/env python3
"""Physical TANG joystick to CuGoV4 motor-command dry-run.

This script reads the real MCP3004 joystick and speed buttons, but constructs
Rs485DualMotorBridge with dry_run=True. It never opens or writes to RS-485.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import spidev
from gpiozero import Button, LED


SRC_DIR = Path(__file__).resolve().parents[2]
CUGO_SCRIPTS_DIR = SRC_DIR / "cugo_rs485_motor_control" / "scripts"
if str(CUGO_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(CUGO_SCRIPTS_DIR))

from propo_control import MotorBridgeConfig, Rs485DualMotorBridge


# Values measured on the physical TANG joystick.
STEERING_MIN = 70
STEERING_CENTER = 500
STEERING_MAX = 960
THROTTLE_MIN = 50
THROTTLE_CENTER = 500
THROTTLE_MAX = 960
THROTTLE_SIGN = -1.0
STEERING_SIGN = 1.0
ADC_DEADBAND = 35

STEERING_CHANNEL = 0
THROTTLE_CHANNEL = 1

LOW_SPEED_BUTTON_PIN = 3
HIGH_SPEED_BUTTON_PIN = 4
LOW_SPEED_LED_PIN = 25
HIGH_SPEED_LED_PIN = 26

LOW_MAX_V_MPS = 0.15
LOW_MAX_W_RADPS = 0.6
HIGH_MAX_V_MPS = 0.30
HIGH_MAX_W_RADPS = 1.0

LOOP_PERIOD_SEC = 0.05
PRINT_PERIOD_SEC = 0.20


def normalize_axis(raw: int, minimum: int, center: int, maximum: int) -> float:
    """Normalize an asymmetric ADC range to -1..1 around a measured center."""
    offset = raw - center
    if abs(offset) <= ADC_DEADBAND:
        return 0.0
    if offset > 0:
        value = offset / float(maximum - center)
    else:
        value = offset / float(center - minimum)
    return max(-1.0, min(1.0, value))


def read_adc(spi: spidev.SpiDev, channel: int) -> int:
    response = spi.xfer2([1, (8 + channel) << 4, 0])
    return ((response[1] & 3) << 8) | response[2]


def update_speed_mode(
    current_mode: str,
    low_button: Button,
    high_button: Button,
) -> str:
    low_pressed = low_button.is_pressed
    high_pressed = high_button.is_pressed
    # Ignore simultaneous presses and retain the current mode.
    if low_pressed and high_pressed:
        return current_mode
    if low_pressed:
        return "LOW"
    if high_pressed:
        return "HIGH"
    return current_mode


def set_speed_leds(mode: str, low_led: LED, high_led: LED) -> None:
    if mode == "LOW":
        low_led.on()
        high_led.off()
    else:
        low_led.off()
        high_led.on()


def main() -> int:
    spi = spidev.SpiDev()
    low_button = Button(LOW_SPEED_BUTTON_PIN, pull_up=True, bounce_time=0.04)
    high_button = Button(HIGH_SPEED_BUTTON_PIN, pull_up=True, bounce_time=0.04)
    low_led = LED(LOW_SPEED_LED_PIN)
    high_led = LED(HIGH_SPEED_LED_PIN)
    bridge = None

    try:
        spi.open(0, 0)
        spi.max_speed_hz = 100000

        bridge = Rs485DualMotorBridge(
            port="/dev/ttyUSB0",
            baudrate=9600,
            timeout=0.3,
            left_slave=2,
            right_slave=1,
            config=MotorBridgeConfig(
                op_no=2,
                wheel_radius_left=0.03858,
                wheel_radius_right=0.03858,
                tread=0.376,
                reduction_ratio=20.0,
                max_rpm=2600.0,
                min_rpm=80.0,
                anti_creep_start_rpm=120.0,
                left_motor_sign=-1,
                right_motor_sign=1,
            ),
            dry_run=True,
        )

        speed_mode = "LOW"
        set_speed_leds(speed_mode, low_led, high_led)
        next_print = 0.0
        print("MANUAL RS-485 DRY-RUN (no serial output). Ctrl+C to stop.")
        print("GPIO3=LOW, GPIO4=HIGH, simultaneous press=ignored")

        while True:
            started_at = time.monotonic()
            speed_mode = update_speed_mode(speed_mode, low_button, high_button)
            set_speed_leds(speed_mode, low_led, high_led)

            raw_steering = read_adc(spi, STEERING_CHANNEL)
            raw_throttle = read_adc(spi, THROTTLE_CHANNEL)
            raw_x = normalize_axis(
                raw_steering,
                STEERING_MIN,
                STEERING_CENTER,
                STEERING_MAX,
            )
            # The joystick is mounted front-to-back in the reversed direction.
            # Apply the sign here so robot-forward is positive ROS linear.x.
            throttle = THROTTLE_SIGN * normalize_axis(
                raw_throttle,
                THROTTLE_MIN,
                THROTTLE_CENTER,
                THROTTLE_MAX,
            )

            # After the mounting change, CH0 increases toward robot-left
            # (west). ROS positive angular velocity is also left/counter-clockwise.
            steering = STEERING_SIGN * raw_x
            if speed_mode == "LOW":
                max_v = LOW_MAX_V_MPS
                max_w = LOW_MAX_W_RADPS
            else:
                max_v = HIGH_MAX_V_MPS
                max_w = HIGH_MAX_W_RADPS

            requested_v = throttle * max_v
            requested_w = steering * max_w
            limited_v, limited_w, left_rpm, right_rpm = bridge.apply_body_velocity(
                requested_v,
                requested_w,
            )

            now = time.monotonic()
            if now >= next_print:
                print(
                    f"{speed_mode:4s} "
                    f"CH0={raw_steering:4d} CH1={raw_throttle:4d} | "
                    f"throttle={throttle:+.3f} steering={steering:+.3f} | "
                    f"v={limited_v:+.3f} m/s w={limited_w:+.3f} rad/s | "
                    f"left={left_rpm:+7.0f} rpm right={right_rpm:+7.0f} rpm",
                    flush=True,
                )
                next_print = now + PRINT_PERIOD_SEC

            remaining = LOOP_PERIOD_SEC - (time.monotonic() - started_at)
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nStopping dry-run.")
        return 0
    finally:
        if bridge is not None:
            bridge.stop(force=True)
            bridge.close()
        low_led.off()
        high_led.off()
        spi.close()
        low_button.close()
        high_button.close()
        low_led.close()
        high_led.close()


if __name__ == "__main__":
    raise SystemExit(main())
