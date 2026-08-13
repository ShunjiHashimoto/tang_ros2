#!/usr/bin/python3
"""TANG mode and speed switch dry-run. No motor commands are sent."""

import time

from gpiozero import Button, LED


FOLLOW_BUTTON_PIN = 16
MANUAL_BUTTON_PIN = 21
MODE_LED_PIN = 14

LOW_SPEED_BUTTON_PIN = 3
HIGH_SPEED_BUTTON_PIN = 4
LOW_SPEED_LED_PIN = 25
HIGH_SPEED_LED_PIN = 26

LOOP_PERIOD_SEC = 0.01
DEBOUNCE_SEC = 0.05


def select_speed_mode(
    current_mode,
    low_pressed,
    high_pressed,
    previous_low_pressed,
    previous_high_pressed,
):
    low_was_pressed = low_pressed and not previous_low_pressed
    high_was_pressed = high_pressed and not previous_high_pressed

    # Ignore simultaneous presses and retain the current speed mode.
    if low_pressed and high_pressed:
        return current_mode
    if low_was_pressed:
        return "LOW"
    if high_was_pressed:
        return "HIGH"
    return current_mode


def debounce_input(raw, candidate, candidate_since, stable, now):
    if raw != candidate:
        candidate = raw
        candidate_since = now
    elif raw != stable and now - candidate_since >= DEBOUNCE_SEC:
        stable = raw
    return candidate, candidate_since, stable


def set_speed_leds(mode, low_led, high_led):
    if mode == "LOW":
        low_led.on()
        high_led.off()
    else:
        low_led.off()
        high_led.on()


def main():
    follow_button = Button(FOLLOW_BUTTON_PIN, pull_up=True, bounce_time=0.05)
    manual_button = Button(MANUAL_BUTTON_PIN, pull_up=True, bounce_time=0.05)
    red_led = LED(MODE_LED_PIN, initial_value=False)
    low_speed_button = Button(
        LOW_SPEED_BUTTON_PIN,
        pull_up=True,
        bounce_time=None,
    )
    high_speed_button = Button(
        HIGH_SPEED_BUTTON_PIN,
        pull_up=True,
        bounce_time=None,
    )
    low_speed_led = LED(LOW_SPEED_LED_PIN, initial_value=False)
    high_speed_led = LED(HIGH_SPEED_LED_PIN, initial_value=False)

    def switch_to_follow():
        red_led.on()
        print("FOLLOW mode: red LED on")

    def switch_to_manual():
        red_led.off()
        print("MANUAL mode: red LED off")

    follow_button.when_pressed = switch_to_follow
    manual_button.when_pressed = switch_to_manual

    speed_mode = "LOW"
    started_at = time.monotonic()
    low_candidate = low_speed_button.is_pressed
    high_candidate = high_speed_button.is_pressed
    low_candidate_since = started_at
    high_candidate_since = started_at
    low_pressed = False
    high_pressed = False
    previous_low_pressed = False
    previous_high_pressed = False

    set_speed_leds(speed_mode, low_speed_led, high_speed_led)
    print("MODE AND SPEED SWITCH DRY-RUN (no motor commands).")
    print("Mode: GPIO16 LOW=FOLLOW, GPIO21 LOW=MANUAL")
    print("Speed: GPIO3 LOW=LOW, GPIO4 LOW=HIGH, both LOW=ignored")
    print(f"Startup speed: {speed_mode}")

    try:
        while True:
            now = time.monotonic()
            low_candidate, low_candidate_since, low_pressed = debounce_input(
                low_speed_button.is_pressed,
                low_candidate,
                low_candidate_since,
                low_pressed,
                now,
            )
            high_candidate, high_candidate_since, high_pressed = debounce_input(
                high_speed_button.is_pressed,
                high_candidate,
                high_candidate_since,
                high_pressed,
                now,
            )

            selected_speed = select_speed_mode(
                speed_mode,
                low_pressed,
                high_pressed,
                previous_low_pressed,
                previous_high_pressed,
            )
            if selected_speed != speed_mode:
                previous_speed = speed_mode
                speed_mode = selected_speed
                print(
                    f"Speed: {previous_speed} -> {speed_mode}",
                    flush=True,
                )

            previous_low_pressed = low_pressed
            previous_high_pressed = high_pressed
            set_speed_leds(speed_mode, low_speed_led, high_speed_led)
            time.sleep(LOOP_PERIOD_SEC)
    except KeyboardInterrupt:
        print("\nMode and speed switch test stopped.")
    finally:
        red_led.off()
        low_speed_led.off()
        high_speed_led.off()
        follow_button.close()
        manual_button.close()
        low_speed_button.close()
        high_speed_button.close()
        red_led.close()
        low_speed_led.close()
        high_speed_led.close()


if __name__ == "__main__":
    main()
