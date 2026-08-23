#!/usr/bin/env python3
"""ハードウェアを使わずにTANG統合制御の基本ロジックを確認する。"""

import math
import unittest

from cugo_rs485_motor_control.bridge import (
    MotorBridgeConfig,
    Rs485DualMotorBridge,
)
from tang_control.config import Control, LiDARParam
from tang_control.controller_core import (
    FOLLOW,
    HIGH,
    IDLE,
    LOW,
    MANUAL,
    TangControlState,
    TangControlRuntime,
    joystick_is_active,
    joystick_to_body_velocity,
    limit_follow_velocity,
    limit_follow_acceleration,
    normalize_axis,
    scan_contains_nearby_obstacle,
    smooth_command,
)


class TangControlStateTest(unittest.TestCase):
    def test_startup_is_idle_and_low(self):
        state = TangControlState()
        self.assertEqual(IDLE, state.mode)
        self.assertEqual(LOW, state.speed_mode)

    def test_manual_transition_resets_speed_to_low(self):
        state = TangControlState(mode=FOLLOW, speed_mode=HIGH)
        self.assertTrue(state.select_mode(MANUAL))
        self.assertEqual(MANUAL, state.mode)
        self.assertEqual(LOW, state.speed_mode)

    def test_follow_transition_resets_speed_to_low(self):
        state = TangControlState(mode=MANUAL, speed_mode=HIGH)
        self.assertTrue(state.select_mode(FOLLOW))
        self.assertEqual(FOLLOW, state.mode)
        self.assertEqual(LOW, state.speed_mode)

    def test_reselecting_current_mode_is_ignored(self):
        state = TangControlState(mode=MANUAL)
        self.assertFalse(state.select_mode(MANUAL))

    def test_speed_buttons_only_act_on_new_press_in_manual(self):
        state = TangControlState()

        # MANUAL以外で押された高速ボタンは記憶するが、速度には反映しない。
        self.assertFalse(state.update_speed_buttons(False, True))
        state.select_mode(MANUAL)
        self.assertFalse(state.update_speed_buttons(False, True))
        self.assertEqual(LOW, state.speed_mode)

        state.update_speed_buttons(False, False)
        self.assertTrue(state.update_speed_buttons(False, True))
        self.assertEqual(HIGH, state.speed_mode)

        state.update_speed_buttons(False, False)
        self.assertTrue(state.update_speed_buttons(True, False))
        self.assertEqual(LOW, state.speed_mode)

    def test_speed_buttons_select_low_and_high_in_follow(self):
        state = TangControlState(mode=FOLLOW, speed_mode=LOW)
        self.assertTrue(state.update_speed_buttons(False, True))
        self.assertEqual(HIGH, state.speed_mode)

        state.update_speed_buttons(False, False)
        self.assertTrue(state.update_speed_buttons(True, False))
        self.assertEqual(LOW, state.speed_mode)

    def test_simultaneous_speed_press_is_ignored(self):
        state = TangControlState(mode=MANUAL, speed_mode=HIGH)
        self.assertFalse(state.update_speed_buttons(True, True))
        self.assertEqual(HIGH, state.speed_mode)

    def test_single_button_after_simultaneous_press_is_accepted(self):
        state = TangControlState(mode=MANUAL, speed_mode=LOW)
        self.assertFalse(state.update_speed_buttons(True, True))
        self.assertTrue(state.update_speed_buttons(False, True))
        self.assertEqual(HIGH, state.speed_mode)

    def test_speed_button_baseline_does_not_select_speed(self):
        state = TangControlState(mode=MANUAL, speed_mode=LOW)
        state.remember_speed_buttons(False, True)
        self.assertFalse(state.update_speed_buttons(False, True))
        self.assertEqual(LOW, state.speed_mode)


class JoystickConversionTest(unittest.TestCase):
    def test_center_and_deadband_are_zero(self):
        self.assertEqual(0.0, normalize_axis(500, 70, 500, 960))
        self.assertEqual(0.0, normalize_axis(535, 70, 500, 960))
        self.assertEqual(0.0, normalize_axis(465, 70, 500, 960))
        self.assertEqual((0.0, 0.0), joystick_to_body_velocity(500, 500, LOW))

    def test_low_and_high_limits(self):
        # 取付変更後はCH1最小側が前進、CH0最大側が西向き旋回になる。
        low_v, low_w = joystick_to_body_velocity(960, 50, LOW)
        high_v, high_w = joystick_to_body_velocity(960, 50, HIGH)
        self.assertAlmostEqual(Control.manual_low_max_v_mps, low_v)
        self.assertAlmostEqual(Control.manual_low_max_w_radps, low_w)
        self.assertAlmostEqual(Control.manual_high_max_v_mps, high_v)
        self.assertAlmostEqual(Control.manual_high_max_w_radps, high_w)

    def test_reverse_input_is_negative_after_mounting_change(self):
        low_v, _ = joystick_to_body_velocity(500, 960, LOW)
        high_v, _ = joystick_to_body_velocity(500, 960, HIGH)
        self.assertAlmostEqual(-Control.manual_low_max_v_mps, low_v)
        self.assertAlmostEqual(-Control.manual_high_max_v_mps, high_v)

    def test_east_input_is_negative_angular_velocity_after_mounting_change(self):
        _, low_w = joystick_to_body_velocity(70, 500, LOW)
        _, high_w = joystick_to_body_velocity(70, 500, HIGH)
        self.assertAlmostEqual(-Control.manual_low_max_w_radps, low_w)
        self.assertAlmostEqual(-Control.manual_high_max_w_radps, high_w)

    def test_axes_are_clamped(self):
        v, w = joystick_to_body_velocity(2000, -100, HIGH)
        self.assertAlmostEqual(Control.manual_high_max_v_mps, v)
        self.assertAlmostEqual(Control.manual_high_max_w_radps, w)

    def test_activity_uses_the_same_center_deadband_as_manual_control(self):
        self.assertFalse(joystick_is_active(465, 535))
        self.assertTrue(joystick_is_active(464, 500))
        self.assertTrue(joystick_is_active(500, 536))


class ObstacleDetectionTest(unittest.TestCase):
    def test_lidar_and_front_extent_match_tang_measurement(self):
        self.assertAlmostEqual(0.32, LiDARParam.position_x_m)
        self.assertAlmostEqual(0.32, LiDARParam.body_front_length_m)

    def scan_with_point(self, x, y, **kwargs):
        distance = (x * x + y * y) ** 0.5
        angle = math.atan2(y, x)
        return scan_contains_nearby_obstacle(
            [distance],
            angle,
            0.0,
            0.05,
            10.0,
            **kwargs,
        )

    def test_front_clearance_is_measured_from_body_front(self):
        self.assertTrue(self.scan_with_point(0.30, 0.0))
        self.assertFalse(self.scan_with_point(0.301, 0.0))

    def test_side_clearance_uses_body_half_width(self):
        self.assertTrue(self.scan_with_point(0.0, 0.35))
        self.assertFalse(self.scan_with_point(0.0, 0.351))

    def test_front_and_side_clearances_define_corner(self):
        self.assertTrue(self.scan_with_point(0.30, 0.35))
        self.assertFalse(self.scan_with_point(0.301, 0.35))
        self.assertFalse(self.scan_with_point(0.30, 0.351))

    def test_manual_clearance_is_five_centimeters(self):
        kwargs = {
            "front_clearance_m": LiDARParam.manual_obstacle_front_clearance_m,
            "side_clearance_m": LiDARParam.manual_obstacle_side_clearance_m,
        }
        self.assertTrue(self.scan_with_point(0.05, 0.0, **kwargs))
        self.assertFalse(self.scan_with_point(0.051, 0.0, **kwargs))
        self.assertTrue(self.scan_with_point(0.0, 0.30, **kwargs))
        self.assertFalse(self.scan_with_point(0.0, 0.301, **kwargs))

    def test_invalid_and_out_of_range_measurements_are_ignored(self):
        self.assertFalse(scan_contains_nearby_obstacle(
            [float("nan"), float("inf"), 0.01],
            0.0,
            0.1,
            0.05,
            10.0,
        ))


class FollowVelocityTest(unittest.TestCase):
    def test_follow_turn_limits_are_centralized_in_config(self):
        self.assertAlmostEqual(
            math.radians(15.0),
            Control.follow_normal_max_w_radps,
        )
        self.assertAlmostEqual(
            math.radians(45.0),
            Control.follow_extreme_angle_rad,
        )
        self.assertAlmostEqual(
            math.radians(35.0),
            Control.follow_extreme_max_w_radps,
        )
        self.assertEqual(
            Control.follow_extreme_max_w_radps,
            Control.follow_max_w_radps,
        )

    def test_command_ema_matches_tang_follow_smoothing(self):
        self.assertAlmostEqual(0.75, smooth_command(1.0, 0.0))
        self.assertAlmostEqual(0.775, smooth_command(1.0, 0.1))

    def test_follow_low_and_high_linear_limits(self):
        requested_v = Control.follow_high_max_v_mps
        requested_w = 0.20

        low_v, low_w = limit_follow_velocity(requested_v, requested_w, LOW)
        self.assertEqual(Control.follow_low_max_v_mps, low_v)
        self.assertAlmostEqual(
            -requested_w * Control.follow_low_max_v_mps / requested_v,
            low_w,
        )
        self.assertEqual(
            (Control.follow_high_max_v_mps, -requested_w),
            limit_follow_velocity(requested_v, requested_w, HIGH),
        )

        low_v, low_w = limit_follow_velocity(-requested_v, -requested_w, LOW)
        self.assertEqual(-Control.follow_low_max_v_mps, low_v)
        self.assertAlmostEqual(
            requested_w * Control.follow_low_max_v_mps / requested_v,
            low_w,
        )

    def test_follow_low_preserves_curve_radius_and_in_place_turn_limit(self):
        low_v, low_w = limit_follow_velocity(0.30, 0.20, LOW)
        high_v, high_w = limit_follow_velocity(0.30, 0.20, HIGH)
        self.assertAlmostEqual(abs(high_v / high_w), abs(low_v / low_w))
        self.assertEqual(
            (0.0, -Control.follow_max_w_radps),
            limit_follow_velocity(0.0, 10.0, LOW),
        )

    def test_non_finite_follow_velocity_becomes_stop(self):
        self.assertEqual((0.0, 0.0), limit_follow_velocity(float("nan"), 0.1))
        self.assertEqual((0.0, 0.0), limit_follow_velocity(0.1, float("inf")))

    def test_follow_acceleration_is_limited_but_deceleration_is_immediate(self):
        self.assertAlmostEqual(
            0.1,
            limit_follow_acceleration(0.15, 0.0, 0.1),
        )
        self.assertEqual(0.0, limit_follow_acceleration(0.0, 0.15, 0.1))
        self.assertEqual(0.0, limit_follow_acceleration(-0.15, 0.10, 0.1))


class ProvenBridgeIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.bridge = Rs485DualMotorBridge(
            port="/dev/ttyUSB0",
            baudrate=9600,
            timeout=0.3,
            left_slave=2,
            right_slave=1,
            config=MotorBridgeConfig(
                op_no=2,
                wheel_radius_left=Control.wheel_radius_left,
                wheel_radius_right=Control.wheel_radius_right,
                tread=Control.tread,
                reduction_ratio=Control.reduction_ratio,
                max_rpm=Control.rs485_max_motor_rpm,
                min_rpm=Control.rs485_min_motor_rpm,
                anti_creep_start_rpm=Control.anti_creep_start_rpm,
                left_motor_sign=-1,
                right_motor_sign=1,
                deceleration_stop=True,
            ),
            dry_run=True,
        )

    def tearDown(self):
        self.bridge.close()

    def test_forward_command_uses_proven_motor_signs(self):
        _, _, left_rpm, right_rpm = self.bridge.apply_body_velocity(0.15, 0.0)
        self.assertLess(left_rpm, 0.0)
        self.assertGreater(right_rpm, 0.0)
        self.assertAlmostEqual(abs(left_rpm), abs(right_rpm))

    def test_zero_and_small_start_command_stop(self):
        self.bridge.stop(force=True)
        self.assertEqual(
            (0.0, 0.0),
            self.bridge.apply_body_velocity(0.0, 0.0)[2:],
        )
        self.assertEqual(
            (0.0, 0.0),
            self.bridge.apply_body_velocity(0.001, 0.0)[2:],
        )

    def test_motor_rpm_is_limited(self):
        _, _, left_rpm, right_rpm = self.bridge.apply_body_velocity(10.0, 0.0)
        self.assertLessEqual(abs(left_rpm), Control.rs485_max_motor_rpm)
        self.assertLessEqual(abs(right_rpm), Control.rs485_max_motor_rpm)


class FakeBridge:
    def __init__(self):
        self.calls = []

    def stop(self, force=False):
        self.calls.append(("stop", force))

    def apply_body_velocity(self, v, w):
        self.calls.append(("velocity", v, w))
        return v, w, -100.0, 100.0


class TangControlRuntimeTest(unittest.TestCase):
    def test_mode_change_stops_before_selecting_manual(self):
        bridge = FakeBridge()
        runtime = TangControlRuntime(bridge)
        self.assertTrue(runtime.select_mode(MANUAL))
        self.assertEqual([("stop", True)], bridge.calls)
        self.assertEqual(MANUAL, runtime.state.mode)
        self.assertEqual(LOW, runtime.state.speed_mode)

    def test_same_mode_does_not_send_another_stop(self):
        bridge = FakeBridge()
        runtime = TangControlRuntime(
            bridge,
            TangControlState(mode=MANUAL),
        )
        self.assertFalse(runtime.select_mode(MANUAL))
        self.assertEqual([], bridge.calls)

    def test_idle_never_applies_manual_velocity(self):
        bridge = FakeBridge()
        runtime = TangControlRuntime(bridge)
        self.assertIsNone(runtime.apply_manual_input(70, 960))
        self.assertEqual([("stop", False)], bridge.calls)

    def test_manual_applies_limited_joystick_request(self):
        bridge = FakeBridge()
        runtime = TangControlRuntime(
            bridge,
            TangControlState(mode=MANUAL, speed_mode=LOW),
        )
        result = runtime.apply_manual_input(960, 50)
        self.assertAlmostEqual(Control.command_ema_alpha * Control.manual_low_max_v_mps, result[0])
        self.assertAlmostEqual(Control.command_ema_alpha * Control.manual_low_max_w_radps, result[1])
        self.assertEqual("velocity", bridge.calls[0][0])

    def test_follow_applies_limited_cmd_vel(self):
        bridge = FakeBridge()
        runtime = TangControlRuntime(
            bridge,
            TangControlState(mode=FOLLOW),
        )
        result = runtime.apply_follow_input(0.30, -0.20, 0.05)
        self.assertAlmostEqual(Control.follow_accel_limit_mps2 * 0.05, result[0])
        self.assertAlmostEqual(Control.command_ema_alpha * 0.10, result[1])
        self.assertEqual("velocity", bridge.calls[0][0])

        result = runtime.apply_follow_input(0.30, -0.20, 0.05)
        self.assertAlmostEqual(Control.follow_accel_limit_mps2 * 0.10, result[0])

    def test_follow_zero_command_stops_without_ema_delay(self):
        bridge = FakeBridge()
        runtime = TangControlRuntime(bridge, TangControlState(mode=FOLLOW))
        runtime.apply_follow_input(0.15, 0.2, 0.05)

        result = runtime.apply_follow_input(0.0, 0.0, 0.05)

        self.assertEqual((0.0, 0.0), result[:2])
        self.assertEqual(0.0, runtime.follow_smoothed_v_mps)
        self.assertEqual(0.0, runtime.follow_smoothed_w_radps)

    def test_follow_translation_zero_is_immediate_while_turning(self):
        bridge = FakeBridge()
        runtime = TangControlRuntime(bridge, TangControlState(mode=FOLLOW))
        runtime.apply_follow_input(0.15, 0.2, 0.05)

        result = runtime.apply_follow_input(0.0, 0.5, 0.05)

        self.assertEqual(0.0, result[0])
        self.assertLess(result[1], 0.0)

    def test_manual_never_applies_follow_velocity(self):
        bridge = FakeBridge()
        runtime = TangControlRuntime(
            bridge,
            TangControlState(mode=MANUAL),
        )
        self.assertIsNone(runtime.apply_follow_input(0.1, 0.2, 0.05))
        self.assertEqual([("stop", False)], bridge.calls)


if __name__ == "__main__":
    unittest.main()
