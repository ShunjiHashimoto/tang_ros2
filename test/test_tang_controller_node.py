#!/usr/bin/env python3
"""ROSを起動せずにTangControllerの処理順序を確認する。"""

import sys
import types
import unittest


def install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    rclpy.node = rclpy_node
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node

    class Joy:
        def __init__(self):
            self.axes = []
            self.buttons = []

    class LaserScan:
        pass

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0)
            self.angular = types.SimpleNamespace(z=0.0)

    class String:
        def __init__(self):
            self.data = ""

    for package, symbols in (
        ("sensor_msgs", {"Joy": Joy, "LaserScan": LaserScan}),
        ("geometry_msgs", {"Twist": Twist}),
        ("std_msgs", {"String": String}),
    ):
        package_module = types.ModuleType(package)
        message_module = types.ModuleType(f"{package}.msg")
        for name, value in symbols.items():
            setattr(message_module, name, value)
        package_module.msg = message_module
        sys.modules[package] = package_module
        sys.modules[f"{package}.msg"] = message_module


install_ros_stubs()

from tang_control.controller_core import FOLLOW, HIGH, LOW, MANUAL
from tang_control.config import Control
from tang_control.tang_control import TangController
from cugo_rs485_motor_control.modbus_rtu import ModbusTimeoutError


class FakeBridge:
    def __init__(self, events):
        self.events = events

    def stop(self, force=False):
        self.events.append(("stop", force))

    def apply_body_velocity(self, v, w):
        self.events.append(("velocity", v, w))
        return v, w, -100.0, 100.0

    def reconnect(self):
        self.events.append(("reconnect",))

    def close(self):
        self.events.append(("close",))


class FakePublisher:
    def __init__(self, name, events):
        self.name = name
        self.events = events

    def publish(self, msg):
        value = msg.data if hasattr(msg, "data") else tuple(msg.buttons)
        self.events.append((self.name, value))


class FakeLed:
    def __init__(self, name, events):
        self.name = name
        self.events = events

    def on(self):
        self.events.append((self.name, "on"))

    def off(self):
        self.events.append((self.name, "off"))

    def close(self):
        self.events.append((self.name, "close"))


class FakeInput:
    def __init__(self, pressed=False):
        self.is_pressed = pressed

    def close(self):
        pass


class FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class FakeSpi:
    def close(self):
        pass


def make_node(mode=MANUAL, speed_mode=LOW):
    from tang_control.controller_core import TangControlRuntime, TangControlState

    events = []
    node = TangController.__new__(TangController)
    node.state = TangControlState(mode=mode, speed_mode=speed_mode)
    node.bridge = FakeBridge(events)
    node.runtime = TangControlRuntime(node.bridge, node.state)
    node.requested_mode = None
    node.obstacle_near = False
    node.standard_obstacle_near = False
    node.manual_obstacle_near = False
    node.last_cmd_vel = sys.modules["geometry_msgs.msg"].Twist()
    node.last_cmd_vel_time = 0.0
    node.last_follow_control_time = 0.0
    node.next_manual_log = 0.0
    node.next_follow_log = 0.0
    node.joy_publisher = FakePublisher("joy", events)
    node.mode_publisher = FakePublisher("mode", events)
    node.mode_led = FakeLed("mode_led", events)
    node.low_speed_led = FakeLed("low_led", events)
    node.high_speed_led = FakeLed("high_led", events)
    node.buzzer = FakeLed("buzzer", events)
    node.buzzer_off_at = 0.0
    node.motor_fault_active = False
    node.low_speed_button = FakeInput()
    node.high_speed_button = FakeInput()
    node.follow_button = FakeInput()
    node.manual_button = FakeInput()
    node.spi = FakeSpi()
    node.read_adc = lambda _channel: 500
    node.closed = False
    node.get_logger = lambda: FakeLogger()
    return node, events


class TangControllerOrchestrationTest(unittest.TestCase):
    def test_lidar_callback_uses_manual_clearance_in_manual_mode(self):
        node, _events = make_node(mode=MANUAL)
        scan = types.SimpleNamespace(
            ranges=[0.05],
            angle_min=0.0,
            angle_increment=0.0,
            range_min=0.05,
            range_max=10.0,
        )

        node.lidar_callback(scan)
        self.assertTrue(node.obstacle_near)

        scan.ranges = [0.051]
        node.lidar_callback(scan)
        self.assertFalse(node.obstacle_near)

    def test_mode_change_uses_cached_clearance_for_selected_mode(self):
        node, _events = make_node(mode=MANUAL)
        scan = types.SimpleNamespace(
            ranges=[0.20],
            angle_min=0.0,
            angle_increment=0.0,
            range_min=0.05,
            range_max=10.0,
        )
        node.lidar_callback(scan)
        self.assertFalse(node.obstacle_near)

        node.requested_mode = FOLLOW
        node.apply_requested_mode()
        self.assertTrue(node.obstacle_near)

    def test_modbus_timeout_reconnects_retries_stop_and_remains_idle(self):
        node, events = make_node(mode=MANUAL)
        original_stop = node.bridge.stop
        stop_calls = 0

        def fail_once_stop(force=False):
            nonlocal stop_calls
            stop_calls += 1
            if stop_calls == 1:
                raise ModbusTimeoutError("timeout while reading 2 bytes")
            return original_stop(force=force)

        node.bridge.stop = fail_once_stop
        node.obstacle_near = True

        self.assertTrue(node.control_once_with_motor_recovery())

        self.assertEqual(2, stop_calls)
        self.assertIn(("reconnect",), events)
        self.assertIn(("stop", True), events)
        self.assertEqual("idle", node.state.mode)
        self.assertFalse(node.motor_fault_active)

    def test_failed_stop_retry_keeps_node_in_faulted_idle(self):
        node, events = make_node(mode=MANUAL)

        def always_fail_stop(force=False):
            raise ModbusTimeoutError("timeout while reading 2 bytes")

        node.bridge.stop = always_fail_stop
        node.obstacle_near = True

        self.assertFalse(node.control_once_with_motor_recovery())

        self.assertIn(("reconnect",), events)
        self.assertEqual("idle", node.state.mode)
        self.assertTrue(node.motor_fault_active)

    def test_follow_to_manual_stops_before_follow_stop_and_resets_low(self):
        node, events = make_node(mode=FOLLOW, speed_mode=HIGH)
        node.requested_mode = MANUAL
        self.assertTrue(node.apply_requested_mode())

        self.assertEqual(("stop", True), events[0])
        joy_event = next(event for event in events if event[0] == "joy")
        self.assertEqual(1, joy_event[1][6])
        self.assertEqual(MANUAL, node.state.mode)
        self.assertEqual(LOW, node.state.speed_mode)

    def test_joystick_input_during_follow_switches_to_manual_low(self):
        node, events = make_node(mode=FOLLOW, speed_mode=HIGH)
        adc_values = {
            0: 960,
            1: 50,
        }
        node.read_adc = lambda channel: adc_values[channel]

        node.control_once()

        self.assertEqual(MANUAL, node.state.mode)
        self.assertEqual(LOW, node.state.speed_mode)
        self.assertEqual(("stop", True), events[0])
        joy_event = next(event for event in events if event[0] == "joy")
        self.assertEqual(1, joy_event[1][6])
        velocity_event = next(event for event in events if event[0] == "velocity")
        self.assertGreater(velocity_event[1], 0.0)

    def test_centered_joystick_does_not_leave_follow(self):
        node, _events = make_node(mode=FOLLOW)

        node.control_once()

        self.assertEqual(FOLLOW, node.state.mode)

    def test_obstacle_still_prevents_motion_after_joystick_takeover(self):
        node, events = make_node(mode=FOLLOW)
        node.read_adc = lambda _channel: 960
        node.obstacle_near = True
        node.standard_obstacle_near = True
        node.manual_obstacle_near = True

        node.control_once()

        self.assertEqual(MANUAL, node.state.mode)
        self.assertIn(("stop", False), events)
        self.assertFalse(any(event[0] == "velocity" for event in events))

    def test_manual_to_follow_stops_and_publishes_follow_buttons(self):
        node, events = make_node(mode=MANUAL)
        node.last_cmd_vel_time = 123.0
        node.requested_mode = FOLLOW
        self.assertTrue(node.apply_requested_mode())

        self.assertEqual(("stop", True), events[0])
        joy_events = [event for event in events if event[0] == "joy"]
        self.assertEqual(1, joy_events[0][1][4])
        self.assertEqual(1, joy_events[1][1][7])
        self.assertIn(("mode_led", "on"), events)
        self.assertIn(("buzzer", "on"), events)
        self.assertEqual(0.0, node.last_cmd_vel_time)

    def test_mode_beep_turns_off_after_deadline(self):
        node, events = make_node(mode=MANUAL)
        node.buzzer_off_at = 1.0

        node.update_buzzer()

        self.assertIn(("buzzer", "off"), events)
        self.assertEqual(0.0, node.buzzer_off_at)

    def test_reselecting_mode_has_no_side_effect(self):
        node, events = make_node(mode=MANUAL)
        node.requested_mode = MANUAL
        self.assertFalse(node.apply_requested_mode())
        self.assertEqual([], events)

    def test_obstacle_stops_before_manual_control(self):
        node, events = make_node(mode=MANUAL)
        node.obstacle_near = True
        node.control_once()
        self.assertIn(("stop", False), events)

    def test_cmd_vel_is_ignored_outside_follow(self):
        node, _events = make_node(mode=MANUAL)
        msg = sys.modules["geometry_msgs.msg"].Twist()
        msg.linear.x = 0.2
        node.cmd_vel_callback(msg)
        self.assertEqual(0.0, node.last_cmd_vel_time)

    def test_fresh_follow_cmd_vel_is_applied(self):
        node, events = make_node(mode=FOLLOW)
        msg = sys.modules["geometry_msgs.msg"].Twist()
        msg.linear.x = 10.0
        msg.angular.z = -10.0
        node.cmd_vel_callback(msg)
        node.control_once()

        velocity_event = next(event for event in events if event[0] == "velocity")
        self.assertAlmostEqual(Control.follow_accel_limit_mps2 * 0.05, velocity_event[1])
        self.assertAlmostEqual(
            Control.command_ema_alpha * Control.follow_max_w_radps,
            velocity_event[2],
        )

    def test_stale_follow_cmd_vel_stops(self):
        node, events = make_node(mode=FOLLOW)
        node.last_cmd_vel_time = 1.0
        node.runtime.follow_v_mps = 0.1
        node.control_once()
        self.assertIn(("stop", False), events)
        self.assertEqual(0.0, node.runtime.follow_v_mps)

    def test_obstacle_stops_fresh_follow_cmd_vel(self):
        node, events = make_node(mode=FOLLOW)
        msg = sys.modules["geometry_msgs.msg"].Twist()
        msg.linear.x = 0.2
        node.cmd_vel_callback(msg)
        node.obstacle_near = True
        node.runtime.follow_v_mps = 0.1
        node.control_once()

        self.assertIn(("stop", False), events)
        self.assertFalse(any(event[0] == "velocity" for event in events))
        self.assertEqual(0.0, node.runtime.follow_v_mps)

    def test_manual_transition_ignores_already_held_high_button(self):
        node, _events = make_node(mode=FOLLOW, speed_mode=HIGH)
        node.high_speed_button.is_pressed = True
        node.requested_mode = MANUAL
        node.obstacle_near = True
        node.control_once()
        self.assertEqual(LOW, node.state.speed_mode)

        node.update_speed_mode()
        self.assertEqual(LOW, node.state.speed_mode)

    def test_close_uses_bridge_close_once(self):
        node, events = make_node(mode=MANUAL)
        node.close_hardware()
        node.close_hardware()
        self.assertEqual(1, events.count(("close",)))
        self.assertTrue(node.closed)


if __name__ == "__main__":
    unittest.main()
