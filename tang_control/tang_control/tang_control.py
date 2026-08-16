#!/usr/bin/env python3
"""TANGのGPIO、ジョイスティック、CuGoV4用RS-485指令をまとめる制御ノード。"""

import time

import rclpy
import spidev
from geometry_msgs.msg import Twist
from gpiozero import Button, LED
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String

from cugo_rs485_motor_control.bridge import (
    MotorBridgeConfig,
    Rs485DualMotorBridge,
)
from tang_control.config import Control, LiDARParam, Pin
from tang_control.controller_core import (
    FOLLOW,
    MANUAL,
    TangControlState,
    TangControlRuntime,
)


CONTROL_PERIOD_SEC = 0.05
PRINT_PERIOD_SEC = 0.20
MODE_BEEP_DURATION_SEC = 0.20


class TangController(Node):
    def __init__(self):
        super().__init__("tang_control")

        # 起動直後はIDLEとし、ボタンを押すまでモーター指令を出さない。
        self.state = TangControlState()
        # GPIOのコールバックでは状態を直接変更せず、メインループへ要求を渡す。
        # これにより、モード変更と停止指令が別スレッドで競合するのを防ぐ。
        self.requested_mode = None
        self.obstacle_near = False
        self.closed = False
        self.next_manual_log = 0.0
        self.next_follow_log = 0.0
        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = 0.0
        self.last_follow_control_time = 0.0
        self.buzzer_off_at = 0.0

        # 安全のため既定はdry-runとし、launch引数で明示した場合だけ
        # 実機出力する。
        self.declare_parameter("motor_dry_run", True)
        motor_dry_run = bool(self.get_parameter("motor_dry_run").value)

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 100000

        # モード切替ボタンはプルアップ入力なので、押下時のGPIOレベルはLOW。
        self.follow_button = Button(
            Pin.follow_mode,
            pull_up=True,
            bounce_time=0.05,
        )
        self.manual_button = Button(
            Pin.manual_mode,
            pull_up=True,
            bounce_time=0.05,
        )
        # 速度切替も独立したモーメンタリボタンを使用する。
        # GPIO3=低速、GPIO4=高速。ボタンを離しても選択速度は保持する。
        self.low_speed_button = Button(
            Pin.low_speed_button,
            pull_up=True,
            bounce_time=0.05,
        )
        self.high_speed_button = Button(
            Pin.high_speed_button,
            pull_up=True,
            bounce_time=0.05,
        )
        # GPIO14はモード表示、GPIO25/26は低速・高速表示に使用する。
        self.mode_led = LED(Pin.mode_led, initial_value=False)
        self.low_speed_led = LED(Pin.low_speed_led, initial_value=False)
        self.high_speed_led = LED(Pin.high_speed_led, initial_value=False)
        self.buzzer = LED(Pin.buzzer, initial_value=False)

        self.follow_button.when_pressed = lambda: self.request_mode(FOLLOW)
        self.manual_button.when_pressed = lambda: self.request_mode(MANUAL)

        # 実機動作済みのCuGoV4設定を変更せず使用する。
        # 左モーターは符号反転、右モーターは通常方向として扱う。
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
            dry_run=motor_dry_run,
        )
        self.runtime = TangControlRuntime(self.bridge, self.state)

        # LiDARは従来どおり近接停止に使用する。
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            10,
        )
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )
        # FOLLOW開始・停止用の仮想Joyと、確認用の現在モードを発行する。
        self.joy_publisher = self.create_publisher(Joy, "/joy", 10)
        self.mode_publisher = self.create_publisher(String, "/tang/mode", 10)

        self.update_indicators()
        self.publish_mode()
        dry_run_state = "enabled" if motor_dry_run else "disabled"
        self.get_logger().info(
            f"TangController ready: IDLE, RS-485 dry-run={dry_run_state}"
        )

    def request_mode(self, mode):
        """GPIOで選択されたモードを、次の制御周期で処理するため保存する。"""
        self.requested_mode = mode

    def apply_requested_mode(self):
        """停止指令を先に生成してから、安全にモードを切り替える。"""
        selected = self.requested_mode
        self.requested_mode = None
        if selected is None or selected == self.state.mode:
            return False

        previous = self.state.mode
        # 入力元を変更する前に、左右モーターを必ず停止させる。
        self.runtime.select_mode(selected)
        if previous == FOLLOW:
            # FOLLOWを離れる場合は追従ノードにも停止ボタンを送る。
            self.publish_fake_joy_button_press(Pin.followme_stop_button)

        if selected == FOLLOW:
            # モード切替前に受信した古い速度指令は使用しない。
            self.last_cmd_vel_time = 0.0
            self.last_follow_control_time = 0.0
            # 非常停止解除と追従開始は、既存icartと同じJoyボタン番号を使う。
            self.publish_fake_joy_button_press(Pin.unlock_emergency_button)
            self.publish_fake_joy_button_press(Pin.followme_start_button)

        self.update_indicators()
        self.publish_mode()
        self.start_mode_beep()
        self.get_logger().info(f"Mode: {previous.upper()} -> {selected.upper()}")
        return True

    def start_mode_beep(self):
        """モード切替を短いブザー音で通知する。"""
        self.buzzer.on()
        self.buzzer_off_at = time.monotonic() + MODE_BEEP_DURATION_SEC

    def update_buzzer(self):
        """ブザー時間が満了したら、制御ループを止めずに消音する。"""
        if self.buzzer_off_at > 0.0 and time.monotonic() >= self.buzzer_off_at:
            self.buzzer.off()
            self.buzzer_off_at = 0.0

    def lidar_callback(self, msg):
        """停止距離より近い有効な測距値があるかを記憶する。"""
        self.obstacle_near = any(
            0.0 < distance < LiDARParam.stop_distance_thresh
            for distance in msg.ranges
        )

    def cmd_vel_callback(self, msg):
        """FOLLOW中だけ最新の速度指令と受信時刻を保存する。"""
        if self.state.mode != FOLLOW:
            return
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = time.monotonic()

    def publish_fake_joy_button_press(self, button_index):
        """追従ノードへ、指定したボタンだけが押されたJoyを1回送る。"""
        msg = Joy()
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 12
        msg.buttons[button_index] = 1
        self.joy_publisher.publish(msg)

    def publish_mode(self):
        """現在モードを/tang/modeへ発行する。"""
        msg = String()
        msg.data = self.state.mode
        self.mode_publisher.publish(msg)

    def read_adc(self, channel):
        """MCP3004の指定チャンネルから10bitのADC値を読み取る。"""
        response = self.spi.xfer2([1, (8 + channel) << 4, 0])
        return ((response[1] & 3) << 8) | response[2]

    def update_speed_mode(self):
        """MANUAL中の新しい速度ボタン押下だけを反映する。"""
        changed = self.state.update_speed_buttons(
            self.low_speed_button.is_pressed,
            self.high_speed_button.is_pressed,
        )
        if changed:
            self.update_indicators()
            self.get_logger().info(
                f"Speed: {self.state.speed_mode.upper()}"
            )

    def update_indicators(self):
        """現在のモードと速度設定を3つのLEDへ反映する。"""
        # FOLLOWはGPIO14を点灯し、IDLEとMANUALでは消灯する。
        if self.state.mode == FOLLOW:
            self.mode_led.on()
        else:
            self.mode_led.off()

        # 選択中の速度に対応するLEDだけを点灯する。
        if self.state.speed_mode == "low":
            self.low_speed_led.on()
            self.high_speed_led.off()
        else:
            self.low_speed_led.off()
            self.high_speed_led.on()

    def manual_control(self):
        """ジョイスティックを読み、dry-runの左右モーター指令へ変換する。"""
        # CH0は操舵、CH1は前後操作として実機確認済み。
        raw_steering = self.read_adc(Pin.vrx_channel)
        raw_throttle = self.read_adc(Pin.vry_channel)
        result = self.runtime.apply_manual_input(
            raw_steering,
            raw_throttle,
        )
        limited_v, limited_w, left_rpm, right_rpm = result

        now = time.monotonic()
        # ログ量を抑えつつ、入力値と演算結果を追える周期で表示する。
        if now >= self.next_manual_log:
            self.get_logger().info(
                f"MANUAL {self.state.speed_mode.upper()} "
                f"CH0={raw_steering} CH1={raw_throttle} "
                f"v={limited_v * 3.6:+.2f}km/h w={limited_w:+.3f}rad/s "
                f"left={left_rpm:+.0f}rpm right={right_rpm:+.0f}rpm"
            )
            self.next_manual_log = now + PRINT_PERIOD_SEC

    def follow_control(self):
        """FOLLOWの最新指令を、タイムアウト付きでモーターへ渡す。"""
        now = time.monotonic()
        if (
            self.last_cmd_vel_time <= 0.0
            or now - self.last_cmd_vel_time > Control.follow_cmd_timeout_sec
        ):
            self.bridge.stop()
            self.runtime.reset_motion_filters()
            self.last_follow_control_time = 0.0
            return

        if self.last_follow_control_time <= 0.0:
            dt_sec = CONTROL_PERIOD_SEC
        else:
            dt_sec = max(
                0.0,
                min(CONTROL_PERIOD_SEC, now - self.last_follow_control_time),
            )
        self.last_follow_control_time = now

        result = self.runtime.apply_follow_input(
            self.last_cmd_vel.linear.x,
            self.last_cmd_vel.angular.z,
            dt_sec,
        )
        limited_v, limited_w, left_rpm, right_rpm = result
        if now >= self.next_follow_log:
            self.get_logger().info(
                f"FOLLOW v={limited_v * 3.6:+.2f}km/h "
                f"w={limited_w:+.3f}rad/s "
                f"left={left_rpm:+.0f}rpm right={right_rpm:+.0f}rpm"
            )
            self.next_follow_log = now + PRINT_PERIOD_SEC

    def control_once(self):
        """1制御周期分のモード、速度、安全停止、手動操作を処理する。"""
        self.update_buzzer()
        mode_changed = self.apply_requested_mode()
        if mode_changed and self.state.mode == MANUAL:
            # MANUALは必ず低速で開始する。切替時から速度ボタンが押されて
            # いた場合は採用せず、一度離してからの再押下を要求する。
            self.state.remember_speed_buttons(
                self.low_speed_button.is_pressed,
                self.high_speed_button.is_pressed,
            )
        else:
            self.update_speed_mode()

        if self.obstacle_near:
            # 障害物を検出した周期では、ジョイスティック入力より停止を優先する。
            self.bridge.stop()
            self.runtime.reset_motion_filters()
            self.last_follow_control_time = 0.0
            return
        if self.state.mode == MANUAL:
            self.manual_control()
            return
        if self.state.mode == FOLLOW:
            self.follow_control()
            return

        # IDLEでは常に停止を維持する。
        self.bridge.stop()
        self.runtime.reset_motion_filters()
        self.last_follow_control_time = 0.0

    def start(self):
        """ROSイベントと車体制御を50ms周期で処理する。"""
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD_SEC)
                self.control_once()
        finally:
            self.close_hardware()

    def close_hardware(self):
        """終了時に停止指令を生成し、GPIO、SPI、RS-485資源を解放する。"""
        if self.closed:
            return
        self.closed = True
        try:
            self.bridge.close()
        finally:
            self.mode_led.off()
            self.low_speed_led.off()
            self.high_speed_led.off()
            self.buzzer.off()
            self.follow_button.close()
            self.manual_button.close()
            self.low_speed_button.close()
            self.high_speed_button.close()
            self.mode_led.close()
            self.low_speed_led.close()
            self.high_speed_led.close()
            self.buzzer.close()
            self.spi.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TangController()
        node.start()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_hardware()
            node.destroy_node()
        # SIGINT時はrclpy側ですでにshutdownされている場合があるため、
        # 未停止の場合だけ明示的に終了する。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
