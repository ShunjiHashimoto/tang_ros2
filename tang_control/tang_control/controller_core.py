"""GPIOやROSに依存しない、TangControllerの状態管理と入力変換。"""

from dataclasses import dataclass

from tang_control.config import Control


IDLE = "idle"
MANUAL = "manual"
FOLLOW = "follow"
LOW = "low"
HIGH = "high"


STEERING_MIN = 70
STEERING_CENTER = 500
STEERING_MAX = 960
THROTTLE_MIN = 50
THROTTLE_CENTER = 500
THROTTLE_MAX = 960
ADC_DEADBAND = 35


def normalize_axis(raw, minimum, center, maximum, deadband=ADC_DEADBAND):
    """左右非対称なADC実測範囲を、中心0の-1.0～1.0へ正規化する。"""
    offset = raw - center
    if abs(offset) <= deadband:
        return 0.0
    if offset > 0:
        value = offset / float(maximum - center)
    else:
        value = offset / float(center - minimum)
    return max(-1.0, min(1.0, value))


def joystick_to_body_velocity(raw_steering, raw_throttle, speed_mode):
    """実測ADC値を、選択速度内の並進速度・角速度へ変換する。"""
    steering_axis = normalize_axis(
        raw_steering,
        STEERING_MIN,
        STEERING_CENTER,
        STEERING_MAX,
    )
    throttle_axis = normalize_axis(
        raw_throttle,
        THROTTLE_MIN,
        THROTTLE_CENTER,
        THROTTLE_MAX,
    )

    if speed_mode == HIGH:
        max_v = Control.manual_high_max_v_mps
        max_w = Control.manual_high_max_w_radps
    else:
        max_v = Control.manual_low_max_v_mps
        max_w = Control.manual_low_max_w_radps

    # 操舵ADCは右へ倒すと増加するが、ROSの正の角速度は左旋回なので反転する。
    return throttle_axis * max_v, -steering_axis * max_w


@dataclass
class TangControlState:
    """ROS/GPIOノードと自動テストで共有するモード・速度状態。"""

    mode: str = IDLE
    speed_mode: str = LOW
    previous_low_pressed: bool = False
    previous_high_pressed: bool = False

    def select_mode(self, selected_mode):
        """モードを変更し、MANUALへ入る場合は必ず低速へ戻す。"""
        if selected_mode not in (MANUAL, FOLLOW):
            raise ValueError(f"unsupported mode: {selected_mode}")
        if selected_mode == self.mode:
            return False
        self.mode = selected_mode
        if selected_mode == MANUAL:
            self.speed_mode = LOW
        return True

    def update_speed_buttons(self, low_pressed, high_pressed):
        """MANUAL中に新しく押された単独の速度ボタンだけを採用する。"""
        low_was_pressed = low_pressed and not self.previous_low_pressed
        high_was_pressed = high_pressed and not self.previous_high_pressed

        # 一瞬の重なりを両押しと判定しても、押下履歴は更新しない。
        # その後に片方だけが残った場合は、新しい単独押下として扱う。
        if self.mode == MANUAL and low_pressed and high_pressed:
            return False

        self.previous_low_pressed = low_pressed
        self.previous_high_pressed = high_pressed

        if self.mode != MANUAL:
            return False

        selected = self.speed_mode
        if low_was_pressed:
            selected = LOW
        elif high_was_pressed:
            selected = HIGH

        changed = selected != self.speed_mode
        self.speed_mode = selected
        return changed

    def remember_speed_buttons(self, low_pressed, high_pressed):
        """速度を変更せず、押下エッジ判定の基準状態だけを更新する。"""
        self.previous_low_pressed = low_pressed
        self.previous_high_pressed = high_pressed


class TangControlRuntime:
    """状態変更とモーター指令の順序をRS-485ブリッジ越しに管理する。"""

    def __init__(self, bridge, state=None):
        self.bridge = bridge
        self.state = state if state is not None else TangControlState()

    def select_mode(self, selected_mode):
        if selected_mode == self.state.mode:
            return False
        # 指令元のモードを変更する前に、必ず左右の停止を指令する。
        self.bridge.stop(force=True)
        return self.state.select_mode(selected_mode)

    def apply_manual_input(self, raw_steering, raw_throttle):
        """MANUAL時だけジョイスティック入力をモーター指令へ渡す。"""
        if self.state.mode != MANUAL:
            self.bridge.stop()
            return None
        requested_v, requested_w = joystick_to_body_velocity(
            raw_steering,
            raw_throttle,
            self.state.speed_mode,
        )
        return self.bridge.apply_body_velocity(requested_v, requested_w)
