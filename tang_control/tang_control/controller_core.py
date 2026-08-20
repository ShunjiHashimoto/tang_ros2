"""GPIOやROSに依存しない、TangControllerの状態管理と入力変換。"""

import math
from dataclasses import dataclass

from tang_control.config import Control, LiDARParam


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


def scan_contains_nearby_obstacle(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
):
    """LiDAR点が方向別の安全余裕を加えた車体外形内にあればTrueを返す。"""
    min_body_x = -LiDARParam.body_rear_length_m
    max_body_x = (
        LiDARParam.body_front_length_m
        + LiDARParam.obstacle_front_clearance_m
    )
    max_abs_y = (
        LiDARParam.body_half_width_m
        + LiDARParam.obstacle_side_clearance_m
    )

    for index, distance in enumerate(ranges):
        if not math.isfinite(distance):
            continue
        if distance < range_min or distance > range_max:
            continue

        angle = angle_min + index * angle_increment
        body_x = LiDARParam.position_x_m + distance * math.cos(angle)
        body_y = LiDARParam.position_y_m + distance * math.sin(angle)

        if (
            min_body_x - 1e-9 <= body_x <= max_body_x + 1e-9
            and abs(body_y) <= max_abs_y + 1e-9
        ):
            return True
    return False


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

    # 取付方向を設定値で補正し、前進をROSの正方向、西側への旋回を正角速度にする。
    return (
        throttle_axis * Control.manual_throttle_sign * max_v,
        steering_axis * Control.manual_steering_sign * max_w,
    )


def limit_follow_velocity(v_mps, w_radps):
    """FOLLOW指令をTANG側の最終速度上限に収める。"""
    if not math.isfinite(v_mps) or not math.isfinite(w_radps):
        return 0.0, 0.0
    limited_v = max(
        -Control.follow_max_v_mps,
        min(Control.follow_max_v_mps, v_mps),
    )
    limited_w = max(
        -Control.follow_max_w_radps,
        min(Control.follow_max_w_radps, w_radps),
    )
    return limited_v, limited_w * Control.follow_angular_sign


def limit_follow_acceleration(target_v_mps, current_v_mps, dt_sec):
    """FOLLOWの加速だけを制限する。減速は安全のため即時反映する。"""
    if not all(math.isfinite(value) for value in (target_v_mps, current_v_mps, dt_sec)):
        return 0.0
    if dt_sec <= 0.0 or target_v_mps == current_v_mps:
        return current_v_mps

    # 減速は即時反映し、進行方向の反転時はいったん停止してから再加速する。
    same_direction = target_v_mps * current_v_mps >= 0.0
    if not same_direction:
        return 0.0
    if abs(target_v_mps) <= abs(current_v_mps):
        return target_v_mps

    max_step = Control.follow_accel_limit_mps2 * dt_sec
    if max_step <= 0.0:
        return target_v_mps
    step = math.copysign(min(max_step, abs(target_v_mps - current_v_mps)), target_v_mps)
    return current_v_mps + step


def smooth_command(target, previous):
    """従来TANGと同じEMAで速度指令を平滑化する。"""
    if not math.isfinite(target) or not math.isfinite(previous):
        return 0.0
    alpha = Control.command_ema_alpha
    if not 0.0 < alpha <= 1.0:
        return target
    return alpha * target + (1.0 - alpha) * previous


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
        self.manual_v_mps = 0.0
        self.manual_w_radps = 0.0
        self.follow_smoothed_v_mps = 0.0
        self.follow_smoothed_w_radps = 0.0
        self.follow_v_mps = 0.0

    def reset_follow_ramp(self):
        self.follow_smoothed_v_mps = 0.0
        self.follow_smoothed_w_radps = 0.0
        self.follow_v_mps = 0.0

    def reset_motion_filters(self):
        self.manual_v_mps = 0.0
        self.manual_w_radps = 0.0
        self.reset_follow_ramp()

    def select_mode(self, selected_mode):
        if selected_mode == self.state.mode:
            return False
        # 指令元のモードを変更する前に、必ず左右の停止を指令する。
        self.bridge.stop(force=True)
        self.reset_motion_filters()
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
        self.manual_v_mps = smooth_command(requested_v, self.manual_v_mps)
        self.manual_w_radps = smooth_command(requested_w, self.manual_w_radps)
        return self.bridge.apply_body_velocity(self.manual_v_mps, self.manual_w_radps)

    def apply_follow_input(self, v_mps, w_radps, dt_sec):
        """FOLLOW時だけ、制限後の速度指令をモーターへ渡す。"""
        if self.state.mode != FOLLOW:
            self.bridge.stop()
            self.reset_follow_ramp()
            return None
        limited_v, limited_w = limit_follow_velocity(v_mps, w_radps)

        # 追従対象への到達や旋回優先など、上流から各軸へのゼロ指令は
        # 平滑化で遅らせず即時反映する。
        if limited_v == 0.0:
            self.follow_smoothed_v_mps = 0.0
            self.follow_v_mps = 0.0
        else:
            self.follow_smoothed_v_mps = smooth_command(
                limited_v,
                self.follow_smoothed_v_mps,
            )
            self.follow_v_mps = limit_follow_acceleration(
                self.follow_smoothed_v_mps,
                self.follow_v_mps,
                dt_sec,
            )

        if limited_w == 0.0:
            self.follow_smoothed_w_radps = 0.0
        else:
            self.follow_smoothed_w_radps = smooth_command(
                limited_w,
                self.follow_smoothed_w_radps,
            )
        return self.bridge.apply_body_velocity(
            self.follow_v_mps,
            self.follow_smoothed_w_radps,
        )
