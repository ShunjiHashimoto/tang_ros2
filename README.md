# tang_ros2

人追従ロボットのモード切替・モータ制御ノード

---

## 📦 概要

`tang_control` は、LiDAR と joystick（アナログスティック）からの入力を用いて、ロボットの動作モードを切り替えつつ、人追従・手動操作・緊急停止を制御する ROS 2 ノードです。

---

## ビルド
```bash
cd ~/icart_ws
colcon build --symlink-install --packages-up-to tang_bringup
source install/setup.bash
```

## 統合起動

URG、固定TF、脚追従ノード、TANG制御ノードを1つのlaunchで起動します。

```bash
ros2 launch tang_bringup tang_bringup.launch.py
```

既定ではRS-485出力を行わないdry-runです。クローラを浮かせ、非常停止を
使用できる状態で実機出力を明示的に有効化する場合は次を使用します。

```bash
ros2 launch tang_bringup tang_bringup.launch.py motor_dry_run:=false
```

## 🚦 モード切替機能

本ノードでは、以下の3種類のモードを切り替えてロボットを制御します。

### 🟢 追従モード (`follow`)

- `leg_tracker_ros2` ノードから受信した人物位置を基に、追従対象の中心位置を推定
- 目標の角速度・直進速度を計算し、それを PWM 信号に変換してモータを駆動
- LiDAR による障害物検知も行い、安全に停止処理を実施

### 🟠 手動モード (`manual`)

- joystick のアナログ値を読み取り、現在の操作量から速度・角速度を算出
- PWM 制御により左右モータを個別に操作
- モータドライバに応じた調整が可能（DC／BLDC）

### 🔴 緊急停止モード (`emergency`)

- ボタンまたは障害物検知により、即座にモータ停止
- 自動的に手動モードに遷移

---

## 🧩 使用しているトピック

| トピック名             | 型                             | 説明                              |
|----------------------|--------------------------------|-----------------------------------|
| `/scan`              | `sensor_msgs/msg/LaserScan`    | LiDARによる障害物検知             |
| `/follow_target_person` | `leg_tracker_ros2/msg/Person`   | 人物の位置情報（追従対象）        |
| `/joy`               | `sensor_msgs/msg/Joy`           | モード切替用にpublish（模擬入力）  |

---

## ⚙️ ノード内部構成

### クラス: `TangController`

| メソッド名                | 概要                                                 |
|-------------------------|------------------------------------------------------|
| `switch_on_callback_follow()` | 追従モードへ切替、joyトピックを発行                     |
| `switch_on_callback_manual()` | 手動モードへ切替、joyトピックを発行                     |
| `manual_pwm_control()`       | joystickの入力から左右モータのPWMを計算・制御             |
| `follow_control()`           | 追従対象の座標から速度・角速度を算出しPWMで制御           |
| `lidar_callback()`           | LiDARからの障害物を検出                                 |
| `start()`                   | モードに応じて各処理ループを実行                        |

---

## 💡 実装補足

- `spidev` を使用してアナログスティックのADCを取得
- `gpiozero` により物理ボタン／ブザー／LEDを制御
- PWM計算は `motor.py` に分離
- `buzzer` によって動作状態を音で通知可能
- LiDAR距離が一定以下の場合は安全のため停止

---

## 🔧 必要な設定

- `tang_control/config.py` にて以下を定義：
  - ピン番号：`Pin`
  - PWMパラメータ：`PWM`
  - 追従PIDや閾値：`FOLLOWPID`, `HumanFollowParam`, `Control`
- `motor.py` にてモータごとのPWM制御ロジックを実装

---

## ▶️ 実行方法

```bash
ubuntu@raspi5:/etc/systemd/system$ cat startup_raspi.service 
[Unit]
Description=Start TANG ROS2 docker container with ROS2 launch
After=network.target docker.service
Requires=docker.service

[Service]
Restart=always
ExecStart=/home/ubuntu/icart_ws/src/tang_ros2/shell_scripts/auto_start.sh
WorkingDirectory=/home/ubuntu/icart_ws/src/tang_ros2/shell_scripts
User=ubuntu

[Install]
WantedBy=multi-user.targetros2 run tang_control tang_control
```

ログ確認
```bash
journalctl -u startup_raspi.service -f
```
停止
```bash
sudo systemctl stop startup_raspi.service
```
