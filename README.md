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

統合bringupでは`initial_mode:=manual`が既定で、停止指令を送ってLOWへ初期化した後、
MANUAL操作を受け付けます。保守作業などでIDLE起動する場合は次を使用します。

起動時にモータドライバと通信できない場合は、TANG制御ノードを終了させずIDLEを維持し、
1秒周期で再接続します。左右への強制停止が成功した場合だけMANUAL LOWへ移ります。
操作可能になると、DNEと同じ80msの短音を3回鳴らして起動完了を通知します。

```bash
ros2 launch tang_bringup tang_bringup.launch.py initial_mode:=idle
```

既定ではRS-485出力が有効です。クローラを浮かせ、非常停止を
使用できる状態で起動してください。モータへ指令を送らず確認する場合は次を使用します。

```bash
ros2 launch tang_bringup tang_bringup.launch.py motor_dry_run:=true
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

## ▶️ 電源投入時の自動起動

Raspberry Piの共通systemdサービスから、TANG運転またはDNE運転のどちらか一方を起動します。
初期設定はTANGです。

TANG運転ではDockerコンテナ内で次のlaunchを実行し、MANUAL LOWで起動します。

```bash
ros2 launch tang_bringup tang_bringup.launch.py
```

DNE運転ではDockerを使用せず、ホスト上で次のハンドラを実行します。

```bash
python3 ~/icart_ws/src/tang2dne_handler/scripts/rs485Handler.py \
  --host 192.168.212.1 \
  --port-odm 18080 \
  --port-ctl 28080 \
  --robot CuGoV4
```

初回だけ、リポジトリに含まれる導入スクリプトを実行します。

```bash
cd ~/icart_ws/src/tang_ros2
./shell_scripts/install_auto_start.sh
```

次回の電源投入で使用するモードは、次のコマンドだけで切り替えられます。

```bash
# TANG運転へ切替
./shell_scripts/select_startup_mode.sh tang

# DNE運転へ切替
./shell_scripts/select_startup_mode.sh dne
```

選択値は`~/.config/tang/startup_mode`に保存されます。共通サービスは必ず片方だけを起動し、
もう一方の制御プロセスが動いている場合はRS-485競合を避けるため起動を拒否します。

導入時は次回の電源投入から有効になります。その場で起動する場合は明示的に開始します。

```bash
sudo systemctl start startup_robot.service
```

状態・ログ確認：

```bash
systemctl status startup_robot.service
journalctl -u startup_robot.service -f
```

### 自動起動したプロセスの停止

systemdによって自動起動したTANG運転またはDNE運転は、サービスを停止します。

```bash
sudo systemctl stop startup_robot.service
```

TANG運転中はDockerコンテナ`icart_mini_ros2`が停止し、DNE運転中は
`rs485Handler.py`が停止します。停止できたことは次のコマンドで確認できます。

```bash
systemctl status startup_robot.service
docker ps --filter name=icart_mini_ros2
```

`stop_auto_start.sh`はsystemdの`ExecStop`から呼び出される内部スクリプトです。
これだけを直接実行すると、サービスに設定された`Restart=always`によって再起動する可能性が
あるため、自動起動したプロセスの停止には`systemctl stop`を使用してください。

`./shell_scripts/auto_start.sh`をsystemdを介さず手動で実行した場合に限り、
次のコマンドで停止できます。

```bash
./shell_scripts/stop_auto_start.sh
```

自動起動を無効化する場合：

```bash
sudo systemctl disable startup_robot.service
```

systemdからはTTYなしでDockerを起動するため、`icart_mini_ros2/docker/run.sh`は
対話端末がある場合だけ`-it`を付けます。GUI用のX11マウントも`DISPLAY`がある場合だけ追加します。
