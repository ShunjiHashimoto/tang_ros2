# CuGoV4・TANG統合仕様と実装状況

## 目的と方針

TANGのモード選択、MANUAL・FOLLOW入力、安全停止、CuGoV4へのRS-485モータ指令を、既存の`TangController`へ集約する。

- 統合bringupでは停止指令と入力初期化を先行し、`MANUAL LOW`で起動する。
- `TangController`単体起動の既定値は安全側の`IDLE`とし、launchの`initial_mode`で起動モードを明示する。
- 現行の走行モードは`MANUAL`と`FOLLOW`とする。
- PROPOはモード選択方法が確定するまで統合対象外とする。
- DNE運転は`TangController`と同時起動せず、別の起動構成で扱う。
- `/dev/ttyUSB0`を開く制御プロセスは常に1つだけにする。
- 旧GPIO PWM制御は互換用としてコードを残すが、現行`TangController`では使用しない。

## 現在の実装状況（2026年8月23日）

### 完了

- GPIO16によるFOLLOW選択、GPIO21によるMANUAL選択。
- GPIO14によるモード表示（FOLLOWで点灯、IDLE・MANUALで消灯）。
- GPIO3・GPIO4による低速・高速切替と、GPIO25・GPIO26による速度表示。
- MANUAL・FOLLOWともモード遷移時は必ず低速へ初期化。
- MANUALのジョイスティック入力、EMA平滑化、速度上限、RS-485指令への変換。
- FOLLOWの仮想`/joy`開始・停止、`/cmd_vel`保存、0.5秒タイムアウト、速度モード別制限。
- FOLLOW中のジョイスティック操作を優先し、MANUAL低速へ切り替える処理。
- 車体外形とLiDAR位置を考慮した近接障害物停止。
- Modbus異常時の一時IDLE化、再接続、強制停止確認、直前モードへの復帰。
- 起動時のModbus通信不能ではIDLEを維持し、1秒周期で通信と強制停止を再確認してから要求された初期モードへ遷移。
- 通信と強制停止を確認して操作可能になった時点で、80 msの短音を3回鳴らして起動完了を通知。
- FOLLOWの通信復旧中に`/follow_me/control`へ`pause`、復旧後に`resume`を送る連携。
- 統合launchからURG、TF、人物追従ノード、`TangController`を起動する構成。
- Raspberry Piの共通systemdサービスからTANGまたはDNEの片方を選択して自動起動する構成。
- 統合bringupのMANUAL LOW起動と、単体起動時のIDLE維持。
- コアテスト38件、TangControllerノードテスト31件、bringup launchテスト2件、自動起動テスト4件の自動確認。

### 保留・実機確認が必要

- PROPOのモード選択方法とTANGへの統合。
- DNE側`rs485Handler.py`への車体速度上限とCuGoV4ブリッジの統合確認。
- Raspberry Pi停止やプロセス強制終了時にもドライバ単体で停止する通信タイムアウト／ハートビート。
- 異常時の瞬時停止を使用するかどうかの実機評価。
- USB-RS485切断、電源断、再接続を含むフェイルセーフ試験。

## 状態とモード遷移

### IDLE

- `TangController`単体起動、またはbringupで`initial_mode:=idle`を指定した場合の初期状態。
- 左右モータ停止を維持する。
- GPIO14のモードLEDは消灯する。
- `/cmd_vel`は採用しない。
- GPIO3・GPIO4を押しても速度モードは変更しない。

### MANUAL

- 統合bringupの既定初期モード。モータ停止、LOW初期化、最初の制御周期での停止確認を経て入力を受け付ける。
- GPIO21の新しい押下で遷移する。
- 遷移前に左右へ強制停止を送り、速度モードをLOWへ戻す。
- MCP3004のCH0を操舵、CH1を前後入力として使用する。
- GPIO3でLOW、GPIO4でHIGHを選択し、次の単独押下まで保持する。
- GPIO3・GPIO4の同時押しは無視し、現在値を保持する。
- 速度ボタンには50 msのチャタリング除去を適用する。
- MANUALの周期的な速度・rpmログは、実機ターミナルの出力量を抑えるため停止している。
- GPIO14のモードLEDは消灯する。

### FOLLOW

- GPIO16の新しい押下で遷移する。
- 遷移前に左右へ強制停止を送り、速度モードをLOWへ戻す。
- `/joy`へロック解除ボタン4と追従開始ボタン7を送る。
- 人物追従ノードが発行する`/cmd_vel`を保存し、50 ms周期の制御ループでモータへ反映する。
- 受信から0.5秒を超えた`/cmd_vel`は使用せず停止する。
- GPIO3・GPIO4でLOW・HIGHを切り替えられる。
- LOWで並進速度を制限した場合は角速度も同じ比率で下げ、カーブ半径を維持する。
- FOLLOWを離れるときは、先にモータを停止してから`/joy`へ追従停止ボタン6を送る。
- FOLLOW中にジョイスティックがデッドゾーン外へ入ると、MANUAL LOWへ切り替える。
- GPIO14のモードLEDは点灯する。

### モード選択の共通仕様

- モードボタンには50 msのチャタリング除去を適用する。
- 現在と同じモードの再選択は無視し、停止や追従開始信号の再送を行わない。
- モード変更では、入力元を変更する前に左右へ強制停止を送る。
- 遷移時点ですでに押されていた速度ボタンは採用せず、一度離してからの再押下を要求する。
- モード変更時はブザーを0.2秒鳴らし、`/tang/mode`へ現在モードを発行する。

## 速度・旋回設定

現行設定の参照元は`tang_control/tang_control/config.py`の`Control`とする。

| モード | 速度設定 | 最大並進速度 | 時速 | 最大角速度 |
| --- | --- | ---: | ---: | ---: |
| MANUAL | LOW | `0.15 m/s` | `0.54 km/h` | `0.6 rad/s` |
| MANUAL | HIGH | `0.30 m/s` | `1.08 km/h` | `1.0 rad/s` |
| FOLLOW | LOW | `0.15 m/s` | `0.54 km/h` | 下記のFOLLOW旋回上限 |
| FOLLOW | HIGH | `0.30 m/s` | `1.08 km/h` | 下記のFOLLOW旋回上限 |
| PROPO | 保留 | `0.2625 m/s` | `0.945 km/h` | `1.57 rad/s` |
| DNE | 別構成 | `0.4167 m/s` | `1.5 km/h` | `1.396 rad/s`（80 deg/s） |

FOLLOWの旋回設定は次のとおり。

- 通常最大角速度：`15 deg/s`（約`0.262 rad/s`）
- 極端角度の判定境界：`45 deg`
- 45度を超えた場合の最大角速度：`35 deg/s`（約`0.611 rad/s`）
- TANG側の最終角速度上限：`35 deg/s`
- `angular.z`の符号補正：`-1.0`
- EMA係数：`0.75`
- FOLLOW並進加速度上限：`1.0 m/s²`

`tang_leg_tracker.launch.py`は`Control.follow_high_max_v_mps`とFOLLOW旋回設定を参照する。したがってTANG用FOLLOW速度・旋回値は`config.py`だけで変更し、launchへ数値を重複定義しない。

## RS-485モータ制御

現行`TangController`は`cugo_rs485_motor_control`の`Rs485DualMotorBridge`を使用する。

| 項目 | 設定値 |
| --- | ---: |
| デバイス | `/dev/ttyUSB0` |
| ボーレート | `9600` |
| 左スレーブID | `2` |
| 右スレーブID | `1` |
| 車輪半径 | `0.03858 m` |
| トレッド | `0.376 m` |
| 減速比 | `20` |
| 左右モータ符号 | `-1 / +1` |
| モータ最終上限 | `2600 rpm` |
| 最低回転数 | `80 rpm` |
| 発進しきい値 | `120 rpm` |

- 2600 rpmはモータ保護と車体速度変換の最終上限であり、モード速度の調整には使用しない。
- 左右のどちらかが2600 rpmを超える場合は、並進速度と角速度を同じ比率で縮小する。
- 通常停止は実機動作済みの減速停止を使用する。
- 統合launchの`motor_dry_run`既定値は`false`であり、そのまま起動すると実機へRS-485指令を出す。
- 通信なしで確認するときは、必ず`motor_dry_run:=true`を明示する。
- `TangController`を単体実行した場合のROSパラメータ既定値は安全側の`true`だが、統合launchが`false`を上書きする。

起動例：

```bash
ros2 launch tang_bringup tang_bringup.launch.py motor_dry_run:=true
```

実機出力を有効にする場合は、非常停止を使用できる状態にし、RS-485ポートを他プロセスが開いていないことを確認する。

## 障害物停止

LiDAR基準ではなく、旋回中心を基準にした車体外形と停止余裕で判定する。

| 項目 | 設定値 |
| --- | ---: |
| 車体前方長 | `0.320 m` |
| 車体後方長 | `0.430 m` |
| 車体半幅 | `0.250 m` |
| LiDAR位置 | 前方`0.320 m`、横`0.0 m` |
| FOLLOW・IDLE前方余裕 | `0.300 m` |
| FOLLOW・IDLE側方余裕 | `0.100 m` |
| MANUAL前方余裕 | `0.050 m` |
| MANUAL側方余裕 | `0.050 m` |

- 現在モードに応じて標準判定とMANUAL判定を切り替える。
- 障害物を検出した周期はモード入力より停止を優先する。
- 停止時にMANUAL・FOLLOWの平滑化状態をリセットする。

## Modbus異常時の復旧

### 起動時

1. `TangController`ノード、GPIO、ROS publisher/subscriptionをIDLEで初期化する。
2. RS-485ブリッジを生成し、左右モータへの強制停止を試みる。
3. シリアルポート未検出、Modbusタイムアウト、応答異常の場合はIDLEを維持する。
4. モードボタンとジョイスティック入力を採用せず、1秒周期で再接続する。
5. 通信と左右の強制停止が成功した場合だけ、bringupで要求されたMANUAL LOWへ遷移する。
6. DNEと同じ80 ms ON・80 ms OFFの短音を3回鳴らす。
7. MANUAL遷移後も最初の制御周期は停止のみとし、次周期からジョイスティック入力を受け付ける。

非常停止がモータドライバの通信を遮断する構成でも、非常停止解除後の再試行で安全な停止を確認してから操作可能になる。

### 走行開始後

1. モータ指令で`ModbusError`を検出する。
2. 異常発生時のモードを記憶し、一時的にIDLEへ移る。
3. FOLLOWだった場合は`/follow_me/control`へ`pause`を送り、追従対象を保持したまま再捕捉タイマーと速度出力を止める。
4. RS-485を再接続し、左右へ強制停止を再送する。
5. 再接続または停止が失敗した間はIDLEを維持して再試行する。
6. 強制停止に成功したら異常前のIDLE・MANUAL・FOLLOWへ復帰する。
7. FOLLOW復帰時は古い`/cmd_vel`を破棄し、`resume`を送って新しい指令を待つ。

この処理はUSB-RS485切断中の物理停止を保証しない。ドライバ単体の通信タイムアウトとハードウェア非常停止は別途必要である。

## 起動構成と排他

### TANG運転

- 起動する：URG、TF、`leg_cluster_tracking_node`、`TangController`
- 起動しない：YPSpur、F710用teleop、旧GPIO PWMモータ制御、DNE用`rs485Handler.py`、`propo_control.py`、モータ単体CLI
- 起動モードをTANGに設定した場合、`startup_robot.service`がDockerを起動し、`ros2 launch tang_bringup tang_bringup.launch.py`を実行する。
- 統合bringupの`initial_mode`既定値は`manual`とし、保守時は`initial_mode:=idle`を指定できる。

### DNE運転

- 起動する：`tang2dne_handler/scripts/rs485Handler.py`
- 起動しない：`TangController`、`propo_control.py`、`cugo_rs485_motor_control/scripts/main.py`
- 起動モードをDNEに設定した場合、`startup_robot.service`がDockerを使わず、ホスト上でCuGoV4用`rs485Handler.py`を実行する。
- DNE指令は正負とも`0.4167 m/s`以内へ制限してからRS-485ブリッジへ渡す計画とする。
- DNE/TANG間のモード切替は今回のTANG統合には含めない。

自動起動モードは`~/.config/tang/startup_mode`へ保存し、次のコマンドで切り替える。

```bash
./shell_scripts/select_startup_mode.sh tang
./shell_scripts/select_startup_mode.sh dne
```

共通サービスはTANGとDNEを同時起動しない。選択外の制御プロセスを検出した場合は、`/dev/ttyUSB0`の二重所有を避けるため起動を拒否する。

## 確認状況

### 自動確認済み

- `test/test_controller_core.py`：38件成功
- `test/test_tang_controller_node.py`：31件成功
- `test/test_tang_bringup_launch.py`：2件成功
- `test/test_auto_start_scripts.py`：4件成功
- Python構文チェックと`git diff --check`：成功
- 単体ノードのIDLE起動、bringupのMANUAL LOW起動、起動時停止、速度切替、同時押し無視、押下済みボタン無視。
- MANUAL・FOLLOWの速度上限、曲率維持、EMA、FOLLOW加速制限、即時停止。
- `/cmd_vel`タイムアウト、障害物停止、ジョイスティックによるMANUAL移行。
- Modbus異常時の再接続、直前モード復帰、FOLLOWのpause/resume。
- 2600 rpm最終制限、最低回転数、発進しきい値。

### 実機で再確認する項目

- クローラを浮かせ、左右符号、前後進、旋回方向、指令rpm、停止を確認する。
- LOWから開始し、無人区画でMANUAL・FOLLOWの停止距離と旋回速度を確認する。
- FOLLOWの通常旋回15 deg/sと、45度超での35 deg/s旋回を確認する。
- GPIO3・GPIO4の速度切替とLED表示を確認する。
- 障害物停止と、FOLLOW中のジョイスティック介入を確認する。
- USB-RS485切断時はハードウェア非常停止を使用し、復旧後に古い指令で再発進しないことを確認する。

## 他リポジトリとの依存

- `cugo_rs485_motor_control`：RS-485通信、左右モータ変換、rpm制限。
- `icart_mini_ros2/icart_mini_leg_tracker`：距離連動追従、極端角度用旋回速度、`/follow_me/control`のpause/resume。
- `tang2dne_handler`：DNE指令受信とDNE運転時のRS-485所有。

`follow_extreme_angular_radps`およびpause/resumeは`icart_mini_ros2`側の対応と組み合わせて使用する。関連リポジトリを別々に展開する場合は、受け側を先に反映する。

## 今後のTODO

- `icart_mini_ros2`側の未コミット変更を内容別に整理し、テスト後にコミットする。
- DNE用`rs485Handler.py`の速度上限とRS-485ブリッジ統合を完了する。
- BLVD10KMの通信タイムアウト停止と定期ハートビートを設計・実装する。
- Raspberry Pi停止、プロセス強制終了、USB-RS485切断時の停止を実機確認する。
- PROPO用の専用モード入力を決定し、TANGへ統合する。
- 通常停止と異常時瞬時停止の使い分けを実機評価する。
