# 最小変更でのCuGoV4・TANG統合計画

## 方針

新しい制御ノード群は増やさず、既存の `TangController` 1ノードにモード選択、3種類の入力処理、RS-485モータ指令を集約します。

起動時は必ず `IDLE` とし、GPIOボタンが起動後に押されるまでモータ停止を維持します。起動時のGPIO状態、ジョイスティック中立、プロポ中立は確認しません。

## 状態遷移

- `IDLE`
  - 起動時の初期状態。
  - モータ停止、追従停止。
  - GPIO14のモード表示LEDは消灯する。
  - 起動前から押されていたボタンは採用せず、起動後の新しい押下でモード選択する。

- `MANUAL`
  - GPIO16のみを押すと遷移。
  - MCP3004から車載ジョイスティックを読み、その時点の操作量を即時反映する。
  - 異なるモードからMANUALへ遷移するたびに、速度設定を必ず低速に初期化する。
  - GPIO4を押下すると低速、GPIO3を押下すると高速を選択し、次の速度選択ボタン押下まで設定を記憶する。ボタンを離しても選択速度は維持する。
  - 低速上限は `0.15 m/s・0.6 rad/s`、高速上限は `0.30 m/s・1.0 rad/s`。
  - GPIO3/4同時押しは低速を優先する。
  - GPIO3/4の速度選択ボタンにも30〜50 msを目安とするチャタリング除去を設ける。
  - MANUAL以外でのGPIO3/4操作は無視し、次回MANUAL遷移時の低速初期化に影響させない。
  - GPIO14のモード表示LEDは消灯する。

- `FOLLOW`
  - GPIO21のみを押すと遷移。
  - `TangController` が `/joy` へ仮想ボタン7を送り、`icart_mini_leg_tracker` の追従を開始する。
  - ゲームパッドや `joy_node` は使用せず、`TangController` が `sensor_msgs/Joy` を仮想的に発行する。
  - `icart_mini_leg_tracker` の既存 `/cmd_vel` を使用する。
  - `/cmd_vel` コールバックではモータを直接動かさず、最新指令を保存するだけに変更する。
  - 0.5秒以上指令が届かなければ停止する。
  - 他モードへ移ると、`TangController` が先にモータを停止し、続けて `/joy` へ仮想ボタン6を送って追従処理を停止する。
  - 仮想ボタン番号は現行icartと共通の「追従開始=7、追従停止=6」とする。
  - GPIO14のモード表示LEDは連続点灯する。

- `PROPO`
  - GPIO16と21を150 ms以内に押すと遷移。
  - CH-C/D/Hから現在の操作量を読み、そのまま反映する。
  - GPIOは `CH-C=24、CH-D=23、CH-H=7` とする。
  - GPIO7は未使用コネクタへ裏配線する。
  - 3chのいずれかが100 ms以上途絶えた場合だけ停止し、復帰後は現在の操作量を即時反映する。
  - GPIO14のモード表示LEDは、0.5秒点灯・4.5秒消灯の5秒周期で点滅する。

起動後はGPIO16/21の両方が一度離されるまでモード選択を受け付けません。モード選択ボタンには30〜50 msを目安とするチャタリング除去を設けます。モード判定は最初の押下から150 ms待って両GPIOを確認します。現在と同じモードが再選択された場合は、停止、再初期化、追従開始信号の再送を行わず無視します。異なるモードへの切替時には一度停止指令を送り、GPIO14 LEDの点滅周期をリセットした後、選択モードの現在入力を反映します。

## 最小限のコード変更

### tang_ros2

- `TangController.mode` の初期値を `idle` に変更する。
- 現在のGPIO PWM用 `Motor` を、cugoの `Rs485DualMotorBridge` に置き換える。
- 既存のループ構造を維持し、`idle/manual/follow/propo` の4分岐にする。
- プロポ受信は既存 `RcPwmReceiver` と `VehicleControl` を再利用する。
- `/cmd_vel` はFOLLOW時だけ採用し、MANUAL・PROPO・IDLEでは無視する。
- モード変更時、終了時、例外時には必ず左右停止を送る。
- Modbus通信異常時は停止指令を試みて `IDLE` に戻し、再接続後もモードボタンの再操作を要求する。ただし、USB-RS485切断時は停止指令も送信できないため、今回の試作ではソフトウェアだけでモータ停止を保証しない。
- `/tang/mode` を `std_msgs/String` で発行し、現在モードを確認可能にする。
- 既存のLiDAR近距離停止処理は変更せず維持する。
- 旧GPIO PWM関連コードは直ちに削除せず、launchから使わない状態にして差分を抑える。

### cugo_rs485_motor_control

- 制御ロジック自体は変更しない。
- `propo_control.py` 内の `RcPwmReceiver`、`VehicleControl`、`Rs485DualMotorBridge` をTANGからimportできるよう、最小限のPythonパッケージ設定だけ追加する。
- 既存CLIの動作は維持する。
- `propo_control.py`、`TangController`、DNE用 `rs485Handler.py` は同時起動しない。RS-485を開くプロセスは常に1つだけにする。

車体パラメータは現行cugo設定を使います。

- 車輪半径：`0.03858 m`
- トレッド：`0.376 m`
- 減速比：20
- 左右符号：`-1 / +1`
- 最大回転数：2600 rpm
- 最低回転数：80 rpm
- 発進しきい値：120 rpm

### 参考：モードごとの速度上限

並進速度の `m/s` から `km/h` への変換は `m/s × 3.6` とする。現在の計画値および既存プログラムの設定値は次のとおりとする。

| モード | 最大並進速度 | 時速 | 最大角速度 | 値の根拠 |
| --- | ---: | ---: | ---: | --- |
| MANUAL低速 | `0.15 m/s` | `0.54 km/h` | `0.6 rad/s` | 本計画で定めるTANG用上限 |
| MANUAL高速 | `0.30 m/s` | `1.08 km/h` | `1.0 rad/s` | 本計画で定めるTANG用上限 |
| PROPO | `0.2625 m/s` | `0.945 km/h` | `1.57 rad/s` | 現行 `propo_control.py` のデフォルト値 |
| FOLLOW | `0.25 m/s` | `0.90 km/h` | `π/3 ≈ 1.047 rad/s` | 現行 `icart_mini_leg_tracker` の上限 |
| DNE | `0.417 m/s` | `1.5 km/h` | `80 deg/s ≈ 1.396 rad/s` | CuGo V4の公称最高速度 `1.8 km/h` より低い本計画のDNE用上限 |

PROPOのCH-Hは現行設定で出力倍率を約50〜100%の範囲で変更する。そのため並進速度の目安は、CH-H最小時が約 `0.131 m/s（0.473 km/h）`、CH-H最大時が `0.2625 m/s（0.945 km/h）` となる。

### 参考：速度を変更する場所

- MANUALの低速・高速上限は、統合実装時に `tang_control/tang_control/config.py` へモード別の並進速度・角速度設定として追加し、以後はそこで変更する。現行コードには本計画の `m/s`・`rad/s` 上限はまだ実装されていない。
- PROPOの現行デフォルトは `cugo_rs485_motor_control/scripts/propo_control.py` の `DEFAULT_MAX_V_KMH` と `DEFAULT_MAX_W_RADPS` で定義されている。単体CLIでは `--max-v-kmh` と `--max-w-radps` で一時的に変更できる。TANG統合後のPROPO上限は `tang_control/tang_control/config.py` へ集約し、通常運用で `propo_control.py` と二重管理しない。
- FOLLOWの指令生成側の上限は `icart_mini_ros2/icart_mini_leg_tracker/include/icart_mini_leg_tracker/leg_cluster_tracking.hpp` の `MAX_SPEED` と `MAX_TURN_SPEED` で定義されている。この値を変更した場合は `icart_mini_leg_tracker` の再ビルドが必要となる。
- TANG統合後は、FOLLOWも `tang_control/tang_control/config.py` に最終安全上限を持ち、`/cmd_vel` がそれを超えても `TangController` 側で制限する。追従アルゴリズムの速度特性を変える場合は追従ノード側、CuGoの最終上限だけを変える場合はTANG側を変更する。
- DNE運転の最大並進速度は `1.5 km/h（0.4167 m/s）` とする。統合時に `tang2dne_handler/scripts/rs485Handler.py` へDNE用の最終車体速度上限を追加し、DNEからこれを超える指令が届いてもRS-485ブリッジへ渡す前に制限する。
- `Rs485DualMotorBridge` の `max_rpm=2600` はモータ保護・車体変換上の最終上限であり、MANUAL・PROPO・FOLLOWの操作速度上限とは別の設定とする。モード速度を調整する目的で `max_rpm` を変更しない。
- DNEの `1.5 km/h` 制限は車体速度として適用し、`rs485_motor_bridge.py` のモータ保護上限 `max_rpm=2600` は変更しない。

速度設定を変更した際は、クローラを浮かせた状態で左右符号と回転数を確認した後、低速・無人区画で停止距離と旋回速度を再評価する。

### 起動構成

- TANG用launchではURG、`leg_cluster_tracking_node`、`TangController` のみ起動する。
- YPSpur、F710用teleop、旧GPIO PWMモータ制御は起動しない。
- `icart_mini_leg_tracker` のコードと `/cmd_vel` 名は変更しない。
- 自動起動スクリプトはTANG用launchを指定する。

### DNEとの関係

- DNEからの指令は `tang2dne_handler` の `rs485Handler.py` で受信し、同ハンドラからRS-485経由でCuGoを制御する。
- DNEから受信する並進速度指令は、`rs485Handler.py` で最大 `1.5 km/h（0.4167 m/s）` に制限してからRS-485ブリッジへ渡す。
- DNE制御は、MANUAL・FOLLOW・PROPOを扱う `TangController` とは別プログラム・別起動構成とする。
- DNE制御時は `TangController` を起動せず、TANG制御時は `rs485Handler.py` を起動しない。
- 入力処理とプログラムは独立とするが、RS-485ポートとモータの所有者は必ず1プロセスに限定する。
- 今回のTANG統合実装には、DNE指令の取り込みやDNE/TANG間のモード切替は含めない。
- DNE運転時に起動する制御プロセスは `tang2dne_handler/scripts/rs485Handler.py` だけとする。`cugo_rs485_motor_control` は同ハンドラからRS-485通信クラスをimportするライブラリとして使い、別プロセスとしては起動しない。
- `cugo_rs485_motor_control/scripts/main.py` はRS-485・モータ単体確認用CLIとし、DNE運転中は起動しない。`rs485Handler.py` と同時起動すると、両方が `/dev/ttyUSB0` を操作して競合するため禁止する。

DNE運転時の起動例は次のとおりとする。

```bash
cd /home/hashimoto/src/tang2dne_handler
python3 scripts/rs485Handler.py \
  --host 192.168.212.1 \
  --port-odm 18080 \
  --port-ctl 28080 \
  --robot CuGoV4
```

DNE運転時の起動区分は次のとおりとする。

- 起動する：`rs485Handler.py`
- 起動しない：`TangController`、`propo_control.py`、`cugo_rs485_motor_control/scripts/main.py`

## テスト

- RS-485なしのdry-runで、起動後に `IDLE` のまま動かないことを確認する。
- 起動前からGPIO16/21を押していてもモードが変わらず、一度離して押し直すと遷移することを確認する。
- GPIO16/21のチャタリングを模擬し、30〜50 msの範囲で設定した除去時間内の変化で複数回遷移しないことを確認する。
- GPIO16、GPIO21、両方押しの状態遷移を確認する。
- 現在と同じモードを再選択しても、停止、再初期化、追従開始信号の再送が発生しないことを確認する。
- GPIO14 LEDがIDLE・MANUALで消灯、FOLLOWで連続点灯、PROPOで0.5秒点灯・4.5秒消灯し、モード切替時に点滅周期がリセットされることを確認する。
- MANUAL遷移時は必ず低速で開始し、GPIO3・GPIO4の押下で選択した高速・低速設定がボタンを離した後も保持されることを確認する。
- GPIO3/4の同時押しで低速が優先され、チャタリングで速度設定が複数回切り替わらないことを確認する。
- MANUAL以外でGPIO3/4を押しても速度設定が保持されず、次回MANUAL遷移時は低速で開始することを確認する。
- DNEから `1.5 km/h（0.4167 m/s）` を超える並進速度指令を入力しても、RS-485ブリッジへ渡される値が正負とも `0.4167 m/s` 以内に制限されることをdry-runで確認する。
- FOLLOW遷移時に `/joy` へ仮想ボタン7、FOLLOW離脱時に仮想ボタン6が発行されることを確認する。
- MANUAL・PROPO選択時、倒れたスティックの値が即時反映されることを確認する。
- 選択されていない入力と `/cmd_vel` では動かないことを確認する。
- クローラを浮かせ、左右モータ方向と追従時の `angular.z` 符号を確認する。追従符号の初期値は現行互換の反転とし、結果をパラメータへ固定する。
- プロポ電源OFF、追従ノード停止、TangControllerの正常終了時に停止することを確認する。
- USB-RS485切断は、通信タイムアウト実装後のフェイルセーフテストで実施し、今回の走行テスト中には意図的に切断しない。
- 最後に低速・無人区画で走行し、既設のハードウェア非常停止を確認する。

## 前提

- MR-8のPWM信号は3.3Vなので、GPIO7へ直接入力する。
- GPIO23/24はLCDと共用せず、GPIO7の裏配線先も他用途に使わない。
- TANG統合時はプロポCH-HをGPIO7に接続し、GPIO14はモード表示LED専用とする。
- モード選択後の中立確認は行わない。
- 今回の試作実装では、BLVD10KMの通信タイムアウト設定と通信ハートビート実装は対象外とする。
- 通信タイムアウトを実装するまでは、Raspberry Pi停止、プロセス強制終了、USB-RS485切断時の自動停止は保証できない。試験時は必ず既設のハードウェア非常停止を使用できる状態にする。

## 今後のTODO

- BLVD10KMの通信タイムアウト停止を有効化し、Raspberry Pi停止やUSB-RS485切断時にドライバ単体で停止できるようにする。
- 通信タイムアウトを有効化する際は、正常時にタイムアウトしないよう、左右両ドライバへの定期ハートビート通信を実装する。
- 通信タイムアウトで発生するドライバアラームの確認、左右停止、アラームリセット、再操作要求までを一連の安全復帰手順として設計・テストする。
- 通信タイムアウト実装後、クローラを浮かせた状態からUSB-RS485切断、Raspberry Pi停止、プロセス強制終了を順に試験し、規定時間内に左右とも停止することを確認する。
