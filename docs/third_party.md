# 第三者ソフトウェアの利用と出典

so_arm101 のドライバ（`so_arm101_hardware` の `Sts3215System`）は **LeRobot に依存しない**（import も実行時依存も無い）。
ただし実装にあたり LeRobot 0.4.2 と Feetech SDK の仕様・式を参照した。ここに出典とライセンスを記録する。

## LeRobot 0.4.2

- 場所: `~/miniforge3/envs/ros2_lerobot/lib/python3.10/site-packages/lerobot/`（参照のみ）
- プロジェクト: https://github.com/huggingface/lerobot
- ライセンス: **Apache License 2.0**（Copyright 2024 The HuggingFace Inc. team）。https://www.apache.org/licenses/LICENSE-2.0
- LeRobot のソースコードそのものは so_arm101 のリポジトリにコピーしていない。以下の「式・数値・仕様」を C++ で再実装した。

| 利用したもの | LeRobot 0.4.2 の出典（ファイル / 関数・表） | so_arm101 での場所 |
|---|---|---|
| raw → 正規化値の式（angle: DEGREES、gripper: RANGE_0_100 と範囲 clamp） | `lerobot/motors/motors_bus.py` `MotorsBus._normalize()`（770-797 行） | `src/so_arm101_hardware/src/calibration.cpp` `raw_to_rad()` |
| モータ定義（angle 5 関節 = DEGREES、gripper = RANGE_0_100） | `lerobot/robots/so101_follower/so101_follower.py` `SO101Follower.__init__`（48-59 行） | `config/follower_calibration.yaml` の `kind` |
| 度 → rad（gripper の 0..100 も度として扱う） | so_arm101_lerobot_bridge の `bridge.py` 132-133 行（`math.radians`）。LeRobot 本体ではなく、元は nimiCurtis/so101_ros2（MIT） | 同上 |
| STS3215 のレジスタ表（アドレス・バイト数） | `lerobot/motors/feetech/tables.py` `STS_SMS_SERIES_CONTROL_TABLE`（41-101 行） | `include/so_arm101_hardware/sts3215_bus.hpp` `reg::`（読み取りに使う 6 つだけ） |
| 機種番号 777 | 同 `MODEL_NUMBER_TABLE["sts3215"]` | 同 `kModelNumberSts3215` |
| sign-magnitude の符号ビット（Present_Position = 15、Homing_Offset = 11） | 同 `STS_SMS_SERIES_ENCODINGS_TABLE`（207-215 行） | 同 `kPositionSignBit`, `kHomingOffsetSignBit` |
| sign-magnitude のデコード | `lerobot/motors/encoding_utils.py` `decode_sign_magnitude()` | `src/sts3215_bus.cpp` `decode_sign_magnitude()` |
| `Present_Position = Actual_Position − Homing_Offset`（homing_offset は EEPROM 側の値） | `lerobot/motors/feetech/feetech.py` `_get_half_turn_homings()` の docstring（283-292 行） | `docs/inventory.md`（設計メモ）、yaml の `eeprom.*`（診断専用） |
| 変換の正解表の生成 | `MotorsBus._normalize()` を実際に呼ぶ（接続せず、シリアルは開かない） | `tools/gen_conversion_golden.py` → `src/so_arm101_hardware/test/data/conversion_golden.csv` |
| 比較用の読み取り専用 bus | `FeetechMotorsBus.connect()` と `sync_read("Present_Position")` のみ | `tools/lerobot_readonly_state.py`（比較専用。ドライバは依存しない） |
| 逆変換 rad → raw（Goal_Position 用。段階2b-3a で追加、まだ書き込みには使わない） | `lerobot/motors/motors_bus.py` `MotorsBus._unnormalize()`（angle: `val·max_res/360 + mid`、gripper: 0..100 に clamp して min..max）と、bridge の `radians_to_normalized`（`math.degrees`）。**最後の丸めだけ四捨五入**（LeRobot は `int()` の切り捨て。差は 0 か +1 tick） | `src/so_arm101_hardware/src/calibration.cpp` `rad_to_raw()` |
| サーボ設定の期待値（段階2b-3a。**比較して表示するだけ**で、書かない） | `so101_follower.py` `SO101Follower.configure()`（149-171 行: Operating_Mode=0、P=16、I=0、D=32。gripper のみ Max_Torque_Limit=500、Protection_Current=250、Overload_Torque=25）、`feetech.py` `configure_motors()`（222-230 行: Return_Delay_Time=0、Maximum_Acceleration=254、Acceleration=254） | `src/so_arm101_hardware/src/servo_config.cpp` `expected_servo_config()`（Maximum_Acceleration と Acceleration は表示のみで比べない） |
| 追加で読むレジスタのアドレス（Operating_Mode 33、P/I/D 21/23/22、Max_Torque_Limit 16、Protection_Current 28、Overload_Torque 36、Torque_Enable 40、Acceleration 41、Goal_Velocity 46、Torque_Limit 48、Lock 55、電圧 62、温度 63、Status 65、Maximum_Acceleration 85）と EEPROM／SRAM の区分 | `lerobot/motors/feetech/tables.py` `STS_SMS_SERIES_CONTROL_TABLE`（`# EPROM`／`# SRAM`／`# Factory` の見出し） | `include/so_arm101_hardware/sts3215_bus.hpp` `reg::`（読み取りに使う） |
| 逆変換の正解表の生成 | `MotorsBus._unnormalize()` を実際に呼ぶ（接続せず、シリアルは開かない） | `tools/gen_conversion_golden.py` → `src/so_arm101_hardware/test/data/unnormalize_golden.csv` |
| 書き込む設定（段階2b-3b）。**書くのは `apply_servo_config:=true` のときの EEPROM（P/I/D、Return_Delay_Time、gripper の 3 項目。LeRobot が書く値と同じ）と、SRAM の Acceleration / Torque_Limit / Torque_Enable / Lock / Goal_Position**。Operating_Mode・Goal_Velocity・Maximum_Acceleration は書かない | 値と順序（Lock=0 → 設定 → Torque_Enable=1 → Lock=1）は `so101_follower.py` `configure()` と `feetech.py` `configure_motors()`、`enable_torque()` / `disable_torque()`（296-310 行）。Acceleration の値は LeRobot の 254 ではなく **50（このドライバの既定。単位は未確認）** | `src/so_arm101_hardware/src/sts3215_write_bus.cpp`、`sts3215_system.cpp` `activate_write_mode()` / `shutdown_servos()` |
| 書き込みパケット（WRITE 0x03 と SYNC_WRITE 0x83）の形式 | `scservo_sdk` の `protocol_packet_handler.py`（`writeTxRx`、`syncWriteTxOnly`）を読んで仕様として確認（コードはコピーしていない） | `src/sts3215_write_bus.cpp`（試験 `test_write_bus.cpp` で長さ・チェックサムを照合） |
| キャリブレーション用の EEPROM の書き込み（段階2c-1。`so_arm101_calibration` の `restore`。Homing_Offset と Min/Max_Position_Limit だけ） | `feetech.py` `write_calibration()`（273-280 行: Homing_Offset、Min_Position_Limit、Max_Position_Limit の順に書く）、`motors_bus.py` `reset_calibration()`（672-684 行）、`disable_torque()` / `enable_torque()`（Lock の扱い）。レジスタの表と bit11 の符号は上と同じ。コードはコピーしていない。**LeRobot と違う点**: `reset_calibration` の中間書き込み（0 / 0 / 4095）をしない、各項目を読み戻して確認する、終了時に Lock=1、失敗時の復元 | `so_arm101_calibration/src/calib_write_bus.cpp`、`restore.cpp`（手順は `docs/stage2c_plan.md`） |
| 終了時の Lock=1 | LeRobot は Lock=0 のまま残す（`disable_torque`）。**LeRobot と違う（EEPROM を保護するため、このプロジェクトで追加）** | `shutdown_servos()` |

`tools/gen_conversion_golden.py` と `tools/lerobot_readonly_state.py` は開発・比較用で、`lerobot` を import する。ビルドにも実行時のドライバにも含まれない。
`conversion_golden.csv` と `unnormalize_golden.csv` は LeRobot の `_normalize()` / `_unnormalize()` の出力（数値）であり、LeRobot のコードを含まない。

**STS3215 の仕様書**（`doc.feetech.cn`）は参照元だが、原本は確認していない。レジスタの意味・単位（例: Acceleration の単位）は LeRobot の `tables.py` とコメントからの引用で、実機での確認は未了（`docs/feetech_servo_config_check.md`）。

## Feetech SDK / プロトコル仕様

- パケット形式・命令コード（PING=1, READ=2, WRITE=3, SYNC_READ=0x82, SYNC_WRITE=0x83, BROADCAST_ID=0xFE）・チェックサム・SYNC_READ の要求形式は、LeRobot が使う Python SDK
  `scservo_sdk`（pip の `feetech-servo-sdk 1.0.0`、https://github.com/Adam-Software/FEETECH-Servo-Python-SDK、License: UNLICENSE）の
  `protocol_packet_handler.py`（`txPacket` / `rxPacket` / `syncReadTx`）と `scservo_def.py` を読んで、仕様として確認した。コードはコピーしていない。リンクもしていない。
- STS3215 のコントロールテーブルの原典（LeRobot の `tables.py` が参照）: http://doc.feetech.cn/#/prodinfodownload?srcType=FT-SMS-STS-emanual-229f4476422d4059abfb1cb0

## so101_ros2（nimiCurtis）

段階1で取り込んだ `so101_ros2` 由来のパッケージ（MIT）の LICENSE は `LICENSE.so101_ros2`。

## 削除したもの（2026-11-12、段階 L2。`git rm`。archive は作っていない）

LeRobot 経路と、そのための道具・ブリッジを、リポジトリから削除した。**戻すときは、タグ `stage2c-done`（コミット 1a3628e）から取り出せる**（例: `git checkout stage2c-done -- src/so_arm101_lerobot_bridge`、`git checkout stage2c-done -- tools/gen_conversion_golden.py`）。

- 上の表の `tools/gen_conversion_golden.py`・`tools/lerobot_readonly_state.py`（および、表にない `tools/convert_lerobot_calibration.py`）は**削除した**。上の「開発・比較用で」の段落は、**削除前の説明**（歴史的）。再生成が必要なら、タグから取り出す。
- 正解表 `conversion_golden.csv`・`unnormalize_golden.csv` は**残している**（LeRobot 0.4.2 の出力の数値。Apache-2.0 の帰属はこの文書とヘッダに残す。ヘッダの文言の中立化は L3）。
- 表の「so_arm101_lerobot_bridge の `bridge.py`」の行は、削除したパッケージ内のコードへの言及（元は nimiCurtis/so101_ros2、MIT）。
- `LICENSE.so101_ros2`（nimiCurtis、MIT）は、description の urdf・meshes・controller の出所が確定するまで残す。

## SO-ARM100（TheRobotStudio）の URDF とメッシュ（2026-11-12、URDF の置き換え）

- 場所: `src/so_arm101_description/third_party/SO-ARM100/`（`so101_new_calib.urdf`、`LICENSE`、`SOURCE.md`、`SHA256SUMS`）と `src/so_arm101_description/meshes/`（13 個）。
- 取得元: https://github.com/TheRobotStudio/SO-ARM100 の `Simulation/SO101/so101_new_calib.urdf`。ファイルの最終変更は `385e8d7c68e24945df6c60d9bd68837a4b7411ae`（2025-07-02）。取得時の HEAD は `a758567c3978dfeefe282ede0500085a48fe8f78`。URDF の sha256 は `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c`。
- ライセンス: **Apache License 2.0**（`third_party/SO-ARM100/LICENSE` に同じ版のコピー。NOTICE は無い）。
- 扱い: URDF と `meshes/*.stl`（13 個。385e8d7 の `Simulation/SO101/assets/*.stl` とバイト一致）は**無変更**。`SHA256SUMS` と試験（`urdf_original_checksums`）で保護。インストールされる URDF は、ビルド時に `cmake/gen_urdf.cmake` が**メッシュのパスの接頭辞だけ**を置換して生成する別ファイル（先頭に変更の旨のコメント 1 行）。試験 `urdf_generated_matches_original` が、逆置換でオリジナルとバイト一致することを確かめる。
- 以前の URDF は nimiCurtis/so101_ros2（MIT）経由のコピー。内容は TheRobotStudio の 385e8d7 と同一（`docs/original_urdf_plan.md`）。

## 実行時に使う apt パッケージ（配布しない。2026-11-16、カメラの段 C1-a）

`so_arm101_camera` の launch が、次の apt パッケージを実行時に起動する。コードはリポジトリに入れていない（コピーも改名もしない）。リポジトリに入れるのは、それぞれに渡す yaml（`config/realsense.yaml`、`config/apriltag.yaml`）だけで、yaml は新しく書いた（apt の例のコメント・本文は写していない）。そのため `THIRD_PARTY_NOTICES.md` には足さない（計画 `docs/camera_stage_plan.md` 5 章）。

- `realsense2_camera` 4.58.4（Apache License 2.0）。`rs_launch.py` を include する。
- `apriltag_ros` 3.4.0（MIT、Christian Rauch）。`apriltag_node` を起動する。
