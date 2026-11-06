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

`tools/gen_conversion_golden.py` と `tools/lerobot_readonly_state.py` は開発・比較用で、`lerobot` を import する。ビルドにも実行時のドライバにも含まれない。
`conversion_golden.csv` は LeRobot の `_normalize()` の出力（数値）であり、LeRobot のコードを含まない。

## Feetech SDK / プロトコル仕様

- パケット形式・命令コード（PING=1, READ=2, WRITE=3, SYNC_READ=0x82, SYNC_WRITE=0x83, BROADCAST_ID=0xFE）・チェックサム・SYNC_READ の要求形式は、LeRobot が使う Python SDK
  `scservo_sdk`（pip の `feetech-servo-sdk 1.0.0`、https://github.com/Adam-Software/FEETECH-Servo-Python-SDK、License: UNLICENSE）の
  `protocol_packet_handler.py`（`txPacket` / `rxPacket` / `syncReadTx`）と `scservo_def.py` を読んで、仕様として確認した。コードはコピーしていない。リンクもしていない。
- STS3215 のコントロールテーブルの原典（LeRobot の `tables.py` が参照）: http://doc.feetech.cn/#/prodinfodownload?srcType=FT-SMS-STS-emanual-229f4476422d4059abfb1cb0

## so101_ros2（nimiCurtis）

段階1で取り込んだ `so101_ros2` 由来のパッケージ（MIT）の LICENSE は `LICENSE.so101_ros2`。
