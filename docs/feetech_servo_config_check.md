# 実機確認手順: サーボ設定（EEPROM／SRAM）の読み取り（段階2b-3a）

対象: `hardware:=feetech`（読み取り専用）の起動時診断 `[diag] ... EEPROM: ... | SRAM: ...`。
**サーボには何も書かない。トルクにも触れない。** 腕を動かす操作は含まない（ただし手順 2 でサーボの電源を切るので、腕が落ちる。支えること）。

## 目的
段階2b-3（書き込み）の前に、次の 3 つを実機で確かめる。
1. **EEPROM と SRAM の区分の確認**（推定: アドレス 39 以下は EEPROM で電源を切っても残る。40 以降は SRAM で電源を入れ直すと既定値に戻る）。特に P/I/D（LeRobot が 16 / 0 / 32 に変更）、Operating_Mode、Return_Delay_Time、gripper の Max_Torque_Limit / Protection_Current / Overload_Torque。
2. **電源投入時の既定値**: `Acceleration`、`Torque_Limit`、`Goal_Velocity`、`Lock`、`Torque_Enable`（2b-3b の初期値を決める根拠）。
3. 今のサーボが LeRobot の値と一致しているか（不一致は WARN で出る。read-only なので起動は続く）。

## 0. 準備
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_so-arm101/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
mkdir -p ~/so_arm101_check
cd ~/ros2_so-arm101 && colcon build --symlink-install     # 最新のビルドであること
```
- 同じシリアルポートは同時に開けない。LeRobot 経路と feetech 経路は同時に起動しない。
- conda を activate していない新しいターミナルを使う。

## 1. 記録 A: LeRobot が設定した直後（電源を切る前）
LeRobot の `configure()` を一度走らせて、設定を書かせた直後の状態を読む。

1. 腕を折りたたみ姿勢にして、片手を添える。
2. LeRobot 経路を起動して止める（`LECONDA_SITE_PACKAGES` を設定した端末で）:
   ```bash
   ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=lerobot use_rviz:=false 2>&1 | tee ~/so_arm101_check/servo_config_lerobot_launch.log
   ```
   `joint_state_broadcaster` が active になって `/joint_states_raw` が来たら Ctrl-C。**停止でトルクが OFF になり、腕が脱力する**ので支える。
3. 続けて feetech（読み取り専用）を起動して、診断の出力を保存し、数秒後に Ctrl-C（新しい端末）:
   ```bash
   ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech use_rviz:=false 2>&1 | tee ~/so_arm101_check/servo_config_A.log
   ```
   `[diag] ... EEPROM: ...` が 6 行と `servo settings vs LeRobot: n/6 joints match` が出る。

## 2. 電源の入れ直し
1. 腕を折りたたみ姿勢にして**しっかり支える**（サーボの電源を切ると、保持されなくなる）。
2. **サーボの電源（外部電源。USB ケーブルだけではない）を切って 10 秒以上待つ**。USB は挿したままでよい（ポート名が変わらない）。
3. 電源を入れる。

## 3. 記録 B: 電源投入直後
LeRobot は起動しない（設定を書き直さないため）。**電源を入れた直後に、feetech（読み取り専用）だけを起動する**:
```bash
ls -l /dev/ttyACM0                 # ポートがあること
ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech use_rviz:=false 2>&1 | tee ~/so_arm101_check/servo_config_B.log
```
数秒後に Ctrl-C。

## 4. 抜き出して報告する
```bash
for f in A B; do echo "== $f"; grep -a "servo settings\|EEPROM:" ~/so_arm101_check/servo_config_$f.log | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-600; done
grep -a "READ-ONLY\|ERROR\|cannot" ~/so_arm101_check/servo_config_B.log | head
```
`servo_config_A.log`、`servo_config_B.log`、`servo_config_lerobot_launch.log` の 3 つのパスと、上の出力を報告する。

## 5. 結果の見方
各行: `EEPROM: operating_mode P I D return_delay max_torque_limit protection_current overload_torque | SRAM: torque_enable lock acceleration torque_limit goal_velocity | other: maximum_acceleration status voltage temperature`。

| 項目 | A と B で同じ | A と B で違う |
|---|---|---|
| P / I / D、Operating_Mode、Return_Delay、gripper の 3 項目 | **EEPROM と確認**（電源を切っても残る）。LeRobot の値のまま（A が MATCH なら B も MATCH） | 想定外。区分の推定が違う。2b-3 の設計を見直す |
| `acceleration`、`torque_limit`、`goal_velocity`、`lock`、`torque_enable` | 電源を切っても残る（SRAM の推定と違う）。設計を見直す | **SRAM と確認**。B の値が電源投入時の既定値 |
| `maximum_acceleration` | 同じなら EEPROM 相当 | 違えば SRAM 相当（区分が未確認だった項目の確認） |

- A の `acceleration` は 254（LeRobot が書いた値）のはず。B の値が既定値。
- B の `torque_limit`（電源投入時の値）が 1000 なら、「腕の関節は LeRobot でも Torque_Limit を下げていない」という前提と一致。
- B の `goal_velocity` が 0 なら、2b-3 で書かなくてよい根拠の 1 つ（0 の意味は別途確認）。
- B で LeRobot の値と違う項目があると `DIFFERS from LeRobot values: ...` の WARN が出る。**read-only なので起動は続く**。書き込みモード（2b-3b）では、同じ状況は起動の中止になる。
- `status`、`voltage`、`temperature` は参考（`status` が 0 以外ならサーボの保護が働いている）。
- `cannot read servo settings (...)` が出たら通信の問題（電源投入直後でサーボがまだ起動していない場合は数秒待って再実行）。

## 6. 報告してほしいもの
- A と B の診断行（上の抜き出し）
- WARN／ERROR があればその行
- 想定と違う挙動（腕が動いた、ポートが開けなかった、など）
