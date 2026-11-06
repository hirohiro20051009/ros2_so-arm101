# 実機確認手順: hardware:=feetech（読み取り専用）と LeRobot 経路の比較（段階2b-2）

対象: `bringup.launch.py mock:=false hardware:=feetech`（STS3215 を直接読む。レジスタには一切書き込まない。トルクにも触れない）。
目的: 「`so101_follower.json` を使った LeRobot 経由の `/joint_states`」と、自前ドライバ（`follower_calibration.yaml`）の `/joint_states` が一致することを確認する。
**実機を動かす操作は含まない。** 腕を動かすのは、ユーザーが手で動かす場合だけ。

## 0. 準備（ターミナルごと）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_so-arm101/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
mkdir -p ~/so_arm101_check          # 記録の保存先（リポジトリの外）
```

- 同じシリアルポートは同時に開けない。LeRobot 経路と feetech 経路は**同時に起動しない**（feetech 側は `TIOCEXCL` で排他にしているので、他が開いていれば起動時に失敗する）。
- ビルド済みであること: `cd ~/ros2_so-arm101 && colcon build --symlink-install`。

## 1. 基準を取る（LeRobot 経路、折りたたみ姿勢 P1）

腕を **home の折りたたみ姿勢**（機械的に静止する姿勢）にしておく。

ターミナル1（LeRobot 用の環境変数を設定して起動）:
```bash
export LECONDA_SITE_PACKAGES=/home/hirok/miniforge3/envs/ros2_lerobot/lib/python3.10/site-packages
ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=lerobot use_rviz:=false
```
`joint_state_broadcaster` が active になり、`Failed to send commands` が出ていないことを確認する。bridge は接続時にトルクを ON にする（腕が保持される）。

ターミナル2（約 5 秒、動かさずに記録）:
```bash
python3 ~/ros2_so-arm101/tools/record_joint_states.py --seconds 5 --out ~/so_arm101_check/lerobot_P1.csv
```

**停止する**: 腕に片手を添えてから、ターミナル1で Ctrl-C。bridge は終了時にトルクを OFF にする（`disable_torque_on_disconnect: true`）ので**腕が脱力する**。
折りたたみ姿勢なら機械的に止まるはず。そのまま 2 秒待つ（脱力で数 tick ずれることがある）。

## 2. feetech で同じ姿勢を記録して比較する

**新しいターミナルを使う（conda を activate していない）。** 手順 6 の確認もここで行う。

```bash
echo "CONDA_PREFIX=[$CONDA_PREFIX] LECONDA_SITE_PACKAGES=[$LECONDA_SITE_PACKAGES]"   # どちらも空であること
source /opt/ros/humble/setup.bash && source ~/ros2_so-arm101/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech use_rviz:=false 2>&1 | tee ~/so_arm101_check/feetech_launch.log
```

腕は動かさない（手順 1 の停止後の姿勢のまま）。別ターミナルで記録して比較する:
```bash
python3 ~/ros2_so-arm101/tools/record_joint_states.py --seconds 5 --out ~/so_arm101_check/feetech_P1.csv
python3 ~/ros2_so-arm101/tools/compare_joint_states.py ~/so_arm101_check/lerobot_P1.csv ~/so_arm101_check/feetech_P1.csv
```

### 合否の見方（`compare_joint_states.py`）
- tick 換算: angle 関節 = 2π/4095 rad、gripper = yaml の range_min/max から（100% ÷ (max−min) を度とみなして rad）。
- 平均差 = |mean(A) − mean(B)|、最大差 = A と B の全サンプルの組合せで最悪の差。
- **合格: 平均差 ≤ 1 tick かつ 最大差 ≤ 2 tick**（関節ごとに PASS/FAIL、全関節 PASS で終了コード 0）。
- `std A / std B` は各記録中の標準偏差 [tick]。1 を超えると「静止していない可能性」と注意が出る（腕が動いている・脱力でずれている）。その場合は取り直す。
- NaN を含む関節は FAIL（ドライバが読み取りに失敗した）。

## 3. 起動時診断のログの見方

`ros2 launch` の出力（`feetech_launch.log`）に、`[Sts3215System]` の行が出る。

```
[INFO] [Sts3215System]: initialized: port=/dev/ttyACM0 calibration=...follower_calibration.yaml [converted_from_lerobot_json, ... file_date=2026-09-07] ...
[INFO] [Sts3215System]: [diag] shoulder_pan id=1 model=777 return_delay=0 homing_offset=135 (yaml 135) min_limit=743 (yaml 743) max_limit=3452 (yaml 3452): MATCH
 ... （6 関節ぶん）
[INFO] [Sts3215System]: [diag] EEPROM vs yaml: 6/6 joints match
[WARN] [Sts3215System]: READ-ONLY mode: servo registers (including torque) are never written; commands are DISCARDED. The arm will not move.
```

| ログ | 意味 | 対応 |
|---|---|---|
| `[diag] ...: MATCH` | サーボの EEPROM（Homing_Offset、Min/Max_Position_Limit）が yaml と一致。変換は json 作成時と同じ前提 | 問題なし |
| `[diag] ...: MISMATCH homing_offset(servo A, yaml B)` | サーボの値が yaml と違う。**変換値が実機とずれる可能性が高い**（衝撃で offset がリセットされた、別のキャリブレーションが書かれた、など）。ドライバは止まらず読み続ける | 比較結果を信用しない。どの関節か控えて報告する。`test/readme.txt` の `lerobot-calibrate` 手順（または段階2c）でキャリブレーションし直す |
| `model=777` 以外 / `cannot read servo (...)` | 機種違い・応答なし（電源、ケーブル、ID） | 配線・電源を確認。ドライバは起動を中止する |
| `cannot open serial port: ... (port in use?)` | 他のプロセス（LeRobot の bridge など）が同じポートを開いている | 他を止める |
| `READ-ONLY mode ... DISCARDED` | 起動時に 1 回。読み取り専用で腕は動かないという警告 | 想定どおり |
| `read failed (...); consecutive=n/10; holding previous values` | 読み取りに失敗（タイムアウト、チェックサム不良、ID 違い等）。最初の 1 回と 10 回ごとに警告。1 回は自動で再試行している | 頻発するならケーブル・電源・ボーレートを確認 |
| `read recovered after n consecutive failures` | 失敗から復帰 | 回数が多ければ通信品質を疑う |
| `read failed 10 times in a row ... giving up` | 連続 10 回失敗。ハードウェアが `unconfigured` になり、`/joint_states` は **NaN** になる（古い値は流れない） | launch をやり直す。原因（切断・電源断）を確認 |
| `READ-ONLY: command for '...' differs ... and is discarded` | 軌道などの指令が来たが破棄した（1 回だけ表示） | Plan & Execute をしていないか確認 |

`return_delay` は参考表示（LeRobot が過去に書いた EEPROM の値。0 なら最短応答）。

## 4. 重力の影響を受けない関節での比較（shoulder_pan、wrist_roll、gripper）

折りたたみ姿勢だけだと、可動域の中央付近の値しか確認できない。pan・roll・gripper は重力で姿勢が変わらない（摩擦で保持される）ので、手で動かした姿勢で、LeRobot と feetech を順に読んで比較できる。

LeRobot の bridge は接続時にトルクを ON にして手で姿勢を作れないので、**読み取り専用の比較スクリプト**を使う（`bus.connect()` の ping とレジスタ読み出しと `sync_read` だけ。configure もトルク操作も書き込みも不可能にしてある。終了時もトルクに触れない）。

1. 手順 1〜2 の後、いずれの launch も止まっていること。腕は脱力しているので、折りたたみ姿勢で支える。
2. 手で動かす: shoulder_pan を片側へ約 30°、wrist_roll を約 60°、gripper を半開き。**他の関節（lift、elbow、wrist_flex）は折りたたみのまま動かさない**。手を離しても pan・roll・gripper が動かないことを確認する。
3. LeRobot 側を記録（conda の Python を使う。ROS は不要）:
   ```bash
   ~/miniforge3/envs/ros2_lerobot/bin/python ~/ros2_so-arm101/tools/lerobot_readonly_state.py --seconds 5 --out ~/so_arm101_check/lerobot_P2.csv
   ```
4. 腕に触れずに、手順 2 の feetech を起動して記録・比較する（出力ファイル名を `feetech_P2.csv` にする）:
   ```bash
   python3 ~/ros2_so-arm101/tools/record_joint_states.py --seconds 5 --out ~/so_arm101_check/feetech_P2.csv
   python3 ~/ros2_so-arm101/tools/compare_joint_states.py ~/so_arm101_check/lerobot_P2.csv ~/so_arm101_check/feetech_P2.csv
   ```
5. 判定は **shoulder_pan、wrist_roll、gripper の 3 行**を見る（他の 3 関節は脱力で数 tick 動くことがあるので参考）。
6. 逆向きの姿勢（pan を反対側へ約 30°、roll を反対側へ約 60°、gripper を閉じる）でも 2〜5 を繰り返す。符号が逆なら差が大きく出る。

## 5. hardware:=feetech では RViz の Plan & Execute を実行しない

- 読み取り専用なので、**指令は破棄され腕は動かない**。軌道は（open_loop のため）「成功」と表示されることがあるが、実機は動かない。実行しても意味がなく、混乱のもとになる。
- 確認は `use_rviz:=false` で行う（`/joint_states` の記録だけ）。RViz を開く場合も、表示を見るだけにする。
- 書き込みモード（トルク ON、Goal_Position）は段階2b-3 で別途承認してから実装する。

## 6. hardware:=feetech が LeRobot を使っていないことの確認

手順 2 の launch を起動したまま、別ターミナルで（**conda を activate していないこと**）:

```bash
# (1) 環境: どちらも空、python3 は /usr/bin/python3
echo "CONDA_PREFIX=[$CONDA_PREFIX] LECONDA_SITE_PACKAGES=[$LECONDA_SITE_PACKAGES]"; which python3

# (2) lerobot / conda / LeRobot bridge のプロセスが無い（出力が空であること）
ps -eo pid,args | grep -E 'follower_ros2_node|lerobot|miniforge3' | grep -v grep

# (3) bridge / follower / interface という名前のノードが無い（出力が空）
ros2 node list | grep -iE 'bridge|follower|interface'

# (4) LeRobot 経路のトピックが無い（出力が空）
ros2 topic list | grep -E 'joint_commands|joint_states_raw'

# (5) ros2_control_node が lerobot / conda のライブラリを読み込んでいない（0 であること）
P=$(pgrep -f '^/opt/ros/humble/lib/controller_manager/ros2_control_node'); echo "pid=$P"
grep -c -E 'lerobot|miniforge3' /proc/$P/maps

# (6) ドライバのライブラリが python / lerobot / conda に依存していない（出力が空）
ldd ~/ros2_so-arm101/install/so_arm101_hardware/lib/libso_arm101_sts3215.so | grep -iE 'python|lerobot|conda|miniforge'

# (7) シリアルポートを開いているのは ros2_control_node 自身
ls -l /proc/$P/fd | grep ttyACM
```

注意: `ps` に `python3 .../ros2 launch ...` は出る（ros2 launch 自体が Python）。見るのは **`lerobot` / `miniforge3` / `follower_ros2_node` を含むか**であって、python の有無ではない。

## 7. 報告してほしいもの
- `compare_joint_states.py` の出力（P1、P2 の両方）
- `feetech_launch.log` の `[diag]` 行（MATCH / MISMATCH）
- 手順 6 の出力
- 想定と違う挙動（例: `read failed` が頻発する、FAIL の関節がある）
