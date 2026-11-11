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

## 0.5 LeRobot 側の記録には何を使うか（launch か、読み取り専用ツールか）と、腕のトルク

LeRobot 側の基準は、**姿勢によって使うものが違う**。

| 姿勢 | LeRobot 側の記録に使うもの | 理由 |
|---|---|---|
| P1: home の折りたたみ姿勢 | **`hardware:=lerobot` の launch**（＋ `record_joint_states.py`） | 「LeRobot 経由の `/joint_states`」そのもの（比較の基準の定義）。bridge の変換経路ごと確認できる |
| P2: 手で動かした姿勢（pan・roll・gripper） | **`tools/lerobot_readonly_state.py`**（launch は使わない） | launch は接続時にトルクを ON にするので、手で姿勢を作ったり変えたりできない（ON の間は保持される）。このツールはトルクに触れないので、脱力したまま手で作った姿勢を読める |

各ケースでの**腕のトルク**と、サーボに書き込むもの:

| | 開始時 | 実行中 | 終了時 | サーボへの書き込み |
|---|---|---|---|---|
| `hardware:=lerobot` の launch | 接続時の `configure()` が一度トルクを OFF にして設定を書き、**最後に ON にする**（Torque_Enable=1, Lock=1） | **ON**（腕は保持される。指令 Goal_Position も毎周期書く） | Ctrl-C で **OFF**（Torque_Enable=0, Lock=0。`disable_torque_on_disconnect: true`）→ **腕が脱力する** | する（Return_Delay_Time、加速度、PID の P/I/D、gripper のトルク制限、Operating_Mode、トルク、Goal_Position） |
| `tools/lerobot_readonly_state.py` | **触れない**（接続時の状態のまま） | **触れない** | **触れない**（`disconnect(disable_torque=False)`） | しない（送るのは PING / READ / SYNC_READ だけ。下記のとおりバイト列で強制） |
| `hardware:=feetech`（自前ドライバ） | **触れない** | **触れない** | **触れない** | しない（READ / SYNC_READ だけ。型として書き込み命令が存在しない） |

つまり、手順 1 の launch を止めた後は**腕が脱力した状態**になり、その後のツールと feetech はどちらもその状態を変えない（脱力のまま）。
逆に、腕が保持されている（手で動かせない）状態でツールを実行すると、**ツールはトルクを切らない**ので保持されたまま。無理に動かさず、先に保持している側（launch など）を正常に終了させる。

`tools/lerobot_readonly_state.py` の書き込み禁止は二重:
1. **送信バイト列の検査**（最終防衛線）: SDK の `port_handler.writePort` をラップし、送るパケットの命令コードが PING(0x01) / READ(0x02) / SYNC_READ(0x82) 以外なら例外にする。公開メソッド、低レベルの `_write` / `_sync_write` / `_disable_torque`、SDK の直呼びのどれでも、書き込み命令はシリアルに出ない。
2. LeRobot の公開メソッド `write` / `sync_write` / `enable_torque` / `disable_torque` / `write_calibration` / `configure_motors` を例外にする（呼んだ箇所で早く失敗させる）。既定の `disconnect()`（トルクを OFF にする書き込みをする）もこれで例外になる。

## 1. 基準を取る（LeRobot 経路、折りたたみ姿勢 P1）

腕を **home の折りたたみ姿勢**（機械的に静止する姿勢）にしておく。

ターミナル1（LeRobot 用の環境変数を設定して起動）:
```bash
export LECONDA_SITE_PACKAGES=/home/hirok/miniforge3/envs/ros2_lerobot/lib/python3.10/site-packages
ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=lerobot use_rviz:=false 2>&1 | tee -i ~/so_arm101_check/lerobot_launch.log
```
（ログは必ず保存する。起動に失敗したときの原因調査に要る。3.5 節）
`joint_state_broadcaster` が active になり、`Failed to send commands` が出ていないことを確認する。bridge は接続時にトルクを ON にする（腕が保持される）。

ターミナル2（約 5 秒、動かさずに記録）:
```bash
python3 ~/ros2_so-arm101/tools/record_joint_states.py --seconds 5 --out ~/so_arm101_check/lerobot_P1.csv
```

**停止する**: 腕に片手を添えてから、ターミナル1で Ctrl-C。bridge は終了時にトルクを OFF にする（`disable_torque_on_disconnect: true`）ので**腕が脱力する**。
折りたたみ姿勢なら機械的に止まるはず。そのまま 2 秒待つ（脱力で数 tick ずれることがある）。

**（推奨）ツール自体の確認**: 腕は脱力した折りたたみ姿勢のまま、同じ姿勢で読み取り専用ツールを実行して、launch の記録と比べる。これで「ツールの値 ＝ bridge 経由の値」を確かめてから、手順 4（P2）でツールを基準に使える。
```bash
~/miniforge3/envs/ros2_lerobot/bin/python ~/ros2_so-arm101/tools/lerobot_readonly_state.py --seconds 5 --out ~/so_arm101_check/lerobot_P1_tool.csv
python3 ~/ros2_so-arm101/tools/compare_joint_states.py ~/so_arm101_check/lerobot_P1.csv ~/so_arm101_check/lerobot_P1_tool.csv
```
全関節 PASS なら、ツールは基準として使える。FAIL の関節があれば、手順 4 の前に原因（脱力による姿勢のずれなど）を確認する。このツールはポートを開く（他のプロセスが使っていないこと）。トルクには触れない。

## 2. feetech で同じ姿勢を記録して比較する

**新しいターミナルを使う（conda を activate していない）。** 手順 2.5 の確認もここで行う（launch を止める前に）。

```bash
echo "CONDA_PREFIX=[$CONDA_PREFIX] LECONDA_SITE_PACKAGES=[$LECONDA_SITE_PACKAGES]"   # どちらも空であること
source /opt/ros/humble/setup.bash && source ~/ros2_so-arm101/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech use_rviz:=false 2>&1 | tee -i ~/so_arm101_check/feetech_launch.log
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

## 2.5 hardware:=feetech が LeRobot を使っていないことの確認（手順 2 の launch 中に行う）

**手順 2 の launch を起動したまま**（止めると (3)〜(7) が確認できない）、別ターミナルで（**conda を activate していないこと**）:

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

## 3.5 `hardware:=lerobot` で `/joint_states` が NaN、または腕が固定されない

2026-11-10 に 1 回起きた（ログは残っていない。再起動で正常に戻った）。原因は未特定。コードから考えられる順:

1. **bridge（Python）が接続に失敗して落ちた**（最有力）。`bridge.py` の `robot.connect()` が例外で終了すると `joint_states_raw` が来ず、`so101_hardware_bridge.cpp` が NaN で初期化した位置が更新されないので `/joint_states` が NaN になる。`configure()` はトルク ON を最後に行うので、途中で落ちれば腕も固定されない。2 つの症状を 1 つで説明できる。接続失敗の原因候補: USB の切断・再列挙（落下の衝撃や電源の揺れ）、前のプロセスがポートを掴んでいる、サーボの無応答、`LECONDA_SITE_PACKAGES` 未設定による import 失敗。
2. `publish_joint_states` の例外（`Failed to publish joint states`）。
3. ウォッチドッグ `_alive` が「ROS 時間が進まない」と判断して終了（`ROS time not updating`）。
4. 起動順の競合。bridge が出れば回復するので、「再起動まで NaN」とは合いにくい。

再発したら、**launch を止める前に**（実機を動かさない、読み取りだけ）:
```bash
ros2 node list | grep -iE 'follower|bridge'       # 無ければ bridge が落ちている
ros2 topic hz /joint_states_raw                    # 30 Hz 来ているか
ros2 topic echo --once /joint_states               # NaN か
grep -nE 'Failed to connect|Failed to publish|ROS time not updating|Traceback|ImportError|Port is in use|Permission' ~/so_arm101_check/lerobot_launch.log
ls -l /dev/ttyACM*; fuser -v /dev/ttyACM0          # ポートを誰が掴んでいるか
dmesg | tail -30                                   # USB の切断・再列挙
```
結果と `lerobot_launch.log` を報告する。

## 4. 重力の影響を受けない関節での比較（shoulder_pan、wrist_roll、gripper）

折りたたみ姿勢だけだと、可動域の中央付近の値しか確認できない。pan・roll・gripper は重力で姿勢が変わらない（摩擦で保持される）ので、手で動かした姿勢で、LeRobot と feetech を順に読んで比較できる。

P2 では LeRobot 側の記録に **launch を使わず、読み取り専用ツール `tools/lerobot_readonly_state.py` を使う**（理由とトルクの扱いは 0.5 節）。launch は接続時にトルクを ON にして腕を保持してしまい、手で姿勢を作れない。ツールは `bus.connect()` の ping とレジスタ読み出しと `sync_read` だけを行い、configure もトルク操作も書き込みもしない（バイト列検査で強制）。終了時もトルクに触れない。

1. 手順 1〜2 の後、いずれの launch も止まっていること。腕は脱力しているので、折りたたみ姿勢で支える。
2. 手で動かす: shoulder_pan を片側へ約 30°、wrist_roll を約 60°、gripper を半開き（これを P2a とする）。**他の関節（lift、elbow、wrist_flex）は折りたたみのまま動かさない**。手を離しても pan・roll・gripper が動かないことを確認する。
3. LeRobot 側を記録（conda の Python を使う。ROS は不要）:
   ```bash
   ~/miniforge3/envs/ros2_lerobot/bin/python ~/ros2_so-arm101/tools/lerobot_readonly_state.py --seconds 5 --out ~/so_arm101_check/lerobot_P2a.csv
   ```
4. 腕に触れずに、手順 2 の feetech を起動して記録・比較する（出力名は `feetech_P2a.csv`。launch のログも `tee -i ~/so_arm101_check/feetech_launch_P2a.log` で保存する）:
   ```bash
   python3 ~/ros2_so-arm101/tools/record_joint_states.py --seconds 5 --out ~/so_arm101_check/feetech_P2a.csv
   python3 ~/ros2_so-arm101/tools/compare_joint_states.py ~/so_arm101_check/lerobot_P2a.csv ~/so_arm101_check/feetech_P2a.csv
   ```
5. 判定は **shoulder_pan、wrist_roll、gripper の 3 行**を見る（他の 3 関節は脱力で数 tick 動くことがあるので参考）。
6. 逆向きの姿勢 P2b（pan を反対側へ約 30°、roll を反対側へ約 60°、gripper を**大きく開く**）でも 2〜5 を繰り返す（`*_P2b.*`）。符号が逆なら差が大きく出る。gripper の閉じた状態は P1 で確認済みなので、P2b では開く側を見る。
7. ツールと feetech は**別の時刻**に読むので、1 tick 程度の差は手でひねった直後の揺らぎの可能性がある。`std A` も見る（2026-11-10 の P2a の wrist_roll は平均差 0.97 tick、LeRobot 側 std 0.18 で、P2b では 0.00 だった）。

## 5. hardware:=feetech では RViz の Plan & Execute を実行しない

- 読み取り専用なので、**指令は破棄され腕は動かない**。軌道は（open_loop のため）「成功」と表示されることがあるが、実機は動かない。実行しても意味がなく、混乱のもとになる。
- 確認は `use_rviz:=false` で行う（`/joint_states` の記録だけ）。RViz を開く場合も、表示を見るだけにする。
- 書き込みモード（トルク ON、Goal_Position）は段階2b-3 で別途承認してから実装する。

## 5.5 後始末の確認

後始末の確認: `ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher" | grep -v -E "grep|gvfsd"` と `ros2 node list` の両方で残りがないことを確認する。**`ros2 node list` に残っていても `ps` が空なら、`ros2 daemon stop; ros2 daemon start` の後にもう一度確認する**（move_group が Segmentation fault で終わると、daemon に古い一覧が残ることがある。推測）。それでも残るなら、ps に出ない原因を調べる。

## 6. 報告してほしいもの
- `compare_joint_states.py` の出力（P1、ツール自体の確認 `lerobot_P1` vs `lerobot_P1_tool`、P2a・P2b）
- `feetech_launch.log` の `[diag]` 行（MATCH / MISMATCH）
- 手順 2.5 の出力
- 想定と違う挙動（例: `read failed` が頻発する、FAIL の関節がある）
