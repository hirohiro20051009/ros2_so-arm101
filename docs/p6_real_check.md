# 実機確認手順: VR 追従ノード（vr_follow）P6c

計画: `docs/p6_plan.md`。モックの結果: `docs/p6_mock_check.md`。

> **この手順書のコマンドは、Claude Code が一つも実行していない。** 実機（腕）・シリアルポート・launch は、Claude は動かさない。実行するのはユーザーだけ。チャットが 1 段ずつ出す。
>
> **コードの状態**: 実機用の偽入力（`--real`）、launch の引数（`max_tracking_error`・`gripper_open_rad`・`gripper_closed_rad`・`floor_z`）、c0 の床の守り（3 章）が入ったコミットの後で使う。それらはモックで確かめた（`~/so_arm101_check/p6/b4_*`。3 章）。**実機では、まだ何も確かめていない。**

## 1. 先に確認した事実（コード・設定を読んだ）

1. **折りたたみ姿勢から T への ik_move 1 回は、拒否される。** `goal_checks`（`ik_move.py`）の `MOVE_JOINT_TOO_FAR`（各関節 1.0 rad まで）に当たる: lift −88° → −22° は 1.15 rad、elbow +90° → +30° は 1.05 rad。さらに、出発点の先端 z が 0.045 m 未満なら、`start_checks` が `START_FLOOR_MEASURED` で拒否する（F6 の出発点の読みからの手計算では約 0.049 m で、余裕は 4 mm。たたみ方で変わる）。**対処**: 2 段に分ける（中間の姿勢 M を経由）。M は (0, −55, 60, 40, 0) 度（手計算で先端 z 約 0.08 m。各関節の動きは 33°・30°・7° と 33°・30°・43° で 1.0 rad 以内）。`--dry-run` の結果が正で、**拒否されたら、F6 の段 c と同じ RViz（実績あり）で T へ行く**。
2. **読み取り専用のドライバは、毎周期 `hw_commands_ = pos` に上書きする**（`sts3215_system.cpp:531`）。最初に指令が測定と 0.02 rad 以上離れたとき、`READ-ONLY: command for '…' differs from the measured position by … rad and is discarded (no motion)` の WARN が **1 回**出る（`:546-550`）。c0 では**正常**（共通の止める条件に入れない）。
3. **クラッチの前の検査**（IDLE → 追従）は、測定の先端 z が床（既定 0.045 m）以上、`|reference − 測定| ≤ 0.06 rad`、elbow ≤ +91°、動いている goal がないこと。**折りたたみ姿勢の先端 z が 0.045 m 未満なら、クラッチは `REFUSED FLOOR` で始まらない**（状態に出る）。F6 の出発点の読みからの手計算では約 0.049 m（余裕 4 mm）で、たたみ方で変わる。
4. **c0 は、腕を支えず、折りたたみ姿勢のまま行う**（トルク OFF の腕を長く支えると、手の揺れで測定が動き、0.06 rad の検査と 0.2 rad の判定がぶれるため）。まず既定の床で vr_follow を起動し、`REFUSED FLOOR` になったときだけ、**c0 に限り**床を `floor_z:=0.02` に下げて起動し直す（c0 は書き込まないので腕は動かない）。**ノードは、床が 0.045 m 未満のとき、`/robot_state_publisher` の robot_description が `read_only` true であることを確かめ、違えば起動しない**（モックで確認済み: exit 2）。**c1 以降は既定の床（`floor_z` を付けない）に戻す。**
5. **c0 で BLOCKED が出る量**: 折りたたみでは先端が pan 軸に近く、y の動きで pan が大きく動くので、T より BLOCKED が出やすい見込み（pan 軸から先端までの距離は、読みから計算しない。出なければ下の手順）。BLOCKED は、指令（偽入力が出す点）が測定から 0.2 rad 離れたときに出る。
6. **c0-1 の不具合（2026-11-17。直した）**: 実機の robot_description は `<param name="read_only">True</param>`（xacro が Python の bool を文字にするため、先頭が大文字）で、read_only の判定が小文字の `true` だけを認めていたため、偽入力が `refused: the robot_description does not say read_only` で止まった。判定を大文字小文字を区別しない形にした（`description_is_read_only`。偽入力とノードの床の守りは同じ関数。true・True・TRUE と前後の空白は read_only、false・False・それ以外・欠けは read_only でない。実機と同じ 1 行を試験に入れた）。ドライバ（`parse_bool`）とカメラの起動引数（`parse_bool_argument`）は、もともと大文字小文字を区別しない。

## 2. 共通の約束

| 約束 | 内容 |
|---|---|
| 端末 | A: 腕の launch（操作しない）。B: `vr_follow`（操作しない）。C: ik_move と偽入力。D: 確認のコマンド（`ros2 topic echo --no-daemon /vr_follow/status` など）。`ROS_DOMAIN_ID` は設定しない（87 はモック専用。偽入力の `--real` は 87 だと拒否する）。各コマンドのブロックの先頭に、`cd ~/ros2_so-arm101` と、`/opt/ros/humble/setup.bash` と `install/setup.bash` の source が入っている（そのまま貼れる。端末ごとに 1 回で足りるが、毎回入れてよい） |
| 保存先 | `~/so_arm101_check/p6_real_20261117/`（c0 の最初のコマンドが `mkdir -p` する）。**ファイルを書く前に、`ls` で同名が無いことを確かめる。同名があれば、そのコマンドは実行しない**（`No such file or directory` が出るのが正常）。再実行は別名（`_2`）で |
| 腕を支える | トルク ON の起動の前に、腕を折りたたみ姿勢にして手で支える。止める前にも支える（Ctrl-C でトルクが OFF になり、腕は落ちる）。**動いている間は、片手を 12 V の電源の近くに置く**。（c0 の読み取り専用は支えない。腕は折りたたみ姿勢で自重で置く） |
| 床・周り | 先端 z ≥ 0.045 m（c0 の床の引き下げを除く）。腕の周り（x 0.45 m、y ±0.25 m）に物を置かない |
| 偽入力の安全 | 動きは**腕の目標の空間で ±30 mm 以内**（超える指定は拒否。`--arm-read-only` で robot_description が read_only のときだけ 60 mm）、速さは 20 mm/s 以内、軸は y。scale は偽入力が `/vr_follow` から読み、手の動きは「目標の動き ÷ scale」になる。確認の Enter が要る |
| 実機のコマンド | Claude Code は実行しない（実機・シリアルポート・launch）。コマンドはチャットが 1 段ずつ出す |
| 後始末 | 停止のたびに ps と `ros2 node list --no-daemon` を確認（6 章） |
| 非常停止 | 12 V を切る（最後の手段） |

### 共通の止める条件（出たら、その場で止めて、腕を支えて Ctrl-C。原因を報告してから再開）

- A のログに `FAULT:`・`COMMUNICATION LOST`・`read failed`・**`step limit`**（ドライバの 1 周期の上限 0.06 rad に当たった。腕では出ないはず。グリッパーの `ramping toward it` は除く）・`clamp: '`。（c0 の `READ-ONLY: command … discarded` の WARN 1 回は除く。）
- 異音・腕の予期しない動き。
- 状態（`/vr_follow/status`）が、その段の予測にない `HOLD_RANGE`・`BLOCKED`・`BUSY`（c1 で出たら止めて報告。c0-1 の BLOCKED は予測どおり）。
- 偽入力が「目標の空間で ±30 mm」の外へ動かそうとした（拒否されるはず。出たらコードの不具合）。
- elbow が +91° を超えた（RViz で T に行くときも、RViz の Joints タブで +91° より上にしない）。

## 3. コードの内容（モックで確かめた。実機では未確認）

1. `examples/vr_fake_input.py` の `--real`: `--mock` と同時指定は不可、ROS_DOMAIN_ID が 87 なら拒否。腕を動かす準備をしない（`/arm_controller/joint_trajectory` の publisher と `FollowJointTrajectory` のクライアントを作らない。試験で確認）。目標の空間で ±30 mm・20 mm/s の上限。シナリオは line・stale・jump・gripper のみ。`/vr_follow` の `scale` を読んで手の動きを決める。各シナリオの前に Enter の確認。`jump` は、目標の空間で 60 mm の階段（> 50 mm）を 1 回だけ出し、腕が動かない（先端 5 mm 未満）ことを見る。`gripper` は、ボタンの押しだけ（クラッチなし）。各シナリオで `max abs(measured - reference)`（読み − 指令）と BLOCKED の有無を出す。
2. `--arm-read-only`（c0 だけ）: robot_description の `read_only` が true のときだけ、上限を 60 mm に上げる。`--expect blocked`（TRACKING → BLOCKED。腕が動かない）と `--expect stationary`（IDLE → TRACKING → IDLE。腕が動かない）は、これが要る。
3. `launch/vr_follow.launch.py` の引数: `max_tracking_error`・`gripper_open_rad`・`gripper_closed_rad`・`floor_z`（c0 の床の引き下げ）。
4. `vr_follow` の床の守り: `floor_z` が 0.045 m 未満（0.01 m 以上）のときは、robot_description が read_only のときだけ起動する。
5. モックでの確認（2026-11-17。`~/so_arm101_check/p6/b4_*`）: 7 シナリオの回帰が合格。モック専用の `--real-plans`（`--real` と同じプラン・上限・Enter）で line・stale・jump・gripper が合格。`floor_z:=0.02`（0.01 未満は設定の検査）の起動拒否（exit 2）。引数の拒否の試験（`--real` が 87、`--mock` と同時、range・rotation・busy、31 mm、`--expect` だけ）。

## 4. 段

### 段 c0. 読み取り専用（腕は動かさない。トルク OFF）

**目的**: 実機でも、クラッチ・状態・C3（BLOCKED）が動くこと。サーボには書かないので、**指令だけ進み、測定は動かない**。**腕を支えず、折りたたみ姿勢のまま行う。**

**合格の条件**: c0-1 で TRACKING → BLOCKED、c0-2（`max_tracking_error:=0`）で IDLE → TRACKING → IDLE。**どちらも腕が動かない**（偽入力が、先端の動きが 3 mm 未満であることを見る）。

**準備**: 腕を折りたたみ姿勢にして、自重で置く（支えない）。

端末 A:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p ~/so_arm101_check/p6_real_20261117
ls ~/so_arm101_check/p6_real_20261117/c0_arm.log
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=true 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c0_arm.log; echo "exit=${PIPESTATUS[0]}"
```

端末 D（10 秒ほど待ってから）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c0_start_pose.txt
ros2 topic echo --once --no-daemon --field position /joint_states 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c0_start_pose.txt
```

見るもの: `c0_start_pose.txt` の読みが、a の出発点の範囲（elbow は +88〜+92.2°）にあること。

#### c0-1. C3 有効。TRACKING → BLOCKED

**既定の床で起動する。** 端末 B（scale は c1 と同じ 0.5。偽入力は目標の空間で振幅を決めるので、腕の目標の動きは scale に依らない。c0 だけ変える理由はない）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c0_node.log
ros2 launch so_arm101_ik vr_follow.launch.py scale:=0.5 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c0_node.log; echo "exit=${PIPESTATUS[0]}"
```

端末 C（Enter で開始）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c0_line.txt ~/so_arm101_check/p6_real_20261117/c0_line_csv
python3 -u ~/ros2_so-arm101/src/so_arm101_ik/examples/vr_fake_input.py --real --arm-read-only --expect blocked \
  --scenarios line --amplitude-mm 30 --out ~/so_arm101_check/p6_real_20261117/c0_line_csv 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c0_line.txt; echo "exit=${PIPESTATUS[0]}"
```

| 見るもの | 予測（合格の条件） | 扱い |
|---|---|---|
| 状態の遷移 | IDLE → TRACKING → **BLOCKED**（理由 `TRACKING_ERROR 0.2…` rad）→ IDLE（RELEASED）。BLOCKED になるのは、指令が測定から 0.2 rad 離れたとき | 偽入力の PASS・FAIL の表示を記録 |
| **IDLE のまま `REFUSED FLOOR`**（折りたたみの先端 z が 0.045 m 未満） | 出うる（F6 の読みからの手計算は約 0.049 m で、余裕は 4 mm） | **下の「床を下げて起動し直す」** |
| BLOCKED の後 | 点は来ない（偽入力の「TRACKING 以外で点なし」が PASS） | |
| 腕 | 動かない（偽入力の「the arm did not move」が PASS） | 動いたら止めて報告 |
| A のログ | `READ-ONLY: command … discarded` の WARN が **1 回**（正常）。ほかの WARN・FAULT なし | 共通の止める条件 |

- **床を下げて起動し直す（`REFUSED FLOOR` のときだけ。c0 に限る）**: 端末 B を Ctrl-C で止め、**床を下げて**起動し直す（ノードが、robot_description の read_only を確かめて、違えば起動しない）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c0_node_floor002.log
ros2 launch so_arm101_ik vr_follow.launch.py scale:=0.5 floor_z:=0.02 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c0_node_floor002.log; echo "exit=${PIPESTATUS[0]}"
```

  ノードのログに、`floor_z 0.020 m (default 0.045 m): the arm is read_only, so it cannot move. Use the default floor from the writing mode on.` の WARN が出る。端末 C は、同じコマンドで、**保存先を別名**（`c0_line_floor002.txt`・`c0_line_floor002_csv`）に変えて再実行する。
- **BLOCKED が出なかったとき**（振幅 30 mm では 0.2 rad に届かない）: 偽入力の `max abs(sent point - measured)` の値を記録し、**`--amplitude-mm 60` で、別名（`c0_line_60.txt`・`c0_line_60_csv`）で 1 回だけやり直す**。それでも出なければ、報告して指示を待つ（C3 の判定の見直し）。
- 終わったら、B を Ctrl-C で止める（A は残す）。

#### c0-2. C3 を無効にした起動で、遷移を見る

端末 B（c0-1 で床を下げた場合は、`floor_z:=0.02` を付ける。付けた場合のログ名は `c0_node_noc3_floor002.log`）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c0_node_noc3.log
ros2 launch so_arm101_ik vr_follow.launch.py scale:=0.5 max_tracking_error:=0.0 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c0_node_noc3.log; echo "exit=${PIPESTATUS[0]}"
```

端末 C:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c0_line_noc3.txt ~/so_arm101_check/p6_real_20261117/c0_line_noc3_csv
python3 -u ~/ros2_so-arm101/src/so_arm101_ik/examples/vr_fake_input.py --real --arm-read-only --expect stationary \
  --scenarios line --amplitude-mm 30 --out ~/so_arm101_check/p6_real_20261117/c0_line_noc3_csv 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c0_line_noc3.txt; echo "exit=${PIPESTATUS[0]}"
```

| 見るもの | 予測 | 止める条件 |
|---|---|---|
| 状態の遷移 | IDLE → TRACKING → IDLE（RELEASED）だけ。BLOCKED なし。点が TRACKING の間だけ来る。偽入力の `max abs(sent point - measured)` が 0.2 rad を超える（C3 がなければ指令は進み続ける） | BLOCKED・HOLD が出る |
| 腕 | 動かない（偽入力の「the arm did not move」が PASS） | 動いたら止めて報告 |

c0 が終わったら、B を Ctrl-C で止め、**A を Ctrl-C で止め**（read_only なので、腕は折りたたみ姿勢のまま）、6 章の確認をする。**c1 以降の vr_follow は、`floor_z` を付けない（既定の床 0.045 m）。**

### 段 c1. 書き込みモード（腕を支える。トルク ON）

**ここからトルク ON。** 腕を折りたたみ姿勢にして、前腕の下を支える。

**値の出典（確認済み）**: `torque_limit:=500` は F6 の手順書（`docs/zero_offset_apply_real_check.md` の段 c、146 行）と `docs/large_move_real_check.md`（114 行）と同じ。T の関節値 (0, −22, 30, 83, 0) 度は、F6 の段 c（同 169 行。RViz の Joints タブ）と `large_move_real_check.md`（160 行。`--joints-deg 0 -22 30 83 0`）と同じ。

端末 A:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c1_arm.log
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=false torque_limit:=500 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c1_arm.log; echo "exit=${PIPESTATUS[0]}"
```

端末 D（10 秒ほど待ってから）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control list_controllers
ls ~/so_arm101_check/p6_real_20261117/c1_start_pose.txt
ros2 topic echo --once --no-daemon --field position /joint_states 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c1_start_pose.txt
```

`arm_controller`・`gripper_controller`・`joint_state_broadcaster` が active。有効化で腕が動かない（F6 の段 c と同じ。10 秒あけて読み直した差が 0.005 rad 以内）。

#### c1-1. T へ（ik_move の 2 段。腕を支え、12 V の近くに手）

各実行の前に `--dry-run` を見る。**拒否されたら、実行せず、報告（または RViz の F6 段 c と同じ手順で T へ）。**

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c1_M_dry.log
python3 -u ~/ros2_so-arm101/tools/ik_move_run.py --dry-run --joints-deg 0 -55 60 40 0 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c1_M_dry.log; echo "exit=${PIPESTATUS[0]}"
```

見込み: `DRY-RUN: the goal passes every check`、`exit=0`。**`START_FLOOR_MEASURED`（出発点の z < 0.045 m）や `MOVE_JOINT_TOO_FAR` が出たら、この ik_move は使えない。RViz（F6 の段 c と同じ）で T へ行く。**

通ったら:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c1_M.csv ~/so_arm101_check/p6_real_20261117/c1_M.log
python3 -u ~/ros2_so-arm101/tools/ik_move_run.py --joints-deg 0 -55 60 40 0 \
  --csv ~/so_arm101_check/p6_real_20261117/c1_M.csv 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c1_M.log; echo "exit=${PIPESTATUS[0]}"
```

M に着いたら、T へ（dry-run → 実行。名前は `c1_T_dry.log`・`c1_T.csv`・`c1_T.log`。コマンドは `--joints-deg 0 -22 30 83 0`）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c1_T_dry.log
python3 -u ~/ros2_so-arm101/tools/ik_move_run.py --dry-run --joints-deg 0 -22 30 83 0 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c1_T_dry.log; echo "exit=${PIPESTATUS[0]}"
ls ~/so_arm101_check/p6_real_20261117/c1_T.csv ~/so_arm101_check/p6_real_20261117/c1_T.log
python3 -u ~/ros2_so-arm101/tools/ik_move_run.py --joints-deg 0 -22 30 83 0 \
  --csv ~/so_arm101_check/p6_real_20261117/c1_T.csv 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c1_T.log; echo "exit=${PIPESTATUS[0]}"
ls ~/so_arm101_check/p6_real_20261117/c1_T_pose.txt
ros2 topic echo --once --no-daemon --field position /joint_states 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c1_T_pose.txt
```

| 見るもの | 予測 | 止める条件 |
|---|---|---|
| ik_move の `Outcome` | M・T とも `REACHED`（終了コード 0）。先端 z ≥ 0.045 m | `SHORT`（記録して報告。次へは進まない）・`PLAN_REFUSED`・`NOT_SETTLED` など |
| T に着いたことの確認（**読み − 目標**。目標は 0、−22、30、83、0 度を rad にした値） | pan・lift・wrist_flex・wrist_roll は 0.0082 rad 以内が目安。elbow は大きい側（読み − 目標 > 0）+0.045 rad 以内、小さい側 0.0082 rad 以内が目安（F6 の段 c の基準） | **pan・lift・wrist_flex・wrist_roll は 0.04 rad 超、elbow は大きい側 +0.05 rad 超・小さい側 0.04 rad 超** |
| A のログ | `clamp: '` なし、WARN なし | 共通の止める条件 |

`c1_T_pose.txt` の値と目標との差を、報告に書く（`読み − 目標` の表）。

#### c1-2. vr_follow（scale 0.5）で、偽入力

端末 B:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c1_node.log
ros2 launch so_arm101_ik vr_follow.launch.py scale:=0.5 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c1_node.log; echo "exit=${PIPESTATUS[0]}"
```

端末 D（状態を見続ける）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo --no-daemon /vr_follow/status
```

端末 C で、シナリオを 1 つずつ（**各 Enter の前に、腕を支え、12 V の近くに手を置く**）。各実行で、偽入力が出す `max abs(measured - reference)`（読み − 指令）と BLOCKED の有無を記録する。各段のあとに、`/joint_states` を控える（コマンドは各段の最後。端末 C で続けて実行してよい）。

**c1-2a 小さな往復（目標が y 方向に ±10 mm）**

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c1_line10.txt ~/so_arm101_check/p6_real_20261117/c1_line10_csv
python3 -u ~/ros2_so-arm101/src/so_arm101_ik/examples/vr_fake_input.py --real --scenarios line --amplitude-mm 10 \
  --out ~/so_arm101_check/p6_real_20261117/c1_line10_csv 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c1_line10.txt; echo "exit=${PIPESTATUS[0]}"
ls ~/so_arm101_check/p6_real_20261117/c1_line10_pose.txt
ros2 topic echo --once --no-daemon --field position /joint_states 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c1_line10_pose.txt
```

（最後の 2 行は、同じ端末 C で続けて実行してよい。）予測: IDLE → TRACKING → IDLE。腕が y 方向に ±10 mm 動く。`max abs(measured - reference)` が小さい（C3 の 0.2 rad に近づかない。**値を記録**）。BLOCKED・HOLD なし。点は TRACKING の間だけ。止める条件: BLOCKED・HOLD_RANGE・HOLD_LAG・異音。

**c1-2b 大きめの往復（±30 mm）**

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c1_line30.txt ~/so_arm101_check/p6_real_20261117/c1_line30_csv
python3 -u ~/ros2_so-arm101/src/so_arm101_ik/examples/vr_fake_input.py --real --scenarios line --amplitude-mm 30 \
  --out ~/so_arm101_check/p6_real_20261117/c1_line30_csv 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c1_line30.txt; echo "exit=${PIPESTATUS[0]}"
ls ~/so_arm101_check/p6_real_20261117/c1_line30_pose.txt
ros2 topic echo --once --no-daemon --field position /joint_states 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c1_line30_pose.txt
```

予測と止める条件は c1-2a と同じ（先端 z ≥ 0.045 m）。

**c1-2c 入力が止まる（stale）**

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c1_stale.txt ~/so_arm101_check/p6_real_20261117/c1_stale_csv
python3 -u ~/ros2_so-arm101/src/so_arm101_ik/examples/vr_fake_input.py --real --scenarios stale --amplitude-mm 10 \
  --out ~/so_arm101_check/p6_real_20261117/c1_stale_csv 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c1_stale.txt; echo "exit=${PIPESTATUS[0]}"
ls ~/so_arm101_check/p6_real_20261117/c1_stale_pose.txt
ros2 topic echo --once --no-daemon --field position /joint_states 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c1_stale_pose.txt
```

予測: TRACKING → STALE POSE（入力を止めて 0.5 s 後）→ TRACKING（同じ位置に戻る）→ IDLE。STALE 中は腕が止まる（点なし）。止める条件: STALE から戻らない・STALE 中に腕が動く。

**c1-2d 入力が飛ぶ（jump）**

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c1_jump.txt ~/so_arm101_check/p6_real_20261117/c1_jump_csv
python3 -u ~/ros2_so-arm101/src/so_arm101_ik/examples/vr_fake_input.py --real --scenarios jump \
  --out ~/so_arm101_check/p6_real_20261117/c1_jump_csv 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c1_jump.txt; echo "exit=${PIPESTATUS[0]}"
ls ~/so_arm101_check/p6_real_20261117/c1_jump_pose.txt
ros2 topic echo --once --no-daemon --field position /joint_states 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c1_jump_pose.txt
```

予測: TRACKING → HOLD_JUMP（TARGET_JUMP。目標の空間で 60 mm の階段）→ TRACKING → IDLE。**腕は動かない**（先端が 5 mm 未満）。止める条件: 腕が階段の向きに 5 mm 以上動く。

**c1-2e クラッチを離す**: 各シナリオの終わりで自動（IDLE の RELEASED）。離すと点が来なくなる（偽入力の「TRACKING 以外で点なし」が PASS）。止める条件: 点が来続ける。

- **BLOCKED が出なかったこと**（実機の C3 の余裕: 読み − 指令の最大が 0.2 rad 未満。各段の値を記録）。
- 途中で止めたくなったら、クラッチを離す（偽入力を Ctrl-C で止めると、Joy が止まって 1 s 後に JOY_TIMEOUT で IDLE になる）。
- vr_follow は**既定の床**（`floor_z` を付けない）で起動している。


### 段 c2. グリッパー

**グリッパーは、腕を T に置いたまま、vr_follow を起動し直して行う。** 値の変更は launch の引数。

**c2-a: open_rad 0.3 rad（closed 0.05 の近く）で 1 往復**

端末 B（c1 の B を Ctrl-C で止めてから）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c2a_node.log
ros2 launch so_arm101_ik vr_follow.launch.py scale:=0.5 gripper_open_rad:=0.3 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c2a_node.log; echo "exit=${PIPESTATUS[0]}"
```

端末 C（Enter で、CLOSE、OPEN を 1 回ずつ。クラッチは使わない）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c2a_gripper.txt ~/so_arm101_check/p6_real_20261117/c2a_gripper_csv
python3 -u ~/ros2_so-arm101/src/so_arm101_ik/examples/vr_fake_input.py --real --scenarios gripper --gripper-cycles 1 \
  --out ~/so_arm101_check/p6_real_20261117/c2a_gripper_csv 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c2a_gripper.txt; echo "exit=${PIPESTATUS[0]}"
```

| 見るもの | 予測 | 止める条件 |
|---|---|---|
| `/vr_follow/gripper` | CLOSE で 1.0、OPEN で 0.0（各 1 回） | 違う |
| グリッパーの動き | CLOSE で closed 側（`/joint_states` の gripper が約 0.05 rad 付近。閉じた読み 0.028 の近く）、OPEN で 0.3 rad 付近。ゆっくり動く（ドライバが 1 周期の上限で追従）。異音なし | 異音・動かない・0.3 rad を大きく超える |
| 腕 | 動かない | 動く |
| A のログ | `step limit` の WARN が出てもよい（グリッパーの目標は階段なので、ドライバが上限で追従する。`ramping toward it`）。それ以外の WARN・FAULT なし | 共通の止める条件（`step limit` のグリッパー以外） |

`/joint_states` の gripper の値を、CLOSE の後・OPEN の後に `~/so_arm101_check/p6_real_20261117/c2a_gripper_closed.txt`・`c2a_gripper_open.txt` に控える。

**c2-b: open_rad 1.0 rad で 1 往復**（c2-a に問題がなければ。B を Ctrl-C で止めてから、起動し直す）

端末 B:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c2b_node.log
ros2 launch so_arm101_ik vr_follow.launch.py scale:=0.5 gripper_open_rad:=1.0 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c2b_node.log; echo "exit=${PIPESTATUS[0]}"
```

端末 C:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c2b_gripper.txt ~/so_arm101_check/p6_real_20261117/c2b_gripper_csv
python3 -u ~/ros2_so-arm101/src/so_arm101_ik/examples/vr_fake_input.py --real --scenarios gripper --gripper-cycles 1 \
  --out ~/so_arm101_check/p6_real_20261117/c2b_gripper_csv 2>&1 \
  | tee ~/so_arm101_check/p6_real_20261117/c2b_gripper.txt; echo "exit=${PIPESTATUS[0]}"
```

見るものは c2-a と同じで、OPEN の後の gripper が約 1.0 rad。**ここで、closed_rad と open_rad の値を決める**（記録: 閉じた読み、開いた読み、動きの様子。決めた値は、`config/vr_follow.yaml` に書く。値の変更は別に報告して承認を得る）。

## 5. 終わり

1. **vr_follow（端末 B）を Ctrl-C で止める。**
2. 腕を T から折りたたみ姿勢へ戻す。ik_move の 2 段（M を経由。各段の前に dry-run）。**折りたたみの値（`<P> <L> <E> <W> <R>`、度）は、`c1_start_pose.txt` の読みを度に直して四捨五入した整数（elbow は +91° 以下）。チャットが、読みから計算して値を埋めて出す。** dry-run が `GOAL_FLOOR`（折りたたみ姿勢の先端 z が 0.048 m 未満）などで拒否されたら、**RViz（F6 の段 e と同じ）で戻す**。腕を支え、12 V の近くに手を置く。

端末 C（T から M へ）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c5_M_dry.log ~/so_arm101_check/p6_real_20261117/c5_M.csv ~/so_arm101_check/p6_real_20261117/c5_M.log
python3 -u ~/ros2_so-arm101/tools/ik_move_run.py --dry-run --joints-deg 0 -55 60 40 0 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c5_M_dry.log; echo "exit=${PIPESTATUS[0]}"
python3 -u ~/ros2_so-arm101/tools/ik_move_run.py --joints-deg 0 -55 60 40 0 \
  --csv ~/so_arm101_check/p6_real_20261117/c5_M.csv 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c5_M.log; echo "exit=${PIPESTATUS[0]}"
```

端末 C（M から折りたたみへ。`<P> <L> <E> <W> <R>` は上のとおりチャットが埋める）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ls ~/so_arm101_check/p6_real_20261117/c5_fold_dry.log ~/so_arm101_check/p6_real_20261117/c5_fold.csv ~/so_arm101_check/p6_real_20261117/c5_fold.log
python3 -u ~/ros2_so-arm101/tools/ik_move_run.py --dry-run --joints-deg <P> <L> <E> <W> <R> 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c5_fold_dry.log; echo "exit=${PIPESTATUS[0]}"
python3 -u ~/ros2_so-arm101/tools/ik_move_run.py --joints-deg <P> <L> <E> <W> <R> \
  --csv ~/so_arm101_check/p6_real_20261117/c5_fold.csv 2>&1 \
  | tee -i ~/so_arm101_check/p6_real_20261117/c5_fold.log; echo "exit=${PIPESTATUS[0]}"
```

3. 腕を折りたたみ姿勢で支える。**端末 A で Ctrl-C を 1 回。**
4. 6 章の確認。

## 6. 後始末（各段の終わりと、この手順の終わり）

1. A に出る最後の行 `exit=…` を貼る（`exit=0` が期待）。**貼ってから、ps を実行する。**
2. A のログに `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints` と `deactivated. reads=…`（c0 の read_only では、トルクの行はなし）。bus のカウンタがすべて 0。`grep -c "clamp: '" ~/so_arm101_check/p6_real_20261117/c1_arm.log` が 0。
3. 数秒待ってから、端末 D で（`--no-daemon`）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|ik_move_run|vr_follow|vr_fake_input" | grep -v -E "grep|gvfsd"
ros2 node list --no-daemon
```

どちらも空（`kill -9`・`pkill`・`killall` はしない。`--no-daemon` にだけ残り ps が空なら、数秒待ってもう一度。それでも残れば報告して指示を待つ。ros2 daemon には触らない）。

## 7. 合格の基準（まとめ）

- c0-1: TRACKING → BLOCKED（指令が測定から約 0.2 rad 離れたところ）。BLOCKED 後に点なし。**腕が動かない**。`READ-ONLY … discarded` の WARN は 1 回だけ。`REFUSED FLOOR` のときは、`floor_z:=0.02` で起動し直した（c0 に限る）。
- c0-2: C3 無効（`max_tracking_error:=0`）で IDLE → TRACKING → IDLE。**腕が動かない**。c1 以降は既定の床に戻す。
- c1-1: M・T が `REACHED`。T の読み − 目標が基準内。
- c1-2: 各段で予測どおりの状態の遷移。`読み − 指令` の最大と BLOCKED なしを記録。jump で腕が動かない。TRACKING 以外で点なし。
- c2: CLOSE・OPEN がそれぞれ 1 回で動く。腕は動かない。closed_rad・open_rad を決める。
- 終わり: 折りたたみ姿勢、`exit=0`、ps と `ros2 node list --no-daemon` が空。

## 8. 記録に残すもの（`~/so_arm101_check/p6_real_20261117/`）

A・B のログ、各偽入力の出力とタイムライン（CSV）、`*_pose.txt`、ik_move の CSV とログ、`c2*_gripper_*.txt`。材料の sha256 は `docs/inventory.md` の P6c の節に。

## 9. 実機の結果（2026-11-17 23:12〜23:53。ユーザーが実施。全段合格）

材料は `~/so_arm101_check/p6_real_20261117/`（git 管理外。sha256 は `docs/inventory.md` の「P6c の実機の結果」の節）。Claude Code は実機・launch・シリアルポートを動かしていない。ここに書くのは、ログとファイルを読んで確かめた事実。

### c0（読み取り専用。腕は動かない）

| 項目 | 結果 |
|---|---|
| c0-1 の 1 回目（`c0_arm.log`・`c0_node.log`・`c0_line.txt`） | 偽入力が `refused: the robot_description does not say read_only` で止まった。実機の値は `True`（大文字始まり）。d57734c で直した（1 章の 6）。腕は動いていない |
| やり直し（`c0_arm_2.log`・`c0_node_2.log`） | A と B を起動し直した |
| c0-1、30 mm（`c0_line_2.txt`） | IDLE → TRACKING → IDLE。BLOCKED は出なかった。読み − 指令の最大 0.1668 rad（C3 の 0.2 rad に届かない）。腕は動かず（先端 0.0 mm）。「TRACKING の後に BLOCKED」の項目は FAIL（予測どおりの手順の分岐） |
| c0-1、60 mm（`c0_line_60.txt`、手順書どおりの 1 回） | TRACKING → BLOCKED（`TRACKING_ERROR_0.203_rad`）→ IDLE（RELEASED）。BLOCKED の後に点なし。腕は動かず（先端 0.0 mm）。読み − 指令の最大 0.2032 rad。全項目 PASS |
| `REFUSED FLOOR` | 既定の床（0.045 m）で出なかった。`floor_z:=0.02` は使っていない |
| c0-2 の 1 回目（`c0_node_noc3.log`・`c0_line_noc3.txt`） | `IDLE REFUSED_WINDUP windup=0.2032`（偽入力は FAIL。腕は動かず） |
| 原因 | c0-1 の 60 mm の BLOCKED の後、JTC が、測定から 0.203 rad 離れた最後の点を指令として持ち続けた（読み取り専用なので腕は追いつかない）。次のクラッチの前の検査（`|reference − 測定| ≤ 0.06 rad`）が、正しく拒否した。コードの不具合ではない |
| やり直し | A を起動し直し（`c0_arm_3.log`。指令が測定に戻る）、B は `max_tracking_error:=0.0`（`c0_node_noc3_2.log`）。`c0_line_noc3_2.txt`: IDLE → TRACKING → IDLE。BLOCKED なし。読み − 指令の最大 0.1680 rad（C3 がなければ指令は進み続ける）。腕は動かず。全項目 PASS |
| A のログ | `READ-ONLY: command for 'shoulder_pan' differs … and is discarded` の WARN が起動ごとに 1 回（`c0_arm_2.log`・`c0_arm_3.log`。`c0_arm.log` では偽入力が起動しなかったので出ない）。`clamp: '` は 0 件。FAULT なし。`deactivated (no register was written)`、bus のカウンタはすべて 0 |
| 出発点（`c0_start_pose.txt`） | 約 −5.2°、−86.9°、89.7°、34.3°、−2.7°（折りたたみ。elbow は +91° 以下） |

### c1（書き込みモード。腕を支えた）

| 項目 | 結果 |
|---|---|
| ik_move M（0, −55, 60, 40, 0 度） | dry-run 合格 → `REACHED` error 9.55 mm（基準 10 mm 以内）。`c1_M.log` |
| ik_move T（0, −22, 30, 83, 0 度） | `REACHED` error 5.94 mm。経路の最低の z 0.0566 m。`c1_T.log` |
| T の読み − 目標（`c1_T_pose.txt`、rad） | pan −0.0031、lift −0.0004、elbow +0.0331、wrist_flex −0.0094、wrist_roll −0.0084。基準内（elbow は大きい側の +0.045 rad 以内。ほかは 0.0082 rad の目安を wrist_flex・wrist_roll がわずかに超えるが、止める条件の 0.04 rad より小さい） |
| line 10 mm | IDLE → TRACKING → IDLE。先端 10.6 mm。読み − 指令の最大 0.0354 rad。BLOCKED・HOLD なし |
| line 30 mm | 先端 28.9 mm。最大 0.0413 rad（送った点との差は 0.0423 rad）。BLOCKED・HOLD なし |
| stale | TRACKING → STALE（POSE）→ TRACKING → IDLE。STALE 中は点なし。最大 0.0270 rad |
| jump | TRACKING → HOLD_JUMP（TARGET_JUMP、jump_mm 60）→ TRACKING → IDLE。**先端 0.0 mm（腕は階段の向きに動かず）**。最大 0.0270 rad |
| BLOCKED・HOLD_RANGE・HOLD_LAG・BUSY | c1 のすべてで出なかった。C3 の余裕: 読み − 指令の最大は 0.0423 rad（0.2 rad の約 1/5） |
| `rate_limited=38`（`c1_arm.log` の終わり） | **グリッパーの動きだけによる**と判断できる。根拠: ドライバは、関節ごとに 5 秒に 1 回、`step limit` の WARN を出す（`sts3215_system.cpp:1010-1016`）。腕の関節で上限に当たれば、最初の 1 回は必ず WARN が出て、理由は `unexpected jump for an angle joint` になる。ログに、この文は **0 件**。`step limit` の WARN は gripper の 3 行だけ（`gripper goal is a step: ramping toward it`、limited 1・5・24 times so far）。グリッパーの目標は階段で、ドライバが 1 周期の上限 51 ticks で追う（予測どおり） |
| A のログ | `clamp: '` 0 件。FAULT・COMMUNICATION LOST・read failed なし |
| jump の C1（`c1_node.log`） | 飛びの瞬間 `ik_jump=0.3911`（HOLD_JUMP）。**再開の瞬間 `ik_jump=0.3453`（C1 の 0.35 の手前）**。再開の判定の比べ相手が、受け付けなかった（飛んだ）メッセージの解になっている疑い。次の変更（2 つ目の依頼）の根拠 |

### c2（グリッパー。腕は T に置いたまま）

| 設定 | CLOSE の後 | OPEN の後 |
|---|---|---|
| open_rad 0.3（`c2a_*`） | 0.0469 rad | 0.2952 rad |
| open_rad 1.0（`c2b_*`） | 0.0562 rad | 0.9886 rad |

- `/vr_follow/gripper` は CLOSE で 1.0、OPEN で 0.0 を各 1 回。ゴールは 2 回。クラッチは使わず、状態は IDLE のまま。腕は動かず（c2a 0.0 mm、c2b 0.7 mm）。
- **既定の closed_rad 0.05・open_rad 1.0 のまま確定する。`config/vr_follow.yaml` は変えない。**

### 終わり

- グリッパーを action で 0.05 rad に閉じた（ユーザーの操作）。
- ik_move で T → M: `REACHED` error 5.54 mm（`c5_M.log`）。
- M → 折りたたみ (−5, −87, 90, 34, −3) 度の ik_move は、dry-run が `START_REFUSED: GOAL_FLOOR`（目標の先端 z 0.0465 m < 床 0.048 m）で拒否した（`c5_fold_dry.log`）。手順書の予告どおり、**RViz（−5, −88, 90, 34, −3 度）で戻した**。
- `c5_fold_pose.txt` は、`/joint_states` がまだ検出できなかった一時的な失敗で、警告だけが入っている。正しい読みは `c5_fold_pose_2.txt`（約 −4.4°、−86.9°、90.1°、34.7°、−2.8°、gripper 0.0574 rad）。
- 停止: A の `exit=0`。`[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints`、`deactivated. reads=53127 … goal_writes=53127 … rate_limited=38 … bus` のカウンタはすべて 0。`move_group`・`rviz2` の終了時の exit code -11 は既知（`docs/feetech_write_check.md`）。

### まとめ

- 計画で決めた基準のすべてを満たした。実機で、クラッチ・状態・C3（BLOCKED）・stale・jump・グリッパーが予測どおりに動いた。
- 見つかったこと: (1) read_only の判定が大文字小文字を区別していた（d57734c で修正）。(2) BLOCKED の後、読み取り専用では指令が測定に戻らないため、次のクラッチが `REFUSED_WINDUP` になる（正しい動作。書き込みモードの見込み。**実機では未確認**: 書き込みモードで BLOCKED が出るのは、腕が何かに当たって指令に追いつけないとき。腕が追いつけないままなら、次のクラッチは `REFUSED_WINDUP` になる見込み。当たっていた物が外れると、腕は JTC が持つ最後の点（測定から最大約 0.2 rad 先）へ、クラッチなしで動く見込み。**BLOCKED の後は、障害物を外す前に腕を支えること。** 戻すには、bringup の起動し直し、または RViz・ik_move で今の位置への goal を送る。`docs/vr_follow_quickstart.md` の (6) にも入れる）。(3) 再開時の `ik_jump` が 0.345 と、C1 の閾値の手前だった（修正は次のコミット）。(4) 折りたたみ姿勢へは ik_move の `GOAL_FLOOR`（0.048 m）に当たるので、RViz で戻す。
