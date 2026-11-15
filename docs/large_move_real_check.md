# 実機確認手順: 大きな移動の道具（`tools/ik_move_run.py`）

計画と結果: `docs/large_move_plan.md`（決定は 1 章、実機の段は 11 章と 16.8、モックの結果は 16.6・16.8、実機で見る点は 16.9）。コミットは `72ffa2e`（道具）と `a8c6ca0`（Ctrl-C の行、`ALREADY_AT_GOAL`）。

> **実機を動かす段（M1 以降）はユーザーだけが実行する。Claude Code は実機に対して実行しない。** Claude が実行したのはモック（`ROS_DOMAIN_ID=87`）だけ。**この手順書の M0〜M3' は、2026-11-15 にユーザーが実機で実施し、すべて合格した（結果は `docs/large_move_plan.md` 17 章）。**

この道具: 解析 IK で関節の目標を出し、MoveIt に **計画だけ**させ（`plan_only`）、検査した軌道を `ExecuteTrajectory` で実行する。到達の判定は、止まってから測った関節角（`SUCCEEDED` は到達の根拠にしない。`open_loop_control: true`）。大きな移動の道具で、細かい補正（±3 mm）は `DiffMode`（`docs/diff_mode_real_check.md`）とカメラの段の仕事。

## 共通の約束（この手順書で参照する元）

次は書き写さず、参照する。**この手順書でも同じに守る。**

| 約束 | 参照 |
|---|---|
| 実行するのはユーザーだけ。端末は launch 用（ログを流す。操作しない）と操作用の 2 つ以上。止めるのは Ctrl-C だけ（`kill -9`・`pkill`・`killall` はしない） | `diff_mode_real_check.md:10-24` の約束 1・2、`servo_real_check.md:18-19` |
| 新しい端末（**conda を activate していない**端末）。準備の 3 行（`source /opt/ros/humble/setup.bash`、`source ~/ros2_so-arm101/install/setup.bash`、`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`）。**`ROS_DOMAIN_ID` は設定しない（87 はモック専用）** | `servo_real_check.md:18`、`:88-95`（「0. 準備（共通）」） |
| 書き込みモードの前に、腕を折りたたみ姿勢にして手で支える。止める前にも支える（Ctrl-C でトルクが OFF になり、腕は落ちる） | `diff_mode_real_check.md` の約束 2 |
| 床: 先端 z ≥ 0.045 m。腕の周りに物を置かない | 約束 3 |
| 非常停止は 12 V を切る。常に片手を電源の近くに置く | 約束 4、`servo_real_check.md:22` |
| `ros2` のコマンドは `--no-daemon` が使えるものには付ける（`ros2 control list_controllers` は使えない。付けない） | 約束 6、`diff_mode_real_check.md:72` |
| ログは `tee -i` と `echo "exit=${PIPESTATUS[0]}"`。気になるログ（`FAULT:`・`COMMUNICATION LOST`・`read failed`・`step limit` の WARN）が出たら中止して、支えながら Ctrl-C。原因を報告してから再開する | `servo_real_check.md:23`（約束 6）、`:113`、`:123`、`:371-377`（ログの表） |
| **停止のたびに**、ログに `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints` と `deactivated. reads=…` の 2 行があること。無ければ、腕を支えて 12 V を切る（S6） | `servo_real_check.md:73-80` |
| `move_group`・`rviz2` が exit code −11 は、終了時の既知の挙動（`deactivated.` の後なら無視してよい） | `servo_real_check.md:376` |
| 止まっていることの確認（コマンド A） | `servo_real_check.md:47` |
| 後始末の確認: `ps aux \| grep -E "ros2_control_node\|move_group\|robot_state_pub\|rviz2\|spawner\|static_transform_publisher" \| grep -v -E "grep\|gvfsd"` が空、`ros2 node list --no-daemon` が空 | `servo_real_check.md:25`（約束 8）、`diff_mode_real_check.md:146` |

この道具に固有のこと:

- **Servo は削除済み**（`8dc64f2`）。`servo_real_check.md` の止め方 S1〜S4 は Servo 用で、**使えない。腕を止める手段は S5 と S6 だけ**（下の「止め方」）。
- **CSV は上書きされる**（`--csv` は `'w'` で開く）。**同名のファイルが無いことを、毎回 `ls` で確かめる**（下の各段）。
- `--dry-run` は何も送らず、計画もせず、CSV も作らない。読み取り専用の起動でも動く（desired が無くてよい）。
- 確認は 2 回（開始、送信）。各 30 秒で、無入力だと送らずに終わる（`NOT_CONFIRMED`）。
- 実行（`ExecuteTrajectory`）は **Scaling 0.1 固定**。引数で変えられない（`--yes`・scaling の引数は無い）。

## ユーザーがすること（最初に 1 回）

1. 新しい端末で、準備の 3 行を実行する（conda なし）。
2. **ros2 daemon を起動し直す**（cyclonedds を設定したこの端末で。別の RMW の daemon が残っている場合に備えて、止めてから起動する。2026-11-15 の段 B の終わりの時点で、daemon のプロセスが見えなかった。実機の前にユーザーが行う）:

   ```bash
   ros2 daemon stop; ros2 daemon start; sleep 3
   ros2 daemon status
   ```
3. ps が空であること（前回の起動の残りが無い）:

   ```bash
   ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher|ik_move_run" | grep -v -E "grep|gvfsd"
   ros2 node list --no-daemon
   ```
4. 保存先を作る（`<日付>` は実行する日。例: `large_20261116_M`）:

   ```bash
   mkdir -p ~/so_arm101_check/large_<日付>_M
   cd ~/ros2_so-arm101
   ```

ログ・CSV・bag は `~/so_arm101_check/large_<日付>_M/` に置く。**各ファイルは、書く前に `ls` で同名が無いことを確かめる**（`No such file or directory` が出るのが期待どおり）。bag は `ros2 bag record -o ~/so_arm101_check/large_<日付>_M/bag_M /joint_states /arm_controller/controller_state`（M1 の launch の後から、最後まで 1 本）。

## 止め方（先に読む）

| 状況 | 何が止まるか | 操作 |
|---|---|---|
| **確認の画面**（`Start from this state?`、`Send this trajectory?`） | **何も送らずに終わる**（腕は動いていない） | `n` と Enter、または Ctrl-C（終了コード 2 または 130）。30 秒の無入力でも `NOT_CONFIRMED` |
| 計画の最中（確認の後、実行の前） | 何も動いていない。計画は捨てられる | Ctrl-C（130） |
| **送った後**（`ExecuteTrajectory` の実行中） | **道具の Ctrl-C は腕を止めない。** 道具は終わるが、進行中のゴールは軌道の最後まで進む（モックで確認。16.6 の d: kill の後も reference が約 3 秒動き続け、`SUCCEEDED` で終わった） | 道具の Ctrl-C は、何も送らず、何も取り消さず、終了コード 130 で終わるだけ |
| **腕を止めたいとき** | S5（launch の端末で Ctrl-C を 1 回。**トルク OFF で腕が落ちる**。腕を支える）、S6（12 V を切る。最後の手段。条件は `servo_real_check.md:39-43`） | これだけ |

**M3' は 5 秒ほど動く**（モックの計画で 4.96〜5.11 秒）。M2 は 3 秒ほど。**動いている間は、片手を 12 V の電源の近くに置き、もう片手で腕の近くを支える（落ちるときに受ける）。**

## 終了コードと表示

| 終了コード | 意味 | 扱い |
|---|---|---|
| 0 | `REACHED`（先端の誤差 10 mm 以内）、`DRY_RUN` | 合格の基準を見る |
| 2 | 停止（確認の拒否、`PLAN_REFUSED`、`ARM_MOVED`、`OTHER_GOAL`、`ALREADY_AT_GOAL` など） | **再試行しない。理由を報告する**（`ALREADY_AT_GOAL` は、すでに目標にいて何も動いていない。報告だけ） |
| 3 | 開始の拒否（`START_REFUSED`） | M0 では期待どおり。ほかでは中止して報告 |
| 4 | `SHORT`（先端の誤差が 10 mm 超） | 失敗ではない（手前止まり）。記録して報告。続けるかは下の各段 |
| 130 | Ctrl-C | CSV に 1 行（`stage`・`goal_sent`・`goal_in_flight`・`goal_cancelled` = False） |

見る値（各段の道具の出力と CSV。`~/so_arm101_check/large_<日付>_M/` の CSV の列）:

- **到達の表**（`ARRIVAL`）: 関節ごとの `q_meas − q_goal`（CSV `joint_error`）、先端の `FK(q_meas) − 目標`（CSV `tip_error_vec_mm`、大きさ `tip_error_mm`）、`FK(desired) − 目標`（`ref_error_mm`）。
- **手前止まり**: `desired − q_meas`（CSV `stuck`。関節ごと）。T の elbow は 0.034 rad だった（V7）。
- **計画の最後の点 − 目標**: 計画の行の `last vs goal`（CSV `last_vs_cmd`）。許容 0.001 以内。
- 計画の行: `PLAN OK`、長さ、最大の速さ、`lowest z`（CSV `path_min_z`。0.045 以上、モックでは 0.054 以上）。

## M0. 読み取り専用・出発点のまま（腕を支えない）

出発点（折りたたみ姿勢、先端 z ≈ 0.033 m）に腕を置いたまま、読み取り専用で起動する（トルク OFF。書き込まない）。

```bash
ls ~/so_arm101_check/large_<日付>_M/M0_launch.log ~/so_arm101_check/large_<日付>_M/M0_dry.log
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=true use_rviz:=false 2>&1 \
  | tee -i ~/so_arm101_check/large_<日付>_M/M0_launch.log; echo "exit=${PIPESTATUS[0]}"
```

操作用の端末で:

```bash
python3 -u tools/ik_move_run.py --dry-run --goal 0.200 0.000 0.060 --pitch-deg -91.0 --roll-deg 0 2>&1 \
  | tee -i ~/so_arm101_check/large_<日付>_M/M0_dry.log; echo "exit=${PIPESTATUS[0]}"
```

- **見込み（モックの結果。16.6 の e）**: `STOP START_REFUSED: START_FLOOR_MEASURED: the measured tip is at z = 0.03… m (minimum 0.045 m)`、`exit=3`。**他の理由も一緒に出ることがある**（`MOVE_JOINT_TOO_FAR`: 出発点から T の付近への lift・elbow の移動は約 1.1 rad で 1.0 rad を超える）。
- **合格: 拒否されること**（理由に `START_FLOOR_MEASURED` が含まれる）。何も送られていない（CSV も作られない）。`START_FLOOR_MEASURED` が出なければ、中止して報告（出発点が床の上にある、または検査が効いていない）。
- 止める: launch の端末で Ctrl-C を 1 回（読み取り専用なのでトルクは入っていない）。`deactivated (no register was written)` の行（`servo_real_check.md:122`）と `exit=0`。

## M1. 書き込みの起動と T への移動（RViz）。T で dry-run

**ここからトルク ON・書き込みモード。** 腕を折りたたみ姿勢にして、前腕の下を支える（共通の約束）。

```bash
ls ~/so_arm101_check/large_<日付>_M/M1_launch.log
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=false torque_limit:=500 2>&1 \
  | tee -i ~/so_arm101_check/large_<日付>_M/M1_launch.log; echo "exit=${PIPESTATUS[0]}"
```

操作用の端末で（`diff_mode_real_check.md` の V2a と同じ）: `ros2 control list_controllers`（`--no-daemon` は付けない）で `arm_controller` が active、コマンド A（`servo_real_check.md:47`）で `STOPPED`、ドライバの WARN なし。続けて bag を始める（別の端末。`ls` で `bag_M` が無いことを確かめてから）:

```bash
ls ~/so_arm101_check/large_<日付>_M/bag_M
ros2 bag record -o ~/so_arm101_check/large_<日付>_M/bag_M /joint_states /arm_controller/controller_state
```

**T へ移動する（RViz。W1 の手順。`servo_real_check.md:253` 以降）**: MotionPlanning で **Planner は OMPL**、**Velocity Scaling と Acceleration Scaling は 0.1 のまま**。Planning タブで **Start State と Goal State を `<current>` に選び直してから**、Joints タブで整数の度に設定する: **shoulder_pan 0、shoulder_lift −22、elbow_flex +30、wrist_flex +83、wrist_roll 0**（gripper は触らない）。Plan してアニメーションを見てから（先端が下がらない・動かさない関節が動かない）、Execute。動きが止まってから、実測を控える（`ros2 topic echo --once --no-daemon --field position /joint_states`）。**この実測が、T のばらつきの記録**（2026-11-15 の V2b の実測は 0、−0.38282、0.56234、1.45610、−0.00690）。

T で、動かさない確認（コマンド A で `STOPPED` の後）。**M2・M3' の実際のコマンド（下の M2・M3' の段と同じ目標・同じ `--pitch-deg -91.0 --roll-deg 0`）に `--dry-run` を付けただけのものを、実機の T で、2 つとも通す**（`--csv` は付けない。`--dry-run` は CSV を作らない）。

```bash
ls ~/so_arm101_check/large_<日付>_M/M1_M2dry.log
python3 -u tools/ik_move_run.py --dry-run --goal 0.200 0.000 0.060 --pitch-deg -91.0 --roll-deg 0 2>&1 \
  | tee -i ~/so_arm101_check/large_<日付>_M/M1_M2dry.log; echo "exit=${PIPESTATUS[0]}"
ls ~/so_arm101_check/large_<日付>_M/M1_M3pdry.log
python3 -u tools/ik_move_run.py --dry-run --goal 0.230 0.060 0.060 --pitch-deg -91.0 --roll-deg 0 2>&1 \
  | tee -i ~/so_arm101_check/large_<日付>_M/M1_M3pdry.log; echo "exit=${PIPESTATUS[0]}"
```

1 つ目（`M1_M2dry`）が M2 の dry-run、2 つ目（`M1_M3pdry`）が **M3' の dry-run**。M3' の段の「M1 の dry-run が通っていなければ飛ばす」は、この 2 つ目（`M1_M3pdry`）を指す。

- **見込み（V2b の実測に近い開始点からのモックの結果。16.8）**: 2 つとも `DRY-RUN: the goal passes every check`、`exit=0`。M2: 先端の移動 約 32 mm、`q_goal [deg]` が −0.003、−10.457、18.326、83.132、0。M3': 約 86 mm、−17.428、9.235、−5.265、87.030、0。wrist_flex の端までの余裕は M2 が約 0.207 rad、M3' が約 0.139 rad（要求 0.02 rad）。
- **M3' の dry-run が通らなければ（`JOINT_LIMIT`・`JOINT_RANGE` など）、M3' は飛ばす**（理由を報告。M2 だけで続ける）。M2 が通らなければ、中止して報告。
- 先端 z ≥ 0.045 m、異音なし、ドライバの WARN なし。

## M2. 小さな一歩（`--goal 0.200 0.000 0.060`）

```bash
ls ~/so_arm101_check/large_<日付>_M/M2.csv ~/so_arm101_check/large_<日付>_M/M2.log
python3 -u tools/ik_move_run.py --goal 0.200 0.000 0.060 --pitch-deg -91.0 --roll-deg 0 \
  --csv ~/so_arm101_check/large_<日付>_M/M2.csv 2>&1 \
  | tee -i ~/so_arm101_check/large_<日付>_M/M2.log; echo "exit=${PIPESTATUS[0]}"
```

手順: ① 開始の確認（`GOAL` の表: `q_goal`、`dq`、`windup before`）を読んで Enter。② 計画の行（`PLAN OK`、長さ 約 3 秒、`first vs desired`・`first vs meas`、`last vs goal`、`lowest z`）を読む。**`PLAN REFUSED` なら何も動かない**（理由を報告）。③ **腕を支え、12 V の近くに手を置いて**、送信の確認で Enter。約 3 秒動き、止まるのを待つ（整定の待ちは 1.5〜8 秒）。④ `ARRIVAL` の表を読む。

- **見るもの**: 到達の表（関節ごとの差、先端の mm）、`desired − q_meas`（手前止まり）、`last vs goal`（0.001 以内）。`first vs desired`（手前止まりが残っていれば 0 でない。0.06 以内）。
- **合格**: `REACHED`（10 mm 以内、終了コード 0）、先端 z ≥ 0.045 m、`FAULT`・WARN・異音なし。CSV が 1 行。
- **`SHORT`（終了コード 4）**: 失敗ではない。値を記録して報告する。**続けて M3' に進む前に、手前止まりの大きさ（`stuck`）を見る**（0.05 rad を超えるなら、報告して指示を待つ。**この 0.05 rad は新しく決めた推測の値**で、`NOT_SETTLED`（desired と測定の差が 0.06 rad 超で拒否）より手前で止めるためのもの。観察では 0.040〜0.053 rad で動く、0.032 rad 以下では動かない。7.16.4 の 1）。
- **拒否・停止**: `START_REFUSED`（3）・`PLAN_REFUSED`・`ARM_MOVED`・`OTHER_GOAL`・`NOT_SETTLED` は、再試行しない。理由を報告する。`NOT_CONFIRMED`（30 秒の無入力）は何も送られていない。同じコマンドを、**CSV の名前を変えて**（`M2_2.csv`・`M2_2.log`。`ls` で確認）再実行してよい。

**T へ戻す（`--joints-deg 0 -22 30 83 0`）**:

```bash
ls ~/so_arm101_check/large_<日付>_M/M2_return.csv ~/so_arm101_check/large_<日付>_M/M2_return.log
python3 -u tools/ik_move_run.py --joints-deg 0 -22 30 83 0 \
  --csv ~/so_arm101_check/large_<日付>_M/M2_return.csv 2>&1 \
  | tee -i ~/so_arm101_check/large_<日付>_M/M2_return.log; echo "exit=${PIPESTATUS[0]}"
```

- 見るもの・合格・扱いは M2 と同じ。**この戻しの終点が、整数度の目標（0、−22、30、83、0 度）からどれだけずれるか**が、`docs/large_move_plan.md` 12 章の「終点のずれ」の実機の確認（`last vs goal` と `ARRIVAL` の `q_meas − q_goal`、`reference − 目標`の 3 つ）。V2b・V8 の RViz では最大 0.0082 rad ずれた。この道具の許容 0.001 では、計画の終点は 0.001 以内のはず（モックで確認済み）。
- `ALREADY_AT_GOAL`（終了コード 2）は、すでに T の許容以内にいて、何も動かなかったという意味（報告だけ）。

## M3'. 約 90 mm の動き（`--goal 0.230 0.060 0.060`）

**M1 の M3' の dry-run（`M1_M3pdry.log`。上の M1 の 2 つ目）が通っていなければ、この段は飛ばす。** M2 の後に T に戻っていること（M2 の戻しの後、コマンド A で `STOPPED`）。

```bash
ls ~/so_arm101_check/large_<日付>_M/M3p.csv ~/so_arm101_check/large_<日付>_M/M3p.log
python3 -u tools/ik_move_run.py --goal 0.230 0.060 0.060 --pitch-deg -91.0 --roll-deg 0 \
  --csv ~/so_arm101_check/large_<日付>_M/M3p.csv 2>&1 \
  | tee -i ~/so_arm101_check/large_<日付>_M/M3p.log; echo "exit=${PIPESTATUS[0]}"
```

- 手順は M2 と同じ。**計画の長さは 5 秒ほど**（モックで 4.96〜5.11 秒）。**動いている間、片手を 12 V の電源の近くに置く**（腕は軌道の最後まで進む。止まらない）。
- **見るもの**: M2 と同じ。加えて、`lowest z`（0.045 以上）、`stuck`（手前止まり）の最大、**先端の誤差が 10 mm を超えるか**。
- **`SHORT` がありうる**。**[推測]** 実機の T から M3' へは、lift +0.54、elbow −0.61 rad（elbow が開く向き＝自重に逆らう向きに約 0.61 rad）、wrist_flex +0.07、pan −0.30 rad 動き、腕も伸びる。手前止まりが T の 0.034 rad より大きくなりうる。10 mm を超えれば `SHORT`（終了コード 4）。失敗ではない。値を記録して報告する（手前止まりの大きさと、先端の誤差の向き）。
- **着いた点（測定）について見る点**（`docs/large_move_plan.md` 16.9）: ① `FK(q_meas).z` と 0.045 m の差（`DiffMode.start` の `FLOOR_MEASURED` の余裕は、目標の床 0.048 との差で 3 mm しかない）。② `desired − q_meas` の最大（`NOT_SETTLED` は 0.06 rad で拒否。T では 0.034 だった）。どちらも記録する。
- 合格: `REACHED` または `SHORT`（記録つき）、先端 z ≥ 0.045 m、`FAULT`・WARN・異音なし。

**T へ戻す**:

```bash
ls ~/so_arm101_check/large_<日付>_M/M3p_return.csv ~/so_arm101_check/large_<日付>_M/M3p_return.log
python3 -u tools/ik_move_run.py --joints-deg 0 -22 30 83 0 \
  --csv ~/so_arm101_check/large_<日付>_M/M3p_return.csv 2>&1 \
  | tee -i ~/so_arm101_check/large_<日付>_M/M3p_return.log; echo "exit=${PIPESTATUS[0]}"
```

戻しも約 5 秒動く（手を電源の近くに）。見るもの・扱いは M2 の戻しと同じ。

## 最後: RViz で出発点へ戻す（V8）

**出発点は床の下（先端 z ≈ 0.033 m）なので、この道具では戻せない**（`START_FLOOR_MEASURED` ではなく、目標の `GOAL_FLOOR` で拒否される）。`diff_mode_real_check.md` の V8（`:140-146`）と同じに、RViz で戻す。2026-11-15 の V8 の実績の目標: **shoulder_pan −4、shoulder_lift −85、elbow_flex +96、wrist_flex +25、wrist_roll −3 度**（出発点との差は最大 0.0184 rad だった）。

1. コマンド A で `STOPPED`。実測を読む。
2. RViz の MotionPlanning: OMPL、Scaling 0.1。Planning タブで **Start State と Goal State を `<current>` に選び直してから**、Joints タブで上の整数の度に設定。Plan → アニメーションを見る（先端が床に近づきすぎない）→ Execute。**elbow_flex は +1.69009 rad を超える値には戻さない**（`servo_real_check.md:341`）。実行の開始位置と実測の差が 0.01 rad 以内であること（`allowed_start_tolerance` 0.01。超えると実行が拒否される。拒否されたら、実測を読み直して計画し直す）。
3. 腕を折りたたみ姿勢で支え、**launch の端末で Ctrl-C を 1 回**（S5）。bag の端末でも Ctrl-C。
4. 停止後の確認: `[deactivate] torque OFF and Lock=1 confirmed …` と `deactivated.` の 2 行、`exit=0`。`ps` と `ros2 node list --no-daemon` が空。

合格: 出発点（V2a の実測 `V2a_start_pose.txt` など）との差が小さい。停止後の確認の 2 行。ps が空。

## 中止の基準（どの段でも）

`diff_mode_real_check.md:148` の基準（ドライバの WARN、異音、先端 z < 0.045 m、`FAULT`・`COMMUNICATION LOST`）に加えて、この道具では:

- M0 で `START_FLOOR_MEASURED` が出ない、M1 以降で `START_FLOOR_MEASURED` が出る（T は床の上のはず）。
- M2 の `dry-run` が通らない。
- `PLAN_REFUSED`、`ARM_MOVED`、`OTHER_GOAL`、`DESIRED_CHANGED`、`NOT_SETTLED`。
- 計画の長さが 20 秒を超える（`DURATION`）。
- `SHORT` で手前止まり（`stuck`）が 0.05 rad を超える。
- 停止後の `[deactivate]` の 2 行が無い（腕を支えて 12 V を切る）。

そこで止めて報告する（再試行しない）。

## 道具の引数（`python3 tools/ik_move_run.py --help` と同じ）

`tools/ik_move_run.py` の `parse_args` と一致（2026-11-15 にコードで確認）:

- `--goal X Y Z`（base_link、m）、`--offset DX DY DZ`（今の先端からの差 [m]）、`--joints-deg DEG × 5`（pan lift elbow wrist_flex roll [度]。各値は ±180 の内側）: **ちょうど 1 つ**。
- `--pitch-deg`・`--roll-deg`: 任意（`--goal`・`--offset` のとき。`--joints-deg` とは併用できない。度であってラジアンではない）。省略は今の姿勢の値。**この手順書の M2・M3' では明示する**（実機の T のばらつきに目標の姿勢が左右されない。値は T の目標の FK: pitch −91.0°、roll 0°）。
- `--moveit-tolerance`（既定 0.001 rad、上限 0.01。この手順書では使わない）、`--dry-run`、`--csv FILE`（既定 `ik_move_<日時>.csv`。この手順書では毎回指定する）。
- 無いもの: `--yes`、scaling、`--route`、`--auto`（変えられない）。移動量の上限（先端 0.15 m、各関節 1.0 rad）も引数では変えられない。
