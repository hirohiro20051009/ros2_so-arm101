# 実機確認手順: 解析 IK の差分モード（P5。`tools/diff_mode_run.py`）

> **【範囲の変更（2026-11-15。ユーザーの方針）】** 実機の確認は **V1、V2a〜V2c、V3、V3b、V8** だけ。V4〜V6 は取りやめ。V7 は V3・V3b の記録の整理だけ。`docs/analytic_diff_plan.md` 7.15 の 9。

計画と根拠: `docs/analytic_diff_plan.md` の 7.14（決定は 7.14.7、反映の記録は 7.14.9）。この手順書の V0〜V8 は 7.14.5 の表と同じ。
止め方 S1〜S6、12 V を切る条件、止まっていることの確認（**コマンド A**）、停止したあとの確認は **`docs/servo_real_check.md` と同じ**（このファイルでは繰り返さない）。出発点・T の関節角と実測は下のとおり。

> **【実機を動かす段（V2a 以降）はユーザーだけが実行する。Claude Code は実機に対して実行しない。】** Claude が実行するのは、モック（ROS_DOMAIN_ID=87）と、V7 の bag・CSV の整理だけ。**実機の確認は V1、V2a〜V2c、V3、V3b、V8 だけ**（V4〜V6 は取りやめ）。

## 共通の約束

1. 実行するのはユーザーだけ。端末は launch 用（ログを流す。操作しない）と操作用の 2 つ以上。止めるのは Ctrl-C（SIGINT）だけ（`kill -9`・`pkill`・`killall` はしない）。
2. **書き込みモード（`read_only:=false`）はトルク ON で書き込む起動**。起動の前に腕を折りたたみ姿勢にして手で支え、止める前にも支える（Ctrl-C でトルクが OFF になり、腕は落ちる）。
3. 床: 先端 z ≥ 0.045 m（測った位置。`DiffMode` は指令の z も 0.048 m 未満を送らない）。腕の周りに物を置かない。
4. 非常停止はサーボの外部電源（12 V）を切る。常に片手を電源の近くに置く。
5. **道具が止まったら（`STOP` の行、終了コード 0 以外）、再試行しない。理由を報告する**。`START_REFUSED` は V1・V2a では期待どおり。
6. `ros2` のコマンドは `--no-daemon` が使えるものには付ける（ros2 daemon には触らない）。
7. 各段で、`ros2 bag record /joint_states /arm_controller/controller_state` と、道具の CSV（`--csv` のパス）を残す。
8. **任意の記録（各段、合否には使わない）**: TCP（指先の間）の床からの高さを定規で測った値（mm）を、同じ時点の道具の `FK(q_meas).z`（`gripper_frame_link` の z。指先の下端はフレームの z より約 6.5 mm 下）と並べて書く。V1 は出発点、V2a・V2c・V3・V3b・V8 は動きの前後に 1 回ずつ。

| 姿勢 | 関節角（pan, lift, elbow, wrist_flex, roll）[rad] | 先端（base_link）[m] | 出どころ |
|---|---|---|---|
| 出発点 `start (W1)` | -0.0951, -1.4952, 1.6901, 0.4219, -0.0453 | z = 0.033（床の下の扱い） | [実測] 2026-11-13（`test_fk_chain.py`） |
| T `T (W1)` | 0.0031, -0.3813, 0.5501, 1.4454, -0.0069 | (0.172, 0.000, 0.055) | [実測] 2026-11-13 |

## 道具の使い方（引数の要点）

- `--route direct`（JTC に直接）または `--route moveit`（MoveIt が `plan_only` で計画 → 検査した軌道を `ExecuteTrajectory` で実行）。**必須。既定なし。実機の手順では `direct` だけを使う**（`moveit` は道具に残してあるが、実機では使わない）。
- `--offset DX DY DZ`（今の先端＝測った位置の FK からの差 [m]）または `--goal X Y Z`（`base_link`）。
- `--max-dq`（既定 0.03 rad）、`--max-steps`（既定 10。V3 は 1、V3b は 4）、`--max-abs-d`（既定 0.99）、`--gain`（既定 1.0）。
- `--auto N`: 最初のステップのあと N ステップを Enter なしで送る（`AT_LIMIT` と `HOLD` は Enter に戻る）。**実機の手順（V1〜V3）では使わない**（V4 以降を取りやめたため）。`--yes` は無い。
- `--origin` は**省略する**（その実行の最初の `start` の位置が起点）。`--offset` の起点は `FK(q_meas)`（今の先端）。`--origin-radius` の既定・上限は 0.05 m。
- `--dry-run`: 何も送らない（クライアントも作らない）。`--assume-desired-is-measured` は `--dry-run` のときだけ受け付ける。
- Scaling は 0.1 固定（引数なし）。`--moveit-tolerance`（既定 0.001 rad）、`--move-seconds`（direct。既定 2 秒。速度は 0.1 rad/s まで）。
- Enter の待ちは 30 秒（無入力なら送らずに終了）。**入力が EOF のとき、`y`・`yes`・空行以外のとき（`n` など）も送らない。まとめて 2 行入力しても、1 回の Enter（1 行）で 1 ステップだけ**。Ctrl-C は**何も送らず、何もキャンセルしない**（終了コード 130。進行中のゴールは指令まで進む）。
- 終了コード: 0 = 正常（`CONVERGED`・`LIMITED`・`DRY_RUN`）、2 = 停止、3 = 開始の拒否、130 = Ctrl-C。

## V0. 共通の約束の確認

約束 1〜8 を確認する。bag と CSV の保存先を決める（例: `~/so_arm101_check/diff_<日付>_V<段>/`）。

## V1. 読み取り専用・置いたままの出発点（腕を支えない）

起動（トルク OFF。書き込まない。腕は出発点＝折りたたみ姿勢に置いたまま）:

```bash
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=true use_rviz:=false
```

操作用の端末で:

```bash
python3 tools/ik_model_check.py
python3 tools/diff_mode_run.py --route direct --dry-run --assume-desired-is-measured --offset 0.002 0 0
```

合格:

- (a) 解析 FK と MoveIt の FK の差（今の姿勢）が 0.1 mm 以内。roll の掃引の差が連続。
- (b) IK の往復の行は、成功したものは差 0.5 mm 以内。失敗した行は理由が表に出ている（出発点は elbow が上限を 0.0001 rad 超え、先端 z が床の下なので、範囲・床の理由で失敗する行がありうる。失敗は不合格にしない）。
- (c) `--dry-run` の `start` が**拒否される**（`JOINT_LIMIT`・`FLOOR`・`FLOOR_MEASURED` の理由が出る見込み）。**拒否されることが合格**。何も送られていない（CSV も作られない）。

## V2a. 書き込みの起動（トルク ON）。出発点のまま、何も送らない

**ここからトルク ON・書き込みモード。** 腕を折りたたみ姿勢にして、前腕の下を支える。

```bash
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=false torque_limit:=500 2>&1 \
  | tee -i ~/so_arm101_check/diff_V2a.log; echo "exit=${PIPESTATUS[0]}"
```

操作用の端末で: `ros2 control list_controllers --no-daemon`、コマンド A、`python3 tools/diff_mode_run.py --route direct --dry-run --offset 0.002 0 0`（`--assume-desired-is-measured` は付けない）。

合格: `arm_controller` が active／`controller_state` が来る／`reference` と `q_meas` の差が 5 関節とも 0.06 rad 以内／コマンド A が `STOPPED`／`--dry-run` の `start` が出発点の理由で拒否される／ドライバの WARN（`step limit` など）なし／bag が取れている。**指令は送らない。**

## V2b. T へ移動（W1 の手順。ユーザーが実行）

`docs/servo_real_check.md` の W1（RViz の MotionPlanning で OMPL、Velocity・Acceleration Scaling 0.1、Goal State を整数の度で T に設定 → Plan → Execute）。出発点の実測を控える（V8 で戻す先）。この段で P5 の道具は使わない。合格は W1 の基準（T に着いた／止まった／WARN なし／先端 z ≥ 0.04 m）。

## V2c. T で動かさない確認

コマンド A、`python3 tools/ik_model_check.py`、`python3 tools/diff_mode_run.py --route direct --dry-run --offset 0.002 0 0`。

合格: コマンド A が `STOPPED`／`reference` と `q_meas` の差が 0.06 rad 以内／`start` が通る／先端 z ≥ 0.045 m（先端は T の付近 (0.172, 0.000, 0.055) m）／FK の差 0.1 mm 以内／WARN なし／bag が取れている。

**ここで止まり、ユーザーが「V3 に進む」とチャットで答えてから V3 に進む（これが最初の送信の許可）。**

## V3. JTC 直接・1 ステップ（最初の送信）

```bash
python3 tools/diff_mode_run.py --route direct --max-steps 1 --offset 0.005 0 0 --csv ~/so_arm101_check/diff_V3_px.csv
python3 tools/diff_mode_run.py --route direct --max-steps 1 --offset -0.005 0 0 --csv ~/so_arm101_check/diff_V3_mx.csv
```

**offset は 5 mm**（+x 5 mm、続けて −x 5 mm）。起点は `FK(q_meas)`（今の先端）。不感帯は 3 mm なので、2 mm では誤差が不感帯の内側になり、何も送られない（道具は「目標は不感帯の内側（誤差 X mm < 3 mm）。送りません。」と表示して終わる。その場合は offset を 3 mm より大きくする）。T の 5 mm の最初のステップの δ は +x だけの約 2.3 mm（`--max-dq 0.03` で縮める。[計算]）。

1 ステップごとに表示を見て Enter（`n` で終了）。**`--max-steps 1` なので、1 ステップ送ったあとで終わる。`Outcome` は `LIMITED`（`limit_reason` は `MAX_STEPS`）か、1 ステップで残りの誤差が 3 mm 以内になれば `CONVERGED` のどちらも正常**（終了コード 0。残りの誤差が 3 mm を超えて `LIMITED`（`MAX_STEPS`）になるのが、摩擦で十分に動かなかったときの形。**モック（摩擦なし）では 5 mm の offset で、1 ステップの δ 2.28 mm のあと残り 2.7 mm になり、`CONVERGED` で終わる**。この段の目的は、最初の送信と、積み上げでサーボが実際に動くかを見ること）。

合格:

- 表示どおりの `q_cmd` が送られた。
- ゴールが `SUCCEEDED`（到達の証拠ではない）。
- 止まったあとの `q_desired` が `q_cmd` と 1e-4 rad 以内。
- `q_meas` が `q_cmd` へ向かった。先端の動き ÷ δ を記録する（0 でも失敗ではない: 摩擦で止まる [推測]）。
- 送る前後 1 秒の `desired − measured` に垂れの兆候なし（elbow が閉じる向きに数 tick 動かない）。
- 先端 z ≥ 0.045 m、異音なし、ドライバの WARN なし。
- `Outcome` が `LIMITED`（`MAX_STEPS`）または `CONVERGED` で終わる（終了コード 0）。`REFUSED`・タイムアウトは不合格（報告）。

## V3b. JTC 直接・積み上げの確認（2 歩目以降）

V3 は 1 ステップなので、2 歩目の積み上げ（前回の `desired` に積み上がるか）を確かめられない。V3 が合格してから行う。**`--auto` は使わない**（1 ステップごとに Enter）。各実行は T から始める（実行のあと、V2b と同じ手順で T へ戻す）。

```bash
python3 tools/diff_mode_run.py --route direct --max-steps 4 --offset 0.008 0 0 --csv ~/so_arm101_check/diff_V3b_px.csv
python3 tools/diff_mode_run.py --route direct --max-steps 4 --offset -0.008 0 0 --csv ~/so_arm101_check/diff_V3b_mx.csv
```

+x 8 mm、続けて −x 8 mm。**`CONVERGED` まで行かなくても失敗にしない**（`LIMITED`（`MAX_STEPS`・`SPIN`・`REVERSAL` など）で終わってもよい。8 mm は 1 ステップ約 2.3 mm なので、4 ステップで足りるかは摩擦しだい）。

合格:

- `REFUSED`・ドライバの WARN（`step limit` など）・異音がない。タイムアウト・`NOT_SETTLED`・`OTHER_GOAL`・`DESIRED_MISMATCH` で止まらない。先端 z ≥ 0.045 m。
- **2 歩目以降の `q_cmd` が、前回の `q_desired` に積み上がっている**: CSV で、ステップ k+1 の `q_desired_before` がステップ k の `q_cmd` と 1e-4 rad 以内（`desired` は前回の指令のまま）、かつ ステップ k+1 の `q_cmd` と `q_desired_before` の差（dq）が `max_dq`（0.03 rad）以内。**`q_meas`（測った位置）から作り直されていないこと**（`q_cmd` が `q_meas_before` ではなく `q_desired_before` から進む）。
- **各ステップで記録する**: 「先端の動き ÷ δ」（`tip_move_over_delta`）、ワインドアップの量（`windup_after` = 止まったあとの `q_cmd` と `q_meas` の差の最大 [rad]。0.06 rad を超えない）、`track_gap`（指令した先端と測った先端の差）、整定の秒数（`wait_settle`）。先端の動き ÷ δ が 0 に近いステップが続くときは、ワインドアップが増えているか（積み上げが摩擦を破るか）を見る。

## V4〜V6（取りやめ。2026-11-15。ユーザーの方針）

±3 mm への追い込み（V4）、MoveIt 経由（V5）、交互の比較（V6）は、実機では行わない。大きな移動は MoveIt の姿勢の目標（経路計画）が基本で、細かい補正は俯瞰カメラの画像との差分で `DiffMode` を回す。**画像なしで、実機で ±3 mm に追い込む確認はしない**。この手順書の目的は、「積み上げでサーボが実際に動くか」を、カメラの誤差と混ぜずに確かめること（V3）。`--route moveit` と `--auto` は道具に残っているが、実機の手順では使わない（`docs/analytic_diff_plan.md` 7.15 の 9・11）。

## V7. V3・V3b の記録の整理（Claude が bag と CSV から。実機を使わない）

V3・V3b の CSV（`tip_move_over_delta`・`track_gap`・`q_desired_before`・`q_desired_after`・`wait_settle` など）と bag（`/joint_states`・`controller_state`）から、次を表にする: 先端の動き ÷ δ、送る前後の `desired − measured`（垂れ）、整定の秒数、止まったあとの `q_desired` と `q_cmd` の差。**積み上げでサーボが動いたか**（`q_meas` が `q_cmd` に向かったか。V3b の 2 歩目以降で `windup_after` が増え、先端の動き ÷ δ が戻るか）の材料にする。`max_dq`・abs(D) の見直しの材料は、V3・V3b では足りないので、カメラの段で決める。

## V8. 出発点へ戻す（W6 の手順。ユーザーが実行）

`docs/servo_real_check.md` の W6: 動きが止まっているのを確認 → 実測を読む → 戻し先は V2b で控えた出発点の実測 → OMPL（Scaling 0.1）で戻す（**elbow_flex は +1.69009 を超える値には戻さない**。実行の開始位置と実測の差が 0.01 rad 以内）。戻したら S5（腕を折りたたみ姿勢で支えて Ctrl-C 1 回）、bag を止める。

合格: 出発点の実測との差が小さい（W6 の基準）／停止後の確認の 2 行（`torque OFF and Lock=1 confirmed`・`deactivated`）／`ps` と `ros2 node list --no-daemon` が空。

## 中止の基準（どの段でも）

`REFUSED`（V1・V2a の `start` の拒否は期待どおりで除く）、タイムアウト、ドライバの WARN、異音、先端 z < 0.045 m、`desired` の不一致、`SPIN` が出たら、そこで止めて報告する（再試行しない）。
