# 大きな移動の道具の計画: 解析 IK で関節の目標を出し、MoveIt に関節の目標だけを渡す

2026-11-15。**計画だけ**。コード・ビルド・モック・colcon test は実施していない（読むことだけで作った）。実装は承認後。実機は Claude Code は動かさない（確認はユーザーが手順書で行う）。

- 前提: `analytic_diff_plan.md` の 7.14〜7.16（P5 の `step_runner.py`・`tools/diff_mode_run.py`、実機の確認 V1〜V8）。HEAD は 9359561 の後。
- ユーザーの方針（2026-11-15）: 大きな移動は解析 IK で関節の目標を出し、MoveIt には関節の目標だけを渡して経路計画させる（MoveIt の IK は使わない）。一回の誤差は 1 cm 程度まで許容。細かい補正は俯瞰カメラの差分で DiffMode を回す。
- 書き方: **事実**（コード・docs・ログで確かめたもの。ファイル:行）と **[推測]** を分ける。新しく決める値は「推測の値」と書く。表のセルに縦線は使わない。
- 略記: `sr` = `src/so_arm101_ik/so_arm101_ik/step_runner.py`、`dm` = `.../diff_mode.py`、`aik` = `.../analytic_ik.py`、`run` = `tools/diff_mode_run.py`。

## 0. 先に分かったこと（計画に効く 4 つ）

1. **P5 の moveit 経路の「最初の点は q_desired と 0.005 rad 以内」は、この道具では使えない。**
   - 事実: MoveIt の計画は測定（`/joint_states`）から始まる（モックの 3a。7.15 の 12）。
   - 事実: 実機の T で elbow は desired から 0.0337 rad 手前で止まっていた（7.16.4 の 4）。
   - よって、実機で一回動かした後は、最初の点と desired の差が毎回 0.005 を超え、`FIRST_DESIRED` で毎回拒否される。許容を替える（決定 7）。
2. **出発点（rest）は床の下。** 先端 z = 0.0325〜0.033 m（7.16.2 の V1）。床 0.048 m、測定の床 0.045 m を下回る。rest からの持ち上げは、この道具では拒否する（決定 6、A）。最後の戻しは RViz の V8 のまま。
3. **進行中のゴールを止める手段が実質ない。** P5 の道具はキャンセルしない。S1〜S4 は Servo 用（`servo_real_check.md:32-35`）で、Servo は 8dc64f2 で削除済み。残る S5・S6 はどちらも腕が落ちる。大きな移動は約 6 秒動く（B）。
4. **一回の誤差の見込みは 1 cm の許容にぎりぎり。** T の手前止まりは先端で 7.39 mm（7.16.4 の 4）。[推測] 手前止まりが 0.04 rad に達する姿勢では 10 mm を超えうる。道具は正直に出す（6 章）。

## 1. ユーザーの決定（2026-11-15）

- 決定 1〜5、7〜14、16: 推奨どおり。
- 決定 6: 拒否のまま。拒否を出す検査は A のとおり。
- 決定 15: M0〜M3 と、最後は RViz の V8。F を反映。
- 決定 17（新しい）: 移動量の上限。「先端の移動 `abs(FK(q_goal) − FK(q_meas))` が 0.15 m 以内」「各関節の `abs(q_goal − q_meas)` が 1.0 rad 以内」を、目標の入口の検査に入れる（どちらも推測の値。打ち間違いの `--offset 0.3` などを止めるため）。守りを外す確認の対象に入れる。
- 直し A〜F: 2〜12 章に反映した（対応は 14 章の表）。

決定の一覧（確定）:

| # | 項目 | 決定 |
|---|---|---|
| 1 | 計画の記録場所 | このファイル `docs/large_move_plan.md`。`analytic_diff_plan.md` には 1 行のリンクだけ |
| 2 | 名前と置き場所 | `so_arm101_ik/ik_move.py`（純粋）と `tools/ik_move_run.py`（ROS の入出力） |
| 3 | 入力 | `--goal` / `--offset` / `--joints-deg` のどれか 1 つ。`--pitch-deg`・`--roll-deg` は任意で、省略は FK(q_meas) の値 |
| 4 | 枝 | q_meas に近い枝（5 関節の 2-norm）。q_meas と違う枝は `BRANCH_SWITCH` で拒否 |
| 5 | 目標の検査の値 | DiffMode の既定（床 0.048、radial 0.05、abs(D) 0.99、端の余裕 0.02 rad）。入口は `DiffMode.start(q_goal, q_goal)` |
| 6 | 開始位置が床の下 | v1 は拒否。検査は 3 章の `START_FLOOR_MEASURED`（A） |
| 7 | 最初の点と desired の許容 | 0.06 rad（推測の値）。測定との許容は 0.01 のまま |
| 8 | 全点の床・範囲の検査 | やる |
| 9 | 共通部分の切り出し | 今はしない。純粋な関数は import、状態を持つ部分はコピー。整理のときに切り出す |
| 10 | 計画の長さの上限 | 20 秒（推測の値） |
| 11 | Ctrl-C でキャンセル | v1 はしない。モックでキャンセルの挙動を確かめてから別に決める |
| 12 | 到達の判定と終了コード | 10 mm 以内で `REACHED`（0）、超えたら `SHORT`（4）。3 mm 超は補正の案内 |
| 13 | Scaling と許容 | Scaling 0.1 固定。`--moveit-tolerance` は既定 0.001、上限 0.01 |
| 14 | 手順書 | 新しいファイル `docs/large_move_real_check.md` |
| 15 | 実機の段 | M0〜M3、最後は RViz の V8（11 章） |
| 16 | `CMakeLists.txt` | 試験名 3 つの追加だけ。実装の前に承認をもらう |
| 17 | 移動量の上限 | 先端 0.15 m、各関節 1.0 rad（どちらも推測の値）。**この組み合わせでは、出発点（rest）から T へは動かせない**（出発点は床の下なので開始の検査 `START_FLOOR_MEASURED` で拒否される。lift・elbow の移動も 1.0 rad を超える）。**出発点と T の間の行き来は RViz が受け持つ**（ユーザー確認済み、2026-11-15） |

## 2. 入力の形

事実:
- P5 は `--goal` か `--offset` のどちらか 1 つだけ受ける（`sr:376-378`）。
- 解析 IK は x・y・z・pitch・roll を取る（`aik:169` の `ik`）。`solve_ik` は pitch・roll を省略すると、今の関節角の FK の値を使う（`aik:187-196`）。
- pitch は接近軸の向き、roll は関節角 S5·q5 そのもの（`aik` の docstring）。

案:
- `--goal X Y Z`（base_link、m）、`--offset DX DY DZ`（FK(q_meas) からの差）、`--joints-deg q1 q2 q3 q4 q5` のうち、ちょうど 1 つ。
- `--pitch-deg`・`--roll-deg` は任意。省略は FK(q_meas) の pitch・roll（`--goal`・`--offset` のとき）。`--joints-deg` と一緒には指定できない。
- **度の取り違えの検査（D）**: `--joints-deg` の 5 つ、`--pitch-deg`、`--roll-deg` の各値は、有限で、`abs(値) ≤ 180` でなければ、ノードを作る前に拒否する（`validate_move_options`。`sr:363` の `validate_run_options` と同じ位置づけ）。
  - 範囲の外の例: `--roll-deg 3.14` は通る（3.14 度として）。ラジアンの打ち間違いは、この検査では見つからない。この種の間違いは、各関節の範囲の検査（端の余裕 0.02 rad）と、移動量の上限（決定 17）が拾う。
- 内部の単位は rad。度は入口で rad に直す。CSV と表示は rad と度の両方を出す。

## 3. 入口の検査

### 3.1 順序と、理由の集め方

1. 引数（有限、±180 度、組み合わせ）→ ノード・クライアントを作る前。
2. 入力（q_meas は必須。q_desired は実行のとき必須、`--dry-run` では任意）。
3. 開始の検査（3.2）。
4. 目標の解法と検査（3.3）。
5. 移動量の上限（3.4）。
6. 1 つでも外れたら、**すべての理由を集めて表示**して終了コード 3（`StartRefused` と同じ形）。何も送らない。

### 3.2 開始の検査（A: 決定 6 の拒否を出す検査）

**事実**: `DiffMode.start(q_goal, q_goal)` の `FLOOR_MEASURED`（`dm:450-452`）は、第 2 引数の測った位置の床（`measured_floor_z` 0.045 m、`dm:129`）の検査。**この呼び方では q_meas ではなく目標を見ている**。したがって今の姿勢の床は見ていない。ユーザーの指摘のとおり。

**決定（Claude が選んだ。承認を待つ）**: **開始時に FK(q_meas) の z を検査する**。新しい検査で、理由の名前は `START_FLOOR_MEASURED`。
- 条件: `ik.fk(q_meas)` の z が 0.045 m 未満なら拒否。値は `DiffModeConfig.measured_floor_z`（`dm:129`、H3 案 P。`analytic_diff_plan.md` 7.12.4 の出どころの表）。
- 計画の前（`plan_only` の前）に落とす。したがって `--dry-run`（計画しない）でも効く。
- 全点の床の検査（3.5）の最初の点は、同じ値（0.045）で見る。最初の点は q_meas なので、最初の点で別の理由で落ちることはない（2 重の検査で、理由は常に `START_FLOOR_MEASURED`）。
- `DiffMode.start(q_goal, q_goal)` が返す `FLOOR_MEASURED` は、目標の z が 0.045 未満のとき（このとき `FLOOR`（0.048）も同時に出る）。ユーザーに混乱がないよう、道具は `GOAL_FLOOR` に統一して表示し、`FLOOR_MEASURED` は表示しない。

**M0 の見込み（rest で `--dry-run`）**: 理由に `START_FLOOR_MEASURED`（出発点の z = 0.0325 m は 0.045 未満。事実: 7.16.2 の V1）が出て、終了コード 3、何も送らない。[計算: `analytic_diff_plan.md` の T の値と出発点の値から手計算] 目標が T の付近なら `MOVE_JOINT_TOO_FAR` も出る（出発点の lift −1.4952 → T の −0.3813 で 1.114 rad、elbow 1.6901 → 0.5501 で 1.140 rad。どちらも 1.0 rad を超える）。

### 3.3 目標の検査

| 検査 | 使うもの | 判定 |
|---|---|---|
| 入口のまとめ | `DiffMode.start(q_goal, q_goal, cfg)`（`dm:395`）。理由の一覧は `StartRefused.reasons`。FLOOR（z < 0.048）・RADIAL（< 0.05）・SINGULAR（abs(D) > 0.99）・JOINT_LIMIT（厳密な範囲）・NO_BRANCH | そのまま使える。`NOT_SETTLED` は q_desired = q_meas なので出ない。`FLOOR_MEASURED` は上のとおり読み替える |
| UNREACHABLE | `aik:132` の `ik_geometry` が `IKUnreachableError`（r² < 0、または abs(D) > 1 + 1e-9） | そのまま使える（`--goal`・`--offset`） |
| JOINT_RANGE（厳密） | `aik:169` の `ik()`（`IKJointLimitError`）。`--joints-deg` は `aik:102` の `within_limits` | そのまま使える |
| JOINT_RANGE（端の手前 0.02 rad） | 値は `dm:144` の `margin_joint`。検査は `DiffMode._excess`（`dm:487-512`）の中で private | 値だけ使い、検査は新しく書く（5 関節すべて。`--joints-deg` にも同じ） |
| SINGULAR の余裕 | 値は `max_abs_d` 0.99 と `margin_abs_d` 0.001。`dm:275` の `abs_d_of_q`（q だけから）。`ik_geometry` の `cos_elbow` | 判定は新しく書く（DiffMode の目標の縮めと同じ値） |
| 床 | `floor_z` 0.048（`dm:128`）。`aik` の `fk` | `FK(q_goal).z ≥ 0.048` を新しく書く。目標の z が 0.048 未満の `--goal` は IK の前に拒否 |
| 枝 | `dm:268` の `branch_of_q`、`aik:178` の `select_nearest_solution` | 3.6 |
| `verify_command`（`dm:310`） | DQ_TOO_BIG と wind-up の検査があり、大きな移動では必ず落ちる | 使えない |

事実: 値（0.048、0.045、0.05、0.99、0.02）はすべて `DiffModeConfig` の既定（`dm:128-144`）。DiffMode の出どころ（7.12.4）をそのまま引き継ぐ。これにより、着いた点は DiffMode の開始点になれる。

### 3.4 移動量の上限（決定 17）

- `MOVE_TIP_TOO_FAR`: `norm(FK(q_goal) − FK(q_meas)) > 0.15 m`（推測の値。打ち間違いを止める）。
- `MOVE_JOINT_TOO_FAR`: 5 関節のどれかで `abs(q_goal − q_meas) > 1.0 rad`（推測の値）。角度の差は包まない（関節の範囲が ±π の内側なので、差をそのまま使う）。
- 値は `ik_move.py` の定数（`MAX_TIP_MOVE`、`MAX_JOINT_MOVE`）。引数では変えられない（変える引数を作らない。静的試験で確かめる）。

### 3.5 計画の軌道の全点の検査

計画の結果（`planned_trajectory`）の全点について、新しく書く:
- 厳密な関節範囲。
- z ≥ 0.045 m（FK）。目標の点は 3.3 で 0.048 以上。**全点を 0.045、目標を 0.048 にする理由（ユーザー承認、2026-11-15）**:
  - 0.045 は、DiffMode が測った位置に使う床（`measured_floor_z`、`dm:129`）と同じ値で、DiffMode の規則と揃う。目標と指令は 0.048（`floor_z`、`dm:128`）。
  - 最初の点は q_meas で、z が 0.045〜0.048 のことがある（`START_FLOOR_MEASURED` は 0.045 未満だけを拒否する）。全点を 0.048 にすると、このとき最初の点で落ちて、開始の検査と食い違う。0.045 なら、最初の点では必ず通る。
- 点の間は、関節空間で線形に補間し、**関節の角度差が 0.01 rad 以下の間隔**（推測の値）で FK を取る。

事実（ログ）: 時間最適化の `resample_dt` は 0.1 秒、`path_tolerance` は 0.1 で、どちらも既定値（`~/so_arm101_check/diff_20261115_V2/V2_launch.log:85,101-102`。git 管理外）。[計算] 0.3 rad/s で動く点の間隔は約 0.03 rad になるので、補間が要る。
[推測] SRDF にはリンク間の衝突の設定（15 件）だけで、床の物体はない。MoveIt は軌道が床の下を通るかを見ない。

### 3.6 枝（D を含む）

- **`--goal`・`--offset`**: `ik()` が返す、範囲内の枝の候補から、q_meas に最も近いものを選ぶ（`select_nearest_solution`。5 関節の角度差を包んだ 2-norm。q1・q5 は 2 つの枝で共通なので、実質 q2・q3・q4 の差）。基準は q_meas（MoveIt の計画が q_meas から始まるため）。選んだ枝が `branch_of_q(q_meas)` と違えば `BRANCH_SWITCH` で拒否。
- **`--joints-deg`**: 枝の選択はない。**`branch_of_q(q_goal)` が `branch_of_q(q_meas)` と違えば `BRANCH_SWITCH` で拒否する（D）**。加えて、`DiffMode.start(q_goal, q_goal)` の `NO_BRANCH`（FK → IK で q_goal が一意の枝に戻らない）も拒否。
- 他方の枝が範囲内に残る頻度は計算していない（実行しない約束のため）。実装後の試験で数える。

## 4. P5 の moveit 経路の使い回し

| 要素 | 場所 | 判定 |
|---|---|---|
| plan_only のゴール | `PlanGoalSpec`・`build_plan_goal_spec`（`sr:311-342`。`plan_only` は常に真）。`MoveItSender.prepare`（`run:194-238`）で `spec.plan_only` を設定（:214） | spec はそのまま使える。送信部分は新しい道具にコピー |
| 最初の点（測定との差） | `check_planned_trajectory`（`sr:247-308`）の `first_meas_tol`（既定 0.01、:84。MoveIt の `allowed_start_tolerance`） | そのまま使える |
| 最初の点（desired との差） | 同じ関数の `first_desired_tol`（既定 0.005、:83） | **引数だけ変える**（0.06。決定 7）。関数は変えない |
| 最後の点（目標との差） | 同じ関数の `tolerance`。q_goal と比べる | そのまま使える |
| 最大の速さ | 同じ関数の `max_speed`（0.5 rad/s、:71）。位置の差と velocities の両方 | そのまま使える |
| 全点の床・範囲 | なし | 新しく書く（3.5） |
| 排他の 3 重 | `STATUS_TOPICS`（`run:52-56`）、`other_active_goals`（`run:106-117`）、`ACTIVE_STATUSES`（`sr:90`） | コピー |
| 送る直前の再検査 | `StepLoop._recheck`（`sr:757-777`） | **新しく書く**（5 章） |
| 2 段の確認 | `LineReader`（`sr:437`）、`confirm_answer`（:467）、`CONFIRM_TIMEOUT` 30 秒（:58）。`RunNode._poll`・`confirm`（`run:119-129`） | 純粋な部分はそのまま使える。ノード側はコピー |
| 送信と結果 | `MoveItSender.execute`（`run:240-256`）、`classify_goal_result`（`sr:345`） | コピー |
| 整定 | `StepLoop._settle`（`sr:599-610`）。`self._read` に依存 | 小さな関数にして新しく書く（約 25 行）。値は `SETTLE_MIN`・`STOP_INTERVAL`・`STOP_TOL`・`SETTLE_TIMEOUT` を使う |

確認の順（P5 の「確認 → 計画 → 実行」から変える）:
1. 開始の確認（表: 入力・枝・検査・q_goal）。
2. `plan_only` で計画（何も動かない）。
3. 計画の検査と要約の表示（最初の点・最後の点の差、最大の速さ、長さ、全点の最小 z）。
4. 送信の確認。
5. 再検査 → `ExecuteTrajectory`。

共通部分の切り出し（決定 9）: 今はしない。理由は、`so_arm101_ik` に ROS の import を入れると `step_runner.py` に ROS を import しない静的試験や `package.xml` の依存に響くこと、tools/ に共通ファイルを作ると既存の道具を変更することになること、DiffMode の moveit 経路はリリース時の整理の対象であること。整理のとき、新しい道具が唯一の moveit 経路になるので、そこで切り出す。

## 5. 送る直前の再検査（C）

`ik_move.py` の `recheck`。すべての送る経路（送る経路は `ExecuteTrajectory` の 1 つだけ）の直前で、毎回やる。

| 検査 | 許容 | 出どころ |
|---|---|---|
| q_meas が計画のときから動いていない | 5 関節とも 0.003 rad 以内 | **既存の値**: `STOP_TOL`（`sr:59`。コマンド A。`StepLoop._recheck` の `sr:759-760` と同じ比較） |
| q_desired（reference）が計画のときから変わっていない | 5 関節とも 1e-4 rad 以内 | **既存の値の再利用**: `DESIRED_TOL_DIRECT`（`sr:74`）。もとは推測の値。根拠は、止まっているとき reference は変わらず、実機の V3・V3b で reference と q_cmd が 1e-5 以内で一致したこと（7.16.3 の T4）。何も送らない間の比較なので、この厳しさで足りる **[推測]** |
| 入力が新しい | 0.3 秒以内 | 既存の値: `MAX_AGE`（`sr:56`） |
| 他のアクションにアクティブなゴールがない | 0 件 | 既存の `other_active_goals` |
| 送る軌道に対して、目標・全点の検査をもう一度 | 3.3・3.5 と同じ | 新しい（同じ関数を呼ぶ） |
| 最初の点が q_meas と q_desired から許容の内側 | 0.01 と 0.06 | 4 章 |

どれか 1 つでも外れたら送らずに止める（終了コード 2）。静的試験で、送る呼び出しの前に必ず再検査があることを確かめる（9 章）。

## 6. 到達の判定

- `SUCCEEDED` は根拠にしない（`open_loop_control: true`、`controllers.yaml:50`。`classify_goal_result`、`sr:345`）。ゴールの status は「受理され、終わった」ことだけに使う。
- 整定してから測る（`SETTLE_MIN` 1.5 秒の後、コマンド A: 0.6 秒間隔の 2 回の測定で差が 0.003 rad 以下。最大 8 秒）。
- 表示と CSV: 関節ごとの `q_meas − q_goal`（rad）、先端の `FK(q_meas) − 目標`（ベクトルと大きさ [mm]）。参考として `FK(reference) − 目標`（計画の終点のずれ）と `reference − q_meas`（手前止まり e、関節ごと）。
- ラベル（表示と終了コードだけ。失敗にしない）:
  - 10 mm 以内: `REACHED`（終了コード 0）。10 mm は、ユーザーの方針（2026-11-15。1 cm 程度）。
  - 10 mm 超: `SHORT`（終了コード 4。推測の値）。
  - 3 mm 超: 「DiffMode・カメラの段で補正できる」と表示（3 mm は DiffMode の不感帯 `deadband_enter`）。
- 見込み（一回の誤差）:
  - 事実: V2b の T で `FK(desired)` と `FK(q_meas)` の差が 7.39 mm（7.16.4 の 4）。
  - [計算] 計画の終点のずれは、許容 0.001 rad で最大約 0.3 mm 相当（腕の長さ 0.3 m 程度の概算）。
  - [計算] サーボの 1 tick（0.00153 rad、7.16.1）は約 0.5 mm 相当。
  - [推測] 合計は 8 mm 前後。手前止まりが 0.04 rad に達する姿勢（自重に逆らう関節。観察では 0.040〜0.053 で動く。7.16.4 の 1）では 10 mm を超えうる。
- 事実: 開始時の desired と q_meas の差が大きいと、DiffMode の `NOT_SETTLED`（`dm:446-449`、0.06 rad）で止まる。この道具の終了後は e が残るので、DiffMode に渡す前に表示する（7 章）。

## 7. DiffMode（direct）の後に使うとき

- 事実: 積み上げの `windup_after` は 0.0179〜0.0349 rad（7.16.3 の T1）。DiffMode の `max_windup` は既定 0.06、硬い上限 0.08（`dm:71`、:133）。
- 事実: 計画は測定から始まる（7.15 の 3a）。
- [推測] 実行の瞬間に reference が測定へ飛び、支えられていた分が抜けて、わずかに垂れる恐れがある（`analytic_diff_plan.md` 2.2 の懸念）。

案: **表示だけ**。開始時に「desired と q_meas の差の最大 X rad。積み上げた分は MoveIt の計画で捨てられる」と表示し、CSV に出す。0.06 rad（決定 7）を超えたら、最初の点の検査で止める。何かして解く案（事前に 1 歩で寄せる）は、書き込み経路が増えるので採らない。

## 8. 止め方・長さ・終了コード（B を含む）

### 8.1 計画の長さの見積り（B: 加速度の出どころ）

- **事実**: `joint_limits.yaml` に加速度の制限がある。5 関節とも `has_acceleration_limits: true`、`max_acceleration: 1.0`（`src/so_arm101_moveit_config/config/joint_limits.yaml:14-15, 19-20, 24-25, 29-30, 34-35, 39-40`）。速度は 5.0 rad/s（:12-13 ほか）。`default_velocity_scaling_factor`・`default_acceleration_scaling_factor` は 0.1（:5-6）。
- **事実**: 道具はリクエストで Scaling を 0.1 に設定する（`run:200-202`、`sr:78` の `MOVEIT_SCALING`）。
- [計算] 実際に使われる値は、加速度 1.0 × 0.1 = 0.1 rad/s²、速度 5.0 × 0.1 = 0.5 rad/s。
- [計算] 距離 d [rad] の台形の速度の山は、d が 2.5 rad 未満なら三角形（山の速さ = sqrt(0.1 × d)、時間 = 2 × sqrt(d / 0.1)）。d = 1.0 で約 6.3 秒、山の速さ約 0.32 rad/s（0.5 rad/s の上限の内側）。上限 1.0 rad の動きの最大は約 6.3 秒。
- **整合の確認（事実 + 計算）**: V2b の実行は、`Execution request received`（`V2_launch.log:219`、エポック秒 1794728429.75）から `Controller 'arm_controller' successfully finished`（:232、1794728436.62）まで約 6.87 秒。そのときの最大の関節の移動は、rest → T で elbow 1.140 rad（7.16.2・手順書の値から手計算）。三角形の式で 6.75 秒。**6.87 秒と 1.8% 以内で一致**した。実行時に使われる値の直接の確認（`ros2 param get`）はしていない。
- 以上から、計画の長さの上限 20 秒（決定 10）は、移動量の上限 1.0 rad（約 6.3 秒）の 3 倍の余裕。ずれた設定を拾う安全網として置く。

### 8.2 Ctrl-C

事実: P5 の道具は Ctrl-C で何も送らず、何も取り消さない（終了コード 130。`run:335-337`、`sr` の `StepLoop.run`）。direct はモックで確認済み（7.15 の 12 の 4）。`ExecuteTrajectory` の Ctrl-C は未確認。

| 時点 | Ctrl-C で止まるもの | 止まらないもの |
|---|---|---|
| 確認の待ち・計画中 | 道具だけ。何も動いていない。plan_only の計画は捨てられる | なし |
| `ExecuteTrajectory` の実行中 | 道具だけ | [推測] 進行中のゴール（クライアントが消えてもサーバーは続ける。direct と同じ）。最大約 6.3 秒、腕は検査済みの軌道を最後まで動く |
| 結果のあと・整定中 | 道具だけ | なし（腕は止まっている） |

- 実行中に止める手段は S5（launch の Ctrl-C。トルク OFF で腕が落ちる）と S6（12 V を切る）だけ。S1〜S4 は Servo の削除で使えない。手順書に「手を電源の近くに置く」「S5 の前に腕を支える」を書く。
- キャンセルはしない（決定 11）。理由: 全点を検査済みの軌道を最後まで動かす方が、測った位置で保持して垂れるより危険が小さいと見込む **[推測]**。モックで `ExecuteTrajectory` のキャンセルの挙動を確かめてから、別に決める。
- 終了コード: 0 `REACHED`、2 停止、3 開始の拒否、4 `SHORT`、130 Ctrl-C（既存は `sr:92-95`）。

## 9. 静的試験で保証すること（`test_ik_move_run_static.py`）

AST。ROS を import しない。`test_diff_mode_run_static.py` と同じ形（`FORBIDDEN_NAMES` などを流用）。

| 保証すること | 方法 |
|---|---|
| `send_goal_async` は 2 か所だけ（`MoveGroup` の計画 1・`ExecuteTrajectory` の実行 1）。`FollowJointTrajectory` のクライアントは無い | クラスと型で区別 |
| `plan_only` は `spec.plan_only` だけで決まり、他の代入・キーワードがない | 既存の試験の形 |
| `cancel_goal`・`create_publisher`・`Popen`・`pkill`・`subprocess`・`create_timer` が無い（ツールと `ik_move.py` の両方） | 禁止名 |
| 実行を送る呼び出し（`execute(`）は 1 か所。同じ関数で、送信の確認 → 再検査 → 実行の順 | 呼び出しの行番号の順 |
| 移動量の上限の定数が引数にならない（`--max-tip-move` などが無い）。`--yes`・scaling の引数が無い | `add_argument` の列挙 |
| 引数の検査がノード・クライアントを作る前にある | 既存と同じ |

動作の試験（偽の入出力。`test_ik_move_loop.py`）: 再検査の失敗（q_meas が動いた、desired が変わった、他のゴール、全点の床）のそれぞれで `execute` が呼ばれないこと。

**守りを 1 つずつ外して落ちることの確認**: リポジトリの中で外すと作業ツリーを変える操作になる。**スクラッチにパッケージとツールのコピーを作り、そこで外す**。リポジトリ本体は触らない。対象（実装後に、7.15 の 6 と同じ形で表にして記録）:

- 開始の床（`START_FLOOR_MEASURED`）
- 目標の床（`GOAL_FLOOR`）
- 端の余裕（JOINT_RANGE の 0.02）
- SINGULAR の余裕
- 枝（`BRANCH_SWITCH`。`--goal` と `--joints-deg` の両方）
- 度の範囲（±180）
- 移動量の上限（先端 0.15 m。各関節 1.0 rad）
- 最初の点の検査 2 つ（desired 0.06、測定 0.01）
- 最後の点
- 最大の速さ
- 計画の長さの上限
- 全点の床
- 全点の範囲
- 排他
- 再検査（q_meas、desired、他のゴール、軌道の再検査）
- 2 段の確認
- plan_only

## 10. 置き場所・試験の件数

- 純粋な計算と手順（解く・検査・到達判定・CSV・オプション）は `so_arm101_ik/ik_move.py`。lint と pytest の対象。
- ROS の入出力は `tools/ik_move_run.py`。**lint の対象外**。静的試験で守りを保証し、手で `flake8` を当てる（行は 99 文字以内。7.15 の 3 と同じ）。

件数の見込み [推測。実装後に実数を記録]:

| 区分 | 現在 | 見込み | 内訳 |
|---|---|---|---|
| pytest（ik） | 212 | +56（46〜66）= 268 | 純粋 約 28（解く・枝・目標の検査・開始の検査・移動量・度の範囲・全点・到達・オプション）、動作 約 19、静的 約 9 |
| lint（ik） | 41 | +8 = 49 | 新しい `.py` が 4 つ（`ik_move.py` と試験 3 つ）× copyright・pep257 |
| CTest の要約（ik） | 18 | +3 = 21 | `ament_add_pytest_test` が 3 つ増える |
| ik の合計 | 271 | 約 338（328〜348） | |
| colcon test 全体 | 498 | 約 565（555〜575） | bringup 47、calibration 73、description 2、hardware 105、moveit_config 0 は変わらない |

## 11. 実機の確認（最小の段。F を反映）

手順書は新しいファイル `docs/large_move_real_check.md`。共通の約束は `diff_mode_real_check.md` の「共通の約束」を参照する（書き写さない）。実機のコマンドはユーザーが実行する。

T の値（事実。`diff_mode_real_check.md:24` と 7.12.4）: T の目標の関節値は pan 0、lift −22、elbow 30、wrist_flex 83、roll 0 度（先端 (0.17666, −0.00001, 0.05877)、pitch −91.0°）。T の実測（W1）は 0.0031、−0.3813、0.5501、1.4454、−0.0069 rad（先端 (0.17239, −0.00037, 0.05523)、pitch −92.5°）。

### 11.1 目標（F。具体的な座標）

> **2026-11-15 の決定:** M3 は実機の T（V2b の実測）から解けない可能性があるため、**M3' = (0.230, 0.060, 0.060)** に変更した。M2・M3' には `--pitch-deg -91.0 --roll-deg 0` を付ける（16.8）。以下は計画の当時の記述（履歴）。

| 段 | 目標 | 先端の移動 | 使う理由 |
|---|---|---|---|
| M2 | `--goal 0.200 0.000 0.060` | 約 23〜32 mm（T の基準しだい。下の表）。**当初の「約 28 mm」は W1 の T の実測 (172.39, −0.37, 55.23) からの値で、2026-11-15 の V2b の T ではない** | 小さな一歩。IK・枝・全点の最小 z・到達の表を実機で見る |
| M2 の戻し | `--joints-deg 0 -22 30 83 0` | M2 の終点から約 23 mm（T の目標まで。dx 23.3、dz 1.2。手計算） | 戻しにもこの道具を使い、V2b・V8 の RViz と同じ整数度の目標で、項目 5 の終点のずれを実機で取る |
| M3 | `--goal 0.240 0.060 0.065` | 約 88〜94 mm（T の基準しだい。下の表）。**当初の「約 91 mm」は W1 の T の実測からの値** | 約 90 mm の動き。移動量の上限 0.15 m の内側。radial 約 0.247 m |
| M3 の戻し | `--joints-deg 0 -22 30 83 0` | M3 の終点から約 88 mm（T の目標まで。dx 63.3、dy 60.0、dz 6.2。手計算） | 同上 |

**T の基準ごとの先端の移動（手計算。訂正の記録）**: 先端は base_link の座標 [mm]。M2 の目標は (200, 0, 60)、M3 の目標は (240, 60, 65)。距離は、3 成分の差の 2 乗和の平方根。

| T の基準 | 先端 [mm] | 出どころ | M2 まで | M3 まで |
|---|---|---|---|---|
| T の目標の関節値の FK | (176.66, −0.01, 58.77) | 7.12.4 の表 | 23.4 | 87.5 |
| V2b の desired の FK | (174.91, −0.95, 57.87) | ユーザーの指摘（2026-11-15） | 25.2 | 89.5 |
| W1 の T の実測の FK | (172.39, −0.37, 55.23) | 7.12.4 の表（2026-11-13）。**当初の 28 mm・91 mm はこの値から** | 28.0 | 91.2 |
| V2b の実測の FK | (168.62, 0.05, 54.13) | ユーザーの指摘（2026-11-15）。2026-11-15 の実機の T | 31.9 | 93.8 |

- 当初の計算（28 mm・91 mm）は、W1 の T の実測に対しては合っていた。基準を書かなかったのが誤りで、ユーザーが挙げた 3 つの基準には 28 mm になるものがない（上の表の 23.4・25.2・31.9）。
- **実機の M2・M3 の出発点は V2b の実測に近い**（z が約 1 mm 低く、x が約 4 mm 手前）と見込む [推測]。実際の移動は、実行の直前の FK(q_meas) で決まる（道具が表示する）。どの基準でも、移動量の上限 0.15 m の内側で、M2 は約 23〜32 mm、M3 は約 88〜94 mm。
- 戻しの距離（M2・M3 の終点から T の目標まで）は、上の表の「T の目標の関節値の FK」の行と同じ値（23.4・87.5 mm）。

- M3 を x = 0.25 でなく 0.24 にした理由: [推測] T の付近は、pitch を保つと wrist_flex の範囲が先に当たる（7.12.4。z = 58.8 mm で x = 0.270 m、z = 80 mm で x = 0.2355 m が範囲の外。端の余裕 0.02 rad でさらに手前）。z = 65 mm・radial 0.257 m は余裕が小さいと見込み、0.247 m にした。この値で解けるかは、次の「モックの事前確認」で確かめる。
- pan は y = 0.06 m でも約 14°（範囲 ±110° の内側）。[計算]
- M2・M3 の後は、**この道具の `--joints-deg 0 -22 30 83 0` で T に戻す**。拒否されたら RViz（V2b と同じ手順）。
- 最後の出発点への戻しは、**RViz の V8**。出発点は床の下（z 0.033）なので、この道具では戻せない（3.2）。

### 11.2 段

| 段 | 内容 | 確認すること |
|---|---|---|
| M0 | 読み取り専用の起動（V1 と同じ）。出発点で `python3 tools/ik_move_run.py --dry-run --goal 0.200 0.000 0.060` | **`START_FLOOR_MEASURED` で拒否**（終了コード 3）。何も送らない・クライアントも CSV もない。`--dry-run` は q_desired が無くても動く（読み取り専用では desired が来ないため `--assume-desired-is-measured` は不要。desired があれば表示する）。見込み: `MOVE_JOINT_TOO_FAR` も出る（3.2） |
| M1 | 書き込みの起動（V2a と同じ）→ V2b の手順（RViz）で T → T で `--dry-run --goal 0.200 0.000 0.060` | 枝・q_goal・各検査の余裕・FK の差（0.001 mm 以内）。何も送らない |
| M2 | T で `--goal 0.200 0.000 0.060` を実行 → 2 段の確認 → 結果。続けて T へ戻す | 計画の要約（最小 z、長さ、最大の速さ）、整定、到達の表（mm・関節ごと）、`REACHED`/`SHORT`。戻しの終点が整数度の目標からどれだけずれるか（項目 12 の 3 つの差） |
| M3 | T で `--goal 0.240 0.060 0.065` を実行。続けて T へ戻す | 長さ（約 5〜6 秒の見込み）、最小 z、到達の誤差、`SHORT` が出るか |
| 戻し | RViz で出発点へ（V8） | 出発点の実測との差。停止後の確認の 2 行、`ps` と `ros2 node list --no-daemon` が空 |

- 各段で、bag（`/joint_states`・`/arm_controller/controller_state`）と CSV を残す（共通の約束 7）。
- 中止の基準は `diff_mode_real_check.md` の「中止の基準」に、`START_FLOOR_MEASURED` 以外の拒否・計画の長さの超過・`SHORT` が続く場合を足す。

### 11.3 モックの事前確認（F。実装後、実機の前に必ず行う）

モックで、M2・M3・戻しの目標が解け、検査を通り、全点の最小 z が十分あるかを、**実機の前に**確かめる。ユーザーが実機・launch を使っていないと答えた後に、許可を得て Claude が実行する（`ROS_DOMAIN_ID=87`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、前景で `exec`、PID を控える、SigIgn を確認、止めるのは `kill -INT` だけ、停止後に `ps` と `ros2 node list --no-daemon`）。

手順:
1. モックの腕を、次の 2 つの出発点で試す（T の実測と T の目標は 5 mm ほど違うため）。
   - T の実測: 0.0031、−0.3813、0.5501、1.4454、−0.0069
   - T の目標: 0、−0.3840、0.5236、1.4486、0
   - モックの初期姿勢が床の上かは未確認。床の下なら、スクラッチの補助（リポジトリの外。コミットしない）で上の姿勢へ動かす（7.15 の確認と同じ形）。
2. それぞれの出発点で、M2・M3・戻しの目標を `--dry-run` にかけ、次を表にする: 解けたか（候補の数）、枝の番号、`FK(q_goal)` と目標の差（1e-6 m 以内）、q_goal、abs(D)、各関節の端までの余裕の最小、先端の移動、関節の最大の移動。
3. 実行（モックなので動かしてよい）し、次を表にする: 計画の長さ、最大の速さ、全点の最小 z、最初の点・最後の点の差、整定後の `q_meas − q_goal`。
4. **実機に進む基準（推測の値）**: すべての検査を通り、全点の最小 z が 0.048 m 以上（道具の検査は 0.045。実機の前は 3 mm 厳しく見る）。外れたら、目標を直して（この計画に記録して）からやり直す。
5. 結果を `large_move_plan.md` の結果の節と `inventory.md` に記録する。

項目 12 の「終点のずれ」の確認（同じモックで、手順 3 のあとに）は 12 章。

## 12. 計画の終点のずれ（V2b・V8。E を含む）

### 12.1 事実

- reference の終点は、RViz の整数度の目標から最大 0.0082 rad ずれた（pan +0.00705、wrist_flex −0.00815 など。V2b・V8 の両方。`analytic_diff_plan.md:1553`）。最後の 8 点は 0.004 から 0 へ滑らかに収束していた。入力は RViz の Joints タブのスライダー。原因は未確認。
- 道具の `moveit` 経路をモックで動かしたときは、最後の点と q_cmd の差が 0.00076〜0.00095 rad（許容 0.001 の内側。縁に近い値。7.15 の 12）。
- 道具の許容は `--moveit-tolerance`（既定 0.001、上限 0.01。`sr:76-77`）で、`JointConstraint` の `tolerance_above/below` に入る（`run:209-210`）。
- リポジトリには、許容に関わる設定がない（`moveit_controllers.yaml` に許容の項目なし。`controllers.yaml` の arm_controller に `constraints:` なし。`allowed_start_tolerance` の設定なし）。
- **`allowed_start_tolerance` は 0.01（事実）**: 実行時に `Validating trajectory with allowed_start_tolerance 0.01` が出る（`~/so_arm101_check/servo_W1.log:246` ほか。git 管理外）。
- 計画のパイプラインの時間最適化は、`resample_dt` 0.1、`path_tolerance` 0.1（既定値。`V2_launch.log:85,101-102`）。終点には効かない [推測。TOTG は最後の点を通る]。

### 12.2 E: MoveGroupInterface の `goal_joint_tolerance` の既定（1e-4 rad という話）

読むだけで確かめた範囲:
- **事実**: `/opt/ros/humble/include/moveit/move_group_interface/move_group_interface.h:256` に `getGoalJointTolerance()`、:274-276 に `setGoalJointTolerance(double)`。説明は「joint value target に使う各関節の許容」とあるだけで、**既定値は書かれていない**。
- **事実**: `.../moveit_cpp/planning_component.h:192` に `// double goal_joint_tolerance_;`（コメントアウトされていて値がない）。
- **事実**: RViz の MotionPlanning プラグインのライブラリ（`libmoveit_motion_planning_rviz_plugin.so.2.5.10`、`..._core.so.2.5.10`）の文字列に `tolerance` を含むものは無かった（大文字小文字を区別せずに検索）。プラグインに許容の入力欄がある形跡は見つからなかった。
- **事実**: `/opt/ros/humble/share` の下の設定・ドキュメントに、`goal_joint_tolerance` の値の記述は無かった。
- **未確認**: 1e-4 rad という既定値。数値は MoveIt のコンパイル済みコードの中にあり、インストール済みのヘッダー・ドキュメントからは確かめられない。逆アセンブルでの確認は試みていない（読むだけの範囲を超えると判断した）。

### 12.3 推測

- H1: OMPL は、`JointConstraint` の許容の中の任意の点を終点にする。ずれは許容以下になる（モックの 0.00076〜0.00095 と整合）。**ただし RViz の許容が 1e-4 だとすれば、H1 だけでは RViz の 0.0082 を説明できない**（許容より 80 倍大きい）。RViz の許容の値が未確認なので、H1 の当否は決められない。
- H2: スライダーの値の丸めなど、RViz 側の入力の問題。
- H3: JTC の reference の終点が、計画の最後の点と違う。

### 12.4 実装後にモックで確かめる手順（11.3 の後）

1. `ros2 param get /move_group trajectory_execution.allowed_start_tolerance` が 0.01 か（`--no-daemon` が使えるか確かめ、使えなければ報告して指示を待つ）。実行時の加速度の値（`robot_description_planning.joint_limits.elbow_flex.max_acceleration`）も同じ方法で見て、8.1 の 0.1 rad/s²（1.0 × 0.1）と照合する。
2. T の目標（整数度。`--joints-deg 0 -22 30 83 0`）を、`--moveit-tolerance` を 0.0005 / 0.001 / 0.005 / 0.01 にして、各 5 回、別の出発点（11.3 の M2 の目標など）から送る。
3. CSV に 3 つの差を出す: 計画の最後の点 − 目標（`last_vs_cmd`）、止まったあとの reference − 目標、止まったあとの q_meas − 目標。
4. 読み方:
   - 差が許容に比例すれば H1（許容が効いている）。RViz の許容を、別の方法で調べる課題が残る。
   - 許容によらず 0.008 前後なら H2 か H3。
   - 計画の最後の点と reference の終点が違えば H3。
5. 実機の M2・M3 の戻し（整数度の目標）で、同じ 3 つの差を取る。実機で手前止まりが加わった値は、モックと分けて記録する。

## 13. 変更するもの・新しく作るもの

moveit_config・launch・SRDF・`package.xml`・MoveIt の設定: **変更なし**（`allowed_start_tolerance` も変えない。7.14.9 の 18 の方針）。

| 区分 | ファイル |
|---|---|
| 新規 | `src/so_arm101_ik/so_arm101_ik/ik_move.py` |
| 新規 | `src/so_arm101_ik/test/test_ik_move.py` |
| 新規 | `src/so_arm101_ik/test/test_ik_move_loop.py` |
| 新規 | `src/so_arm101_ik/test/test_ik_move_run_static.py` |
| 新規 | `tools/ik_move_run.py` |
| 新規 | `docs/large_move_real_check.md`（手順書） |
| 変更 | `src/so_arm101_ik/CMakeLists.txt`（試験名 3 つの追加だけ。実装の前に承認） |
| 変更 | `docs/inventory.md`（結果の記録） |
| 変更しない | `diff_mode.py`・`step_runner.py`・`analytic_ik.py`・`tools/diff_mode_run.py`・MoveIt 一式・`controllers.yaml` |

実装の進め方（段ごとに承認）:
1. 承認: `CMakeLists.txt` の変更と、ファイル一覧。
2. `ik_move.py` と純粋な試験 → `colcon build --packages-select so_arm101_ik` → 試験。
3. 動作の試験・静的試験・`tools/ik_move_run.py`。
4. `colcon test`（全体。`/opt/ros/humble/setup.bash` と `install/setup.bash` を source）。失敗は先に報告。コミット（コード・試験・CMake）。
5. 守りを外す確認（スクラッチ。9 章）。
6. モックの事前確認（11.3）と 12.4。ユーザーが実機を使っていないと答えてから、許可を得て。
7. 手順書 `docs/large_move_real_check.md` と結果の記録。コミット（docs）。
8. 実機の M0〜M3 はユーザー。

## 14. 直し A〜F の対応

| 直し | 反映した所 |
|---|---|
| A 決定 6 の拒否を出す検査 | 3.2（`START_FLOOR_MEASURED`。開始時に FK(q_meas) の z を検査。M0 の見込み）、3.5（全点の最初の点） |
| B 加速度の出どころ | 8.1（`joint_limits.yaml:14-15` ほか、スケーリング 0.1、V2b の 6.87 秒との整合） |
| C 再検査の許容 | 5 章（q_meas 0.003 = `STOP_TOL`、desired 1e-4 = `DESIRED_TOL_DIRECT`、どちらも既存の値の再利用） |
| D `--joints-deg` の枝と度の範囲 | 2 章（±180 度）、3.6（`BRANCH_SWITCH` を `--joints-deg` にも適用） |
| E `goal_joint_tolerance` の既定 | 12.2（読んだ範囲では確かめられない。未確認のまま） |
| F M3 の具体的な目標、モックの事前確認、戻し方 | 11.1（座標）、11.3（モックの事前確認）、11.2（M2・M3 の戻しは `--joints-deg`、最後は RViz の V8） |

## 15. 未確認・限界

- 他方の枝が範囲内に残る頻度（計算していない）。
- `ExecuteTrajectory` の Ctrl-C（モックでも未確認）。
- 手前止まりが 0.032〜0.040 rad の間の挙動（7.16.6）。
- RViz の目標の許容の既定値（12.2）。
- M2・M3 の目標が解けるか、全点の最小 z（実装後のモックで確かめる。11.3）。手計算の移動量だけを置いた。
- 加速度 0.1 rad/s² は、設定値と V2b の所要時間からの見積り。実行時の値は未確認（12.4 の 1 で確認）。
- 度とラジアンの取り違えのうち、±180 の内側のもの（`--roll-deg 3.14` など）は、入口では見つからない（2 章）。

## 16. 実装と結果の記録（2026-11-15）

事実（コード・試験・モックの出力で確かめたもの）と **[推測]** を分ける。CSV・ログの全文は写さない（パスと要点だけ。16.10）。実機は使われていない（モックだけ）。

### 16.1 コミット

| コミット | 内容 |
|---|---|
| `9464d65` | この計画（`large_move_plan.md`）。`analytic_diff_plan.md` に 1 行のリンク、`inventory.md` に V7 のスクリプトの場所 |
| `72ffa2e` | 段 1: `ik_move.py`、試験 3 ファイル、`tools/ik_move_run.py`、`CMakeLists.txt`（試験名 3 つ） |
| `a8c6ca0` | 段 A: Ctrl-C の行、`ALREADY_AT_GOAL`（`ik_move.py`、`tools/ik_move_run.py`、試験 3 ファイル） |

### 16.2 試験の件数（実数と見込みの差）

| 時点 | pytest（ik） | lint（ik） | CTest の要約（ik） | ik の合計 | 全体 |
|---|---|---|---|---|---|
| 計画の見込み（10 章） | +56（46〜66）= 268 | 49 | 21 | 約 338（328〜348） | 約 565（555〜575） |
| 段 1 の実数（`72ffa2e`） | **+76** = 288 | 49 | 21 | **358** | **585** |
| 段 A の見込み（段 1 の実数から） | +7（288 → 295） | 49 | 21 | 約 365（363〜367） | 約 592（590〜594） |
| 段 A の実数（`a8c6ca0`） | **+6（288 → 294）**。新しい 3 ファイルの合計は 82 件（40 + 32 + 10） | 49 | 21 | **364** | **591** |

- 段 1 の差（pytest +20、見込みの範囲の上限 +66 を +10 超え）の理由: 計画の内訳（純粋 約 28・動作 約 19・静的 約 9）に対して、実際は純粋 40・動作 27・静的 9 だった。守りごとに試験を分けたこと（options 6、開始 3、各理由、全点 4、計画 5、再検査 3、到達 3 など）、`parametrize` の展開（目標 2、確認 2×2）、守りを外す確認で見つかった穴を塞ぐ試験（`NOT_SETTLED`、`NO_BRANCH`）を足したことによる。lint（+8）と CTest の要約（+3）は見込みどおり。
- 段 A の差（+6、見込み +7）: `ik_move` の列の確認を、新しい試験にせず既存の試験に足したため。
- 全体のうち他のパッケージは変わらない（bringup 47、calibration 73、description 2、hardware 105、moveit_config 0）。numpy の 2 版（`python3 -s` 1.21.5 と `python3` 1.26.4）の両方で、新しい試験 82 件が通る。手動の flake8（99 文字）は `tools/ik_move_run.py` で問題なし。

### 16.3 計画との食い違い（実装側が計画を変えず、報告したもの）

1. **`DiffMode.start(q_goal, q_goal)` は `NO_BRANCH` の検出だけに使った**（計画の 3.3 は入口のまとめとして使う案）。床・radial・abs(D)・端の余裕・厳密な範囲は、自前の検査（`goal_checks`、`ik_move.py`）で、DiffMode の `DiffModeConfig` の値を import して行う。理由: 理由コードをこの道具で統一するため、守りを外す確認で 1 つずつ外せるようにするため。「通った目標は DiffMode の開始点になれる」ことは、ランダムな 300 点の試験（`test_a_goal_that_passes_is_a_valid_start_of_the_differential_mode`）で確かめる。
2. **0.989 と 0.051 の出どころ**: DiffMode の既定（入口 0.99・0.05）より厳しいが、**DiffMode が動いている間の目標の検査（`_excess`）の内側の限界と同じ**（`diff_mode.py:504-505`: `max_abs_d − margin_abs_d`、`min_radial + margin_radial`）。これで、着いた点が DiffMode の境界（`AT_LIMIT`）に最初から乗らない。余裕 0.001（abs(D)）と 0.001 m（radial）の値は、`analytic_diff_plan.md` 7.12.4 の表で**推測の値**（丸め 1e-9 より十分大きく、感度を損なわない小ささ。実測の根拠はない）。
3. **計画に書いていなかった動作（4 点）**: ① 拒否・停止でも CSV に 1 行書く（`--dry-run` を除く）。② `--dry-run` は q_desired が無くても動く（読み取り専用の起動のため。計画の 11.2 の M0 には書いてあった）。③ 計画の直後に、q_meas が `STOP_TOL`（0.003 rad）以内で動いていないかを確かめる（`ARM_MOVED`）。④ `START_FLOOR_MEASURED` を、送る直前の再検査でも見る。
4. **定数の import**（`ik_move.py` の 3 行）: `FIRST_DESIRED_TOL = _CONFIG.max_windup`、`BAND_MM = _CONFIG.deadband_enter * 1000.0`、`LIMIT_ROUNDING = dm.LIMIT_ROUNDING`（書き写しだったものを import に直した。ユーザー承認）。import できなかったもの（元の定数が無い）: `step_runner.py:597` の `wait(0.1)` と同じ値、`ik_move.py:325` の `ceil` の丸め 1e-9。新しく決めた値（元の定数なし）: `MAX_TIP_MOVE` 0.15、`MAX_JOINT_MOVE` 1.0、`MAX_ANGLE_DEG` 180、`MAX_PLAN_DURATION` 20、`PATH_STEP` 0.01、`REACHED_MM` 10。
5. **静的試験の直し（`FIRST_DESIRED_TOL`）**: 4 の import で、`test_the_limits_of_the_move_are_constants_that_the_goal_check_uses` が（右辺がリテラルでなくなって）落ちた。ユーザーの承認の条件どおり、**`FIRST_DESIRED_TOL` だけ**、右辺が `_CONFIG.max_windup`（`ast.Attribute`、値が `Name` の `_CONFIG`、属性が `max_windup`）であることを認め、他の定数（`MAX_TIP_MOVE` など）は今までどおりリテラルに限った。守りを外す確認（この試験について 1 回）: (a) 値を `check_plan` の引数から受ける、(b) `_CONFIG.max_steps`、追加の (c) リテラル 0.06、(d) `MAX_TIP_MOVE = _CONFIG.max_distance`（他の定数に属性を許さない）は、**すべて同じ試験が落ちた**。
   `test_the_limits_are_values_of_the_differential_mode_and_of_the_plan`（値が DiffModeConfig と等しいこと）はそのまま残した。

### 16.4 守りを外す確認（スクラッチのコピー。リポジトリの作業ツリーは変えていない）

段 1（47 項目）。最初は 44 項目で試験が落ち、生き残りが 3 件だった。直して再実行した結果は、47 項目すべてで落ちた。

| 外した守り | 落ちた試験（代表） |
|---|---|
| 開始の床（`START_FLOOR_MEASURED`） | `test_the_start_pose_below_the_floor_is_refused_by_the_measured_floor` ほか 3 件 |
| 目標の床（`goal_checks` の検査。`solve_goal` の前検査も同時） | `test_a_target_below_the_floor_is_refused_and_the_floor_is_the_goal_floor` ほか |
| 端の余裕（0.02）、厳密な範囲 | `test_a_goal_outside_the_joint_range_is_refused_in_two_steps`、`test_a_joint_goal_near_a_limit_is_refused` |
| `SINGULAR`、`BRANCH_SWITCH`、`NO_BRANCH` | `test_a_pose_near_the_stretched_arm_is_singular`、`test_the_other_elbow_branch_is_refused_for_a_joint_goal_and_a_target` |
| 先端 0.15 m、関節 1.0 rad | `test_the_move_is_limited_for_the_tip_and_for_every_joint` ほか |
| ±180 度、pitch・roll と関節の併用、入力は 1 つだけ | `test_an_angle_in_degrees_must_be_inside_plus_minus_180` ほか |
| 定数 4 つ（0.15、1.0、0.06、0.045）を変える | `test_the_limits_are_values_of_the_differential_mode_and_of_the_plan` ほか |
| 計画: 最初の点（desired・測定）、最後の点、速さ、長さ、全点の床・範囲、検査を使わない | `test_the_first_point_is_checked_against_the_desired_and_the_measurement`、`test_a_plan_through_the_floor_is_refused_although_both_ends_are_above_it` ほか |
| 再検査 5 つ、結果の無視、計画中の `ARM_MOVED` | `test_the_arm_moved_the_desired_changed_or_another_goal_is_active` ほか |
| 確認 2 回、実行時間の上限 | `test_no_send_confirmation_no_execution`、`test_a_run_that_takes_too_long_is_not_sent` |
| 到達（常に `REACHED`）、送信失敗の無視、整定の判定 | `test_succeeded_is_not_an_arrival_when_the_arm_did_not_move`、`test_an_arm_that_does_not_stop_is_not_judged` |
| ツール: `plan_only` を False、キャンセルの追加、scaling 引数、検査の順、直接の JTC クライアント、モジュールに ROS の import、`execute` を 2 回 | 静的試験の各 1 件 |

- 最初の生き残り 3 件と直し: ① 整定の「止まった」判定を外す → `test_an_arm_that_does_not_stop_is_not_judged` を足した。② ツールに JTC 直接のクライアントを足す → 静的試験が `from … import 名前` の名前を見ていなかった。直した。③ モジュールに ROS の import を足す → ROS を source していない環境で、`rclpy` の import エラーで偶然落ちていた（試験が捕まえたものではなかった）。ROS を source して再実行し、`test_the_module_has_no_ros_and_the_tool_has_no_direct_route` が落とすことを確かめた。
- 目標の床は冗長（`solve_goal` の前検査と `goal_checks` の検査が重なる）なので、`goal_checks` 側だけ外した場合は関節入力の経路の試験が落とし、両方外した場合は 2 件が落ちる。

段 A（11 項目、Ctrl-C の行と `ALREADY_AT_GOAL`）。すべて落ちた。

| 外した守り | 落ちた試験（代表） |
|---|---|
| Ctrl-C の行を書かない | `test_ctrl_c_writes_one_row_of_what_is_known_without_reading_or_waiting` ほか 3 件 |
| `goal_cancelled` を True に、`goal_sent`・`q_meas_last` を書かない、送信確認の段階の記録 | 同じ試験、`test_ctrl_c_during_the_plan_and_during_the_execution_is_recorded` |
| 割り込みの処理で待つ・取り消す・送る | 同じ試験と `test_the_interruption_only_says_logs_and_returns`（静的） |
| 割り込みの処理で入力を読む | 静的試験が落とす。pytest 全体は `KeyboardInterrupt` の再発生で中断した（静的試験だけを流すと落ちる） |
| 「すでに目標にいる」の分岐を外す、遠い 1 点まで同じ扱い | `test_an_arm_that_is_already_at_the_goal_is_told_so_and_nothing_is_executed`、`test_a_single_point_plan_far_from_the_goal_is_still_refused_as_empty` |

### 16.5 段 A の内容（コード）

- **Ctrl-C（終了コード 130）にも CSV に 1 行**: 列は `stage`（最後に入った段階）、`goal_sent`、`goal_in_flight`、`goal_cancelled`（`_log()` の中のリテラル False。取り消さない）、`q_meas_last`（最後に読んだ関節角）、`label`（`INTERRUPTED`）、`note`。割り込みの処理は、表示・行の記録・結果の返却だけで、入力を読まず、待たず、送らず、取り消さない。止まる経路（`_Stop`）の行にも同じ列が入る。`--dry-run` では書かない。静的試験で、割り込みの処理が呼べる関数を `_say`・`_log`・`update`・`RunResult` に限り、`execute` の呼び出しが 1 か所のままであることを保証する。
- **`ALREADY_AT_GOAL`**: 計画が 1 点だけで、q_meas と目標の差が許容（`--moveit-tolerance`）以内なら、`ALREADY_AT_GOAL`（終了コード 2）で止まる。メッセージは「the arm is already at the goal (…): MoveIt planned a single point, so nothing was executed」。1 点でも目標から遠い場合は、これまでどおり `PLAN_REFUSED`（`EMPTY`）。

### 16.6 段 2: モックの確認 a〜e（2026-11-15。`mock:=true use_rviz:=false`、`ROS_DOMAIN_ID=87`、cyclonedds、前景の `exec`。PID 58318、SigIgn に SIGINT なし。停止は `kill -INT` だけ）

**モックを開始点に置く方法（モックの中だけ）**: モックの初期姿勢は折りたたみ（`initial_positions.yaml`: −1.57、1.57、0.8。先端 z = 0.0302 m）。T などへは、リポジトリの外のスクリプト（`place.sh`。`ros2 action send_goal` で JTC に 1 点の軌道を送る）で置いた。ユーザー了承済み（domain 87 のモックだけ）。開始点: **S1** = T の目標（0、−0.383972、0.523599、1.448623、0）、**S2** = V2b の実測（0、−0.38282、0.56234、1.45610、−0.00690。`V3_px2.csv` の `q_meas_before`）。

**a. 実行時の値**

| 項目 | 値 | 出どころ |
|---|---|---|
| `max_acceleration`・`max_velocity`（5 関節） | 1.0・5.0 | `ros2 param get /move_group robot_description_planning.joint_limits.<関節>.…` |
| `default_*_scaling_factor` | 0.1 | 同上（道具もリクエストで 0.1 を設定） |
| `trajectory_execution.allowed_start_tolerance` | `param get` は「Parameter not set」。実行時のログ `Validating trajectory with allowed_start_tolerance 0.01` で **0.01** | モックの launch のログ（8 回とも） |

加速度 1.0 × 0.1 = 0.1 rad/s²（8.1 の見積り）は、設定値として確認できた。

**b. 事前確認（S1・S2 から。モックは摩擦なし）**

| 開始 | 目標 | 結果 | 先端の移動 | 長さ | 最大の速さ | 全点の最小 z |
|---|---|---|---|---|---|---|
| S1 | M2 | 解ける（候補 1、枝 1） | 23.4 mm | 2.86 s | 0.140 rad/s | 0.0588 m |
| S1 | M3 | 解ける | 87.5 mm | 5.58 s | 0.275 rad/s | 0.0588 m |
| S1 | 戻し | 同じ姿勢で EMPTY（この時点では `PLAN_REFUSED`。段 A で `ALREADY_AT_GOAL`） | 0 | － | － | － |
| S2 | M2 | 解ける | 31.9 mm | 3.51 s | 0.173 rad/s | 0.0541 m |
| S2 | **M3** | **拒否: `JOINT_LIMIT`（全候補が関節範囲の外）** | － | － | － | － |
| S2 | 戻し | 解ける | 9.3 mm | 1.24 s | 0.058 rad/s | 0.0541 m |

M3 が S2 から解けない原因: `--goal` は pitch を今の姿勢のまま保ち、V2b の実測の pitch では wrist_flex が上限（95°）を超える。S2 で通る候補を dry-run で確かめた: (0.240, 0.060, 0.060) は wrist 93.3°（余裕 1.15° に近い）、**(0.230, 0.060, 0.060) は wrist 90.9°（余裕 約 4°）**。→ 決定: M3' = (0.230, 0.060, 0.060)（16.8）。

**c. 終点のずれ（`--joints-deg 0 -22 30 83 0`、M3 の終点から、各 5 回。3 つの差の最大絶対値 [rad] の最小〜最大）**

| `--moveit-tolerance` | 計画の最後の点 − 目標 | 止まった後の reference − 目標 | 止まった後の q_meas − 目標 | 先端の誤差の最大 |
|---|---|---|---|---|
| 0.0005 | 0.00041〜0.00049 | 0.00041〜0.00048 | 0.00041〜0.00048 | 0.10 mm |
| 0.001 | 0.00069〜0.00099 | 0.00070〜0.00099 | 0.00070〜0.00099 | 0.30 mm |
| 0.005 | 0.00347〜0.00484 | 0.00347〜0.00485 | 0.00347〜0.00485 | 1.46 mm |
| 0.01 | 0.00595〜0.00966 | 0.00595〜0.00966 | 0.00595〜0.00966 | 2.51 mm |

- 集計は `python3 -I -`（標準ライブラリだけ。CSV を読むだけ）。
- 事実: ずれは 20 回すべてで許容以下で、許容に比例し、許容 0.01 では最大 0.0097 rad。reference の終点は計画の最後の点と一致。→ **H1（OMPL は許容の中の点を終点にする）が成り立ち、H3（JTC の終点が最後の点と違う）は否定された**（この道具の経路で）。H2（RViz のスライダーの丸めなど）はモックでは試せない。
- 12 章の読み方: V2b・V8 の 0.0082 rad は、この道具の経路では H1 で説明できた（許容 0.01 で最大 0.0097）。**RViz の許容の値は未確認のまま**（12.2。`goal_joint_tolerance` の既定 1e-4 という話は、インストール済みのヘッダー・ドキュメントでは確かめられなかった）。**[推測]** RViz の許容がおよそ 0.008 以上だった可能性と整合する。

**d. Ctrl-C のとき進行中のゴール（`ExecuteTrajectory` の実行中に、道具の PID へ `kill -INT`）**

- 最初の 2 回は、kill が実行の終わった後に届いて無効だった（別の bash の `killer.sh` が「Send」のプロンプトの 2.5 秒後に kill する形に直して成功）。
- 3 回目: 道具は終了コード 130、「a goal was in flight; it is not cancelled and goes on to its end」。reference は kill の後も 3.09 秒間動き続け、最後の値と `q_goal` の差は最大 0.00075 rad（許容 0.001 の内側）。ログに `Controller 'arm_controller' successfully finished` と `Execution completed: SUCCEEDED` が kill の約 3.2 秒後に出た。→ **進行中のゴールは、道具が終わっても止まらず、軌道の最後まで続く**（計画 8.2、決定 11 の見込みどおり）。
- 当時は Ctrl-C のとき CSV の行が書かれなかった（段 A で直した。16.5、16.8）。

**e. M0〜M3 の通し**

| 段 | 結果 |
|---|---|
| M0（モックの初期姿勢で `--dry-run --goal 0.200 0.000 0.060`） | `START_FLOOR_MEASURED`（z = 0.0302 m）、終了コード 3 |
| M1（T で dry-run） | 全検査を通る |
| M2（S1・S2）＋戻し | 実行、`REACHED`（0.02〜0.07 mm、戻しは 0.14〜0.19 mm）。終了コード 0 |
| M3（S1）＋戻し | `REACHED`（0.09 mm、戻しは 0.17 mm） |
| M3'（S2）＋戻し | `REACHED`（0.20 mm、戻しは 0.32 mm） |

モックは摩擦も手前止まりも無い（`desired − q_meas` は 0）ので、到達の誤差は手前止まりの見積り（7.39 mm）とは比べられない。

### 16.7 気づいた点（段 2 の途中）

- 同じ姿勢の目標（S1 での戻し）は、当時は `PLAN_REFUSED`（`EMPTY`）で「すでに目標にいる」と分からなかった → 段 A で `ALREADY_AT_GOAL`。
- 実行の終わりの `move_group` の exit −11 は、これまでと同じ既知の挙動。

### 16.8 M3 → M3'、pitch・roll、段 B の確認（2026-11-15）

**決定（ユーザー、2026-11-15）**: M3 は M3' = (0.230, 0.060, 0.060)。実機の T のばらつきに目標の姿勢が左右されないよう、手順書の M2・M3' では `--pitch-deg -91.0 --roll-deg 0` を明示する。

**pitch・roll の値の出どころ（手計算）**: T の目標の関節（0、−22、30、83、0 度）の FK。`analytic_ik.py` の式 `pitch = S2·q2 + S3·q3 + S4·q4 + D4`（S2=S3=S4=−1、`D4` = 6.5055536e-06）から、pitch = −(−22+30+83)° + 0.00037° = −90.9996°（丸めて **−91.0°**）、roll = S5·q5 = **0°**。`analytic_diff_plan.md` 7.12.4 の表の「pitch −91.0°」と一致する（ユーザーの計算とも一致）。

**S1・S2 から `--dry-run` と計画の要約（実行なし。pitch・roll を明示）**

| 開始 | 目標 | 結果 | 先端の移動 | 長さ | 最大の速さ | 全点の最小 z |
|---|---|---|---|---|---|---|
| S1 | M2 | 解ける（候補 1、枝 1、abs(D) 0.038） | 23.4 mm | 2.86 s | 0.140 rad/s | 0.0588 m |
| S1 | M3' | 解ける（候補 1、枝 1、abs(D) 0.366） | 80.3 mm | 4.96 s | 0.245 rad/s | 0.0588 m |
| S2 | M2 | 解ける | 31.9 mm | 3.12 s | 0.153 rad/s | 0.0541 m |
| S2 | M3' | 解ける | 86.0 mm | 5.11 s | 0.253 rad/s | 0.0541 m |

- **q_goal は開始点によらず同じ**（pitch・roll を明示した効果）: M2 は −0.00328、−10.457、18.326、83.132、0 度。M3' は −17.428、9.235、−5.265、87.030、0 度。
- wrist_flex の端までの余裕（手計算。上限 95.0° = 1.65806 rad）: M2 は 83.13° で **0.207 rad**、M3' は 87.03° で **0.139 rad**（要求 0.02 rad の約 7 倍）。他の関節はもっと余裕がある。

**Ctrl-C の行の確認（1 回、`killer.sh` の方法）**: S2 から M3' を実行し、「Send」のプロンプトの 2.5 秒後に `kill -INT`。終了コード 130、CSV は 1 行で、`stage = execute`、`goal_sent = True`、`goal_in_flight = True`、`goal_cancelled = False`、`label = INTERRUPTED`、`q_meas_last` = S2 の姿勢。ログに実行の完了（`SUCCEEDED`）が約 5.2 秒後に出た。

**後片付け**: モックの停止は、自分で起動した PID への `kill -INT` だけ。停止後、`ps` に ROS のプロセスなし、`ros2 node list --no-daemon` は 2 つの domain とも空。ros2 daemon のプロセスは、段 B の停止後の確認では見えなかった（開始時は PID 39502 があった。こちらでは触っていない）。

### 16.9 実機の M2・M3' で見る点

**「着いた点が DiffMode の開始点になれる」は、目標についての保証であり、実機の手前止まりがあると、着いた点（測定）については保証されない。** 理由は 2 つ。

1. **測定の床の余裕**: `DiffMode.start` は `FLOOR_MEASURED`（測定の z が 0.045 m 未満）で拒否する。目標の床は 0.048 m なので、余裕は 3 mm。T の手前止まりは先端で 7.39 mm（7.16.4 の 4）。向きによっては測定の z が 0.045 を割りうる **[推測]**。→ M2・M3' の後、`FK(q_meas).z` を見て、0.045 との差を記録する。
2. **desired と測定の差の余裕**: `NOT_SETTLED` は、desired と測定の差が 0.06 rad を超えると拒否する。T の elbow の差は 0.034 rad なので、余裕は 0.026 rad。→ `reference − q_meas`（CSV の `stuck`）の最大を記録する。

加えて、道具が終わった後の desired は q_goal ではなく計画の終点で、許容 0.001 rad の中で q_goal とずれうる。abs(D)・radial の余裕 0.001 の境界に近い目標では、desired が境界を越える可能性がある **[推測]**。

**[推測] M3' は SHORT になる可能性がある。** 実機の T から M3' へは、lift +0.544、elbow −0.614（約 0.61 rad。モックの S2 から。elbow が開く向き＝自重に逆らう向き）、wrist_flex +0.069、pan −0.305 rad 動く。腕も伸びるので、手前止まりが T の 0.034 rad より大きくなりうる。10 mm（`REACHED`）を超えれば `SHORT`（終了コード 4）。手前止まりは 0.040〜0.053 rad で動く、0.032 以下では動かない、という観察（7.16.4 の 1）がある。0.032〜0.040 の間は未観察。

### 16.10 材料の場所（リポジトリの外・git 管理外。`/tmp` の下なので再起動で消えうる）

`/tmp/claude-1000/-home-hirok-ros2-so-arm101/e1755a91-8ede-4d9e-bc80-0ab14d9b0d83/scratchpad/` の下:
- `mut.sh`・`mut2.sh`・`mut3.sh`（守りを外す確認のスクリプト）、`mut_result.txt`・`mut_result2.txt`（結果の表）。
- `stage2/`: `place.sh`（モックを置く）、`tool.sh`（道具をモックに対して実行）、`killer.sh`（Ctrl-C の確認）、`mock.log`・`mock2.log`（モックの launch のログ）、`b/`・`b2/`（事前確認の CSV）、`c/`（終点のずれの CSV 40 本）、`d/`（Ctrl-C の確認の CSV・ログ・reference の記録）、`e/`（M0〜M3 の通しの CSV）。

### 16.11 未確認・限界（実装後）

- RViz の許容の既定値（`goal_joint_tolerance` の 1e-4 という話）は未確認のまま（12.2）。H1 は、この道具の経路でだけ確かめた。
- 実機の手前止まりが加わった到達の誤差、M3' が `SHORT` になるか、実機の T で M3' が解けるか。
- 他方の枝が範囲内に残る頻度（計算していない）。
- モックは摩擦も手前止まりも無いので、`REACHED` の誤差（0.02〜0.32 mm）は実機の見込みにならない。
- wrist_flex の余裕（0.207・0.139 rad）は手計算（FK の式と IK の出力から）。
