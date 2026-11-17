# P6: 解析 IK の ROS ノード化（VR からの追従。MoveIt を使わず JTC へ直送）

2026-11-17。承認済みの計画。段は P6a（計算部分と試験）、P6b（ノード・launch・モック）、P6c（実機）、P6d（簡易説明書）。値はすべて暫定のパラメータ。

## 1. 目的と範囲

- VR の右手の Joy と PoseStamped を入力にし、相対の動作（クラッチで基準を取る）で、解析 IK の関節の目標を `/arm_controller/joint_trajectory` へ 1 点ずつ送る（周期は既定 20 Hz。b0 の結果で変えてよい）。
- 大きな移動の道具（ik_move）、差分モード（DiffMode）、MoveIt とは、追従していない間は何も送らないことで取り合わない。
- VR 機器は手元にない。骨格を作ってリリースし、別の設計者が別のマシンで合わせ込む（VR の座標の合わせ込みは次の担当者）。
- ノードは so_arm101_ik パッケージに置く。bringup.launch.py（ros2_control・MoveIt・RViz）は変えず、その上に vr_follow.launch.py を追加で立ち上げる。

## 2. 事前の確認の結果（コードと設定を読んだ。出典つき）

1. **ドライバの step limit は 1 書き込み（1 制御周期）あたりの上限**。`max_step_raw_ = floor(max_joint_speed × limit_window_s / tick)`（`sts3215_system.cpp:851`）で、0.75 × 0.08 / 0.001534 = 39 tick（0.06 rad）。`update_rate` は 50 Hz なので、速さに直すと約 3 rad/s（0.75 rad/s ではない）。**0.5 rad/s の制限はノード側が担い、ドライバの上限は安全網**（異常な飛びを削る。約 6 倍の余裕）。0.5 rad/s なら JTC の 1 更新（40 ms）で 0.02 rad、20 Hz の 1 点で 0.025 rad で、0.06 rad に届かない。
2. **JTC のトピック入力**: 新しい軌道を受けた最初の更新は直前の指令をそのまま出し、進まない（servo_integration_plan.md 3.2。ソース読みで未実行の推論）。arm_controller の更新は 25 Hz（40 ms）、ノードは 20 Hz（50 ms）なので、更新の約 80% が「最初の更新」になり、腕が進まない、または不規則になるおそれがある。**b0 でモックを使って実測し、周期と time_from_start を決める**。update_rate の変更（controllers.yaml。ドライバの試験にも関わる）が要るとわかったら、変更せずに止めて報告する。周期と time_from_start の変更で済むなら、その値をここに記録して進める。
3. **open_loop_control: true**: JTC は測定を見ない。止まった関節は、指令が測定より 0.04〜0.05 rad 先に行く。出発点は測定ではなく controller_state の reference（指令）にする（DiffMode と同じ原則）。追従中に腕が何かに当たって指令だけが進み続けるのを、C3（BLOCKED）で防ぐ。
4. **JTC に排他はない**（トピックもアクションも、最後に来たものを採用する）。クラッチを入れるときと追従中に、`/arm_controller/follow_joint_trajectory/_action/status`、`/move_action/_action/status`、`/execute_trajectory/_action/status` を見る（ik_move_run.py の STATUS_TOPICS と同じ）。動いている goal があれば、追従を始めず、追従中なら BUSY で止まる。
5. **グリッパーは action だけ**（GripperActionController。`/gripper_controller/gripper_cmd`、control_msgs/GripperCommand）。トピックはない。ノードが ActionClient を持ち、goal は結果を待たない。新しい goal が前の goal を置き換える。max_effort は 0.0。humble の allow_stalling は既定 false（stall_timeout 1.0 s）。物をつかんで止まったときの goal の扱いはソース未確認なので、モックと実機で確かめる。
6. **グリッパーの量**: 校正は linear_percent（min 1454 から max 2944 が 0〜100%、関節の値は 0〜1.745 rad）。ドライバは範囲外を黙ってクランプする（許容 50 tick）。既定は closed_rad 0.05、open_rad 1.0（暫定として可）。どちらが閉かの確認は P6c-2。ノードは [0.0, 1.745] の外を拒否する。
7. **elbow の上限 +91°** は `ik.ik(..., q_max=…)` の引数で上書きする（`elbow_max_deg`）。ik のコードは変えない。URDF の Q_MAX は elbow 1.69 rad（96.8°）。
8. so_arm101_ik は ROS なしの純 Python のパッケージで、実行ファイルも launch もない。ノードの実行ファイルと launch のインストールのため、CMakeLists.txt と package.xml を変更する（P6b で行う。承認済み）。

## 3. 設計

### 3.1 クラッチ

- 既定は hold 式（押している間だけ追従）。離す、または Joy が 1 s（C4）途切れると、すぐ IDLE になり、何も送らない。理由: ボタンの押し損ね、通信の切れ、VR の追跡の外れが、そのまま停止になる。
- `clutch_mode: toggle` も残す（押すたびに切替。Joy の途切れでは止まる）。
- 開始は立ち上がり（離している状態から押した変化）だけ。起動時にすでに押されているボタンは、押し直すまで開始しない。
- 押し直すたびに基準を取り直す（ラチェット操作ができる）。

### 3.2 空間（重要）

**ずれの判定（50 mm・30°）、再開の判定（20 mm・10°）、遅れ（C2）は、どれも倍率をかけた後の腕の目標の空間（base_link）で行う。** VR の手の空間ではない。式:

- p_目標 = p_腕の基準 + 倍率 × R_fix (p_入力 − p_入力の基準)
- R_目標 = (R_fix R_入力 R_入力の基準ᵀ R_fixᵀ) R_腕の基準（姿勢は倍率 1 の相対の回転。base_link の軸まわりの差）

R_fix は `input_rotation_rpy_deg`（既定 0 0 0。VR の軸を base_link に合わせる回転）。frame が base_link でない PoseStamped は TF で base_link に変換する（P6b）。

### 3.3 姿勢の投影

5 関節の腕は、pan が yaw を決めるため、pitch と roll だけを作れる。`project_orientation` は tool 軸を pan 平面へ射影して pitch を求め、その軸まわりの回転から roll を読む（作れる姿勢なら厳密）。捨てた角度は状態の `proj_deg` に出す。回転行列と (pitch, roll) の対応は、URDF の関節の origin から手で導き、`tip_rotation(q)` にした。**試験で URDF の鎖（UrdfChain）と 1e-9 で一致を確認した**（P6a）。tool 軸が pan 平面に垂直（pitch が決まらない）なら ORIENT で止まる。

### 3.4 IK と速さ

- `ik.ik(q_max=elbow を +91°)` の解のうち、直前の解に関節距離が近い方（枝）を取る。theta と roll は互いに依存する（TCP の横のずれ）ので 4 回反復する。
- 速さは全関節を同じ比率で縮める方式（最大の関節が 0.5 rad/s になるように 1 点の変化量を縮める。関節空間の方向が保たれる）。dt は実測の経過時間（上限は周期の 2 倍）。

### 3.5 状態と止まる条件

追従していない間は JTC に何も送らない。止まったあと、腕は最後に送った点で保持される。

- IDLE: 何も送らない。理由 START_WAIT・RELEASED・JOY_TIMEOUT・REFUSED（WINDUP・FLOOR・RANGE・POSE・JOINT_STATE・CONTROLLER_STATE・BUSY）。
- TRACKING: 毎周期 1 点。理由 OK・SPEED_LIMITED。
- HOLD_JUMP: **入力のメッセージごとに**、目標（倍率後・base_link）が 1 つ前のメッセージの目標から 50 mm または 30° 以上（1 周期に複数のメッセージが入っても、飛びと戻りを見逃さない。周期ごとの比較だと、10 Hz・倍率 1 で 0.5 m/s の手が飛びになるため、b1 で直した）。再開は、目標が止まった（最後に受け付けた）目標から 20 mm かつ 10° 以内（基準は保ったまま）。
- HOLD_RANGE: UNREACHABLE・JOINT_RANGE・ELBOW_LIMIT・FLOOR（z < 0.045 m）・ORIENT・IK_JUMP（C1）。エラーのログも出す。再開は HOLD_JUMP と同じ。
- HOLD_LAG（C2）: 目標が指令の先端から 0.15 m 以上先。受け付けた目標は手に追従し、遅れが半分（0.075 m）以下に戻ると再開。
- STALE: pose または joint_states が 0.5 s 途切れた。再開は HOLD_JUMP と同じ（遠くへ戻ったら、クラッチの押し直し）。
- BUSY: ほかの goal が動いている（クラッチの押し直しが必要）。
- BLOCKED（C3）: 測定と指令の差が 0.2 rad 超（クラッチの押し直しが必要）。

追加の検査（ユーザーが承認。どれもパラメータで無効にできる。0 で無効）: C1 `ik_jump` 0.35 rad（**入力のメッセージごと**: 各メッセージの目標の IK の解を 1 つ前のメッセージの解と比べ、0.35 rad 以上なら IK_JUMP で止まる。解析 IK なので、1 周期の全メッセージを解く（上限 100）。再開の周期では、解が最後に受け付けた解から 0.35 rad 以上離れていないことも見る）、C2 `max_lag` 0.15 m、C3 `max_tracking_error` 0.2 rad（C2・C3 は緩めの暫定）、C4 `joy_timeout_s` 1.0 s。

クラッチの前の検査（IDLE → 追従）: pose・joint_states・controller_state が新しい、動いている goal がない、指令と測定の差が `clutch_windup`（パラメータ。既定 0.06 rad。elbow で約 0.034 rad、lift で約 0.027 rad の追従の残りがあるため、今の値で余裕がある）以内、測定の先端 z が 0.045 m 以上、指令が関節の範囲（elbow は +91°）の中。

倍率は実行中に変えられる（`ros2 param set`）。変えた瞬間に基準を取り直し、目標は飛ばない。範囲は (0, 3.0]。

### 3.6 グリッパー

- ボタン（OPEN・CLOSE）の立ち上がりだけを見る。OPEN で量 0.0、CLOSE で量 1.0。同時押しは無視する。
- 関節の値 = open_rad + 量 × (closed_rad − open_rad)（量 0 全開、1 全閉）。
- 起動直後や指令がない間は何も送らない。腕の止まる条件（HOLD・STALE など）でも、グリッパーは動かさない。
- クラッチとは独立（`gripper_requires_clutch` が false の既定）。Joy が途切れたあとに戻ったとき、押されたままのボタンは立ち上がりとみなさない。
- 制限: 1 周期（50 ms）より短い押しは取りこぼす。

### 3.7 トピック（launch で remap できる）

入力: `/vr/right/joy`（sensor_msgs/Joy）、`/vr/right/pose`（geometry_msgs/PoseStamped）。
出力: `/arm_controller/joint_trajectory`（5 関節、位置のみ、1 点、stamp 0）、`/vr_follow/target_pose`（PoseStamped、base_link。倍率と投影の後の目標）、`/vr_follow/gripper`（std_msgs/Float64。0 全開〜1 全閉。ボタンの変化があったときだけ。transient_local）、`/vr_follow/status`（std_msgs/String。`STATE REASON key=value …`。20 Hz の出力に加えて、状態が変わったときにも出す。transient_local）。
action: `/gripper_controller/gripper_cmd`。
読み取り: `/joint_states`、`/arm_controller/controller_state`、各 `_action/status`。

## 4. 段

### P6a: 計算部分と試験（ROS なし）【完了。コミット待ち】

- `so_arm101_ik/follow_calc.py`: 回転（`tip_rotation`、`project_orientation`、四元数）、相対の目標、ずれ・再開の判定、IK（`solve_target`）、速さの制限（`limit_step`）、開閉量の変換、設定（`FollowConfig`。範囲の検査つき）。
- `so_arm101_ik/follow_machine.py`: 状態機械（`Follower.update`）。時計・ROS・I/O なし。
- 試験: `test_follow_calc.py` 88 件、`test_follow_machine.py` 65 件。so_arm101_ik の colcon test は 364 → 527 件、失敗 0。
- CMakeLists.txt: 試験 2 本の登録だけ（インストールの変更は P6b）。

### P6b: ノードと launch とモック

- b0: JTC の応答の実測（`examples/jtc_probe.py`。モック、ROS_DOMAIN_ID=87、rmw_cyclonedds_cpp）。周期 {20, 12.5, 10} Hz × time_from_start {0.05, 0.10, 0.12} s。結果は `~/so_arm101_check/p6/`。update_rate の変更が要るなら、止めて報告する。
- b1: ノード `vr_follow_node.py`（薄い ROS の殻。単一スレッド）。b2: `launch/vr_follow.launch.py` と `config/vr_follow.yaml`（ノード 1 つ）。b3: `examples/vr_fake_input.py`（line・jump・stale・range・rotation・gripper）。
- CMakeLists.txt（`install(PROGRAMS …)`、`install(DIRECTORY launch config …)`）、package.xml（rclpy・sensor_msgs・geometry_msgs・std_msgs・trajectory_msgs・control_msgs・action_msgs・tf2_ros_py・tf2_geometry_msgs・launch・launch_ros）。
- モックの確認の記録は `docs/p6_mock_check.md`。

### P6c: 実機（ユーザーが動かす。手順書 docs/p6_real_check.md）

- 実機を使い始める前に、ユーザーが知らせる。それ以降は、モックの起動とビルドをしない。
- c0: `read_only:=true` で偽入力を流す。サーボには書かないので、**指令だけ進み、測定は動かない**。そのため C3（BLOCKED）が出るのが正常。**合格の条件: TRACKING → 指令が 0.2 rad 進んだところで BLOCKED。** 次に C3 を無効（`max_tracking_error:=0`）にした状態で、遷移（TRACKING → HOLD_JUMP → 再開、STALE、RELEASED、IDLE）を確認する。
- c1: 書き込みモード（腕を支える）。倍率 0.5、±10 mm の往復から。次に ±30 mm。止まる条件を 1 つずつ（ボタンを離す、入力を止める、50 mm の飛び、床へ向かう動き）。腕は折りたたみ姿勢から始め、終わりは折りたたみ姿勢に戻して腕を支え、Ctrl-C。
- c2: グリッパー（小さな量から）。closed_rad・open_rad を決める。
- c3: 記録から閾値を直す。c4（実際の VR）は範囲外。

### P6d: 簡易説明書（P6c の後）

`docs/vr_follow_quickstart.md`。別の設計者が別のマシンで確認・修正するための最低限の説明書（前提とビルド、腕の校正、起動、VR 側のトピックとボタン、基準の取り方、状態と再開、停止、暫定の値と残課題）。zero_offset_deg は別のマシンではすべて 0 から始める（この腕の elbow_flex -6.3 は暫定なので書かない。値の測り方と決め方は「後で対応」として残課題に入れる）。docs だけなので colcon test は省略。

## 5. 変えないもの

bringup.launch.py、controllers.yaml（update_rate 25 のまま）、URDF・SRDF・MoveIt の設定、ドライバ、so_arm101_camera。

## 6. 暫定の値と残課題

- 0.5 rad/s、50 mm・30°、20 mm・10°、0.5 s、1.0 s、z 0.045 m、elbow +91°、C1 0.35 rad、C2 0.15 m、C3 0.2 rad、clutch_windup 0.06 rad、グリッパー 0.05 / 1.0 rad は、すべて暫定。P6c の記録で直す。
- 送信周期と time_from_start は b0 の結果で決める。
- 手の yaw は腕に伝わらない（pan が位置で決める）。
- 1 周期より短いボタンの押しは取りこぼす。
- C1 の 0.35 rad は、入力のメッセージごとの判定にした（解を 1 つ前のメッセージの解と比べる。速い手（1.5 m/s）でも当たらず、解の枝の入れ替わりは当たる）。値そのものは P6c の記録で決める。

## 7. b0 の結果（2026-11-17。モック、ROS_DOMAIN_ID=87、rmw_cyclonedds_cpp。update_rate は変更していない）

`examples/jtc_probe.py`（モック専用。ROS_DOMAIN_ID=87 と --mock が無いと動かない）で、shoulder_pan を一定の速さ（0.4 または 0.5 rad/s、3 s）で動かす 1 点の軌道を、周期 × time_from_start の組で送り、arm_controller の reference（`/arm_controller/controller_state`。25 Hz で出る）を測った。材料は `~/so_arm101_check/p6/`（git 管理外）。

**事実（実測）**: 新しい軌道を受けた最初の更新は進まない（2 章の 2 の推論どおり）。20 Hz では、reference が 200 ms に 1 回、0.1 rad（0.5 rad/s のとき）ずつ飛んで進んだ（更新の 79% が止まる）。この 0.1 rad は、ドライバの 1 周期の上限（0.06 rad）を超える。平均の速さは目標どおり（99〜100%）で、遅れ（lag）が伸びるだけ。

速さ 0.5 rad/s の結果（made = reference の速さ、lag = 目標との差の平均、step max = 25 Hz の更新 1 回で動いた最大、still = 動かなかった更新の割合）:

| 周期 Hz | time_from_start s | made rad/s | lag 平均 rad | step 標準偏差 rad | step max rad | still |
|---|---|---|---|---|---|---|
| 20 | 0.05 | 0.501 | 0.088 | 0.0406 | 0.100 | 79% |
| 20 | 0.08 | 0.494 | 0.159 | 0.0396 | 0.100 | 79% |
| 12.5 | 0.05 | 0.500 | 0.048 | 0.0200 | 0.040 | 50% |
| 12.5 | 0.08 | 0.500 | 0.088 | 0.0200 | 0.040 | 50% |
| 12.5 | 0.12 | 0.498 | 0.122 | 0.0198 | 0.040 | 50% |
| 12.5 | 0.16 | 0.495 | 0.155 | 0.0197 | 0.040 | 51% |
| 10 | 0.05 | 0.500 | 0.054 | 0.0202 | 0.048 | 39% |
| 10 | 0.08 | 0.500 | 0.060 | 0.0170 | 0.038 | 39% |
| 10 | 0.12 | 0.500 | 0.100 | 0.0165 | 0.036 | 39% |
| 10 | 0.16 | 0.499 | 0.129 | 0.0162 | 0.035 | 39% |
| 8.33 | 0.08 | 0.501 | 0.060 | 0.0140 | 0.030 | 32% |
| 8.33 | 0.12 | 0.500 | 0.085 | 0.0141 | 0.030 | 33% |
| 8.33 | 0.16 | 0.499 | 0.130 | 0.0140 | 0.030 | 33% |
| 6.25 | 0.08 | 0.501 | 0.077 | 0.0200 | 0.040 | 45% |
| 6.25 | 0.12 | 0.500 | 0.082 | 0.0114 | 0.027 | 24% |
| 6.25 | 0.16 | 0.500 | 0.104 | 0.0114 | 0.027 | 24% |

（速さ 0.4 rad/s の最初の 9 組も、同じ傾向。20 Hz は step max 0.08 rad、still 80%。）

**判断（暫定。P6c の実機で確認）**:

- **update_rate の変更は要らない**。送信周期を下げれば、滑らかに進む。
- 20 Hz は使わない（0.1 rad の飛びが出て、ドライバの 1 周期の上限 0.06 rad を超える）。
- **既定: `rate_hz` 10、`point_time_s`（time_from_start）0.08 s**。理由: 滑らかさ（step max 0.038 rad。ドライバの上限の 63%）と遅れ（0.06 rad）のバランスが良く、周期が丸い値で応答も速い。代わりの案: 8.33 Hz と 0.08 s（step max 0.030、遅れは同じ 0.06）。周期が 120 ms と長くなる。
- 参考: update_rate を 50 に上げれば 20 Hz でも滑らかになる見込みだが、controllers.yaml の変更とドライバの試験（`DesignLimits.StepLimitCoversTheWorstCaseTrajectoryStep`）に関わるため、ここでは行わない（将来の課題）。
- `FollowConfig` に `point_time_s` を足す（b1。範囲 (0, 0.5]）。`rate_hz` の既定を 20 → 10 に変える。
- 注意: reference の遅れ（約 0.06 rad）に、サーボの追従の遅れが足される。C3（測定と指令の差 0.2 rad）は、この合計より大きいことを P6c で確認する。

材料の sha256（全桁）: `b0_probe.txt` `e50a3ef90202efad9793abd3638e020b78edb5e946e54a4defc8bde1f726915d`、`b0_probe.csv` `acadfc19f2c09d2b73896b5ab74139f746143a5753b3e3aa06d53ec63bc667fc`、`b0_probe2.txt` `24203ef9ad6a91bbe4022a24fcd72a385d70f6121f8b50606159e389587a569b`、`b0_probe2.csv` `a1ef32a424bcb272a5ba7a8b0b3ce49f1663757f09e106ebda9d153ad95be70c`、`b0_probe2_raw.csv` `fd64c3ca323603cda5ece981b6a4544cc1bd7f40b804fbbad38c03a4a6ef3399`、`b0_probe3.txt` `ea15c24d441fd495fff715b16d37df9582c634da27e9222e402a01c69e75a625`、`b0_probe3.csv` `94d45323f2d28593f32fc422ee23f920e28fd80cca35e9007937573836a2f5c1`、`b0_probe3_raw.csv` `8d84517ac058ab3cbe3bce289f991a90b5ae179b20a63e79954cea8dff18b48b`、`b0_bringup.log` `9b93d4edc3f285b86b55ad0a6feda1d373fde040cdb811e0eae99252aa32fd41`。

停止の確認: bringup（PID 99902）は kill -INT で止め、ps に関係するプロセスなし、`ros2 node list --no-daemon` は 0 件。ログの終わりに move_group の exit code -11（既知の終了時の挙動）。

## 8. P6d の説明書に入れる注意（追加）

1 周期（既定 100 ms）より短いボタンの押しは取りこぼす（クラッチもグリッパーも）。ボタンは 0.2 s 以上押す。
