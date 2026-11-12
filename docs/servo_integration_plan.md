# MoveIt Servo（Diff-IK）を bringup に統合する計画

2026-11-12。**調査と計画だけ。コード・launch・設定の変更、モック・実機の起動はしていない**（ファイルを読む、`xacro` の展開、`MoveItConfigsBuilder` の Python 実行、numpy による運動学の計算、公式ソースの取得だけ。ノードもポートも使っていない）。承認が出るまで実装しない。

## 0. 要点（先に）

目的は、最終位置への移動で、カメラとのずれを残差で詰めること（近くでの追従に Servo を使う。大きな移動は OMPL か自前の解析 IK）。カメラの統合は今回の対象外。

調べて分かった、**計画を左右する事実**（根拠は各章。[事実] は読んだコード・実行結果、[推測] はそこからの推論で未確認）:

1. **今の `servo.yaml` + `servo.launch.py` のままでは、動かない・または危険**。理由は次の 5 つ。
   - (a) **5 関節に 6 次元の Twist を送ると、Servo 2.5.10 のコードが範囲外を読む**（特異点の計算 `utilities.cpp:104` が、6×5 の Jacobian の U の 6 列目を読む）。drift dimensions で 1 軸を捨てて Jacobian を 5×5 にする必要がある **[事実：コード]**。
   - (b) **drift dimensions は、Servo が IK プラグイン（今の `kinematics.yaml` の KDL）を使う経路では壊れている**。`servo.launch.py` は `robot_description_kinematics` を渡しているので、今はその経路になる。**Servo のノードに `robot_description_kinematics` を渡さない**（`kinematics.yaml` 自体は変更しない）と、逆 Jacobian の経路になり、drift dimensions が正しく効く **[事実：コード。起動での確認は未実施]**。
   - (c) **`start_servo` は必須**。呼ばないと Servo は何も計算しない **[事実：`servo_node.cpp:111-114`]**。
   - (d) **特異点のしきい値の既定（17 / 30）では、この腕はほぼどこでも「特異点」と判定される**。5×5 の Jacobian の条件数を計算したら、関節空間の一様サンプルで 17 未満は **0 %**、30 未満は 12〜28 %（4 章・付録）。そのまま使うと、ほとんどの姿勢で減速か停止になる。
   - (e) **JTC（`arm_controller`、25 Hz）に Servo（29.4 Hz）の軌道を送ると、ほとんど進まない見込み**（4d の「合っていない」より重い）。Servo の軌道は 1 点（34 ms 先）で `header.stamp=0`。JTC は新しい軌道の最初の更新で「直前の指令」をそのまま出す。JTC の周期（40 ms）が Servo の周期（34 ms）より長いので、**毎回の JTC の更新が「新しい軌道の最初の更新」になり、指令が進まない**（3 章 3.2）**[推測：JTC と Servo のソースを読んだ推論。動かして未確認]**。
2. **MoveIt の実行と Servo の同時入力を防ぐ方法**: JTC は、トピックの軌道とアクションの軌道を**区別せず、あとから来たものに黙って置き換える**（`joint_trajectory_controller.cpp:1235-1248`、アクションを中止しない）**[事実：コード]**。同じ `arm_controller` に両方を入れると、排他の仕組みが無い。**推奨は、Servo の出力先を別のコントローラ（`forward_position_controller`、Float64MultiArray）にして、`arm_controller` とは ros2_control の排他（同じ関節のコマンドを同時に握れない）で切り替える案**。(e) も同時に解消する。
3. **ドライバの制限は、Servo を守らない**。1 周期の変化量の上限は 0.48 rad（312 tick）。Servo が URDF の速度 10 rad/s で出せる最大は 0.34 rad/周期なので、ドライバの上限には**かからない**。実際の上限は Servo 側で決める（Servo 専用の関節速度の上限と、unitless の `scale`）。
4. 実機で、サーボの 1 tick（約 0.0015 rad）より小さい変化は動かない見込み。Servo は「**今の測定位置＋小さな差**」を目標にするため、差が 0.5 tick より小さいと丸めで同じ tick に戻り、**動かない**（約 0.03 rad/s 未満）**[推測]**。残差を詰める精度の下限は、先端で約 0.3 mm（1 tick）。

決めてほしいことは 8 章の表。

## 1. 現状の把握

### 1.1 何を読んだか（版）

- インストール版: `ros-humble-moveit-servo` **2.5.10-1jammy.20261006.192111**（`dpkg -l`）。**この .deb にはソース（.cpp）もパラメータの説明も入っていない**（`/opt/ros/humble/include/moveit_servo/*.h` と `share/moveit_servo/{config,launch}` だけ）。
- 動作はソースで確認した。ソースは公式 `moveit/moveit2` の**タグ `2.5.10`**（`moveit_ros/moveit_servo/src/` の `servo_calcs.cpp`、`servo_node.cpp`、`servo.cpp`、`utilities.cpp`、`enforce_limits.cpp`、`collision_check.cpp`、`servo_parameters.cpp`。`moveit_ros/planning/robot_model_loader`、`kinematics_plugin_loader`。`moveit_core/online_signal_smoothing/src/butterworth_filter.cpp`）を `raw.githubusercontent.com` から取得して読んだ。**インストール版の .deb がこのタグから作られたことは、バージョン番号の一致だけで、バイナリとの照合はしていない [推測]**。取得したファイルは作業用の一時ディレクトリ（リポジトリの外）にあり、リポジトリには入れていない。
- ヘッダ（`servo_calcs.h`、`servo_parameters.h`、`servo_node.h`）はインストール版を直接読んだ。ソースと矛盾しない（メンバ・パラメータの名前と既定値が一致）。
- JTC と forward controller は、インストール版 `joint_trajectory_controller` 2.54.1 のタグ（`ros-controls/ros2_controllers` 2.54.1）、`controller_manager` 2.54.3 のタグ（`ros2_control`）のソースを同様に読んだ（版は `package.xml` のもの）。

### 1.2 `servo.launch.py` と `servo.yaml` の中身

`src/so_arm101_moveit_config/launch/servo.launch.py`（37 行）:
- `MoveItConfigsBuilder("so101_new_calib", package_name="so_arm101_moveit_config").to_moveit_configs()`（**mappings なし**）。`servo.yaml` を `{"moveit_servo": yaml}` として渡す。
- ノード: `moveit_servo/servo_node_main`、名前 `servo_node`。parameters = `[servo_params, robot_description, robot_description_semantic, robot_description_kinematics]`。**`robot_description_planning`（joint_limits）は渡していない**。

`src/so_arm101_moveit_config/config/servo.yaml`（Panda の例 `panda_simulated_config.yaml` を日本語コメント付きで写したもの。キー 36 個のうち 32 個を設定、**4 個は既定のまま**: `halt_all_joints_in_cartesian_mode`（true）、`halt_all_joints_in_joint_mode`（true）、`monitored_planning_scene_topic`、`override_velocity_scaling_factor`（0.0））。宣言されていないキーはない（`servo_parameters.cpp` の宣言と照合）。

| 項目 | 値（`servo.yaml` の行） | 意味・注記 |
|---|---|---|
| `command_in_type` | unitless（:6） | 入力は [-1, 1]。**1 を超える成分があると、その Twist は捨てられる**（`servo_calcs.cpp:977`） |
| `scale.linear / rotational / joint` | 0.1 / 0.2 / 0.1（:12-14） | 入力 1.0 のときの 0.1 m/s、0.2 rad/s、0.1 rad/s（Panda の例は 0.4 / 0.8 / 0.5） |
| `publish_period` | 0.034（:17） | 29.4 Hz。`low_latency_mode: false` |
| `command_out_type` / `command_out_topic` | JointTrajectory / `/arm_controller/joint_trajectory`（:21, :57） | **JTC のトピックに直接出す** |
| `publish_joint_positions / velocities` | true / true | 軌道の点に位置と速度を入れる |
| `smoothing_filter_plugin_name` | Butterworth（:27 付近） | 1 次の低域通過（係数の既定 1.5。`y_k = 0.4(x_k + x_{k-1}) + 0.2 y_{k-1}`） |
| `is_primary_planning_scene_monitor` | false | planning scene は `move_group` から取る（`requestPlanningSceneState`） |
| `move_group_name` / `planning_frame` / `ee_frame_name` / `robot_link_command_frame` | arm / base_link / gripper_frame_link / base_link | いずれも URDF・SRDF に存在 |
| `incoming_command_timeout` | 0.1 s（:42） | 入力が途切れたと見なす時間 |
| `num_outgoing_halt_msgs_to_publish` | 4 | 実際にはほとんど効かない（3 章 3.4） |
| `lower_/hard_stop_singularity_threshold` | 17.0 / 30.0（:46-47） | **この腕には合わない**（付録） |
| `leaving_singularity_threshold_multiplier` | 2.0 | 特異点から離れる向きの許容 |
| `joint_limit_margin` | 0.1 rad（:48） | URDF の limit の 0.1 rad 手前で止める |
| `check_collisions` / rate / 閾値 | true / 10 Hz / 自己 0.01 m・環境 0.02 m | 衝突までの距離が閾値内で指数的に減速、衝突なら 0 |
| 入出力トピック | `~/delta_twist_cmds`、`~/delta_joint_cmds`（:50-51）、`/joint_states`、`~/status` | ノード名が `servo_node` なので `/servo_node/delta_twist_cmds` など |

### 1.3 bringup の robot_description・SRDF・kinematics との違い

- **robot_description**: `servo.launch.py` は mappings なし（`use_fake_hardware` の既定は true、`urdf.xacro:5`）。bringup（`mock:=false`）は `use_fake_hardware:=false` ＋ 実機の引数。**違いは `<ros2_control>` ブロックだけ**。Python で、mappings なしの robot_description と、実機の読み取り専用の mappings の robot_description を作り、`<ros2_control>…</ros2_control>` を取り除いて比べたら**一致**した（全体は当然不一致）**[事実：実行結果]**。Servo は `<ros2_control>` を使わないので、運動学・limit・衝突形状は bringup と同じ。`move_group` 自身も mappings なし（`move_group.launch.py:15-18`）で、実機でも同じ状態。
- **SRDF**: 同じ（`MoveItConfigsBuilder` の同じ `so101_new_calib.srdf`）。group `arm` = chain `base_link` → `gripper_frame_link`、関節は 5 つ（pan・lift・elbow・wrist_flex・wrist_roll）。
- **kinematics**: `kinematics.yaml` は KDL（`arm`）。`servo.launch.py` はこれを渡す → **Servo は IK プラグインの経路になる**（`servo_calcs.cpp:624` の `searchPositionIK`、`return_approximate_solution=true`）。5 関節の KDL に 6 次元の姿勢を要求することになり、近似解で動くが、drift dimensions と両立しない（3.3）。
- **joint_limits**: 渡していない。Servo のロボットモデルは URDF の limit だけ（velocity 10 rad/s、位置は URDF のもの）。`joint_limits.yaml`（velocity 5.0、acceleration 1.0）は読まれない。

URDF の関節の位置の範囲（rad）とドライバ（校正）の範囲の比較 **[事実]**:

| 関節 | URDF | ドライバ（校正の min..max を rad に換算） |
|---|---|---|
| shoulder_pan | ±1.920 | ±2.094 |
| shoulder_lift | ±1.745 | ±1.822 |
| elbow_flex | **±1.690** | **±1.719**（`±1.7192`） |
| wrist_flex | ±1.658 | ±1.815 |
| wrist_roll | −2.744..+2.841 | ±2.948 |

→ **URDF の方が全関節で狭い**。Servo は URDF の limit の 0.1 rad 手前で止まるので、ドライバのクランプには届かない（かつ、届く前に止まる）。注意: **実機の休止姿勢の elbow_flex は +1.6901（inventory の記録）で、URDF の上限 1.69 に等しく、Servo の margin（1.59）の内側**。Servo は「limit に近づく向きに動く関節がある」と全関節を止める（`halt_all_joints_in_cartesian_mode: true`、`servo_calcs.cpp:710-722`）。休止姿勢からは、elbow を増やす向きの指令だけが止まり、減らす向きは動く（`servo_calcs.cpp:835-850` の判定は、limit に近づく向きに動く関節だけを対象にする）。

### 1.4 inventory に記録した 4d の課題の再確認（1 つずつ）

| # | 課題 | 結論 |
|---|---|---|
| 1 | bringup に組み込まれていない | **確認**。`bringup.launch.py` が include するのは `static_virtual_joint_tfs`・`spawn_controllers`・`move_group`・`moveit_rviz`・`warehouse_db` だけ（`:190-226`）。`servo.launch.py` は install されるが、誰も起動しない |
| 2 | `publish_period` 0.034 と JTC 25 Hz | **確認し、重さを訂正**。「Servo の方が速い」は事実だが、結果は「ずれる」ではなく「**ほぼ進まない**」見込み（3.2）。JTC を使い続けるなら、JTC を Servo の周期の半分以下にする（`arm_controller` の `update_rate` を 50 に上げる）か、Servo の周期を JTC の 2 倍以上にする。forward controller 案ではこの問題が無い |
| 3 | `joint_limits` を読まない | **確認**。`RobotModelLoader` は `robot_description_planning.joint_limits.<関節>.*` をノードのパラメータから読んで limit を上書きする（`robot_model_loader.cpp:128`〜）。`servo.launch.py` はそれを渡さない。**逆に、これを利用して Servo 専用の速度上限を入れられる**（Servo 専用のファイルを別に作れば、MoveIt の `joint_limits.yaml` は変えずに済む）。`joint_limits.yaml` の 5.0 rad/s を渡しても、初期の試験には速すぎる |
| 4 | `servo.launch.py` の robot_description が `use_fake_hardware` 既定 | **確認。ただし実害なし**（1.3。`<ros2_control>` 以外は同一）。bringup に統合するなら、bringup の `moveit_config.robot_description` をそのまま渡せば、この問題自体が消える |
| 5 | drift dimensions | **確認し、必須と判明**（2 章・3.3）。設定ファイルにパラメータは無く、**サービス `~/change_drift_dimensions`**（`moveit_msgs/srv/ChangeDriftDimensions`）で、実行時に設定する。既定は全部 false（`servo_calcs.h`）。**設定しないまま 6 次元の Twist を送ると未定義動作**（`utilities.cpp:104`）。さらに IK プラグインの経路では drift が壊れている（3.3） |
| 6 | `start_servo` の要否 | **必須と確認**。`ServoNode` のコンストラクタは `Servo` を作るだけで、`start()`（計算スレッドの起動）は `~/start_servo` サービスの中でしか呼ばれない（`servo_node.cpp:111-114`）。`~/stop_servo` と `~/pause_servo` は同じ動作（どちらも `setPaused(true)`、`servo_node.cpp:118-130`）で、`~/unpause_servo` で再開。**起動しただけでは、Servo は何も出さない**（安全側）。`start()` は、joint_states が揃うのを最大 10 秒待ち、**タイムアウトしても `success=true` を返す**（`servo.cpp:62`、`servo_node.cpp:114-115`）ので、「成功」を信用せず、`/servo_node/status` と出力を見る |

### 1.5 Servo の入力・出力・途切れ・特異点・衝突

- **入力**（`servo_calcs.cpp:97-104`）: `geometry_msgs/TwistStamped`（`~/delta_twist_cmds`）、`control_msgs/JointJog`（`~/delta_joint_cmds`）。QoS は `SystemDefaultsQoS`（reliable、depth 10）。**Twist が JointJog より優先**（両方有効なら Twist だけ処理、`:394`）。
- **command frame**: Twist の `header.frame_id` は、planning frame（`base_link`）、空または `robot_link_command_frame`（= `base_link`）、`ee_frame_name`（`gripper_frame_link`）、それ以外の**モデル内のリンク名**を受け付ける（`:537-556`）。**モデルに無いフレーム名（カメラ）を入れると `getGlobalLinkTransform` が例外を投げて Servo のスレッドが落ちる見込み [推測]**。カメラ座標の差分は、**送る側で `base_link` に変換してから送る**（4d・e の方針どおり）。
- **出力**: `command_out_type` が JointTrajectory なら `command_out_topic` に、点 1 つ（`time_from_start = publish_period`、`header.stamp = 0`、`:488`）。Float64MultiArray なら位置**または**速度のどちらか一方だけ（両方 true は設定の検証で拒否、`servo_parameters.cpp:422-428`）。
- **「今の測定位置 + 差」**: 毎周期、`/joint_states` の測定位置から計算し直す（`internal_joint_state_ = original_joint_state_`、`updateJoints`）。目標を積算しない。**積算誤差が出ず、ノードが落ちても目標は測定位置から高々 1 歩しか離れない**（3.4）。
- **途切れ**: Twist の `header.stamp` が 0.1 s（`incoming_command_timeout`）より古いと stale（`:342`）。**stamp が 0 のメッセージは時刻を更新しない**（`:1161`）ので、stamp を入れない送信側は永久に stale で、**動かない（安全側）**。逆に**未来の stamp は stale にならない**ので、送る側は必ず `now()` を入れる。Servo は最新の 1 件を、stale になるまで毎周期使い続ける（1 回だけ送ると 0.1 s / 0.034 s ≒ 3 周期ぶん動く）。**twist も joint も stale になると、何も publish しない**（`:440-446`）。`num_outgoing_halt_msgs_to_publish`（4）は「入力が全部ゼロ」のときの停止メッセージの回数だが、実際には stale の判定のほうが先に効き、**停止メッセージはほとんど出ない**（読んだコードの推論 **[推測]**）。したがって「止まる」は「**出力が止まり、アクチュエータが最後の目標を保持する**」であって、減速の軌道を出すわけではない。
- **特異点**: reduced Jacobian（drift で捨てた行を除く）の条件数（最大／最小特異値）で、下限（`lower`）を超えると減速、上限（`hard_stop`）で停止（`utilities.cpp:139-`）。
- **衝突**: 10 Hz で、現在の状態の距離を計算し、`~/collision_velocity_scale`（0..1）を publish。0 なら `HALT_FOR_COLLISION`（`collision_check.cpp:118`）。**衝突の判定は 10 Hz なので、0.1 s 遅れる**。

## 2. 5 関節への対処: drift dimensions

### 2.1 なぜ必要か

- 6 次元の Twist に対し、関節は 5 つ。Jacobian は 6×5。Servo の特異点の計算は、`svd.matrixU().col(num_dimensions - 1)`（`utilities.cpp:104`）で、`num_dimensions` は Twist の次元（6）。thin SVD の U は 6×5 なので、**6 列目（添字 5）は範囲外** [事実：コード]。release ビルドでは Eigen の境界検査が無いので、未定義の値を読む。
- drift で 1 軸を捨てると、reduced Jacobian が 5×5 になり、`col(4)` で範囲内になる。**1 軸だけ捨てるのが必須**（2 軸以上捨てても動くが、制御できる軸が減る）。

### 2.2 どの軸が制御できないか（計算した）

URDF（`so101_new_calib`、`xacro` で展開）から、base_link 基準・gripper_frame_link の原点の Jacobian（6×5）を numpy で計算した（付録のスクリプト。ノード不使用）。左零空間（「どの向きの Twist が、どの関節でも作れないか」）:

| 姿勢 | 作れない Twist の向き（x, y, z, rx, ry, rz） | 捨てる行ごとの条件数（reduced 5×5） |
|---|---|---|
| center（全関節 0、先端が水平に前） | ≈ (0, −0.94, 0, 0, 0, +0.33) | **y: 29.6、yaw: 29.6**、roll(x): 240、x・z・pitch: 悪い（10⁶〜） |
| home（SRDF の `home`） | ≈ (0, −0.97, 0, 0.17, 0, 0.15) | y・roll・yaw: 71.8（同じ） |
| 実機の休止姿勢（inventory の記録の位置） | ≈ (0.09, −0.97, 0, 0.13, 0.01, 0.18) | y・yaw・roll: 約 124 |
| 作業範囲の中ほど（`mid`、`reach`） | y 並進が主 | y・yaw・roll・pitch: 22〜41 |
| initial（wrist_flex 1.57、**先端が真下**） | **≈ (0, 0, 0, 1.0, 0, 0)**（x 軸まわりの回転） | **roll(x): 29**、y: 3085、yaw: 2183 |

意味 [推測を含む解釈]: パン（z 軸まわり）が「横（y）方向の並進」と「ヨー（z まわりの回転）」を**一緒に**作る。先端が水平なら、横に動くことと向きを回すことが同じ関節で結ばれていて、独立には指令できない。**先端が真下を向く姿勢では、その代わりに「先端を横に傾ける（x まわりの回転）」が作れない**（パンの軸と wrist_roll の軸が平行になる）。

### 2.3 案と推奨

| 案 | 捨てる軸 | 利点 | 欠点 |
|---|---|---|---|
| A | **yaw（z まわりの回転）**（`drift_z_rotation`） | x・y・z の並進と、ロール・ピッチ（先端の傾き）を制御できる。**カメラの残差は並進が主なので、横（y）も直接詰められる**。ヨーはパンに従って決まる | 先端が真下の姿勢では特異（条件数 2183）。先端が水平〜斜めの姿勢向き |
| B | y（横の並進） | 水平の姿勢で条件数が A と同等 | **横の残差を指令できない**（y の指令は捨てられる）。残差の補正の目的に合わない |
| C | roll(x)（x まわりの回転） | 先端が真下の姿勢で正しい（条件数 29） | 水平の姿勢（center）で 240 と悪い |
| D | 姿勢で切り替える（水平: A、真下: C。`change_drift_dimensions` を切り替え時に呼ぶ） | 両方に対応 | 切り替えの手順と試験が増える。切り替え中の Twist の扱いを決める必要 |

**推奨: A（yaw を捨てる）を既定とし、最終位置への接近が真下（トップダウン）なら D**。→ **8 章の決めてほしいこと（最終位置への接近は水平か、真下か）**。

### 2.4 drift の設定の仕方（安全に）

- 設定は**サービス**のみ（パラメータなし）。Servo の起動直後（`start_servo` の前）に呼ぶ。**送信ツールが毎回、`change_drift_dimensions` を呼び、成功を確認してから Twist を送る**（既に設定済みでも同じ値を呼ぶ。冪等）。ツールは、設定できなければ Twist を送らない。
- Servo 側から「drift が未設定」を検知する方法は無い [事実：パラメータにもステータスにも無い]。だから、**ツールが唯一の門**になる。ほかのクライアントが生の Twist を送ると、未設定のまま 6 次元で処理される恐れがある（2.1）。将来のブリッジ（rosbridge）も同じ門を通すこと。

### 2.5 逆 Jacobian の経路にする（kinematics を渡さない）

- Servo は、`joint_model_group_->getSolverInstance()` が空だと逆 Jacobian の経路（`use_inv_jacobian_ = true`、`servo_calcs.cpp:189`）を使う。solver は `robot_description_kinematics.arm.kinematics_solver` から作られる（`kinematics_plugin_loader.cpp:315-316`）。**そのパラメータをノードに渡さなければ solver は無い** **[事実：コード。起動での確認（ログに `No kinematics solver instantiated for group 'arm'. Will use inverse Jacobian` が出ること）はモックで行う]**。
- IK プラグインの経路が drift と両立しない理由 [事実：コード]: `removeDriftDimensions` が `delta_x`（6 要素）の要素を削って 5 要素にする（`servo_calcs.cpp:573`、`removeDimension` `:1060`）が、IK の経路は削った後の `delta_x` を `[0]..[5]` で読む（`:587-600`）。yaw（添字 5）を捨てると `delta_x[5]` は範囲外。y を捨てると z と回転がずれて読まれる。
- `kinematics.yaml` は変更しない（MoveIt 本体の設定。`move_group` は引き続き KDL を使う）。変えるのは **Servo ノードに渡すパラメータの一覧**だけ。

## 3. 安全の設計（重点）

### 3.1 MoveIt の実行と Servo の出力が `arm_controller` に同時に入らないようにする

**事実**: JTC は、トピック（`topic_callback`、`joint_trajectory_controller.cpp:1235`）もアクション（`goal_accepted_callback`、`:1291`〜）も、**最後に来たものを黙って採用する**（両方とも `traj_msg_external_point_ptr_` に書く）。アクションの実行中にトピックが来ても、アクションは中止されず、軌道だけが置き換わる（`preempt_active_goal` が呼ばれるのは、新しい**アクション**が来たときだけ、`:1305`）。つまり、**JTC には排他が無い**。MoveIt の実行（`FollowJointTrajectory`）の途中に Servo の軌道が混ざると、MoveIt の軌道が途中で消える。

案:

| 案 | 内容 | 利点 | 欠点 |
|---|---|---|---|
| **X1（推奨）** | Servo の出力を `std_msgs/Float64MultiArray` にして、新しい `position_controllers`-型の **`forward_command_controller`（`forward_position_controller`、5 関節の position）**へ。`arm_controller` と `forward_position_controller` は**同じ関節の position コマンドを要求する**ので、ros2_control が**同時に active にさせない**。切り替えは `ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller`（逆も）。常に片方だけ active | **ハードウェアに近い層での排他**（ソフトの約束に頼らない）。`arm_controller` が inactive の間、MoveIt の実行は**拒否される**（JTC は inactive だとゴールを `REJECT`、`:1260-1261`）。Servo の出力は、JTC のトピックには**届かない**（別のトピック）。**3.2 の JTC との周期の問題が消える**。停止時は最後の目標を保持。実機の Servo の標準的な構成 | コントローラ 1 つと、切り替えの手順が増える。`controllers.yaml` に定義を足し、spawner を `--inactive` で足す。切り替えは、手動（`ros2 control …`）か小さなスクリプト。`moveit_controllers.yaml` は変更しない（MoveIt は `arm_controller` だけを使う） |
| X2 | 今の JTC のまま、Servo を**未開始（`start_servo` を呼ばない）か pause**にしておき、MoveIt の実行は Servo が止まっているときだけ行う（手順で担保） | 追加のコントローラが要らない | **手順だけの排他**。pause し忘れ、`unpause` し忘れで混ざる。3.2 の周期の問題が残る。`servo.yaml` の出力先が `/arm_controller/joint_trajectory` のままなので、うっかり Servo を動かすと MoveIt と衝突する |
| X3 | 排他を管理する小さなノードを作る（Servo の出力を中継し、MoveIt の実行中は遮断） | 柔軟 | 新しいコード・ノード・試験が増える。JTC の問題は残る。過剰 |

**推奨: X1**。理由: (1) 排他が ros2_control の仕組みで担保される、(2) 3.2 の JTC の問題を迂回できる、(3) `servo.yaml` の出力先が JTC から離れる（静的な試験で「Servo の出力先が `/arm_controller/joint_trajectory` でないこと」を確認できる）。

X1 で確認が要る点 **[推測。モックで確認]**:
- Servo が出す Float64MultiArray の関節の順は、`arm` グループの active joint の順（`servo_calcs.cpp:148`）で、**forward controller の `joints` の順と一致させる**必要がある（`forward_controllers_base.cpp:131-142` は要素数しか見ない。順は検査されない）。SRDF の chain（`base_link` → `gripper_frame_link`）の順は pan・lift・elbow・wrist_flex・wrist_roll のはずで、`controllers.yaml` の `joints` と同じ。**静的な試験で、この一致を確認する**（7 章）。
- forward controller は、有効化した直後は、最初のメッセージが来るまで何も書かない（`forward_controllers_base.cpp:106、123-129`）。JTC を止めた後、コマンドは直前の値（ドライバの `hw_commands_`）のまま保持される。切り替えで腕が動かないことを、モックの `/joint_states` と、実機の W 段階で確認する。
- ドライバの `hw_commands_` は、**切り替えの間も最後の値を保つ**（`write_goal_positions` は毎周期 `hw_commands_` を送る）。

### 3.2 JTC に Servo の軌道を送った場合の動き（X1 を採らない場合の注意。open_loop_control: true の下）

**事実（ソース）**: Servo の軌道は 1 点（`time_from_start = 0.034`、`stamp = 0`）。JTC は、新しい軌道を受けた最初の更新で、`open_loop_control: true` のとき `last_commanded_state_`（直前の自分の指令）を始点にして（`joint_trajectory_controller.cpp:206-211`）、その時刻（= 軌道の開始時刻、`stamp = 0` なので最初の更新の時刻、`trajectory.cpp:130`）でサンプルする。始点の時刻と同じ時刻なので、**出力は直前の指令そのもの**（`trajectory.cpp:148-165`）。2 回目の更新から、終点（0.034 s 後）へ進む。

**推論 [推測。未実行]**: `arm_controller` の `update_rate` は 25（周期 40 ms、`controllers.yaml:22`）。Servo の周期は 34 ms。**40 ms の間に必ず Servo のメッセージが 1 つ以上来る**ので、JTC の毎回の更新が「新しい軌道の最初の更新」になり、出力が直前の指令のまま**進まない**。ジッタで、たまに新しいメッセージが来ない更新があり、そのときだけ進む。結果は「ほとんど動かない、または不規則に動く」。`open_loop_control: false` でも、始点が測定位置になるだけで、同じく進まない。
- 進むには、JTC の周期が Servo の周期より十分短いこと（例: JTC 50 Hz = 20 ms、Servo 40 ms 以上）。その場合でも、新しい軌道ごとに最初の 1 更新は進まず、**意図した速度の約半分**になる見込み。
- したがって JTC を使う案では、`arm_controller` の `update_rate` を 25 → 50 にする変更が要る。**これは MoveIt の実行と、ドライバの試験（`DesignLimits.StepLimitCoversTheWorstCaseTrajectoryStep` は `update_rate` から必要な 1 周期の上限を計算する。50 にすると必要量は 0.48 → 0.24 rad に減って通る）に影響する**ので、決めてもらう項目にする。

**X1 ではこの問題を踏まない**（forward controller は軌道を補間せず、毎周期、最新の目標をそのまま書く）。

### 3.3 ドライバの制限と、Servo の 1 周期あたりの変化量

ドライバ（`sts3215_system.cpp`、書き込みモード）の制限 [事実：コード]:

| 制限 | 値 | 場所 |
|---|---|---|
| 1 周期の変化量（`max_step_raw_`） | 6.0 rad/s × 0.08 s / tick = **312 tick ≒ 0.479 rad**（`tick = 2π/4095 = 0.001534 rad`） | `:832`、`sts3215_system.hpp:32-34` |
| 校正範囲へのクランプ | raw の min..max（elbow ±1.7192 など）。範囲外の指令が ±50 tick 以内なら黙ってクランプ、超えると「異常」と数え、50 周期（1 秒）続くとフォルト（トルク OFF にはならず保持） | `:963-995`、`kClampToleranceTicks` |
| NaN・inf | 破棄して直前の目標を保持。異常として数える | `:963-` |
| 書き込みモードの目標の初期値 | 現在位置 | `:828` |
| 読み取り専用 | `hw_commands_` を毎周期、測定位置に上書き（指令は破棄）。指令と測定が 0.02 rad 以上違うと、最初の 1 回だけ WARN | `sts3215_system.cpp:511、525-538` |

**Servo の 1 周期の変化量との関係**:
- Servo の目標は、測定位置から高々（速度の上限 × `publish_period`）しか離れない。URDF の速度（10 rad/s）なら 0.34 rad/周期 < 0.479 rad。**ドライバの上限には達しない**＝ドライバは Servo の暴走を止めない。
- 通常の動作では、Servo の出力はドライバに拒否・制限されない（クランプにも step limit にも触れない）。ドライバの制限が効くのは、Servo の側が壊れたとき（NaN、桁違いの値）だけ。
- 設定の通り Servo 専用の速度上限（3.5）を入れれば、1 周期は 0.01 rad 程度（0.3 rad/s のとき）で、ドライバの上限の約 1/50。
- **サーボの量子化 [推測]**: tick は 0.001534 rad。Servo の差が 0.5 tick（0.00077 rad）未満だと、丸めで元の tick に戻る。`publish_period` 0.034 s なら約 **0.023 rad/s** 以下の関節速度は動かない。先端で約 5 mm/s（腕の長さ 0.2 m、Jacobian の大きさ 約 5 rad/m として約 0.025 rad/s）。**初期の試験の速度の下限**（0.05 rad/s 程度）と、残差補正の精度の下限（約 1 tick ≒ 先端で 0.3 mm）に関わる。実機の W 段階で、動き出す速度を測る。

### 3.4 入力が止まったとき・ノードが落ちたとき

| 状況 | 動き | 根拠 |
|---|---|---|
| 送信側が止まる（Twist を送らなくなる） | 0.1 s 後に stale。**Servo は出力を止める**。腕は最後の目標へ行って止まる（目標は測定位置から高々 1 歩先） | `servo_calcs.cpp:342、440-446` |
| 送信側がゼロの Twist を送る | ゼロ速度の目標（直前の目標の位置）を数周期出し、stale 後に出力を止める | `:404-427` |
| 送信側が stamp 0 で送る | 常に stale。動かない（安全側） | `:1161` |
| 送信側が未来の stamp で送る | stamp + 0.1 s まで動き続ける | 同上。**送信ツールは `now()` を使う** |
| Servo のノードが落ちる | 出力が止まる。X1: forward controller が最後の値を保持し、ドライバが同じ目標を送り続ける（高々 1 歩先で停止）。JTC 案: 点が 1 つの軌道の終点を保持（`allow_nonzero_velocity_at_trajectory_end` が true でも、位置指令は終点の位置） | `servo_calcs.cpp` ／ JTC `trajectory.cpp:205-222` |
| `/joint_states` が NaN（ドライバが通信断で状態を NaN にしたとき） | Servo は入力が NaN でも状態の NaN は検査しない。計算結果が NaN になり、ドライバは NaN を破棄して保持する（異常として数え、1 秒でフォルト）。**Servo 側の検査は無い** [推測] | `servo_calcs.cpp` ／ `sts3215_system.cpp:963-970` |
| ドライバのフォルト（通信断など） | Servo は知らない。目標を出し続けても、ドライバが送らない（`fault_` で `OK` を返すだけ） | `sts3215_system.cpp:951` |
| `ros2 control` で forward controller を deactivate | その瞬間に Servo の出力は届かなくなる（Servo は動き続ける）。**Servo も `pause_servo` する** | — |

→ **停止の確認の順序**: Twist の送信を止める → ツールの Ctrl-C（ゼロ Twist を送って終わる） → `pause_servo` → Servo を `kill -INT`（自分で起動した PID だけ）。CLAUDE.md の規則（`pkill`・`killall` 禁止）を、手順書にも書く。

### 3.5 速度の上限の初期値

| 層 | 設定 | 初期値の案 | 備考 |
|---|---|---|---|
| 入力の上限 | `command_in_type: unitless`＋`scale` | `linear` **0.01 m/s**、`rotational` **0.05 rad/s**、`joint` **0.05 rad/s** | 送信側が最大の 1.0 を送っても、この速度が上限（**1 を超える成分は Servo が捨てる**）。今の値（0.1 m/s、0.2 rad/s）は初期には速すぎる |
| 関節速度の上限（Servo 専用） | `robot_description_planning.joint_limits.<関節>.max_velocity`（Servo のノードだけに渡す別ファイル） | **0.3 rad/s**（5 関節） | 特異点の近くで逆 Jacobian が大きな関節速度を出しても、Servo が全体を縮める（`enforce_limits.cpp`）。`joint_limits.yaml` と `URDF` は変えない |
| 特異点 | `lower_singularity_threshold` / `hard_stop_singularity_threshold` | **50 / 100**（既定 17 / 30） | 付録の分布（5×5、y か yaw を捨てた場合）で、作業範囲らしい領域の中央値 25・95 % 点 44・99 % 点 116。**暫定。モックで `status` を見て決める**。条件数は、メートルとラジアンを混ぜた値で、この腕は小さい（リンク約 0.1 m）ので大きく出る |
| 関節の端 | `joint_limit_margin` | 0.1（現状どおり） | URDF の limit の 0.1 rad 手前（ドライバの校正範囲より内側） |
| 実機の最初の試験の送信側の上限 | ツールの引数 | 入力 0.5 以下、時間 3 秒以下、先端の総移動量 2 cm 以下 | `send_joint_delta.py` と同じ流儀（超える指定は拒否） |

### 3.6 `read_only:=true` で Servo を動かして、指令の計算だけを確かめられるか

**できる**（コードで確認）:
- 読み取り専用のドライバは、`hw_commands_` を毎周期、測定位置に上書きし（`sts3215_system.cpp:511`）、`write()` では何も送らず、指令が測定より 0.02 rad 以上違うときだけ WARN を 1 回出す（`:525-538`）。**サーボには一切書かない**（トルク OFF のまま）。X1 では、forward controller が `hw_commands_` に書くが、すぐ上書きされる。Servo の出力が 0.02 rad より小さければ WARN も出ない。
- Servo は `/joint_states`（測定位置）から計算して出力する。腕が動かないので測定位置は変わらず、**毎周期、同じ測定位置から 1 歩先の目標**を出す（積算されない）。これで確認できる: (1) Servo が起動し、`start_servo`・drift 設定が通ること、(2) 出力（`/forward_position_controller/commands` を `ros2 topic echo`）が期待値（測定位置 + J⁺·Twist·`publish_period`、ツールが同じ URDF で計算して比較）と一致すること、(3) 速度上限・特異点・衝突の減速（`status`・`collision_velocity_scale`）、(4) 入力が止まると出力が止まること、(5) 起動時の CPU・周期の遅れの警告（`run_duration` の WARN）。
- **確認できないこと**: 実際に動いたときの追従（積算、測定位置の遅れ、量子化の不感帯）。これはモック（積算と閉ループ）と、実機の書き込み（量子化）で確認する。
- 読み取り専用の中でも、X1 の切り替え（`switch_controllers`）で、`arm_controller` が inactive になる。これは腕を動かさない。

## 4. 統合の方法の案

前提: どの案でも、Servo ノードは**起動しても動かない**（`start_servo` を呼ぶまで）。既定は Servo なし。

| 案 | 内容 | 利点 | 欠点 |
|---|---|---|---|
| **案 1（推奨）** | `bringup.launch.py` に `use_servo`（既定 `false`、`DeclareBooleanLaunchArg`）を足す。`true` のとき、**bringup 内で `servo_node_main` を直接 `Node` で定義**する（bringup の `moveit_config.robot_description`・semantic をそのまま渡し、`robot_description_kinematics` は渡さず、Servo 専用の joint limits を `robot_description_planning` として渡す）。forward controller の spawner（`--inactive`）も `use_servo` のときだけ起動 | 同じ robot_description を使う（4d の 4 が消える）。1 回の launch で全部立つ。`use_servo:=false`（既定）では今と同じ（検証済みの 6 通りの robot_description・pipelines に影響しない） | 引数が **21 → 22**（`test_launch_arg_check.py:66` の `len(DECLARED) == 21` と、引数ごとの試験 21 件の増減、`--show-args`）。launch が長くなる。`servo.launch.py`（moveit_config）は使われなくなる（残す／`git rm` は別途承認） |
| 案 2 | `bringup.launch.py` に `use_servo` を足し、`IncludeLaunchDescription` で `servo.launch.py` を呼ぶ。`servo.launch.py` を、引数（`use_fake_hardware` など）を受けて robot_description を揃え、Servo 専用の設定を渡す形に直す | 既存の流儀（`include(...)`）と同じ。moveit_config の launch が 1 か所で完結 | **`bringup.launch.py` の未宣言引数の検査**（`_check_launch_arguments`）は「ユーザーが打った引数」だけを見るので、include 先の引数は影響しない（試験 `test_launch_internal_values_are_not_reported`）が、`servo.launch.py` に引数を足すと「宣言された引数の一覧（include 先を含む）」が増え、**22 を超える**。moveit_config の launch が bringup の引数を知る形になり、依存の向きが逆 |
| 案 3 | **別の launch のまま**。`so_arm101_bringup/launch/servo.launch.py` を新設（`ros2 launch so_arm101_bringup servo.launch.py`）。bringup は変更なし | **既存の bringup・試験に一切触れない**（引数 21 のまま、47 件のまま）。Servo だけ起動・停止できる（Servo を止めても、ハードウェアの launch に影響しない）。実機の段階的な試験に向く | 起動が 2 コマンド。robot_description は、同じ `MoveItConfigsBuilder` を**もう一度**組み立てる（mappings なしで `<ros2_control>` 以外は同一、1.3）。forward controller の spawner も別に起動する |

**推奨: 案 1。ただし実装の段階を分け、まず案 3 の形（別 launch）で実機までの確認を通し、最後に案 1 へ畳み込む**手もある。→ 8 章。理由: 実機の最初の試験では、「Servo だけを起動・停止できる」ことが安全上の利点（ハードウェアの launch を動かしたまま、Servo の `kill -INT` ができる）。

引数を足す場合の影響（案 1）:
- `--show-args` の数: **21 → 22**（`use_servo` の 1 つ。`forward_controller` などは足さない）。
- 未宣言の引数の検査: `declared_names` は `ld.get_launch_arguments()` の実行時の一覧なので、`use_servo` を宣言すれば自動で受け付ける。**試験の `len(DECLARED) == 21` は 22 に直す**（既存の試験の変更 1 か所）。`test_every_declared_argument_is_accepted` は引数の数だけ増える（21 → 22 件）。
- `hardware` の削除メッセージなどには影響しない。
- 既存の 6 通りの robot_description（`gen_rd6.sh`）は、`use_servo` を含めず同一のまま（`use_servo` は robot_description の mappings に入れない）。

Servo の設定ファイルの置き場所:
- **推奨**: `so_arm101_bringup/config/` に、**Servo 専用の 2 つ**を置く（`servo.yaml`: 調整した値。`servo_joint_limits.yaml`: Servo 専用の速度上限）。`moveit_config/config/servo.yaml`（Setup Assistant 由来、「残すもの」）は触らず、そのまま。理由: (1) 実行時に使う設定を `controllers.yaml` と同じ場所（bringup）にまとめる、(2) Setup Assistant で開き直しても壊れない、(3) 差分が見やすい。欠点: `servo.yaml` が 2 つになる（`moveit_config` 側は使われない旨を冒頭に書く）。代案: `moveit_config/config/servo.yaml` を直接直す（1 つで済むが、Setup Assistant の保存で上書きされる危険、inventory の注意）。→ 8 章。

## 5. 試験の入力: 手で小さい Twist を送るツール

`tools/send_servo_twist.py`（新規。`send_joint_delta.py` と同じ流儀。**実機を動かす。ユーザーだけが実行する**）。

**動作**:
1. 引数: `--linear X Y Z`（0..1 の割合。`scale.linear` との積が実速度）、`--angular RX RY RZ`、`--seconds`（必須）、`--frame`（`base_link` か `gripper_frame_link` のみ。それ以外は拒否）、`--drift {yaw,roll,none…}`、`--rate`（既定 20 Hz）、`--yes`、`--dry-run`。
2. 拒否する条件: 成分の絶対値が 0.5 を超える（初期。`--allow-fast` で 1.0 まで）、時間が 3 秒を超える、**先端の総移動量**（入力 × `scale` × 時間）が 2 cm（回転は 0.1 rad）を超える、frame_id が許可外。
3. 事前の確認（どれかが失敗したら、何も送らない）: `/joint_states` が新しく NaN が無い、`/servo_node/status` が来ている、`forward_position_controller` が active で `arm_controller` が inactive（`controller_manager` の `list_controllers` サービス。X1 の場合）、`change_drift_dimensions` の呼び出しが成功。
4. **目標を表示して Enter を待つ**（総移動量、速度、時間、drift の設定、現在の先端の位置は TF から）。
5. 送信: `TwistStamped` を一定の周期で、`header.stamp = now()`。終了時に**ゼロの Twist を 0.3 秒**（Ctrl-C でも）。
6. 終了後に、先端の位置の変化（TF）を表示（期待値との比較）。

**安全の仕組み**: (a) 上の入力の上限、(b) 時間を区切る（3 秒）、(c) stamp は必ず `now()`、(d) 事前の確認、(e) 実機は `read_only:=false` でユーザーが起動した場合だけ意味がある（ツールは読み取り専用か書き込みかを区別しない。腕が動くかはドライバ次第）、(f) ツールは `start_servo`・`unpause_servo` を呼ぶが、**コントローラの切り替えはしない**（手順書でユーザーが行う）。

**案**（入力の形）:
- **T1（推奨）**: 上のとおり、1 回の実行で 1 つの Twist を一定時間（`send_joint_delta.py` と同じ形）。
- T2: 軌道（複数の Twist を順に）を YAML で。→ 当面は不要。
- T3: キーボード操作（押している間だけ）。→ 手が離れたら止まる利点はあるが、押しっぱなしの状態が残る・遅延がある。初期には不向き。

## 6. 確認の段階

### 6.1 段階

**S0. 静的な確認**（ノード・ポートを使わない。実装の各段階で行う）
- `xacro` の展開、`MoveItConfigsBuilder` の Python 実行、設定ファイルの解析、パラメータの検査（7 章の試験）、Jacobian の数値の確認（付録の再実行）。
- launch の組み立て（`use_servo` の有無で `LaunchDescription` に何が入るか。ノードは起動しない）。

**S1. モック**（`mock:=true use_servo:=true`。ハードウェアなし。ユーザーの実機が動いていないことを確認してから）
- Servo の起動（`kinematics` を渡さず逆 Jacobian になること、WARN `No kinematics solver…`）。パラメータの値の読み出し（`ros2 param get`）。
- `start_servo`・drift の設定。小さい Twist（ツール）→ 先端が TF で期待どおり動く（x, y, z、ロール・ピッチ）。**積算・閉ループ（モックは測定 = 指令）。JTC 案の停止を確認する場合は、`arm_controller` へ直接送って進まないことを確かめる（3.2 の推論の検証）**。
- 停止の確認（送信停止、ゼロ Twist、`pause_servo`、Servo の `kill -INT`）。入力が止まると `/servo_node/status` と出力が止まる。
- 特異点・関節の端・衝突（休止姿勢での `collision_velocity_scale`。0 なら Servo は動かない）。しきい値の調整。
- コントローラの切り替え（`switch_controllers`）と、MoveIt の実行（`arm_controller` 側）の往復。切り替え中に腕（モックの状態）が動かない。
- ログ: `run_duration` の WARN、intra-process の WARN（`servo_node.cpp` が出す。無視してよい出力に追加してよいか確認）。

**S2. 実機の読み取り専用**（ユーザーが `read_only:=true` で起動）
- 3.6 のとおり。計算の確認（出力 = 期待値）、状態コード、CPU・周期、`/joint_states` に NaN が無い。**サーボには書かない**。
- 実機の休止姿勢（elbow +1.6901）で、`status` の `JOINT_BOUND` の挙動（動く向きによって全関節が止まる、1.3）。

**S3. 実機の書き込み（ごく小さい動き）**（ユーザーが `read_only:=false` で起動。**私は実行しない**）
- 保持のみ → 1〜2 tick 級の単発 → 5 mm/s で 2 秒 → 各軸 → 停止の試験 → 特異点の近く → MoveIt との切り替え。

### 6.2 新しい手順書（実機の段階）の目次の案: `docs/servo_real_check.md`

`feetech_write_check.md` と同じ形（共通の約束、1 ステップずつ、合格基準、報告してほしいもの、ログの読み方）。

```
共通の約束（全段階）
  - 実行するのはユーザー。私は実機を動かすコマンドを実行しない
  - 起動したものを止めるのは、自分で起動した PID への kill -INT のみ（pkill・killall 禁止）
  - 終了後、ps と ros2 node list の両方で残りが無いこと
  - 1 つの段階の合格を確認してから次へ。ログは ~/so_arm101_check/ に
0. 準備（共通）: build、PID の控え、起動コマンド（use_servo:=true、read_only の別）、
   コントローラの状態確認、Servo の状態確認（ノード・サービス・パラメータの値）
R0. 読み取り専用で起動（Servo は未開始）: 何も出ていないこと
R1. start_servo と drift の設定（読み取り専用）: status、WARN の確認
R2. 小さい Twist を送る（読み取り専用・計算だけ）: 出力と期待値の照合、腕が動かないこと
R3. 停止の確認（読み取り専用）: 送信停止、ゼロ Twist、pause_servo、Servo の kill -INT
W0. 書き込みモードで起動し、Servo も開始。Twist なしで保持のみ 10 秒（トルク ON、腕が静止）
W1. 動き出す最小の速度（z 方向。入力 0.1 → 0.2 → 0.5、各 1 秒。量子化の不感帯の測定）
W2. z 方向 5 mm/s で 2 秒（上）→ 戻す（下）。先端の変位を目視と TF で
W3. x 方向、y 方向（ヨーが drift で変わること）、ロール・ピッチ
W4. 停止の試験（実機で）: 送信の停止、ツールの Ctrl-C、pause_servo、Servo の kill -INT。
    ドライバ・controller は止めない。止まる距離を記録
W5. 特異点・関節の端の近く（休止姿勢から）: status の確認、全関節が止まること
W6. MoveIt との切り替え: Servo → MoveIt（OMPL）の小さな移動 → Servo。
    切り替え中の保持、arm_controller が inactive の間の MoveIt の拒否
合格基準（段階ごと）／報告してほしいもの／ログの読み方／失敗したときの対応
実機でエラーを起こす試験はしない（通信断など。W5 の前の「通信断」は feetech_write_check.md で済み）
```

## 7. 試験（colcon test）の追加の案と件数の見込み

**置き場所**: `so_arm101_bringup/test/`（pytest。launch の組み立てと設定の静的な検査。ノードは起動しない）。`ament_lint` の対象に入るので、新しい .py は MIT ヘッダ・docstring・行長（E501）を満たす（`test_launch_arg_check.py` と同じ流儀）。

| 試験 | 内容 | 件数 |
|---|---|---|
| 引数（既存の変更） | `test_declared_arguments`: `len(DECLARED) == 22`、`use_servo` を含む。`test_every_declared_argument_is_accepted`: 引数が 1 つ増えて 1 件増える | 既存の 1 か所の変更、+1 |
| Servo の設定（`servo.yaml`）の検査 | (1) 全キーが Servo 2.5.10 の宣言（36 個を固定リストにする）にある、(2) `command_out_type` が Float64MultiArray のとき位置か速度の**どちらか一方**だけ true（`servo_parameters.cpp:422`）、(3) しきい値の順序（`lower < hard_stop`）・`publish_period > 0`・`command_in_type` の値、(4) **`command_out_topic` が `/arm_controller/joint_trajectory` でない**、(5) `scale` が初期値の上限以下（0.01 / 0.05 / 0.05）、(6) `incoming_command_timeout` が 0.1 以下 | 約 6 |
| Servo 専用 joint limits | 5 関節すべてに `max_velocity` があり、`has_velocity_limits: true`、URDF の速度以下 | 約 2 |
| 関節の順 | SRDF の group `arm` の関節の順 = `controllers.yaml` の `forward_position_controller.joints` = `arm_controller.joints`（`MoveItConfigsBuilder` と xacro で求める） | 約 2 |
| 排他 | `arm_controller` と `forward_position_controller` が同じ関節の position を要求している（排他が成り立つ前提）。`moveit_controllers.yaml` は `arm_controller` のまま（MoveIt は forward controller を使わない） | 約 2 |
| launch の組み立て（案 1 の場合） | `use_servo:=false` で Servo のノードが `LaunchDescription` に**入らない**。`true` で、(a) `servo_node_main` が 1 つ、(b) その parameters に `robot_description_kinematics` が**含まれない**、(c) `robot_description_planning`（Servo 専用）が含まれる、(d) forward controller の spawner が `--inactive` | 約 4 |
| 運動学の確認（numpy。付録のスクリプトを `tools/` に置いて関数を試験） | 選んだ drift の行を捨てた 5×5 が、休止・home・中ほどの姿勢で特異でない（条件数がしきい値の `hard_stop` 未満）、6×5 の最小特異値 > 0、5×5 の条件数の期待値の再現 | 約 3 |
| 送信ツールの純関数（`tools/send_servo_twist.py` の検査部分） | 上限の拒否（成分、時間、総移動量、frame）、`--dry-run` の出力 | 約 4〜5 |

**件数の見込み**: bringup は現在 47 件・失敗 0 → **約 47 + 1 + 6 + 2 + 2 + 2 + 4 + 3 + 4〜5 ＝ 約 71〜72 件**（案 3 の場合は launch の組み立ての試験が別の launch ファイル用になり、引数の変更は無い: 約 66 件）。hardware（105）・calibration（73）・description（2）・moveit_config（0）は変わらない（**ドライバのコードは変更しない**）。全体は 231 → 約 255 件。**既存の試験で変わるのは `len(DECLARED) == 21` の 1 か所だけ**。`DesignLimits.StepLimitCoversTheWorstCaseTrajectoryStep` は、`arm_controller` の `update_rate` を変えなければ影響しない（変える場合は 8 章）。

コミットの前に、関係するパッケージ（bringup、変更があれば hardware）の `colcon test` を実行し、失敗は報告してからコミットする（CLAUDE.md）。

## 8. 決めてほしいこと（項目・推奨）

| # | 項目 | 選択肢 | 推奨 |
|---|---|---|---|
| D1 | Servo の出力先と排他（3.1） | X1 forward controller＋切り替え ／ X2 JTC のまま（手順で排他）／ X3 中継ノード | **X1** |
| D2 | JTC のままにする場合の `arm_controller` `update_rate` | 25 のまま ／ 50 に上げる | X1 なら **25 のまま**（影響なし）。X2 なら 50 が必須（3.2） |
| D3 | 捨てる軸（drift、2 章） | A yaw ／ B y ／ C roll(x) ／ D 姿勢で切り替え | **A**。**最終位置への接近が真下（トップダウン）なら D** |
| D4 | 最終位置への接近の姿勢 | 水平（先端が前）／ 斜め ／ 真下 | **ユーザーの答えが必要**（D3 に影響） |
| D5 | 統合の方法（4 章） | 案 1 use_servo ／ 案 2 include ／ 案 3 別 launch | **案 1**（実機までは案 3 の形でも可。ユーザーの好み） |
| D6 | Servo の設定の置き場所 | bringup/config に Servo 専用 2 ファイル ／ moveit_config/config/servo.yaml を直接直す | **bringup/config**（moveit_config の Servo 一式は触らない） |
| D7 | Servo に `robot_description_kinematics` を渡さない（逆 Jacobian にする） | 渡さない ／ 渡す | **渡さない**（`kinematics.yaml` は変更しない。2.5） |
| D8 | 初期の速度の上限 | `scale` 0.01 m/s・0.05 rad/s・0.05 rad/s、Servo 専用の関節速度 0.3 rad/s | **この値**（3.5） |
| D9 | 特異点のしきい値の初期値 | 既定 17/30 ／ 50/100 ／ モックの結果で決める | **50/100 を初期値にし、モックで調整**（既定のままは不可） |
| D10 | 衝突チェック | `check_collisions: true` のまま ／ false | **true のまま**。休止姿勢で `collision_velocity_scale` が 0 になるなら、SRDF の ACM が絡むので**報告して相談**（SRDF は勝手に変えない） |
| D11 | 送信ツールの形（5 章） | T1 一定時間 ／ T2 YAML ／ T3 キーボード | **T1** |
| D12 | `servo.launch.py`（moveit_config）の扱い | 残す ／ `git rm`（案 1 の後） | **当面は残す**（「Servo 一式は残す」のユーザーの判断。削除する場合は別途、一覧と理由を示して承認を得る） |
| D13 | 実装の段階の分け | (1) 設定ファイル＋静的な試験 → (2) launch（`use_servo`）＋試験 → (3) 送信ツール → (4) モックの確認 → (5) 手順書 → (6) 実機（ユーザー） | **この順。各段階で colcon build・colcon test・commit** |
| D14 | `inventory.md` の「無視してよい出力」に追加してよいもの | Servo の `Intra-process communication is disabled` WARN、`No kinematics solver instantiated…` WARN（逆 Jacobian にしたとき） | 追加してよい（モックで確認した後） |

## 付録: Jacobian の条件数の計算（ノード不使用）

方法: `xacro`（`use_fake_hardware:=true`）で展開した URDF から、`base_link` → `gripper_frame_link` の chain の関節（origin の rpy・xyz、axis）を読み、numpy で幾何 Jacobian（先端の原点、base_link 基準、6×5）を計算。SVD の左零空間と、各行を捨てた 5×5 の条件数を求めた。`MoveIt` の `getJacobian(group)`（既定の参照点 = 先端の原点）と同じ定義 [推測。MoveIt の出力との照合はしていない]。

関節空間の一様サンプル（URDF の範囲から 0.1 rad の margin を除く、先端の z ≥ 0、約 3 万点）の条件数の分布（捨てた行ごと）:

| 捨てる行 | 5 / 25 / 50 / 75 / 95 % 点 | 17 未満 | 30 未満 | 100 未満 |
|---|---|---|---|---|
| y | 25 / 43 / 96 / 294 / 2613 | 0 % | 12 % | 51 % |
| yaw | 22 / 29 / 47 / 125 / 663 | 0 % | 28 % | 71 % |
| roll(x) | 25 / 48 / 121 / 391 / 4141 | 0 % | 11 % | 45 % |

作業範囲らしい領域（pan ±0.5、lift −1.0〜−0.2、elbow 0.3〜1.4、wrist_flex 0〜1.0、roll ±0.5。y を捨てた場合）: 5 / 50 / 95 / 99 % 点 = 22 / 25 / 44 / 116、30 未満 75 %。**これは暫定の分布で、実際の作業範囲が決まったら取り直す**。

スクリプトは作業用の一時ファイルに作った（リポジトリの外）。実装の段階で `tools/` に置き、7 章の試験の元にする。計算の核:

```python
# 各関節: 親からの origin(rpy, xyz) を掛け、revolute なら axis を世界系に直して
#   J[:3, i] = cross(axis_i, p_tip - p_i),  J[3:, i] = axis_i
# 5x5: np.delete(J, row, axis=0) の特異値の比 s_max / s_min
```

## 参照

- Servo 2.5.10（タグ）: `moveit_ros/moveit_servo/src/{servo_node,servo,servo_calcs,utilities,enforce_limits,collision_check,servo_parameters}.cpp`。行番号は本文のとおり（取得したファイルのもの）。
- JTC 2.54.1: `joint_trajectory_controller/src/{joint_trajectory_controller,trajectory}.cpp`。forward controller 2.54.1: `forward_command_controller/src/forward_controllers_base.cpp`。ros2_control 2.54.3: `controller_manager`（コントローラごとの `update_rate`）。
- このリポジトリ: `docs/inventory.md` 末尾の「将来の目標と IK の経緯、Servo 統合のときの記録、注意」、`docs/original_urdf_plan.md` 22.8、`docs/feetech_write_check.md`（手順書の形）、`tools/send_joint_delta.py`（送信ツールの形）、`src/so_arm101_hardware/src/sts3215_system.cpp`（ドライバの制限）、`src/so_arm101_bringup/config/controllers.yaml`。
