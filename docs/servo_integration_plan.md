# MoveIt Servo（Diff-IK）を bringup に統合する計画

2026-11-12。**調査と計画だけ。コード・launch・設定の変更、モック・実機の起動はしていない**（ファイルを読む、`xacro` の展開、`MoveItConfigsBuilder` の Python 実行、numpy による運動学の計算、公式ソースの取得だけ。ノードもポートも使っていない）。承認が出るまで実装しない。


## 範囲と前提（2 回目の依頼で追記。2026-11-12）

- **主な用途**: 地面に置いた物体を、**上から（真下に近い接近で）掴む**。
- **流れ**: 外部（AnyGrasp など）から把持の位置と姿勢（XYZ＋クォータニオン）を受ける → **ユーザーの共通の層**（座標の変換、5 関節で作れる姿勢への変換＝ラッパー、範囲のチェック、デッドマン・タイムアウト・飛びの検知、モードの切り替え。ROS ブリッジの中。**このリポジトリの外。今回は扱わず、変えない**）→ **解析 IK**（ユーザーのコード。同上）で大まかに移動 → 画像とのずれがあれば **Servo（Diff-IK）**で、近い姿勢のまま小さく補正。
- **VR**（手の姿勢を流す追従）は解析 IK だけで行い、**Servo は使わない**。VR は後の段階（13 章は案だけ）。
- **今回の計画の範囲**: Servo を bringup に統合し、共通の層が使う**入口**（コントローラの切り替え、Twist の入力、開始・停止、状態の通知。9 章）を作るまで。**実機の確認は「Python からの単発の目標に対する Servo の小さい補正」まで**（6.2）。
- 解析 IK・ラッパーはこのリポジトリに無いので、**変換の約束（Twist を作れる向きに射影する。2.3）は、共通の層への要求として書く**。

## 0. 要点（先に）

目的は、最終位置への移動で、カメラとのずれを残差で詰めること（近くでの追従に Servo を使う。大きな移動は MoveIt（OMPL）か自前の解析 IK）。カメラの統合は今回の対象外。用途は、地面の物体を**真下に近い接近**で掴むこと。

調べて分かった、**計画を左右する事実**（根拠は各章。[事実] は読んだコード・実行結果、[推測] はそこからの推論で未確認）:

1. **今の `servo.yaml` + `servo.launch.py` のままでは、動かない・または危険**。
   - (a) **5 関節に 6 次元の Twist を送ると、Servo 2.5.10 のコードが範囲外を読む**（`utilities.cpp:104`）。drift dimensions で 1 軸を捨てて Jacobian を 5×5 にする必要がある **[事実：コード]**。
   - (b) **drift は、IK プラグイン（今の `kinematics.yaml` の KDL）を使う経路では壊れている**。Servo のノードに `robot_description_kinematics` を渡さなければ逆 Jacobian の経路になり、drift が効く（`kinematics.yaml` は変更しない）**[事実：コード。起動での確認は未実施]**。
   - (c) **`start_servo` は必須**（`servo_node.cpp:111-114`）。
   - (d) **特異点のしきい値の既定（17 / 30）は、この腕に合わない**。**真下に近い把持姿勢**（地面の物体の典型）で条件数は 19〜37（腕を伸ばしきる近くで 71）。既定では減速か停止になる（2.3、3.5）。暫定の案は **40 / 100**。
   - (e) **JTC（`arm_controller`、25 Hz）に Servo の軌道を送ると、ほとんど進まない見込み**（3.2）**[推測]**。
2. **drift（2 回目の再計算の結果）**: 真下に近い把持姿勢で、**作れない回転は、shoulder_pan の作る鉛直面から接近方向を横に傾ける回転**で、その軸は `a(pan) = (cos pan, −sin pan, 0)`（base_link 基準）。**pan とともに回る**ので、base_link に固定した **yaw を捨てる案は真下では特異（条件数 10⁴ 以上）**で、**使えない**（前回の推奨を取り消す）。**捨てるのは rx か ry**。`|pan| ≤ 1.2 rad` なら **rx 固定**で条件数が最良に近く（ry は pan ≈ 0 で特異）、それ以上は ry。command frame は drift と無関係（drift は常に base_link の軸。2.3）。
3. **作れない向きの Twist は、共通の層で射影する**（`ω·a(pan) = 0` にする）。射影した Twist なら、rx か ry を捨てても**正確に追従**する（2.3）。
4. **出発点（elbow +1.690）からは Servo は動き出せない**。関節の端の判定は、内側への動きなら許す（コードで確認）が、**自己衝突までの距離が 3.9 mm（しきい値 10 mm 未満）で、Servo は全関節の動きを約 1.5 % に縮める**（距離を本物の MoveIt のコードで計算。1.6）。**MoveIt（OMPL）で中間姿勢へ出てから Servo に切り替える**運用が必須。
5. **JTC と forward controller の切り替えで腕は飛ばない**見込み（根拠と運用の条件は 3.1.1）。**forward controller は指令が来るまで何も書かない**。JTC は止めるとき・起動するときに**直前の指令をそのまま保つ**。
6. **JTC は、トピックとアクションを区別せず、あとから来たものに置き換える**（排他が無い）。**Servo の出力は別のコントローラ（forward 系）へ**出し、ros2_control の排他で切り替える（D1）。
7. **速度の守り**: Servo 専用の関節速度の上限は、`robot_description_planning.joint_limits.<関節>.max_velocity` を Servo のノードに渡すと、`enforce_limits.cpp` が強制する（3.3.1）。**ただし forward controller の経路では JTC の制限もドライバの上限（0.48 rad/周期）も守りにならない**。**下の層に小さい 1 周期の変化量の上限を置く案**を 3.3.2 に書いた（VR の連続出力にも使える形）。
8. **地面（床）はプランニングシーンに無い**（Servo の衝突チェックは床を見ない。12 章）。
9. **残差を詰める精度には限界がある**: 約 0.023 rad/s 以下は動かない見込みに加え、**Servo は「測定位置＋小さな差」を目標にし積分しない**ので、**開く向きで約 0.04 rad 手前で止まる摩擦**を超えられない見込み（11 章）。

決めてほしいことは 8 章の表（D1〜D24）。

## 1. 現状の把握

### 1.1 何を読んだか（版）

- インストール版: `ros-humble-moveit-servo` **2.5.10-1jammy.20261006.192111**（`dpkg -l`）。**この .deb にはソース（.cpp）もパラメータの説明も入っていない**（`/opt/ros/humble/include/moveit_servo/*.h` と `share/moveit_servo/{config,launch}` だけ）。
- 動作はソースで確認した。ソースは公式 `moveit/moveit2` の**タグ `2.5.10`**（`moveit_ros/moveit_servo/src/` の `servo_calcs.cpp`、`servo_node.cpp`、`servo.cpp`、`utilities.cpp`、`enforce_limits.cpp`、`collision_check.cpp`、`servo_parameters.cpp`。`moveit_ros/planning/robot_model_loader`、`kinematics_plugin_loader`。`moveit_core/online_signal_smoothing/src/butterworth_filter.cpp`）を `raw.githubusercontent.com` から取得して読んだ。**インストール版の .deb がこのタグから作られたことは、バージョン番号の一致だけで、バイナリとの照合はしていない [推測]**。取得したファイルは作業用の一時ディレクトリ（リポジトリの外）にあり、リポジトリには入れていない。
- ヘッダ（`servo_calcs.h`、`servo_parameters.h`、`servo_node.h`）はインストール版を直接読んだ。ソースと矛盾しない（メンバ・パラメータの名前と既定値が一致）。
- JTC と forward controller は、インストール版 `joint_trajectory_controller` 2.54.1 のタグ（`ros-controls/ros2_controllers` 2.54.1）、`controller_manager` 2.54.3 のタグ（`ros2_control`）のソースを同様に読んだ（版は `package.xml` のもの）。

2 回目の調査では、上に加えて次を読んだ・実行した: `pose_tracking.cpp`、`robot_model_loader.cpp`、`kinematics_plugin_loader.cpp`、`joint_model.cpp`（`moveit_core`）、`forward_controllers_base.cpp`、`joint_trajectory_controller.cpp` の `on_activate`／`on_deactivate`（いずれも該当タグ）。**本物の MoveIt のコード（`moveit_core` の `RobotModel`・`PlanningScene`）を使う単体の C++ プログラム**（ノードを使わない。作業用の一時ディレクトリで build・実行。リポジトリには入れていない）で、自己衝突までの距離と、`arm` グループの関節の順を求めた。

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

### 1.6 出発点（elbow +1.690）からの開始

出発点の実測（新しい校正）: pan −0.0982、lift −1.48295、**elbow +1.69009**、wrist_flex +0.50327、roll −0.04833（inventory）。elbow が URDF の上限（1.69）とほぼ同じで、実測が **+1.707（範囲外）**になることもある。モデルの衝突は +1.74 から（+1.70〜+1.73 は有効）。Servo の 2 つの判定を、コードと計算で確かめた。

**(1) 関節の上限の判定は、内側への動きなら許す [事実：コード]**
- `enforcePositionLimits`（`servo_calcs.cpp:835-850`）は、「`joint_limit_margin`（0.1）を引いた範囲の外にある」関節のうち、**範囲の外へ向かう速度の関節だけ**を止める対象にする（`velocity < 0` かつ下限側、`velocity > 0` かつ上限側）。出発点の elbow（+1.690、+1.707 とも、1.59 を超える）は範囲の外だが、**減らす向き（内側）の速度なら対象にならない**。
- **増やす向きの関節があると、全関節が止まる**（`halt_all_joints_in_cartesian_mode: true`、`suddenHalt`。位置を元に戻し速度 0）。出発点から、**逆運動学の解が elbow を増やす向きの Twist は、全体が止まる**。
- なお、状態が範囲外でも、Servo の状態の読み取りは範囲で切らない（`CurrentStateMonitor` は範囲を強制しない）**[推測。コードは読んでいない]**。

**(2) 衝突チェックが、出発点からの動き出しを止める [事実：計算＋コード]**
- Servo の衝突チェックは、**向きを見ない**（距離だけ）。`collision_check.cpp:118-` 付近で、自己衝突までの距離が `self_collision_proximity_threshold`（0.01 m）より近いと、`velocity_scale = exp(k·(距離 − 0.01))`、`k = −ln(0.001)/0.01 = 690.8`。**この倍率は全関節の `delta_theta` に掛かる**（`internalServoUpdate`、`servo_calcs.cpp`）。接触なら 0。
- **距離は、本物の MoveIt のコード（`planning_scene` と `getCollisionEnvUnpadded()->checkSelfCollision`、`distance=true`、group `arm`、SRDF の ACM つき。ノードを使わない単体のプログラムで計算）で求めた**:

| elbow_flex（他は出発点） | 自己衝突までの距離 | Servo の倍率 |
|---|---|---|
| +1.49009 | 25.0 mm | 1.0 |
| +1.59009 | 13.9 mm | 1.0 |
| +1.65 | 7.7 mm | 0.20 |
| **+1.69009（出発点）** | **3.9 mm** | **0.015** |
| **+1.70697** | **2.4 mm** | **0.005** |
| +1.72 | 1.3 mm | 0.0003 |
| +1.73 | 0.5 mm | 0.0000 台 |
| +1.74 | 接触（upper_arm_link–wrist_link） | 0（`HALT_FOR_COLLISION`） |

- 出発点で、Servo の 1 周期の変化（小さな Twist で約 0.0017 rad）に 0.015 を掛けると 2.5×10⁻⁵ rad で、**サーボの 0.5 tick（0.00077 rad）より桁違いに小さく、動かない**。**内側（開く向き）への動きでも同じ**（距離しか見ない）。
- したがって、**Servo は出発点から動き出せない**。**MoveIt（OMPL）で elbow を開いた中間姿勢（距離 10 mm 以上。この姿勢では elbow が約 1.62 以下）まで出てから、Servo に切り替える**。これは計画の流れ（OMPL で中間姿勢へ → Servo）と一致する。`self_collision_proximity_threshold` を下げれば出発点からも動けるが（0.003 なら出発点は倍率 1）、衝突の手前の余裕が減るので**推奨しない**（D20）。
- 真下に近い把持姿勢では、**自己衝突までの距離は 20 mm 以上**（3.5 の表）で、減速は起きない。
- 環境（床・物体）の距離は、シーンに何も無いので **DBL_MAX（= 倍率 1）**（12 章）。

## 2. 5 関節への対処: drift dimensions

### 2.1 なぜ必要か

- 6 次元の Twist に対し、関節は 5 つ。Jacobian は 6×5。Servo の特異点の計算は、`svd.matrixU().col(num_dimensions - 1)`（`utilities.cpp:104`）で、`num_dimensions` は Twist の次元（6）。thin SVD の U は 6×5 なので、**6 列目（添字 5）は範囲外** [事実：コード]。release ビルドでは Eigen の境界検査が無いので、未定義の値を読む。
- drift で 1 軸を捨てると、reduced Jacobian が 5×5 になり、`col(4)` で範囲内になる。**1 軸だけ捨てるのが必須**（2 軸以上捨てても動くが、制御できる軸が減る）。

### 2.2 どの軸が制御できないか（計算した。1 回目。水平寄りの姿勢が中心。真下の接近は 2.3）

URDF（`so101_new_calib`、`xacro` で展開）から、base_link 基準・gripper_frame_link の原点の Jacobian（6×5）を numpy で計算した（付録のスクリプト。ノード不使用）。左零空間（「どの向きの Twist が、どの関節でも作れないか」）:

| 姿勢 | 作れない Twist の向き（x, y, z, rx, ry, rz） | 捨てる行ごとの条件数（reduced 5×5） |
|---|---|---|
| center（全関節 0、先端が水平に前） | ≈ (0, −0.94, 0, 0, 0, +0.33) | **y: 29.6、yaw: 29.6**、roll(x): 240、x・z・pitch: 悪い（10⁶〜） |
| home（SRDF の `home`） | ≈ (0, −0.97, 0, 0.17, 0, 0.15) | y・roll・yaw: 71.8（同じ） |
| 実機の休止姿勢（inventory の記録の位置） | ≈ (0.09, −0.97, 0, 0.13, 0.01, 0.18) | y・yaw・roll: 約 124 |
| 作業範囲の中ほど（`mid`、`reach`） | y 並進が主 | y・yaw・roll・pitch: 22〜41 |
| initial（wrist_flex 1.57、**先端が真下**） | **≈ (0, 0, 0, 1.0, 0, 0)**（x 軸まわりの回転） | **roll(x): 29**、y: 3085、yaw: 2183 |

意味 [推測を含む解釈]: パン（z 軸まわり）が「横（y）方向の並進」と「ヨー（z まわりの回転）」を**一緒に**作る。先端が水平なら、横に動くことと向きを回すことが同じ関節で結ばれていて、独立には指令できない。**先端が真下を向く姿勢では、その代わりに「先端を横に傾ける（x まわりの回転）」が作れない**（パンの軸と wrist_roll の軸が平行になる）。

### 2.3 真下に近い把持姿勢での再計算と、drift・command frame の推奨（2 回目の調査）

**方法**: URDF から、地面の物体を上から掴む典型の姿勢を解いた。接近軸（`gripper_frame_link` の z 軸）が真下（傾き 2 mm 相当以内）、先端の高さ z = 0.01〜0.06 m（`base_link` 基準）、shoulder_pan の軸からの水平距離 r = 0.03〜0.32 m、pan = 0〜±1.5。関節は URDF の範囲から 0.1 rad の margin を除く（Servo が止まる範囲）。各姿勢で Jacobian（6×5、base_link 基準、先端の原点）を計算し、左零空間（作れない Twist の向き）と、各行を捨てた 5×5 の条件数を求めた（付録）。

**結果 1: 作れない向きは、純粋な回転で、軸が pan とともに回る** [事実：計算]
- 左零空間の並進成分は 0（< 0.001）、回転成分は **`a(pan) = (cos pan, −sin pan, 0)`** まわりの回転（pan = 0.5 で (0.88, −0.48, 0)、pan = 1.0 で (0.54, −0.84, 0)。符号は SVD の任意性で入れ替わる）。意味は、**接近方向を、pan の作る鉛直面から横に傾ける回転**（ユーザーの見立てどおり）。r・z・lift・elbow・wrist_flex によらず同じ。
- 真下では、x・y・z・rz（yaw）のどの行を捨てても、**5×5 は特異**（条件数 5×10⁴〜無限大）。**rx か ry だけが使える**。

**結果 2: rx と ry の条件数（r = 0.14、z = 0.03 の姿勢。pan だけ変えた）** [事実：計算]

| pan [rad] | 0 | 0.3 | 0.6 | 0.79 | 1.0 | 1.2 | 1.4 | 1.5 |
|---|---|---|---|---|---|---|---|---|
| rx を捨てる | 21.7 | 20.8 | 18.1 | 17.7 | 18.1 | 19.0 | 24.2 | 45.0 |
| ry を捨てる | **468000** | 19.8 | 18.0 | 17.7 | 18.5 | 20.3 | 21.4 | 21.7 |

→ **rx は |pan| ≤ 1.2 rad（約 69°）で最良に近い**（pan ≈ 0 でも正常）。**ry は pan ≈ 0 で特異**。pan が 1.4 を超えたら ry のほうがよい。

**結果 3: command frame は drift に関係しない** [事実：コード]
- Servo は、Twist を **planning frame（base_link）に変換してから**（`servo_calcs.cpp:535-562`）、Jacobian（モデルのルート = `base_link`。`RobotModel` で確認）の行を捨てる（`removeDriftDimensions`、`:573`）。**捨てる行は、command frame（`base_link` でも `gripper_frame_link` でも）によらず、常に base_link の軸**。`gripper_frame_link` で Twist を送っても、drift は変わらない。`planning_frame` を別のリンクにしても、Jacobian の座標は変わらない（`getJacobian` はルートの座標）。
- 「pan とともに回る軸」を drift に使うことは、Servo ではできない。**固定の軸（rx か ry）を選ぶしかない**。

**結果 4: 作れない向きを共通の層で射影すれば、drift と矛盾しない** [事実：数学＋計算]
- 6×5 の Jacobian の値域は、`u·t = 0`（`u` = 左零空間の単位ベクトル）を満たす Twist `t` の集合。真下では `u = (0, 0, 0, a(pan))`。
- **共通の層が、Servo に送る Twist を値域に射影しておく**（`ω ← ω − (ω·a)a`、`a = (cos pan, −sin pan, 0)`、pan は `/joint_states` の現在値。一般の姿勢では、`u` を同じ URDF の Jacobian から求めて `t ← t − (u·t)u`）。
- すると、rx（または ry）を捨てた 5×5 の方程式は、**射影した Twist を厳密に再現する**（5 つの行を満たす解は一意で、その解が作る Twist は値域の中にあり、`u·t = 0` の条件と合わせて捨てた行も満たす）。**ラッパー（目標の姿勢を作れる姿勢にする）と同じ考え方**で、Twist 版の変換。
- 射影しない Twist（作れない向きの成分を含む）は、**捨てた行の誤差が無視される**（成分は実現されない）。安全上は、その成分が実現されないだけで、暴走はしない。ただし、**射影の有無を Servo は検知しない**ので、共通の層の責任（9 章に約束事として書く）。

**案と推奨**

| 案 | 捨てる軸 | 利点 | 欠点 |
|---|---|---|---|
| **A（推奨）** | **rx 固定**。`\|pan\| > 1.2` のときだけ **ry** に切り替える（停止中に `change_drift_dimensions`。ヒステリシス ±0.05 rad） | 作業範囲の大半（正面の ±69°）で切り替えなし。条件数が最良に近い | 範囲外の物体では切り替えが要る。切り替え中の Twist の扱いを決める必要 |
| B | rx 固定のみ。\|pan\| > 1.2 では Servo を使わない（解析 IK だけ） | 最も単純 | 横の物体で補正できない |
| C | pan に応じて毎周期 rx／ry を切り替える | 常に最良 | 切り替えの回数が増える。drift の変更は Servo の動作中の呼び出しになり、安全上よくない |
| D | yaw を捨てる（前回の案 A） | 水平の接近向き | **真下では特異（条件数 10⁴ 以上）。使えない** |

**推奨: A**。どの案でも、**command frame は `base_link`**（共通の層が base_link の Twist を作る。Servo の外のカメラ座標は共通の層で変換）。

**作業範囲の目安**（上の姿勢の探索）: 真下の接近で届くのは、pan の軸から水平に **r ≈ 0.07〜0.26 m**、高さ z = 0.01〜0.06 m。**r ≤ 0.05 m**（先端が pan の軸の真上に近い）は、lift・elbow が URDF の範囲（margin を除く）に収まらず**解けなかった**。**r ≥ 0.28 m** も解けなかった。**z = 0.10 m** は解けなかった（範囲外か、私の解法の失敗かは未判別）。

### 2.4 drift の設定の仕方（安全に）

- 設定は**サービス**のみ（パラメータなし）。Servo の起動直後（`start_servo` の前）に呼ぶ。**共通の層（または送信ツール）が、毎回、`change_drift_dimensions` を呼び、成功を確認してから Twist を送る**。切り替え（rx ↔ ry）は、**腕が止まっていて Twist を送っていないとき**だけ（モード管理ノードを置く場合は、そこで行う。9 章）。
- Servo 側から「drift が未設定」を検知する方法は無い [事実：パラメータにもステータスにも無い]。**ほかのクライアントが生の Twist を送ると、未設定のまま 6 次元で処理される恐れがある（2.1）**。共通の層と将来のブリッジは、同じ門を通すこと。

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

X1 で確認が要る点（**3.1.1・3.1.2 に、2 回目の調査の結果を書いた**。以下は 1 回目の記述）**[推測。モックで確認]**:
- Servo が出す Float64MultiArray の関節の順は、`arm` グループの active joint の順（`servo_calcs.cpp:148`）で、**forward controller の `joints` の順と一致させる**必要がある（`forward_controllers_base.cpp:131-142` は要素数しか見ない。順は検査されない）。SRDF の chain（`base_link` → `gripper_frame_link`）の順は pan・lift・elbow・wrist_flex・wrist_roll のはずで、`controllers.yaml` の `joints` と同じ。**静的な試験で、この一致を確認する**（7 章）。
- forward controller は、有効化した直後は、最初のメッセージが来るまで何も書かない（`forward_controllers_base.cpp:106、123-129`）。JTC を止めた後、コマンドは直前の値（ドライバの `hw_commands_`）のまま保持される。切り替えで腕が動かないことを、モックの `/joint_states` と、実機の W 段階で確認する。
- ドライバの `hw_commands_` は、**切り替えの間も最後の値を保つ**（`write_goal_positions` は毎周期 `hw_commands_` を送る）。

### 3.1.1 JTC ⇔ forward controller の切り替えの瞬間（D1 の前提。2 回目の調査）

**forward_command_controller**（インストール版 `ros-humble-forward-command-controller` の `package.xml` は 2.54.1。読んだのは `ros-controls/ros2_controllers` のタグ 2.54.1 の `forward_command_controller/src/forward_controllers_base.cpp`）:
- `on_activate`（`:86-108`）: 指令の置き場を空にする（`rt_command_ptr_ = …(nullptr)`、`:106`）。**無効の間に来た指令は捨てる**。ハードウェアの command には**何も書かない**。
- `update`（`:120-142`）: **指令がまだ来ていないとき（置き場が空）は、何も書かずに `OK` を返す**（`:123-129`）。指令が来たら、その値を command interface に書く（`set_value`）。要素数が合わないと `ERROR`（`:131-138`。順序は見ない）。
- したがって、**有効にした直後〜最初の Servo の指令が来るまで、command には何も書かれない**。

**JTC**（インストール版 `joint_trajectory_controller` 2.54.1、同タグの `joint_trajectory_controller.cpp`）:
- `on_deactivate`（`:1055-1100`）: **position の command に、その command の今の値を書き戻す**（`:1071-1075`。値は変わらない）。NaN は書かない。
- `on_activate`（`:953-1030`）: `set_last_command_interface_value_as_state_on_activation`（既定 true。パラメータの構造体、`joint_trajectory_controller_parameters.hpp:23`）のとき、**command interface の今の値（NaN でなければ）を始点にして**（`:1008-1013`）、**その位置を保持する軌道**（`set_hold_position`）から始める（`:1028-1029`）。**測定値ではなく、直前の指令を保持する**。

**ドライバ**（`sts3215_system.cpp`、書き込みモード）が command を受けたときの振る舞い:
- 起動時: `hw_commands_ = 現在位置`（`:835`）。`write()` は毎周期 `write_goal_positions` を呼ぶ（`:524`）。
- **NaN・inf**: 破棄して**直前の目標を保持**し、「異常」と数える（`:966-967`）。**50 周期（1 秒）続くとフォルト**（`anomaly_cycles`、`:989-995` 付近）で、送信を止める（トルクは ON のまま保持）。
- **変化なし**: 差 0 なので、**同じ目標を毎周期書き続ける**（`:963-1003`）。
- **直前の JTC の値**: そのまま（変化なし、または 1 周期の上限 0.479 rad 以内の差ならそのまま通る。`:973`）。

**切り替えの瞬間に腕が飛ばない根拠 [事実：コード。実機・モックの動作は未確認]**:
1. JTC → forward: JTC を止めても、command は**直前の値のまま**（ドライバは同じ目標を送り続ける）。forward controller は**指令が来るまで何も書かない**。腕は保持される。
2. 最初の Servo の指令は「**測定位置＋小さな差**」。直前の JTC の指令との差は、**測定位置と JTC の指令の差**（open_loop の JTC は測定を見ないので、開く向きの摩擦で止まったままの関節は、**指令が測定より最大約 0.04〜0.05 rad 先**にある。inventory）。つまり、**目標が約 0.04 rad「測定に戻る」**。腕は目標に近い（動く必要がない）ので、ほとんど動かないが、サーボには「戻す向きの目標」が行く（重力で下がる向きなら、わずかに動く可能性。**[推測]**）。この差は、ドライバの上限（0.479 rad）より小さい。**下の層に上限（3.3.2）があれば、その上限で緩やかに移る**。
3. forward → JTC: JTC の `on_activate` は**直前の指令を保持**する（上）ので、飛ばない。以後の MoveIt の軌道は、`open_loop_control` のため、**この指令（= Servo の最後の目標）から**始まる。MoveIt の軌道の最初の点と実測の差が `allowed_start_tolerance`（既定 0.01 rad）を超えると拒否されうる。**Servo の後は測定と指令がほぼ同じなので問題になりにくいが、止まっている位置の差を確認してから実行する** **[推測]**。

**運用の条件**:
- **切り替えは、腕が止まっているときだけ**: 実測位置の変化が 0.5 秒間で 2 tick 以内（関節速度は有限差分で 1 tick/周期 ≒ 0.077 rad/s のノイズがあるので、速度では判定しない）、アクションの目標が無い（MoveIt の実行が終わっている。`/arm_controller/follow_joint_trajectory/_action/status` の最後の状態が終了）、Servo は `pause_servo` 済み。
- **切り替え中は Servo を一時停止**し（`pause_servo`）、**切り替え完了を確認してから `unpause_servo`**。forward controller が有効の間に Servo が出力を続けていると、有効化の直後に最初の指令が届く。
- **1 回の `switch_controller` で、停止と開始を同時に行う（`--strict`）**。2 回に分けると、どちらも無効の間が生じる（保持はされるが、誰も指令できない）。**同時の切り替えが、同じ関節の command を要求する 2 つのコントローラでも通ること**は、モックで確認する **[推測。`controller_manager.cpp` のこの経路は読んでいない]**。
- 起動時は、`arm_controller` を有効、forward controller を `--inactive`（読み込みだけ）にする。
- 切り替えの直後、**最初の Servo の指令の前後で、`/joint_states` の変化が 2 tick 以内**であることを、モックと実機の手順書で確認する。

### 3.1.2 関節の順番の一致の保証

- **Servo の `Float64MultiArray` の順**は、グループ `arm` の active joint の順（`servo_calcs.cpp:148`、`getActiveJointModelNames()`）。**本物の MoveIt のコード（`RobotModel`、同じ URDF・SRDF）で求めた順**: **shoulder_pan、shoulder_lift、elbow_flex、wrist_flex、wrist_roll**（モデルのルートは `base_link`）[事実：計算]。
- forward controller（またはその派生）の `joints` は、**同じ順で書く**。コントローラは**要素数しか検査しない**（順は検査されない。`forward_controllers_base.cpp:131-138`）ので、**保証は次の 3 つで行う**: (1) **静的な試験**: `RobotModel` で求めた順 = `controllers.yaml` の `forward…joints` = `arm_controller.joints`（7 章。MoveIt の C++ が要るので、試験は URDF・SRDF の chain から決まる順を Python で求めて、上の事実（C++ の結果）と一致させる）、(2) **モックでの検証**: Servo の `~/delta_joint_cmds`（名前つきの `JointJog`）で 1 関節だけ動かし、**動いた関節が名前どおり**であること（`/joint_states`）、(3) 実機の手順書の最初の動き（R 段階の出力の確認で、1 関節ずつ）。

### 3.2 JTC に Servo の軌道を送った場合の動き（D1 が X1 なら踏まない）（X1 を採らない場合の注意。open_loop_control: true の下）

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
- Servo 専用の速度上限（3.3.1、3.5）を入れれば、1 周期は 0.01 rad 程度（0.3 rad/s のとき）で、ドライバの上限の約 1/50。
- **サーボの量子化 [推測]**: tick は 0.001534 rad。Servo の差が 0.5 tick（0.00077 rad）未満だと、丸めで元の tick に戻る。`publish_period` 0.034 s なら約 **0.023 rad/s** 以下の関節速度は動かない。先端で約 5 mm/s（腕の長さ 0.2 m、Jacobian の大きさ 約 5 rad/m として約 0.025 rad/s）。**初期の試験の速度の下限**（0.05 rad/s 程度）と、残差補正の精度の下限（約 1 tick ≒ 先端で 0.3 mm）に関わる。実機の W 段階で、動き出す速度を測る。

### 3.3.1 「Servo 専用の関節速度 0.3 rad/s」を強制するコード・パラメータ（2 回目の調査）

**事実（Servo 2.5.10、`moveit_servo`、タグ 2.5.10）**:
1. 速度の上限は、Servo のロボットモデルの**関節の境界**（`variable_bounds_`。`velocity_bounded_`、`max_velocity_`、`min_velocity_ = −max_velocity_`）。ノードの起動時に、`RobotModelLoader` が**ノードのパラメータ** `robot_description_planning.joint_limits.<関節>.has_velocity_limits`（`robot_model_loader.cpp:164`）と `….max_velocity`（`:193`）を読んで、URDF の値を上書きする（`setVariableBounds`、`:241`、`joint_model.cpp:148-152`）。
2. 強制するのは `enforceVelocityLimits`（`enforce_limits.cpp:70-100`）。`servo_calcs.cpp:706` から、**Cartesian でも関節の指令でも**、平滑化の後に呼ばれる。各関節の速度（= (平滑化後の位置 − 測定位置) ÷ `publish_period`、`:755-760`）を `[min_velocity_, max_velocity_]` に収める倍率を求め（`getVelocityScalingFactor`、`:50-65`）、**倍率 < 1 なら全関節の速度と位置の差を縮める**（`:84-`）。倍率が 1 でなければ、位置も `positions −= (1 − 倍率) × 速度 × publish_period` で戻す。
3. **`override_velocity_scaling_factor`（既定 0.0）が 0.01 以上だと、この計算が無効になり、その値が倍率になる**（`enforce_limits.cpp:80-81`）。**0.0 のままにする**（試験で固定。7 章）。
4. 加速度の上限は、Servo は**使わない**（Butterworth の平滑化が効くだけ）。

**4d の「joint_limits を読まない」との関係**: 同じ仕組みで、**`servo.launch.py` が `robot_description_planning` をノードに渡していないので、MoveIt の `joint_limits.yaml`（5.0 rad/s）も読まれず、URDF の 10 rad/s が使われる**（事実は変わらない）。**逆に、`robot_description_planning` を Servo のノードにだけ渡せば、Servo 専用の速度上限を入れられる**。渡すのは別ファイル（Servo 専用）で、**MoveIt の `joint_limits.yaml`・URDF は変えない**（`move_group` は引き続き `joint_limits.yaml`）。`default_velocity_scaling_factor` などは Servo は使わない。**起動での確認（ログに上限が反映されること、速度を超える Twist が縮められること）はモックで行う**。

### 3.3.2 下の層に置く 1 周期の変化量の上限（案。hardware の変更になるので案と利点・欠点だけ）

**問題**: forward controller の経路には、JTC の速度の制限（MoveIt の `joint_limits.yaml`。軌道の計画で守られる）が無く、ドライバの上限（6.0 rad/s × 0.08 s = **0.479 rad/周期**。`sts3215_system.cpp:832`）は**大きすぎて守りにならない**（Servo の URDF 速度 10 rad/s の最大 0.34 rad/周期より大きい）。**共通の層・変換・解析 IK（VR の連続出力を含む）の不具合で、目標が大きく飛んでも、速く動かないための最後の守り**を、下の層に置きたい。

| 案 | 内容 | 利点 | 欠点 |
|---|---|---|---|
| **L1** ドライバの上限を下げる（設定だけ） | `max_joint_speed`・`limit_window_s` はドライバの hardware パラメータ（`sts3215_system.cpp:63-64`）で、xacro の `<param>` で渡せる（今は渡しておらず既定）。例: `max_joint_speed` 0.75 rad/s × `limit_window_s` 0.08 s = **0.06 rad/周期** | **コード変更なし**。全経路（JTC・forward・VR）を守る。ドライバの既存の仕組み（超えたら削る＋ WARN。`:973-`） | **MoveIt の実行が削られる**: `joint_limits.yaml` の max_velocity（5.0）× スケーリングの JTC の 1 更新（25 Hz）の変化が上限を超えると、open_loop のため**気づかずに遅れる**（inventory 631-641）。ドライバの試験 `DesignLimits.StepLimitCoversTheWorstCaseTrajectoryStep` は定数（6.0 × 0.08）を使うので、xacro の値を変えても試験は通るが、**設計の整合が崩れる**。整合させるには `joint_limits.yaml`（MoveIt 本体の設定）の max_velocity を下げる必要がある（**勝手には変えない**） |
| **L2** ドライバを変える（コード） | 上限を、動作の種類（MoveIt／streaming）で切り替える（例: 追加の command interface やサービスで「厳しい上限」を有効にする） | 1 か所で全経路を守る | **ドライバのコード変更**（hardware の試験・実機の再確認が要る）。切り替えの状態を持つので、**状態の食い違いが新しい危険**になる。過剰 |
| **L3（推奨）** 制限つきの forward controller（仮称 `limited_position_controller`。forward_command_controller を基にした小さなコントローラ） | Float64MultiArray を受けて、**毎周期、直前に書いた値からの変化を ±`max_step`（例: 0.02 rad/周期 ≒ 1 rad/s。Servo の初期は 0.01）に制限**して command に書く。有効化時は、**今の command（または測定位置）から始める**（飛ばない）。指令が一定時間（`cmd_timeout`）来なくなったら、現在の目標で保持（更新しない）。目標が `max_jump`（例: 0.15 rad）を超えて飛んだら、**受け付けずにフォルト**（再有効化まで）。NaN・サイズ違いは破棄して保持 | **Servo と VR の連続出力（streaming）の両方を同じ守りで包む**。**MoveIt の経路（JTC）に影響しない**。パラメータで調整でき、**単体で試験できる**（ドライバ・実機なしで gtest）。切り替えの瞬間も緩やかに移る（3.1.1）。ドライバの上限は、その下の最後のバックストップとして残る | **新しい C++ のコントローラ（パッケージ 1 つ）**。ros2_control の plugin 登録・試験が要る。この守りが壊れると、守りが無くなる（試験で固める）。JTC の経路（MoveIt）は守らない（MoveIt の計画の速度上限が守る） |
| L4 守りなし（Servo 専用の速度上限だけ） | — | 何も増えない | Servo 以外（VR）と、Servo の外側の不具合を守れない。**forward controller は、受けた値をそのまま command に書く** |

**推奨: L3。ただし最初の実機の確認（Servo の小さい補正）は、L1（ドライバ 0.06 rad/周期）を併用してもよい**（実機の初期の守りを二重にする。MoveIt の実行は、スケーリング 0.1 なら 0.5 rad/s で、1 更新 0.02 rad、2 更新分 0.04 rad < 0.06 なので削られない見込み **[推測。inventory 636 の計算の流用]**。ただし `joint_limits.yaml` と整合しない状態になるので、設定は一時的に）。→ D15。

**VR で同じ守りを使える形**: 解析 IK の連続出力は、**`limited_position_controller` の同じトピックに出す**（streaming）。上限（`max_step`）は VR 用に上げられる（パラメータ）。JTC には送らない（13 章）。

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
| 特異点 | `lower_singularity_threshold` / `hard_stop_singularity_threshold` | **40 / 100**（既定 17 / 30） | 真下に近い把持姿勢（r = 0.07〜0.22 m）の条件数 20〜37 で減速なし、伸ばしきり近く（66〜71）で約 0.57 倍、100 以上で停止（3.5.1）。**暫定。モックで調整** |
| 関節の端 | `joint_limit_margin` | 0.1（現状どおり） | URDF の limit の 0.1 rad 手前（ドライバの校正範囲より内側） |
| 実機の最初の試験の送信側の上限 | ツールの引数 | 入力 0.5 以下、時間 3 秒以下、先端の総移動量 2 cm 以下 | `send_joint_delta.py` と同じ流儀（超える指定は拒否） |

### 3.5.1 真下に近い把持姿勢での条件数としきい値の再提案（2 回目の調査）

前回は関節空間の一様サンプルで見たが、**今回は、地面の物体を上から掴む典型の姿勢**で計算した（接近軸が真下、z = 0.01〜0.06 m、pan = 0.4。rx を捨てた 5×5。`s5` は 6×5 の最小特異値）。`self` は自己衝突までの距離（1.6 と同じ方法）。

| r [m] | z [m] | lift / elbow / wrist_flex [rad] | 条件数（rx を捨てる） | self [mm] |
|---|---|---|---|---|
| 0.07 | 0.01 | −1.06 / 1.30 / 1.34 | 34 | 20.5 |
| 0.10 | 0.01 | −0.65 / 1.10 / 1.12 | 26 | 50.5 |
| 0.14 | 0.01 | −0.26 / 0.79 / 1.04 | 20 | 90.5 |
| 0.18 | 0.01 | +0.07 / 0.41 / 1.09 | 22 | 110 |
| 0.22 | 0.01 | +0.39 / −0.05 / 1.23 | 28 | 112 |
| **0.26** | 0.01 | +0.84 / −0.79 / 1.52 | **66**（腕を伸ばしきる近く） | 109 |
| 0.07 | 0.03 | −1.10 / 1.16 / 1.51 | 29 | 22.3 |
| 0.10〜0.18 | 0.03 | — | 20〜23 | 51〜111 |
| 0.22 | 0.03 | +0.35 / −0.13 / 1.36 | 30 | 112 |
| 0.10〜0.18 | 0.06 | — | 21〜24 | 63〜112 |

pan を変えても（0〜±1.0）、条件数は同じ程度（18〜28。r = 0.25 の伸ばしきりで 48〜57）。**先端が pan の軸の真上に近い（r ≤ 0.05）姿勢は、関節の範囲（margin を除く）に収まらず解けなかった**。r = 0.07 の近くで s5 が小さい（0.047〜0.055）ので、**この近くは条件数が上がる**（32〜37）。

**しきい値の案**:
- 既定の 17 / 30 では、r = 0.07〜0.22 の**ほぼ全域（条件数 20〜34）が減速（17 超）か停止（30 超）**になる。**使えない**。
- **案: `lower_singularity_threshold: 40`、`hard_stop_singularity_threshold: 100`**（`leaving_singularity_threshold_multiplier: 2.0` のまま = 離れる向きは 180 まで）。r ≤ 0.22 の全域（条件数 ≤ 37）は**減速なし**。伸ばしきり近く（66〜71）は倍率 `1 − (66−40)/(100−40) ≈ 0.57`。100 以上（さらに伸ばしきる）で停止。
- 注意: この条件数は**メートルとラジアンが混ざった値**で、腕が小さい（リンク約 0.1 m）ので大きく出る。**値そのものに物理的な意味は薄く、「ここまで来たら止める」の目安**。**暫定。モックで `status` と `collision_velocity_scale` を見て決める**。

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

**統合で追加する部品（2 回目の依頼を受けた全体像）**: (1) bringup: `use_servo`、Servo ノード（逆 Jacobian・Servo 専用の joint limits）、`servo.yaml`（Servo 専用）、`servo_joint_limits.yaml`、forward 系のコントローラの定義と spawner（`--inactive`）。(2) **モード管理ノード**（案 M2。9.2）。(3) **下の層の上限**（案 L3: 制限つき forward controller。3.3.2）。(4) 送信ツール（5 章）。(5) 試験（7 章）。(6) 手順書（6.2）。**共通の層・解析 IK・ラッパー（リポジトリの外）には触れない**。

## 5. 試験の入力: 手で小さい Twist を送るツール

`tools/send_servo_twist.py`（新規。`send_joint_delta.py` と同じ流儀。**実機を動かす。ユーザーだけが実行する**）。

**動作**:
1. 引数: `--linear X Y Z`（0..1 の割合。`scale.linear` との積が実速度）、`--angular RX RY RZ`、`--seconds`（必須）、`--frame`（`base_link` か `gripper_frame_link` のみ。それ以外は拒否）、`--drift {rx,ry,auto}`（既定 auto = pan から選ぶ）、`--rate`（既定 20 Hz）、`--yes`、`--dry-run`。
2a. **Twist の射影**: 現在の関節角（`/joint_states`）から、同じ URDF の Jacobian を求め、**Twist を値域に射影してから送る**（`t ← t − (u·t)u`。真下では `ω·a(pan) = 0`。2.3）。射影で捨てた成分の大きさを表示し、**大きいとき（例: 入力の 50 % 以上）は拒否**する。drift（rx／ry）は pan から選ぶ（2.3 の案 A）。
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

`feetech_write_check.md` と同じ形（共通の約束、1 ステップずつ、合格基準、報告してほしいもの、ログの読み方）。**実機の確認は「Python からの単発の目標に対する Servo の小さい補正」まで**。VR は目次だけ。

**流れ**: 読み取り専用で確認 → 書き込みモードで MoveIt（OMPL）を使って中間姿勢へ → 止まっていることを確かめて Servo に切り替え → Python から単発の小さい補正 → Servo を止めて JTC に戻す → MoveIt で出発点へ。

```
0. 共通の約束・準備（全段階）
   - 実行するのはユーザー。私は実機を動かすコマンドを実行しない
   - 起動したものを止めるのは、自分で起動した PID への kill -INT のみ（pkill・killall 禁止）
   - 終了後、ps と ros2 node list の両方で残りが無いこと
   - 1 つの段階の合格を確認してから次へ。ログは ~/so_arm101_check/ に
   - 準備: build、PID の控え、起動コマンド（use_servo:=true、read_only の別）、
     コントローラの状態（arm_controller 有効、forward 系は inactive）、Servo の状態（ノード・サービス・パラメータ値）
   - 【止め方の表（全段階で共通。上から順に試す）】
       S1 Twist の送信を止める（ツールの Ctrl-C = ゼロの Twist）。0.1 秒後に Servo の出力が止まる
       S2 pause_servo（Servo の一時停止。以後、出力しない）
       S3 to_trajectory（JTC に戻す。切り替え後は保持。forward 系は無効）／手動なら
          ros2 control switch_controllers --strict --deactivate <forward系> --activate arm_controller
       S4 Servo のノード（または launch）に kill -INT（自分で起動した PID のみ）。forward 系は最後の目標を保つ
       S5 ハードウェアの launch に kill -INT（トルク OFF。腕が自重で落ちる。折りたたみ姿勢か、手で支えてから）
       S6 12 V を切る（最後の手段。トルクが切れて腕が落ちる。下の条件のときだけ）
   - 【12 V を切る条件】 次のどれかで、S1〜S4 が効かないとき:
       (a) 停止の操作の後も、腕が 1 秒以上動き続ける、または床・自分の体・物に向かって動いている
       (b) 異音（ガリガリ、うなり）、焦げた匂い、煙、サーボの異常な発熱（触れない熱さ）
       (c) ドライバが COMMUNICATION LOST を出し、腕が保持せずに動く
       (d) 手が届かない、または S5 の操作ができない状況
     切る前に、腕の下から手と物をどける。切ると、腕は落ちる（姿勢によっては床に当たる）。**事前に、落ちる姿勢の下に障害物が無いことを確認**
R0. 読み取り専用で起動（use_servo:=true。Servo は未開始）: 何も出ていないこと、forward 系が inactive
    （腕は動かない）。起動時の WARN の一覧（Intra-process、逆 Jacobian の WARN。無視してよい出力に追加）
R1. start_servo・drift の設定（読み取り専用）: 出力（forward 系のトピック）と期待値の照合
    （出発点では、collision_velocity_scale ≈ 0.015 で、動かないことを確認）、status、関節の順の確認
R2. 小さい Twist を送る（読み取り専用・計算だけ）: 出力 = 期待値（射影した Twist）。腕が動かないこと
R3. 停止の確認（読み取り専用）: S1、S2、S4 のそれぞれで出力が止まること
W0. 書き込みモードで起動（JTC 有効、Servo は一時停止）。保持のみ 10 秒（腕が静止）
W1. MoveIt（RViz）で出発点から中間姿勢へ（OMPL。Pilz は使わない。出発点の elbow は上限を超えるため）。
    中間姿勢 = 自己衝突までの距離が 10 mm 以上の姿勢（elbow を開いた姿勢）で、真下の接近の作業姿勢の近く
    （feetech_write_check.md の W3.5・W4 の既存の手順を参照）
W2. 止まっていることを確認（位置の変化が 0.5 秒で 2 tick 以内、MoveIt の実行が終了）→ to_servo（切り替え）。
    切り替えの前後で /joint_states の変化が 2 tick 以内（飛ばない）、mode が servo
W3. Python から単発の小さい補正（tools/send_servo_twist.py）: 最初は、重力に従う向き（z の下げ）で 3〜5 mm、
    次に開く向き（上げ）で 5 mm。動き出す最小の速度と、方向ごとの止まり位置を記録（11 章）
W4. 停止の試験（実機）: 送信停止、ツールの Ctrl-C、pause_servo、Servo の kill -INT。それぞれで止まり、止まる距離を記録。
    ドライバ・forward 系は止めない
W5. Servo を止めて JTC に戻す: pause_servo → to_trajectory（保持。飛ばない）。mode が trajectory
W6. MoveIt で出発点へ戻す（開く向きの手前止まりと、出発点の elbow の上限超えを考慮。閉じる向きの端の判定は
    feetech_write_check.md の手順どおり）
合格基準（段階ごと）／報告してほしいもの／ログの読み方／失敗したときの対応
実機でエラーを起こす試験はしない
--- 後の段階（目次のみ。今回は実機で行わない）---
V. VR（解析 IK の連続出力。Servo は使わない）
   V0. streaming モード（stream）への切り替えと保持
   V1. 小さな振幅の手の動き（デッドマンあり、入力のタイムアウト、目標の飛びの検知）
   V2. 下の層の上限（max_step）が効くことの確認（わざと大きく飛ばした目標を、共通の層を通さずに送る試験は
       しない。モックで）
   V3. 停止の確認（デッドマンを離す、入力を止める、ノードを kill -INT する）
```

**止め方の表の根拠**: S1・S2 は Servo の stale・pause（3.4）、S3 は ros2_control の排他と保持（3.1.1）、S4・S5・S6 は CLAUDE.md の規則（自分で起動したものを PID で止める。実機の launch を止めるとトルクが切れて腕が落ちる）。

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
| drift の規則・射影（純関数。真下の姿勢の表の値を再現） | pan から rx／ry を選ぶ規則（\|pan\| ≤ 1.2 で rx、ヒステリシス）、射影後の Twist が値域に入る（`u·t = 0`）、rx（または ry）を捨てた 5×5 が特異でない（真下の姿勢 10 点で条件数 < 100）、**`override_velocity_scaling_factor` が 0**、特異点のしきい値の順序と値（40 / 100） | 約 5 |
| 入口の名前（9.1） | 入力・サービス・状態のトピック名と型、座標系（`base_link`）を定数で固定し、設定（`servo.yaml` の `cartesian_command_in_topic` など）と一致 | 約 3 |
| モード管理（案 M2。モックのサービスで） | 止まっていないと切り替えを拒否、失敗で元のモードに戻る、`pause_servo` が切り替えの前、`to_servo` で drift を設定、モードの通知 | 約 6 |
| 制限つき forward controller（案 L3。gtest。実機・ドライバなし） | 1 周期の上限、有効化時に今の command から始める、飛びの検知（フォルト）、`cmd_timeout`、NaN・サイズ違い | 約 6〜8（新しいパッケージ） |
| 送信ツールの純関数（`tools/send_servo_twist.py` の検査部分） | 上限の拒否（成分、時間、総移動量、frame）、`--dry-run` の出力 | 約 4〜5 |

**件数の見込み（2 回目で改訂）**: bringup は現在 47 件・失敗 0 → **約 47 + 1 + 6 + 2 + 2 + 2 + 4 + 3 + 5 + 3 + 6 + 4〜5 ＝ 約 85〜86 件**（案 3 の場合は引数の変更が無く約 80 件）。制限つき forward controller（D15 が L3 のとき）の新しいパッケージに約 6〜8 件。hardware（105）・calibration（73）・description（2）・moveit_config（0）は変わらない（**ドライバのコードは変更しない**。D15 が L2 のときだけ変わる）。全体は 231 → 約 330 件前後。**既存の試験で変わるのは `len(DECLARED) == 21` の 1 か所だけ**。`DesignLimits.StepLimitCoversTheWorstCaseTrajectoryStep` は、`arm_controller` の `update_rate` と `joint_limits.yaml` を変えなければ影響しない（D15 が L1 のときは設計の整合が崩れる。3.3.2）。

コミットの前に、関係するパッケージ（bringup、変更があれば hardware）の `colcon test` を実行し、失敗は報告してからコミットする（CLAUDE.md）。

## 8. 決めてほしいこと（項目・選択肢・推奨・理由）

2 回目の調査で、D3・D4・D9 の内容が変わった。D15〜D24 を追加した。

| # | 項目 | 選択肢 | 推奨 | 理由（1 行） |
|---|---|---|---|---|
| D1 | Servo の出力先と排他（3.1） | X1 forward 系＋ ros2_control の切り替え／X2 JTC のまま（手順で排他）／X3 中継ノード | **X1** | JTC には排他が無く、forward 系なら ros2_control の排他で担保でき、JTC の周期の問題（3.2）も消える |
| D2 | `arm_controller` の `update_rate` | 25 のまま／50 に上げる | **25 のまま**（X1 のとき） | X1 なら Servo は JTC を通らず、変更の影響（ドライバの試験・MoveIt）を避けられる |
| D3 | 捨てる軸（drift。2.3） | **A rx 固定（\|pan\| > 1.2 で ry）**／B rx のみ／C 毎周期切り替え／D yaw（前回の案。**真下で特異のため取り消し**） | **A** | 真下の把持姿勢で、作れない回転の軸が `(cos pan, −sin pan, 0)` で、rx は \|pan\| ≤ 1.2 で条件数が最良に近い |
| D4 | 最終位置への接近の姿勢 | — | **回答済み: 真下に近い** | D3・D9 に反映した |
| D5 | 統合の方法（4 章） | 案 1 `use_servo`／案 2 include／案 3 別 launch | **案 1**（実機までは案 3 の形でも可） | 同じ robot_description と 1 回の launch。既定 false で今と同じ |
| D6 | Servo の設定の置き場所 | bringup/config に Servo 専用 2 ファイル／moveit_config の `servo.yaml` を直す | **bringup/config** | Servo 一式（moveit_config）は残すユーザーの判断。Setup Assistant の上書きを避ける |
| D7 | Servo に `robot_description_kinematics` を渡さない | 渡さない／渡す | **渡さない** | 渡すと IK プラグインの経路で drift が壊れる（`kinematics.yaml` は変更しない） |
| D8 | 初期の速度の上限 | `scale` 0.01 m/s・0.05 rad/s・0.05 rad/s、Servo 専用の関節速度 0.3 rad/s | **この値** | 量子化の不感帯（約 0.023 rad/s）より上で、最初の実機の試験に十分遅い |
| D9 | 特異点のしきい値 | 既定 17/30／**40/100**／モックで決める | **40/100 を初期値にし、モックで調整** | 真下の把持姿勢の条件数が 19〜37（伸ばしきりで 66〜71）で、既定では減速・停止になる（3.5.1） |
| D10 | 衝突チェック | `check_collisions: true` のまま／false | **true のまま** | 真下の把持姿勢では自己衝突まで 20 mm 以上で、減速は起きない（出発点は別。D20・D23） |
| D11 | 送信ツールの形 | T1 一定時間／T2 YAML／T3 キーボード | **T1** | 目標の表示と Enter、時間の区切りで、`send_joint_delta.py` と同じ安全の形 |
| D12 | `servo.launch.py`（moveit_config）の扱い | 残す／`git rm` | **当面は残す** | 「Servo 一式は残す」のユーザーの判断。削除は別途承認 |
| D13 | 実装の段階 | (1) 設定＋静的な試験 (2) launch＋試験 (3) 下の層（D15）(4) モード管理（D17）(5) 送信ツール (6) モック (7) 手順書 (8) 実機（ユーザー） | **この順** | 各段階で colcon build・test・commit。下の層とモード管理を実機の前に入れる |
| D14 | 「無視してよい出力」への追加 | Servo の `Intra-process` WARN、`No kinematics solver…`（逆 Jacobian の WARN） | **モックで確認した後に追加** | 想定内の出力を区別するため |
| **D15** | 下の層の 1 周期の変化量の上限（3.3.2） | L1 ドライバの設定（`max_joint_speed`・`limit_window_s`）／L2 ドライバのコード／**L3 制限つき forward controller**／L4 なし | **L3（最初の実機は L1 の併用も可）** | MoveIt に影響せず、Servo と VR の連続出力を同じ守りで包み、単体で試験できる |
| **D16** | Twist を作れる向きに射影する責任 | 共通の層（ユーザー）／Servo の前に Servo 側のフィルタ | **共通の層**（要求として書く）。送信ツールは射影を実装 | 共通の層にラッパーがあり、Servo は射影の有無を検知できない（2.3） |
| **D17** | モード切り替えの入口（9.2） | M1 共通の層が直接呼ぶ／**M2 モード管理ノード** | **M2** | 「止まっているときだけ」「Servo を止めてから」の順序を 1 か所で守る |
| **D18** | 床（12 章） | F0 なし／F1 シーンに床／**F2 共通の層で z の下限** | **F2。F1 は床の高さが決まってから** | Servo の衝突チェックは床を見ない。F1 は把持の直前に減速する副作用がある |
| **D19** | Twist の単位 | `unitless`（[−1,1]×`scale`）／`speed_units`（m/s、rad/s） | **`unitless`（初期）** | 1 を超える成分を Servo が捨てる、内蔵の上限になる。後で `speed_units` に変えられる |
| **D20** | 自己衝突のしきい値 `self_collision_proximity_threshold` | 0.01（既定）／0.003 | **0.01 のまま** | 出発点からは Servo を使わない運用で足りる。下げると衝突の手前の余裕が減る |
| **D21** | VR の連続出力の送り先（13 章） | JTC／**forward 系（streaming）** | **forward 系** | 速度の守り・飛びの検知・MoveIt との排他が下の層にある |
| **D22** | pose tracking（10 章） | 使う／**使わない（Twist）** | **使わない** | 自前の C++ ノードが要り、作れない向きで収束せず、command frame を先端にする必要がある |
| **D23** | 出発点からの Servo の使用 | 出発点から使う／**MoveIt で中間姿勢へ出てから使う** | **中間姿勢から** | 出発点は自己衝突までの距離が 3.9 mm で、Servo は動かない（1.6） |
| **D24** | 関節の順番の保証（3.1.2） | 静的な試験＋モックの JointJog 検証／ランタイムの検査 | **静的な試験＋モックの検証＋実機の最初の 1 関節** | コントローラは要素数しか検査しない。順は MoveIt の `RobotModel` で確認済み |

## 9. 入口の約束事（共通の層が使う入口。2 回目の依頼）

作るものの範囲: Servo の統合と、下の表の入口。**共通の層・解析 IK・ラッパーは、このリポジトリの外のユーザーのコードで、変えない**。

### 9.1 入口の表

| 目的 | 名前 | 型 | 約束 |
|---|---|---|---|
| **Twist の入力** | `/servo_node/delta_twist_cmds` | `geometry_msgs/msg/TwistStamped` | **座標系は `base_link`**（`header.frame_id = "base_link"`。空でも `base_link` として扱う）。`gripper_frame_link` も可。**それ以外（カメラなど、モデルに無いフレーム名）は送らない**（Servo のスレッドが落ちる見込み。1.5）。**`header.stamp = now()`**（0 や過去は動かない。未来は stale にならない）。**10 Hz 以上**（`incoming_command_timeout` 0.1 s）、推奨 30 Hz。**単位**: `command_in_type: unitless` で [−1, 1]（1 を超える成分は捨てられる）。実速度 = 値 × `scale`（初期: 並進 0.01 m/s、回転 0.05 rad/s）。**作れない向きは射影してから送る**（2.3。`ω·a(pan) = 0`）。**速度で送りたいなら `speed_units`（D19）** |
| 関節の指令（予備） | `/servo_node/delta_joint_cmds` | `control_msgs/msg/JointJog` | 使わない（モックの関節の順の検証にだけ使う） |
| **Servo の開始・停止** | `/servo_node/start_servo`、`pause_servo`、`unpause_servo`、`stop_servo`（= `pause_servo`） | `std_srvs/srv/Trigger` | **`start_servo` は 1 回必須**（起動しただけでは何もしない）。**`success` は信用しない**（タイムアウトでも true）。`pause_servo` 中は何も出力しない |
| drift の設定 | `/servo_node/change_drift_dimensions` | `moveit_msgs/srv/ChangeDriftDimensions` | 2.4。rx（または ry）だけ true。`start_servo` の前、または止まっているとき |
| 状態の通知（Servo） | `/servo_node/status` | `std_msgs/msg/Int8` | Servo の周期（約 29 Hz）で出る（開始後）。**0 NO_WARNING、1 DECELERATE_FOR_APPROACHING_SINGULARITY、2 HALT_FOR_SINGULARITY、3 DECELERATE_FOR_COLLISION、4 HALT_FOR_COLLISION、5 JOINT_BOUND、6 DECELERATE_FOR_LEAVING_SINGULARITY、−1 INVALID**（`status_codes.h`） |
| 衝突の倍率 | `/servo_node/collision_velocity_scale` | `std_msgs/msg/Float64` | 10 Hz。0〜1 |
| 状態リセット | `/servo_node/reset_servo_status` | `std_srvs/srv/Empty` | 衝突などの状態コードを戻す |
| **コントローラの切り替え** | `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | `strict`。`arm_controller`（JTC）⇔ forward 系。**止まっているときだけ**（3.1.1） |
| **モードの指示（案 M2）** | `/so_arm101/mode/to_servo`、`/so_arm101/mode/to_trajectory` | `std_srvs/srv/Trigger` | 下の 9.2 |
| **モードの通知（案 M2）** | `/so_arm101/mode` | `std_msgs/msg/String`（transient_local） | `trajectory`（JTC が有効。MoveIt・解析 IK の軌道）／`servo`（forward 系が有効・Servo が動作中）／`stream`（forward 系が有効・Servo は一時停止。VR 用。後の段階）／`switching`／`fault` |
| 出力（Servo → コントローラ） | `/limited_position_controller/commands`（案 L3。D1・D15 で決める。L3 でなければ `/forward_position_controller/commands`） | `std_msgs/msg/Float64MultiArray` | 5 要素 [rad]。順は pan、lift、elbow、wrist_flex、wrist_roll。**共通の層は直接出さない**（Servo の出力。streaming の間は VR の出力もここ。13 章） |
| 姿勢の入力 | **提供しない** | — | **Twist だけ**（10 章。共通の層が、画像のずれから Twist を作る） |
| gripper | `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | 今のまま。この計画で変えない |

### 9.2 モードの切り替えを、共通の層から指示する方法（案）

| 案 | 内容 | 利点 | 欠点 |
|---|---|---|---|
| M1 | 共通の層が、`switch_controller`、`pause_servo`・`unpause_servo`、`change_drift_dimensions` を**自分で順番に呼ぶ** | 新しいノードが要らない | **「止まっているときだけ切り替える」「Servo を止めてから切り替える」を共通の層が守る**（ユーザーのコードに安全の責任）。手順の食い違いで、切り替え中に Servo が出力しうる。**手順書の人間の操作も同じ順番を覚える必要** |
| **M2（推奨）** | bringup に小さな**モード管理ノード**（Python。`so_arm101_mode`）。`to_servo`: (1) 実測位置が 0.5 秒で 2 tick 以内、(2) アクションの目標が無い、(3) `pause_servo`、(4) `switch_controller`（strict）、(5) pan で rx／ry を選んで `change_drift_dimensions`、(6) `unpause_servo`、(7) `/so_arm101/mode` を `servo` に。どこかで失敗したら**元のモードに戻し**、`Trigger` の `success=false`＋理由。`to_trajectory`: (1) Twist の停止（Servo を `pause_servo`）、(2) 止まっていることを確認、(3) 切り替え、(4) `trajectory` | **安全の順序を 1 か所に置く**（共通の層・手順書・ツールが同じ入口を使う）。試験できる（モックのサービスで）。`/so_arm101/mode` で状態を通知 | 新しい Python のノード（bringup に 1 つ）と試験が増える |

**推奨: M2**。**モード管理は、コントローラの切り替えと Servo の開始・停止の「止まっているときだけ」の条件を守るだけ**で、IK・座標変換・Twist の作成はしない（共通の層）。

### 9.3 Servo の開始・停止の流れ

1. `use_servo:=true` で起動（Servo は起動するが**何も出さない**）→ 2. （M2）`to_servo` が `start_servo` を（初回だけ）呼び、drift を設定し、`unpause_servo` → 3. 共通の層が Twist を送る → 4. 止めるときは、Twist の送信を止める（stale で 0.1 秒後に出力が止まる。ゼロの Twist を数回送ってもよい）→ `to_trajectory`（`pause_servo` ＋ JTC に戻す）。
- **Servo のノードを止める**（`kill -INT`、自分で起動した PID のみ）のは、異常時と終了時。止めたあとも、forward 系は最後の指令を保つ（腕は止まる）。

## 10. 入力の方式: Twist と pose tracking（2 回目の調査）

**Servo 2.5.10 に pose tracking はある**（`pose_tracking.h`、`src/pose_tracking.cpp`、`config/pose_tracking_settings.yaml`、実行ファイル `servo_pose_tracking_demo`。インストール版にも入っている）。**読んだ結果 [事実：コード]**:
- **ノードではなく C++ のクラス**（`moveit_servo::PoseTracking`）。**自分のプロセスの中で `Servo` を作って開始する**（`pose_tracking.cpp:96-97`）ので、`servo_node_main` と一緒には使えない。**使うには、自前の C++ ノードが要る**（`servo_pose_tracking_demo` のような）。
- 入力は、`target_pose`（`geometry_msgs/PoseStamped`。ノードの相対名。`:100-101`）。planning frame に変換される（`:261-270`）。**目標が 0.1 秒以内に更新され続けないと中止**（`NO_RECENT_TARGET_POSE`）。出力は、Servo の `cartesian_command_in_topic` への Twist（`:105-106`）。
- **収束の判定**は `moveToPose(positional_tolerance, angular_tolerance, timeout)` の `satisfiesPoseTolerance`（`:241-253`）。位置（各軸）と**向きの角度誤差**（`q_desired · q_current⁻¹` の回転角、**3 軸すべて**）が許容内になるまで、**ループを回し続ける**。**収束の時間切れは無い**（`target_pose_timeout` は目標の更新の時間切れで、収束の時間切れではない）。
- **位置は「command frame」の姿勢と比べる**（`command_frame_transform_`、`:121、244-246`）。**command frame は `robot_link_command_frame`**で、今の `base_link` のままでは**先端と比べない**（恒等変換）。**先端（`gripper_frame_link`）にする必要がある**。
- **PID は位置と向きの P 制御**（`pose_tracking_settings.yaml`: 位置 P 1.5、向き P 0.5、I・D は 0、windup 0.05）。**速度の上限は無い**（Servo の入力の `unitless` の「1 を超える成分は捨てる」と `scale` が上限になる）。

**比較**:

| | Twist（共通の層が作る） | pose tracking（Servo の機能） |
|---|---|---|
| 実装 | 追加の C++ なし。共通の層（または送信ツール）が、画像のずれから P 制御で Twist を作る | **自前の C++ ノード**が要る（Servo をそのノードの中で動かす。`servo_node_main` を使わない。bringup の起動・停止・状態の入口の約束が変わる） |
| 作れない姿勢の目標 | 共通の層が射影するので**問題なし**。射影しないとき、作れない成分は drift で捨てられ、**他の成分は動く**（収束判定は共通の層の側にある） | **向きの角度誤差が 0 にならず、収束しない**（時間切れが無いので回り続ける）。**ラッパーを通した目標（作れる姿勢）なら、収束する** |
| 収束の判定 | 共通の層が、画像のずれで判定 | 内蔵（位置・角度の許容） |
| 速度の上限 | `scale`＋Servo 専用の関節速度上限 | 同じ（P ゲイン次第で大きな Twist が出る） |
| 安全の設計 | Twist の stale（0.1 秒）、共通の層のデッドマン | 目標の stale（0.1 秒）。ループは別スレッドで動き続ける |

**推奨: Twist**（共通の層が P 制御）。理由: 追加の C++ が要らない、画像のずれによる制御は共通の層にある、作れない目標でも暴走しにくい、安全の入口が単純。**pose tracking は使わない**（D22）。

## 11. 目的に対する限界: 残差をどこまで詰められそうか（2 回目の調査。推測は明記）

材料: (a) **サーボの量子化**: 1 tick = 2π/4095 = 0.001534 rad。(b) **Servo は「測定位置＋小さな差」を目標にし、積分しない**（`internal_joint_state_ = original_joint_state_`、`servo_calcs.cpp`）。(c) **開く向き（自重に逆らう向き）で、目標の約 0.04 rad（約 27 tick）手前で止まる**（inventory 1069。W3・W3.5。閉じる向きは約 3 tick、pan は約 3 tick）。トルク OFF でも手を離して止まっているほど摩擦が大きい（ユーザーの観察）。

**推論 [推測。未確認]**:
1. **最小の動き（量子化）**: 目標 = 測定 + delta は、整数の tick に丸められる（ドライバの `rad_to_raw`。`std::lround`、`calibration.cpp:152-155`）。**delta が 0.5 tick（0.00077 rad）より小さいと、元の tick に戻って動かない**。`publish_period` 0.034 s なら、約 **0.023 rad/s 以下の関節速度は動かない**（Cartesian では、先端で約 3〜5 mm/s 以下。腕の長さ 0.1〜0.25 m）。
2. **摩擦（開く向き）**: Servo の目標は、**常に測定位置の近く**（delta は 1 周期で 0.002〜0.01 rad。Servo 専用の上限 0.3 rad/s なら最大 0.01 rad）。**約 0.04 rad の手前止まりの範囲より小さいので、開く向きの補正は、Servo の目標を何周期出し続けても、関節が動かない**見込み。**目標が積分されない（測定位置から離れない）ため、「押し続けて摩擦を超える」ことができない**。JTC の軌道の場合は、目標が遠い（積分される）ので、0.04 手前まで動く。
3. **閉じる向き**（重力の向き）と pan・roll は、約 3 tick（0.005 rad）の遅れ。**先端で約 0.5〜1 mm**（r = 0.1〜0.2 m の lever arm）。

**寄せられる見込み（先端。推測）**:

| 条件 | 見込み |
|---|---|
| 重力に従う向き・pan・roll の補正 | 約 **0.3〜1 mm**（1〜3 tick）。最小の速度（約 3〜5 mm/s 以上で送る）を守れば |
| **重力に逆らう向き（lift・elbow・wrist_flex の開く向き）の補正** | **関節で約 0.04 rad 未満の補正は届かない**見込み。先端で、lever arm 0.1〜0.25 m として **約 4〜10 mm**。**この方向の残差は Servo だけでは詰められない** |
| 全体 | 地面の物体を上から掴むとき、**補正の方向（z の上げ下げ、近づく・遠ざかる）によって**、届く精度が違う。**画像のずれが 4〜10 mm を下回る補正は、開く向きでは動かない**ことがある |

**関連する対策の方向（今は不要。記録）**: (1) 補正を**閉じる向き（重力に従う向き）から入る**ように、いったん開いて戻す（バックラッシュと摩擦の方向をそろえる）、(2) 目標を**積分する形**（共通の層が累積した目標を送る。ただし Servo は積分しない）、(3) サーボの内部の設定（P ゲイン、不感帯のレジスタ。**未調査**）、(4) 最後の数 mm は把持の公差（グリッパの開き）で吸収。**モック（量子化も摩擦もない）では確認できない。実機の W 段階で、動き出す最小の速度と、方向ごとの止まり位置を測る**。

## 12. 床（プランニングシーン。2 回目の調査。対策は案だけ）

**事実**:
- **プランニングシーンに床も物体も無い**。リポジトリに、シーンを作る設定・launch・コード（collision object、floor、ground、scene ファイル）は無い（`grep` で確認。`warehouse_db` は既定で無効）。
- **Servo の衝突チェックは、床を見ない**。本物の MoveIt のコードで、物体の無いシーンの「ロボットと環境の距離」を求めたら **DBL_MAX**（倍率 1。1.6）。**自己衝突だけ**を見る。
- OMPL の計画も、床を知らない。

**案**:
| 案 | 内容 | 利点 | 欠点 |
|---|---|---|---|
| F0 | 何もしない（床を知らない） | 変更なし | **先端が床にぶつかる向きの Twist でも Servo は止めない**。目視と、共通の層の範囲チェック（z の下限）が頼り |
| F1 | 起動時に、床（`shape_msgs/Plane` か薄い箱、`base_link` の z = 床の高さ）を、`move_group` のシーンに入れる（`/apply_planning_scene`。launch か小さなノード） | Servo・OMPL とも床を見る | 床の高さ（`base_link` の原点と地面の関係。**未確認**）を決める必要。**把持の直前、指が床に近づいて Servo が減速する**（`scene_collision_proximity_threshold` 0.02 m 以内で指数的に減速。0.005 などに下げる必要）。物体も床に触れる |
| F2 | 共通の層で、z の下限（床 + 余裕）を、**目標と Twist の両方**にかける | 変更がブリッジの中だけ。Servo の減速の副作用が無い | 共通の層の責任。Servo 単体では守らない |

**推奨: F2（共通の層の z の下限）を先に。F1 は、床の高さが決まってから検討**（D18）。今回は実装しない。

## 13. VR（後の段階）の安全の置き場所（案だけ。VR は解析 IK だけで、Servo は使わない）

| 層 | 守り | 置き場所 |
|---|---|---|
| **共通の層**（ユーザー） | **デッドマンスイッチ**（離したら出力停止）、**入力のタイムアウト**（手の姿勢が来なくなったら止める）、**目標の飛びの検知**（目標が大きく飛んだら**追わずに止めて**、「つかみ直し」を求める。再開は人の操作）、座標の範囲チェック、変換（作れる姿勢へ）。**フィルタは使いながら、変換側で調整する** | 共通の層（変更しない） |
| **下の層（コントローラ）** | **1 周期の変化量の上限**（`max_step`）、**指令のタイムアウト**（`cmd_timeout`。来なくなったら更新せず保持）、**飛びの検知**（`max_jump`。超えたら受け付けずフォルト）、NaN・サイズ違いの破棄。**Servo と共通**（3.3.2 の L3） | `limited_position_controller`（案 L3） |
| ドライバ | 0.479 rad/周期の上限、校正範囲のクランプ、NaN の破棄、異常の連続でフォルト（今のまま。最後のバックストップ） | 既存 |

**解析 IK の連続出力を、JTC と forward 系のどちらに送るのが安全か（D1 の排他の設計と合わせて）**:

| | JTC（`arm_controller`。トピックの軌道） | **forward 系（`limited_position_controller`）** |
|---|---|---|
| 補間 | スプライン。**`header.stamp = 0`のメッセージは、周期が JTC の更新より短いと進まない**（3.2）。**`stamp` を入れれば進む** | 補間しない（目標をそのまま、上限つきで書く） |
| 速度の守り | **無い**（計画の時間とスケーリング頼み。`joint_limits.yaml` は軌道を作る側が守るもの） | **上限（`max_step`）・タイムアウト・飛びの検知** |
| MoveIt との排他 | **無い**（あとから来たものに置き換わる。3.1） | **ros2_control の排他**（同じ関節を同時に握れない） |
| 途切れたとき | 最後の軌道の終点を保持 | 最後の目標を保持（上限つき） |
| 向く用途 | 時間つきの軌道（MoveIt の OMPL、解析 IK の大きな移動の一発） | **連続の目標（VR の追従、Servo）** |

**推奨: 連続出力（VR）は forward 系（streaming）**。理由: (1) 速度の守りと飛びの検知が下の層にある、(2) MoveIt との排他が ros2_control で担保される、(3) Servo と同じモード・同じ守り。**JTC は、時間つきの軌道（大きな移動の一発）に使う**。**注意**: streaming の間、Servo と VR が**同じトピックに出す**ので、`stream` モード（Servo は一時停止）と `servo` モードを、モード管理（M2）が排他にする（9.2）。

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

**2 回目の調査で追加した計算**（作業用の一時ファイル。リポジトリの外。実装の段階で `tools/` に置く）:
- 真下の把持姿勢の探索: 接近軸（`gripper_frame_link` の z 軸。center で +x、initial で −z）が真下になるよう、(pan, r, z) を与えて lift・elbow・wrist_flex を Gauss-Newton で解き（範囲は URDF の limit から 0.1 rad の margin を除く。許容: r・z で 2 mm、傾き 2 mm 相当）、Jacobian の左零空間と各行を捨てた条件数を求める。
- 自己衝突の距離: `moveit_core` の `RobotModel`・`PlanningScene` で、Servo と同じ呼び出し（`getCollisionEnvUnpadded()->checkSelfCollision`、`CollisionRequest{group_name="arm", distance=true}`、SRDF の ACM）。ノードを使わない単体の C++。環境（`checkRobotCollision`）の距離は DBL_MAX（シーンが空）。
- `arm` グループの active joint の順と、モデルのルート（`base_link`）。

## 参照

- Servo 2.5.10（タグ）: `moveit_ros/moveit_servo/src/{servo_node,servo,servo_calcs,utilities,enforce_limits,collision_check,servo_parameters}.cpp`。行番号は本文のとおり（取得したファイルのもの）。
- JTC 2.54.1: `joint_trajectory_controller/src/{joint_trajectory_controller,trajectory}.cpp`。forward controller 2.54.1: `forward_command_controller/src/forward_controllers_base.cpp`。ros2_control 2.54.3: `controller_manager`（コントローラごとの `update_rate`）。
- このリポジトリ: `docs/inventory.md` 末尾の「将来の目標と IK の経緯、Servo 統合のときの記録、注意」、`docs/original_urdf_plan.md` 22.8、`docs/feetech_write_check.md`（手順書の形）、`tools/send_joint_delta.py`（送信ツールの形）、`src/so_arm101_hardware/src/sts3215_system.cpp`（ドライバの制限）、`src/so_arm101_bringup/config/controllers.yaml`。
