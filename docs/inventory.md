# 旧ワークスペース `~/ros2_so101/src` インベントリ

調査日: 2026-11-05 / 調査方法: 読み取りのみ（ファイルの変更・コピーなし）

## 1. 概要

- 対象: `~/ros2_so101/src`（`package.xml` 10個、トップレベル3ディレクトリ: `so101_ros2/`, `so101_moveit_config/`, `apriltag_test/`）
- 由来は「寄せ集め」: nimiCurtis 製 `so101_ros2` 一式 + 自作の MoveIt 設定 + 自作の AprilTag 起動
- **LeRobot 依存は `so101_ros2_bridge` に集中**しており、実機起動 `so101_robot.launch.py` は必ずこれを起動する

## 2. ネストした .git の列挙

| 場所 | .git | remote URL |
|---|---|---|
| `~/ros2_so101/src` 配下（全深度） | **0個**（`.gitmodules` も無し） | - |
| `~/ros2_so101/.git`（親。src の1階層上） | 1個 | `https://github.com/hirohiro20051009/ros2_so101.git`（origin, fetch/push） |

- `~/ros2_so101` 配下（build/install/log除く）に存在する .git は親の1個だけ。
- src は親リポジトリが 225 ファイルを丸ごと追跡（履歴: "Track so101_ros2 source in parent repository" 等）。
- `so101_ros2` と `system_data_recorder` は本来 clone / submodule だが、**.git が剥がされたベンダリング状態**。
  したがって下表の「由来」のうち nimiCurtis 系は README / LICENSE / package.xml からの**推定**であり、
  `git remote` で確認できた URL ではない。

## 3. パッケージ表

凡例: ○=使用 / ×=不使用。「起動で使用」は include を辿った推移的な使用を含む。

| パッケージ | パス（src からの相対） | 由来 | 役割 | 主な依存 | LeRobot依存 | `so101_robot.launch.py` | `demo.launch.py` |
|---|---|---|---|---|---|---|---|
| so101_description | so101_ros2/so101_description | 推定: github.com/nimiCurtis/so101_ros2（MIT、maintainer Nimrod） | URDF/xacro/メッシュ/rviz。`rsp.launch.py` が `mode`(real/isaac/gazebo) で ros2_control プラグインを切替 | urdf, xacro, robot_state_publisher, controller_manager, rviz2, ign_ros2_control, topic_based_ros2_control, ros_gz_* | なし | ○ rsp.launch.py（`display:=true` で display.launch.py） | ○ moveit_config の urdf.xacro が `so101_new_calib.urdf` を include |
| so101_controller | so101_ros2/so101_controller | 同上 | controller_manager 起動 + spawner（arm / gripper / joint_state_broadcaster）。`so101_controllers.yaml` | rclcpp, rclpy, controller_manager, so101_description | なし | ○ | ×（moveit_config 側に自前の ros2_controllers.yaml） |
| so101_hardware_interface | so101_ros2/so101_hardware_interface | 同上 | ros2_control SystemInterface `So101HardwareBridge`（C++）。`joint_commands` を publish、`joint_states` を subscribe するトピック橋渡し。**Feetech を直接叩くわけではない** | hardware_interface, pluginlib, rclcpp_lifecycle, rmw_cyclonedds_cpp | 直接なし（相手の bridge ノード経由で間接依存） | ○ description の xacro が `mode=real` でプラグイン指定（launch には直接現れない） | × |
| so101_ros2_bridge | so101_ros2/so101_ros2_bridge | 同上 | LeRobot の SO101Follower/Leader ↔ ROS2 トピック変換、SmolVLA policy runner。entry: `follower_ros2_node` / `leader_ros2_node` / `policy_runner_ros2_node` | rclpy（package.xml に lerobot の記載なし＝pip 側の暗黙依存） | **あり（強い）**: `bridge/bridge.py`, `bridge/build.py`, `utils/conversion.py`, `policy/smolvla.py` が `import lerobot.*`。config の `calibration_dir` が `~/.cache/huggingface/lerobot/calibration/...` | ○ `follower_ros2_node` のみ（leader / policy は未使用） | ×（real 運転時は `/follower/*` トピックで間接依存） |
| so101_bringup | so101_ros2/so101_bringup | 同上 | ランチャ集。`so101_robot.launch.py` → `include/follower.launch.py`。ほか with_cameras / record / teleoperate、`include/{leader,camera,sim_isaac,policy}` | usb_cam, realsense2_camera, realsense2_description | なし（include 先で間接） | ○（本体） | × |
| so101_teleop | so101_ros2/so101_teleop | 同上 | leader 腕 → follower の teleop（C++ component `LeaderTeleopComponent`）、MoveIt 連携 | moveit_*, control_toolbox, joy, tf2_eigen 等 | なし | × | × |
| so101_ros2 | so101_ros2/so101_ros2 | 同上 | メタパッケージ（exec_depend のみ） | 上記 6 パッケージ | なし | × | × |
| system_data_recorder | so101_ros2/ros2_externals/system_data_recorder | 推定: github.com/nimiCurtis/system_data_recorder（README に submodule として記載）。maintainer は Geoffrey Biggs (Open Robotics)、Apache-2.0 | rosbag2 記録（分割ファイルの外部バックアップ） | rosbag2_*, rclcpp_lifecycle, yaml_cpp_vendor | なし | ×（`so101_record.launch.py` からのみ） | × |
| so101_moveit_config | so101_moveit_config | 自作（MoveIt Setup Assistant 生成物。maintainer hirok。package.xml の url は moveit2 の雛形のまま） | MoveIt 設定（SRDF / kinematics / joint_limits / servo）と `demo.launch.py`。`use_fake_hardware` で mock / 実機を切替 | moveit_ros_move_group, moveit_kinematics, moveit_planners, controller_manager, so101_description, warehouse_ros_mongo 等 | なし（real 時は `joint_state_topic_hardware_interface` で `/follower/*` を購読＝bridge 経由で間接依存） | × | ○（本体。default は `mock_components/GenericSystem`） |
| apriltag_test | apriltag_test | 自作（`ros2 pkg create` の雛形のまま。description / license は TODO） | `apriltag_d415.launch.py` で apriltag_ros を D415 画像に接続、`tags_36h11.yaml` | package.xml に runtime 依存の宣言なし（実際は apriltag_ros, realsense2_camera が必要） | なし | × | × |

## 4. 起動別の使用範囲

### `so101_robot.launch.py`（so101_bringup）
`include/follower.launch.py` を呼び、以下を起動する。
1. so101_description `rsp.launch.py`（`mode:=real`, `type:=follower`）
2. so101_ros2_bridge `so101_ros2_bridge.launch.py`（`follower_ros2_node`、**lerobot 使用**）
3. so101_controller `controller_manager.launch.py`（3 秒遅延）
4. so101_controller `so101_controllers.launch.py`（6 秒遅延、3 つの spawner）
5. `display:=true` のとき so101_description `display.launch.py`（rviz、3 秒遅延）

→ 使用パッケージ: bringup / description / ros2_bridge / controller / hardware_interface（プラグインとして）。**LeRobot 依存あり**。

### `demo.launch.py`（so101_moveit_config）
static_virtual_joint_tfs, rsp, move_group, moveit_rviz, (warehouse_db: `db:=true`), ros2_control_node（**`use_fake_hardware` が true のときのみ**）, spawn_controllers を起動する。

- 既定（`use_fake_hardware:=true`）: `mock_components/GenericSystem`。**LeRobot 不要**。
- `use_fake_hardware:=false`: `joint_state_topic_hardware_interface` が `/follower/joint_commands` / `/follower/joint_states` を使う。ただし現行 `demo.launch.py` は real 時に ros2_control_node を起動しない（`demo.launch.py.bak` には real 用ノードがあり、現行では削除されている）。bridge 側が別途起動している前提。

## 5. 所見

1. **LeRobot 依存は `so101_ros2_bridge` に集中**。実機経路（so101_robot.launch.py）はこれを必ず起動する。依存が無いのは demo の mock 経路のみ。
2. 実機用 ros2_control プラグインが 2 系統併存している。
   - description 側: `so101_hardware_interface/So101HardwareBridge`（トピック橋渡し）
   - moveit_config 側: `joint_state_topic_hardware_interface/JointStateTopicSystem`

   どちらも最終的に bridge（= lerobot）頼みで、目標の `so_arm101_hardware`（Feetech 直接）の置換対象。
3. `so101_new_calib.ros2_control.xacro` が description と moveit_config に**重複**している（内容は別: 前者は mode 切替、後者は use_fake_hardware 切替）。MoveIt 本体の設定は変更禁止のため扱いに注意。
4. 不要物の候補（事実の記録のみ。削除はせず、整理時は archive/ へ移動し理由を記録）:
   `*.bak*`（`demo.launch.py.bak` 等、`so101_controllers.yaml.bak_before_openloop`）、`bridge/bridge.py_org`、`__pycache__`、`so101_ros2/docs/_build`、sim_isaac / policy 系 launch。
5. 親リポジトリの `.gitignore` は `archive/`, `*.bak*`, `*_org`, `setup_so101.sh`, `readme.txt` 等を除外している。
6. 参照元として src の外に `~/ros2_so101/test`（自作の解析 IK・Diff-IK・キャリブ用スクリプト）がある。目標構成の ik / calibration の元になるが、今回の調査範囲外なので詳細は扱わない。
7. README が要求する forked LeRobot（`github.com/nimiCurtis/lerobot`）は src 外の Python 環境にあり、このインベントリには含まれない。

## 6. 確度の注記

- **確認済み**: .git の有無と親リポジトリの remote、package.xml の内容、launch の include 関係、`import lerobot` の箇所。
- **推定**: nimiCurtis 系パッケージの GitHub 由来（README の clone 先 / LICENSE の著作権者 / maintainer 名による）。.git が無いため、どのコミットを取り込んだかは不明。
- **未確認**: 実行時の挙動（launch は起動しておらず、コードの静的読解のみ）。サーボを動かすコマンドは一切実行していない。

---

# 段階1: 取り込み結果（2026-11-05）

旧WS `~/ros2_so101/src` は読み取りのみ。新WS `src/` にパッケージ名を変えずにコピーした（階層は `src/<pkg>` に平坦化）。

## 残したもの

| パッケージ | 由来 | 範囲 |
|---|---|---|
| so101_description | 推定: nimiCurtis/so101_ros2 | 丸ごと |
| so101_controller | 同上 | 丸ごと（`*.bak_before_openloop` を除く） |
| so101_hardware_interface | 同上 | 丸ごと |
| so101_ros2_bridge | 同上 | 丸ごと（`bridge.py_org`、`__pycache__` を除く）。leader / policy ノードも含む |
| so101_bringup | 同上 | 下記の一部のみ |
| so101_moveit_config | 自作（Setup Assistant 生成） | 丸ごと（`*.bak`、`__pycache__` を除く）。設定は未変更 |
| apriltag_test | 自作（雛形） | 丸ごと |

`so101_bringup` に残したファイル（follower と camera 系）:
`launch/so101_robot.launch.py`, `launch/so101_robot_with_cameras.launch.py`, `launch/include/follower.launch.py`, `launch/include/camera.launch.py`, `rviz/*.rviz`（2個）, `config/{so101_realsense2,so101_usb_cam,so101_cameras}.yaml`, `CMakeLists.txt`, `package.xml`。

## LICENSE

`so101_ros2` の LICENSE（MIT, Copyright 2025 nimiCurtis）を `LICENSE.so101_ros2` として残した。取り込んだ so101_description / controller / hardware_interface / ros2_bridge / bringup が対象。

## 外したものと理由

| 対象 | 理由 |
|---|---|
| so101_teleop | leader 腕テレオペ用。目標構成（bridge 経由の座標・姿勢指令）で不要。他パッケージから参照なし |
| so101_ros2（メタ） | exec_depend のみ。外したパッケージを含むため無意味 |
| system_data_recorder | 模倣学習用の rosbag 記録。目標構成に無い（Apache-2.0、同梱しないためライセンス転記なし） |
| bringup: leader / policy / sim_isaac / record / teleoperate の launch、`so101_sdr.yaml` | leader・LeRobot policy・Isaac Sim・記録は目標構成の範囲外。残す launch からの参照なし（静的に確認） |
| `*.bak*`, `*_org`, `*.py_org`, `__pycache__`, `docs/_build`, `*.yaml_org` | バックアップ・生成物 |
| so101_ros2 直下の README / CONTRIBUTING / docs / media / .github / build.sh / pyproject.toml 等 | パッケージではない。LeRobot 前提の手順が中心 |

旧WSのファイルは削除していない。

## 既知の問題（記録のみ。MoveIt 設定は変更していない）

静的調査と mock 起動確認（`demo.launch.py use_rviz:=false`、40 秒）で判明した。

1. **MoveIt の送信先は `/follower`**: `moveit_controllers.yaml` のコントローラ名は `follower/arm_controller`、`follower/gripper_controller`。アクションは `/follower/arm_controller/follow_joint_trajectory`、`/follower/gripper_controller/gripper_cmd`。`move_group` / `rsp` の `/joint_states` も `/follower/joint_states` にリマップされている。
2. **mock の ros2_control はルート名前空間**: `ros2_controllers.yaml` は `/follower` なしのため、mock のコントローラ（`/arm_controller` 等）と食い違う。mock の `joint_state_broadcaster` は `/joint_states` に出すが、購読者は 0（`move_group` / `rsp` は `/follower/joint_states` を購読）。
3. **mock 起動時に arm / gripper の spawner が失敗する**: `generate_spawn_controllers_launch` が `moveit_controllers.yaml` から spawner を作るため、`spawner_follower/arm_controller` が不正なノード名（`/` を含む）になり `InvalidNodeNameException` で落ちる。結果、active なのは `joint_state_broadcaster` のみ。mock では軌道実行できない。
4. `demo.launch.py` は `use_fake_hardware:=false` のとき `ros2_control_node` を起動しない（`.bak` には存在した）。
5. `demo.launch.py` 単体では `/follower` 側は起動しない。`so101_robot.launch.py` を同時起動すると、ルートの mock と `/follower` の実機側が併存し、`rsp` も二重になる。
6. SRDF に関する warning（`Link 'elbow_flex' is not known to URDF`）が出る。

## 検証結果

- `colcon build --symlink-install`（`/opt/ros/humble` のみ source）: 7 パッケージ成功。
- `demo.launch.py`（mock）: `move_group`、`robot_state_publisher`、`controller_manager`、`joint_state_broadcaster`（active）が起動。arm / gripper は上記 3 により未 spawn。`/follower` ノードは存在しない。
- 実機は起動していない。

---

# 段階2a: パッケージ改名と bringup 統合（2026-11-05）

## パッケージ名の対応

| 旧 | 新 |
|---|---|
| so101_description | so_arm101_description |
| so101_moveit_config | so_arm101_moveit_config |
| so101_controller | so_arm101_controller |
| so101_hardware_interface | so_arm101_hardware |
| so101_ros2_bridge | so_arm101_lerobot_bridge（暫定名。段階2bで置き換え。`so_arm101_bridge` は rosbridge 用に予約） |
| so101_bringup | so_arm101_bringup |
| apriltag_test | （2a の対象外、変更なし） |

変更したのはパッケージ名のみ（ディレクトリ、package.xml、CMake、Python モジュール名、`package://` と `$(find)`、プラグイン名 `so_arm101_hardware/So101HardwareBridge`）。URDF のリンク名・関節名・フレーム名、C++ の namespace・クラス名・ライブラリ名、実行ファイル名、launch ファイル名は変えていない。

## 起動入口

`so_arm101_bringup/launch/bringup.launch.py`（引数 `mock`、既定 true / `use_rviz` / `db`）と、これにカメラを加えた `bringup_with_cameras.launch.py`。

- 共通（ルート名前空間）: static TF（world→base_link）、`robot_state_publisher` 1つ、`ros2_control_node` 1つ、spawner（3 秒遅延）、move_group、RViz。
- `mock:=true`: `mock_components/GenericSystem`。
- `mock:=false`: `So101HardwareBridge` + LeRobot bridge ノード（`type:=follower`、namespace なし）を追加で起動。
- `/follower` 名前空間と TF の `follower/` プレフィックスは廃止した。

## /follower をルートに揃えた方法

bridge と `So101HardwareBridge` のトピック名（`joint_commands`、`joint_states_raw`）は相対名で、`/follower` は起動側が付けた namespace だけだった（`ros2_control_node` の `__ns` がプロセス内の全ノードに効くため、プラグイン内ノードも同じ namespace になっていた）。bridge のコード変更は不要で、launch から `namespace=` を外した。実機のトピックは `/joint_commands`、`/joint_states_raw`、`/joint_states`（joint_state_broadcaster）。

## コントローラ設定の統合

統合先: `so_arm101_controller/config/controllers.yaml`（ルート）。

| 項目 | moveit_config `ros2_controllers.yaml` | 旧実機側 `/follower` | 統合後 |
|---|---|---|---|
| cm `update_rate` | 50 | 50 | 50 |
| arm `type` | なし | JointTrajectoryController | あり |
| arm `update_rate` | なし | 25 | 25 |
| arm `gains`（p/i/d=0） | なし | あり | 引き継ぎ |
| arm `open_loop_control` | なし | true | **true（引き継ぎ済み。起動後に `ros2 param get` で True を確認）** |
| arm `state_publish_rate` | なし | 50.0 | 50.0（確認済み） |
| arm `action_monitor_rate` | なし | 20.0 | 20.0 |
| arm `time_scaling` | なし | 1.1 | 1.1 |
| gripper `type` | なし | あり | あり |
| `/leader` ブロック | - | あり | 廃止 |

`ros2_controllers.yaml`（moveit_config）は無変更のまま未使用（Setup Assistant の再生成で上書きされるため）。原本の `so101_controllers.yaml` は `archive/stage2a/controller/config/` に保存。

## MoveIt 側の変更（承認済み A〜F）

| # | 対象 | 変更 |
|---|---|---|
| A | `config/moveit_controllers.yaml` | `follower/` プレフィックスを削除（controller_names と2つのキー） |
| B | `launch/move_group.launch.py` | `/joint_states`→`/follower/joint_states` のリマップを削除 |
| C | `launch/rsp.launch.py` | 同上 |
| D | `config/so101_new_calib.ros2_control.xacro` | real 分岐を `JointStateTopicSystem`+`/follower/*` から `so_arm101_hardware/So101HardwareBridge` に置換。関節定義は変更なし（mock/real で関節とリンクの一覧が同一であることを確認） |
| E | `config/servo.yaml` | 2 行から `/follower` を削除 |
| F | `launch/demo.launch.py` | `archive/stage2a/` へ移動 |

SRDF、kinematics、joint_limits、pilz、initial_positions、`ros2_controllers.yaml`、URDF は変更していない。

## 段階1の既知の問題の解消

| 問題 | 結果 |
|---|---|
| 1. MoveIt の送信先が `/follower` | 解消。A・B・C・E |
| 2. mock と MoveIt の名前空間の食い違い | 解消。全てルート |
| 3. spawner が不正ノード名で失敗 | 解消。A。arm / gripper / joint_state_broadcaster の 3 つが active |
| 4. real 時に `ros2_control_node` が無い | 解消。モードに関わらず常に 1 つ起動 |
| 5. rsp 二重 | 解消。rsp は 1 つ |
| 6. SRDF の warning（`Link 'elbow_flex' is not known to URDF`） | **未解消**。SRDF は変更禁止のため記録のみ |

## archive/stage2a/ に移したもの（理由は `archive/stage2a/README.md`）

bringup の `so101_robot.launch.py`、`include/follower.launch.py`、`robot_display*.rviz`。controller の launch 2 つと旧 yaml。description の launch 3 つと `display.rviz`。moveit_config の `demo.launch.py`。`build/install/log` は `archive/stage1_build/`（gitignore）へ退避。

## 文字列のみ変更

`io.yaml`、`runner.py`（ポリシー用、未使用）の `/follower/...` と、`so101_usb_cam.yaml` の `/follower/cam_front` をルートにした。

## 検証結果

- `colcon build`（`/opt/ros/humble` のみ source）: 7 パッケージ成功（改名後と統合後の両方）。
- `mock:=true`（`use_rviz:=false`）: `/follower` を含むノード・トピック・アクションなし。`robot_state_publisher` 1、`ros2_control_node` 1。`arm_controller` / `gripper_controller` / `joint_state_broadcaster` が active。
- 軌道実行: `/arm_controller/follow_joint_trajectory` に goal → SUCCEEDED、関節が目標へ移動。`/move_action`（plan+execute、RViz の MotionPlanning が使う経路）→ SUCCEEDED、関節が目標付近へ移動。
- `use_rviz:=true`: RViz が起動し controller は active のまま。ログのエラーは octomap センサ未定義と `/recognize_objects` 不在のみ。RViz 画面上の操作は未実施（人手で確認）。
- `mock:=false`: **起動していない**。xacro 展開のみ確認（plugin が `So101HardwareBridge`、`/follower` 文字列なし）。実機での確認は未実施。

## 残課題（段階2a の範囲外）

- description の `so101_new_calib.urdf.xacro` / `ros2_control.xacro`（`mode` による real/isaac/gazebo 切替）は、launch が archive へ移ったため未使用。moveit_config 側の xacro と重複している。
- 実機側の bridge は LeRobot 依存のまま（段階2bで置き換え予定）。
- `so101_cameras.yaml` の実カメラは `cam_side` のみ。`camera.launch.py` は mock でも起動する。
- xacro の `load_yaml() directly is deprecated` 警告。

---

# 段階2a 追補: 実機起動時の `Failed to send commands to robot: 10`（2026-11-05）

## 症状と原因

`mock:=false` で `follower_ros2_node` が 50 Hz（ros2_control の `update_rate`）で `Failed to send commands to robot: 10` を出し続け、腕が動かない。

`/follower` 名前空間を外したことで、`so101_follower_params.yaml` が**初めて適用された**ため。

- yaml の最上位キーは `so101_follower_interface`（先頭に `/` なし）。launch のノード名と一致するのは名前空間なしのときだけ。
  段階1（`namespace=follower`、実ノードは `/follower/so101_follower_interface`）では適用されていなかった。
- ダミーノードで実測（同一 launch 設定、宣言のみ。ハード・LeRobot には触れない）:

  | namespace | `max_relative_target` | `publish_rate` |
  |---|---|---|
  | follower（段階1相当） | 0 | 30.0 |
  | なし（2a） | 10 | 50.0 |

- `bridge.py:251-254` は int の `max_relative_target` を取り、`0` のときだけ `None` に変換する。非0 の int はそのまま `SO101FollowerConfig` に渡る。
- LeRobot 0.4.2 の `ensure_safe_goal_position`（`lerobot/robots/utils.py:76-83`）は `float` と `dict` のみ許容し、それ以外は `raise TypeError(max_relative_target)`。`isinstance(10, float)` は False なので `TypeError(10)`。`send_action` は `sync_write` の前に例外となり、腕に指令が届かない。
- yaml に `10.0` と書く方法も不可（宣言が int なので型不一致）。

## 修正

`so_arm101_lerobot_bridge/config/so101_follower_params.yaml`:

| キー | 旧 | 新 |
|---|---|---|
| `max_relative_target` | 10 | **0**（無制限＝None。段階1と同じ） |
| `publish_rate` | 50.0 | **30.0**（段階1と同じ。今回のエラーの原因ではない） |

`id`、`calibration_dir`、`use_degrees`、`disable_torque_on_disconnect` は変更していない。インストール済み yaml をダミーノードで読み、`max_relative_target=0`、`publish_rate=30.0` を確認した。実機は起動していない。

注意: `max_relative_target: 0` は 1 コマンドあたりの最大変位の安全上限を無効にする（段階1と同じ状態に戻るだけ）。上限を有効にしたい場合は、bridge で `float()` にする小変更が必要（未実施）。

## 見つかった事実: キャリブレーションファイルが変わっていた

yaml が未適用だった段階1では、`id` と `calibration_dir` もデフォルトだった。

| | 段階1（yaml 無効） | 2a（yaml 有効） |
|---|---|---|
| `id` | `Tzili` | `so101_follower` |
| ファイル | パッケージ同梱 `config/calibration/Tzili.json`（2026-03-09、上流由来） | `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/so101_follower.json`（2026-09-07） |

2 つのファイルは全関節で値が異なる（例: gripper の range は 1515〜3009 と 1900〜3385）。bridge は `connect(calibrate=False)` でモータには書き込まない。2a では自機のキャリブレーションが初めて使われていると考えられる（モータ側の値との一致は未確認）。段階1の角度変換は上流の `Tzili.json` に基づいていた可能性がある。このため `id` / `calibration_dir` は段階1相当に戻していない。
同梱の `Tzili.json` / `Gili.json` は上流の他機体用で、`id` を変えない限り使われない。

## 未確認

実機（`mock:=false`）の実動作。ユーザーの確認待ち。

---

# 段階2a 完了: 実機確認で判明した問題2（2026-11-06）

実機（`mock:=false`）での確認結果（ユーザー報告）。

## 問題2: home の折りたたみ姿勢が衝突と判定され、Plan & Execute が中止される

- 現象: home の折りたたみ姿勢（shoulder_lift −1.749、elbow_flex 1.700、wrist_flex 1.313 rad）が `upper_arm_link`–`wrist_link` の衝突と判定され、Plan & Execute が中止された。
- 付随して、shoulder_lift と elbow_flex は関節リミットをそれぞれ 0.004 rad、0.010 rad 超えている。ただしこれは中止の原因ではない（原因は上記の衝突判定）。
- 回避方法: 最初の動作を home から少し開いた姿勢で始める（実機で確認済み）。
- 対応: **SRDF は変更していない**（MoveIt 本体の設定は変更しない方針）。
- 未修正の課題（段階2b 以降）:
  - SRDF の `elbow_flex`（関節名）の行。段階1から出ている警告 `Link 'elbow_flex' is not known to URDF. Cannot disable/enable collisons.` と同じ箇所とみられる（リンクとして指定されている可能性。未調査）。
  - home 姿勢と関節リミットの関係（上記 0.004 / 0.010 rad の超過）。

## 段階2a の状態

| 項目 | 状態 |
|---|---|
| `colcon build` | 通過（7 パッケージ） |
| mock（`mock:=true`）の軌道実行 | 確認済み |
| 実機（`mock:=false`） | 起動・動作を確認（ユーザー報告）。問題2の回避方法を含む |
| 既知の問題1〜5 | 解消 |
| 既知の問題6（SRDF の warning） | 未解消（問題2と同じ SRDF の課題として段階2b 以降） |
| 問題2 | 回避方法あり。根本対応は段階2b 以降 |

---

# 段階2b-1: Feetech 自前ドライバの調査と設計（2026-11-06）

方針: 最終的に LeRobot（コードもキャリブファイルも）に依存しない。`so101_follower.json`（2026-09-07 作成）は yaml を作るための**一回だけの変換元**で、ドライバは json を読まない。比較の基準は「`so101_follower.json` を使った LeRobot 経由の `/joint_states`」（段階1の `Tzili.json` ではない）。
**[事実]** = ソースで確認、**[推測]** = 未確認。`SP` = `~/miniforge3/envs/ros2_lerobot/lib/python3.10/site-packages`（参照のみ。import しない）。

## 変換の連鎖

```
サーボ Present_Position(reg56, 2B, bit15=符号)
 → decode_sign_magnitude(bit15)              SP/lerobot/motors/motors_bus.py:1088, encoding_utils.py:29
 → _normalize()                              motors_bus.py:770-797
     angle(DEGREES):       deg = (raw − (min+max)/2)·360/4095        :790-793（clamp なし、分母は 4095）
     gripper(RANGE_0_100): pct = (clamp(raw,min,max)−min)/(max−min)·100   :787-789
 → obs["<joint>.pos"]                        robots/so101_follower/so101_follower.py:173-191
 → bridge: rad = math.radians(値)            so_arm101_lerobot_bridge/bridge/bridge.py:132-133（use_degrees=true）
 → /joint_states_raw → So101HardwareBridge → joint_state_broadcaster → /joint_states
```

- モータ定義: angle 5 関節 = `MotorNormMode.DEGREES`、gripper = `RANGE_0_100`（`so101_follower.py:48-59`）。
- **ソフト変換で使うのは `range_min/max` だけ**。`homing_offset` はサーボの EEPROM(reg31) に書かれる値で、`Present_Position = Actual_Position − Homing_Offset`（`feetech/feetech.py:283-292`）。**読んだ raw に既に含まれる**。bridge は `connect(calibrate=False)`（`bridge.py:103`）なので書き込まない。
- `drive_mode` は全関節 0。DEGREES 分岐は参照しない（`motors_bus.py:779` で取得、790-793 で未使用）→ angle 関節に符号反転はない。
- gripper: LeRobot は 0〜100%、bridge はこれを**度とみなして `radians()`**（`bridge.py:132-133`）。gripper_rad = pct·π/180（0〜1.745 rad、URDF 上限 1.74533 と一致）。
- 1 tick = 2π/4095 = 0.0015344 rad（angle 関節）。

`so101_follower.json`（2026-09-07）の値:

| joint | id | range_min | range_max | center=(min+max)/2 | homing_offset |
|---|---|---|---|---|---|
| shoulder_pan | 1 | 743 | 3452 | 2097.5 | 135 |
| shoulder_lift | 2 | 842 | 3216 | 2029.0 | 112 |
| elbow_flex | 3 | 862 | 3088 | 1975.0 | 122 |
| wrist_flex | 4 | 889 | 3185 | 2037.0 | 89 |
| wrist_roll | 5 | 164 | 3965 | 2064.5 | 1164 |
| gripper | 6 | 1900 | 3385 | （linear 0→min, 100→max） | 107 |

## STS3215 通信仕様（`SP/lerobot/motors/feetech/tables.py`、`SP/scservo_sdk/`）

- プロトコル 0、**1,000,000 bps**、8 データビット（`port_handler.py:92-107`）。ポートは `/dev/ttyACM0`（`so101_follower_params.yaml`）。
- パケット `FF FF <ID> <LEN> <INST> <PARAM…> <CHK>`、`CHK = ~sum(ID..PARAM) & 0xFF`（`protocol_packet_handler.py:69-100`）。応答 `FF FF <ID> <LEN> <ERR> <DATA…> <CHK>`、`LEN`=データ長+2。
- 命令: PING=1, READ=2, WRITE=3, SYNC_READ=0x82, SYNC_WRITE=0x83, BROADCAST=0xFE（`scservo_def.py`）。SYNC_READ 要求: `ID=0xFE, LEN=n+4, INST=0x82, [addr, 長さ, id1…idn]`（`protocol_packet_handler.py:431-447`）。応答は**各サーボが個別のステータスパケットを順に返す**（`group_sync_read.py:66-69`）。
- レジスタ（`tables.py:41-101`）: Model_Number(3,2) / ID(5) / Return_Delay_Time(7) / Min_Position_Limit(9,2) / Max_Position_Limit(11,2) / Homing_Offset(31,2) / Torque_Enable(40) / Goal_Position(42,2) / Lock(55) / **Present_Position(56,2)**。
- 符号は sign-magnitude: Present/Goal_Position は bit15、Homing_Offset は bit11（`tables.py:207-215`）。リトルエンディアン。
- LeRobot の `SO101Follower.connect()` は `configure()`（`so101_follower.py:149-171`）で書き込み、最後に Torque_Enable=1/Lock=1 を書く＝**bridge 起動中はトルク ON**。`disconnect()` は `disable_torque_on_disconnect: true` で Torque_Enable=0/Lock=0（`motors_bus.py:450-469`）＝**bridge 停止で腕が脱力する**。
- **[推測]** EEPROM の現在値（homing_offset など）が json と一致しているか、Return_Delay_Time の現在値は未確認。2b-2 の起動時診断（読み取りのみ）で確認する。

## yaml の置き場所と形式

置き場所: `src/so_arm101_hardware/config/follower_calibration.yaml`（この腕固有の値。コミットする）。
段階2c は**同じ形式の中身を差し替えるだけ**にする。

```yaml
schema_version: 1
source: {kind: converted_from_lerobot_json, file: <json のパス>, file_date: 2026-09-07, converted_at: <日付>}
bus: {baudrate: 1000000, protocol: 0}
conversion: {angle_ticks: 4095}        # LeRobot 等価。物理的には 4096 の可能性（2c で再検討）
joints:                                # URDF の関節名で引く
  shoulder_pan: {id: 1, kind: angle, direction: 1, center: 2097.5, min: 743, max: 3452,
                 eeprom: {homing_offset: 135, min_limit: 743, max_limit: 3452}}
  gripper:      {id: 6, kind: linear_percent, direction: 1, min: 1900, max: 3385, eeprom: {...}}
```

- angle: `rad = direction·(raw − center)·2π/angle_ticks`（clamp なし）。
- linear_percent: `pct = (clamp(raw,min,max)−min)/(max−min)·100`（`direction=−1` なら `100−pct`）、`rad = radians(pct)`。
- `center` は LeRobot 等価では (min+max)/2 だが明示的に保持する。`eeprom.*` は**変換に使わない診断専用**（EEPROM の実値との比較用）。

**段階2c で「LeRobot に頼らない」ために必要なもの**: ① `Homing_Offset` の書き込み（sign-magnitude bit11。EEPROM 書込みには Torque_Enable=0 と Lock=0 が必要。値は `pos − int((4096−1)/2)`、事前に offset=0・limit=0〜4095 へリセット: `motors_bus.py:660-711`、`feetech.py:283-292`）② `Min/Max_Position_Limit` の書き込み ③ 脱力状態での MIN/MAX 記録手順 ④ yaml への保存。`wrist_roll` は span 3801 tick（ほぼ 1 回転）で、homing_offset が無いと 0/4095 で値が折り返すため、②③ とあわせて省略できない。⑤ 書き込み経路（2b-3）が前提。

## ドライバ設計

`so_arm101_hardware` に新クラス `so_arm101_hardware/Sts3215System`（`hardware_interface::SystemInterface`）を追加。`So101HardwareBridge` は比較用に残す。シリアルは **termios 直書き**（追加依存なし。libserial / Boost.Asio / 公式 C++ SDK は採用しない。公式 SDK は rosdep に無くライセンス未確認）。追加依存は yaml-cpp のみ。

| ファイル | 役割 |
|---|---|
| `serial_port.{hpp,cpp}` | termios（`B1000000`、8N1、raw、`poll` 付き read、`TIOCEXCL`） |
| `sts3215_bus.{hpp,cpp}` | パケット組立・チェックサム・`ping`・`read`・`sync_read`・sign-magnitude。**書き込み API を持たない** |
| `calibration.{hpp,cpp}` | yaml 読込と raw→rad の純関数 |
| `sts3215_system.{hpp,cpp}` | `SystemInterface`。Present_Position の読み取りのみ。`write()` は何もしない |

launch: `bringup.launch.py` に `hardware:=lerobot|feetech`（`mock:=false` のときだけ有効、既定 `lerobot`）、`serial_port`、`calibration_file`、`read_only` を追加。`mock:=true` の挙動は変えない。

## MoveIt 設定の変更（G・H。承認済み）

SRDF、kinematics、joint_limits、`moveit_controllers.yaml`、`ros2_controllers.yaml`、URDF のリンク・関節・フレームは変更しない。

- **G** `so_arm101_moveit_config/config/so101_new_calib.urdf.xacro`: `xacro:arg` を 4 つ追加（`hardware`=lerobot、`serial_port`=/dev/ttyACM0、`calibration_file`=空、`read_only`=true）し、`so101_new_calib_ros2_control` マクロへ渡す。
- **H** `so_arm101_moveit_config/config/so101_new_calib.ros2_control.xacro`: マクロの引数に同じ 4 つを追加。`use_fake_hardware` が false の分岐を `hardware == 'feetech'` で分け、feetech のとき `<plugin>so_arm101_hardware/Sts3215System</plugin>` と `<param>`（`serial_port`、`calibration_file`、`read_only`）を出す。それ以外は現行の `So101HardwareBridge`。関節定義は不変。

## 比較の前提（実機手順は 2b-2 で docs に書く）

同じポートは同時に使えない。LeRobot 経路は起動中トルク ON、停止でトルク OFF（腕が落ちる）。基準は「LeRobot 経路で記録 → 停止（腕に手を添える）→ 同じ姿勢で feetech を記録」。合格基準: 静止姿勢で各関節の平均差 ≤ 1 tick、最大差 ≤ 2 tick、EEPROM の homing_offset / limit が json と一致。

## 段階 2b-2 以降

- 2b-2: 変換スクリプト、読み取り専用ドライバ、テスト、比較スクリプト（書き込み・トルクには触れない）。
- 2b-3: 書き込みモード（トルク ON、Goal_Position）。別途承認。
