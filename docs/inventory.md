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

---

# 段階2b-2: 読み取り専用ドライバ `Sts3215System`（2026-11-06）

実機（サーボ）は動かしていない。実ポートも開いていない（結合確認は pty 上の偽サーボ）。実機比較はユーザーが `docs/feetech_readonly_check.md` に従って行う。

## 追加・変更したファイル

**新規**
| ファイル | 内容 |
|---|---|
| `src/so_arm101_hardware/include/so_arm101_hardware/{serial_port,sts3215_bus,calibration,sts3215_system}.hpp` | ヘッダ |
| `src/so_arm101_hardware/src/{serial_port,sts3215_bus,calibration,sts3215_system}.cpp` | termios シリアル / 読み取り専用バス / yaml 読込と変換 / `SystemInterface` |
| `src/so_arm101_hardware/config/follower_calibration.yaml` | 自前キャリブ yaml（変換元 `so101_follower.json`、2026-09-07、変換日 2026-11-06） |
| `src/so_arm101_hardware/test/test_conversion.cpp` | 変換の正解表照合ほか |
| `src/so_arm101_hardware/test/test_sts3215_bus.cpp` | パケット・エラー処理（pty 上の偽サーボ） |
| `src/so_arm101_hardware/test/data/conversion_golden.csv` | LeRobot 0.4.2 `_normalize` による正解表（24,596 行、1.3 MB） |
| `tools/convert_lerobot_calibration.py` | json → yaml 変換（一回だけ使う。lerobot を import しない） |
| `tools/gen_conversion_golden.py` | 正解表の生成（lerobot を使う開発用。シリアルは開かない） |
| `tools/record_joint_states.py` | `/joint_states` を指定秒数記録して CSV に保存 |
| `tools/compare_joint_states.py` | 2 つの CSV を関節ごとに比較（平均・最大・標準偏差、rad と tick、合否） |
| `tools/lerobot_readonly_state.py` | 【比較専用】LeRobot の bus を読み取り専用で開いて CSV 記録（実機ポートを開く。書き込みを不可能にしてある） |
| `tools/fake_sts3215.py` | pty 上の偽サーボ（結合確認用） |
| `docs/third_party.md` | LeRobot 0.4.2（Apache-2.0）等の出典とライセンス |
| `docs/feetech_readonly_check.md` | ユーザー向け実機確認手順（コマンドつき） |

**変更**
| ファイル | 変更 |
|---|---|
| `src/so_arm101_hardware/{CMakeLists.txt,package.xml,so101_hardware_bridge_plugins.xml}` | 新ライブラリ `so_arm101_sts3215`、yaml-cpp、gtest、プラグイン登録（ルートを `<class_libraries>` に変更し 2 クラス） |
| `src/so_arm101_moveit_config/config/so101_new_calib.urdf.xacro` | **G**: `xacro:arg` 4 つ（hardware, serial_port, calibration_file, read_only）を追加しマクロへ渡す |
| `src/so_arm101_moveit_config/config/so101_new_calib.ros2_control.xacro` | **H**: マクロ引数を追加。`hardware=='feetech'` で `Sts3215System` と `<param>`（serial_port, calibration_file, read_only）。それ以外は従来の `So101HardwareBridge`。関節定義は不変 |
| `src/so_arm101_bringup/launch/bringup.launch.py` | `hardware:=lerobot｜feetech`（`mock:=false` のときだけ有効、既定 lerobot）、`serial_port`、`calibration_file`、`read_only` を追加。rsp を bringup 内の Node にした |
| `src/so_arm101_bringup/launch/bringup_with_cameras.launch.py` | `hardware` を引き継ぐ |
| `docs/inventory.md` | この節 |

MoveIt の他の設定（SRDF、kinematics、joint_limits、moveit_controllers.yaml、ros2_controllers.yaml、URDF のリンク・関節・フレーム）は変更していない。

## ドライバの仕様

- **書き込み禁止**: バスクラスが送れる命令は `Read(0x02)` と `SyncRead(0x82)` だけ。書き込み系（WRITE / REG_WRITE / ACTION / SYNC_WRITE）は型として存在しない。トルクにも触れない。テスト（`NeverSendsAnythingButReadInstructions`）で、偽サーボが受けた命令コードが 0x02 / 0x82 のみであることを確認。結合確認でも偽サーボが「想定外の命令」を受けていない。
- **読み取り**: 周期ごとに全関節の `Present_Position`(reg56) を 1 回 SYNC_READ。bit15 を sign-magnitude として解釈。ステータスパケットごとにチェックサム・ID・ERR バイト・長さを検証。
- **起動時診断**（読み取りのみ）: 各関節の Model_Number(777 以外は起動中止)、Return_Delay_Time、Homing_Offset(**bit11 の sign-magnitude**)、Min/Max_Position_Limit を読み、yaml の `eeprom.*` と比較。関節ごとに 1 行ログ（一致は INFO `MATCH`、差があれば WARN `MISMATCH homing_offset(servo A, yaml B)` 等）。最後に `n/6 joints match`。不一致でも止まらない。
- **読み取り失敗の扱い**:
  1. 失敗（タイムアウト / チェックサム / ID 不一致 / ERR / 長さ不正）したら受信バッファを捨てて **1 回再試行**。
  2. 再試行も失敗したら前回値を保持し、`read failed (...); consecutive=n/10; holding previous values` を WARN（最初の 1 回と 10 回ごと）。
  3. **連続 10 回**失敗で `read()` が `ERROR` を返し、**state を NaN にする**（古い値が有効値として流れ続けない）。controller_manager はハードウェアを `unconfigured` にして、`/joint_states` は NaN になる（結合確認で確認）。
  4. 復帰したら `read recovered after n consecutive failures`。再試行・失敗の回数は deactivate 時に `reads / retried / failed_cycles` として INFO に出す。
  5. 後続 ID が先に応答した場合は「その ID は無応答」として Timeout 扱い（欠落サーボの原因が分かるように）。
  - 既定値: パケットあたりのタイムアウト 10 ms、連続失敗の上限 10（ハードウェアパラメータ `timeout_ms` / `max_consecutive_failures` で変更可。xacro は出していない）。
- **command は破棄**: `write()` は何もしない。command は read のたびに state へ追従させる（`open_loop_control: true` を維持）。起動時に WARN を 1 回（`READ-ONLY mode: ... commands are DISCARDED. The arm will not move.`）。さらに実際に指令と実測が 0.02 rad 以上ずれたとき 1 回だけ WARN。
- **velocity の state**: 現状の `So101HardwareBridge` と同じ**位置の有限差分**。`So101HardwareBridge` は LeRobot bridge が計算した値をそのまま受ける（`bridge.py:138-143`: `(pos − 前回pos) / dt`、フィルタなし、最初の 1 回は 0.0、publish_rate 30 Hz）。`Sts3215System` も `(pos − 前回pos) / dt`（dt は controller_manager の時刻差、50 Hz）、フィルタなし、起動後の最初の 1 回は 0.0。静止していても 1 tick の量子化で最大 0.077 rad/s のノイズが出る（1 tick = 0.00153 rad ÷ 0.02 s）。
- **排他**: `TIOCEXCL` でポートを排他にする。他のプロセスが開いていれば起動時に `cannot open serial port` で失敗する。
- 変換は純関数 `raw_to_rad(const JointCalibration&, int angle_ticks, int raw)`（`calibration.cpp`）として切り出し、テストから直接呼んでいる。

## 変換の検証（ハード不要）

LeRobot 0.4.2 `MotorsBus._normalize()` を**接続せずに**直接呼んで正解表を作った（`tools/gen_conversion_golden.py`。`PortHandler.openPort` を例外にしてシリアルを開けないようにしている）。出典: `lerobot/motors/motors_bus.py` の `MotorsBus._normalize()`（Apache-2.0、`docs/third_party.md`）。

- **LeRobot の clamp をソースで確認した**（`motors_bus.py:783`）: `bounded_val = min(max_, max(min_, val))`。これは RANGE_M100_100 と RANGE_0_100 に使われ、**DEGREES は `val` をそのまま使う（clamp なし）**（790-793）。したがって C++ も gripper は `[min,max]` に clamp、angle 関節は clamp なし。
- 正解表: 全 6 関節 × raw 0〜4095 の全値 + 範囲外 3 点（−100、4096、5000）= 各 4,099 点、計 24,594 点。
- **一致の基準: |Δrad| ≤ 1e-12**。どちらも IEEE754 double で数回の四則演算だけで、演算順序も LeRobot に合わせている（`(raw − center)·360/angle_ticks` の後に `·(π/180)`、CPython の `math.radians` と同じ）。誤差は 1e-15 程度のはずで、基準には 3 桁の余裕がある。
- **結果: 全 6 関節で最大誤差 0 rad（ビット単位で一致）**。gripper の clamp（範囲外の raw が 0 と 100% になる）と angle 関節の clamp なし（raw=0 で −184.4°）も確認。

テスト結果（`colcon test --packages-select so_arm101_hardware`）: `test_conversion` 7 件 PASSED、`test_sts3215_bus` 15 件 PASSED、失敗 0。
- `test_conversion`: 正解表との一致 / gripper の clamp / angle の clamp なし / gripper の tick 換算 / yaml の内容 / 壊れた yaml の拒否 / sign-magnitude。
- `test_sts3215_bus`: 要求パケットのバイト列（SDK と一致）/ 書き込み系命令が型に無い / SYNC_READ・READ / タイムアウト（失敗 ID の特定）/ チェックサム不良 / ERR バイト / ID 不一致 / 前に雑音が入るケース / 失敗後の復帰 / ポート未オープン / 書き込み命令を送らないこと / ポートの排他。

## 結合確認（pty 上の偽サーボ。実機・実ポートは開いていない）

`tools/fake_sts3215.py` に対して `bringup.launch.py mock:=false hardware:=feetech serial_port:=<pty>` を起動。
- 起動時診断が 6 関節とも MATCH（`6/6 joints match`）。Homing_Offset を −140 にした偽サーボでは該当関節だけ WARN `MISMATCH homing_offset(servo -140, yaml 112)`（負の値の bit11 デコードも確認）。
- `/joint_states` は期待値どおり（pan: raw 2197 → 0.15267 rad、gripper: raw 2400 → 0.5877 rad）、velocity は静止で 0。コントローラ 3 つ active、50 Hz。
- 一時的な無応答（0.15 s）→ `read recovered after 7 consecutive failures`。長い無応答 → 連続 10 回で ERROR、ハードウェアが `unconfigured`、`/joint_states` が NaN。
- 実在しないポートを指定 → `cannot open serial port: open(...) failed: No such file or directory` を出して `ros2_control_node` が停止。LeRobot bridge は起動しない。
- `hardware:=feetech` が LeRobot を使っていないこと: 環境変数が空の環境で、`follower_ros2_node` / lerobot / miniforge3 を含むプロセスなし、bridge 系ノード・トピックなし、`ros2_control_node` の maps に lerobot / miniforge3 なし（0）、`libso_arm101_sts3215.so` の依存に python / lerobot / conda なし。

## mock の回帰（変更なし）

- `mock:=true`: `mock_components/GenericSystem`、rsp 1・ros2_control_node 1、コントローラ 3 つ active、`/follower` なし、`open_loop_control` True、`FollowJointTrajectory` と `/move_action`（plan+execute）が SUCCEEDED。
- **`mock:=true hardware:=feetech serial_port:=/tmp/nonexistent_tty` でも結果は同じ**（hardware / serial_port は無視される。ログに Sts3215 / ポートの記述なし）。
- xacro 展開: mock では `hardware` に関わらず `mock_components/GenericSystem`、`mock:=false` は `hardware:=lerobot`（既定）で `So101HardwareBridge`、`hardware:=feetech` で `Sts3215System`。関節・リンク・limit の定義は全ケースで同一（md5 一致）。
- bridge の起動条件（`mock=false` かつ `hardware=lerobot` のときだけ）を 8 通りで評価して確認。`mock:=false hardware:=lerobot` の起動は実機に触れるので実行していない。

## 2b-1 の計画から変えた点

1. バスクラスに `ping` を作っていない。機種番号の読み取り（reg3）が存在確認を兼ねる。
2. xacro H の `<param>` は `serial_port` / `calibration_file` / `read_only` の 3 つ。`baudrate` は yaml の `bus.baudrate` を使う（二重定義を避ける）。`timeout_ms` / `max_consecutive_failures` はハードウェアパラメータとして読むが xacro は出さない。
3. rsp は `moveit_config` の `rsp.launch.py`（新しい mappings を渡せない）を使わず、bringup 内の Node にした。`ros2_control_node` と同じ `robot_description` になる。`rsp.launch.py` 自体は無変更・未使用。
4. 変換の一致基準を 1e-9（2b-1 の案）から **1e-12** に厳しくした（実測は 0）。
5. 連続失敗時に **state を NaN にする**処理を追加。結合確認で、ERROR 後はハードウェアが `unconfigured` になるが `/joint_states` は 50 Hz で配信され続けることが分かった。そのとき値が前回値のまま流れる可能性があったため NaN にした（修正前の値そのものは確認していない。修正後は NaN になることを確認）。
6. 比較の「最大差」を「A と B の全サンプルの組合せで最悪の差」と定義した（`max(|minA−maxB|, |maxA−minB|)`）。両者に ±1 tick の揺らぎがあると、平均が同じでも 2 tick になりうる（合格基準の上限ちょうど）。
7. `tools/fake_sts3215.py`（偽サーボ）を追加した（結合確認用）。
8. `read_only` の launch 引数は `true` のみ許可（`false` は xacro 側にも来ない。万一渡るとドライバが起動を拒否する）。
9. テスト用の正解表 CSV を 1.3 MB でコミットする（全 raw 値を網羅するため）。

## 残課題（段階2b-2 以降）

- Eye-to-Hand の TF（11/04）と ~/ros2_so101/test の数値は Tzili.json で変換した関節角が前提。新環境で取り直す（段階2e）
- 実機での比較（`docs/feetech_readonly_check.md`）は未実施。サーボ EEPROM の Homing_Offset / limit が json と一致しているかは、実機で起動時診断を見るまで不明。
- 角度の分母 `angle_ticks: 4095`（LeRobot 等価）は物理的には 4096 の可能性がある。誤差は最大でも 1500 tick で約 0.4 tick。段階2c で再検討。
- 書き込みモード（トルク ON、Goal_Position）は段階2b-3（別途承認）。段階2c は Homing_Offset / Min/Max_Position_Limit の書き込み、MIN/MAX の記録手順、yaml への保存が必要（設計は「段階2b-1」）。
- `hardware:=feetech` は 50 Hz の SYNC_READ を 1 サイクルで実行する。実機での往復時間（Return_Delay_Time の現在値を含む）は未測定。失敗が頻発する場合は `timeout_ms` を調整する。

---

# 段階2b-2 追補: `tools/lerobot_readonly_state.py` の位置づけと書き込み禁止の強化（2026-11-06）

## 追加した理由
`hardware:=lerobot` の launch は、接続時に `configure()`（`so101_follower.py:149-171`）がトルクを一度 OFF にして設定を書き、**最後にトルクを ON にする**（`torque_disabled()`、`motors_bus.py:583-597`）。実行中は腕が保持され、手で姿勢を作れない。停止するとトルク OFF（脱力）。
重力の影響を受けない関節（shoulder_pan、wrist_roll、gripper）を**手で動かした姿勢**で比較したいので、LeRobot 側の基準を「トルクに触れずに読むだけ」で取る手段が必要になった。それがこのツール（比較専用。ドライバは依存しない）。
使い分け: P1（折りたたみ姿勢）は launch（基準の定義そのもの）、P2（手で動かした姿勢）はこのツール。詳細とトルクの表は `docs/feetech_readonly_check.md` の 0.5 節。P1 では launch の記録とツールの記録も比較し、ツールが bridge 経路と同じ値を返すことを確かめる手順にした。

## 書き込み禁止の方法（二重）
1. **送信バイト列の検査**（最終防衛線）: `bus.port_handler.writePort` をラップし、送るパケット `FF FF <ID> <LEN> <INST> …` の `INST` が PING(0x01) / READ(0x02) / SYNC_READ(0x82) 以外なら `RuntimeError`（違反時は SDK の `is_using` フラグも戻す）。公開メソッド・低レベルメソッド・SDK 直呼びのどの経路でも、書き込み命令がシリアルに出ない。
2. LeRobot の公開メソッド `write` / `sync_write` / `enable_torque` / `disable_torque` / `write_calibration` / `configure_motors` を、bus インスタンスの属性として `RuntimeError` を投げる関数に差し替える（呼んだ箇所で早く失敗させる）。終了は `disconnect(disable_torque=False)`。既定の `disconnect()` は内部で `self.disable_torque()` を呼ぶので、1 と 2 のどちらでも例外になり書き込まれない。

## 初版の抜けと修正（実装時に見つけた）
初版は 2 だけで、**低レベルの `_write` / `_sync_write` / `_disable_torque`（`setup_motor` などが直接呼ぶ）は塞がれていなかった**。現行のスクリプトはこれらに到達しないが、保証としては弱かったので 1 を追加した。
さらに検証で、違反時に SDK の `is_using` フラグが立ったまま残り、後続のケースが「Port is in use」で**偶然**止まる（ガードの効果ではない）ことが分かったため、違反時にフラグを戻すようにし、検証は各ケースを独立に行うようにした。

## 検証（pty 上の偽サーボ。実機・実ポートは開いていない）
`tools/fake_sts3215.py`（PING / READ / SYNC_READ のみ応答、受けた命令の回数を記録）に対して、conda の Python でスクリプトを実行した。
- 正常系: `connect()`（ping と読み出し）と `sync_read` が通り、CSV を出力。
- 次の 7 経路がすべて**ガードによる例外**で止まった（偽サーボの命令カウンタに書き込み命令は無し。受けたのは PING / READ / SYNC_READ のみ）: `bus.write('Torque_Enable')`、`bus.enable_torque()`、`bus._write(Torque_Enable=1)`（INST=3）、`bus._disable_torque()`（INST=3）、`bus._sync_write(Goal_Position)`（INST=0x83）、SDK 直呼び `packet_handler.write1ByteTxRx(Torque_Enable)`（INST=3）、既定の `bus.disconnect()`。例外の後も読み取りは続けられる。
- 同じ偽サーボ・同じ raw 値で、ツールの CSV（LeRobot 経由の変換）と C++ ドライバ（`hardware:=feetech`）の `/joint_states` の CSV を `compare_joint_states.py` で比較 → 全関節 PASS、差は 0 tick。ソフト側（変換とパケット処理）の等価性の確認であり、実機の確認ではない。

## 未確認
実機でこのツールを実行していない。実機の `connect()` 中に、LeRobot が読み取り以外の命令を送らないことは、ソース確認（`motors_bus.py:414-444`、`feetech.py:156-168` は ping と読み取りのみ）と偽サーボでの確認、およびバイト列検査で担保している。


---

# 段階2b-2 実機確認（2026-11-10）

実機の記録は `~/so_arm101_check/`（リポジトリ外）。私（Claude）は CSV とログを読み、`tools/compare_joint_states.py` を再実行して判定した。実機・シリアルポートは操作していない。実機の操作と手順6のコマンド実行はユーザー。

## 結果: 合格（MISMATCH なし、`read failed` なし）

| 比較 | 結果 | 備考 |
|---|---|---|
| P1: `lerobot_P1`（launch）vs `feetech_P1` | PASS、全関節 0.00 tick | std 0 |
| ツール確認: `lerobot_P1` vs `lerobot_P1_tool` | PASS、全関節 0.00 tick | ツールの値 ＝ bridge 経路の値。P2 の基準に使える |
| P2a: `lerobot_P2a`（ツール）vs `feetech_P2a` | PASS | wrist_roll のみ 平均差 0.97 / 最大差 1.00 tick（基準 1.0 / 2.0 の内側）、他は 0.00 |
| P2b: `lerobot_P2b`（ツール）vs `feetech_P2b` | PASS、全関節 0.00 tick | pan 0.588 rad、roll 1.000 rad、gripper 1.588 rad |
| 起動時診断（3 つの launch ログ） | 6/6 MATCH | 衝撃（腕の落下）のあとでも EEPROM は yaml と一致 |
| 手順6（LeRobot 不使用） | 7 項目すべて想定どおり（**ユーザー報告**。出力ファイルは未確認） | conda 変数は空、プロセス・ノード・トピックなし、maps の該当 0、ldd の該当なし、ttyACM0 を開いているのは ros2_control_node |

姿勢は 3 つ（折りたたみ、pan と roll をマイナス側＋gripper 半開き、pan と roll をプラス側＋gripper 大開き）。`feetech_launch*.log` に `read failed` は 0 件。

## wrist_roll の 0.97 tick
差は raw で 1 tick（-1.034872 vs -1.033388 rad、1 tick = 0.001534 rad）。LeRobot 側 150 サンプルの一部が 1 tick 隣を読んだ（std 0.18 tick）。feetech 側は std 0.00。変換式の誤差なら静止中の全サンプルが同じ向きにずれて std も 0 になるはずなので、**手でひねった直後の摩擦・バックラッシュによる揺らぎ**という説明と矛盾しない。P2b の roll は 0.00 tick で、系統差の証拠は無い。
ただし、ツールと feetech は別の時刻に読んだので、揺らぎと小さな系統差はこのデータだけでは区別できない。判定は PASS のまま。**段階2b-3 以降で再現するなら再検討する。**

## 記録の補足
- 手順書の「P2」を P2a / P2b に分けた。P2b は gripper を「閉じる」でなく「大きく開く」に変更（閉じた状態は P1 で確認済み）。
- feetech の launch ログは P1 / P2a / P2b で別ファイル。LeRobot の launch ログ `lerobot_launch.log` は 2 回目（正常）のもの。

## 未確認
- 書き込み系（トルク ON、Goal_Position）は未実装・未検証（段階2b-3）。
- gripper の変換は 3 点（閉、半開、大開）のみ。pan / roll の端付近（リミット近く）は未確認。
- MoveIt / RViz 経由の実行は対象外（読み取り専用なので指令は破棄される）。
- この結果は `angle_ticks: 4095` を含む LeRobot 等価の変換と一致したことを示す。物理的に 4096 が正しいかは別問題（段階2c）。
- 上の「残課題」にある「実機での比較は未実施」「EEPROM が json と一致しているか不明」は、**この節で解消**（比較は合格、EEPROM は 6/6 MATCH）。

## 1 回目の `hardware:=lerobot` で `/joint_states` が NaN、腕が固定されなかった件
ユーザー報告: 1 回目の起動では腕が固定されず `/joint_states` が NaN。再起動で正常（`/joint_states_raw` 30 Hz）。1 回目のログは残っていない。**原因は未特定**。
候補（確からしい順）:
1. **bridge（Python）が接続に失敗して落ちた**。`bridge.py:103-105` の `robot.connect()` が例外でノード終了 → `joint_states_raw` が来ず、`so101_hardware_bridge.cpp:45-47` で NaN 初期化した位置が更新されない → NaN。`configure()` はトルク ON を最後に行うので、途中で落ちれば腕も固定されない。2 症状を 1 つで説明できる。接続失敗の原因は、USB の切断・再列挙（直前の落下の衝撃）、ポートを掴んだままの前のプロセス、サーボの無応答、`LECONDA_SITE_PACKAGES` 未設定、のいずれか。
2. `publish_joint_states` の例外。NaN の経路としては弱い。
3. ウォッチドッグ `_alive` が時間停止と判断して終了。
4. 起動順の競合。bridge が出れば回復するので、「再起動まで NaN」とは合いにくい。
再発時の確認と、launch ログを `tee` で保存する手順は `docs/feetech_readonly_check.md` の 1 と 3.5 に追記した。

## 手順書の修正（`docs/feetech_readonly_check.md`）
- 手順6を **2.5** に移動（手順2の launch 中に行う必要があるため）。「報告してほしいもの」は 6 に繰り上げ。
- 手順1の launch を `tee` で保存、3.5（NaN／固定されない場合の確認）を追加。
- 手順4を P2a / P2b に合わせて修正（P2b の gripper は大きく開く、別時刻の読み取りによる 1 tick の揺らぎの注意）。

---

# 段階2b-3 の計画（書き込みモード: トルク ON、Goal_Position）— 未承認・未実装

目的: `read_only:=false` で MoveIt / ros2_control の指令をサーボに送る。LeRobot 非依存。URDF・関節リミット・planning_frame・IK solver・SRDF・ros2_controllers.yaml は変えない。安全を最優先（落ちない・急に動かない・限界を超えない）。`read_only` の既定は true のまま。

1. **書く範囲**: RAM のみ（`Torque_Enable`(40)、`Goal_Position`(42)、必要なら `Acceleration`(41)、`Goal_Speed`(46)、`Torque_Limit`(48)）。EEPROM（Homing_Offset、Min/Max_Position_Limit、Operating_Mode、Lock）は書かない（段階2c）。LeRobot の `configure()`（`so101_follower.py:149-171`）が書く項目のうち、電源再投入で戻る RAM がどれかを、起動時診断に読み取り表示（Acceleration、P/I/D、Operating_Mode、Torque_Limit）を足して確認する。アドレスは `tables.py` と突き合わせ、出典を `docs/third_party.md` に追記。
2. **型で閉じ込める**: `Sts3215Bus` は読み取り専用のまま。別クラス `Sts3215WriteBus`（`Write` 0x03、`SyncWrite` 0x83）を追加し、`read_only` の時は生成しない。書けるのは `set_torque()` と `write_goal_positions()` に限り、任意アドレスの汎用 write は公開しない。`read_only:=true` は将来のリーダーアーム（`/leader`、トルク OFF）にそのまま使える。
3. **指令の処理**: NaN / inf は破棄。rad → raw の逆変換を `calibration.cpp` に追加（符号付きエンコード、gripper の range を含む）。yaml の `min_limit` / `max_limit` でクランプ（MoveIt のリミットは変えない、ドライバ側の最終防衛線）。1 周期あたりの最大変化量を制限。全関節を 1 回の `SYNC_WRITE` で送る。
4. **トルク**: 開始は ①現在位置を読む → ②`Goal_Position = 現在位置` → ③`Torque_Enable = 1`（読み取り失敗なら ON にしない）。初期は `Torque_Limit` と加速度を控えめに。終了は既定でトルク OFF（LeRobot と同じ。腕が落ちる）。読み取りが連続失敗したら指令の送信を止めて NaN を流す。
5. **テスト（実機なし）**: WRITE / SYNC_WRITE のバイト列を scservo_sdk の固定ベクタと照合、チェックサム、符号付きエンコード、逆変換の往復、クランプ、レート制限。`tools/fake_sts3215.py` に WRITE / SYNC_WRITE とレジスタメモリを足し、起動順序（Goal → Torque ON）、指令の反映、クランプ、NaN 破棄、`read_only:=true` で書き込み命令が 0 件、を確認。mock と read-only の回帰。段階ごとに `colcon build` → commit。
6. **実機確認（`docs/feetech_write_check.md`、実行はユーザー）**: W0 トルク ON のまま指令なしで 10 秒保持（動かない）→ W1 wrist_roll、gripper を数度 → W2 shoulder_pan を数度 → W3 lift / elbow / wrist_flex を腕を支えて小さく → W4 MoveIt の Plan & Execute（速度小、近い目標）。全段階で手を添え、電源をすぐ切れる状態で。
7. **実装前に決めること**: 終了時の扱い（トルク OFF / 現在位置で保持 / 折りたたみへ）、異常時にトルクを保つか切るか、初期の `Torque_Limit`・`Acceleration`・1 周期の最大変化量、LeRobot が書いている RAM 設定（P/I/D、加速度）を自前で書くかサーボの現在値のままにするか。
