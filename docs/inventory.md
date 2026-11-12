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

# 段階2b-3 の計画 v3（書き込みモード: トルク ON、Goal_Position）— 承認済み（2026-11-10）、2b-3a から実装

2b-3 は 3 つに分けて、それぞれ `colcon build`（と `colcon test`）が通ってから commit する。2b-3a は読み取りだけ。2b-3b で初めて書き込みを入れる。
目的: `read_only:=false` で MoveIt / ros2_control の指令をサーボに送る。LeRobot 非依存。URDF・関節リミット・planning_frame・IK solver・SRDF・`joint_limits.yaml`・`controllers.yaml` は変えない。`read_only` の既定は true。安全を最優先（落ちない・急に動かない・限界を超えない）。

## 承認された決定
1. 終了時は LeRobot と同じ**トルク OFF**。トルク ON のままの終了は作らない。手順書に「**止める前に折りたたみ姿勢へ動かし、腕を支えて Ctrl-C**」を明記する。「ゆっくり折りたたむ」機能は 2b-3 では作らない（将来の課題）。
2. 異常時: 通信が切れたら**トルク OFF も送れない。サーボは最後の目標で保持し続ける**（この前提は実機で未確認）。ドライバは指令の送信を止めて NaN を流し、ログで明示する。書き込みが通る異常（指令値の異常）では、送信を止めてトルクは保持する。
3. 数値は LeRobot の `configure()` の値を出発点にする（下の表）。
4. 自前で書く。ただし `apply_servo_config` は**既定で拒否**（`true` を明示したときだけ EEPROM レジスタを書く）。Operating_Mode は**検証のみ**で書かない。0 以外ならトルクを ON にせず起動を中止する。
5. 2b-3a（読み取りだけの実機確認）を先に行う。

## 訂正: EEPROM と SRAM（LeRobot `tables.py` の区分、推定）
アドレス 39 以下は EEPROM（電源を切っても残る）。P/I/D、Operating_Mode、Return_Delay_Time、gripper の Max_Torque_Limit / Protection_Current / Overload_Torque は EEPROM。電源再投入で既定値に戻るのは SRAM（`Acceleration` 41、`Torque_Limit` 48 など）。ソースの区分からの推定で、**サーボの電源を入れ直して読むまで未確認**（2b-3a で確かめる）。

## LeRobot が書く値（`so101_follower.py` `configure()` 149-171 行、`feetech.py` `configure_motors()` 222-230 行、`enable/disable_torque` 296-310 行）
トルク OFF・Lock=0 の間に書き、終了時にトルク ON・Lock=1。

| レジスタ（アドレス） | 区分 | 値 | 対象 |
|---|---|---|---|
| Return_Delay_Time (7) | EEPROM | 0 | 全関節 |
| Maximum_Acceleration (85) | Factory 表（区分不明） | 254 | 全関節 |
| Acceleration (41) | SRAM | 254 | 全関節 |
| Operating_Mode (33) | EEPROM | 0（位置制御） | 全関節 |
| P / I / D (21 / 23 / 22) | EEPROM | 16 / 0 / 32（既定の P は 32） | 全関節 |
| Max_Torque_Limit (16) | EEPROM | 500（最大 1000 の 50%） | gripper のみ |
| Protection_Current (28) | EEPROM | 250（最大 500 の 50%） | gripper のみ |
| Overload_Torque (36) | EEPROM | 25 | gripper のみ |
| Torque_Limit (48)、Goal_Velocity (46) | SRAM | **書かない** | 腕は電源投入時の値のまま |

## 初期値の提案
| 項目 | 値 | 理由 |
|---|---|---|
| Operating_Mode | 0 を要求（書かない） | 0 以外ではトルク ON しない |
| P / I / D、Return_Delay_Time、gripper の 3 項目 | LeRobot と同じ（検証し、`apply_servo_config:=true` のときだけ書く） | 実績のある値 |
| Acceleration（RAM） | 50（パラメータ。実績の 254 へは動作確認後） | 単位は未確認（STS の資料では 100 step/s²） |
| Torque_Limit（RAM） | lift / elbow / wrist_flex: **1000（下げない。下げるとトルク ON でも沈む）**。pan / roll: 1000、W1〜W2 の初回だけパラメータで 500。gripper: 500 | 安全は速度・加速度・変化量の制限と腕を支えることで取る |
| Goal_Velocity | 書かない | 既定値の意味が未確認。2b-3a の読み取りで決める |
| Lock | 起動完了時に 1。0 は設定書き込みの間だけ | LeRobot と同じ |

## 1 周期あたりの最大変化量（v3 で訂正）
`controllers.yaml` の確認で分かったこと: `arm_controller` は **`update_rate: 25`**（controller_manager は 50 Hz）、`open_loop_control: true`（43 行）、許容誤差の設定なし。JTC は 25 Hz でしか指令を更新しないので、**ハードウェアの 50 Hz の周期では、2 周期に 1 回、40 ms ぶんの動きが一度に来る**。v2 の「19 tick/周期」はこれを無視していて、スケーリング 0.3 を超えると正当な軌道を削る値だった。open_loop のため、削られても追従誤差で気づけない。
前提: 1 tick = 2π/4095 = 0.001534 rad（角度関節 651.7 tick/rad）。gripper は 100% = 1485 tick、1 rad = 57.30% なので 850.8 tick/rad。`joint_limits.yaml` の `max_velocity` は全関節 5.0 rad/s、スケーリング 1.0。

| 場合 | 窓 [s] | 角度関節 [rad / tick] | gripper [tick] |
|---|---|---|---|
| MoveIt の既定（スケーリング 0.1 → 0.5 rad/s）、JTC 1 更新 | 0.04 | 0.02 / 13.0 | 17.0 |
| **スケーリング 1.0（5.0 rad/s）、JTC 1 更新** | 0.04 | 0.20 / 130.3 | 170.2 |
| **スケーリング 1.0、JTC の更新が 1 回遅れた（2 更新ぶん）** | 0.08 | 0.40 / 260.7 | 340.3 |
| **提案の制限: 6.0 rad/s × 0.08 s** | 0.08 | **0.48 / 312.8** | **408.4** |

制限は最悪の場合（2 更新ぶん）の 1.2 倍、通常の 1 更新ぶんの 2.4 倍。`max_joint_speed`（6.0 rad/s）と `limit_window_s`（0.08 s）はパラメータ。試験で `joint_limits.yaml` の `max_velocity` と `controllers.yaml` の `update_rate` を読み、`max_joint_speed × limit_window_s ≥ 1.2 × max_velocity × 2 / update_rate` を確認する（どちらかを変えたら試験が落ちる）。
この制限は**飛び（NaN の誤変換、軌道の異常）を止めるためのもの**で、通常の速度の安全は MoveIt の速度スケーリング（既定 0.1）、サーボの Acceleration、腕を支えることで取る。
制限が実際に効いたら **WARN**（関節名、要求した変化量、制限、効いた回数。同じ関節は 5 秒に 1 回に間引く）。deactivate 時に効いた回数の合計も出す。**gripper の GripperActionController は目標へ一気に飛ぶ指令を出すので、gripper の WARN は動作のたびに出る想定**（ランプ状に追従させるのが意図。メッセージに `ramping` と書く）。
基準は直前に送った目標。起動時は `hw_commands_` を現在位置にして始める（JTC が最初の指令を現在位置から出せる）。

## 異常時の分類
| ケース | 動作 | 腕 | 復帰 |
|---|---|---|---|
| A. バスが死んだ | 連続 10 回の読み失敗で送信停止、NaN。ERROR: 「通信断。サーボは最後の目標でトルク ON のまま。ドライバから OFF にできない。電源を切るか再接続」 | ON のまま（制御不能） | launch をやり直す。起動時は 現在位置を読む → Goal=現在位置 → トルク ON |
| B. 読みだけ失敗／間欠 | 1 回再試行、失敗した周期は前回値を保持。連続 10 回で A | ON | 読めたら通常に戻る |
| C. 書き込みは通るが指令が異常 | NaN／inf は破棄して直前の目標を保持。限界外はクランプ、飛びは変化量を制限（WARN）。異常が 50 周期（1 秒）続いたらフォルトにして送信停止。ERROR: 「送信停止。トルクは保持」 | ON のまま保持 | フォルトは自動解除しない。deactivate するとトルク OFF |
| **D. サーボ側の保護（ERR バイト）** | **ERR を読んだら送信を止める。再開は起動し直したときだけ**（deactivate→activate では解除しない。保護が解けた瞬間に溜まった指令で急に動かないため）。ERROR にビットの内容。 | サーボが自分でトルクを切ることがある | launch のやり直し（プロセスの再起動）だけ。起動時は現在位置から始める |
任意（必須にしない）: 1 Hz で `Torque_Enable`・温度・負荷を読む監視。

## 設定不一致の扱い（v3 で確定）
サーボの EEPROM が LeRobot の値（上の表）と違うとき:
- **`read_only:=false`**: **起動を中止**する（ログに差を表示）。`apply_servo_config:=true` のときだけ、トルク OFF → `Lock=0` → 書く → `Lock=1` → 読み戻し、の順で書く（同じ値なら書かない）。Operating_Mode は書かない。0 以外ならどちらの場合も起動中止。
- **`read_only:=true`**: **WARN のみ**で起動を続ける（2b-3a の確認と、将来のリーダーアームのため）。Operating_Mode が 0 以外でも WARN のみ（トルクは ON にしない）。

## 実装の構成
### 2b-3a: 読み取りだけの準備（書き込みなし）
- 起動時診断に読み取りを追加: Operating_Mode、P/I/D、Return_Delay_Time、Max_Torque_Limit、Protection_Current、Overload_Torque、Acceleration、Torque_Limit、Goal_Velocity、Maximum_Acceleration、Torque_Enable、Lock、Status、電圧、温度。LeRobot の値との比較（MATCH／MISMATCH、`read_only` なら WARN）。
- rad → raw の逆変換 `rad_to_raw()`（`calibration.cpp`）。LeRobot の `_unnormalize` と一致する正解表（`tools/gen_conversion_golden.py` を拡張）と試験。
- 実機確認の手順を `docs/feetech_servo_config_check.md` に書く（サーボの電源を入れ直してから読み取り専用で起動し、診断を報告する）。
### 2b-3b: 書き込み
- `Sts3215WriteBus`（別クラス・別ヘッダ）に `Write`(0x03)、`SyncWrite`(0x83)。`Sts3215Bus` は読み取り専用のまま。`read_only` の時は生成しない。公開する書き込みは用途別の関数だけ（任意アドレスの汎用 write は出さない）。
- 起動: 診断（Operating_Mode≠0 なら中止）→ 設定の検証（不一致なら中止）→ RAM の Acceleration／Torque_Limit → 現在位置を読む → Goal=現在位置 → Torque_Enable=1、Lock=1（読み戻し）。失敗したらトルクを ON にしない。
- write(): NaN／inf 破棄 → rad→raw → yaml の `min`/`max` でクランプ → 変化量の制限（WARN）→ 1 回の `SYNC_WRITE`。ERR を読んだら送信停止（D）。
- 終了: `on_deactivate` / `on_cleanup` / デストラクタからトルク OFF（読み戻して確認、3 回再試行、失敗は ERROR）。
- 試験: バイト列を scservo_sdk と照合、符号付きエンコード、クランプ、変化量の制限（合成した 25 Hz のステップ指令が削られないこと）、設定ファイルの整合試験、異常ケース A〜D、`read_only:=true` で書き込み命令が 0 件。`tools/fake_sts3215.py` を WRITE 対応にする。
### 2b-3c: ドキュメント
- `docs/feetech_write_check.md`（W0 保持のみ 10 秒 → W1 wrist_roll・gripper → W2 pan → W3 lift/elbow/wrist_flex（腕を支える）→ W4 MoveIt（スケーリング 0.1）。通信断の確認は最後に 1 回。各段階の終わりに「折りたたみ姿勢へ動かし、腕を支えて Ctrl-C」。強制終了したら電源を切る）、`docs/third_party.md`（LeRobot の出典と書いた値、STS3215 の仕様書は原本未確認と明記）。

## 未確認として残るもの
EEPROM／SRAM の区分（2b-3a で確認）、電源投入時の Torque_Limit・Acceleration・Goal_Velocity の既定値と Acceleration の単位、通信断でサーボが最後の目標を保持し続けること、JTC が実際に 25 Hz で指令を更新していること（2b-3b の確認項目）、Maximum_Acceleration(85) の区分（書かない）。

## 将来の課題
ゆっくり折りたたみ姿勢へ動かして終了する機能。1 Hz のサーボ状態監視。

---

# 段階2b-3a: 読み取りだけの準備（2026-11-10）

書き込みは入れていない。サーボ・実ポートは操作していない（確認は pty 上の偽サーボとユニットテスト）。

## 追加・変更したファイル
- `include/so_arm101_hardware/sts3215_bus.hpp`: 読むだけのレジスタ定数を追加（Operating_Mode、P/I/D、Max_Torque_Limit、Protection_Current、Overload_Torque、Torque_Enable、Acceleration、Goal_Velocity、Torque_Limit、Lock、電圧、温度、Status、Maximum_Acceleration）。`Instruction` は Read / SyncRead のまま（書き込み命令は型に存在しない）。
- `servo_config.hpp` / `servo_config.cpp`（新規）: LeRobot の値との比較（`expected_servo_config`、`compare_servo_config`）と、不一致の扱い `config_action(diffs, read_only)`。
- `sts3215_system.cpp`: `run_servo_config_diagnostics()` を `on_configure` の診断に追加。各関節について `[diag] <joint> id=N EEPROM: ... | SRAM: ... | other: ...: MATCH (LeRobot)` または `DIFFERS from LeRobot values: ...`、最後に `servo settings vs LeRobot: n/6 joints match`。**`read_only:=true` では不一致は WARN のみで起動を続ける。`read_only:=false` では ERROR で起動を中止する**（この分岐は 2b-3b で使う。今は `read_only:=false` 自体を拒否している）。
- `calibration.cpp`: `rad_to_raw()`、`rad_to_raw_continuous()`（Goal_Position 用の逆変換。まだ書き込みには使っていない）。
- `tools/gen_conversion_golden.py`: `MotorsBus._unnormalize()` の正解表 `test/data/unnormalize_golden.csv`（6 関節 × 約 1070 行）を作る。`conversion_golden.csv` は変更なし。
- `tools/fake_sts3215.py`: LeRobot の `configure()` 後の設定を返し、`--reg ID:ADDR:LEN=値` で上書きできる。
- `test/test_servo_config.cpp`（新規 7 件）、`test/test_conversion.cpp`（逆変換 4 件を追加）。
- `docs/feetech_servo_config_check.md`（実機確認の手順）、`docs/third_party.md`（出典の追記）。

## 逆変換の仕様（LeRobot との差）
式は LeRobot の `_unnormalize` と同じ（angle: `val·ticks/360 + center`、gripper: 0..100 に clamp して min..max）。**最後の丸めだけ四捨五入**（LeRobot は `int()` の切り捨て）。理由: raw→rad→raw が恒等になり、保持指令が 1 tick ずれない。切り捨てとの差は 0 か +1 tick（正の範囲）。angle は clamp しない（限界のクランプは 2b-3b の write()）。

## 検証
- `colcon build`（7 パッケージ）、`colcon test --packages-select so_arm101_hardware`: 36 件すべて成功。
- 既存の変換（raw→rad）の正解表との一致は変わらず（最大誤差 0）。
- 逆変換: 6 関節 × 約 1070 値で LeRobot の `_unnormalize` と比べ、差は 0 か +1 tick（負の raw になる範囲外は 0 か −1）。丸める前の連続値は LeRobot の式と一致。範囲内の全 raw で raw→rad→raw が恒等（gripper は min..max）。
- pty の偽サーボに `hardware:=feetech` を起動: 設定が LeRobot と同じ 4 関節は `MATCH (LeRobot)`、ID 2 の P を 32、gripper の Max_Torque_Limit を 1000 にした 2 関節は `DIFFERS from LeRobot values: P_Coefficient(servo 32, expected 16)` などの WARN を出して**起動を続けた**（`4/6 joints match`）。偽サーボが受けた命令は READ（0x02）と SYNC_READ（0x82）だけ（`UNEXPECTED INSTRUCTION` なし）。

## 未確認（実機）
- EEPROM／SRAM の区分、電源投入時の `Torque_Limit`・`Acceleration`・`Goal_Velocity`・`Lock` の値、`Acceleration` の単位。手順は `docs/feetech_servo_config_check.md`（LeRobot 設定直後 A と電源再投入直後 B を読んで比べる）。
- 実機の診断で、全 17 レジスタの読み取りが 1 ms 前後で完了し、`on_configure` が長引かないこと（偽サーボでは 6 関節で約 8 ms）。
- 診断に使う読み取りの追加でサーボが応答しなくなる、などの副作用が無いこと（読み取りのみ。LeRobot の `read` と同じ命令）。

## 次
実機確認の結果（A と B の診断行）を受けて、2b-3b の初期値（Acceleration、Torque_Limit、Goal_Velocity の扱い）を確定する。2b-3b の実装は、その結果を見てから。

---

# 段階2b-3a 実機確認（2026-11-10）

実機の記録は `~/so_arm101_check/`（リポジトリ外）。私（Claude）はログを読んで判定した。実機・シリアルポート・launch は操作していない。実機の操作はユーザー。
ログ: `servo_config_A.log`（LeRobot 設定直後）、`servo_config_B.log`（サーボ電源の入れ直し直後）、LeRobot 起動ログは **`servo_config_lerobot_la`**（ファイル名が途中で切れている。手順書の名前は `servo_config_lerobot_launch.log`）。

## 結果: 合格
- A・B とも **6/6 MATCH (LeRobot)**。`status=0x0`、電圧 11.8〜11.9 V、温度 28〜34 ℃。
- A・B とも `read failed` は 0 件。コントローラ 3 つが active（A・B・LeRobot 起動ログとも `Configured and activated` が 3 件）。診断の読み取り追加による副作用は見られない。
- 診断の所要時間: 1 関節あたり約 8.5 ms、6 関節で約 44 ms（`on_configure` のみ。事前の見積り「1 ms 前後」より長いが問題ない）。

## A と B の比較
| 項目 | A（LeRobot 直後） | B（電源投入直後） | 判定 |
|---|---|---|---|
| P / I / D | 16 / 0 / 32 | 同じ | **EEPROM と確定**（P は既定 32 から変更した値が残った） |
| gripper の Max_Torque_Limit / Protection_Current / Overload_Torque | 500 / 250 / 25 | 同じ | **EEPROM と確定**（腕の 1000 / 310 / 80 と違う値が残った） |
| Operating_Mode、Return_Delay_Time | 0、0 | 同じ | EEPROM と矛盾しない。ただし 0 は既定値の可能性があり、**これだけでは区別できない** |
| Maximum_Acceleration | 254 | 254 | 電源を切っても残った（EEPROM 相当）。工場出荷値も 254 の可能性があり区別できない。書かない |
| **Acceleration** | **254** | **0** | **SRAM と確定。電源投入時の既定値は 0** |
| **Lock** | **0** | **1** | **SRAM と確定。電源投入時の既定値は 1**。LeRobot の停止後は Torque_Enable=0・Lock=0 のまま残る |
| Torque_Limit | 1000（gripper 500） | 同じ | **区別できない**。電源投入時の値は Max_Torque_Limit と同じ（1000 / gripper 500）で、「腕は 1000 のまま」の前提と一致 |
| Goal_Velocity | 0 | 0 | 区別できない。LeRobot は書かないので、0 が電源投入時の値 |
| Torque_Enable | 0 | 0 | 区別できない（LeRobot の停止で 0 になるため） |

## Acceleration=0 の意味（確度つき）
- **この環境で確認できた出典**: `tables.py:100` のコメント `Acceleration_Multiplier: Acceleration multiplier in effect when acceleration is 0`（レジスタ 86）。`feetech.py:227` のコメント `Set Maximum_Acceleration to 254 to speedup acceleration and deceleration`。**どちらも「0 = 制限なし」とは書いていない**。0 のときに別の係数（86）が効く、という読み方もでき、「加速度制御なし」と「別の加速度で動く」を区別できない。
- 記憶による情報（Feetech の SCServo SDK／資料で ACC は 0〜254、0 は特別扱い、単位は 100 step/s²）は**原本を見ていないので確度は低い**。
- 実機で見えた事実: B の 0 は電源投入時の値。**0 のときの動き方は観測していない**（254 は LeRobot で動作済み）。
- 結論: **0 の意味は未確認**。「0 = 制限なし（目標へ最大の勢いで動く）」の可能性を排除できないので、**0 のままトルクを ON にしてはいけない**として設計する。
- 参考（実機で観測済み）: Goal_Velocity=0 のまま LeRobot で腕が目標へ動いている（段階2a）ので、Goal_Velocity=0 は速度停止ではない。速度の上限が無いなら、動きの滑らかさは Acceleration と指令側の制限だけで決まる。

## 判明した設計上の影響
1. サーボの電源が一時的に落ちて復帰すると、Acceleration=0・Lock=1・Torque_Enable=0 に戻る（腕は脱力）。ドライバが動き続けていても、トルクは勝手には戻らない。これは**検出して止める**必要がある（下の 2b-3b 計画）。
2. 停止後の Lock=0 と、電源投入後の Lock=1 の両方から始まりうる。起動は、どちらの状態でも同じ手順で安全に進める必要がある。

## 未確認として残るもの
- Acceleration=0 の動作、Acceleration の単位。
- Torque_Limit(48) / Goal_Velocity(46) が SRAM か（このデータでは不明。設計は区分に依存しない）。
- Lock=1 のまま SRAM（Acceleration など）へ書けるか。
- 通信断でサーボが最後の目標を保持し続けること、JTC が 25 Hz で指令を更新していること（2b-3b）。

---

# 段階2b-3b の計画 v4（書き込みモード）— 承認済み（2026-11-10）

v3（上の「段階2b-3 の計画 v3」）からの差分。v3 の表・異常時の分類 A〜D・変化量の制限・設定不一致の扱いはそのまま有効。根拠は「段階2b-3a 実機確認」。
承認された確認事項: ① Acceleration 既定 50（単位は未確認。W1 で調整）、② Goal_Velocity≠0 なら中止（書き直さない）、③ 1 Hz 監視は必須、④ W0 に「Lock=1 での SRAM 書き込み確認」を入れる。追加の 2 点（⑤ 終了時の Lock=1、⑥ 監視はスレッドを増やさない）も承認。

## 初期値
| 項目 | 扱い |
|---|---|
| Acceleration | **トルク ON の前に毎回書く**（値を保持・省略しない）。既定 50（`acceleration`）。読み戻して不一致か 0 なら**トルクを ON にしない**。電源投入時は 0 で、0 の意味は未確認（「0 = 制限なし」の可能性を排除できない）。Maximum_Acceleration 以下であることも確認 |
| Torque_Limit | 腕 1000（`torque_limit`。W1〜W2 の初回だけ 500）、gripper は min(値, 500)。現在値と違うときだけ書き、読み戻す。EEPROM／SRAM のどちらでも成り立つ |
| Goal_Velocity | 書かない。0 でなければトルクを ON にせず中止 |
| Maximum_Acceleration | 書かない |
| Operating_Mode | 書かない。0 以外なら中止 |

## 起動の手順（どの状態から始めても同じ。`on_activate`）
1. 読む: Operating_Mode=0、Goal_Velocity=0、Torque_Enable（1 なら WARN。前回が異常終了した可能性）。違えば中止（まだ何も書いていない）。
2. Lock=0（読み戻し）。
3. `apply_servo_config:=true` で差があるときだけ: Torque_Enable=0 を確認 → EEPROM の項目を書く（P/I/D、Return_Delay_Time、gripper の 3 項目。Operating_Mode は書かない）→ 読み戻し。トルク ON のままなら中止。
4. Acceleration を書く（読み戻し。0 や不一致は中止）。Torque_Limit は違うときだけ書く（読み戻し）。
5. 現在位置を読み、Goal_Position=現在位置を書く（読み戻し）。
6. Torque_Enable=1（読み戻し）。
7. Lock=1（読み戻し）。
8. 全項目を読み直して検証。`probe_lock_sram:=true` のときだけ確認 1: Acceleration を 1 だけずらして書き、読み戻し、元に戻して読み戻す（腕は動かず、EEPROM に触れない）。
途中で失敗したら、書き込みを始めていれば下の「終了」の手順を実行する。トルクは ON にならない（6 より前）か、OFF に戻る。

## 終了（⑤）
`on_deactivate`、`on_cleanup`、デストラクタ、起動の失敗時（書き込みを始めていたとき）のすべてで、同じ手順:
1. 全関節の Torque_Enable=0（読み戻し、3 回まで再試行）。
2. その後、全関節の **Lock=1 に戻す（読み戻し、3 回まで再試行）**。LeRobot は Lock=0 のまま残していた。EEPROM を保護するため。
3. 結果を INFO（成功）／ERROR（失敗した関節）で出す。2 度目以降の呼び出しは何もしない。
異常で終了する場合も、書き込みが通るなら同じ手順。通信が切れていれば出来ない（ケース A のとおり、ERROR に明記）。

## 1 Hz 監視（③⑥）
- 内容: Torque_Enable(40) と Acceleration(41) の 2 バイトを全関節に 1 回の SYNC_READ（`monitor_period_s`、既定 1.0）。期待は Torque_Enable=1 と、起動時に書いた Acceleration。
- 違えば**フォルト**にして送信を停止（ERROR「サーボの電源断または再起動。自動では復帰しない」）。ERR バイトを読んだ場合も同じ（ケース D）。監視の読み取りが 3 回連続で失敗してもフォルト。
- **スレッド・タイマー・非同期は使わない**。監視は `read()` の中で、位置の読み取りが終わった後に同じ周期処理の一部として行う。ドライバのコードに `std::thread`／`std::async`／`pthread`／`create_wall_timer` を使わない。
- シリアルポートの同時使用を防ぐ二重の仕組み: ① `SerialPort` に再入可能な mutex を持たせ、バスの 1 往復（要求の送信と応答の受信）を `Transaction` として mutex で守る。② ドライバの `read`／`write`／`on_*`／デストラクタを 1 本の mutex で守る（ros2_control の周期処理と lifecycle の呼び出しが別スレッドになりうるため）。
- 試験: ソースに禁止語が無いこと（ソース走査）、2 スレッドが同時にバスを叩いても応答が壊れず（mutex が無ければ壊れる）、`Transaction` の競合回数が数えられること、監視付きで長く動かしても競合が 0 であること。

## 変えないもの
URDF・関節リミット・planning_frame・IK solver・SRDF・`joint_limits.yaml`・`controllers.yaml`。`read_only` の既定は true。`read_only:=true` では書き込み用のクラスを生成せず、書き込み命令は 0 件。終了時はトルク OFF。変化量の制限（6.0 rad/s × 0.08 s）、異常の分類 A〜D。

## 試験の方針
偽サーボ（pty）で次を自動試験にする: 書き込み順（起動／終了）、Acceleration=0 または読み戻し不一致でのトルク ON 拒否、Goal_Velocity≠0 での中止、サーボのリセット（SRAM が既定値に戻る）の検出と送信停止、`read_only:=true` で書き込み 0 件、終了時の Lock=1、異常終了時の後始末、バイト列の照合（SYNC_WRITE／WRITE）。

---

# 段階2b-3b: 書き込みモードの実装（2026-11-10）

実機（サーボ）は動かしていない。実ポート（`/dev/ttyACM0`）も開いていない。確認は pty 上の偽サーボ（C++ の試験用と `tools/fake_sts3215.py`）。
計画は「段階2b-3b の計画 v4」。**書き込みを行うのは `read_only:=false` のときだけ**（既定は true）。

## 追加・変更したファイル
| ファイル | 内容 |
|---|---|
| `include/so_arm101_hardware/sts3215_write_bus.hpp`、`src/sts3215_write_bus.cpp`（新規） | `Sts3215WriteBus`（`Sts3215Bus` を継承）。WRITE(0x03)／SYNC_WRITE(0x83)。公開するのは用途別の関数だけ（`write_torque_enable`、`write_lock`、`write_acceleration`（0〜254）、`write_torque_limit`（0〜1000）、`write_goal_position`、`sync_write_goal_position`、`write_servo_setting`）。`write_servo_setting` は allow-list（P/I/D、Return_Delay_Time、Max_Torque_Limit、Protection_Current、Overload_Torque）。**Operating_Mode は型として書けない**。任意アドレスの汎用 write は無い |
| `sts3215_bus.{hpp,cpp}` | 読み取り専用のまま（`Instruction` は Read／SyncRead）。派生が使う `build_packet`／`receive_status`／`fail` を protected に。ERR バイトを保持し、`describe_servo_error()` で内容を表示。仮想デストラクタ。`encode_sign_magnitude()` |
| `serial_port.{hpp,cpp}` | `SerialPort::Transaction`（再入可能 mutex）。バスの 1 往復をポート単位で排他にする。待たされた回数を `contention_count()` で数える |
| `sts3215_system.{hpp,cpp}` | 書き込みモード。起動（手順 1〜8）、`write()`（NaN 破棄 → rad→raw → 校正範囲でクランプ → 1 周期の変化量の制限 → SYNC_WRITE）、1 Hz 監視、フォルト（A〜D）、終了の手順。`read_only` のときは `Sts3215Bus` だけを生成（書き込み用クラスは存在しない）。read／write／on_*／デストラクタを 1 本の mutex で守る |
| `servo_config.{hpp,cpp}` | `ConfigAction::Apply`、`config_action(diffs, read_only, apply_servo_config)` |
| `so_arm101_moveit_config/config/so101_new_calib.{urdf,ros2_control}.xacro` | `acceleration`、`torque_limit`、`apply_servo_config`、`probe_lock_sram` の 4 引数を追加し、feetech の `<param>` に渡すだけ。関節・リンク・limit は不変。mock のとき無視（展開結果で確認） |
| `so_arm101_bringup/launch/bringup.launch.py` | `read_only` に `false` を許可。上の 4 引数を追加。既定は `read_only:=true` |
| `tools/fake_sts3215.py` | 書き込み対応・状態を持つ。電源投入直後の状態（Acceleration=0、Lock=1、Torque_Enable=0、Goal_Position=0）から始める。EEPROM は Lock=1 のとき書けない。`reset` ファイルでサーボの電源再投入を模擬。`--strict-readonly`。書き込みを `writes.jsonl` に記録 |
| `tools/send_joint_delta.py` | 1 関節を現在位置からゆっくり動かす補助（実機用。**実行していない**。\|delta\|≤0.5 rad、≤0.1 rad/s を超える指定は拒否） |
| `test/fake_servo.hpp`、`test_write_bus.cpp`、`test_sts3215_system.cpp`、`test_servo_config.cpp`（追記） | 下の試験 |
| `docs/feetech_write_check.md` | 2b-3c の実機確認手順（W0〜W4、任意の W5）。各段階に「腕の支え方」「止め方」 |
| `docs/third_party.md` | 書き込む値と順序の出典、LeRobot との違い（Acceleration=50、終了時 Lock=1） |

## 仕様の確定事項（v4 からの補足）
- **起動**: v4 の手順 1〜8 のとおり。手順 1（読むだけ）で、Operating_Mode≠0、Goal_Velocity≠0、設定が LeRobot の値と違う（`apply_servo_config` なし）、`acceleration` が Maximum_Acceleration を超える、のいずれでも**何も書かずに中止**。Acceleration は毎回書き、読み戻しが 0 か不一致ならトルクを ON にしない。Torque_Enable がすでに 1 なら WARN（前回の異常終了の可能性）して手順を続ける。
- **Torque_Limit**: `torque_limit` が効くのは **shoulder_pan と wrist_roll だけ**（v3 の取り決めどおり。lift / elbow / wrist_flex は常に 1000、gripper は min(値, 500)）。現在値と違うときだけ書く。
- **1 周期の変化量の制限**: 6.0 rad/s × 0.08 s ＝ 0.48 rad（角度関節 312 tick、`tick_in_rad` から算出）。試験が `joint_limits.yaml` と `controllers.yaml` を読み、`max_joint_speed × limit_window_s ≥ 1.2 × max_velocity × 2 / update_rate` を確認（現在の値でちょうど満たす）。WARN は関節ごとに 5 秒に 1 回。
- **異常の数え方（追加した決め）**: 「異常な指令」は NaN／inf、または校正範囲を **50 tick（≈0.077 rad）より大きく**超えた指令。範囲を少し超えるだけの指令（関節が限界で静止しているとき等）は黙ってクランプし、異常と数えない。50 周期（1 秒）続くとフォルト（送信停止。トルクは保持）。
- **フォルトの復帰**: 監視／指令／書き込みのフォルトは deactivate → activate で解除できる（起動の手順が電源投入直後と同じ状態から安全に進むため）。**サーボの ERR バイトを読んだときはプロセスの再起動まで解除しない**（`on_activate` が拒否）。
- **フォルトでも `read()` は OK を返す**（ERROR を返すと controller_manager が hardware を deactivate してトルクが OFF になるため。ケース C は「トルク保持」）。通信断（連続 10 回の読み失敗）だけが ERROR で、`COMMUNICATION LOST` を出す。
- **終了の手順**: トルク OFF（読み戻し、3 回まで）→ Lock=1（読み戻し、3 回まで）。`on_deactivate`／`on_cleanup`／デストラクタ／起動の失敗時。成功すると 2 度目は何もしない。通信が死んでいるときは終了のたびに待たないよう、失敗は 2 回までで打ち切る。
- **`read_only` の厳密な解釈（試験中に見つけた穴を塞いだ）**: 初版は「true 以外は false」で、`read_only` の打ち間違い（`ture`、`1` 等）が書き込みモードになりうる状態だった。**`true`／`false`（大文字小文字は問わない）以外は起動を拒否**する（`apply_servo_config`、`probe_lock_sram` も同じ）。
- **別スレッド・タイマー無し**: 監視は `read()` の中（位置の読み取りの後）。ドライバのソースに `std::thread`／`std::async`／`pthread`／`create_wall_timer` 等が無いことを試験が走査して確認する。

## 検証
- `colcon build --symlink-install`（7 パッケージ）、`colcon test --packages-select so_arm101_hardware`: **73 件すべて成功**（このうち今回の追加は、書き込みバス `test_write_bus` 10、ドライバ `test_sts3215_system` 24、設定 `test_servo_config` 1）。5 回連続で実行して安定。
- **書き込み順**（偽サーボの書き込み記録）: 最初が Lock=0 → Acceleration（全関節 50。0 は一度も書かれない）→ Goal_Position（現在位置）→ Torque_Enable=1 → Lock=1。Torque_Limit は既定値と同じなら書かない。Operating_Mode と Goal_Velocity には書かない。
- **Acceleration=0 の拒否**: Acceleration が書けない偽サーボ（0 のまま）で、起動は ERROR、**Torque_Enable=1 は一度も書かれず**、終了の手順で Lock=1 に戻る。Goal_Position の読み戻し不一致、Goal_Velocity≠0（診断で／診断後に変わった場合）、Operating_Mode≠0（`apply_servo_config` でも）でも同様に拒否（後者 2 つは何も書かない）。
- **リセット検出**: 動作中にサーボを電源再投入の状態（Acceleration=0、Lock=1、Torque_Enable=0）にすると、`monitor_period_s`+余裕（0.14 s）以内にフォルト（`powered off or reset`）、**以後 Goal_Position を送らない**。ERR バイトは再起動までの latch。
- **終了**: deactivate／cleanup／デストラクタのいずれでも「全関節トルク OFF → その後 Lock=1」（書き込み記録で順序を確認）。フォルト後の deactivate でも Lock=1。
- **`read_only:=true`**: 200 周期、指令あり、deactivate／cleanup まで通して、**書き込み記録が 0 件、受けた命令は 0x02 と 0x82 だけ**、サーボの Torque_Enable=0・Acceleration=0（電源投入時のまま）・Lock=1。
- **ポートの同時使用**: ① ソース走査で禁止語なし。② 2 スレッドが同じポートを 300 往復ずつ叩いても全成功、要求パケットのチェックサム不良 0。**排他を一時的に無効にして同じ試験を実行したところ失敗した（1724 件）ので、試験は排他を検証できている**（無効化は元に戻した。最終版で 10 件成功）。③ 監視付きで 10 秒ぶん（500 周期）動かして `port_contention=0`、フォルトなし。
- **launch 全体（pty 上の偽サーボ。ros2_control_node は実際のスレッド構成）**: `mock:=false hardware:=feetech read_only:=false` で、起動 → 保持 → Ctrl-C（SIGINT）で「トルク OFF → Lock=1」を確認。15 秒で `reads=777 goal_writes=777 monitors=15 rate_limited=0 port_contention=0`。`FollowJointTrajectory`（pan +0.3 rad / 2 s）は SUCCEEDED、偽サーボの Goal_Position は 2097 → 2293（＝+196 tick ＝ 0.3 rad）へ 4 tick ずつ増加。動作中にサーボのリセットを模擬するとフォルトが出て送信が止まり、Ctrl-C で終了の手順が通った。`read_only:=true` の launch は書き込み記録 0 バイト・命令は 0x02／0x82 のみ。
- xacro 展開: `use_fake_hardware:=true` は `mock_components/GenericSystem`（新引数は出ない）。feetech は新しい 4 つの `<param>` を出す。

## 計画 v4 から変えた点・補足
1. 手順書にあった確認 1（`probe_lock_sram`）は既定 off のパラメータとして実装。結果は INFO（ACCEPTED）／WARN（NOT accepted）。NOT accepted でも起動の手順は Lock=0 で SRAM を書くので影響しない。
2. Torque_Enable がすでに 1 のときは中止せず WARN（v4 の手順 1 の記述どおり）。EEPROM を書く必要があって Torque_Enable=1 のときだけ中止（トルク OFF が前提のため）。
3. 監視の読み取りが 3 回連続で失敗してもフォルト（v4 の「監視の読み取りが失敗してもフォルト」を 3 回連続と具体化）。
4. xacro の変更が 4 引数ぶん増えた（moveit_config の 2 ファイル。2b-1 の G・H と同じ種類の変更で、関節・リンク・limit は不変）。

## 未確認（実機）
- 書き込み全般（トルク ON の瞬間に動かないこと、Acceleration=50 の効き方と単位、Lock=1 のまま SRAM に書けるか＝確認 1）。手順は `docs/feetech_write_check.md`。
- Acceleration=0 の意味（このドライバは 0 のままトルクを ON にしない）。
- 通信断でサーボが最後の目標で保持し続けること（W5、任意）。
- JTC が実際に 25 Hz で指令を更新していること（`controllers.yaml` の値からの設計）。W4 で `rate_limited=0` なら整合。
- `SYNC_WRITE` には応答が無いので、Goal_Position が各サーボに届いたかは周期ごとには確認できない。Torque_Enable／Acceleration の 1 Hz 監視と読み取りの失敗でバス異常は検出する。
- 実機では Lock=1 のとき EEPROM への書き込みが無視される、という想定を偽サーボに入れている（`apply_servo_config` は Lock=0 の間に書き、読み戻して確認する）。実機の挙動は未確認。
- 偽サーボの限界: サーボ内部の加減速・保護・トルクの物理挙動は再現していない。ここまでの合格は、ドライバの手順・順序・検出ロジックのもの。

## 事後承認（2026-11-11）
- 対象: 2b-3b（ff91fd6）で変更した moveit_config の xacro 2 ファイル（2b-1 の G・H と同じファイル）。
  - `src/so_arm101_moveit_config/config/so101_new_calib.urdf.xacro`
  - `src/so_arm101_moveit_config/config/so101_new_calib.ros2_control.xacro`
- 変更内容: 引数 4 つ（`acceleration`、`torque_limit`、`apply_servo_config`、`probe_lock_sram`）を追加。`<param>` に渡すだけで、関節・リンク・limit は不変。mock への影響なし。
- 承認: ユーザーが 2026-11-11 に事後承認した。
- 今後のルール: MoveIt の設定（URDF・SRDF・関節リミット・planning_frame・IK solver・コントローラ設定・xacro）は、**実装前に**変更箇所と理由を示して承認を得てから変える。
- 2b-3 関連のコミット（確認済み、6 つ）: 438c4c9（計画 v3）、8151336（2b-3a）、e7b2194（2b-3a 実機記録）、b12b384（計画 v4）、ff91fd6（2b-3b）、43748b7（2b-3c 手順書）。

## 段階2b-3c 実機確認 W0 の途中経過（2026-11-11）。**W0 は合格していない**
ログ（`~/so_arm101_check/`）: `write_W0.log`（1 回目）、`write_W0_after.log`（停止後の読み取り専用）、`write_W0_2.log`、`write_W0_3.log`（起動失敗）。

### 事実
- **確認 1 の結果**: Lock=1 かつトルク ON のまま、SRAM の Acceleration に書き込めた（`[probe] ... ACCEPTED on all joints`）。トルクが入っても腕は動かなかった（1 回目）。
- **停止後の状態**（`write_W0_after.log`）: 6 関節とも SRAM が `torque_enable=0 lock=1 acceleration=50 goal_velocity=0`、`torque_limit`=1000（gripper は 500）。EEPROM vs yaml 6/6 match、設定 vs LeRobot 6/6 match。LeRobot では lock=0 のまま残っていたのに対し、こちらは lock=1 に戻る。
- 1 回目は停止時のログが 1 行も残っていない（tee が Ctrl-C で先に終了したため、と考えている）。`[deactivate]` の行と `deactivated. reads=…` の数値は未取得。`tee -i` なら残る（`write_W0_after.log` で確認）。
- **起動失敗 2 回**（`write_W0_2.log`、`write_W0_3.log`）: どちらも Goal_Position の読み戻しで `checksum mismatch`（2 回目は shoulder_pan id=1、3 回目は wrist_roll id=5）→ `activation failed: Goal_Position`。`[activation failed] torque OFF and Lock=1 confirmed` を確認。トルクは一度も入っていない。続いて `ros2_control_node` が exit code -6（controller_manager が `Failed to set the initial state of the component ... to active` の例外。Humble の仕様）。
- 1 回目は電源投入直後（Acceleration=0、Lock=1）、2・3 回目は Acceleration=50、Lock=1、トルク OFF が残った状態からの起動。

### コードを読んだ事実（原因の調査。`sts3215_system.cpp` 起動手順、`sts3215_bus.cpp`、`sts3215_write_bus.cpp`）
- 受信は 10 ms のタイムアウトで、`FF FF`→ID・LEN→本体を読み、チェックサムを検証する。失敗が `timeout` ではなく `checksum mismatch` なので、応答は届いており、ID（1、5）は期待どおり。壊れたのは応答の ERR・データ・チェックサムのどこか。
- 書き込みは ACK を読み切る。要求の直前に受信バッファを捨てる（`tcflush(TCIFLUSH)`）。書き込みの ACK から次の要求までの待ち時間は無い。
- 起動手順の各段階の並び:

| 段階 | 並び | 読み戻し（1 台あたり） |
|---|---|---|
| 2 Lock=0 | 関節ごとに「書く→読み戻す」の交互 | Lock |
| 4 Acceleration・Torque_Limit | 関節ごとに「書く→読み戻す→Torque_Limit を読む（違えば書く→読み戻す）」の交互 | Acceleration、Torque_Limit |
| 5 Goal_Position | 先に全関節の位置を SYNC_READ で 1 回読み、その後は関節ごとに「書く→読み戻す」の交互 | Goal_Position（2 バイト） |
| 6 Torque_Enable=1 | 関節ごとに交互 | Torque_Enable |
| 7 Lock=1 | 関節ごとに交互 | Lock |
| 8 最終確認 | 関節ごとに読み出しだけ（書かない） | Torque_Enable、Lock、Acceleration、Torque_Limit、Goal_Velocity |

- 起動手順の読み戻しは全部で約 60 回あり、Goal_Position はそのうち 6 回。**Goal_Position の段階だけが特有なのは**、(a) 2 バイトのレジスタへの書き込み直後の読み戻しであること（Torque_Limit も 2 バイトだが、値が同じなら書かない）、(b) 直前に SYNC_READ（全関節の位置）が走ること、(c) 書き込む値が `Present_Position` 相当の値であること。ほかの段階と同じ「書く→読み戻す」の交互パターンは段階 2・4・6・7 にもある。
- 起動手順に再試行は無かった。失敗時に受信したバイト列はログに出なかった。
- 偽サーボは化け・遅延・ノイズを注入できず、応答は常に正しく即時なので、この失敗は再現しなかった（再現できなくて当然で、ドライバの手順・順序の確認だけを担っていた）。

### 既存ログの確認（周期の読み取りの失敗。ファイルを読んだだけ）
- `write_W0_after.log`（読み取り専用、約 12 秒）: `deactivated (no register was written). reads=618 retried=0 failed_cycles=0`。再試行も失敗周期も 0。
- ほかの全ログ（`feetech_launch.log`、`feetech_launch_P2a.log`、`feetech_launch_P2b.log`、`servo_config_A.log`、`servo_config_B.log`、`write_W0.log`、`write_W0_2.log`、`write_W0_3.log`）: `read failed`、`consecutive=`、`COMMUNICATION LOST`、`FAULT` の行は 0 件（WARN・ERROR は URDF・MoveIt 由来の既知のものだけ。`write_W0_2/3` を除く）。
- ただし、これらのログには終了時の集計行（`deactivated. reads=… retried=… failed_cycles=…`）が無い（tee が先に終了したため）。数値として確認できるのは `write_W0_after.log` の 1 件だけ。`feetech_P1/P2a/P2b.csv` には NaN が 0 件、行数は各 251〜252（5 秒、約 50 Hz）。
- 解釈（推測）: 周期の読み取りは、確認できた範囲でほぼ失敗していない。通信品質が全般に悪いという仮説（候補 A）は弱まる。一方で、Goal_Position の段階で 2 回続けて失敗した事実から、その段階に特有の原因（書き込み直後の 2 バイトの読み戻し、直前の SYNC_READ との並び）が残る。

### 原因の候補（確定していない）
- A. 通信品質（USB-シリアル、ノイズ、電源）。周期の読み取りが安定していることから弱い。
- B. Goal_Position の書き込み直後にサーボが内部処理中で、直後の READ 応答が壊れる。段階特有の原因として有力。
- C. 直前の SYNC_READ の末尾が遅れて届き、後続の応答と混ざる（受信バッファを捨てるのは要求の直前のみ）。ただしヘッダ・ID が正しい点と合いにくい。
- D. 別プロセスがポートを触った。ユーザーが `ps` で否定済み。

### 対応（段階 2b-3d、コミット c56774c。実機未確認）
- 起動手順の読み戻しだけ、**通信エラー（チェックサム不良・フレーム不良・タイムアウト）のときに限り**最大 3 回（再試行 2 回、間に 5 ms）まで読み直す。値が期待と違うときは再試行せず即中止（Acceleration=0 を含む）。3 回失敗したらトルクは ON にならない。周期の read/write と 1 Hz 監視は変えていない。
- 通信エラーのとき、`last_error()` に `[tx=<送った要求> rx=<受信したバイト> flushed_before_request=<要求の前に捨てた古いバイト数>]` を追加。WARN に出る。
- 再試行の合計回数を `write mode active` の行と終了時の `deactivated. …` の行の末尾（`startup_read_retries=N`）、`activation failed` の行に出す。
- 試験: 偽サーボに化け・無応答の注入（`corrupt_read_replies`、`drop_read_replies`）を追加。化け 2 回→再試行で成功／無応答 1 回→成功／3 回化け→トルクを入れない（Lock=1、Torque_Enable=0）／値の不一致は再試行 0／Acceleration=0 は再試行 0／バイト列がエラーに出る。`colcon test` は 79 件（既存 73＋追加 6）合格。
- 手順書: `tee` を `tee -i` に、`ros2_control_node` の exit code -6 の説明を追加、W4 の止め方の順序を変更、報告用の grep に `retrying`・`startup_read_retries` を追加。

### W0 の次
再実行して、再試行が出るか・出たときのバイト列を見る。**W0 は未合格**のまま。

### USB-シリアルの情報（ユーザー報告、2026-11-11。W0 の結果を記録するコミットに含める）
- 事実（ユーザー報告）: USB-シリアルは 1a86:55d3（QinHeng CH343 系）で、cdc_acm の `/dev/ttyACM0`。RealSense とは別の USB バス。`sudo dmesg` に USB のエラーは出ていない（ただし接続時の行は流れていて見えない）。
- 解釈（推測）: USB 層のリセット・切断は起きていない。バス競合の可能性は低い。チェックサム不良の原因は、受信バイト列（`rx=…`）を見ないと切り分けられない。

## W0 合格（2026-11-11、`write_W0_4.log`、`tee -i`）
### 結果（事実。ログを読んで確認）
- `WRITE mode` → `[probe] ... ACCEPTED on all joints` → `write mode active ... startup_read_retries=0`。
- トルクが入っても腕は動かなかった（ユーザー報告）。Ctrl-C で脱力した。
- `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints`。
- `deactivated. reads=1083 retried=0 failed_cycles=0 goal_writes=1083 monitors=21 rate_limited=0 port_contention=0 startup_read_retries=0`（約 21 秒動かしたので monitors=21。1 Hz 監視として数は合う）。
- 停止後の状態は `write_W0_after.log` で確認済み: `torque_enable=0 lock=1 acceleration=50`、6/6 match。
- `FAULT`・`COMMUNICATION LOST`・`activation failed` は無し。ERROR は octomap（3D センサー未設定）と、終了時の move_group の exit code -11（既知の挙動。対応不要）だけ。
- `sudo dmesg` に ttyACM0 / cdc_acm / USB のエラーは無い（ユーザー報告）。
- **合格基準**: 腕が動かない／`rate_limited=0`／`port_contention=0`／`monitors` が時間どおり／停止後に `torque_enable=0 lock=1` ― すべて満たした。確認 1（Lock=1・トルク ON で SRAM に書けるか）は「書ける」。

### checksum mismatch の経過（原因は未特定のまま）
- 起動 4 回のうち、1 回目・4 回目が成功、2・3 回目が Goal_Position の読み戻しで失敗。
- 3 回目と 4 回目の間（約 1.5 時間）、サーボの外部電源の入れ直しは無し（ユーザー報告）。
- 4 回目は再試行 0 回で成功したため、受信バイト列は取れていない。
- 推測: 「電源投入後の最初の起動は成功し、直前に書き込みモードで起動していると失敗する」という形かどうかは、回数が足りず言えない。4 回目は電源の入れ直し無しで成功しており、その形とは合わない（ただし 1 件）。
- 今後: W1 以降も起動ごとに `startup_read_retries` と `retrying` の WARN の行を報告する。再試行が出たときのバイト列（`tx=… rx=… flushed_before_request=…`）で切り分ける。再試行 0 回が続くなら、頻度を記録する。

### 状態
- W0: **合格**。W1 に進む。
- 未確認のまま: Acceleration=50 の単位と効き方、通信断の保持（W5、任意）、checksum mismatch の原因。

## W1 合格（2026-11-11、`write_W1.log`、`torque_limit:=500`）
### 経過（事実。ユーザー報告とログの確認）
- 1 回目の起動で wrist_roll +0.2 を送った後、誤って Ctrl-C した（折りたたみ姿勢のままで腕は落ちていない）。同じファイル名で起動し直したため、1 回目のログは上書きされて残っていない。
- 2 回目の起動（`write_W1.log`）: `write mode active ... startup_read_retries=0`。起動時の `[diag]` は pan・roll・gripper が `torque_limit=500`（gripper は元から 500）、lift・elbow・wrist_flex は 1000。
- wrist_roll: -0.4（+0.1220 → 目標 -0.2780、実測 -0.2739）→ +0.2（実測 -0.0760。W1 開始前の -0.0775 に戻った）。result status 4（SUCCEEDED）。指令の向きに回り、異音・振動なし。
- gripper: +0.3（閉→開、実測 0.3044）→ -0.3（閉。指の間に約 1 mm 残って止まり、押し付け続けることはなかった）。
- `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints`。
- `deactivated. reads=22106 retried=0 failed_cycles=0 goal_writes=22106 monitors=437 rate_limited=0 port_contention=0 startup_read_retries=0`。
- `FAULT`・`COMMUNICATION LOST`・`step limit`・`retrying` の行は無し。
- **判定: 合格**（指令の向きに動く、位置が合う、異音・振動なし、`rate_limited=0`、`port_contention=0`）。

### 手順書との違い（gripper の `ramping` の WARN）
- 事実（コード）: `write_goal_positions` は、gripper の GripperActionController（`position_controllers/GripperActionController`）が出す目標をそのまま受け、1 周期の変化量が `max_step_raw_` を超えるときだけ削って WARN を出す。gripper の `max_step_raw_` は `floor(6 rad/s × 0.08 s ÷ tick)`。gripper の校正範囲は 1900〜3385（1485 tick）で 100 ≙ 100°（1.745 rad）、1 tick ≈ 0.001175 rad なので、上限は **408 tick ≈ 0.48 rad**。起動ログの `step limit: 312 ticks` は arm（angle 関節）の値。
- 事実（計算）: `send_joint_delta.py` の gripper ±0.3 rad は約 255 tick で、上限（408 tick）に届かない。よって WARN は出ず `rate_limited=0` になるのが正しい。上限に掛かるのは 0.48 rad より大きな一歩（ツールの上限 0.5 rad = 約 425 tick の場合）だけ。
- 訂正: 計画 v3 の「gripper の WARN は動作のたびに出る想定」と、手順書 W1 の「`ramping` の WARN が出るのは正常」は誤り（上限が gripper の小さな動きより大きいため）。手順書を修正した（W1 の記述、合格基準、「ログの読み方」）。コードの変更は不要（安全側の上限であり、動作は意図どおり）。

### checksum mismatch
- W0_4 と W1（2 回とも）の起動で `startup_read_retries=0`。失敗は 2・3 回目の 2 回のまま。原因は未特定（受信バイト列は未取得）。

### 状態
- W1: **合格**。W2 に進む。

## W2 合格（2026-11-11、`write_W2.log`、`torque_limit:=500`）
### 結果（事実。ユーザー報告とログの確認）
- `write mode active ... startup_read_retries=0`。
- shoulder_pan: +0.2（-0.0790 → 目標 +0.1210、実測 +0.1158）→ -0.4（目標 -0.2842、実測 -0.2785）→ +0.2（目標 -0.0785、実測 -0.0790）。3 回とも result status 4（SUCCEEDED）。実測は目標より 約 0.005 rad（約 3 tick）手前で止まり、往復で元（-0.0790）に戻った。
- `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints`。
- `deactivated. reads=16200 retried=0 failed_cycles=0 goal_writes=16200 monitors=320 rate_limited=0 port_contention=0 startup_read_retries=0`。
- `unexpected jump`・`step limit`・`FAULT`・`COMMUNICATION LOST`・`retrying` の行は無し。
- **判定: 合格**（W1 と同じ基準: 指令の向きに動く、位置が合う、`rate_limited=0`、`port_contention=0`、`unexpected jump` なし）。

### 観察
- 事実（ユーザー報告）: `+` の指令で、上から見て**時計回り**に回った。URDF の軸の向きと合っているかは未確認。**W4 で RViz の表示と見比べる**。
- 事実（ユーザー報告）: 低速（0.05〜0.067 rad/s）でやや階段状の動き（カクカク感）があった。
- 事実（計算）: angle 関節の 1 tick は 2π/4095 ≈ 0.001534 rad。0.05〜0.067 rad/s を 50 Hz の周期で割ると 1 周期あたり 0.001〜0.0013 rad で、**1 tick 未満**（約 0.65〜0.87 tick）。
- 推測（未確認）: 1 周期の目標の変化が 1 tick 未満で、整数の tick に丸めるため、目標が数周期ごとに 1 tick ずつ進む。サーボの不感帯・静止摩擦・機構の剛性と重なって階段状に見えた。Acceleration=50 の効き方も未確認。
- 今後: W4（速度スケーリング 0.1 の MoveIt 軌道）と比べる。改善が必要なら、計画を出して承認を得てから（コードは変えない）。

### checksum mismatch
- W2 の起動も `startup_read_retries=0`。失敗は 2・3 回目の 2 回のまま。原因は未特定。

### 状態
- W2: **合格**。W3 に進む（W3 は重力を受ける関節。前腕の下を支える）。

## W3 合格（2026-11-11、`write_W3.log`）

> **注（2026-11-11 追記。元の記録は書き換えていない）**: この節の関節の値（出発点・W3.5/W4 の elbow_flex など）は **旧校正の座標**（C3 の前）。C3 の後（新しい校正の座標）の出発点は pan −0.09820、lift −1.48295、elbow +1.69009、wrist_flex +0.50327、roll −0.04833、gripper +0.01523（「段階2c 実機確認: C3」）。
### 結果（事実。ユーザー報告とログの確認）
- `write mode active ... startup_read_retries=0`。
- `deactivated. reads=38348 retried=0 failed_cycles=0 goal_writes=38348 monitors=758 rate_limited=0 port_contention=0 startup_read_retries=0`、`[deactivate] torque OFF and Lock=1 confirmed`。`FAULT`・`step limit`・`COMMUNICATION LOST`・`retrying` の行は無し。
- 3 関節とも沈まず、停止後も動きなし。
- 測定の条件: 動かすあいだ、腕は手で支えなかった（手で支えると遊びの分で腕が動き、測定に影響するため）。値は自重だけを受けた状態のもの。
- **判定: 合格**（沈まない、指令の向きに動く、`FAULT` なし、`rate_limited=0`）。

### 関節ごと（開始時: lift -1.5374、elbow +1.6771、wrist_flex +0.9329）
| 関節 | 指令 | 目標 | 実測 | 備考 |
|---|---|---|---|---|
| wrist_flex | -0.1（開く向き。手先が前腕から離れる） | +0.8329 | +0.8454（約 8 tick 手前） | 止まっている間は沈まない |
| wrist_flex | +0.1（戻す） | +0.9454 | +0.9406 | |
| elbow_flex | -0.1（開く向き。前腕が上腕から離れる） | +1.5771 | +1.6187（約 27 tick 手前） | 2 秒おき 3 回とも同じ値 |
| elbow_flex | +0.1 のつもりで戻す | +1.7187 になるため送らず | | 校正範囲の端（計算で約 +1.71）を越えるため、+0.055 だけ送った: 目標 +1.6737、実測 +1.6694 |
| shoulder_lift | +0.1（腕全体が前へ倒れる。先端が下がる向き） | -1.4374 | -1.4454 | 2 秒おき 3 回とも同じ値。**gripper の先端が台に接触**。強い押し付け・異音なし。台の影響か自重かは区別できない |
| shoulder_lift | -0.09（戻す） | | -1.5328 | |
- 推測（未確認）: 開く向き（自重に逆らう向き）では目標の手前で釣り合って止まり、戻す向きでは差が約 3 tick。

### 手順書の修正（ドキュメントのみ）
- W3 の戻し方を「開始時の値 − 今の実測値」を計算して送る形に変更（`send_joint_delta.py` は今の実測値から目標を計算するため、行きで手前に止まった分だけ固定量で戻すと開始位置を越える。端に近い関節は少し手前まで）。
- shoulder_lift の + は先端が下がる向きで、0.1 rad で台に接触したことを明記（W3 では lift は 0.05 から）。
- 支え方を実際に合わせた（動かすあいだは前腕の下に触れずに構える。止めるときは触れて受ける）。

## shoulder_pan の軸の向き（W2 の「+ で時計回り」と URDF の照合。変更なし）
- 事実（URDF `so101_new_calib.urdf`）: `shoulder_pan` の `<origin rpy="3.14159 4.18e-17 -3.14159">`（親 base_link、子 shoulder_link）、`<axis xyz="0 0 1"/>`（子のフレーム）。rpy から計算した回転は diag(-1, 1, -1)（x と z が反転）なので、**軸の向きは base_link の −z**。
- 事実（計算。`numpy` による URDF の FK。実機・ポートは使っていない）: pan=0 で gripper_frame の位置は base_link で (0.391, 0, 0.227)、pan=+0.3 で (0.376, **-0.104**, 0.227)。つまり **+ の回転は、+x（前）から −y へ動く = base_link の +z（上）から見て時計回り**。
- 照合: W2 の観察（+ の指令で上から見て時計回り）と**一致**する（base_link の +z が上向きであるという前提。URDF の base_link は通常の向き）。W4 で RViz の表示と見比べて最終確認する。
- 同じ FK の符号を、W3 の観察とも照合した: lift +0.1 で先端の z が下がる（-0.0082 → -0.0167 m）、elbow −0.1 と wrist_flex −0.1 で先端の z が上がる（-0.0082 → +0.0128、-0.0007 m）。いずれも W3 の観察と一致。
- URDF の関節リミット: elbow_flex ±1.69、shoulder_lift ±1.74533、wrist_flex ±1.65806、shoulder_pan ±1.91986。W3 の開始時の elbow +1.6771 は elbow の URDF 上限（1.69）の 0.013 rad 手前（問題2の記録と同じ、リミット近傍）。校正範囲（約 +1.707）は URDF 上限より広い。

## 問題2への注記（2026-11-11。上の元の記録は変更していない）

> **注（2026-11-11 追記。元の記録は書き換えていない）**: この節の関節の値（出発点・W3.5/W4 の elbow_flex など）は **旧校正の座標**（C3 の前）。C3 の後（新しい校正の座標）の出発点は pan −0.09820、lift −1.48295、elbow +1.69009、wrist_flex +0.50327、roll −0.04833、gripper +0.01523（「段階2c 実機確認: C3」）。
- 事実（モック。`mock:=true use_rviz:=false` の `/check_state_validity`、group=arm、実機は使っていない）:
  - 問題2の値（shoulder_lift −1.749、elbow_flex 1.700、wrist_flex 1.313、pan 0、roll 0）: **無効**、contacts は `upper_arm_link` ↔ `wrist_link`（元の記録と一致。再現した）。
  - 同じ姿勢を URDF のリミット内に丸めた値（−1.745 / 1.69 / 1.313）: **有効**。SRDF の home（−1.57 / 1.57 / 0.8）: 有効。
- 推測（未確認）: 問題2の「衝突」は、関節リミットを超えた値（lift −1.749、elbow 1.700）が原因だった可能性がある。元の記録は「リミット超過は中止の原因ではない」としているが、上の結果はそれと食い違う。リミット内の値で再現するかは、問題2の実機での姿勢を再確認しないと断定できない。
- 変更なし: URDF・SRDF のリミットや home の値は変えていない（変更が必要と考えたら計画を出して承認を取る）。

## 段階2b-3c: W3.5 の最終案（2026-11-11 承認。手順書 `docs/feetech_write_check.md` に反映）

> **注（2026-11-11 追記。元の記録は書き換えていない）**: この節の関節の値（出発点・W3.5/W4 の elbow_flex など）は **旧校正の座標**（C3 の前）。C3 の後（新しい校正の座標）の出発点は pan −0.09820、lift −1.48295、elbow +1.69009、wrist_flex +0.50327、roll −0.04833、gripper +0.01523（「段階2c 実機確認: C3」）。
- W3.5/W4 の出発点と W4 後の戻し先: 調整後の値（pan -0.08362、lift -1.51134、elbow +1.66938、wrist_flex +0.50941、roll -0.07135、gripper +0.01645。`~/so_arm101_check/start_pose.log`）。lift は +0.022 rad、wrist_flex は約 -0.43 rad 調整した（トルク OFF で手で動かした）。
- モック判定（すべて有効）: 出発点、elbow -0.10 / -0.15 / -0.20 / -0.30。先端 z（FK、base_link 基準）の出発点からの差: +0.026 / +0.039 / +0.053 / +0.081 m（出発点の z は +0.0304 m）。
- W3.5: elbow_flex に -0.10 を 2 回（--seconds 4、各回の後に実測確認）。出発点から -0.15〜-0.20 で終了、届かなければ最大 -0.05 を 1 回だけ追加、-0.25 を超える指令は送らない。lift は動かさない。
- 起動: W3.5 と W4 は同じ起動（RViz あり、`torque_limit:=500`、`write_W4.log`）。
- W4 の戻し: RViz は使わず `send_joint_delta.py` で 1 関節ずつ「出発点 − 実測」を送る（elbow_flex は最後）。Ctrl-C の前に各関節が出発点から約 0.02 rad 以内であることを確認する。W4 の目標は elbow_flex を + に動かさない（リミット +1.69 まで 0.021 rad）。
- 後始末の事実: バックグラウンド起動した launch は SIGINT が効かず、SIGTERM と PID 指定で停止した（最後の ps で残りなしを確認）。

## 段階2b-3c: W3.5・W4 合格、W0〜W4 すべて合格（2026-11-11。ユーザー報告。ログ: `~/so_arm101_check/write_W4.log`、`use_rviz:=true torque_limit:=500`）

> **注（2026-11-11 追記。元の記録は書き換えていない）**: この節の関節の値（出発点・W3.5/W4 の elbow_flex など）は **旧校正の座標**（C3 の前）。C3 の後（新しい校正の座標）の出発点は pan −0.09820、lift −1.48295、elbow +1.69009、wrist_flex +0.50327、roll −0.04833、gripper +0.01523（「段階2c 実機確認: C3」）。

### W3.5（elbow_flex を開く）
- 起動: `write mode active ... startup_read_retries=0`。トルク ON で腕は動かず、値は出発点と同じ。
- elbow_flex（各回とも 2 秒おき 3 回で同じ値。沈まない）:

  | 送った量 | 目標 | 実測 |
  |---|---|---|
  | -0.10（出発点 +1.6694 から） | +1.5694 | +1.6203 |
  | -0.10 | +1.5203 | +1.5620 |
  | -0.05 | +1.5120 | +1.5497 |
- **基準からの逸脱**: 出発点からの変化は **-0.120** で、手順書の基準（-0.15〜-0.20）に届かなかった。毎回、目標の約 0.04 rad 手前で止まるため。-0.05 を足して実際に動いたのは -0.012 だった。URDF リミットまで約 0.14 rad あり、モックで -0.10〜-0.15 は有効だったので、ユーザーの判断でここで W3.5 を終えた。
- RViz のモデルは実機の姿勢とおおむね一致。リンク同士がやや重なって（めり込んで）見えた（見た目のみ。Plan は衝突判定で止められていない）。

### W4（RViz の Plan & Execute、スケーリング 0.1）
- Joints タブで shoulder_pan を +1° → Plan のアニメーションも実機も、上から見て時計回り（W2 の実機・URDF の FK と一致。pan の向きの最終確認は済み）。Execute は約 1 秒で SUCCEEDED。
- その後、計 4 回の Execute（pan を -1° や -5° 程度など。各回の詳細な操作は記録できていない）。すべて SUCCEEDED。`Completed trajectory execution with status SUCCEEDED` は 5 回。`step limit:`・`FAULT`・`COMMUNICATION LOST` なし。
- 事実: Joints タブのスライダーは整数の度で、目標の姿勢は全関節が整数の度になる（実行後の実機は pan +2.33°、lift -86.07°、elbow +90.29°、wrist_flex +29.54°。スライダーは 2、-86、90、30）。そのため動かしていない関節も目標が最大 0.5° 程度変わる。elbow_flex は約 88.5° → 90°（閉じる向き）となり、1 回の Execute で「カタン」と落ちる動きがあった（約 1.8°）。MoveIt と arm_controller は渡された目標どおりに動いている。**原理的な挙動なので、コード・設定での対策はしない**（手順書に注意として記載）。
- 1 回目の Execute の後、動かしていない関節が 0.01 rad 未満動いた（微小）。
- 戻し（`send_joint_delta.py`）: pan -0.12、elbow +0.09。出発点との差は pan 0.011、lift 0.009、elbow 0.002、wrist_flex 0.006 rad（すべて 0.02 以内）。elbow の目標は +1.6719 で出発点（+1.6694）をわずかに越えたが、URDF リミット +1.69 未満（実測 +1.6678）。
- 停止: `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints`。停止時に腕の動きなし。
- 末尾: `deactivated. reads=131540 retried=0 failed_cycles=0 goal_writes=131540 monitors=2601 rate_limited=0 port_contention=0 startup_read_retries=0`。

### 推測（未確認）: 開く向きと閉じる向きの差
- 開く向き（自重に逆らう向き）では目標の約 0.04 rad（約 27 tick）手前で止まり、閉じる向きでは約 3 tick。トルク OFF でも手を離して止まっていられるほどギアの摩擦が大きいので、開く向きでは自重とギアの摩擦に逆らう分の力が足りずに止まり、閉じる向きでは自重で動く、とユーザーは考えている。W4 の「カタン」も、閉じる向きの目標で摩擦から外れて自重で動いたためと考えている。
- `open_loop_control: true` のため、こうした手前で止まるずれに軌道の側は気づかない（status は SUCCEEDED になる）。

### checksum mismatch（未解決）
- W0 の 2 回のあと、今日の起動 6 回（W0_4、W1×2、W2、W3、W4）はすべて `startup_read_retries=0`。原因は未特定のまま。再試行と受信バイト列の記録は残す。

### 状態
- W0・W1・W2・W3・W3.5・W4 合格。W5（通信断）は任意で未実施（行うかはユーザーが決める）。
- 段階2b の完了に向けて残り: `open_loop_control` が True のままであることの確認、書き込みモードで LeRobot を使っていないことの確認（`feetech_readonly_check.md` 2.5 節と同じ確認）。その後にタグ `stage2b-done` と、`bringup.launch.py` の `hardware` / `read_only` の既定値の計画（いずれも承認後）。

## 段階2b-3c: 確認 2（書き込みモードでの LeRobot 不使用）合格（2026-11-11、ユーザー報告。ログ: `~/so_arm101_check/write_final.log`）
- 条件: `hardware:=feetech read_only:=false use_rviz:=false`、指令は送っていない。腕は W3.5・W4 の出発点の姿勢（トルク OFF で手を離しても止まっている姿勢）。手順は `feetech_readonly_check.md` の 2.5 節 (1)〜(7) と同じ。
- 起動: `write mode active ... startup_read_retries=0`。
- (1) `CONDA_PREFIX=[] LECONDA_SITE_PACKAGES=[]`、`python3` は `/usr/bin/python3`。(2)(3)(4)(6) は出力なし。(5) `pid=179412`、`lerobot|miniforge3` の数は 0。(7) `fd 10 -> /dev/ttyACM0`（`ros2_control_node` 自身）。
- `ros2 param get /arm_controller open_loop_control` → `Boolean value is: True`。設定の出どころ: `src/so_arm101_controller/config/controllers.yaml:43`（`bringup.launch.py` が `so_arm101_controller` の share から読む）。
- 停止: `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints`。`deactivated. reads=13087 retried=0 failed_cycles=0 goal_writes=13087 monitors=258 rate_limited=0 port_contention=0 startup_read_retries=0`。
- **判定: 合格**。

### 残課題: `ros2 node list` の名前の重複警告（修正は未実施）
- 現象（ユーザー報告）: (3) の実行時に「nodes in the graph that share an exact name」の WARNING。`ros2 node list | sort | uniq -d` の結果は `/static_transform_publisher0` のみ。`ps` では `ros2_control_node`・`move_group`・`robot_state_publisher` は今回の起動（14:36:14）のものだけだった。
- 事実（コード）: 名前を付けて `static_transform_publisher` を起動するのは `src/so_arm101_moveit_config/launch/static_virtual_joint_tfs.launch.py` が呼ぶ `moveit_configs_utils.launches.generate_static_virtual_joint_tfs_launch` だけ。SRDF の `virtual_joint` は 1 つ（world → base_link、`so101_new_calib.srdf:37`）なので、**起動されるのは 1 つで、名前は `static_transform_publisher0`**（名前なしのノードが 2 つあるのではない）。`bringup.launch.py` はこの launch を 1 回だけ include している。他に `static_transform_publisher` を起動する箇所は `src/` に無い（`archive/stage2a/description/launch/rsp.launch.py` にあるが、アーカイブで bringup は使わない）。ユーザーの「名前なしが 2 つ」という推測は、コードの上では成り立たない。
- 原因の候補（事実）: Claude が同日 13:31 に起動したモックの `static_transform_publisher0`（PID 175277、`world`→`base_link`）が**止め損ねて残っていた**。Claude の後始末の `ps` 確認パターン（`ros2_control_node|move_group|robot_state_pub|rviz2|spawner`）に `static_transform_publisher` が含まれておらず、見逃した。ユーザーが `uniq -d` で見つけた重複はこれと一致する（残りの確認は 14 時台ではなく、このコミットの作業中に `ps` で見つけた）。Claude が PID を指定して停止し、`ps` で残りなしを確認した。
- 対応: 今後、モック・実機の後始末の確認パターンに `static_transform_publisher` を加える。コード・設定の修正は不要（再発するなら launch ごとの起動前の残りの確認を手順に入れる）。
- 注意: 実機のログ（`write_final.log`）に影響したのは名前の重複警告のみ。ハードウェアドライバ・トルク・シリアルポートには関係しない（static_transform_publisher は TF を出すだけ）。

## 段階2b 完了条件のチェックリスト（2026-11-11 更新）
※ 指示書そのものはリポジトリに無いため、これまでの作業で確認してきた項目で記録する。

| 項目 | 状態 | 根拠 |
|---|---|---|
| 読み取り専用ドライバ `Sts3215System`（2b-2） | ✅ | 2b-2 実機確認 合格（2026-11-10） |
| 書き込みモードの実装（2b-3b）とビルド・試験 | ✅ | `colcon test` 全試験通過（偽サーボ含む） |
| 実機 W0〜W4 | ✅ | W0・W1・W2・W3・W3.5・W4 合格 |
| `arm_controller` の `open_loop_control` が True | ✅ | `controllers.yaml:43`、起動後 `ros2 param get` → True |
| `hardware:=feetech read_only:=false` が LeRobot を使わない | ✅ | 確認 2 合格（`write_final.log`） |
| W5（通信断） | 未実施（任意） | ユーザーの判断で今日は行わない |
| チェックサム不一致の原因 | 未特定 | 再試行と受信バイト列の記録は残す（今日の起動 7 回は `startup_read_retries=0`） |
| タグ `stage2b-done` | このコミットの直後に付与（承認済み） | |
| `bringup.launch.py` の `hardware` / `read_only` の既定値 | 未（計画を出し、承認後） | 現在 `hardware=lerobot`（既定）、`read_only=true`（既定） |

## 段階2b-4: bringup.launch.py の既定値の変更（2026-11-11）
- 変更（`src/so_arm101_bringup/launch/bringup.launch.py` の `DeclareLaunchArgument` の既定値・説明文と冒頭コメントだけ）: `hardware` の既定を `lerobot` → `feetech`。`read_only` の既定は `true` のまま（説明に「書き込みは `read_only:=false` を明示したときだけ」を追記）。`mock` の説明文から「LeRobot bridge 経由」を除いた。条件式・ノード構成は変更なし。
- 確認（事実）:
  - `colcon build` 成功（7 パッケージ）。`--show-args` で `hardware` の default が `'feetech'`、`read_only` が `'true'`。
  - モック（`use_rviz:=false`）: `gripper_controller`・`joint_state_broadcaster`・`arm_controller` がすべて active。
  - bridge の条件式 `mock=='false' and hardware=='lerobot'` を 4 通りで評価: bridge が起動するのは mock=false かつ hardware=lerobot のときだけ（mock=true/any、mock=false/feetech は起動しない）。
  - 停止後: `ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher" | grep -v -E "grep|gvfsd"` は出力なし、`ros2 node list` も出力なし。
- 手順書: `feetech_*_check.md` の起動コマンドはすべて `hardware:=` を明示しており、意味が変わる箇所なし。`feetech_write_check.md` に既定の説明を 1 段落追記。
- 実機は動かしていない。
- 補足（2b-4、2026-11-11）: mock:=true のとき hardware の値は使われないことの確認。根拠は `src/so_arm101_moveit_config/config/so101_new_calib.ros2_control.xacro` の分岐（`use_fake_hardware` が真なら `mock_components/GenericSystem` のみ。feetech・lerobot の分岐は `unless use_fake_hardware` の内側）。`xacro so101_new_calib.urdf.xacro use_fake_hardware:=… hardware:=…` の 4 通りの出力: true/feetech → GenericSystem、true/lerobot → GenericSystem（serial_port・calibration_file・read_only も出ない）、false/feetech → Sts3215System、false/lerobot → So101HardwareBridge。bridge ノードの条件式（`mock=='false' and hardware=='lerobot'`）も mock=true では常に偽。
- 注意（変更なし）: xacro マクロ自身の `hardware` の既定は `lerobot` のまま（moveit_config の xacro なので触っていない）。`bringup.launch.py` は常に `hardware` を渡すので影響しない。xacro を直接呼ぶ場合に限り、実機（`use_fake_hardware:=false`）では `hardware:=` の明示が要る。段階2c の archive で扱う（承認制）。

## 段階2b-4: 実機確認 合格（2026-11-11、ユーザー報告。ログ: `~/so_arm101_check/stage2b4_default_launch.log`）
- 起動: `ros2 launch so_arm101_bringup bringup.launch.py mock:=false use_rviz:=false`（`hardware:=` も `read_only:=` も付けない）。
- プラグインは `so_arm101_hardware/Sts3215System`。`arm_controller`・`gripper_controller`・`joint_state_broadcaster` がすべて active。
- `[diag] EEPROM vs yaml` 6/6、`servo settings` 6/6 MATCH。`READ-ONLY mode` の WARN あり、`write mode active` なし。全関節 `torque_enable=0`。
- ユーザーが gripper を手で動かし、トルク OFF を確認。そのとき `READ-ONLY: command for 'gripper' differs ... -0.025 rad and is discarded` が 1 回出た（手で動かしたため。異常ではない）。
- bridge のノードなし。`lerobot` / `miniforge3` / `follower_ros2_node` のプロセスなし。
- 停止後: `ps` は空。`ros2 node list` には `/move_group` と `/moveit_simple_controller_manager` が残ったが、`ros2 daemon stop; ros2 daemon start` の後は空。推測（未確認）: move_group が Segmentation fault で終わり、daemon に古い一覧が残った。
- 判定: **合格**（既定の `hardware:=feetech read_only:=true` で LeRobot を使わず読み取り専用で起動する）。
- 同時に修正: `feetech_readonly_check.md` の launch の `tee` に `-i` を付けた（2 箇所）。後始末の確認（`ros2 node list` に残り `ps` が空なら daemon を再起動して再確認）を `feetech_write_check.md`（共通の約束 8）、`feetech_readonly_check.md`（5.5 節）、`CLAUDE.md` に追記。

## 段階2c: 計画（2026-11-11 承認。全文は `docs/stage2c_plan.md`）
- 承認: 2c-1a（書き込みモードのときだけ EEPROM/yaml の不一致で起動拒否）、2c-1b（`so_arm101_hardware` の export。公開ヘッダが他パッケージのヘッダを include するなら `ament_export_dependencies` も同じコミット）、失敗時の動き・再試行・D1/D2/D3・C1 後の判断・0 rad の一覧。
- ユーザー指摘で直した 3 点: (1) `calibrate` を「測定（EEPROM に書かない。測定ファイルに保存）」と「適用（測定ファイルから `--write`）」に分けた。C2 は gripper だけ適用して yaml は更新せず、バックアップへ戻す。C3 は**同じ測定ファイル**で 6 関節（gripper を含む）を 1 回で適用し、yaml はそのとき 1 回だけ更新する。適用の前に今の EEPROM がバックアップと一致していることを確認する。(2) Lock は電源投入時に 1（実機で確認済み、上の「確定した事実」の表）。Lock=1 を確認できなかったときの対処は「サーボの電源を入れ直す」。(3) C3 で新座標の出発点の姿勢を測るとき、URDF の関節リミットまでの余裕（rad）も出し、範囲の外や余裕が小さい関節は C4 の前に報告する。
- C0.5 で `--eeprom-wait-ms 0` などで読み戻しが一致しないと、失敗扱いで戻す処理が走り、終了コード 1 になる。想定どおり（手順書の合格基準に明記）。
- 実機のコマンドとシリアルポートを開くコマンドは Claude は実行しない。

## 段階2c-1a: 書き込みモードで EEPROM と yaml が不一致なら起動を拒否（2026-11-11）
- 変更: `Sts3215System::run_diagnostics()`（`src/sts3215_system.cpp`）。`read_only:=false` のとき、Homing_Offset・Min/Max limit のどれかが yaml と違うと `RCLCPP_ERROR`（`refusing to start in write mode`、restore か yaml の更新を促す）を出して `fatal` にし、`on_configure` が ERROR を返す（トルクは ON にならず、何も書かない）。`apply_servo_config:=true` でも拒否は外れない。解除のスイッチなし。`read_only:=true` は従来どおり WARN で継続。
- 試験（偽サーボ）: 追加 5 件（homing 不一致、min/max 不一致、apply_servo_config でも拒否、一致なら起動、読み取り専用は継続して書き込み 0 件）。試験の `Rig` は、偽サーボの EEPROM（Homing_Offset・Min/Max）を yaml の値に初期化するようにした（実機の状態と同じ。以前は 0 のままで、不一致を検査していなかった）。
- 結果: `colcon build` 成功。`so_arm101_hardware` の `colcon test` は 84 件すべて通過。
- 事実: `colcon test`（全体）で `apriltag_test`・`so_arm101_bringup`・`so_arm101_lerobot_bridge` に失敗（lint の copyright/flake8/pep257、pytest の収集エラー）。この変更を stash した状態でも同じ（106 件中 2 errors・14 failures）で、今回の変更とは無関係。未対応（段階の対象外）。

## 段階2c-1b: `so_arm101_hardware` のライブラリの export（2026-11-11）
- 変更: `src/so_arm101_hardware/CMakeLists.txt` の `ament_package()` の直前に追加（ほかは変更なし。ドライバの動作は変わらない）:
  ```cmake
  ament_export_include_directories(include)
  ament_export_libraries(so_arm101_sts3215)
  ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface std_msgs sensor_msgs yaml-cpp)
  ```
- 依存の export が必要だった理由（事実）: 公開ヘッダのうち `sts3215_system.hpp` が `hardware_interface`・`rclcpp`・`rclcpp_lifecycle`、`so101_hardware_bridge.hpp` が `hardware_interface`・`rclcpp`・`rclcpp_lifecycle`・`std_msgs`・`sensor_msgs` を include する（bus 系のヘッダ `sts3215_bus.hpp` など 5 つは標準ライブラリと互いだけ）。`yaml-cpp` は `so_arm101_sts3215` がリンクしている。
- 確認: 使い捨ての利用側（scratchpad の外部 CMake プロジェクト。`find_package(so_arm101_hardware)` + `ament_target_dependencies`）が、`sts3215_write_bus.hpp`・`calibration.hpp`・`sts3215_system.hpp` を include し、`Sts3215WriteBus::build_write` を呼ぶ実行ファイルをビルド・実行できた。`colcon build` 成功、`so_arm101_hardware` の `colcon test` は 84 件通過。

## 段階2c-1c: `so_arm101_calibration`（バックアップ・復元）と偽サーボの拡張（2026-11-11）
- 新パッケージ `src/so_arm101_calibration`（C++。`so_arm101_hardware` のライブラリを再利用）。実行ファイル `so_arm101_calib`（`ros2 run so_arm101_calibration so_arm101_calib …`）:
  - `backup`: 読み取り専用。全関節の EEPROM（アドレス 0〜39 を 8 バイトずつ）を 2 回読んで一致を確認し、`~/so_arm101_check/eeprom_backup_<日時>.json` に保存し、読み直して一致を確認する。JSON はバイト列のチェックサム付き。保存できなければ先へ進まない（終了コード 3）。
  - `restore`: バックアップから Homing_Offset・Min/Max_Position_Limit だけを戻す。既定は dry-run。`--write` と確認文字列 `WRITE-EEPROM` の手入力の両方が必要。`--joints`、`--rewrite-same`（同じ値でも書く。C0.5 用）、`--eeprom-wait-ms`（既定 50、仮置き。0〜1000）。書く前に全関節の機種番号 777 と Torque_Enable=0 を確認し、バックアップファイルは**ポートを開く前**に検査する。書く順序は Lock=0 → Homing → Min → Max → Lock=1（各項目を待って読み戻し）。再試行は通信エラーのときだけ（3 回）、値の不一致はすぐ失敗。失敗したら残りの関節へ進まず Lock=1 に戻し、全レジスタの状態（実行前/バックアップ/不明）を表示し、`restore` のコマンド例を出す。終了コード 0〜4（`docs/stage2c_plan.md` 2 章、手順書に表）。トルクには書かない。
- 書き込みの allow-list: `CalibWriteBus`（`so_arm101_calibration`）が Homing_Offset と Min/Max だけを書く（生の 16 bit。Homing_Offset は 12 bit まで）。ドライバ（`libso_arm101_sts3215`）にこの書き込みは無い。そのために `Sts3215WriteBus::write_bytes` を private → protected にした（`sts3215_write_bus.hpp` の 1 箇所。ドライバの動作は変わらない）。
- 偽サーボ（`test/fake_servo.hpp`）に追加: ID レジスタ、`ignore_writes_to(id, addr)`、`eeprom_commit_delay_ms`、`drop_write_replies`、`corrupt_write_replies`、`cut_power_after_writes` / `restore_power`、`ignore_lock_one`。`tools/fake_sts3215.py` は ID レジスタ（アドレス 5）を追加（以前は 0 のままで、バックアップの ID 検査で見つかった）。
- 試験（偽サーボ。`colcon test`）: 21 件。バックアップ（40 バイト、書き込み 0、往復、負の Homing_Offset の生バイト保持、破損・欠け・重複・チェックサム・範囲外の拒否、読み取りの再試行）、復元（dry-run、確認なし、トルク ON/機種違い/入力不正で書かない、全関節の順序と Lock、関節の絞り込み、負の offset、同じ値、対象外バイトの注意、通信エラーの再試行と値の不一致は再試行しない、途中の電源断 → 電源入れ直し → 再復元、Lock=1 の失敗、Lock=0 が反映されないとき値を書かない、待ち時間と EEPROM の反映遅れ）。
- 結果: `colcon build`（8 パッケージ）成功。`so_arm101_hardware` 84 件、`so_arm101_calibration` 22 件（gtest 21＋lint）、すべて通過。`apriltag_test`・`so_arm101_bringup`・`so_arm101_lerobot_bridge` の既存の失敗（lint・収集エラー）は変わらず。
- 偽サーボでの CLI の確認（`tools/fake_sts3215.py` の pty。実機・`/dev/ttyACM0` は開いていない）: `backup`（書き込み 0）→ `restore` の dry-run（変更なし）、確認文字列の不一致（書き込み 0、終了コード 4）、`--rewrite-same --joints gripper --write`（書き込みは Lock=0、Homing、Min、Max、Lock=1 の順、終了コード 0）、elbow の Homing_Offset だけ違う状態を `--joints elbow_flex` で戻す（その 1 項目だけ書く）。注意: pty は読み書きを閉じても排他（TIOCEXCL）が残るので、CLI の確認は 1 回ごとに偽サーボを起動し直した（実機のシリアルポートは最後の close で解除される。偽サーボ側の都合）。
- 手順書: `docs/so_arm101_calibration_check.md`（C0 と C0.5。C1 以降は 2c-2 で追記）。出典: `docs/third_party.md` に追記。
- 次: 2c-2（`measure` / `apply`、失敗時の復元、測定ファイル、偽サーボでの全試験、手順書の続き）。

## 段階2c-2: `measure` / `apply` / `compare`、失敗時の復元、測定ファイル、手順書（2026-11-11）
- `so_arm101_calib` に追加（`so_arm101_calibration`）:
  - `measure`: 測定。**EEPROM にも Lock にも書かない**。書くのはトルクが入っている関節の `Torque_Enable=0` だけ（読み戻し確認。腕を支える確認の Enter が先にある）。`actual = (present + 現在の Homing_Offset) mod 4096`（`reset_calibration` の中間書き込みはしない）、新しい Homing_Offset = `actual_center − 2047`、MIN/MAX は中央からの差（±2048 に展開）で記録し、新座標 = 2047 + 差（wrist_roll のように 0/4095 をまたぐ関節も連続して扱える）。中央は 11 回読んで中央値、ばらつきが 4 tick を超えたら中止。検査（|Homing_Offset| ≤ 2047、0 ≤ min < max ≤ 4095、幅 ≥ 200 tick）に落ちた関節があれば終了コード 4（結果は測定ファイルに保存。`apply` は受け付けない）。測定ファイル（JSON、チェックサム付き）: actual_center、測定時の Homing_Offset、新しい Homing_Offset、actual/新座標の min/max、new_center、samples、valid/problems、日時、ツール、ポート、バックアップのファイル名。
  - `compare`: 2 つの測定ファイルの再現性（actual_center・min・max・center・Homing_Offset の差。tick）と、今の yaml との差（tick と rad、`zero_shift` = 0 rad の物理位置 `center + Homing_Offset` の移動量）。シリアルポートを開かない。
  - `apply`: 測定ファイルから `--write` で書く（既定は dry-run、確認文字列 `WRITE-EEPROM`）。書く前に: 全関節の機種番号 777・Torque_Enable=0、**今の EEPROM（全関節）がバックアップと一致**、測定ファイルが valid・測定時の Homing_Offset が今の EEPROM と一致、バックアップが全関節で復元に使える。違えば書かない（終了コード 3/4）。書き込みは関節ごとに Lock=0 → Homing → Min → Max → Lock=1。**失敗したら、この回で触れた全関節（失敗した関節を含む）をバックアップの値へ戻そうとし**（戻す途中の失敗は記録して続ける）、全レジスタの 旧/新/不明 と Lock を表示して、`restore` のコマンド例を出す。終了コード: 1 = 全関節が旧に戻り Lock=1 確認、2 = 一部が新/不明のまま・または Lock=1 を確認できない（電源を入れ直す）、3/4 = 何も書いていない。**yaml を更新するのは、6 関節すべてを 1 回で書き、読み戻しと Lock=1 を確認した後の 1 回だけ**（一時ファイル → 読み直し → rename。シンボリックリンクは実体を更新）。`--joints` で一部だけ書いたとき（C2）は更新しない。yaml を更新できなかったら終了コード 2（EEPROM は新、yaml は古い。書き込みモードは起動を拒否する）。
  - 共通部品: `write_joint_sequence`（Lock=0 → 値 → Lock=1。Lock=0 を試みたら Lock=1 を必ず試みる）。`kConfirmText` を `eeprom_ops.hpp` へ。
- 偽サーボの追加（2c-1c に続き）: `MeasureIO` の `on_center_sample`（試験用）。`tools/fake_sts3215.py`: 定期的に TIOCNXCL を解除（pty は閉じても排他が残るので、続けて何度でもクライアントを起動できるようにする。結合確認用）。
- `tools/pose_margin.py`（新規。オフライン、URDF・SRDF は読むだけ）: 姿勢と校正範囲の両端が、URDF の関節リミットまでどれだけ余裕があるかを表示（OUT < 0、LOW < 0.05 rad は報告のための仮の目安）。今の yaml での出力は既知の事実と一致（lift の校正の端 −1.8213 に対し URDF −1.7453、elbow +1.7077 に対し +1.69、出発点の elbow の上限までの残り +0.0206）。ほかに、校正の範囲が URDF より広い関節は pan（±2.0783 / ±1.9199）、wrist_flex（±1.7614 / ±1.6581）、wrist_roll（±2.9160 / −2.7439..+2.8412）。
- 試験（偽サーボ、`colcon test`）: `so_arm101_calibration` 47 件（gtest 46＋lint）。2c-1c の 21 件に加えて、measure（中央・範囲・wrap・書き込みなし、トルク ON のときの `Torque_Enable=0` だけの書き込み、OFF にできないとき中止、中央で動いたら拒否、動かさなかった関節の NG、検査の境界、記録中の通信エラー、機種違い）、測定ファイルの破損検出、compare、apply（dry-run/確認なし、バックアップ不一致・測定時の Homing_Offset 不一致で書かない、不正な測定・バックアップ、全関節の順序と yaml の 1 回更新、シンボリックリンク、一部の関節だけ（yaml 不変）→ restore で戻す、途中失敗で書いた全関節をバックアップへ戻す、巻き戻しも失敗（電源断）→ 旧/新/不明 → 電源入れ直し後に restore で戻る、Lock=1 を確認できない、yaml を更新できない、通信エラーの再試行）。`so_arm101_hardware` 84 件も通過。`colcon build`（8 パッケージ）成功。
- 偽サーボでの CLI の通し確認（`tools/fake_sts3215.py`。実機・`/dev/ttyACM0` は開いていない）: backup → measure（fifo で Enter と姿勢の変更を与えて 2 回）→ compare → apply の dry-run → apply `--write`（負の Homing_Offset −47 を含む。yaml が更新され、差分は source・center・min/max・eeprom）→ backup で新しい値を確認 → restore `--write`（最初のバックアップ）→ backup し直して**最初のバックアップと全バイト一致**。
- 手順書 `docs/so_arm101_calibration_check.md` に C1・判断・C2・C3・失敗時の共通の対処を追記（C3 の URDF 余裕の確認は `tools/pose_margin.py`）。
- 未実施・実機でしか確認できないこと（計画 6 章どおり）: Lock の実動作と EEPROM の書き込み待ち時間（C0.5 で実測）、Min/Max limit が Present_Position の読み値に影響しないこと（推測）、負の offset の実機での挙動、全 40 バイトを 8 バイトずつ読めること。
- 次: ユーザーが C0（バックアップ）から実機確認を行う。EEPROM への最初の書き込み（C0.5）の前に、ユーザーの確認を取る。

## 段階2c 実機確認: C0（バックアップ）合格（2026-11-11）
- 実施: ユーザー（Claude は実機・シリアルポートに触れていない）。読み取り専用の確認で、EEPROM には何も書いていない。結果はユーザーの報告による。
- `calib_C0_backup.log`: 6 関節すべてで「2 回読んで一致」。Homing_Offset と Min/Max は yaml の `eeprom.*` と一致。終了コード 0。
- バックアップ: `~/so_arm101_check/eeprom_backup_20261111-163729.json`（1843 バイト）。消さない（以降の復元の元）。
- `calib_C0_dryrun.log`（`restore` の dry-run）: 前提の確認 6/6 OK、18 レジスタすべて「同じ（書かない）」、dry-run、終了コード 0。
- 事実（実機で通った）: EEPROM の 40 バイト（アドレス 0〜39）を 8 バイトずつ読むこと。
- 終了コードの取り方: パイプの後に `echo "exit=${PIPESTATUS[0]}"` を付けて取った（`tee` につなぐと `$?` は `tee` の値になるため）。
- 次: C0.5（実機で初めて EEPROM に書く。gripper だけ、バックアップと同じ値）。実行前にユーザーの確認を取る。

## 段階2c 実機確認: C0.5（復元の経路・最初の EEPROM 書き込み）合格（2026-11-11）
- 実施: ユーザー（Claude は実機に触れていない）。対象は gripper 1 関節、書いた値はバックアップと同じ（値は変わらない）。結果はユーザーの報告による。終了コードは `echo "exit=${PIPESTATUS[0]}"` で取った。
- `calib_C05_dryrun.log`: gripper の 3 項目が「同じ（`--rewrite-same` で書き直す）」、exit=0。
- 最初の書き込み（`calib_C05_w50.log`）: 確認文字列を入力せずに Enter を押したため「確認文字列が一致しない。何も書かない」、exit=4。確認文字列の安全装置が実機で働いたことの確認になった。
- 待ち時間 50 / 20 / 10 / 5 / 0 ms の 5 組（`calib_C05_w50_2.log`、`w20`、`w10`、`w5`、`w0`、各 `after_w*.log`）はすべて合格:
  - (a) 書き込み後の値はバックアップと同じ、gripper Lock: 1、`OK: バックアップの値に戻り、Lock=1 を確認した`、exit=0、ERROR なし、腕は動かなかった。
  - (b) C0 の dry-run で 18 レジスタすべて「同じ」、注意なし、exit=0。
- `calib_C05_final.log`: 18 レジスタすべて「同じ」、exit=0。
- 事実: 0 ms でも同じ値の書き込みは失敗しなかった（Lock の 0/1 の切替と、3 レジスタの書き込み・読み戻しが実機で通った）。
- 解釈: 同じ値の書き込みでは、読み戻しが書く前から一致するので、**必要な待ち時間はこの試験では分からない**。手順書にあった「最小の待ち時間の 2〜3 倍」の決め方は当てはめない。
- 決定: `--eeprom-wait-ms` の既定は 50 ms のまま（定数は変えない）。値が変わる書き込み（C2）を 50 ms で行い、各項目の読み戻しの一致を記録する。
- 次: C1（測定 2 回。EEPROM には書かない）。

## 段階2c 実機確認: C1（測定 2 回）と判断（2026-11-11）
- 実施: ユーザー（Claude は実機に触れていない）。結果はユーザーの報告による。`measure` は EEPROM にも Lock にも書かない。
- 2 回とも全関節の検査 OK、終了コード 0。`compare`（`calib_C1_compare.log`）も終了コード 0。測定ファイルは `~/so_arm101_check/measure_C1_1.json`、`measure_C1_2.json`。
- `zero_shift`（0 rad の物理位置の移動量。今の yaml との差）: A・B とも全関節で ±14 tick 以内（最大 0.0215 rad）。
- A と B の `zero_shift` の差（物理的な再現性）: pan 16、lift 2、elbow 6、wrist_flex 7、wrist_roll 0 tick。
- pan は 2 回目のほうが min を 30 tick 先まで押し、0 rad が 16 tick 動いた。端の押し切り方が 0 rad に効く。
- 気づいた点: `compare` の「再現性」の表は新しい座標（Homing_Offset に依存）で比べているため、中央の姿勢の違いが差として出る（「最大の差 70 tick」は誤解を招く）。
- **判断（判断の区切り）**: 選択肢 1「新しい値を書く」で進める（キャリブは ROS のツールで測った値にそろえる）。問題が出たら、その時点で考える（バックアップから戻せる）。
- **ただし先にツールを改善し、改善したツールで C1 をやり直す。C2・C3 は、やり直した C1 の測定ファイルを使う。今回の `measure_C1_1/2.json` は書き込みに使わない。**

## 段階2c-3a/3b: ツールの改善（C1 のやり直しの前。2026-11-11）
C1 の判断で「新しい値を書く」を選んだが、先にツールを改善し、C1 をやり直す（C1b）。今回の `measure_C1_1/2.json` は書き込みに使わない。実機・シリアルポートは開いていない（偽サーボと pty のみ）。

### 2c-3a: compare の再現性（commit 9243da2）
- 再現性の表を、生のエンコーダ値（`actual_min`/`actual_max`。Homing_Offset に依存しない）で比べる表にした: `d_amin`、`d_amax`、`d_span`、`d_zero`（0 rad の物理位置＝範囲の中点の差。tick と rad。angle のみ）、`d_cent`（中央の姿勢の差。参考で、最大の差には入れない）。「最大の差」は `d_amin`・`d_amax`・`d_zero`。差は mod 4096 で折り返す（wrist_roll のように 4095 をまたぐ関節も同じ）。
- gripper は `d_amin`・`d_amax`・`d_span` だけ（`d_zero` と `d_cent` の意味は無い旨を表に注記）。
- 実際の C1 の 2 ファイルで確認（読み取りのみ。ポートは開かない）: `d_zero` は pan −15.5、lift −2.0、elbow −5.5、wrist_flex −7.0、wrist_roll 0.0 tick。ユーザー報告の物理的な再現性（pan 16、lift 2、elbow 6、wrist_flex 7、roll 0）と一致。旧版の「最大の差 70 tick」は、新しい表では 27.0 tick（pan の `d_amin`）になり、中央の姿勢の違い（`d_cent`）は別の列で参考として出る。
- 試験を 3 件追加（中央の姿勢だけ違う同じ範囲、4095 をまたぐ wrist_roll の折り返し、gripper）。

### 2c-3b: measure の Enter 2 回と、記録中の表示
- 流れ: （トルクが入っていれば腕を支えて Enter）→ 中央の姿勢で Enter → 「中央を測定中（約 0.3 秒。動かさない）」→ 「中央の測定 OK、記録中」→ そのまま記録 → Enter で終了。Enter は、トルク OFF で 2 回、トルクが入っていれば 3 回。記録の開始の Enter は無くした。
- 中央の測定中に押した Enter が残っていると記録がすぐ終わるので、記録を始める直前に `tcflush` で入力を捨てる（端末のとき）。pty で、中央の測定中に余計な Enter を送っても記録が続くことを確認した。
- 記録中の表示: 関節ごとに「今・min・max・幅」（中央からの差。tick）と、**前回の端まで あと N tick**（min 側は 記録した min − yaml の min、max 側は yaml の max − 記録した max。どちらも「あと N」が正。届いたら「届いた」、超えたら「届いた(+N)」）。直近 1 秒に更新された端には `*`。
- 今の yaml の生の範囲（`min_limit/max_limit + Homing_Offset`。wrist_roll は約 1328〜5129 で 4095 をまたぐ）は、範囲の中点を mod 4096 で測った中央からの差に折り返して使い、幅は折り返さない（`yaml_range_around`）。
- 出力先: 端末は `/dev/tty` を直接開き（`O_NONBLOCK`。詰まったらそのフレームを捨てる）、約 0.2 秒ごとに 7 行を書き換える。ログ（stdout。tee につながる）には ANSI を出さず、1 秒ごとに 1 関節 1 行（同じ表記）。記録の最後に「記録: 読み取り N 回（失敗）、間隔 平均/最大、表示 K 回（合計 ms、最大 ms）」を出す。
- 試験（偽サーボ）: `colcon test` で `so_arm101_calibration` 58 件、`so_arm101_hardware` 84 件が通過。追加: Enter の回数と順序、入力の破棄が中央と記録の間に 1 回、中央で動いたら記録しない、あと N の向き（min 側・max 側で同じ読み方）、`yaml_range_around` の mod 4096 の折り返し、wrist_roll（yaml の範囲が 4095 をまたぐ）の差が端末の表示とログで正しいこと、表示・ログの間隔、**表示ありなしで読み取りの回数が同じ（SYNC_READ が 1 周期 1 回）で、間隔が許容範囲内**。
- pty での確認（`tools/pty_measure_check.py`。偽サーボ `tools/fake_sts3215.py` と、制御端末つきの pty。stdout はパイプ）。記録 6 秒、同じ動き:

  | | 改善前（9243da2） | 改善後 |
  |---|---|---|
  | 終了コード | 0 | 0 |
  | 記録中の SYNC_READ（実時間 6.14 s） | 301 回（49.0 回/秒、間隔 20.4 ms） | 301 回（49.0 回/秒、間隔 20.4 ms）。もう一度の実行は 313 回（スナップショットのタイミング差）。ツールの統計は読み取り 303 回、間隔 平均 20.4 ms／最大 20.7 ms、失敗 0 |
  | 表示にかかった時間 | - | 30 回で合計 0.8〜1.0 ms（最大 0.1 ms） |
  | stdout の ESC | なし | なし |
  | 端末のカーソル上移動（ESC[7A） | 0 | 29 回 |
  | ログの 1 秒ごとの行 | 0 | 36 行（6 関節 × 6 秒） |

  読み取りの回数と間隔は、表示を足しても変わらない。
- 事実: `colcon test` は、ビルド直後など CPU が忙しいときに `Measure.CommErrorsWhileRecordingAreTolerated` が不安定に落ちることがある（今回の変更を stash した状態でも、8 回中 2 回落ちた＝変更とは無関係。単独では 40 回すべて通る）。原因は未調査。

### 2c-3c: 測定ファイルの schema 2、手順書（C1b・判断の区切り・C2・C3）
- 測定ファイルの形式を `schema_version` 2 にした（項目は 1 と同じ。ツール名は `so_arm101_calib 0.2.0`）。チェックサムは、schema が 2 以上のときだけ `schema_version` も含める（v1 のファイルの検証は変わらず、v1 を v2 に書き換えると検出される）。
- `parse_measure` は 1 と 2 を読む。`apply` は 1 を終了コード 4 で拒否する（`main` で、ポートを開く前。`run_apply` でも検査）。`compare` は 1 を読み、「旧形式。apply には使えない」と警告する。
- 実ファイル（読み取りのみ。存在しないポートを指定して、ポートが開かれないことも確認）: 旧版の `measure_C1_1.json` を `apply` に渡すと終了コード 4、`旧形式（schema_version 1）…何も書かない（ポートも開かない）`。同じファイルの `compare` は警告つきで終了コード 0。
- 偽サーボ + pty（`tools/pty_measure_check.py`）: 新しい `measure` が `schema_version` 2・`so_arm101_calib 0.2.0` のファイルを保存し、それを `apply` の dry-run に通すと終了コード 0、書き込み 0（`writes.jsonl` の行数 0）。記録 4 秒で、読み取り 203 回（間隔 平均 20.4 ms）、表示 20 回（合計 0.8 ms）。
- 試験: `so_arm101_calibration` 60 件、`so_arm101_hardware` 84 件が通過（`colcon build` も成功）。追加: v2 の往復、schema の改ざん（2→1、1→2、0、3）の検出、v1 の読み込み、`measure_not_applicable_reason`、`compare` の旧形式の警告、`apply` の旧形式の拒否（書き込み 0、dry-run も拒否）。
- 手順書: C1 を C1b（ファイル名 `measure_C1b_*.json`、ログ `calib_C1b_*.log`。Enter 2 回の流れ、記録中の表示の見方、読み取りの統計の見方、前回の参考値）に書き直した。判断の区切りは `d_amin`・`d_amax`・`d_zero` を見る。C2・C3 は C1b の測定ファイルを使い、`--eeprom-wait-ms` は 50。終了コードの表に旧形式を追記。C2・C3 のコマンドにも `; echo "exit=${PIPESTATUS[0]}"` を付けた。
- **別件（未対応。承認が要る）**: `Sts3215Bus` は要求の直前に入力を捨てるが（`sts3215_bus.cpp:100`）、直前の失敗した SYNC_READ の残りの応答が、捨てた後に届くと取り違える（試験で `timeout: no answer from id 1 (id 4 answered first)`）。`Measure.CommErrorsWhileRecordingAreTolerated` が不安定に落ちる原因とみられる。実機でも、通信エラー後の再試行で起こりうる（確率は低いとみられる。推測）。直す案: 再試行の前に、残りの応答が出尽くす時間（1 Mbps で 6 関節ぶんの数 ms）待ってから入力を捨てる。`so_arm101_hardware` の変更になるので、承認の後にする。

## 残課題: バス層の古い応答の取り違え（調査のみ。コードは変えていない。2026-11-11）
きっかけ: 試験 `Measure.CommErrorsWhileRecordingAreTolerated` が不安定に落ちる（`timeout: no answer from id 1 (id 4 answered first)`）。実機・シリアルポートは開いていない（コードの読解と、偽サーボの試験の出力だけ）。**[事実]** = コードの行番号で確認、**[推測]** = 未確認。行番号は `src/so_arm101_hardware/src/sts3215_bus.cpp`（以下「bus」）、`sts3215_write_bus.cpp`、`sts3215_system.cpp`、`so_arm101_calibration/src/eeprom_ops.cpp`（以下「ops」）。

### 1. 応答の検査（ID・長さ・チェックサム）と、別 ID／古い応答の扱い
- **[事実]** 要求の前に受信バッファを捨てる（`begin_request`、bus:98-103。`flush_input` は `tcflush(TCIFLUSH)`、`serial_port.cpp:132-139`）。失敗時にも捨てる（`fail`、bus:114）。**捨てた後に届くバイトは、どこでも検査されずに次の読み取りに入る。**
- **[事実]** `receive_status`（bus:118-185）の検査は、この順序だけ。
  1. ヘッダ `FF FF` を探す。`FF` 以外は読み捨てる（最大 64 バイト、bus:124-135、`kMaxJunkBytes`=64）。
  2. ID・LEN を読む。LEN が 2〜250 の外は `BadFrame`（bus:144-146）。
  3. ERR+DATA+CHK を LEN バイト読む（タイムアウト 10 ms、bus:147-152、`sts3215_bus.hpp:95`）。
  4. チェックサム（**受信した ID** と LEN と本体で計算、bus:153-160）。不一致は `BadChecksum`。メッセージの ID は**受信した ID**（bus:159）。
  5. 受信した ID が期待と違えば `IdMismatch`（bus:161-166）。**チェックサムの検査が先**。
  6. ERR が 0 でなければ `ServoError`（bus:167-177）。
  7. データ長が期待と違えば `BadFrame`（bus:178-182）。
- **[事実]** ステータスパケットに、要求の命令・アドレス・通番を示す項目は無い（STS3215 のプロトコルで、バスは検査できない）。したがって**「期待の ID と同じ ID で、データ長が同じで、チェックサムが正しい」古い応答は、新しい要求の応答として受け取られる**。別のレジスタの応答でも、同じレジスタの古い値でも区別できない。
- **[事実]** `IdMismatch` は通信エラーではない（`is_comm_error` は Timeout・BadFrame・BadChecksum だけ、`sts3215_bus.hpp:116-119`）。呼び出し側は再試行せず失敗にする（例: ops:19、`sts3215_system.cpp:541`「ID 違い等は再試行しない」）。ただし SYNC_READ だけは、先に来た ID が**これから来る ID の中にある**ときに `Timeout`（＝通信エラー、再試行される）に読み替える（bus:225-232。試験のメッセージはこの経路）。
- **[事実] 古い応答が残る経路**（どちらも「捨てた後に届く」）:
  1. SYNC_READ が途中の ID で失敗したとき（bus:222-237）。例: id 1 の応答が壊れた（`BadChecksum`）と、その場で返る。id 2〜6 の応答はまだバスを流れている（1 Mbps で 1 フレーム約 80 µs。USB の遅れも加わる）。`fail` の捨てる処理（bus:114）と、再試行の `begin_request`（bus:100）は、その直後（数十 µs〜）に行われるので、残りの応答は**捨てた後**に着く。
  2. 要求のタイムアウト（10 ms）の後に、サーボの応答が遅れて着いたとき（[推測] 起こるかは未確認）。
- **[事実] 受け取ってしまう経路があるか**:
  - **SYNC_READ → SYNC_READ（再試行）: 受け取らない。** 古い応答は ids[i+1..] の ID で、再試行が最初に期待するのは ids[0]。ID が重複しないので、必ず `IdMismatch`（→ 上の読み替えで Timeout）になり、再試行がやり直しになる（誤った値は受け取らない。試験の失敗はこの経路で、再試行 3 回の間に残りが出尽くさないと失敗する）。
  - **SYNC_READ（失敗）→ 単独の READ（同じ ID、同じデータ長）: 受け取りうる。** 例: Present_Position（2 バイト）の SYNC_READ が id 1 で失敗し、id 2 の古い応答が残る → 直後の `read_u16(id=2, 別の 2 バイトのレジスタ)` は、古い Present_Position を答えとして受け取る。データ長が違うレジスタ（1 バイト、4 バイト）なら bus:178 で `BadFrame` になる。
  - **WRITE の ACK（データ無し、LEN=2）→ 読み戻し（2 バイトなら LEN=4、1 バイトなら LEN=3）: 受け取らない。** 長さの検査（bus:178）で `BadFrame`（再試行される）。
  - **WRITE の ACK → 次の WRITE の ACK:** 古い ACK を新しい ACK として受け取る（同じ ID）。書き込み自体は行われているので、書き込みの結果は変わらない。ACK が書き込みの成功を保証するわけではない点は、もともとの限界。

### 2. EEPROM 書き込みの読み戻しで古い応答を受け取った場合（`so_arm101_calibration`）
- **[事実]** 読み戻し: 書く → `wait_ms` 待つ → `read_calib_reg`（ops:14-22、`write_calib_verified` ops:58-76）。通信エラー（Timeout・BadFrame・BadChecksum）は最大 3 回再読み、`IdMismatch` は再試行せず `read-back failed`、値が違えば `read-back mismatch`（**再試行しない**、ops:71-74）。失敗したら `write_joint_sequence` が Lock=1 を試み（ops:117-122）、`apply` は触れた全関節をバックアップへ戻す（`apply.cpp:259-269`）、`restore` は残りの関節へ進まず Lock=1 に戻す（`restore.cpp:158-167`）。
- **[事実] 「不一致 → 失敗 → 戻す」に倒れる場合**: 古い応答が別の値（書く前の値、別のレジスタの値）のとき。Homing_Offset・Min・Max は値が互いに違うので、別のレジスタの応答は不一致になる。`IdMismatch`、長さの違い（ACK）も失敗になる。**安全側に倒れる**（本当は書けていても失敗扱いになり、戻す処理が走る）。
- **[事実] 誤って「一致」になりうる経路**: 古い応答の値が、期待値と同じ**だけ**のとき。
  - 古い応答が、書いた後の時点のその関節のその値なら、実際の値でもあるので、読み戻しとして正しい。
  - 同じ値を書く場合（C0.5 の `--rewrite-same`、バックアップと同じ値）は、書く前の古い値も期待値と同じ。一致しても、実際の状態も同じ。
  - **値が違うのに一致する**のは、別のレジスタの値がたまたま期待値と同じ場合だけ（同じ関節の Homing_Offset・Min・Max が同じ値になる、など）。実際の測定値ではまず起きない（[推測]）。
- **[事実] 残る穴（古い応答とは別）**: 読み戻しはサーボが返す値で、EEPROM に永続したことの証明ではない。これは C2 手順 3（電源の入れ直し後に `backup` で読み直す）が確かめる。
- **[事実] 最終の状態表**（`restore.cpp:168-187`、`apply.cpp:270-287`）も、同じ読み取りを使う。古い応答を受け取ると「旧／新」を取り違えうる（例: 「全部旧」と判断して終了コード 1）。確率は低い（上の経路が前提）が、判断の独立した確認として、**失敗後は電源の入れ直しと `backup` の読み直し**が要る（手順書の「失敗したとき」に近い。明示するのは実装時に）。

### 3. 起動時の `checksum mismatch`（W0 の 2・3 回目）はこの仕組みで説明できるか
- **[事実]**（`docs/inventory.md` 段階2b-3c W0）: Goal_Position の読み戻しで `checksum mismatch`（2 回目は id=1、3 回目は id=5）。タイムアウト（10 ms）ではなく、チェックサム不一致。メッセージの ID は**受信した ID**（bus:159）で、期待の ID と同じ（1、5）。受信バイト列は取れていない（当時は `tx= rx= flushed_before_request=` を出す版の前）。それ以降の起動は 0 回の再試行が続いている。
- **[事実]** その時点までの起動手順（`sts3215_system.cpp` 5 の手前）では、直前の読み取りがすべて成功している（初期位置の SYNC_READ、Lock・Acceleration・Torque_Limit の読み戻し）。失敗した読み取りが無いので、**1 の経路（失敗した SYNC_READ の残り）は、このときは前提が成り立たない**。
- **[事実]** 古い応答の完全なフレーム（別 ID、同じ ID）は、チェックサムが正しいので `BadChecksum` にならない（別 ID なら `IdMismatch`、同じ ID なら受け取られる）。**`checksum mismatch` で、かつ ID が期待どおりという結果は、古い完全なフレームの混入では説明できない。**
- **[推測]** 説明できる可能性が残るもの（どれも未確認）:
  - 部分的なフレームの残り: 直前のフレームの末尾が捨てた後に届き、ヘッダ探索（bus:124-135）が途中のバイトから始まる。ただし ID が期待と一致する偶然が要る（可能性は低い）。
  - 配線・USB・サーボの電源による、バイトの化け（チェックサムが壊れたフレーム）。
- **結論**: 今の事実では、W0 の `checksum mismatch` を「古い応答の取り違え」では説明できない（説明できないことが分かった、という意味）。原因は未特定のまま。次に起きたときは `rx=` と `flushed_before_request=` で切り分ける（`flushed_before_request` が 0 でなく、`rx` に別のフレームの断片があれば、古いバイトの混入）。C1b の記録の統計（`失敗 N 回`）は、実機の通信エラーの頻度の目安になる（コードは変えない）。

### 4. 直す案の計画（実装は C3 の後、C4 の前。承認の後）
- 時期の理由: `apply`/`restore`（C2・C3）は同じ `receive_status` を使うので、C2・C3 の前にバス層を変えると、C2・C3 の合格の意味が変わる。C4（書き込みモードの確認のやり直し）で、ドライバの変更を実機で確かめる。
- **案 A（本命）: 失敗したら、バスが静かになるまで待ってから捨てる。** `fail()` の後と SYNC_READ の失敗の後に、「入力が 2〜3 ms 途切れるまで読み捨てる」（上限 20 ms 程度）を入れる。正常な経路には影響しない（失敗したときだけ）。1 Mbps で 6 関節の応答は 1 ms に満たない（80 µs × 6 に返信遅延を足したもの）。USB の遅れ（数 ms）を見込んだ値は、実機で決める。
- **案 B: SYNC_READ が途中で失敗したら、残りの ID の応答を（短いタイムアウトで）読み切ってから返す。** 残りが来ることを前提にするので、A と併用（来ない場合は A の静寂待ちに任せる）。
- **案 C（多重の確認）**: EEPROM の読み戻しを 2 回続けて同じ値で一致させる（古い応答が 1 回混ざっても通らない）。呼び出し側（`so_arm101_calibration`）だけの変更。失敗後の最終状態表も同じ。
- **案 D（観測）**: バス層に数を持たせる（`flushed_before_request` の合計、`IdMismatch` の数、`no answer` の読み替えの数）。`deactivated.` の行と `measure` の統計に出す。A・B の効果を実機で比べるため。
- 試験: 偽サーボに「応答を ID ごとに遅らせる」機能を足し（`fake_servo.hpp`。今は応答が即時なので、競合は pty の偶然に依存している）、(1) 途中で失敗した SYNC_READ の再試行が、残りの応答を**確実に**捨てて成功する、(2) 単独の READ が、古い応答（同じ ID・同じデータ長）を受け取らない、(3) 読み戻しの古い応答が「一致」にならない、を決定的に確かめる。`Measure.CommErrorsWhileRecordingAreTolerated` は決定的になる見込み。
- 実機での確認（C4 の前に、読み取り専用で）: 記録の統計と `startup_read_retries`・`flushed_before_request` で、通信エラーの頻度が変わらない／減ることを見る。
- **今は直さない**（ユーザーの判断）。この節を、C3 の後の計画の元にする。

## 残課題: 不安定な試験 `Measure.CommErrorsWhileRecordingAreTolerated`（無効にしない）
- 現象: `colcon test` や単独実行で、ときどき失敗する（終了コード 3、`ERROR: Present_Position を読めない: timeout: no answer from id 1 (id 4 answered first)`）。観測: 変更前（2c-3a の前）の状態で 8 回中 2 回、ビルド直後の単独実行で 6 回中 3 回、別の日の 40 回連続では 0 回。`colcon test` 全体は通ることが多い。
- 原因（[事実] メッセージと上の 1 の経路、[推測] 競合のタイミング）: 試験は id 1 の Present_Position の応答を 3 回壊す。壊れた応答で SYNC_READ が失敗した直後に、id 2〜6 の応答が捨てた後に着くかどうかが、偽サーボ（pty）のタイミング次第。着くと再試行も `no answer` になる。
- 扱い: 試験は無効にしない（実機で起こりうる経路を表している）。直るのは、上の案 A・B の実装と、偽サーボの遅延機能の追加の後（C3 の後、C4 の前）。それまで、`colcon test` が 1 件だけ落ちたら再実行する（変更に起因するかは、この試験名かどうかで判断する）。

## 段階2c-3d: C1b の 1 回目の結果と、記録中の表示の修正（2026-11-11）
### C1b の 1 回目（ユーザー報告。`calib_C1b_1.log`、`measure_C1b_1.json`）
- 全関節の検査 OK、終了コード 0。読み取り 3170 回で**失敗 0**、間隔 平均 21.6 ms／最大 22.6 ms（偽サーボでは 20.4 ms。実機は 1 回の読み取りに約 1.6 ms かかる）。実機の通信エラー（バス層の古い応答の残課題の頻度の目安）: この回は 3170 回中 0。
- elbow_flex の max 側は「あと 26」のまま届かなかった（前回の C1 でも max 側が 10〜14 tick 足りなかった）。**原因: 最後の押し込みの強さによる差（ユーザーの判断。下の「C1 の合否の判断」）。許容する。** 角度: 1 tick = 360/4095 ≈ 0.0879° なので、26 tick ≈ 2.3°（ユーザーの報告は「約 1.1°」。1.1° は約 12.5 tick で、前回の 10〜14 tick の足りなさに当たる。**角度の数値は要確認**。下に注記）。
- 要望: 実行は `2>&1 | tee -i` なので、標準出力に出した 1 秒ごとの行（`[64 s] …`）が画面にも流れ、書き換える表を押し流して見にくい。表の見出しも 2 回出ていた。
- **C1b の 1 回目は参考として残し、書き込みには使わない。ツールを直して、C1c として 2 回ともやり直す。**

### 修正（コード）
- **記録中は標準出力に何も出さない**（1 秒ごとの行を廃止。`log_every_ticks` と `format_log_line` を削除）。画面に出るのは `/dev/tty` の書き換える表だけ。
- 記録が終わったら、端末の表を消し（`ESC[7A` + `CR` + `ESC[J`。`TtyLive::clear`、`MeasureIO::on_live_end`）、最後の時点の表を標準出力に **1 回だけ**出す（同じ列。`*` は付けない）。続けて `記録: 読み取り…` の行、測定結果の表。ログに残るのはこれだけ。
- 見出しは 1 回（最後の表の見出し。端末の表は消す）。表の幅は 84 桁から **70 桁**に縮めた（見出しを `min 端まで`・`max 端まで` にして列幅を縮めた）。**[推測]** 80 桁の端末で、84 桁の表が折り返して、書き換えの行数がずれ、見出しが残った（2 回出ていた）ことの一因とみられる。標準出力の 1 秒ごとの行の混入が主因（行数がずれる）。
- `/dev/tty` が開けないとき: `注意: 端末（/dev/tty）を開けないので、記録中の表示は出さない` を出し、**表示なしで記録だけ続ける**（`on_live` なし。最後の表と結果は出る）。

### 確認
- `colcon build` 成功。`so_arm101_calibration` の試験は、変更した試験を含めて通過（不安定な既存の試験 `Measure.CommErrorsWhileRecordingAreTolerated` は、1 回の実行で落ちることがある。上の残課題）。追加・変更した試験: 記録中の標準出力が空（最初のフレームの時点）、最後の表と見出しが 1 回ずつ、順序（最後の表 → 記録: → 測定結果）、`*` は端末の表だけ、表示なしでも記録と最後の表が出る、表の幅が 72 桁以内、min 側と max 側で同じ「あと N」の向き（既存）。
- pty（`tools/pty_measure_check.py`。偽サーボ、制御端末つき、stdout はパイプ）: **記録中に標準出力へ出たバイト数 0**、`[N s]` の行 0、最後の表 1 回・見出し 1 回、順序どおり、端末に ESC[7A の書き換え 30 回と、終了時の消去（`ESC[7A CR ESC[J`）。stdout に ESC なし。読み取り 303 回（失敗 0）、間隔 平均 20.4 ms／最大 20.5 ms、表示 30 回で合計 0.9 ms（2c-3b と同じ）。
- 制御端末なし（`tools/pty_measure_notty_check.py`。`start_new_session`、stdin はパイプ）: 注意の行が出て、終了コード 0、最後の表と結果が出て、ESC なし。
- 手順書: C1b を C1c（`measure_C1c_1/2.json`、`calib_C1c_*.log`。上書きしない名前）に書き直し、2 回ともやり直す形にした。C1b の 1 回目は参考として残す（`apply` は受け付けてしまうので選ばないこと、と明記）。表示の見方を新しい仕様（記録中は端末の表だけ、終了時に最後の表、80 桁以上の幅、`/dev/tty` が開けないときの扱い）に直した。

## C1 の合否の判断（ユーザーの判断。2026-11-11）
- **MIN/MAX の差は、最後の押し込みの強さによるもの**で、**1° 程度なら実運用上は問題なし**とする。
- **elbow_flex の max 側の「あと 26」**は、原因未特定ではなく「押し込みの強さによる差」として扱う（ユーザーの判断）。許容する。
  - 注記（Claude の計算。ユーザーの判断は変えない）: 1 tick = 360/4095 ≈ 0.0879°。「あと 26」は約 **2.3°**、ユーザー報告の「約 1.1°」は約 12.5 tick に当たる（前回の C1 の 10〜14 tick ≈ 0.9〜1.2° とは合う）。「あと N」は**前の yaml の端**との差で、2 回の測定どうしの差ではない。**どちらの角度で許容したか（「1° 程度」の目安との関係）は、ユーザーに確認が要る。**
- **C1c の合格の目安**: 2 回で、**center（`zero_shift`）とおおよその可動範囲**が、測るたびのばらつき（前回の C1 は最大 16 tick ≈ 1.4°）と**同じくらいに収まっていれば合格**。**どちらかの測定ファイルで C2 に進む**（選んだファイルを以降の C2・C3 で同じに使う）。
- 手順書「判断の区切り」に反映済み。C1c の結果は、ユーザーが `calib_C1c_compare.log` を送った後に、この目安で確認する（Claude は数値を並べて報告し、合否は目安に照らして示す。進むかの最終判断はユーザー）。

## 段階2c 実機確認: C1c（測定 2 回のやり直し）合格（2026-11-11）
- 実施: ユーザー（Claude は実機に触れていない）。合格はユーザーの判断。数値は `~/so_arm101_check/` のログ（`calib_C1c_1.log`、`calib_C1c_2.log`、`calib_C1c_compare.log`）で確認した（読み取りのみ）。
- 2 回とも全関節の検査 OK、読み取りの失敗 0、終了コード 0。1 回目: 読み取り 3991 回、間隔 平均 21.7 ms／最大 22.3 ms、表示 399 回（合計 15.5 ms）。2 回目: 2838 回、21.7／22.3 ms、表示 283 回（合計 11.1 ms）。**記録中は端末の表だけが出て、ログには最後の表と結果だけが残った**（906c410 の直しは実機でも有効）。
- `compare`（終了コード 0）:
  - `d_zero`（0 rad の物理位置の差）: pan −10.5、lift 9.0、elbow −2.0、wrist_flex **−12.5（最大）**、wrist_roll 8.5 tick。
  - 端の差（`d_amin`・`d_amax`）の最大: **25 tick（wrist_flex の `d_amax`）**。`d_span` の最大は elbow −30 tick。
  - `zero_shift`（今の yaml との差）: A は最大 11 tick（wrist_roll、0.0169 rad）、B も最大 11 tick（elbow、0.0169 rad）。
  - gripper は yaml との差が大きい（Homing_Offset +440／+470、min/max −441〜−480）。gripper は中央の姿勢を使わず、Homing_Offset は中央の姿勢の位置（`actual_center` 2594）で決まる値なので、差が大きいこと自体は異常ではない。
- 端の到達（最後の表）: 1 回目はほぼ全部の端が「届いた」（elbow と gripper の max が「あと 1」だけ）。2 回目は lift の min が「あと 16」、elbow の max が「あと 18」。
- ユーザーの基準（ばらつき 16 tick 程度、約 1°。前回の C1 は最大 16 tick ≈ 1.4°）に照らして、ユーザーは範囲内と判断した。**Claude の注記**: `d_zero` と `zero_shift` は 16 tick 以内（最大 12.5／11）。端の差は 25 tick（≈ 2.2°）、幅の差は 30 tick（≈ 2.6°）で、16 tick の目安を超える箇所がある。ユーザーは「MIN/MAX の差は最後の押し込みの強さによる差で許容」と判断済み（上の「C1 の合否の判断」）。この判断で進める。
- **決定**: C2・C3 で使う測定ファイルは **`~/so_arm101_check/measure_C1c_1.json`**（`schema_version` 2、`so_arm101_calib 0.2.0`、全関節 valid、バックアップは `eeprom_backup_20261111-163729.json`）。このファイルの値: Homing_Offset / min / max（新座標）は pan 144 / 724 / 3454、lift 117 / 836 / 3211、elbow 119 / 849 / 3090、wrist_flex 143 / 808 / 3174、wrist_roll 1196 / 100 / 3943、**gripper 547 / 1454 / 2944**（今の EEPROM は gripper 107 / 1900 / 3385）。`measure_C1c_2.json`、`measure_C1b_1.json`、旧形式の `measure_C1_*.json` は書き込みに使わない。
- 次: C2（gripper に `measure_C1c_1.json` の値を書き、電源を入れ直して残ることを確かめ、バックアップへ戻す。待ち時間は 50 ms）。**値が変わる EEPROM の書き込みは、これが実機で初めて。** 実行前にユーザーが Claude に伝え、承認を得る。

## 計画（実装は C3 の後。今はツールを変えない）: 記録中の表示で「ほぼ届いた」を出す
- ユーザーの要望: 「届いた」の判定をゆるくしたい。**あと 10 tick 以内（約 1°。10 tick ≈ 0.88°、1° ≈ 11.4 tick）は「ほぼ届いた」のような表示にする。** 表示だけの変更で、実機での再確認は不要（ユーザーの判断）。
- 変更の内容（案）:
  - `format_gap`（`measure.cpp`）に近さの閾値（定数 `kNearTicks` = 10）を足す: remaining > 10 は `あと N`、1〜10 は `ほぼ あと N`、0 は `届いた`、負は `届いた(+N)`。min 側・max 側で同じ向き（今と同じ）。`*`（直近 1 秒に更新）は今のまま。
  - 列幅（14 桁）に収める: `ほぼ あと 10` は表示幅 12 桁。表の全体の幅は 70 桁のまま。
  - 測定ファイル・検査・`compare` は変えない（表示だけ）。
- 試験（偽サーボ）: `format_gap` の境界（0、1、10、11、負）、表の幅が 72 桁以内、min 側・max 側で同じ表記、最後の表と端末の表で同じ表記、既存の表示の試験が通ること。pty で、表示だけの差であること（標準出力に記録中は何も出ない、読み取りの回数と間隔が同じ）を確認する。
- 手順書: C1c の「表示の見方」に `ほぼ あと N`（あと 10 tick 以内）を足す。
- 時期: C3 の後（C4 の前でもよい。バス層の変更とは独立）。実機での再確認はしない（ユーザーの判断）。

## 段階2c 実機確認: C2（gripper だけ新しい値を書く → 電源を入れ直す → バックアップへ戻す）合格（2026-11-11）
- 実施: ユーザー（Claude は実機に触れていない）。合格はユーザーの判断。内容は `~/so_arm101_check/` のログで確認した（読み取りのみ）。**値が変わる EEPROM の書き込みは、これが実機で初めて。**
- `calib_C2_dryrun.log`: 照合 OK（全関節がバックアップと一致）、書き込む値は gripper だけ（Homing_Offset 107→547、Min 1900→1454、Max 3385→2944）、`yaml は更新しない`、終了コード 0。
- `calib_C2_apply.log`: 3 項目とも [新]、gripper Lock: 1、`OK: 測定ファイルの値を書き、読み戻しと Lock=1 を確認した`、`yaml は更新していない（一部の関節だけ）`、終了コード 0。**値が変わる書き込みで、待ち 50 ms の読み戻しが一致した（初めての実測）。** 待ち時間の下限（20 / 10 / 5 / 0 ms で値が変わる書き込みが通るか）は未測定。既定は 50 ms のまま。
- `calib_C2_after.log`: **12 V を入れ直した後**（USB はつないだまま。ポート名は `/dev/ttyACM0` のまま）、gripper は 547 / 1454 / 2944 のまま、他の 5 関節は元のまま（pan 135/743/3452、lift 112/842/3216、elbow 122/862/3088、wrist_flex 89/889/3185、wrist_roll 1164/164/3965）。6 関節とも 2 回読んで一致。`eeprom_after_C2.json` を保存して読み直し、一致を確認した。終了コードは、ユーザーがコピーを取り損ねて記録がない（保存と読み直しの一致の表示で代える）。**EEPROM の値が電源を切っても残ることを実機で確認した。**
- `calib_C2_restore.log`: gripper の 3 項目を「書く」で元に戻し、[バックアップ]、Lock: 1、`OK: バックアップの値に戻り、Lock=1 を確認した`、終了コード 0。
- `calib_C2_restore_check.log`（全関節の dry-run）: 18 レジスタすべて「同じ（書かない）」、終了コード 0。
- **確認済み**: **USB をつないだまま 12 V を入れ直してよい**（手順書の「未確認」を「確認済み」に直した）。
- 次: C3（全関節を `measure_C1c_1.json` で 1 回で書き、yaml を 1 回だけ更新する）。実行の前に、下の「C3 の準備」を確認する。

## C3 の準備（調べた事実と、決めること。2026-11-11。コードは変えていない）
### yaml の更新先（事実）
- `apply` は、既定で `ament_index` の `so_arm101_hardware` の share を引き、`…/share/so_arm101_hardware/config/follower_calibration.yaml` を使う（`main.cpp` の `cal_path`）。ドライバ（launch）も同じ場所を読む（`bringup.launch.py:89-91` の `calibration_file`。2b-4 のログの `calibration=` の行）。
- 今のワークスペースは **`colcon build --symlink-install`**: `install/so_arm101_hardware/share/so_arm101_hardware/config/follower_calibration.yaml` は **`src/so_arm101_hardware/config/follower_calibration.yaml`（git 管理）へのシンボリックリンク**（`readlink -f` で確認）。
- `update_calibration_yaml`（`apply.cpp`）は、`fs::canonical` でリンクを解決し、**実体（src 側）に `.new` を書いて読み直し、rename で置き換える**。リンクはそのまま有効。したがって**更新されるのは src 側の 1 ファイルだけで、install 側は同じものを指している**（そろえる手順は要らない。`colcon build` も要らない）。`--symlink-install` でないビルド（install 側がコピー）だと、install 側だけ古いままになる（今の環境では該当しない）。
- 書き込みの途中で失敗したときは、yaml は更新されない（6 関節すべてを書いて読み戻しと Lock=1 を確認した後の 1 回だけ）。yaml だけ更新できなかったときは、EEPROM は新しいまま、yaml は古いまま（書き込みモードは起動を拒否）。退避先は `~/so_arm101_check/follower_calibration_new_<日時>.yaml`。

### yaml の出所の記録（`source`）と、更新で変わるもの（事実）
- `render_calibration_yaml`（`apply.cpp`）は yaml を**作り直す**。`source` は次になる:
  ```yaml
  source:
    kind: ros_calibrated
    measure_file: <--measure に渡したパス>   # シェルが ~ を展開した絶対パス
    backup_file: <--backup に渡したパス>
    applied_at: <実行時刻、秒まで>
    applied_by: so_arm101_calib 0.1.0
  ```
  - **今の yaml の `source`（`converted_from_lerobot_json`、`file`、`file_date`、`converted_at`、`converted_by`）は消える**（git の履歴に残る）。ファイル冒頭のコメントも、ツールが出す 3 行に置き換わる。`schema_version`、`bus`、`conversion`（`angle_ticks: 4095`）、各関節の `id`・`kind`・`direction` は、今の値を引き継ぐ。
  - 各関節の `center`（angle のみ）・`min`・`max`・`eeprom.homing_offset / min_limit / max_limit` は、測定ファイルの値になる（`center` は新座標の `new_center`。`eeprom.*` は書いた値と同じ）。
- **気づいた点（要判断）**: `applied_by` は、コードに `so_arm101_calib 0.1.0` と固定されている（`apply.cpp:68`）。今のツールは 0.2.0（測定ファイルの `tool` は 0.2.0）。このまま C3 を行うと、yaml の出所に古い版が入る（動作には無関係。出所の記録の不正確さだけ）。直すなら 1 行（0.1.0 → 0.2.0）で、`colcon build` と試験が要る（実機は不要）。
- **気づいた点（要判断）**: `measure_file`・`backup_file` は、リポジトリの外（`~/so_arm101_check/`）を指す。yaml の出所をたどれるように、C3 の後に測定ファイル・バックアップ・`compare` のログをリポジトリへ複製する案（例: `calibration_records/2026-11-11/`。各 2 KB 程度）。実行するかはユーザーが決める。

### 更新前の yaml の控え
- 更新前の yaml は **git の履歴にある**（作業ツリーは clean。最後のコミットは `1e9e968`）。`apply` は yaml の控えを自分では作らない（失敗時の退避先だけ）。
- C3 の前に、念のため、リポジトリの外に複製を作った: `~/so_arm101_check/follower_calibration_before_C3.yaml`（sha256 `822dad7d46dd276ca8bd453645eb5f09e4f29dfaee63bbcbcfdf35b4841998a1`。`src/` 側と一致）。
- C3 の後に元に戻す手順: **EEPROM は `restore --backup …163729.json --write`（全関節。違う項目だけ書く）、yaml は、コミット前なら `git checkout -- src/so_arm101_hardware/config/follower_calibration.yaml`、コミット後なら `git revert <そのコミット>`**（履歴は書き換えない）。`restore` は yaml を更新しない。どちらも戻して、全関節 dry-run が「同じ」で、読み取り専用の起動で 6/6 MATCH になるまで、書き込みモードは起動しない。**yaml をコミットする時期は、C3 の手順 3〜5（読み取りのログ）が合格してから**（失敗したら `git checkout` で戻せる。コミット後は revert）。

### 出発点の姿勢で、新しい座標の値の見込み（計算。実測ではない）
> **注（2026-11-11 追記）**: 表の「旧」は旧校正の座標、「新の見込み」は C3 の前に計算した値。実測は「段階2c 実機確認: C3」。
- 同じ物理姿勢なら、`新 rad = 旧 rad − zero_shift × 2π/4095`（angle、direction=1）。`zero_shift` は `measure_C1c_1.json` と今の yaml の差。gripper は新しい範囲（1454〜2944、旧 1900〜3385）での %。
- 出発点（`start_pose.log` の調整後の値）からの見込み:

  | 関節 | 旧（今の yaml） | 新の見込み | 差 |
  |---|---|---|---|
  | shoulder_pan | −0.08362 | −0.08439 | −0.0008 |
  | shoulder_lift | −1.51134 | −1.51057 | +0.0008 |
  | elbow_flex | +1.66938 | **+1.68242** | +0.0130 |
  | wrist_flex | +0.50941 | +0.49714 | −0.0123 |
  | wrist_roll | −0.07135 | −0.05447 | +0.0169 |
  | gripper | +0.01645 | +0.02342 | +0.0070 |
- **elbow_flex は URDF の上限（+1.69）まで +0.0076 rad しか残らない見込み**（今は +0.0206）。`pose_margin.py` で見込みの姿勢を試した結果（キャリブレーションの範囲は今の yaml のまま、URDF の上限・下限までの残りだけ有効）: elbow_flex が LOW（仮の目安 0.05 rad 未満）。ほかの関節は ok。URDF・SRDF は変えない方針なので、**C3 の手順 5 で実測して報告し、判断を仰ぐ**（出発点の姿勢を手で少し開く、など）。

## C3 の準備（続き）: 版の表記の統一、リポジトリ内の複製、elbow の扱い（2026-11-11）
- **版の表記を 1 か所の定数に**（commit 08ae234）: `include/so_arm101_calibration/version.hpp` の `kToolName`・`kToolVersion`（"0.2.0"）と `tool_id()`。測定ファイルの `tool`（`measure.cpp`）、バックアップの `tool`（`main.cpp`。これまで 0.1.0 だった）、yaml の出所の `applied_by`（`apply.cpp`。これまで 0.1.0 で固定）が同じ値を使う。`package.xml` の `<version>` を 0.2.0 にそろえ、試験 `Version.ToolIdIsOneStringAndMatchesPackageXml` が `version.hpp` と `package.xml` の一致を照合する。測定・書き込み・検査の動きは変えていない。`colcon build` 成功、試験は `so_arm101_calibration` 62 件、`so_arm101_hardware` 84 件が通過。偽サーボの実行ファイルでの通し確認（scratchpad の yaml の複製を使い、リポジトリの yaml は変わらない＝sha256 同じ）: `backup`・`measure` のファイルの `tool` が `so_arm101_calib 0.2.0`、`apply --write` が `applied_by: so_arm101_calib 0.2.0` を書いた。
- **リポジトリ内の複製**: C3 の合格後に、測定ファイル・バックアップ・C3 の後の `backup` を `calibration_records/2026-11-11/` に複製し、更新した yaml と同じコミットに入れる（ユーザーの判断）。
- **yaml の出所にリポジトリ内のパスを書く方法（コードの変更なし）**: `apply` は `--measure`・`--backup` に渡した文字列を、そのまま yaml の `source.measure_file`・`backup_file` に書く（`main.cpp`、`apply.cpp` の `render_calibration_yaml`。偽サーボの通し確認で、渡した絶対パスがそのまま入ることを確認）。そこで、**C3 の前に**、原本の複製を `calibration_records/2026-11-11/`（`measure_C1c_1.json`、`eeprom_backup_20261111-163729.json`。sha256 が原本と一致。未コミット）に置き、C3 の `apply` を**リポジトリの直下で、相対パス**（`calibration_records/2026-11-11/…`）で実行する。コミットは合格後（上の判断どおり）。複製を先に置くのは、yaml をツールが書いたまま（手で書き換えない）にするため。測定ファイルの中の `backup_file` は、測定時のパスのまま（チェックサム付きなので書き換えない）。
- **elbow_flex の余裕**（見込み 0.008 rad、LOW）: C3 はこのまま進め、手順 6 の実測で判断する（ユーザーの判断）。LOW・OUT が出たら報告し、C4 の前に出発点の姿勢の見直し（elbow を少し開く）を検討する。URDF・SRDF は変えない。

## 出発点の elbow_flex が URDF の上限を 0.0001 rad 超える件（OUT）: 調査と計画（2026-11-11。コード・設定は変えていない）
きっかけ（ユーザー報告。`calib_C3_pose_margin.log`）: C3 の後の出発点（新しい座標）は pan −0.0982、lift −1.4830、**elbow +1.6901**、wrist_flex +0.5033、roll −0.0483、gripper +0.0152。elbow_flex の URDF の上限（+1.69）までの残りは **−0.0001 rad（OUT）**。事実（ユーザー）: elbow はトルク OFF では自重で折りたたまれるので、開いた姿勢では手を離して止まらない。出発点は、たたみ切った姿勢（機械的な止まり）で決まる。**[事実]** = ファイル・実行で確認、**[推測]** = 未確認。

### 1. 計画パイプラインの設定（事実）
- `so_arm101_moveit_config/config/` に `*_planning.yaml` は無い（`ompl_planning.yaml` など無し）。`MoveItConfigsBuilder("so101_new_calib", package_name="so_arm101_moveit_config").to_moveit_configs()`（`bringup.launch.py:44-62`、`move_group.launch.py:13-15`。`planning_pipelines()` を指定していない）が、`moveit_configs_utils` の既定の設定を読む（`/opt/ros/humble/lib/python3.10/site-packages/moveit_configs_utils/moveit_configs_builder.py:440-472`）。ビルダーを実際に読み込んで出力した結果（launch が使うのと同じ関数）: パイプラインは `chomp`、`ompl`、`pilz_industrial_motion_planner`、**既定は `ompl`**。
- **OMPL: `FixStartStateBounds` が入っている**（`request_adapters` に `default_planner_request_adapters/FixStartStateBounds`。`/opt/ros/humble/share/moveit_configs_utils/default_configs/ompl_planning.yaml:8`）。**`start_state_max_bounds_error: 0.1`**（同ファイル:2）。CHOMP も同じ（`chomp_planning.yaml:8, 12`）。
- **Pilz: `request_adapters: ""`**（`pilz_industrial_motion_planner_planning.yaml:2`）。境界を直すアダプタは無い。
- 当方の `joint_limits.yaml` は速度・加速度の制限だけで、位置のリミットは URDF（`so101_new_calib.urdf.xacro`）から来る。elbow_flex の URDF の範囲は ±1.6900。`allowed_start_tolerance` は設定していない（MoveIt の既定）。

### 2. この姿勢からそのまま Plan & Execute できるか（mock の move_group での実測。実機は使っていない）
- 方法: `bringup.launch.py mock:=true use_rviz:=false` を起動し、`/check_state_validity` と `/plan_kinematic_path`（計画だけ。実行しない）に、新しい出発点を `start_state` として渡した（elbow を開く目標 +1.5901、速度・加速度スケール 0.1）。後始末は `ps` と `ros2 node list` の両方で空を確認（`ros2 node list` だけに残ったので `ros2 daemon stop; ros2 daemon start` の後に再確認して空）。
- **OMPL（既定）: 出発点 elbow +1.6901 は計画に成功**（error_code 1、22 点）。出発点が上限を超えても、**0.0100 超（+1.7000）は成功、shoulder_pan の上限を 0.0001／0.0301／0.0701／0.0901 超えた出発点も成功、0.1301 超で失敗**（FAILURE）。`start_state_max_bounds_error: 0.1` の設定どおり。
- **Pilz PTP: 上限を超えた出発点は拒否される**: elbow +1.6900（上限ちょうど）は成功、**+1.69005、+1.6901、+1.6950、+1.7000 はすべて error_code −17（`INVALID_ROBOT_STATE`）で拒否される**。上限内（+1.6800）は成功。0.00005 rad の超過でも拒否する。
- `/check_state_validity` は境界を判定しない（elbow +1.7000、pan +2.01 でも `valid=True`）。衝突だけを見る。**新しい出発点は衝突なし（valid）**。同じ姿勢で elbow だけを開く向きとは逆（閉じる向き）に +1.74 まで進めると upper_arm_link と wrist_link が接触する（+1.70 は valid、+1.74 は invalid。接触の始まりは +1.70〜+1.74 の間）。つまり URDF の上限（+1.69）は、衝突の手前の安全側の値になっている。
- **[推測]** 実行: MoveIt の軌道の最初の点は出発点（+1.6901）のままで、現在の実測と同じなので、`allowed_start_tolerance`（既定 0.01 rad）に収まる。`open_loop_control: true`。実機では未確認（C4 で確認）。
- **[事実]** 手順書の流れ（`feetech_write_check.md` の W3.5 → W4）は、MoveIt を使う前に elbow_flex を `send_joint_delta.py` で開く（−0.10 を 2 回で約 −0.12。実測で +1.56 付近）。**MoveIt の計画は、elbow が上限内になってから行う**ので、手順書どおりならこの OUT は計画に影響しない。OUT が効くのは、(a) 起動直後に W3.5 を飛ばして計画するとき、(b) Pilz を使うとき、(c) W4 の後に出発点へ戻すとき（`send_joint_delta.py` は URDF の範囲を見ず、ドライバは校正範囲 ±1.7192 でクランプする。+1.6901 は範囲内なので戻せる）。
- **[推測]** 出発点の読み値は、たたみ切るたびに数 tick ばらつく。旧座標（C3 の前）の +1.66938 と新座標の +1.6901 の差は 0.0207 rad（約 13.5 tick）で、`zero_shift` から計算した見込み（+0.0130）より 0.0077 rad（約 5 tick）大きい（人が姿勢を作る再現性。C3 の合格の目安 0.03 rad 以内には収まっている: pan −0.0138、lift +0.0276、elbow +0.0077、wrist_flex +0.0062、roll +0.0062、gripper −0.0082）。上限を超える量は、今後も 0.01 rad 程度まで変わりうる。OMPL（0.1 まで許容）なら問題ない。

### 3. 対応の計画（承認が要るのは (c) だけ。URDF・SRDF は変えない）
- **(a) 推奨: 何も変えない。** OMPL（既定）は `FixStartStateBounds`（0.1）で扱える。手順書の W3.5 → W4 は elbow を開いてから計画するので、手順どおりなら問題が出ない。C4 の W4 の再確認で、新しい座標の出発点から実機で確かめる（RViz の MotionPlanning の既定は OMPL のまま。**Pilz を選ばない**）。手順書に「Pilz は、出発点が上限を超えると計画が拒否される（INVALID_ROBOT_STATE）。OMPL を使う」を書く。
- **(b) 補助（Pilz などを使うとき、または起動直後に計画したいとき）**: 書き込みモードで起動した直後に `send_joint_delta.py elbow_flex -0.10 --seconds 4` で elbow を開く。事実（W3・W3.5 の実測）: 開く向きは目標の約 0.04 rad 手前で止まるので、−0.10 の指令で約 −0.05 動く（+1.6901 → 約 +1.64）。−0.02 のような小さい指令は動かない可能性が高い。開いた姿勢のまま Ctrl-C すると腕が落ちるので、**戻してから止める**（W4 の「止め方」）。
- **(c) 最後の手段: URDF の elbow の上限を広げる（承認制。今回はしない）。** 校正した物理の範囲は ±1.7192、URDF は ±1.6900。ただし、閉じる向きは約 +1.70〜+1.74 の間で upper_arm_link と wrist_link が接触するので、広げるなら衝突の手前（例: +1.70 まで）にとどめる。広げると MoveIt の設定の変更になる。
- 採らない案: 校正をやり直して、たたみ切りの読み値を 1.69 未満にする（測定の再現性のぶん、また同じことが起こりうる。C3 を変えるだけで根本の解決にならない）。

### 4. 出発点の姿勢の値を、新しい座標に直す文書の一覧（実施は未。承認の後）
旧座標（C3 の前）の値: pan −0.08362、lift −1.51134、elbow +1.66938、wrist_flex +0.50941、roll −0.07135、gripper +0.01645。新座標: pan −0.0982、lift −1.4830、elbow +1.6901、wrist_flex +0.5033、roll −0.0483、gripper +0.0152。

| 文書 | 場所 | 直すこと |
|---|---|---|
| `docs/feetech_write_check.md` | 130〜139 行（出発点の表と `start_pose.log` への言及） | 新座標の値に直す。出所を `~/so_arm101_check/calib_C3_start_pose.log` に |
| 同 | 141 行（モックの MoveIt の判定） | 新しい出発点で再判定した結果（出発点は valid。elbow を −0.10／−0.15／−0.20／−0.30 にした姿勢も、モックで確かめ直す） |
| 同 | 151 行（`+1.66938` との差の計算）、169 行・174〜177 行（戻す先） | `+1.6901` に。「URDF リミット +1.69 まで 0.021 rad の余裕」を「−0.0001 rad（OUT）。OMPL は許容する」に |
| 同 | 167 行（目標の作り方の注意） | 余裕が 0.021 rad から無い（OUT）に変わったこと。elbow を + に動かさない注意は同じ |
| `docs/stage2c_plan.md` | 71 行（出発点の姿勢の表の 1 行） | 新座標に直し、「C4 の前に手順書を直す」を済みにする |
| `docs/so_arm101_calibration_check.md` | C3 の手順 5・6（見込みの表、LOW/OUT の記述） | 見込みの表を、実測の値（新座標）に直す。OUT の判断（上の (a)）を書く |
| `tools/pose_margin.py` | 5 行（使い方の例の `--pose start=…`） | 新座標の値に直す（動作は変えない） |
| `docs/inventory.md` | 1005・1030・1045・1056 行（W3・W3.5・W4 の記録）、1392〜1397 行（C3 の見込みの表） | **書き換えない**（記録。旧座標のときの値）。末尾に新座標の記録を足し、旧座標の記録には「旧座標（C3 の前）」と注記する |
| ユーザーの指示書（リポジトリの外） | 出発点の値、elbow の余裕 0.021 rad、W4 の戻し先 | ユーザーが新座標の値に直す（上の新座標の値） |
| `~/so_arm101_check/start_pose.log` | （リポジトリの外。ユーザーのファイル） | 旧座標の記録として残す。新座標は `calib_C3_start_pose.log` |
- 注: `CLAUDE.md`、SRDF の `home` 姿勢（elbow 1.57 など）、URDF は、出発点の値を含まない（変更しない）。

### 5. C3 の状態（事実）
- `src/so_arm101_hardware/config/follower_calibration.yaml` は C3 で更新済み（作業ツリーの変更。`source` は `calibration_records/2026-11-11/…` の相対パス、`applied_by: so_arm101_calib 0.2.0`）。**まだコミットしていない**（`calibration_records/` も未コミット）。C3 の結果の記録とコミット（手順書の「合格後のコミット」）は、ユーザーの報告と指示を受けて行う。

## 段階2c 実機確認: C3（全関節を `measure_C1c_1.json` で書く）合格（2026-11-11）
- 実施: ユーザー（Claude は実機に触れていない）。合格（手順 2〜6）はユーザーの判断。ログ（`~/so_arm101_check/calib_C3_*.log`、`eeprom_after_C3.json`）は Claude が読んで確認した。終了コード（`exit=`）の数値はログに残っていない（`echo` はログの外）。`OK` の表示で確認した。
- **dry-run**（`calib_C3_dryrun.log`）: 照合 OK（全関節がバックアップと一致）。書き込む値は 6 関節 18 レジスタ（`measure_C1c_1.json`）。`全関節を 1 回で書く。成功したら yaml を 1 回だけ更新する`、`dry-run: 何も書いていない`。
- **書き込み**（`calib_C3_apply.log`。待ち 50 ms、リポジトリの直下から相対パス）: 6 関節とも Homing_Offset・Min・Max が `[新]`、各 `Lock: 1`、`OK: 測定ファイルの値を書き、読み戻しと Lock=1 を確認した`、`yaml を更新した`。
  - 値（Homing_Offset / Min / Max）: pan 144 / 724 / 3454、lift 117 / 836 / 3211、elbow 119 / 849 / 3090、wrist_flex 143 / 808 / 3174、wrist_roll 1196 / 100 / 3943、gripper 547 / 1454 / 2944。
- **12 V を入れ直した後の読み出し**（ユーザー報告。`calib_C3_after.log`、`eeprom_after_C3.json`）: 6 関節とも「2 回読んで一致」。値は上のとおり。**EEPROM の値は、電源を切っても残る**（C2 に続き、全関節で確認）。
- **読み取り専用の起動**（`calib_C3_launch.log`、19:45。C3 の前の起動は `calib_C3_before_launch.log`、19:42）: `[diag] EEPROM vs yaml: 6/6 joints match`、`servo settings vs LeRobot: 6/6 joints match`、`MISMATCH` なし、`READ-ONLY mode`。C3 の前の起動も 6/6 MATCH（旧 EEPROM と旧 yaml）。停止時の move_group の exit code −11 は既知の挙動（段階2b-3c の記録）。
  - 事実（表示の不具合）: 起動ログの 1 行目の出所の表示が、C3 の前は `[converted_from_lerobot_json, file=…, file_date=…]` だったのに、C3 の後は **`[ros_calibrated, file=?, file_date=?]`**（`source` の項目が変わり、`file`・`file_date` が無いため `?`）。動作には無関係。直す計画は下の「計画」。
- **yaml**: `source` は `ros_calibrated`、`measure_file: calibration_records/2026-11-11/measure_C1c_1.json`、`backup_file: calibration_records/2026-11-11/eeprom_backup_20261111-163729.json`、`applied_at: 2026-11-11T19:43:04`、`applied_by: so_arm101_calib 0.2.0`。各関節の `center`・`min`・`max`・`eeprom.*` は、`measure_C1c_1.json` と全項目一致（コミットの前に照合）。`schema_version`・`bus`・`conversion`・`id`・`kind`・`direction` は変わっていない。
- **C3 の前後の比較**（ユーザー報告。腕に触れずに、同じ姿勢のまま、C3 の前と後に読んだ値）: 新座標の値は pan −0.09820、lift −1.48295、elbow +1.69009、wrist_flex +0.50327、roll −0.04833、gripper +0.01523。C3 の前の読み値に `zero_shift`（`measure_C1c_1.json` と旧 yaml の差から計算）を反映した見込みとの差は、**pan 以外は 0.0001 rad 以内、pan は約 1 tick（約 0.0015 rad）**。これは、測定ファイルの値と、書いた EEPROM・yaml・ドライバの変換が、一貫していることの確認になる。**C3 の前後の読み値（ユーザー報告。腕に触れずに同じ姿勢で読んだ値。rad。順に pan, lift, elbow, wrist_flex, roll, gripper）**と、手順書（手順 0・5）の比較スクリプトでの判定（Claude が実行。旧 yaml の複製 `~/so_arm101_check/follower_calibration_before_C3.yaml` と `calibration_records/2026-11-11/measure_C1c_1.json` を使用）:
  - C3 の前（旧校正）: −0.09897, −1.48372, +1.67705, +0.51554, −0.06521, +0.00823
  - C3 の後（新校正）: −0.09820, −1.48295, +1.69009, +0.50327, −0.04833, +0.01523

  | 関節 | before | after | 見込み（`zero_shift` から） | 差（実測 − 見込み） [rad] | 差 [tick] |
  |---|---|---|---|---|---|
  | shoulder_pan | −0.09897 | −0.09820 | −0.09974 | +0.00154 | +1.0 |
  | shoulder_lift | −1.48372 | −1.48295 | −1.48295 | 0.00000 | 0.0 |
  | elbow_flex | +1.67705 | +1.69009 | +1.69009 | 0.00000 | 0.0 |
  | wrist_flex | +0.51554 | +0.50327 | +0.50327 | 0.00000 | 0.0 |
  | wrist_roll | −0.06521 | −0.04833 | −0.04833 | 0.00000 | 0.0 |
  | gripper | +0.00823 | +0.01523 | +0.01523 | 0.00000 | （gripper は % の換算なので tick は出さない） |

  - **判定: 合格**（目安は各関節で 0.003 rad（約 2 tick）以内）。pan 以外は 0.00001 rad 未満（丸めの範囲）で一致、**pan は +1.0 tick（+0.00154 rad）**。ユーザー報告（pan 以外は 0.0001 rad 以内、pan は約 1 tick）と一致する。
  - 意味: 測定ファイルの値（`zero_shift`）、書いた EEPROM、yaml、ドライバの変換が、一貫している。elbow_flex は新座標で +1.69009（URDF の上限 +1.69 を約 0.00009 rad 超える。対応は (a)）。
  - 旧座標の elbow の読み値は +1.67705。C3 の前の別の日の W3.5/W4 の出発点（+1.66938）と比べて約 0.0077 rad 違う（姿勢を手で作り直すぶんのばらつき。残課題「たたみ切りの読み値のばらつき」）。
  - 参考（別の比較。W3.5・W4 の出発点を手で作り直した姿勢の旧座標の値と、`zero_shift` から計算した見込み）: 差は pan −0.0138、lift +0.0276、elbow +0.0077、wrist_flex +0.0062、roll +0.0062、gripper −0.0082 rad（C3 の合格の目安 0.03 rad 以内）。別の姿勢を手で作ったぶんの再現性。
- **URDF の関節リミットとの余裕**（`calib_C3_pose_margin.log`）: 出発点（新座標）で LOW/OUT は **elbow_flex だけ**: 残り −0.0001 rad（OUT。ユーザーの値で elbow +1.69009、上限 +1.69 を約 0.00009 rad 超え）。ほかの関節は ok（lift の下限まで +0.2624 が最小）。校正で記録した範囲の両端は、gripper を除く全関節で URDF の範囲より広い（pan ±2.0944 / ±1.9199、lift ±1.8220 / ±1.7453、elbow ±1.7192 / ±1.6900、wrist_flex ±1.8151 / ±1.6581、wrist_roll −2.9483..+2.9483 / −2.7439..+2.8412。サーボは URDF の外まで動ける）。
- **elbow の OUT の扱い: (a) に決めた（ユーザーの判断）**: 何も変えない。OMPL（既定）は `FixStartStateBounds`（`start_state_max_bounds_error: 0.1`）で扱え、mock の move_group で、この姿勢から計画に成功した（上の「出発点の elbow_flex が URDF の上限を 0.0001 rad 超える件」）。URDF・SRDF は変えない。Pilz は開始状態が URDF の範囲をわずかに超えても拒否する。
- **コミット**: 更新した `follower_calibration.yaml`、`calibration_records/2026-11-11/`（`measure_C1c_1.json`、`eeprom_backup_20261111-163729.json`、`eeprom_after_C3.json`、`README.md`。sha256 は原本と一致、README に記載）、この記録を、同じコミットに入れた（タグは付けない）。
- 次: 出発点の新座標への文書の書き換え（別コミット）、C4（書き込みモードでの W 系の確認のやり直し）。

## 計画（実装は未。承認の後）と残課題（2026-11-11）
### 計画 1（最優先。C4 の前）: 試験が本番の yaml の値に依存していて、C3 のコミット（59e49d6）の後に落ちる
- **事実**（C3 のコミットの後に `colcon test` を実行して確認）: `so_arm101_hardware` の 5 件（`Conversion.YamlMatchesLeRobotJsonSource`、`Conversion.MatchesLeRobotGoldenTable`、`Conversion.AngleIsNotClampedLikeLeRobot`、`Conversion.GripperTickInRad`、`InverseConversion.MatchesLeRobotUnnormalizeWithinOneTick`）と、`so_arm101_calibration` の 3 件（`Measure.MeasuresCenterAndRangeWithoutWritingAnything`、`Backup.CorruptOrInconsistentFilesAreRejected`、`Apply.WhenTheRollbackAlsoFailsTheStateTableShowsOldNewUnknownAndRestoreWorksAfterAPowerCycle`）が失敗する。`colcon build` は成功する。C3 の前（2c-3d のコミットの後）は、不安定な既存の 1 件を除いて全部通っていた。
- **原因**: これらの試験は `CALIBRATION_YAML`（`src/so_arm101_hardware/config/follower_calibration.yaml`。本番の yaml）を読み、**旧 yaml の値を固定で期待している**（例: `wrist_roll` の `center` 2064.5、Homing_Offset 1164、`source` に `2026-09-07` を含む。LeRobot の json から作った黄金表（`gen_conversion_golden.py`）との一致。偽サーボの EEPROM の初期値が yaml の値で、`measured_homing_offset` に 1164 を期待）。C3 で本番の yaml が新しい校正の値になったので、前提が崩れた。試験の不具合ではなく、**試験が本番の設定に結合していた**ことが原因。
- **計画（要承認。動作は変えない）**: 旧 yaml（`1e9e968` の内容。LeRobot の json から変換した値）を、**試験用の固定ファイル**として置く（例: `src/so_arm101_hardware/test/data/follower_calibration_lerobot_20260906.yaml`）。`so_arm101_hardware`・`so_arm101_calibration` の `CALIBRATION_YAML`（CMake の `target_compile_definitions`）を、本番の yaml からこの固定ファイルへ向ける。試験の中身（期待値）は変えない。本番の yaml の更新（今後の再校正）で試験が落ちなくなる。
  - 付け足す試験: 本番の yaml が読めて、構造が正しいこと（6 関節、`min < max`、`eeprom.*` が `min`/`max` と同じ、`center` が範囲の中にある。値は固定しない）。`apply` が書いた形式（`source` が `ros_calibrated`）を読み込めること。
  - 確認: `colcon build`・`colcon test`（`so_arm101_hardware` 84 件、`so_arm101_calibration` 62 件が、不安定な 1 件を除いて通る）。偽サーボでの確認は変わらない。
### 計画 2: 起動ログ 1 行目の出所の表示（`[ros_calibrated, file=?, file_date=?]`）を直す
- 事実: `src/so_arm101_hardware/src/calibration.cpp:54-59` が、`source` の `kind`・`file`・`file_date` だけを表示用の文字列 `c.source` に入れる（無いと `?`）。C3 の後の yaml の `source` は `kind`・`measure_file`・`backup_file`・`applied_at`・`applied_by` なので `?` になる。表示は `sts3215_system.cpp:109`。動作には無関係。
- 計画: 存在する項目だけを、決まった順で出す。旧形式は今と同じ（`[converted_from_lerobot_json, file=…, file_date=…]`）。新形式は `[ros_calibrated, measure_file=…, backup_file=…, applied_at=…, applied_by=…]`。`?` は出さない。`source` が無いときは今と同じ（空）。
- 試験: 旧形式の表示（固定ファイル。既存の `test_conversion.cpp:117` の期待を保つ）、新形式の表示、`apply` が書いた yaml を読んだときの表示（`so_arm101_calibration` の試験）、項目が欠けたとき。`colcon build`・`colcon test`。ドライバの動作（変換・診断・書き込み）は変えない。
- 時期: 計画 1 の後（同じ試験の固定ファイルを使うため）。C4 の前でも後でもよい。
### 計画 3: measure の「ほぼ届いた」の表示（前回の端まであと 10 tick 以内）
- 上の「計画（実装は C3 の後。今はツールを変えない）: 記録中の表示で『ほぼ届いた』を出す」のとおり。C3 は終わったので、実装してよい時期（ユーザーの承認の後）。表示だけの変更で、実機での再確認は不要（ユーザーの判断）。
### 残課題（C4 で確認する項目。出発点の姿勢の見直しは (a) で決着）
- **たたみ切りの読み値のばらつき**: 出発点（たたみ切った姿勢）の読み値は、毎回数 tick ばらつく（elbow_flex: 旧座標 +1.66938、新座標 +1.69009。`zero_shift` からの見込みとの差で、人が姿勢を作る再現性は数 tick）。上限（+1.69）を超える量は、今後も 0.01 rad 程度まで変わりうる。OMPL（0.1 まで許容）なら計画に影響しない。C4 で、W3.5 の前に複数回たたみ直して読み値のばらつきを見る（読み取り専用の起動。腕は動かさない）。
- **モデルの衝突と実機のたたみ切りの差**: モデルでは、出発点の姿勢のまま elbow_flex を閉じる向きに進めると、+1.70（有効）〜+1.74（upper_arm_link と wrist_link が接触）の間で衝突する（mock の `/check_state_validity`）。実機のたたみ切り（機械的な止まり）は +1.69 前後で、校正した範囲の端は +1.7192。**モデルの衝突が始まる値と、実機が止まる値の差**を、C4 で **RViz の重なり表示**（MotionPlanning の現在の状態。Scene Robot とロボットの衝突形状）と一緒に見る: 実機を出発点にしたとき、RViz で upper_arm_link と wrist_link に重なり・接触が見えるか、隙間がどれだけか。`/check_state_validity` に実測の姿勢を渡した結果（valid か）も記録する。
### 未確認として残すもの（C4 で確かめる）
- **実行側の `allowed_start_tolerance`（MoveIt の既定 0.01 rad。設定していない）に収まるか**: 軌道の最初の点（出発点 +1.69009）と、現在の実測の差。mock の計画では最初の点が出発点と同じだった（実行の確認はしていない）。
- **新座標の出発点から、OMPL で Plan & Execute できるか**（実機）。mock の move_group では、`FixStartStateBounds` の範囲内で計画に成功した（実行はしていない）。

## モックの MoveIt の判定のやり直し（新座標の出発点。2026-11-11。実機は使っていない）
- 目的: `feetech_write_check.md` の W3.5 の「モックの判定」を、C3 の後の出発点（pan −0.09820、lift −1.48295、elbow +1.69009、wrist_flex +0.50327、roll −0.04833、gripper +0.01523）でやり直す（承認済み）。
- 方法: `bringup.launch.py mock:=true use_rviz:=false`（mock_components。シリアルポートは開かない）。バックグラウンドで SIGINT が無視されて止められなかった前回の反省で、SIGINT を既定に戻してから `exec` する小さな Python ラッパーで起動し、`kill -INT` で止めた。`/check_state_validity`（衝突）、`/compute_fk`（`gripper_frame_link`、`base_link` 基準）、`/plan_kinematic_path`（OMPL、速度・加速度スケール 0.1。計画のみで実行はしない）を呼んだ。
- 結果（事実）:

  | 姿勢 | 衝突 | 先端 z [m] | 出発点との差 [m] | OMPL の計画（出発点から） |
  |---|---|---|---|---|
  | 出発点 elbow +1.69009 | なし（有効） | +0.0219 | 0 | - |
  | elbow +1.59009（−0.10） | なし | +0.0474 | +0.0255 | 成功（22 点） |
  | elbow +1.54009（−0.15） | なし | +0.0605 | +0.0386 | 成功（26 点） |
  | elbow +1.49009（−0.20） | なし | +0.0739 | +0.0520 | 成功（30 点） |
  | elbow +1.39009（−0.30） | なし | +0.1013 | +0.0794 | 成功（36 点） |
  - 旧校正の座標の出発点での判定（W3.5 の記録）は +0.026 / +0.039 / +0.053 / +0.081 m で、ほぼ同じ。出発点の高さは旧座標で +0.0304 m、新座標で +0.0219 m（約 0.009 m の違い。lift が +0.028 rad 違うため）。
  - 参考（閉じる向き。他の関節は出発点のまま）: elbow +1.69009〜+1.73 は有効、**+1.74 で upper_arm_link と wrist_link が接触（深さ 0.00001 m）**。接触の始まりは +1.73〜+1.74 の間（前回の調査では +1.70〜+1.74 の間までしか絞れていなかった）。実機のたたみ切り（+1.69 前後）とのあいだは約 0.04 rad。
- `feetech_write_check.md` の「モックの判定」の行を、この結果に直した。
- 後始末: 止めた後の `ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher" | grep -v -E "grep|gvfsd"` と `ros2 node list` は、どちらも空（`ros2 daemon` の再起動は不要だった）。

## 起動ログの出所の表示の修正（計画 2。2026-11-11）
- 変更: `src/so_arm101_hardware/src/calibration.cpp` の `source`（ログ用の説明）の組み立てだけ。**旧形式（`file` または `file_date` がある）は従来と同じ表示**（`converted_from_lerobot_json, file=…, file_date=…`。欠けた項目は `?`）。**新形式（`measure_file`・`backup_file`・`applied_at`・`applied_by` のどれかがある。`apply` が書く `ros_calibrated`）は、ある項目だけを決まった順で出す**（`?` は出さない）。`source` が無いときは空（従来どおり）。ドライバの動作（変換・診断・書き込み）は変えていない。
- 試験（`colcon build`、`colcon test`）: `so_arm101_hardware` に 3 件（`CalibrationSource.OldFormatIsShownAsBefore`、`NewFormatShowsTheFieldsThatApplyWrites`、`NoSourceStaysEmpty`。一時ファイルを書くので本番の yaml に依存しない）、`so_arm101_calibration` に 1 件（`Apply.TheRenderedYamlLoadsAndItsSourceIsShownInTheStartupLog`。`apply` が書いた yaml を読んだときの表示）。新しい 4 件はすべて通る。
- **試験の全体（事実）**: 失敗は、C3 の前から続く本番の yaml 依存の 8 件（`so_arm101_hardware` の `Conversion.*` 4 件と `InverseConversion.MatchesLeRobotUnnormalizeWithinOneTick`、`so_arm101_calibration` の 3 件）のまま。**この変更で増えた失敗は無い**（`so_arm101_hardware` 87 件、`so_arm101_calibration` 63 件）。8 件は、試験の固定ファイル化（承認待ち）で直す。
- 偽サーボ（pty）での結合確認（実機のポートは開かない）: `bringup.launch.py mock:=false hardware:=feetech serial_port:=<pty>`（読み取り専用）の起動ログの 1 行目が `initialized: port=/dev/pts/9 calibration=…/follower_calibration.yaml [ros_calibrated, measure_file=calibration_records/2026-11-11/measure_C1c_1.json, backup_file=calibration_records/2026-11-11/eeprom_backup_20261111-163729.json, applied_at=2026-11-11T19:43:04, applied_by=so_arm101_calib 0.2.0]` になり、`[diag] EEPROM vs yaml: 6/6 joints match`。後始末: `ps` は空、`ros2 node list` に `/move_group` などが残ったので `ros2 daemon stop; ros2 daemon start; sleep 3` の後に再確認して、どちらも空。（途中で `pkill -f` が自分のシェルにも一致して、コマンドが中断した。`pkill -f` に、実行中のコマンド自身の文字列を含めない。）
- 手順書: C3 の手順 4 の確認項目に、起動ログの 1 行目に出所が出る（`?` が出ない）ことを足した。

## measure の「ほぼ届いた」の表示（計画 3。2026-11-11）
- 変更（表示だけ。測定・検査・測定ファイル・`compare` は変えていない）: 記録中の表の「前回の端まで」で、あと 10 tick 以内（`kNearTicks` = 10。1 tick ≈ 0.0879° なので約 0.88°）を **`ほぼ あと N`** と表示する（`measure.hpp` の `kNearTicks`、`measure.cpp` の `format_gap`）。`あと N` は 11 tick 以上、`届いた` は 0、`届いた(+N)` は超えた量（今までどおり）。min 側・max 側で同じ表記。`*`（直近 1 秒に更新）は「ほぼ」の後ろに付く。端末の表と、終了時に標準出力へ出る最後の表（ログに残る）で同じ。
- 列幅: `ほぼ あと 10*` は表示幅 13 桁で、列（14 桁）に収まる。表の全体の幅は 72 桁以内のまま（試験で確認）。記録開始の案内の行（「端まで」の説明）に「ほぼ あと N は あと 10 tick（約 0.9°）以内」を足した。
- 試験（偽サーボ）: `format_gap` の境界（11 → `あと 11`、10 → `ほぼ あと 10`、1 → `ほぼ あと 1`、0 → `届いた`、−1 → `届いた(+1)`）、min 側・max 側の両方、`*` が付いても列に収まる・幅 72 桁以内、`measure` の実行（偽サーボ）で最後の表に `ほぼ あと 7`（min 側）と `あと 20`（max 側、`ほぼ` なし）が出て gripper が `届いた`、測定結果（`actual_min`/`actual_max`）は変わらない、の 2 件を追加。
- 確認: `colcon build`、`colcon test`（`so_arm101_calibration` 65 件）。失敗は、C3 の前から続く本番の yaml 依存の 8 件（試験の固定ファイル化で直す。承認待ち）のまま。**この変更で増えた失敗は無い**。不安定な `Measure.CommErrorsWhileRecordingAreTolerated` は、実行によって落ちる（既知。バス層の古い応答の残課題）。pty（`tools/pty_measure_check.py`）で、記録中の標準出力 0 バイト、最後の表・見出し 1 回、順序どおり、読み取り 303 回（失敗 0）、間隔 平均 20.4 ms／最大 20.7 ms、表示 30 回で合計 1.1 ms（2c-3d と同じ。表示を足しても読み取りは遅れない）。
- 手順書: C1c の「表示の見方」に `ほぼ あと N` の意味と表示例を追記し、「見方」を直した。実機での再確認は不要（ユーザーの判断。表示だけの変更）。

## 試験の固定ファイル化（本番の yaml への結合をやめる。2026-11-11。承認済み）
- 背景: C3 のコミット（59e49d6）の後、本番の `follower_calibration.yaml`（新校正）に旧校正の値を固定で期待していた 8 件が赤になっていた（その時点で報告していなかった。以後、コミットの前に `colcon test` を実行して失敗を報告する）。
- 変更:
  - 旧 yaml（`git show 1e9e968:src/so_arm101_hardware/config/follower_calibration.yaml` の内容。旧校正）を、試験専用の固定ファイル **`src/so_arm101_hardware/test/data/follower_calibration_lerobot_json_20260907.yaml`** として置いた。先頭に、出所（`git show 1e9e968` の内容、旧校正、試験専用で本番では使わない、変更しない）のコメントを 8 行書いた。本体は 1e9e968 の内容と同一（`diff` で確認）。
  - `CALIBRATION_YAML`（試験の定義。`so_arm101_hardware/CMakeLists.txt` の `test_conversion`・`test_sts3215_system`、`so_arm101_calibration/CMakeLists.txt` の `test_measure_apply`・`test_calibration`）を固定ファイルへ向けた。既存の試験の期待値・コードは変えていない（`YamlMatchesLeRobotJsonSource` の上に、固定ファイルを読む旨のコメントを足した）。
  - 本番の yaml の試験を新しく足した: `ProductionYaml.LoadsAndIsStructurallyConsistent`（`so_arm101_hardware`。6 関節、id 1〜6、`min < max`、`eeprom.min_limit/max_limit` が `min/max` と同じ、`|Homing_Offset| ≤ 2047`、angle の `center` が範囲の内側。値は固定しない）、`ProductionYaml.MatchesTheEepromReadAfterC3`（`so_arm101_calibration`。本番の yaml の `eeprom.*`（Homing_Offset・Min・Max）が、`calibration_records/2026-11-11/eeprom_after_C3.json` と一致する。**ファイルどうしを比べる**。Homing_Offset は符号つきの値を bit11 の符号付きの生の値に変えて比べる）。再校正で yaml を更新したときは、新しい読み出しを `calibration_records/` に置き、`CMakeLists.txt` の `EEPROM_AFTER_C3_JSON` の指す先を更新する（この試験の趣旨）。
- **リポジトリの外のファイルを読む試験は無い（確認）**: 試験が読むのは、`CALIBRATION_YAML`・`PRODUCTION_YAML`（リポジトリ内）、`GOLDEN_CSV`・`UNNORMALIZE_GOLDEN_CSV`（`test/data`）、`PACKAGE_XML`、`PACKAGE_SOURCE_DIR` のソース（`test_write_bus.cpp` のソース走査）、`JOINT_LIMITS_YAML`・`CONTROLLERS_YAML`（リポジトリ内の他パッケージ）、`::testing::TempDir()` の一時ファイルだけ。**`YamlMatchesLeRobotJsonSource` は、名前に反して LeRobot の json を読まない**（固定ファイルの値を、json の値として試験に書いてある）。`~/.cache/huggingface/lerobot/…`、`~/miniforge3` などを読む試験は無い（`grep` で確認）。LeRobot を使うのは、黄金表を作る `tools/gen_conversion_golden.py`（試験の外。`colcon test` では動かない）だけ。
- **試験の結果**（`colcon build` 成功、`colcon test --packages-select so_arm101_hardware so_arm101_calibration` を 3 回）: **`so_arm101_hardware` 88 件、`so_arm101_calibration` 66 件、3 回とも失敗 0**（8 件の赤が解消。`Measure.CommErrorsWhileRecordingAreTolerated` も、この 3 回では落ちなかった）。
- **不安定な `Measure.CommErrorsWhileRecordingAreTolerated` の頻度（測定。無効にしない）**: ビルド済みの試験バイナリの単独実行を連続で 40 回: **31 回失敗**。`test_measure_apply` 全体の実行を連続で 15 回: **3 回失敗**。`colcon test` 3 回: 0 回。実行の仕方で頻度が大きく違う（連続で起動し直すと、最初の試験として実行されるぶん、偽サーボのスレッドが冷えていて、残りの応答が捨てた後に届く競合が出やすい、と見ている。推測）。原因はバス層の競合（残課題「バス層の古い応答の取り違え」）。計画は `docs/bus_layer_fix_plan.md`。

## バス層の修正の計画書（2026-11-11。計画のみ。実装は承認の後）
- `docs/bus_layer_fix_plan.md` に、案 A（失敗したら静かになるまで待って捨てる）・B（SYNC_READ の残りの応答を読み切る）・C（EEPROM の読み戻しを 2 回一致）・D（取り違えの数を出す）、偽サーボの「応答を遅らせる」機能の設計、試験の計画、「誤って一致」の経路の検討、W0 の `checksum mismatch` との関係（この競合では説明できない）、`Measure.CommErrorsWhileRecordingAreTolerated` の見立て（A・B で安定する見込み。100 回連続で確認する）、実機での確認の手順（C4 と合わせる）、実装の順序を書いた。
- 実機の頻度（事実）: 書き込みモードの運転で約 22.2 万回の読み取り、`measure` の記録で 9999 回の SYNC_READ に、失敗 0。競合が実機で起きた記録は無い。
- 実装の時期: C3 の後、C4 の前（ユーザーの予定）。承認の後に、計画書の 8 章の順序で行う。

## バス層の修正 段階1: 偽サーボの「応答を遅らせる」機能（2026-11-11。承認済み。試験の道具だけ。ドライバは変えていない）
- 承認（ユーザー、2026-11-11）: 計画書 `docs/bus_layer_fix_plan.md` を 8 章の順序で実装する。条件: `drain_quiet_ms` などの新パラメータはコードの既定値で持つ（xacro・ros2_control のタグに書き足す必要が出たら、実装前に差分を示して承認を取る）／失敗時の待ちの周期への影響を偽サーボで測る／案 C は実機確認の手順を `so_arm101_calibration_check.md` に足し、実機への書き込みの前にユーザーの確認を取る／`Measure.CommErrorsWhileRecordingAreTolerated` は単独実行 100 回連続で判定／実機確認は C4 の手順として `feetech_write_check.md` に反映／段階ごとに別コミット。
- 変更: `src/so_arm101_hardware/test/fake_servo.hpp` に、応答のスケジューラ（応答をキューに入れ、due 時刻に `loop` が `write`。要求のパースは遅れない）と `set_reply_delay_us(id, us)`・`set_frame_spacing_us(us)`・`delay_next_replies(id, addr, count, us)`・`reply_log()`・`clear_reply_log()` を足した。遅れが 0 でキューが空のときは従来どおりその場で `write`（既存の試験の挙動は変わらない）。
- **計画書からの修正（事実）**: 1 本のバスなので、応答は追い越さない（先に予約された応答より前に出さない）ようにした。これが無いと、`frame_spacing`（80 µs）が要求の間隔（実測 約 60 µs）より長いとき、新しい要求の応答が古い応答を追い越し、競合が再現しなかった（試験で確認）。また、計画書の「id 2〜6 を 3 ms 遅らせる」は A の `quiet`（2 ms）より長く、A・B で捨てられない。競合の再現は **1.5 ms** にした（窓の中。窓の外は、限界の試験で別に固定する）。
- 試験（`test/test_bus_stale.cpp`、`so_arm101_hardware`）: `FakeServoDelay.*` 3 件（遅れの時刻とログ、`frame_spacing` と順序、`delay_next_replies` が回数だけ効く）。この段階のコミットには、これだけを含める。
- **今のコードで落ちることの確認（再現の記録。この段階ではコミットしない。段階2に含める）**: `BusStale.StaleRepliesAfterAFailedSyncReadAreDiscarded`（即時の再試行が `IdMismatch`）、`BusStale.SingleReadDoesNotAcceptAnOldFrameOfTheSameIdAfterAFailedSyncRead`（単独の READ が古い応答を受け取る）、`Measure.CommErrorsWhileRecordingAreToleratedWithDelayedReplies`（`no answer from id 1 (id 2 answered first)`）。5 回連続で、毎回同じ結果（決定的）。
- 確認: `colcon build`、`colcon test`（`so_arm101_hardware` の全件 通過、`so_arm101_calibration` は既知の不安定な `Measure.CommErrorsWhileRecordingAreTolerated` の 1 件だけ失敗。段階2で直す対象）。

## バス層の修正 段階2: 案 A・B（失敗の後に静かになるまで待って捨てる／SYNC_READ の残りを読み切る）（2026-11-11。偽サーボのみ。実機は使っていない）
- 変更:
  - `SerialPort::drain_until_quiet(quiet, max_total)`（`ppoll`。バイトが来るたびに窓を数え直し、`quiet` の間 1 バイトも来なければ終わる。全体は `max_total`＝10 ms で打ち切り）。
  - `Sts3215Bus`: `set_drain_quiet()`（既定 **2 ms**。**コードの既定値**。0 で従来どおり＝捨てるだけ＋残りを読み切る処理も止める）。`fail()` は、通信エラー・`IdMismatch` のとき `drain_until_quiet` で後始末する（それ以外は捨てるだけ）。`sync_read` は途中の失敗では後始末をせず、最後に 1 回: 案 B（受け取っていない ID の数だけ `discard_frame` で読み切る。各 2 ms、全体 10 ms）→ 案 A（全部読めたら窓を 0.5 ms に短縮、読めなければ 2 ms）。後ろの ID が先に返った場合は、その ID より後の数を読み切る。
  - **追加（周期への影響の測定で見つけた改善）**: タイムアウトの間に **1 バイトも来なかった**（バスが黙っている）ときは、待たずに捨てるだけにした。これが無いと、全サーボが黙るとき 1 回の失敗が 10 → 14 ms に延びた（2 回の再試行で 20 → 28 ms）。
  - ドライバ（`Sts3215System`）: ハードウェアパラメータ `drain_quiet_ms`（**コードの既定値 2**。範囲 0〜10。範囲外は起動拒否）。**xacro・ros2_control のタグは変えていない**（書き足していない。書かなくても動く。書けば `drain_quiet_ms:=0` で比べられる）。`retried_total()`・`failed_cycles_total()` のアクセサを足した（試験・測定用）。
- 試験の結果（偽サーボ）:
  - 段階1で再現した 3 件（`BusStale.StaleRepliesAfterAFailedSyncReadAreDiscarded`、`BusStale.SingleReadDoesNotAcceptAnOldFrameOf…`、`Measure.CommErrorsWhileRecordingAreToleratedWithDelayedReplies`）が通る。
  - 足した試験: `DrainQuietZeroKeepsTheOldBehaviourAndTheRaceStillShowsUp`（0 で競合が再現する）、`FailureInTheMiddleOfASyncReadAlsoDiscardsTheLaterIds`、`ASilentIdInTheMiddleDoesNotPoisonTheNextRead`、`HealthyReadsAreNotSlowedByTheDrain`（正常な 300 回の SYNC_READ: 0 の場合 16.3 ms、2 ms の場合 15.2 ms。差なし）、`ImmediateRetryAfterTimeoutWithLateRepliesIsAKnownLimit`（**限界の固定**: タイムアウトの後 約 14 ms に着く応答は窓の外で、単独の READ が古い値を受け取る。修正の前後で同じ結果）。
  - **`Measure.CommErrorsWhileRecordingAreTolerated` の安定の判定**: ビルド済みバイナリの単独実行を**連続 100 回: 失敗 0 回**（前回は 40 回中 31 回失敗）。決定的な版（`…WithDelayedReplies`）も 100 回: 0 回。`test_bus_stale` も 100 回: 0 回。試験は無効にしていない。
- **失敗時の待ちが周期（20 ms）に与える影響（偽サーボで測定。`BusFixCycleImpact`。毎サイクルに故障を注入、100 サイクル、`timeout_ms=10`。read+write の所要時間、ms）**:

  | 故障 | `drain_quiet_ms=0`（従来） | `drain_quiet_ms=2`（新） |
  |---|---|---|
  | なし | 平均 0.12 / 最大 0.23 / 20 ms 超 0 | 平均 0.12 / 最大 0.26 / 0 |
  | 競合（id 1 が壊れ、他が失敗の後に届く） | 平均 2.2 / 最大 3.6 / 超 0、**失敗サイクル 100（毎回、取り違えで再試行も失敗）** | 平均 5.7 / 最大 8.6 / 超 0、**失敗サイクル 0** |
  | id 1 が 1 回だけ無応答 | 平均 0.19 / 最大 0.42 / 超 0 | 平均 0.75 / 最大 0.94 / 超 0 |
  | id 1 が 2 回無応答（他は届く） | 平均 0.13 / 超 0、失敗 100 | 平均 1.3 / 最大 4.5 / 超 0、失敗 100（同じ） |
  | 全サーボが黙る（奇数サイクル）＝本物のタイムアウト 2 回 | 平均 10.2 / 最大 20.3 / **20 ms 超 50（失敗サイクル 50）** | 平均 10.2 / 最大 20.3 / **超 50（同じ）** |

  - 読み方（事実）: **周期に間に合わなくなる回数は、新旧で同じ**（全サーボが黙る本物のタイムアウト 2 回は、従来から 20 ms を超える。待ちを足していない）。競合では、従来は毎回失敗していたのが、新は毎回成功する代わりに 5〜9 ms かかる（20 ms の中）。失敗していないサイクルの `update` は影響なし。
  - 注意（**[推測]**）: 偽サーボは遅れの時間・間隔を模したもので、実機の USB の遅れ（数 ms の可能性）は未測定。実機の確認（7 章）で、`bus:` の行と `drain_quiet_ms:=0` との比較で見る。
- 確認: `colcon build`、`colcon test --packages-select so_arm101_hardware so_arm101_calibration`: **`so_arm101_hardware` 94 件、`so_arm101_calibration` 65 件（gtest の xml の合計）、失敗 0**。`test_measure_apply` 全体の単独実行 15 回連続: 失敗 0（前回は 15 回中 3 回）。

## バス層の修正 段階3: 案 D（診断カウンタとログの行）（2026-11-11。偽サーボのみ）
- 変更: `BusCounters`（`flushed_requests`・`flushed_bytes`・`drained_bytes`・`id_mismatch`・`answered_first`・`timeouts`・`bad_checksum`）を `Sts3215Bus` に持たせ、`counters()` で参照、`to_string()` で `bus: flushed_requests=… …` の 1 行にした。数え方: `id_mismatch` は呼び出し側に `IdMismatch` を返した数（SYNC_READ で後ろの ID が先に返って `Timeout` に読み替えたものは `answered_first` に数え、`id_mismatch` からは引く）。`drained_bytes` は失敗の後始末（残りの応答の読み捨て・静かになるまで待って捨てる・`drain_quiet_ms=0` のときの捨てるだけ）で捨てた合計。
- 出力: ドライバの `deactivated.` の行（書き込みモード・読み取り専用の両方）の末尾、`measure` の `記録:` の次の行、`apply`/`restore` の**失敗したとき（終了コードが 0 以外）だけ**の終わりの行。書式は 1 行で同じ。
- 試験（偽サーボ）: 健全なバスでは全部 0（50 回の SYNC_READ）、競合の後の `drained_bytes` が残りの応答のバイト数と一致（5 フレーム × 8 = 40）、A を止める（`drain_quiet=0`）と競合が `id_mismatch`/`answered_first` に出る、後ろの ID が先に返ると `answered_first=1`（`id_mismatch=0`）、タイムアウトの後に遅れて着いた応答が次の要求の前の `flushed_requests`・`flushed_bytes`（8 バイト）に出る、`measure` の `記録:` の後に `bus:` の行が出て健全なら `drained_bytes=0 id_mismatch=0 answered_first=0`、`apply` は成功のとき出ず失敗のときだけ出る。偽サーボでのドライバの `deactivated.` の行の実物: `... startup_read_retries=0 bus: flushed_requests=0 flushed_bytes=0 drained_bytes=0 id_mismatch=0 answered_first=0 timeouts=0 bad_checksum=0`。
- 確認: `colcon test`: `so_arm101_hardware` 99 件、`so_arm101_calibration` 67 件、失敗 0。
- 手順書: `docs/feetech_write_check.md` の「ログの読み方」に `bus:` の行（健全なら 0 の項目、0 でないときの報告、`flushed_before_request` による切り分け）を追記。`docs/so_arm101_calibration_check.md` の `記録:` の説明に `bus:` の行を追記。C4 の手順書の全体は、段階5の前（案 C の後）に直す。

## バス層の修正 段階4: 案 C（EEPROM の書き込みの読み戻しを、独立に 2 回とも期待値のときだけ一致）（2026-11-11。偽サーボのみ。**実機の書き込みはしていない**）
- 変更（`so_arm101_calib` の書き込みの動作が変わる）:
  - `write_calib_verified`（`apply`/`restore` が使う）: 待った後の読み戻しを独立に 2 回行い、2 回とも期待値のときだけ一致。1 回目が違えば従来どおり `read-back mismatch`、1 回目が一致で 2 回目が違えば `read-back mismatch … the first read was … but the second read was … (the two reads must agree; not retried)`。どちらも再試行せず、呼び出し側が従来どおり戻す（`apply`）／Lock=1 に戻す（`restore`）。通信エラーは従来どおり 1 回の読み取りごとに最大 `retries` 回再試行。
  - 最終の状態表（`apply` の「書き込み後の状態」、`restore` の「書き込み後の状態」）は `read_joint_regs_confirmed`（独立に 2 回読み、2 回とも読めて同じ値のレジスタだけを表示。違う・読めないものは `(read failed)` 側）。`backup` は従来から 2 回読んでいる（変更なし）。
  - ドライバ側（`verify_reg`）は変えていない（計画書 5 章の結論どおり。A・B で足りる）。書き込みの順序・回数・待ち時間・`apply`/`restore` の手順は変えていない。読み取りが、書き込み 1 項目につき 1 回増える。
- 偽サーボの道具: `override_next_reads(id, addr, count, value)`（次の count 回の READ に実際の値でなく value を返す。古い／化けた応答が「正しいチェックサムの応答」で届く状況の再現）。
- 試験: `WriteVerified.ReadBackNeverMatchesOnAStaleReply`（書き込みは反映されず実際の値は旧のまま、1 回目の読み戻しだけ新の値が届く → **失敗になる**。C が無ければ「一致」になっていた）、`TwoAgreeingReadBacksPassAndCostExactlyTwoReads`（正常なら成功し、読み取りは正確に 2 回）、`SameValueRewriteStillPassesWithTwoReads`（`--rewrite-same` の形）、`TheFinalStateTableShowsOnlyValuesThatTwoReadsAgreeOn`。既存の `apply`/`restore` の試験（失敗時の巻き戻し、通信エラーの再試行など）は変更なしで通る。
- 実機の確認手順: `docs/so_arm101_calibration_check.md` に **C0.6** を追加（gripper への `--rewrite-same`、C0.5 と同じ形。待ち 50 ms と 0 ms、dry-run → 書き込み → 状態確認。合格基準と、失敗時に報告するもの）。**新しいコードで実機に書き込む前に、ユーザーの確認を取る（Claude は実行しない。まだ実施していない）**。
- 確認: `colcon test`: `so_arm101_hardware` 99 件、`so_arm101_calibration` 71 件、失敗 0。

## バス層の修正 段階5の準備: C4 の手順書（2026-11-11。C4 はまだ実施していない）
- `docs/feetech_write_check.md` の先頭に「C4 の全体の順序」の表と、「2. 読み取り専用 10 分」「4. W0 を 5 回」の手順を追加した（順: C0.6 → 読み取り専用 10 分 → 読み値のばらつき（3 回）→ W0 ×5 → W1〜W4 → RViz の重なり・問題2・`allowed_start_tolerance` → 任意の `measure` → 問題が出たときの `drain_quiet_ms:=0` の比較）。各ステップで報告してほしいものを表にした。合格の基準は計画書 7 章どおり（`retried=0`、`failed_cycles=0`、`bus:` の `id_mismatch`・`answered_first`・`drained_bytes` が 0、`reads` の数が従来と同じ）。
- **未承認の変更（実施していない）**: `drain_quiet_ms:=0` を起動引数で渡すには、`bringup.launch.py`（引数の宣言と `mappings`）、`so101_new_calib.urdf.xacro`（`xacro:arg` と呼び出し）、`so101_new_calib.ros2_control.xacro`（マクロの引数と `<param name="drain_quiet_ms">`）の 3 か所の差分が要る。条件（xacro・ros2_control のタグに書き足す必要があるなら、実装前に差分を示して承認を取る）により、差分を示して承認を待つ。承認までは、ドライバはコードの既定値 2 ms で動き、C4 の順 8 は実施できない。

## C4 手順書の修正（承認済みの指示による。コードは変更なし）
- `drain_quiet_ms` の起動引数（`bringup.launch.py`・xacro 2 つ）は**保留**。C4 の順 8 は「問題が出たときに承認を取ってから」のまま。
- `so_arm101_calibration_check.md` C0.6: 手順 0（`backup --out eeprom_backup_before_C0.6.json` と、`eeprom_after_C3.json` との全関節・全バイトの比較コマンド）、dry-run で gripper が 547/1454/2944 であることの確認、戻す方法（`restore --backup <手順 0 のファイル> --write`）と終了コード 0〜4 ごとの対応を追加。以降の手順は手順 0 のファイルを使う。
- `feetech_write_check.md`: 合格の基準を不合格／要観察／合格の 3 つに分けた。順 6 に「範囲をわずかに超えた出発点（elbow +1.69009）からの OMPL の計画」の具体的手順（書き込みモードで起動、send_joint_delta の前に elbow_flex だけ −5° で Plan→Execute、報告項目、Pilz は使わない）を追加。W4 は 10 分に短縮。順 1 の説明を更新。

## C4 手順書の修正 その 2（docs のみ）
- 順 6: Execute の条件を「elbow_flex 以外の関節は 0.5° 以内の変化なら許容、1° 超なら Execute しない」に変更。`<current>` で elbow_flex が 97°（+1.6930 rad、上限 +1.69 超え）に丸められる可能性と、その扱い（表示は未確認として報告、目標は約 92°、無効表示が消えなければ Plan/Execute しない）を注意として追加。
- 確認: `torque_limit` は bringup.launch.py に宣言済み（既定 1000）。W1/W2/W4 の起動と同じ `torque_limit:=500`。
- 確認: eeprom_after_C3.json のキーと `backup` の出力形式（eeprom_backup.cpp の読み書き）が一致。比較スクリプトを、同じファイル 2 つ（コピー、読むだけ）で試して MATCH を確認。

## 段階2c 実機確認: C0.6・C4（順 1〜6）合格（2026-11-12。ユーザーの実機の報告。ログは `~/so_arm101_check/`）
結果はユーザーの報告のとおり。数値のうち、`reads` 2 つ・W0 の `startup_read_retries`・W4 の終了時の出力はログと突き合わせた（下記）。それ以外はユーザー報告の転記。

### C0.6（案 C: EEPROM の書き込みの読み戻しを 2 回一致）
- 待ち 50 ms・0 ms とも **exit=0、`bus:` なし**。
- 0 ms は同じ値の書き直しなので、**完了待ちが要るかどうかの根拠にはならない（既定の 50 ms のまま）**。`time` は手入力を含むので、比較は参考。

### 順 2（読み取り専用 10 分）
- 約 744 s、`reads=37216`（約 50 Hz）、全カウンタ 0（`c4_readonly_10min.log` で確認: `retried=0 failed_cycles=0`、`bus:` の全項目 0）。

### 順 3（たたみ方による elbow の違い）
- 「**支えて戻す**」たたみ方: elbow +1.6840／+1.6855（±1 tick）。「**重力に任せる**」: +1.7208（クランプ ±1.7192 を約 1 tick 超える）。
- **書き込みモードの前の出発点は「支えて戻す」で作る**（手順書に追記）。

### 順 4（W0 × 5）
- 5 回とも `startup_read_retries=0`、全カウンタ 0（ログで確認）。**checksum mismatch は再現せず（原因は未特定のまま）**。停止後は torque_enable=0、lock=1、acceleration=50。

### W1
- 約 74 分、`reads=225734`、失敗 0（`write_W1.log` で確認: `retried=0 failed_cycles=0 rate_limited=0`、`bus:` 全項目 0、`goal_writes=225734`）。
- **gripper の閉じる向き（-0.294）は status 6（ABORTED）**、目標の 0.0105 手前で止まる（閉じ切りのぎりぎり、過剰トルクなし）。**ユーザーの判断: 対策不要。境界付近の数度の差は実運用上気にしない。**

### W2・W3
- 合格。pan の + は時計回り（再確認）。
- **W3.5 で elbow に -0.05 を足しても 2 tick しか動かなかった**（手前止まりが約 0.04 rad あるため。手順書の「効く量は小さい」の裏付け）。

### 順 6（範囲外の開始状態からの Plan & Execute）
- elbow +1.70697（**URDF 上限を 0.017 rad 超え**）から、OMPL で elbow 93° の Plan & Execute に**成功**。move_group に `Starting state is just outside bounds (joint 'elbow_flex'). Assuming within bounds.` が出て、`allowed_start_tolerance` 0.01 の検証も通った（`Validating trajectory with allowed_start_tolerance 0.01`）。
- `<current>` の丸めは **elbow 98°**（手順書の見積りの 97° ではない）。スライダーは端に張り付いて見える。他の関節の読み値は 1 tick も変わらず。
- **問題 2（たたみ姿勢からの Plan & Execute が衝突で中止）は新座標では再現しなかった**。RViz で upper_arm と wrist の重なりなし（gripper のすき間は RViz の方が少し広い）。
- 「未確認として残すもの」（上の「C4 で確かめる」の 2 項目と、モデルの衝突と実機のたたみ切りの差）のうち、実行側の `allowed_start_tolerance` と、実機での OMPL の Plan & Execute は**確認済み**（元の記述は残す）。

### W4（MoveIt の実行、10 分。ユーザーの判断で 10 分）
- MoveIt で他の関節を動かすたびに、**elbow が自重の向きに約 6 tick（0.5°）ずつ入り込む**（原因は未特定。実測の事実のみ）。
- 終了時、`rviz2` と `move_group` が exit code -11（**deactivate の後**。`write_C4_step6.log` の 349-350 行で `[deactivate] torque OFF and Lock=1 confirmed` と `deactivated.` が先、477-478 行で died）。`Maybe failed to update robot state` が 3 回（time diff 0.200 s・0.031 s・0.071 s）。手順書の「ログの読み方」に、終了時の exit -11 は無視してよい旨を追記。

### コードを読んで確かめたこと（実機・モックは使っていない）
**1. FixStartStateBounds（MoveIt2 Humble）**
- 読んだ source: moveit/moveit2 の humble ブランチ、コミット c283a36186a6f7a5985360e6674bf8fd0790e485（2026-09-01、"2.5.10"。タグ 2.5.10 も同じコミット）。インストール済みは moveit-core / ros-planning 等が 2.5.10-1jammy（dpkg -l）。
- 開始状態が範囲外でも `start_state_max_bounds_error` 以内なら、`enforceBounds` で範囲内に直して計画し、**直す前の実際の開始状態を軌道の先頭（時刻 0）に付け足す**（`addPrefixWayPoint`）。許容を超えると付け足さずに WARN。
- パラメータ名は `start_state_max_bounds_error`、OMPL パイプラインで **0.1** が効いている（**実機のログで確認**: `Param 'ompl.start_state_max_bounds_error' was set to 0.100000`、chomp も同じ。`write_C4_step6.log`）。
- `allowed_start_tolerance`（既定 0.01）の検証は `TrajectoryExecutionManager::validate`（同じ 2.5.10 の source）: **軌道の最初の点の位置と現在の関節状態**を比べ、差が許容を超えると `Invalid Trajectory: start point deviates ...` で拒否する。先頭の点が付け足された実際の開始状態なので、通った。
- **確認できなかった範囲**: apt のビルドが公式タグと同一かは未確認（版番号は同じ）。
**2. 書き込みモードの起動時、現在位置がクランプ（elbow ±1.7192）の外のとき**（`sts3215_system.cpp`）
- **起動を拒否しない**。起動の手順 5 で、クランプせずに**生の値を Goal にする**。
- 次の周期から、指令を `[raw_min, raw_max]` にクランプする。**50 tick（`kClampToleranceTicks`）以内の範囲外は黙ってクランプ**し、Goal は校正範囲の端へ引き込まれる。
- **50 tick を超えて範囲外なら、異常と数え、連続 50 周期（約 1 秒）でフォルトになり、Goal の送信が止まる**（トルクは保持）。
- **「起動時に現在位置が範囲外」を直接見る試験はない**。近いのは `CommandsBeyondTheCalibratedRangeAreClamped`（指令が +10 tick 超え）と `StartupWriteOrder`・`HoldsCurrentPositionAndSendsGoalEveryCycle`。
- **試験の候補（未実施。今回コード・試験は変えていない）**: 偽サーボの現在位置を `raw_max` + 1 tick（引き込まれてフォルトにならない）と +51 tick 以上（約 1 秒でフォルトになる）にして書き込みモードで起動し、Goal が端に入ること・フォルトの有無を見る。
**3. GripperActionController（status 6 の理由）**
- `controllers.yaml`（`so_arm101_controller`。bringup が読む）と `ros2_controllers.yaml`（`so_arm101_moveit_config`）の `gripper_controller` は `joint: gripper` だけで、goal_tolerance・stall 関係は**未設定（既定値）**: goal_tolerance 0.01、allow_stalling false、stall_velocity_threshold 0.001、stall_timeout 1.0（ros2_controllers 2.54.1 のヘッダ）。
- 動作: 誤差 < 0.01 rad なら成功。そうでなく、速度が 0.001 以下の状態が 1.0 秒を超えたら stalled で、allow_stalling=false なので **ABORTED（status 6）**。目標 0.0105 手前で止まる（誤差が 0.01 以上）ことと整合する（ログで目標との誤差の時刻までは見ていない）。

### 要望（未実施。記録のみ）
- `send_joint_delta.py` は送る前に Enter を待つ（[send_joint_delta.py:62](tools/send_joint_delta.py#L62)）。**貼り付けた別のコマンドの行が Enter の代わりになって、指令が送られたことがあった**。確認を「**空の Enter のみ**」または「**y**」にする案。実施するかは未定（コードは変えていない）。

## ユーザーの判断の記録（2026-11-12。記録のみ。コード・launch・xacro・URDF・yaml は変更していない）
- **push はしない**。タグ `stage2c-done`（1a3628e）とコミットはローカルのみ。
- **D2: EEPROM の Min/Max_Position_Limit を URDF の範囲に絞らない**（校正で測った物理の範囲のまま）。理由: サーボが安価で非力なこと、可動範囲の外で動けなくなる（URDF の範囲に絞ると、手で動かしたときや重力で入り込んだときに、範囲外で動けなくなる）ほうが困ること。**問題が多発したら将来考える**。
- **将来の案（未実施。承認も実施の予定もない）: キャリブ値から 20〜50 tick 内側に余裕を取る**。注意点（コード読みと計算による。実機では未確認）:
  - **elbow のたたみ切り（+1.684〜+1.721 rad。raw で約 3066〜3091。校正範囲の max は 3090）が、内側に取った範囲の外に出る**。余裕 20 tick なら max' = 3070 で、たたみ切りは最大約 21 tick 外。余裕 50 tick なら max' = 3040 で、最大約 51 tick 外になる。
  - ドライバは、**50 tick（`kClampToleranceTicks`）以内の範囲外は黙ってクランプ、それを超えると異常と数え、連続 50 周期（約 1 秒）でフォルトになる**（上の「コードを読んで確かめたこと」の 2）。起動時は生の値を Goal にして、次の周期で範囲の端へ引き込まれる。余裕を 50 tick 近くにすると、たたみ切りの姿勢で起動したとき、この判定や引き込みに掛かりうる。
  - **EEPROM の Min/Max_Position_Limit と yaml の `min`/`max`・`eeprom.min_limit`/`max_limit` の一致を、起動時に検査している**（書き込みモードでは不一致なら起動を拒否。試験 `RefusesWhenMinOrMaxLimitDiffersFromYaml`）。yaml の min/max だけを内側にすると、この検査と変換（gripper は min/max で正規化）に影響する。
  - 入れるなら、**EEPROM は測った値のまま残し、ドライバ側で関節・向きごとの余裕を持たせる**案がある（余裕は yaml の別項目にする、など）。その場合、yaml の書式（schema_version）と起動時の検査・試験の変更が要るので、承認制で別途計画する。
- **LeRobot 経路は、このプロジェクトの ROS パッケージから将来すべて削除する**。洗い出しと計画は `docs/lerobot_removal_plan.md`（実施は未。承認の後）。

### LeRobot 経路の削除の方針（ユーザー判断 2026-11-12。計画は `docs/lerobot_removal_plan.md` 第 2 版。実施は未）
- **`archive/` は作らず、`git rm` で完全に削除する**（so_arm101_lerobot_bridge、`tools/gen_conversion_golden.py`・`lerobot_readonly_state.py`・`convert_lerobot_calibration.py`、nimiCurtis 由来のコード、`Gili.json`・`Tzili.json`、`bringup_with_cameras.launch.py` を含む）。**戻すときは、タグ `stage2c-done`（1a3628e）から取り出せる**（例: `git checkout stage2c-done -- src/so_arm101_lerobot_bridge`）。CLAUDE.md の「archive へ移動」の規則との違いは、計画書の注意に書いた（CLAUDE.md は未変更）。
- 方針: LeRobot の完全排除と、必要最小限のパッケージにすることを優先する。問題が出たらその都度対策する。
- 決定: `[diag]` の基準値はコードに残して文言を reference に（A）、黄金表は名前を中立にして残す（A）、`hardware` 引数は `feetech` だけ受け付けて他は明示的なエラー、docs の LeRobot の手順は「歴史的」と注記、`setup_so101.sh` の `LECONDA_SITE_PACKAGES` は削除（`CONDA_PREFIX` の確認は手順書に残す）。`LICENSE.so101_ros2` は、MIT の帰属が必要なコードが残らないことを確認してから削除（現状は `camera.launch.py` が残る。計画書 7 章）。

## 今後の予定: URDF まわりの作り直し（ユーザーの方針 2026-11-12。**今は実施しない**）
- LeRobot の削除（`docs/lerobot_removal_plan.md` の L1〜L4）と、候補の削除の後の、**別の段階**。
- 方針: SO-ARM101 のオリジナルの URDF（TheRobotStudio/SO-ARM100 の `Simulation/SO101` にある `so101_new_calib.urdf` と思われる。未確認）から正しく引用する形に作り直す。案は「オリジナルを変更せずに置き、薄い xacro で include して、ros2_control と追加のフレームだけ足す」。取得元の URL・コミットとライセンスを記録する。
- 進め方: 調査（今の xacro とオリジナルの差の表: 0 の位置・軸の向き・関節リミット・リンク名・メッシュのパス、SRDF・controllers.yaml などへの影響）→ 計画 → 承認 → 実施 → 実機での確認のやり直し。詳細は `docs/lerobot_removal_plan.md` の 10 章。


## L1 実施: LeRobot の起動経路を外す（2026-11-12。コード変更は launch・xacro・package.xml のみ。bridge パッケージ自体は L2 まで残す）
- 変更: `bringup.launch.py`（`bridge_launch` と bridge の include を削除。`hardware` は `feetech` だけで、`OpaqueFunction` の検査 `_require_feetech` が `mock` の値に関わらず、ノードの起動の前に、`hardware:=feetech` 以外を `RuntimeError`（「LeRobot 経路は削除した。タグ stage2c-done にある」）で止める。docstring・引数の説明）、`bringup_with_cameras.launch.py`（`git rm`）、`so_arm101_bringup/package.xml`（`exec_depend so_arm101_lerobot_bridge` を削除）、moveit_config の `so101_new_calib.urdf.xacro`（`hardware` の既定 `feetech`）と `so101_new_calib.ros2_control.xacro`（マクロの既定 `feetech`、`So101HardwareBridge` の分岐を削除、`hardware != 'feetech'` のとき `xacro.fatal` で失敗）、description の `so101_new_calib.ros2_control.xacro`（`mode == 'real'` の `So101HardwareBridge` のブロックを削除）。C++・CMake・URDF 本体・関節リミット・yaml は変更なし。
- **robot_description の差（xacro で生成して diff。`xacro src/so_arm101_moveit_config/config/so101_new_calib.urdf.xacro` に、launch と同じ引数）**: mock、feetech（read_only true／false）、mock で `hardware` 未指定、description 側の `mode:=isaac`・`gazebo` は、**変更の前後で完全に同一**。`hardware:=lerobot` は、前は生成できた（`So101HardwareBridge` を含む）が、**後は xacro が rc=2 で失敗**（`error: hardware:=lerobot is not supported: ...`。`use_fake_hardware:=true` のときも失敗）。description 側の `mode:=real`（既定）は、最初は `So101HardwareBridge` の `<plugin>` の 1 行だけが消え、プラグインの無い `<hardware>` を出す状態になった。**そこで追加で修正**: description の `so101_new_calib.ros2_control.xacro` で、`mode == 'real'` のときは `<ros2_control>` のブロック自体を出さない（`xacro:unless`。isaac・gazebo は変更なし）。**この xacro を実行時に読むものは無いことを grep で確認した**: `so101_new_calib.urdf.xacro`（description 側）・`ros2_control.xacro`・`with_transmission.xacro` は、description 内の相互の include だけで、launch・試験・tools・CMake からの参照は無い（`so_arm101_moveit_config` の xacro は description の plain URDF `so101_new_calib.urdf` を include しており、description の xacro は読まない。`so_arm101_bringup`・`so_arm101_controller`・`so_arm101_moveit_config` の package.xml は `so_arm101_description` に依存するだけ）。docs の記録（`inventory.md` の段階 1）でも、description の launch は archive に移っていて未使用。以前に読んでいた `archive/stage2a/` の `rsp.launch.py` などは archive 内。生成し直して before と diff: `mode:=real` は **`<ros2_control>` ブロック（43 行ほど）が消えただけ**（1 つの hunk）、`isaac`・`gazebo` は変更前と**同一**。
- 試験の件数（前 → 後）: so_arm101_hardware gtest **99 → 99**、colcon test-result **105 → 105**。so_arm101_calibration gtest **71 → 71**、**73 → 73**。失敗 0。moveit_config・description は 0 件。**so_arm101_bringup**（lint だけ。`ament_lint_auto`）: **20 件・失敗 14 → 18 件・失敗 12**（flake8 5/5 → 4/4、copyright 3/2 → 2/1、pep257 5/4 のまま、CTest 5/3 のまま）。減った分は `bringup_with_cameras.launch.py` の削除による（失敗の消滅であって解消ではない）。残りの失敗は変更前からあるもの（`bringup.launch.py` の E501 4 件など）。今回の変更で増やした E501（3 件）は整形して 0 にした。
- 注意: `install/` は symlink-install で、`install/so_arm101_bringup/share/.../launch/bringup_with_cameras.launch.py` は、削除したソースを指す**宙ぶらりんのリンクとして残っている**（以前から `so101_robot*.launch.py` も同じ状態）。害はなく、L2 のクリーンビルドで消える。
- **モックの起動確認（ユーザーが「実機を使っていない」と返答した後。自分で起動したプロセスだけを PID で止めた。`mock:=false` は起動していない）**:
  - `mock:=true use_rviz:=false`: 起動前に `ps`・`ros2 node list` とも既存のノードなし。起動 20 秒後、`ros2 control list_controllers` で **arm_controller（joint_trajectory_controller）・gripper_controller（GripperActionController）・joint_state_broadcaster が active**。ノードは controller_manager・move_group・robot_state_publisher など。`kill -INT`（launch の PID だけ）で停止。停止後、`ps` は空。`ros2 node list` に `/move_group`・`/moveit_simple_controller_manager` が残った（`ps` は空）ので、`ros2 daemon stop; ros2 daemon start` の後に再確認して、**両方とも空**。停止時に `move_group` が exit code -11（Segmentation fault。`rclcpp::CallbackGroup::~CallbackGroup()`。実機の W4 の終了時にも出ていたものと同じ形で、今回の変更とは無関係と見られる）。
  - `mock:=true hardware:=lerobot use_rviz:=false`: launch が **exit code 1 で自分で終了**。`[ERROR] [launch]: Caught exception in launch ...: hardware:=lerobot is not supported. The LeRobot path ... was removed from this workspace; use hardware:=feetech (the default). The removed code is in git tag stage2c-done ...`。ノードは 1 つも起動せず、`ps`・`ros2 node list` とも空。
- **未実施**: 実機確認（読み取り専用の起動と W0）。コミットの後にユーザーが行う。

## L1 の後の実機確認: 合格（2026-11-12。ユーザーの実機の報告。ログは `~/so_arm101_check/` の `L1_readonly.log`・`write_L1_W0.log`・`write_L1_W0_after.log`）
- **読み取り専用の起動**: 出所 `ros_calibrated`（`applied_by=so_arm101_calib 0.2.0`）、EEPROM vs yaml **6/6 match**、`servo settings vs LeRobot` **6/6 match**（L3 まで文言は旧いまま）、SRAM は全関節 `torque_enable=0 lock=1 acceleration=50`。停止時 `reads=10331 retried=0 failed_cycles=0`、bus はすべて 0、exit=0。
- **W0（`probe_lock_sram:=true`）**: 腕は動かず保持。`[probe] ACCEPTED on all joints`、`startup_read_retries=0`、`[deactivate] torque OFF and Lock=1 confirmed`、`deactivated. reads=2681 goal_writes=2680 monitors=53`（保持していた時間が約 54 秒だったため）、`rate_limited=0 port_contention=0`、bus はすべて 0、exit=0。
- **W0 の後の読み取り専用**: 全関節 `torque_enable=0 lock=1 acceleration=50`、6/6 match が 2 行、`reads=532`、カウンタ 0。
- `move_group` の exit -11 は、いつもの終了時のもの（無視してよい。W4 でも出ていた）。
- 結論: L1（launch・xacro の変更）は、実機の挙動に影響していない。

## 残課題: SRDF の `Link 'elbow_flex' is not known to URDF`（調査のみ。**SRDF は変更していない**。MoveIt の設定の変更なので、承認の後。2026-11-12）
起動時に `move_group` が `Warning: Link 'elbow_flex' is not known to URDF. Cannot disable/enable collisons.`（`model.cpp:594`）を出す。
1. **どこで使っているか**: `src/so_arm101_moveit_config/config/so101_new_calib.srdf:45` の `<disable_collisions link1="gripper_link" link2="elbow_flex" reason="Never"/>` **の 1 行だけ**。`elbow_flex` は URDF の**関節の名前**であって、リンクではない（リンクは `base_link`・`shoulder_link`・`upper_arm_link`・`lower_arm_link`・`wrist_link`・`gripper_link`・`gripper_frame_link`・`moving_jaw_so101_v1_link` の 8 つ）。SRDF のほかの `elbow_flex`（`:10,17,24`）は `group_state` の**関節**の値で、正しい用法。つまり、関節名とリンク名が混ざっている（リンク名を書くべき `link2` に関節名が入っている）。
2. **過去のログ**: `~/so_arm101_check/` の `*.log`（73 件）のうち、**move_group を起動したログ 36 件の全部に同じ警告が 1 回ずつ**（残りはキャリブ道具のログで、move_group を起動しない）（`write_C4_step6.log`・`write_W4.log` は 2 回＝起動が 2 回）。最古は `lerobot_launch.log`（2026-11-10 18:28。LeRobot 経路でも出ていた）、次が `feetech_launch.log`。C4 の `write_C4_W0_*.log` にもある。**L1 で出始めたものではない**。さらに、旧ワークスペースの SRDF（`~/ros2_so101/src/so101_moveit_config/config/so101_new_calib.srdf`、初回コミット 39c2966）と**現在の SRDF は完全に同一**（`diff` で差なし）。旧ワークスペースの時点からある。
3. **本来の意図と、今の影響**（推測を含む）:
   - `gripper_link` と、**肘の関節 `elbow_flex` で動くリンク**（`elbow_flex` の child は `lower_arm_link`、parent は `upper_arm_link`）の衝突を無視する意図だったと思われる。ただし、どちらのリンクを書くつもりだったかは**確定できない**（[推測]）。
   - **今の影響は無いと見られる**: `gripper_link`×`lower_arm_link`（`:40`）と `gripper_link`×`upper_arm_link`（`:44`）は、SRDF にすでに `Never` で入っている。どちらの意図でも、**無視するはずの衝突は、ほかの行で無視できている**。この行は MoveIt に捨てられるだけで、警告以外の副作用は確認していない（実際の衝突検査の挙動は、この調査では動かして確認していない）。
   - 修正案（**未実施・承認が要る**）: この 1 行を削除する、または `link2` を実在のリンク（`lower_arm_link` など）にする。どちらも、同じ組が既にあるので、実質は「1 行の削除」になる。URDF の作り直しの段階（`lerobot_removal_plan.md` 10 章）で、SRDF 全体を見直すときにまとめて直すのが自然。

## L2 実施: LeRobot のコードと道具の削除（2026-11-12。`git rm`。実機・シリアルポートには触れていない）
- **戻すとき**: タグ `stage2c-done`（1a3628e）から取り出せる（例: `git checkout stage2c-done -- src/so_arm101_lerobot_bridge`）。
- **削除（`git rm`、合計 43 ファイル）**: `src/so_arm101_lerobot_bridge/`（34 ファイル）、`so_arm101_hardware` の `src/so101_hardware_bridge.cpp` と `include/so_arm101_hardware/so101_hardware_bridge.hpp`、`tools/gen_conversion_golden.py`・`tools/lerobot_readonly_state.py`・`tools/convert_lerobot_calibration.py`、候補 B の 4 ファイル（`so_arm101_bringup/launch/include/camera.launch.py`、`config/so101_cameras.yaml`・`so101_realsense2.yaml`・`so101_usb_cam.yaml`）。
  - `convert_lerobot_calibration.py` は、実行前に grep で、今の手順書・試験・CMake・スクリプトから使われていないこと（ヒットは、試験データ yaml のコメントと `converted_by` の文字列、`test_conversion.cpp:203` の文字列リテラル、記録の docs だけ）を確かめてから削除した。文字列の中立化は L3。
  - 削除後に `src/so_arm101_lerobot_bridge/` に残った git 管理外のもの（`__pycache__` の `.pyc` 8 つと vim の `.swp` 1 つ。すべて ignore 対象で、git 管理下は 0）は、ユーザーが vim を開いていないことを確認した上で、ディレクトリごと削除した（find の一覧を報告してから）。
- **編集**: `so_arm101_hardware/CMakeLists.txt`（`so101_hardware_bridge` の `add_library`・`ament_target_dependencies` と `install(TARGETS)` の対象を削除。コメントの `so101_hardware_bridge.hpp` を外した）、`so101_hardware_bridge_plugins.xml`（`So101HardwareBridge` の `<library>` を削除。ファイル名は L3 まで変えない）、`sts3215_system.hpp:8` のコメント、`setup_so101.sh`（`LECONDA_SITE_PACKAGES` の 2 行を削除。**このファイルは `.gitignore` の対象（git 管理外）なので、ローカルの編集だけで、コミットには含まれない。タグ `stage2c-done` にも無い**）。
- **承認の後に追加した CMake の変更**: `so_arm101_bringup/CMakeLists.txt` の `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})` の 2 行を削除。理由: 候補 B の削除で `config/` が空になり git から消え、symlink-install で `can't find .../so_arm101_bringup/config` になったため。他に bringup の `config/` を参照するものは無い（grep）。ユーザーが了承済み。
- **確認（ライブラリ）**: `Sts3215System`（`libso_arm101_sts3215`、`sts3215_system.cpp` の `PLUGINLIB_EXPORT_CLASS`）は、`So101HardwareBridge`（`so101_hardware_bridge`）とは別のライブラリ。
- **クリーンビルド**（ユーザーの指示で 2 回。2 回目は `build/` と `install/` だけを消して、1 回で 7 パッケージ成功。`log/` は消していない）: `ros2 pkg list` に `so_arm101_lerobot_bridge` なし、`install/` にも bridge なし。install の plugin xml は `xmllint` OK、`<class>` は `Sts3215System` の 1 つ、`So101HardwareBridge` は 0（symlink-install のため src へのリンク）。`install/so_arm101_hardware/lib/` は `libso_arm101_sts3215.so` だけ。`nm -DC` で `class_loader::impl::registerPlugin<so_arm101_hardware::Sts3215System, hardware_interface::SystemInterface>` があり、`So101HardwareBridge` のシンボルは 0。
- **試験の件数（前 → 後）**: hardware gtest **99 → 99**、test-result **105 → 105**。calibration gtest **71 → 71**、**73 → 73**。moveit_config 0 → 0。失敗は hardware・calibration とも 0。**bringup（lint のみ）: 18 件・失敗 12 → 16 件・失敗 12**（copyright 2/1 → 1/1、pep257 5/4 → 4/4、flake8 4/4・CTest 5/3 は変更なし）。
  - **減り方の理由**: lint は、ファイルごとに 1 件の試験を作る。削除した `camera.launch.py` の copyright と pep257 の 2 件が消えた。失敗の 12 件は変わらない（個別のファイル名の照合まではしていない）。L1 の `bringup_with_cameras.launch.py` と同じ種類の減り方で、解消ではなく、削除による。ユーザーが了承済み。
  - `so_arm101_lerobot_bridge` の試験（「結果ファイルなし」の失敗 1 件）は、パッケージごと消えた（解消ではなく削除による）。
- **変更していないもの**: SRDF の `elbow_flex` の行（URDF の作り直しの段階で直す）、C++ のロジック、URDF、関節リミット、yaml、`LICENSE.so101_ros2`。`so_arm101_hardware` の package.xml の `std_msgs`・`sensor_msgs` への依存は、bridge の削除で不要になった可能性があるが、**今回は変えていない**（後の整理の候補）。
- **残っている LeRobot への言及（L3 で扱う）**: `[diag]` の文言、試験データ名、plugin xml のファイル名、docs の手順（`LECONDA_SITE_PACKAGES` を使う節など。「歴史的」の注記）。
- **未実施**: L2 の後の、ユーザーによる読み取り専用の起動（`Sts3215System` が読み込めることの確認）。L3 は、その後。

## L2 の後の実機確認: 合格（2026-11-12。ユーザーの実機の報告。読み取り専用の起動 1 回。ログ `~/so_arm101_check/L2_readonly.log`）
- `Loading hardware 'SO101System'` → `[Sts3215System]: initialized: port=/dev/ttyACM0`（**L2 の後でも `Sts3215System` を読み込めた**。bridge のライブラリを消した影響なし）。
- EEPROM vs yaml **6/6 match**（`MATCH` が 6 行）、`servo settings vs LeRobot` **6/6 match**（文言は L3 まで旧いまま）。コントローラ 3 つ（joint_state_broadcaster・gripper_controller・arm_controller）が `Configured and activated`。
- 停止時: `deactivated (no register was written). reads=663 retried=0 failed_cycles=0`、bus はすべて 0、exit=0。
- 結論: L2（bridge・道具・candidate B の削除、CMake・plugin xml の変更）は、実機の読み取りの挙動に影響していない。

## `setup_so101.sh` から削除した 2 行の元の文面（git 管理外・タグ `stage2c-done` にも無いので、ここに残す。2026-11-12）
`setup_so101.sh` は `.gitignore`（24 行目）の対象。L2 で、`# lerobot python path` の行から空行までの 3 行（2 行 + 空行）を削除した。元の文面（`source ~/ros2_so-arm101/install/setup.bash` の `# workspace` の前、`RCUTILS_COLORIZED_OUTPUT` の行の後にあった）:
```
# lerobot python path
export LECONDA_SITE_PACKAGES=/home/hirok/miniforge3/envs/ros2_lerobot/lib/python3.10/site-packages
```

## L3 実施: 名前・文言・試験の整理（2026-11-12。挙動・比較の基準値・yaml の形式は変えていない。実機・シリアルポートには触れていない）
- **事前の確認（本番 `follower_calibration.yaml` のコメントを変える前）**: yaml 全体のハッシュ・文字列の比較は、ドライバ・`so_arm101_calib`（compare・apply・backup）・試験のどこにも無い。`bytes_checksum`・`measure_checksum` はバックアップ／測定ファイルの json 用、ほかはパケットのチェックサム。本番の yaml は `YAML::LoadFile`／`load_calibration` による構造の読み込みだけ（コメントは無視される）。`test_measure_apply.cpp` の `yaml_text()` は一時ディレクトリの yaml の前後比較で、`TheRenderedYamlKeepsTheOldFormat` は `conversion: {angle_ticks: 4095}` などの部分文字列を検査し、コメント行は見ない。`test_calibration.cpp:396` は package.xml の `<version>` だけを読む。
- **ログの文言（`sts3215_system.cpp`）**: `[diag] servo settings vs LeRobot: n/6 joints match` → `... vs reference: ...`、`MATCH (LeRobot)` → `MATCH (reference)`、`DIFFERS from LeRobot values:` → `DIFFERS from reference values:`（3 か所）、`servo settings differ from LeRobot values:` → `... from reference values:`、`(expected 0, the state LeRobot ran with)` → `(expected 0, the reference state)`。**比較の基準値（`servo_config`）・判定（`config_action`）は変えていない**。出典（LeRobot 0.4.2、Apache-2.0）の記載は `servo_config.hpp`・`calibration.hpp`・`sts3215_bus.hpp` に残した。
- **コメントなど**: `calibration.hpp`・`calibration.cpp`・`servo_config.hpp`・`fake_servo.hpp`・`test_sts3215_bus.cpp`・`test_sts3215_system.cpp`・`measure.cpp` の文言。旧形式 `kind: converted_from_lerobot_json` の**読み込みは残した**（`calibration.cpp` のコメントに「互換のため残す」）。
- **yaml のコメント**: `apply.cpp:71` と本番 `follower_calibration.yaml:12`: `# LeRobot 等価（分母 4095）。` → `# 基準の実装（LeRobot 0.4.2）と同じ（分母 4095）。`（キー・値・形式は変更なし）。
- **固定ファイルの改名**: `test/data/follower_calibration_lerobot_json_20260907.yaml` → `follower_calibration_fixture_20260907.yaml`（`git mv`。**git の判定で 100% の改名、中身の差は 0**）。CMake の 4 か所（hardware 2、calibration 2）を更新。黄金表（`conversion_golden.csv`・`unnormalize_golden.csv`）は名前を変えず、**変更はヘッダ 2 行ずつだけ**（1 行目のコメントを「LeRobot 0.4.2 … (Apache-2.0) から生成。生成スクリプトは削除済み（タグ stage2c-done の `tools/gen_conversion_golden.py`）」に、2 行目の列名 `lerobot_value`→`reference_value`、`lerobot_raw`→`reference_raw`）。数値の行は変更なし。試験は列を位置で読むので、列名の変更に影響されない。
- **plugin xml**: `so101_hardware_bridge_plugins.xml` → `so_arm101_hardware_plugins.xml`（`git mv`）。`package.xml` の `plugin_description_file` と、`CMakeLists.txt` の `pluginlib_export_plugin_description_file` を更新。`Sts3215System` の `<description>` を今の挙動に（「既定（read_only:=true）は読み取り専用。read_only:=false でトルク ON と Goal_Position の書き込み」）。最初の通常ビルドでは、`install/` に古い名前の宙ぶらりんのリンクが 1 つ残ったので、ユーザーの条件どおり `build/` と `install/` だけを消してクリーンビルドした（`log/` は消していない）。クリーンビルド後: 古い名前のファイル 0、宙ぶらりんのリンク 0、ament index（`hardware_interface__pluginlib__plugin/so_arm101_hardware`）は `share/so_arm101_hardware/so_arm101_hardware_plugins.xml` だけ、`xmllint` OK、`<class>` は `Sts3215System` の 1 つ、`So101` は 0、`nm -DC` で `registerPlugin<so_arm101_hardware::Sts3215System,...>`、`ros2 pkg list` に lerobot なし。
- **package.xml**: maintainer を `hiro <hirohiro20051009@gmail.com>` に（bringup・controller・description・hardware・calibration・moveit_config の 6 つ。**license 欄は変更なし**）。変更前: bringup・controller・description・hardware は `Nimrod <nimicu21@gmail.com>`、calibration は `hirokazu <hirohiro20051009@gmail.com>`、moveit_config は `hirok <chimay_beer@outlook.jp>`。**`apriltag_test` は変更なし**（package.xml も setup.py も。候補 A として後で削除する）。Nimrod 由来の 4 パッケージ（bringup・controller・description・hardware）には、元の作者の記録として `<author email="nimicu21@gmail.com">Nimrod</author>` を `<license>` の直後に追加。
- **手順書**: `feetech_servo_config_check.md`（`servo settings vs reference`、`DIFFERS from reference values`、「基準の値」。旧文言は「L2 までは…」と併記）、`so_arm101_calibration_check.md:361`（同）。「歴史的」の注記: `feetech_readonly_check.md`（冒頭）、`feetech_servo_config_check.md`（記録 A の節）、`stage2c_plan.md`（archive を実施せず `git rm` にした注記）、`feetech_write_check.md:18`。**`inventory.md` の過去の記録は書き換えていない**。`tools/` の docstring（`lerobot_P1.csv` などの例）は今回触れていない。
- **試験名の対応表**（`--gtest_list_tests` の前後比較。hardware 99 → 99、calibration 71 → 71。差分は下の 7 件の名前だけで、追加・削除なし）:

| 変更前 | 変更後 |
|---|---|
| `Conversion.MatchesLeRobotGoldenTable` | `Conversion.MatchesReferenceGoldenTable` |
| `Conversion.GripperIsClampedToRangeLikeLeRobot` | `Conversion.GripperIsClampedToRangeLikeReference` |
| `Conversion.AngleIsNotClampedLikeLeRobot` | `Conversion.AngleIsNotClampedLikeReference` |
| `Conversion.YamlMatchesLeRobotJsonSource` | `Conversion.YamlMatchesFixtureSource` |
| `SignMagnitude.DecodesLikeLeRobot` | `SignMagnitude.DecodesLikeReference` |
| `InverseConversion.MatchesLeRobotUnnormalizeWithinOneTick` | `InverseConversion.MatchesReferenceUnnormalizeWithinOneTick` |
| `ServoConfig.ExpectedValuesAreLeRobotConfigure` | `ServoConfig.ExpectedValuesAreReferenceConfigure` |

  変数名: `lerobot_arm()`/`lerobot_gripper()` → `reference_arm()`/`reference_gripper()`、`lerobot_raw` → `reference_raw`。calibration の試験の一覧は前後で同一。
- **試験の件数（前 → 後。クリーンビルドの後）**: hardware gtest **99 → 99**、test-result **105 → 105**。calibration gtest **71 → 71**、**73 → 73**。失敗 0。moveit_config 0 → 0。bringup（lint）**16 件・失敗 12 → 16 件・失敗 12**（変化なし）。
- **残り（L3 では触れていない）**: `[diag] EEPROM vs yaml`（変更なし）、`sts3215_system.hpp` など出典・Apache-2.0 の帰属、`LICENSE.so101_ros2`、SRDF の `elbow_flex`、`tools/` の docstring・コメントの LeRobot の語（`fake_sts3215.py` など）、`bringup.launch.py` の `apply_servo_config` の description の「LeRobot の値」（launch の文言なので、次の機会）。
- **未実施**: L3 の後の、ユーザーによる読み取り専用の起動と W0（新しい文言 `servo settings vs reference: 6/6 joints match`、`MATCH (reference)` が 6 行、`[diag] EEPROM vs yaml: 6/6 joints match`、`startup_read_retries=0`、全カウンタ 0、停止後 `torque_enable=0 lock=1 acceleration=50`）。

## L3 の後の実機確認: 合格（2026-11-12。ユーザーの実機の報告。ログは `~/so_arm101_check/` の `L3_readonly.log`・`write_L3_W0.log`・`write_L3_W0_after.log`）
- **読み取り専用の起動**: `initialized: port=/dev/ttyACM0`、`EEPROM vs yaml` **6/6 match**、**`MATCH (reference)` が 6 行**、**`servo settings vs reference` 6/6 match**、`DIFFERS` なし、コントローラ 3 つ activated、`reads=367 retried=0 failed_cycles=0`、bus はすべて 0（`timeouts=0 bad_checksum=0` まで）、exit=0。電圧 11.8〜11.9 V、温度 29〜34 ℃。
- **W0（`probe_lock_sram:=true`）**: 腕は動かず保持。`servo settings vs reference` 6/6 match、`[probe] ACCEPTED on all joints`、`startup_read_retries=0`、`[deactivate] torque OFF and Lock=1 confirmed`、`deactivated. reads=898 goal_writes=898 monitors=17`（約 18 秒）`rate_limited=0 port_contention=0`、bus はすべて 0、exit=0。
- **W0 の後の読み取り専用**: 6 関節とも `torque_enable=0 lock=1 acceleration=50`、6/6 match が 2 行、`reads=399`、カウンタ 0、exit=0。
- 結論: L3（ログの文言・yaml のコメント・固定ファイルと plugin xml の改名・maintainer）は、実機の挙動に影響していない。新しい文言（`reference`）が、手順書の grep の期待どおりに出ることを確認した。

## L4 実施: LeRobot の削除の完了の記録と、CLAUDE.md の規則の更新（2026-11-12。実機・シリアルポートには触れていない）
- **完了の表**（各段階のコミット・試験の件数・実機確認）は `docs/lerobot_removal_plan.md` の冒頭「完了（L1〜L4）」に書いた。L1 `7b5360b`、L2 `b95cc8c`、L3 `b58df44`（実機確認の記録は `ed5620f`・`5bb8c0c`・`9d6fd5b`）。件数は L4 の前後で hardware 99/105、calibration 71/73、bringup lint 16 件・失敗 12 のまま。
- **戻し方の基本**（ユーザーの判断）: 「削除の直前のコミット」から取り出す（例: `git checkout b95cc8c~1 -- src/so_arm101_lerobot_bridge`）。タグ `stage2c-done`（`1a3628e`）は補足（L1 より前の状態）。
- **`CLAUDE.md` の規則を変更**（変更前: 「ファイルの削除はしない。不要なものは archive/ へ移動し、理由を記録する」）。変更後の 3 項目:
  1. `不要なものの削除は git rm で行う（archive/ には移さない）。削除のときは、戻すためのタグかコミットを docs/inventory.md に書く（例: 「タグ stage2c-done から取り出せる」）。承認なしに削除しない（削除の一覧と理由を示し、承認を得てから実行する）`
  2. `git 管理外のファイル（.gitignore の対象など）は git rm の対象外で、タグからも戻せない。削除や書き換えの前に、元の内容を docs/inventory.md に控える`
  3. `build/ と install/ を消すのはクリーンビルドのときだけで、直前に報告する。log/ は消さない`
  ほかの規則（pkill 禁止、コミット前の colcon test、承認ルール、実機に触れない、プロセスの確認など）は変更なし。
- **`bringup.launch.py` の `apply_servo_config` の説明**: 「LeRobot の値と違う EEPROM」→「基準の値（LeRobot 0.4.2 の値）と違う EEPROM」（文字列だけ。引数の名前・既定値 `false`・`choices` は変更なし。`ros2 launch -s` で確認）。
- **`docs/third_party.md`（grep で確認だけ。変更なし）**: LeRobot の語 16 行は出典・帰属・削除の説明。「削除したもの」の節の戻し方は、タグ `stage2c-done` を基本として書いてあり、上の新しい基本（削除の直前のコミット）とは書き方が違う（誤りではない。必要なら別途直す）。

## 候補の削除: A（apriltag_test）・G（tools/ の 5 本）・C の step/stl（2026-11-12。`git rm`。H は保留。実機・シリアルポートには触れていない）
- **戻し方（基本）**: **削除の直前のコミット `3a3b0c2`** から取り出す。例: `git checkout 3a3b0c2 -- src/apriltag_test`、`git checkout 3a3b0c2 -- src/so_arm101_description/assets`、`git checkout 3a3b0c2 -- tools/compare_joint_states.py`。補足: タグ `stage2c-done`（`1a3628e`）からも取り出せる（L1 より前の状態）。
- **削除（51 ファイル。すべて git 管理下で、管理外のファイルは無かった）**:
  - **A**: `src/apriltag_test`（10 ファイル: `launch/apriltag_d415.launch.py`、`config/tags_36h11.yaml`、`package.xml`、`setup.py`、`setup.cfg`、`resource/apriltag_test`、`apriltag_test/__init__.py`、`test/` の 3 本）。将来の AprilTag の Eye-to-Hand の試作は、上のコミットから戻せる。
  - **C の step/stl**: `src/so_arm101_description/assets/step/**` と `assets/stl/**`（36 ファイル）。**`meshes/`（13 個の stl。URDF が参照）と `usd/` には触れていない**。
  - **G**: `tools/pty_measure_check.py`・`pty_measure_notty_check.py`・`fake_sts3215.py`・`compare_joint_states.py`・`record_joint_states.py`（5 本）。**`pose_margin.py` と `send_joint_delta.py` は、手順書が使うので残した**。
- **削除前の `git grep`（記録の docs 以外）**: 参照なし。`fake_sts3215.py` を参照するのは `pty_measure_*` の 2 本だけ（同時に削除）。docs の参照は、`inventory.md`・`lerobot_removal_plan.md`（記録）、`stage2c_plan.md`、`feetech_readonly_check.md`（「歴史的」の注記つきの手順。`compare_joint_states.py`・`record_joint_states.py` を使う部分）だけ。`so_arm101_description/CMakeLists.txt:8` の `assets` はコメント。
- **robot_description の前後の diff（xacro で生成）**: feetech（read_only true / false）、mock、mock で `hardware` 未指定、description の `mode:=real`・`isaac`・`gazebo` の **7 通りすべて、削除の前後で完全に同一**。`hardware:=lerobot` は前後とも rc=2。URDF が参照する `package://so_arm101_description/meshes/*.stl` の 13 個は、install のパスに解決してすべて存在。
- **ビルド**: 通常のビルドでは `build/apriltag_test`・`install/apriltag_test` が残るため、ユーザーの条件どおり、直前に報告して `build/` と `install/` だけを消し（`log/` は消していない）、クリーンビルドした。**6 パッケージが 1 回で成功**（`apriltag_test` は無い）。`ros2 pkg list` と `install/` に `apriltag_test` なし。宙ぶらりんのリンク 0。
- **試験の件数（前 → 後）**: hardware gtest **99 → 99**、test-result **105 → 105**。calibration **71 → 71**、**73 → 73**。失敗 0。moveit_config 0 → 0。bringup（lint）**16 件・失敗 12 → 16 件・失敗 12**。`apriltag_test` の `pytest.xml: 1 test, 1 error`（結果ファイルなし。`colcon test --packages-select apriltag_test` で確認できていた）は、**パッケージごと消えたので出ない（解消ではなく消滅）**。
- **モックの起動確認（ユーザーが「実機は動いていません」と返答した後。`mock:=false` は起動していない）**: `mock:=true use_rviz:=false` を、自分で起動し、起動前の `ps` に ros2 の launch 系プロセスは無く、`ros2 node list` は空だった。20 秒ほど後に `ros2 control list_controllers` で **arm_controller・gripper_controller・joint_state_broadcaster が active**。止めるのは、控えた launch の PID（373222。セッションの子は tf2 の static_transform_publisher・robot_state_publisher・ros2_control_node・move_group）への `kill -INT` だけ。停止後、`ps` は空。`ros2 node list` に `/move_group`・`/moveit_simple_controller_manager` が残ったので、`ros2 daemon stop; ros2 daemon start` の後に再確認して、**両方とも空**。停止時に `move_group` が exit code -11（既知。実機の終了時と同じ形）。
- **H（`archive/`）: 今回は削除していない（保留）**。git 管理下は 13 ファイル（`archive/stage2a/**`）。**git 管理外の `archive/stage1_build/`（`.gitignore:21` の対象。古い build/install 一式。552 ファイル・約 20M。`so101_bringup`・`so101_controller`・`so101_description`・`so101_moveit_config` が 71 ファイルずつ、`so101_hardware_interface` 86、`so101_ros2_bridge` 15、`apriltag_test` 14、`install/` 側など）があり、git にもタグにも無く戻せない**。ユーザーの判断を待つ。
- **残る候補（保留）**: C の `usd/`、D、E、F。

## 候補 H: `archive/` の削除（2026-11-12。実機・シリアルポートには触れていない）
- **削除前の確認**: `git grep` で、記録の docs 以外から `archive/` を参照するものは無い（残っていたのは `.gitignore:21` の `archive/stage1_build/` の行と、`CLAUDE.md` の規則の文面だけ）。`archive/` 直下は `stage1_build/`（git 管理外）と `stage2a/`（git 管理下）だけで、`stage2a/` に git 管理外のファイルは無かった。
- **控え（リポジトリの外）**: `~/so_arm101_check/archive_stage1_build_20261112.tar.gz`（2,124,567 バイト、sha256 `77ef368f3beab9e1a292b7497b9cf3aafd073acd267ce5e8b17f154659850fb0`。`cd archive && tar czf ... stage1_build` で作成）。`tar tzvf` で数を確認: 通常ファイル **539**、シンボリックリンク 163、ディレクトリ 301、合計 1003 エントリ。ディスク上の数（`find`: ファイル 539・リンク 163・ディレクトリ 301）と完全に一致。**訂正**: 以前の報告の「`stage1_build` は 552 ファイル」は誤りで、552 は `archive/` 全体（`stage1_build` の 539 + 管理下の `stage2a` の 13）の通常ファイル数だった。
- **`archive/stage1_build/`（git 管理外。段階 1 の最初の `colcon build` の `build/`・`install/`・`log/` を、移動したもの。作成日 2026-11-05。約 19M）の構成**（通常ファイル数・大きさ）:
  - `build/`: `apriltag_test` 14 ファイル・108K、`so101_bringup` 71・628K、`so101_controller` 71・596K、`so101_description` 71・612K、`so101_hardware_interface` 86・**15M**（最大）、`so101_moveit_config` 71・596K、`so101_ros2_bridge` 15・152K、`build/` 直下 2（`.built_by`・`COLCON_IGNORE`）。
  - `install/`: `apriltag_test` 13・116K、`so101_bringup` 9・192K、`so101_controller` 9・160K、`so101_description` 13・272K、`so101_hardware_interface` 12・188K、`so101_moveit_config` 16・268K、`so101_ros2_bridge` 16・172K、`install/` 直下 12。
  - `log/`: 38 ファイル・356K。
  - 旧名のパッケージ（`so101_*`）の成果物で、今のパッケージ名（`so_arm101_*`）ではない。
- **削除**: `archive/stage1_build/` をディレクトリごと削除（控えは上の tar.gz）。`archive/stage2a/**`（git 管理下の 13 ファイル: `README.md`、`bringup/`（launch 2・rviz 2）、`controller/`（config 1・launch 2）、`description/`（launch 3・rviz 1）、`moveit_config/launch/demo.launch.py`）を `git rm`。**`archive/` ディレクトリは無くなった**。
- **戻し方**: `stage2a/**`（管理下）は、**削除の直前のコミット**から取り出す（例: `git checkout <この削除の 1 つ前のコミット> -- archive/stage2a`。1 つ前は `759893f`）。補足: タグ `stage2c-done` からも取り出せる。`stage1_build/`（管理外）は、`tar xzf ~/so_arm101_check/archive_stage1_build_20261112.tar.gz -C <展開先>`（展開すると `stage1_build/` ができる。`archive/` に戻すなら `-C ~/ros2_so-arm101/archive`。ただし、`.gitignore` の対象で、colcon は `archive/` を見ない）。
- **`.gitignore:21`（`archive/stage1_build/`）**: 対象のディレクトリが無くなったので不要になったが、今回は変えていない（削除の対象ではない。再び作られたときの保険として残る）。
