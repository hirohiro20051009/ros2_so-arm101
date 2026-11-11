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
- 事実（モック。`mock:=true use_rviz:=false` の `/check_state_validity`、group=arm、実機は使っていない）:
  - 問題2の値（shoulder_lift −1.749、elbow_flex 1.700、wrist_flex 1.313、pan 0、roll 0）: **無効**、contacts は `upper_arm_link` ↔ `wrist_link`（元の記録と一致。再現した）。
  - 同じ姿勢を URDF のリミット内に丸めた値（−1.745 / 1.69 / 1.313）: **有効**。SRDF の home（−1.57 / 1.57 / 0.8）: 有効。
- 推測（未確認）: 問題2の「衝突」は、関節リミットを超えた値（lift −1.749、elbow 1.700）が原因だった可能性がある。元の記録は「リミット超過は中止の原因ではない」としているが、上の結果はそれと食い違う。リミット内の値で再現するかは、問題2の実機での姿勢を再確認しないと断定できない。
- 変更なし: URDF・SRDF のリミットや home の値は変えていない（変更が必要と考えたら計画を出して承認を取る）。

## 段階2b-3c: W3.5 の最終案（2026-11-11 承認。手順書 `docs/feetech_write_check.md` に反映）
- W3.5/W4 の出発点と W4 後の戻し先: 調整後の値（pan -0.08362、lift -1.51134、elbow +1.66938、wrist_flex +0.50941、roll -0.07135、gripper +0.01645。`~/so_arm101_check/start_pose.log`）。lift は +0.022 rad、wrist_flex は約 -0.43 rad 調整した（トルク OFF で手で動かした）。
- モック判定（すべて有効）: 出発点、elbow -0.10 / -0.15 / -0.20 / -0.30。先端 z（FK、base_link 基準）の出発点からの差: +0.026 / +0.039 / +0.053 / +0.081 m（出発点の z は +0.0304 m）。
- W3.5: elbow_flex に -0.10 を 2 回（--seconds 4、各回の後に実測確認）。出発点から -0.15〜-0.20 で終了、届かなければ最大 -0.05 を 1 回だけ追加、-0.25 を超える指令は送らない。lift は動かさない。
- 起動: W3.5 と W4 は同じ起動（RViz あり、`torque_limit:=500`、`write_W4.log`）。
- W4 の戻し: RViz は使わず `send_joint_delta.py` で 1 関節ずつ「出発点 − 実測」を送る（elbow_flex は最後）。Ctrl-C の前に各関節が出発点から約 0.02 rad 以内であることを確認する。W4 の目標は elbow_flex を + に動かさない（リミット +1.69 まで 0.021 rad）。
- 後始末の事実: バックグラウンド起動した launch は SIGINT が効かず、SIGTERM と PID 指定で停止した（最後の ps で残りなしを確認）。

## 段階2b-3c: W3.5・W4 合格、W0〜W4 すべて合格（2026-11-11。ユーザー報告。ログ: `~/so_arm101_check/write_W4.log`、`use_rviz:=true torque_limit:=500`）

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
