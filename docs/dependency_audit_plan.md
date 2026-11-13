# 依存の監査の計画（読み取りのみ）

作成: 2026-11-13。**計画だけ。実行していない。** 監査そのものも読み取りだけ（ファイルの変更、ビルド、ノード・モックの起動、シリアルポートを開く操作、ネットワークへのアクセスをしない）。W3 の調査（`docs/servo_w3_investigation_plan.md`）の後の作業。承認を得てから実行する。

## 1. 目的と範囲

5 つのパッケージ（`so_arm101_bringup`・`calibration`・`description`・`hardware`・`moveit_config`）と `tools/`、`setup_so101.sh` について、次の 4 つを、**事実（grep・ソース・ビルド生成物）で**確かめる。

| 観点 | 問い |
|---|---|
| (a) 宣言漏れ | コード・launch・試験が使っているのに、`package.xml`（と CMake の `find_package`）に無い依存 |
| (b) 過剰 | 宣言があるのに、どこからも使っていない依存（削除の候補。**削除は別に承認**） |
| (c) 種類の誤り | `depend`／`build_depend`／`exec_depend`／`test_depend` の区分、`CMakeLists.txt` の `find_package`・`ament_export_dependencies` との食い違い |
| (d) 版・出典 | 使っている外部パッケージ（apt の ROS Humble、pip の `~/.local`）の版と、ライセンス（`THIRD_PARTY_NOTICES.md`・`docs/license_plan.md` との整合） |

範囲外: 依存の追加・削除・版の固定（結果を見てから別に計画）、rosdep のインストール、`~/.local` の anyio/pytest の件（案 0 で決定済み。`docs/inventory.md`）。

## 2. 事前に分かっている候補（grep で見た事実。監査で確定する。**未確定**）

| 候補 | 観察 [事実] | 見立て [推測] |
|---|---|---|
| bringup の `usb_cam`・`realsense2_camera`・`realsense2_description` | `package.xml` にだけ出る。bringup の launch・tools・config に使用なし（`grep -rln "usb_cam\|realsense" src tools` の結果は `package.xml` だけ） | 将来（AprilTag のキャリブ）の置き場所の宣言。**過剰の候補**。CLAUDE.md の最終構成（`so_arm101_calibration`）が使うなら、置き場所が違う |
| bringup の Python の import | launch・試験・tools が `rclpy`、`control_msgs`、`trajectory_msgs`、`sensor_msgs`、`numpy`、`pytest`、`yaml`、`ament_index_python`、`launch`、`launch_ros` を使う。bringup の `package.xml` には、`rclpy` `control_msgs` `trajectory_msgs` `sensor_msgs` `python3-numpy` `python3-yaml` `ament_index_python` `launch` `launch_ros` の宣言なし（`exec_depend` は、description・moveit_config・hardware・controller_manager・moveit_configs_utils・moveit_servo・forward_command_controller・カメラ 3 つ） | **宣言漏れの候補**（`tools/` はパッケージの外で、どのパッケージの責任か決める） |
| hardware の `std_msgs`・`sensor_msgs` | `package.xml` と `ament_export_dependencies`（`CMakeLists.txt:84`）にあるが、`src/` と `include/` の `#include` には出ない（監査で `grep "std_msgs/\|sensor_msgs/"` で確定する） | **過剰の候補**。ただし CMake のコメント（:80）が「ヘッダを include するので」と書く。**コメントと実際の食い違い** |
| hardware の `rmw_cyclonedds_cpp`（`exec_depend`） | コードは使わず、手順書が `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` を設定（`setup_so101.sh`・`docs/feetech_*.md`） | 実行環境の前提。**区分は妥当か（exec_depend で良いか）を確認** |
| hardware の `rclcpp_lifecycle` | `package.xml` に宣言あり、`find_package` に無い。ヘッダ `sts3215_system.hpp:22` で include | **`find_package` の不足（推移的に来ているだけ）の候補** |
| calibration の CMake | `find_package(so_arm101_hardware)`・`ament_index_cpp`・`yaml-cpp` あり。`package.xml` と一致 | 問題なしの見込み |
| moveit_config の `exec_depend` | `moveit_setup_assistant`、`warehouse_ros_mongo`、`moveit_ros_warehouse`、`moveit_ros_visualization`、`joint_state_publisher_gui`、`moveit_planners`、`moveit_simple_controller_manager`、`moveit_kinematics` など多数。launch は `warehouse_db`・`setup_assistant`・`moveit_rviz` を持つ。コメントアウトされた行（`joint_trajectory_controller`、`gazebo_ros_control`）あり | Setup Assistant 一式・Servo 一式は**ユーザーの判断で残す**（`docs/inventory.md`）。**残すものとして、依存の宣言と実使用の対応を記録するだけ（削除候補にしない）**。`joint_trajectory_controller`（実際は controllers.yaml が使う）の宣言漏れの可能性 |
| description の `joint_state_publisher*`・`rviz*`・`tf2_ros`・`ros2launch`・`controller_manager` | `package.xml` に exec_depend。description の launch・xacro は？（`docs/original_urdf_plan.md` 22 章で整理済み。再確認） | 整理済みの見込み。**22 章の結論との一致を確認**する |
| `so_arm101_moveit_config` が `so_arm101_hardware` に exec 依存 | `package.xml:46`。moveit_config の launch・config に hardware の使用が無いなら過剰 | 確認 |

## 3. 手順（すべて読み取り）

| 段 | 内容 | コマンドの種類 | 成果物 |
|---|---|---|---|
| T1 | 宣言の一覧: 5 つの `package.xml` と `CMakeLists.txt` の依存（`depend`／`build_depend`／`exec_depend`／`test_depend`、`find_package`、`ament_target_dependencies`、`target_link_libraries`、`ament_export_dependencies`）を表にする | `grep`・Read | 宣言表 |
| T2 | 使用の一覧: C++ の `#include <...>`・`"..."`、Python の `import`／`from`、launch の `package=`・`FindPackageShare`・`get_package_share_directory`・`IncludeLaunchDescription`・`PythonLaunchDescriptionSource`、xacro の `$(find …)`、yaml・URDF の `package://`、plugin の XML（`hardware_interface` のプラグイン記述）を、パッケージごとに集める | `grep`・Read | 使用表 |
| T3 | 突き合わせ: 宣言 ⇔ 使用（(a) 宣言漏れ、(b) 過剰、(c) 区分の誤り）。**推移的な依存で通っているだけのもの**（`find_package` が無いのにビルドが通る）を別に分ける。外部の標準ライブラリ（`os`・`sys` など）と、ament_* の試験依存は除く | 手計算 | 差分表（確定と推測を分ける） |
| T4 | ビルド生成物との照合（**読むだけ。ビルドしない**）: `install/*/share/*/package.xml`、`install/*/share/*/cmake/*Config.cmake` の依存、`build/*/CMakeCache.txt` の `*_DIR`、`ldd install/so_arm101_hardware/lib/libso_arm101_sts3215.so`（実際にリンクした共有ライブラリ） | `ldd`・Read | 実際のリンクと宣言の一致 |
| T5 | 外部パッケージの版: `dpkg -l` で、使う ROS Humble のパッケージ（`ros-humble-moveit-servo` など）の版（今回 `moveit_servo 2.5.10-1jammy.20261006.192111`）、`pip3 list`／`python3 -c "import numpy; print(numpy.__version__)"`（`-I` を付ける）で Python の版。**どの Python（system か `~/.local`）が `import` されるか**を `python3 -I -c "import numpy, yaml; print(numpy.__file__, yaml.__file__)"` で確認（`~/.local` の混入の把握。変更はしない） | `dpkg -l`・`python3 -I -c` | 版表 |
| T6 | ライセンスの整合: T5 の外部依存を `THIRD_PARTY_NOTICES.md`・`docs/third_party.md`・`docs/license_plan.md` と照合。**配布物（リポジトリ）に含まれるもの**と、**実行時に apt で入る依存**を分ける（後者は通知の対象外が通常。ただし、含める方針はユーザーの判断） | Read | 整合表 |
| T7 | `rosdep` の解決可能性（**ネットワークと rosdep の DB を使うので、まず `rosdep db`／キャッシュの有無だけを読み取りで確認し、`rosdep update` や `rosdep install` はしない**）。キャッシュが無ければ、この段は省略して記録する | `ls ~/.ros/rosdep` | 省略または結果 |
| T8 | 報告: 結果を `docs/dependency_audit.md`（新規）に書く。**確定（事実）と推測を分ける**。直す案は、**案だけ**（削除は `git rm`・承認が要る。`package.xml` の変更は別に承認）。`docs/inventory.md` には 1 節だけ足す | Write | 報告書 |

## 4. 規則との対応

- ファイルの変更は T8 の docs のみ。`package.xml`・`CMakeLists.txt` は**変更しない**（直す案は別に承認を取る）。
- ビルド・`colcon test`・起動・シリアルポート・ネットワーク（`rosdep update`、`pip install`、`apt`）は行わない。`~/.local` は読むだけ（変更しない）。
- 削除の候補（カメラ 3 つなど）は、**一覧と理由を示して承認を得てから**。git 管理外（`.gitignore` の対象）の物は、`build/`・`install/`・`log/` を含め触らない。
- docs のコミットは、`colcon test` の件数を前後で比べてから（CLAUDE.md）。

## 5. 決めてほしいこと

1. この計画（T1〜T8）で実行してよいか。
2. T7（`rosdep`）は、キャッシュの有無の確認だけにしてよいか。
3. `tools/`（パッケージの外のスクリプト）の依存（`numpy`・`rclpy`・`control_msgs` など）を、どのパッケージの責任として監査するか（案: bringup の `exec_depend` に含める前提で、宣言漏れの候補として記録するだけ）。
