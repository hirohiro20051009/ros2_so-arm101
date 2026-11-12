# Copyright 2026 hiro
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
"""
Unified launcher for the SO-ARM101 (root namespace).

SO-ARM101 統合ランチャー（ルート名前空間）。

  mock:=true  (既定) : mock_components/GenericSystem でシミュレーション
  mock:=false        : 実機（Sts3215System。STS3215 を直接制御。LeRobot を使わない）
                       read_only:=true (既定) は読み取り専用で、サーボのレジスタには書き込まない。
                       書き込みモード（トルク ON。腕が動く）は read_only:=false を明示したときだけ
                       （手順は docs/feetech_write_check.md）

宣言していない引数（打ち間違いや、削除した hardware 引数など）が渡されたら、どのノードも起動する前に
エラーで止まる（_check_launch_arguments。LaunchDescription の最初の動作）。
LeRobot 経路は削除した（タグ stage2c-done に残っている）。

robot_state_publisher / ros2_control_node / move_group は1つずつ、
コントローラ名・トピックは mock / 実機で共通（/arm_controller, /gripper_controller, /joint_states）。
"""

import difflib
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


# Names added by "ros2 launch" itself (--launch-prefix / --launch-prefix-filter).
_ROS2LAUNCH_OPTION_ARGS = ("launch-prefix", "launch-prefix-filter")


def _check_launch_arguments(context, declared_names, *args, **kwargs):
    """
    Stop before any node starts if an argument that is not declared was given.

    Only the arguments typed by the user (context.argv) are checked. Values set inside the
    launch description (included launch files, OpaqueFunction, ...) live in
    context.launch_configurations and are never looked at.
    """
    given = [a.split(":=", 1)[0] for a in context.argv if ":=" in a]
    allowed = set(declared_names) | set(_ROS2LAUNCH_OPTION_ARGS)
    unknown = sorted({name for name in given if name not in allowed})
    if not unknown:
        return []
    parts = []
    for name in unknown:
        close = difflib.get_close_matches(name, sorted(allowed), n=2)
        hint = f" (did you mean: {', '.join(close)}?)" if close else ""
        parts.append(f"{name}{hint}")
    message = (
        "Undeclared launch argument(s): " + ", ".join(parts) + ". "
        "Declared arguments: " + ", ".join(sorted(declared_names)) + "."
    )
    if "hardware" in unknown:
        message += (
            " The 'hardware' argument was removed (with the removal of the LeRobot path; "
            "git tag stage2c-done). Remove it and run again."
        )
    raise RuntimeError(message)


def generate_launch_description():
    mock = LaunchConfiguration("mock")
    serial_port = LaunchConfiguration("serial_port")
    calibration_file = LaunchConfiguration("calibration_file")
    read_only = LaunchConfiguration("read_only")
    acceleration = LaunchConfiguration("acceleration")
    torque_limit = LaunchConfiguration("torque_limit")
    apply_servo_config = LaunchConfiguration("apply_servo_config")
    probe_lock_sram = LaunchConfiguration("probe_lock_sram")

    # robot_description は全ノード（rsp / ros2_control_node）で同一。
    # mock で ros2_control のプラグインだけが切り替わる（serial_port 等の hardware 用の引数は mock のとき無視される）。
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so_arm101_moveit_config")
        .robot_description(
            mappings={
                "use_fake_hardware": mock,
                "serial_port": serial_port,
                "calibration_file": calibration_file,
                "read_only": read_only,
                "acceleration": acceleration,
                "torque_limit": torque_limit,
                "apply_servo_config": apply_servo_config,
                "probe_lock_sram": probe_lock_sram,
            }
        )
        .to_moveit_configs()
    )
    moveit_launch = moveit_config.package_path / "launch"
    controllers_yaml = os.path.join(
        get_package_share_directory("so_arm101_bringup"), "config", "controllers.yaml"
    )

    ld = LaunchDescription()
    # 宣言していない引数の検査。最初の動作（どのノードより前）。宣言の一覧は、末尾で埋める
    declared_names = []
    ld.add_action(OpaqueFunction(function=_check_launch_arguments, args=[declared_names]))
    ld.add_action(
        DeclareBooleanLaunchArg(
            "mock", default_value=True, description="true=mock / false=実機（STS3215 を直接制御）"
        )
    )
    ld.add_action(DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"))
    ld.add_action(
        DeclareLaunchArgument(
            "calibration_file",
            default_value=os.path.join(
                get_package_share_directory("so_arm101_hardware"),
                "config",
                "follower_calibration.yaml",
            ),
            description="キャリブレーション yaml",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "read_only", default_value="true", choices=["true", "false"],
            description="true=読み取り専用（既定）/ false=書き込みモード（トルク ON。腕が動く。"
            "read_only:=false を明示したときだけ有効。手順は docs/feetech_write_check.md）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "acceleration", default_value="50",
            description="書き込みモードの Acceleration（1..254。0 は拒否。毎回トルク ON の前に書く）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "torque_limit", default_value="1000",
            description="書き込みモードの Torque_Limit。shoulder_pan と wrist_roll だけに効く"
            "（lift / elbow / wrist_flex は常に 1000、"
            "gripper は min(値, 500)）。W1〜W2 の初回は 500",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "apply_servo_config", default_value="false", choices=["true", "false"],
            description=(
                "true のときだけ、基準の値（LeRobot 0.4.2 の値）と違う EEPROM（P/I/D 等）を起動時に書く。"
                "Operating_Mode は書かない"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "probe_lock_sram", default_value="false", choices=["true", "false"],
            description="true で起動直後に Lock=1 のまま SRAM（Acceleration）へ書けるかを確認（W0 用）",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("db", default_value=False))

    def include(name, condition=None, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit_launch / name)),
            launch_arguments=(args or {}).items(),
            condition=condition,
        )

    # world -> base_link (SRDF の virtual_joint)
    ld.add_action(include("static_virtual_joint_tfs.launch.py"))

    # robot_state_publisher（1つだけ）。ros2_control_node と同じ robot_description を使う
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[moveit_config.robot_description, {"publish_frequency": 15.0}],
        )
    )

    # ros2_control_node（1つだけ。mock / 実機とも常に起動）
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, controllers_yaml],
            output="screen",
        )
    )

    # controller spawner（moveit_controllers.yaml のコントローラ名 + joint_state_broadcaster）
    ld.add_action(TimerAction(period=3.0, actions=[include("spawn_controllers.launch.py")]))

    ld.add_action(include("move_group.launch.py"))
    ld.add_action(
        include(
            "moveit_rviz.launch.py",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    ld.add_action(
        include(
            "warehouse_db.launch.py",
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # 宣言された引数の一覧（include した launch の引数を含む）。検査の関数が実行時に使う
    declared_names.extend(sorted({arg.name for arg in ld.get_launch_arguments()}))

    return ld
