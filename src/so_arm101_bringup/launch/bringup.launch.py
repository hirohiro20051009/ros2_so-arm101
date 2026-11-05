"""
SO-ARM101 統合ランチャー（ルート名前空間）。

  mock:=true  (既定) : mock_components/GenericSystem でシミュレーション
  mock:=false        : 実機。So101HardwareBridge + LeRobot bridge (so_arm101_lerobot_bridge) を起動

どちらのモードも robot_state_publisher / ros2_control_node / move_group は1つずつ、
コントローラ名・トピックは共通（/arm_controller, /gripper_controller, /joint_states）。
実機モードで追加されるのは LeRobot bridge ノードだけ（/joint_commands, /joint_states_raw）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    mock = LaunchConfiguration("mock")

    # robot_description は全ノードで同一（mock で ros2_control のプラグインだけが切り替わる）
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so_arm101_moveit_config")
        .robot_description(mappings={"use_fake_hardware": mock})
        .to_moveit_configs()
    )
    moveit_launch = moveit_config.package_path / "launch"
    controllers_yaml = os.path.join(
        get_package_share_directory("so_arm101_controller"), "config", "controllers.yaml"
    )
    bridge_launch = os.path.join(
        get_package_share_directory("so_arm101_lerobot_bridge"),
        "launch",
        "so101_ros2_bridge.launch.py",
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "mock", default_value=True, description="true=mock / false=実機(LeRobot bridge 経由)"
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

    # robot_state_publisher（1つだけ）
    ld.add_action(include("rsp.launch.py", args={"use_fake_hardware": mock}))

    # ros2_control_node（1つだけ。mock / 実機とも常に起動）
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, controllers_yaml],
            output="screen",
        )
    )

    # 実機のみ: LeRobot bridge（サーボ I/O）。ルート名前空間
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            launch_arguments={"type": "follower"}.items(),
            condition=UnlessCondition(mock),
        )
    )

    # controller spawner（moveit_controllers.yaml のコントローラ名 + joint_state_broadcaster）
    ld.add_action(TimerAction(period=3.0, actions=[include("spawn_controllers.launch.py")]))

    ld.add_action(include("move_group.launch.py"))
    ld.add_action(include("moveit_rviz.launch.py", condition=IfCondition(LaunchConfiguration("use_rviz"))))
    ld.add_action(include("warehouse_db.launch.py", condition=IfCondition(LaunchConfiguration("db"))))

    return ld
