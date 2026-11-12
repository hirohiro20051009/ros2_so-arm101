"""
SO-ARM101 統合ランチャー（ルート名前空間）。

  mock:=true  (既定) : mock_components/GenericSystem でシミュレーション
  mock:=false        : 実機（hardware:=feetech）

  hardware:=feetech (既定。これだけが使える) : Sts3215System（STS3215 を直接制御。LeRobot を使わない）
                               read_only:=true (既定) は読み取り専用で、サーボのレジスタには書き込まない。
                               書き込みモード（トルク ON。腕が動く）は read_only:=false を明示したときだけ
                               （手順は docs/feetech_write_check.md）
  hardware:=feetech 以外（lerobot など）は、mock:=true / false のどちらでも、起動前にエラーで止まる
  （LeRobot 経路は削除した。タグ stage2c-done に残っている）。

robot_state_publisher / ros2_control_node / move_group は1つずつ、
コントローラ名・トピックは mock / 実機で共通（/arm_controller, /gripper_controller, /joint_states）。
"""

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


def _require_feetech(context, *args, **kwargs):
    """hardware:=feetech 以外は、はっきりしたエラーで止める（mock:=true のときも検査する）。"""
    value = context.perform_substitution(LaunchConfiguration("hardware"))
    if value != "feetech":
        raise RuntimeError(
            f"hardware:={value} is not supported. The LeRobot path (hardware:=lerobot) was "
            "removed from this workspace; use hardware:=feetech (the default). "
            "The removed code is in git tag stage2c-done "
            "(e.g. git checkout stage2c-done -- src/so_arm101_lerobot_bridge)."
        )
    return []


def generate_launch_description():
    mock = LaunchConfiguration("mock")
    hardware = LaunchConfiguration("hardware")
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
                "hardware": hardware,
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
        get_package_share_directory("so_arm101_controller"), "config", "controllers.yaml"
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "mock", default_value=True, description="true=mock / false=実機（hardware:=feetech）"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "hardware",
            default_value="feetech",
            description="feetech だけ受け付ける（STS3215 を直接制御。既定。read_only で読み取り専用/書き込みを選ぶ）。"
            "それ以外は、mock の値に関わらず、起動前にエラーで止まる。mock:=true のときは使われない",
        )
    )
    ld.add_action(DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"))
    ld.add_action(
        DeclareLaunchArgument(
            "calibration_file",
            default_value=os.path.join(
                get_package_share_directory("so_arm101_hardware"), "config", "follower_calibration.yaml"
            ),
            description="hardware:=feetech のキャリブレーション yaml",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "read_only", default_value="true", choices=["true", "false"],
            description="hardware:=feetech のみ。true=読み取り専用（既定）/ false=書き込みモード（トルク ON。腕が動く。"
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
            description="書き込みモードの Torque_Limit。shoulder_pan と wrist_roll だけに効く（lift / elbow / wrist_flex は常に 1000、"
            "gripper は min(値, 500)）。W1〜W2 の初回は 500",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "apply_servo_config", default_value="false", choices=["true", "false"],
            description="true のときだけ、LeRobot の値と違う EEPROM（P/I/D 等）を起動時に書く。Operating_Mode は書かない",
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

    # hardware の検査（宣言した引数の後、ノードの起動の前）。feetech 以外はここで止まる
    ld.add_action(OpaqueFunction(function=_require_feetech))

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
    ld.add_action(include("moveit_rviz.launch.py", condition=IfCondition(LaunchConfiguration("use_rviz"))))
    ld.add_action(include("warehouse_db.launch.py", condition=IfCondition(LaunchConfiguration("db"))))

    return ld
