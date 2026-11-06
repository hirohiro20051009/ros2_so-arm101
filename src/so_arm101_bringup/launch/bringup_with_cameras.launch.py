"""bringup.launch.py にカメラ (include/camera.launch.py) を追加したランチャー。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_pkg = get_package_share_directory('so_arm101_bringup')

    args = [
        DeclareLaunchArgument('mock', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('hardware', default_value='lerobot'),
    ]
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'mock': LaunchConfiguration('mock'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'hardware': LaunchConfiguration('hardware'),
        }.items(),
    )
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'include', 'camera.launch.py')
        ),
    )
    return LaunchDescription(args + [bringup, cameras])
