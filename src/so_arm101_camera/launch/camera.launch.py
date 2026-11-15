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
Start the RealSense D415 (apt realsense2_camera) and, if wanted, the AprilTag detection.

The logic (argument check, extrinsics check, conversion of config files) lives in
so_arm101_camera/camera_config.py. This file only wires it to the launch actions.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from so_arm101_camera import camera_config as cc


def _start(context, share):
    # First action: nothing below starts before these checks have passed.
    cc.check_launch_arguments(context.argv)
    use_apriltag = cc.parse_bool_argument(
        'use_apriltag', context.launch_configurations['use_apriltag'])
    extrinsics_file = context.launch_configurations['extrinsics_file']
    # Raises if the file is not valid, or if a non-default path does not exist.
    tf_arguments = cc.plan_extrinsics_tf(
        extrinsics_file, share / 'config' / 'camera_extrinsics.yaml')
    realsense_args = cc.realsense_launch_arguments(share / 'config' / 'realsense.yaml')

    rs_launch = (Path(get_package_share_directory('realsense2_camera'))
                 / 'launch' / 'rs_launch.py')
    actions = [
        # forwarding=False: rs_launch.py warns about launch configurations it does not know
        # (use_apriltag, ...), so the ones of this file are not passed to it.
        GroupAction(
            [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rs_launch)),
                launch_arguments=realsense_args.items())],
            forwarding=False),
    ]
    if use_apriltag:
        actions.append(Node(
            package='apriltag_ros', executable='apriltag_node', name='apriltag',
            output='screen', remappings=list(cc.APRILTAG_REMAPPINGS),
            parameters=[str(share / 'config' / 'apriltag.yaml')]))
    if tf_arguments is None:
        actions.append(LogInfo(msg=cc.no_extrinsics_message(extrinsics_file)))
    else:
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_camera_link_tf', output='screen', arguments=tf_arguments))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('so_arm101_camera'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_apriltag', default_value='false', description='true: also start apriltag_node'),
        DeclareLaunchArgument(
            'extrinsics_file',
            default_value=str(share / 'config' / 'camera_extrinsics.yaml'),
            description='yaml of the base_link to camera_link transform (TF only if it exists)'),
        OpaqueFunction(function=_start, args=[share]),
    ])
