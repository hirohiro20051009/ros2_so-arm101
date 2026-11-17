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
Launch the VR follow node (P6).

Start it in addition to the arm: ros2 launch so_arm101_bringup bringup.launch.py (mock:=true or
the real arm) first, then this file. It does not include bringup.launch.py and starts only the
node. The values are in config/vr_follow.yaml (params_file:=... uses another file); the arguments
below override a few of them when they are given (empty: the file decides).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# argument -> type of the parameter that it overrides
OVERRIDES = {
    'scale': float, 'rate_hz': float, 'point_time_s': float, 'clutch_mode': str,
    'max_tracking_error': float, 'gripper_open_rad': float, 'gripper_closed_rad': float,
    'floor_z': float,
    'button_clutch': int, 'button_open': int, 'button_close': int,
}


def _node(context, *args, **kwargs):
    parameters = [LaunchConfiguration('params_file').perform(context)]
    overrides = {}
    for name, kind in OVERRIDES.items():
        text = LaunchConfiguration(name).perform(context).strip()
        if text:
            overrides[name] = kind(text)
    if overrides:
        parameters.append(overrides)
    return [Node(package='so_arm101_ik', executable='vr_follow', name='vr_follow',
                 output='screen', parameters=parameters)]


def generate_launch_description():
    default_file = os.path.join(get_package_share_directory('so_arm101_ik'), 'config',
                                'vr_follow.yaml')
    actions = [DeclareLaunchArgument('params_file', default_value=default_file,
                                     description='parameter file of the node')]
    for name in OVERRIDES:
        actions.append(DeclareLaunchArgument(
            name, default_value='', description='overrides %s of the file when not empty' % name))
    actions.append(OpaqueFunction(function=_node))
    return LaunchDescription(actions)
