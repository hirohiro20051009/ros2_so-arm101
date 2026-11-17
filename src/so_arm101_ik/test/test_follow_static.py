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

"""Static checks of the VR follow node, launch, parameters and installation (no node is run)."""

import ast
import dataclasses
import os
from pathlib import Path
import re
import types

import pytest
from so_arm101_ik import follow_calc as fc
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
NODE = PACKAGE / 'so_arm101_ik' / 'vr_follow_node.py'
LAUNCH = PACKAGE / 'launch' / 'vr_follow.launch.py'
PARAMS = PACKAGE / 'config' / 'vr_follow.yaml'
SCRIPT = PACKAGE / 'scripts' / 'vr_follow'

BUTTONS = {'button_clutch', 'button_open', 'button_close'}
TOPIC_PARAMETERS = {'joy_topic', 'pose_topic'}


def _text(path):
    return path.read_text()


def _constants(path):
    """Return {NAME: value} of the module level NAME = 'text' lines."""
    tree = ast.parse(_text(path))
    result = {}
    for node in tree.body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str)):
            result[node.targets[0].id] = node.value.value
    return result


def test_the_topic_names_are_the_ones_of_the_plan():
    constants = _constants(NODE)
    assert constants['JOY_TOPIC'] == '/vr/right/joy'
    assert constants['POSE_TOPIC'] == '/vr/right/pose'
    assert constants['TRAJECTORY_TOPIC'] == '/arm_controller/joint_trajectory'
    assert constants['TARGET_TOPIC'] == '/vr_follow/target_pose'
    assert constants['GRIPPER_TOPIC'] == '/vr_follow/gripper'
    assert constants['STATUS_TOPIC'] == '/vr_follow/status'
    assert constants['GRIPPER_ACTION'] == '/gripper_controller/gripper_cmd'
    assert constants['JOINT_STATES'] == '/joint_states'
    assert constants['CONTROLLER_STATE'] == '/arm_controller/controller_state'
    assert constants['BASE_FRAME'] == 'base_link'


def test_the_node_sends_to_the_arm_only_through_one_topic_publisher():
    text = _text(NODE)
    assert text.count('trajectory_pub.publish(') == 1
    for forbidden in ('FollowJointTrajectory', 'MoveGroup', 'ExecuteTrajectory', 'MoveIt',
                      'moveit_msgs'):
        assert forbidden not in text, forbidden
    assert text.count('send_goal_async(') == 1 and 'GripperCommand' in text


def test_the_point_is_sent_only_where_the_machine_says_so():
    text = _text(NODE)
    body = text[text.index('    def _tick(self):'):text.index('    def _publish_status')]
    assert 'if output.point is not None:' in body and 'self._send_point(output.point)' in body
    assert 'if output.gripper is not None:' in body
    assert text.count('self._send_point(') == 1 and text.count('self._send_gripper(') == 1


def test_the_gripper_goal_has_no_effort_and_the_stamp_of_the_point_is_zero():
    text = _text(NODE)
    assert 'goal.command.max_effort = 0.0' in text
    assert 'stamp 0' in text and 'header.stamp' not in text[
        text.index('def _send_point'):text.index('def _send_target')]


def test_the_machine_and_the_calculation_do_not_import_ros():
    for name in ('follow_calc.py', 'follow_machine.py'):
        text = _text(PACKAGE / 'so_arm101_ik' / name)
        for forbidden in ('rclpy', 'geometry_msgs', 'sensor_msgs', 'trajectory_msgs',
                          'import time', 'time.monotonic'):
            assert forbidden not in text, (name, forbidden)


def test_the_parameter_file_has_exactly_the_parameters_of_the_node():
    data = yaml.safe_load(_text(PARAMS))
    parameters = data['vr_follow']['ros__parameters']
    fields = {f.name for f in dataclasses.fields(fc.FollowConfig)}
    assert set(parameters) == fields | BUTTONS | TOPIC_PARAMETERS


def test_the_parameter_file_has_the_defaults_of_the_config():
    parameters = yaml.safe_load(_text(PARAMS))['vr_follow']['ros__parameters']
    defaults = fc.FollowConfig()
    for field in dataclasses.fields(fc.FollowConfig):
        assert list(_as_list(parameters[field.name])) == list(_as_list(
            getattr(defaults, field.name))), field.name
    assert parameters['joy_topic'] == '/vr/right/joy'
    assert parameters['pose_topic'] == '/vr/right/pose'


def _as_list(value):
    return value if isinstance(value, (list, tuple)) else [value]


def test_the_parameter_file_makes_a_valid_config():
    parameters = yaml.safe_load(_text(PARAMS))['vr_follow']['ros__parameters']
    values = {f.name: parameters[f.name] for f in dataclasses.fields(fc.FollowConfig)}
    values['input_rotation_rpy_deg'] = tuple(values['input_rotation_rpy_deg'])
    assert fc.FollowConfig(**values) == fc.FollowConfig()


def test_the_parameter_file_types_are_the_ones_the_node_declares():
    parameters = yaml.safe_load(_text(PARAMS))['vr_follow']['ros__parameters']
    for name in ('scale', 'rate_hz', 'point_time_s', 'max_joint_speed', 'jump_pos', 'max_lag'):
        assert isinstance(parameters[name], float), name  # a double parameter needs a float
    assert all(isinstance(parameters[name], int) for name in BUTTONS)
    assert isinstance(parameters['gripper_requires_clutch'], bool)
    assert parameters['clutch_mode'] in ('hold', 'toggle')
    assert len(parameters['input_rotation_rpy_deg']) == 3


def test_the_button_defaults_of_the_node_are_the_ones_of_the_file():
    parameters = yaml.safe_load(_text(PARAMS))['vr_follow']['ros__parameters']
    tree = ast.parse(_text(NODE))
    defaults = None
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == (
                'BUTTON_PARAMETERS'):
            defaults = ast.literal_eval(node.value)
    assert defaults == {name: parameters[name] for name in BUTTONS}


def test_the_launch_starts_only_the_node_and_does_not_include_bringup():
    text = _text(LAUNCH).split('"""', 2)[2]  # the code, not the docstring
    assert 'IncludeLaunchDescription' not in text and 'bringup' not in text
    assert text.count('Node(') == 1 and "executable='vr_follow'" in text
    assert "package='so_arm101_ik'" in text and "name='vr_follow'" in text
    for forbidden in ('serial_port', 'read_only', 'mock', 'calibration_file'):
        assert forbidden not in text.replace('mock:=true', ''), forbidden


def test_the_launch_overrides_are_parameters_of_the_file_and_empty_by_default():
    text = _text(LAUNCH)
    tree = ast.parse(text)
    overrides = None
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'OVERRIDES':
            overrides = [key.value for key in node.value.keys]
    parameters = yaml.safe_load(_text(PARAMS))['vr_follow']['ros__parameters']
    assert overrides and set(overrides) <= set(parameters)
    assert "default_value=''" in text


def test_the_script_runs_main_and_is_executable():
    text = _text(SCRIPT)
    assert text.startswith('#!/usr/bin/env python3')
    assert 'from so_arm101_ik.vr_follow_node import main' in text and 'main()' in text
    assert os.access(SCRIPT, os.X_OK)


def test_the_package_installs_the_node_the_launch_and_the_parameters():
    cmake = _text(PACKAGE / 'CMakeLists.txt')
    assert re.search(r'install\(PROGRAMS\s+scripts/vr_follow\s+DESTINATION lib/\$\{PROJECT_NAME\}',
                     cmake)
    assert re.search(r'install\(DIRECTORY\s+launch config\s+DESTINATION share/\$\{PROJECT_NAME\}',
                     cmake)
    for name in ('test_follow_calc', 'test_follow_machine', 'test_follow_static'):
        assert name in cmake
    package = _text(PACKAGE / 'package.xml')
    for dependency in ('rclpy', 'sensor_msgs', 'geometry_msgs', 'std_msgs', 'trajectory_msgs',
                       'control_msgs', 'action_msgs', 'tf2_ros_py', 'builtin_interfaces',
                       'rcl_interfaces', 'launch', 'launch_ros', 'python3-numpy'):
        assert '<exec_depend>%s</exec_depend>' % dependency in package, dependency


def test_the_node_does_not_touch_the_driver_or_the_other_packages():
    text = _text(NODE)
    for forbidden in ('so_arm101_hardware', 'so_arm101_calibration', 'so_arm101_camera',
                      'serial', 'subprocess'):
        assert forbidden not in text


# -------------------------------------------------- functions of the node module (needs rclpy)

def _node_module():
    pytest.importorskip('rclpy')
    pytest.importorskip('tf2_ros')
    from so_arm101_ik import vr_follow_node
    return vr_follow_node


def test_config_fields_are_the_fields_of_the_config():
    node = _node_module()
    assert node.config_fields() == [f.name for f in dataclasses.fields(fc.FollowConfig)]


def test_the_status_text():
    node = _node_module()
    from so_arm101_ik import follow_machine as fm
    detail = {'dist_mm': 12.34567, 'scale': 1.0, 'message': 'not in the text'}
    output = fm.Output('HOLD_RANGE', 'ELBOW_LIMIT', detail=detail)
    assert node.format_status(output) == 'HOLD_RANGE ELBOW_LIMIT dist_mm=12.35 scale=1'
    assert node.format_status(fm.Output('IDLE', 'REFUSED WINDUP,FLOOR')
                              ) == 'IDLE REFUSED_WINDUP,FLOOR'
    assert node.format_status(fm.Output('IDLE', '')) == 'IDLE -'


def test_a_quaternion_message_gives_a_rotation():
    node = _node_module()
    q = types.SimpleNamespace(x=0.0, y=0.0, z=0.7071067811865476, w=0.7071067811865476)
    assert abs(node.quaternion_message_to_matrix(q)[1, 0] - 1.0) < 1e-9
    with pytest.raises(ValueError):
        node.quaternion_message_to_matrix(types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0))
