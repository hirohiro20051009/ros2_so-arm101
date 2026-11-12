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

"""Tests of the undeclared-launch-argument check of bringup.launch.py (no node is started)."""

import importlib.util
from pathlib import Path
import re

from launch import LaunchContext, LaunchDescription, LaunchDescriptionSource, LaunchService
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration,
)
from moveit_configs_utils import MoveItConfigsBuilder
import numpy as np
import pytest
import yaml

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'


def _load():
    spec = importlib.util.spec_from_file_location('bringup_launch_under_test', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MODULE = _load()
LD = MODULE.generate_launch_description()
DECLARED = sorted({a.name for a in LD.get_launch_arguments()})


def _check(argv):
    return MODULE._check_launch_arguments(LaunchContext(argv=argv), DECLARED)


def _message(argv):
    with pytest.raises(RuntimeError) as info:
        _check(argv)
    return str(info.value)


def test_check_is_the_first_action():
    first = LD.entities[0]
    assert isinstance(first, OpaqueFunction)
    assert not first.execute(LaunchContext(argv=['mock:=true']))
    with pytest.raises(RuntimeError, match='foo'):
        first.execute(LaunchContext(argv=['foo:=bar']))


def test_declared_arguments():
    assert len(DECLARED) == 21
    assert 'hardware' not in DECLARED
    for name in ('mock', 'read_only', 'torque_limit', 'use_rviz',
                 'monitor_dynamics', 'rviz_config'):
        assert name in DECLARED


def test_no_arguments():
    assert _check([]) == []


@pytest.mark.parametrize('name', DECLARED)
def test_every_declared_argument_is_accepted(name):
    assert _check([f'{name}:=x']) == []


@pytest.mark.parametrize('name', ['launch-prefix', 'launch-prefix-filter'])
def test_ros2launch_option_arguments_are_accepted(name):
    assert _check([f'{name}:=x']) == []


def test_undeclared_argument_is_reported_with_the_declared_list():
    message = _message(['mock:=true', 'foo:=bar'])
    assert 'foo' in message
    assert 'Declared arguments:' in message
    assert 'mock' in message and 'read_only' in message
    assert 'hardware' not in message


@pytest.mark.parametrize('value', ['feetech', 'lerobot'])
def test_hardware_argument_gets_the_removal_message(value):
    message = _message([f'hardware:={value}'])
    assert "The 'hardware' argument was removed" in message
    assert 'stage2c-done' in message
    assert 'Remove it and run again' in message


@pytest.mark.parametrize('typo, expected', [
    ('read_olny', 'read_only'),
    ('use_rvis', 'use_rviz'),
    ('Mock', 'mock'),
])
def test_typos_get_a_suggestion(typo, expected):
    message = _message([f'{typo}:=true'])
    assert f'{typo} (did you mean: ' in message
    assert expected in message.split('did you mean: ')[1].split('?)')[0]


def _run(argv):
    """Run the check inside a LaunchService, after launch-internal values were set."""
    inner = LaunchDescription([
        DeclareLaunchArgument('inner_arg', default_value='v'),
        OpaqueFunction(
            function=lambda context: [SetLaunchConfiguration('added_by_opaque', '1')]),
    ])
    description = LaunchDescription([
        OpaqueFunction(function=MODULE._check_launch_arguments, args=[DECLARED]),
        SetLaunchConfiguration('internal_x', '1'),
        IncludeLaunchDescription(
            LaunchDescriptionSource(inner), launch_arguments={'forwarded': '1'}.items()),
    ])
    service = LaunchService(argv=argv)
    service.include_launch_description(description)
    return service.run()


def test_launch_internal_values_are_not_reported():
    assert _run([]) == 0
    assert _run(['mock:=true', 'monitor_dynamics:=false']) == 0


def test_undeclared_argument_stops_the_launch_service():
    assert _run(['foo:=bar']) != 0


# ---------------------------------------------------------------------------------------------
# Static checks of the Servo (Diff-IK) settings (docs/servo_integration_plan.md, stage 1).
# No node is started. Files are only read; robot_description is expanded with xacro.
# ---------------------------------------------------------------------------------------------
BRINGUP_CONFIG = Path(__file__).resolve().parents[1] / 'config'
REPO = Path(__file__).resolve().parents[3]
MOVEIT_CONFIG = REPO / 'src' / 'so_arm101_moveit_config' / 'config'

# The 36 parameters declared by moveit_servo 2.5.10 (servo_parameters.cpp, tag 2.5.10).
SERVO_2_5_10_KEYS = {
    'use_gazebo', 'status_topic', 'cartesian_command_in_topic', 'joint_command_in_topic',
    'robot_link_command_frame', 'command_in_type', 'scale.linear', 'scale.rotational',
    'scale.joint', 'override_velocity_scaling_factor', 'command_out_topic', 'publish_period',
    'command_out_type', 'publish_joint_positions', 'publish_joint_velocities',
    'publish_joint_accelerations', 'joint_topic', 'smoothing_filter_plugin_name',
    'move_group_name', 'planning_frame', 'ee_frame_name', 'is_primary_planning_scene_monitor',
    'monitored_planning_scene_topic', 'incoming_command_timeout',
    'num_outgoing_halt_msgs_to_publish', 'halt_all_joints_in_joint_mode',
    'halt_all_joints_in_cartesian_mode', 'lower_singularity_threshold',
    'hard_stop_singularity_threshold', 'leaving_singularity_threshold_multiplier',
    'joint_limit_margin', 'low_latency_mode', 'check_collisions', 'collision_check_rate',
    'self_collision_proximity_threshold', 'scene_collision_proximity_threshold',
}
# The order of the active joints of the group 'arm' in MoveIt's RobotModel (checked with a
# standalone C++ program, docs/servo_integration_plan.md 3.1.2). Servo publishes in this order.
SERVO_ARM_ORDER = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
# Top-down grasp poses (lift, elbow, wrist_flex) for r = 0.07..0.22 m, z = 0.01..0.03 m
# (the approach axis is straight down; docs/servo_integration_plan.md 2.3, 3.5.1).
TOP_DOWN_POSES = [
    (-1.0645, 1.3002, 1.3351), (-0.6547, 1.1025, 1.1231), (-0.2585, 0.7856, 1.0437),
    (0.0673, 0.4137, 1.0898), (0.3912, -0.0459, 1.2255), (-0.7291, 0.9916, 1.3084),
]
TOP_DOWN_PANS = [0.0, 0.4, -1.0, 1.2]
FORWARD_TOPIC = '/forward_position_controller/commands'


def _yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _flatten(tree, prefix=''):
    for key, value in tree.items():
        if isinstance(value, dict):
            yield from _flatten(value, prefix + key + '.')
        else:
            yield prefix + key, value


def _load_servo_math():
    spec = importlib.util.spec_from_file_location(
        'servo_math_under_test', REPO / 'tools' / 'servo_math.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _robot_description(mappings=None):
    builder = MoveItConfigsBuilder('so101_new_calib', package_name='so_arm101_moveit_config')
    if mappings is not None:
        builder = builder.robot_description(mappings=mappings)
    return builder.to_moveit_configs().robot_description['robot_description']


SERVO = dict(_flatten(_yaml(BRINGUP_CONFIG / 'servo.yaml')))
SERVO_LIMITS = _yaml(BRINGUP_CONFIG / 'servo_joint_limits.yaml')['joint_limits']
CONTROLLERS = _yaml(BRINGUP_CONFIG / 'controllers.yaml')
SM = _load_servo_math()
CHAIN = SM.chain_from_urdf(_robot_description())
HW_MAPPINGS = {
    'use_fake_hardware': 'false', 'serial_port': '/dev/ttyACM0', 'calibration_file': '/x',
    'read_only': 'true', 'acceleration': '50', 'torque_limit': '1000',
    'apply_servo_config': 'false', 'probe_lock_sram': 'false',
}


def test_servo_yaml_keys_are_declared_by_servo_2_5_10():
    assert set(SERVO) <= SERVO_2_5_10_KEYS


def test_servo_output_goes_to_the_forward_controller_not_to_the_jtc():
    assert SERVO['command_out_type'] == 'std_msgs/Float64MultiArray'
    assert SERVO['command_out_topic'] == FORWARD_TOPIC
    assert SERVO['command_out_topic'] != '/arm_controller/joint_trajectory'
    # A Float64MultiArray carries positions OR velocities (servo_parameters.cpp:422).
    assert SERVO['publish_joint_positions'] is True
    assert SERVO['publish_joint_velocities'] is False


def test_servo_yaml_passes_the_validation_of_servo_2_5_10():
    assert SERVO['publish_period'] > 0
    assert SERVO['num_outgoing_halt_msgs_to_publish'] > 0
    assert SERVO['lower_singularity_threshold'] < SERVO['hard_stop_singularity_threshold']
    assert SERVO['leaving_singularity_threshold_multiplier'] >= 1.0
    assert SERVO['command_in_type'] in ('unitless', 'speed_units')
    assert SERVO['self_collision_proximity_threshold'] > 0
    assert SERVO['scene_collision_proximity_threshold'] > 0
    assert SERVO['collision_check_rate'] > 0
    assert SERVO['joint_limit_margin'] >= 0


def test_servo_singularity_thresholds_cover_the_top_down_poses():
    # The condition number of the top-down poses is 19..37 (the default 17 / 30 does not fit).
    assert SERVO['lower_singularity_threshold'] >= 37.0


def test_servo_initial_speed_limits():
    assert SERVO['command_in_type'] == 'unitless'
    assert SERVO['scale.linear'] <= 0.01
    assert SERVO['scale.rotational'] <= 0.05
    assert SERVO['scale.joint'] <= 0.05
    assert SERVO['incoming_command_timeout'] <= 0.1
    # >= 0.01 disables the velocity limits of servo_joint_limits.yaml (enforce_limits.cpp:80).
    assert SERVO['override_velocity_scaling_factor'] == 0.0


def test_servo_frames_and_entry_points():
    assert SERVO['planning_frame'] == 'base_link'
    assert SERVO['robot_link_command_frame'] == 'base_link'
    assert SERVO['ee_frame_name'] == 'gripper_frame_link'
    assert SERVO['move_group_name'] == 'arm'
    assert SERVO['cartesian_command_in_topic'] == '~/delta_twist_cmds'
    assert SERVO['joint_command_in_topic'] == '~/delta_joint_cmds'
    assert SERVO['status_topic'] == '~/status'
    assert SERVO['joint_topic'] == '/joint_states'
    assert SERVO['is_primary_planning_scene_monitor'] is False


def test_servo_joint_limits_cover_the_arm_and_are_slow():
    assert list(SERVO_LIMITS) == SERVO_ARM_ORDER
    urdf_velocity = {j['name']: j['velocity'] for j in CHAIN if j['type'] == 'revolute'}
    for name, limit in SERVO_LIMITS.items():
        assert limit['has_velocity_limits'] is True
        assert 0 < limit['max_velocity'] <= 0.3
        assert limit['max_velocity'] < urdf_velocity[name]


def test_joint_order_matches_between_servo_and_the_controllers():
    assert SM.active_joint_names(CHAIN) == SERVO_ARM_ORDER
    arm = CONTROLLERS['arm_controller']['ros__parameters']['joints']
    forward = CONTROLLERS['forward_position_controller']['ros__parameters']['joints']
    assert arm == SERVO_ARM_ORDER
    assert forward == SERVO_ARM_ORDER


def test_forward_controller_excludes_the_jtc_and_moveit_ignores_it():
    manager = CONTROLLERS['controller_manager']['ros__parameters']
    assert manager['forward_position_controller']['type'] == (
        'forward_command_controller/ForwardCommandController')
    forward = CONTROLLERS['forward_position_controller']['ros__parameters']
    arm = CONTROLLERS['arm_controller']['ros__parameters']
    assert forward['interface_name'] == 'position'
    assert 'position' in arm['command_interfaces']
    assert forward['joints'] == arm['joints']  # same command interfaces: ros2_control excludes
    moveit = _yaml(MOVEIT_CONFIG / 'moveit_controllers.yaml')['moveit_simple_controller_manager']
    assert moveit['controller_names'] == ['arm_controller', 'gripper_controller']


def test_driver_step_limit_is_passed_only_to_the_real_hardware():
    real = _robot_description(HW_MAPPINGS)
    mock = _robot_description(dict(HW_MAPPINGS, use_fake_hardware='true'))
    assert re.search(r'<param name="max_joint_speed">0.75</param>', real)
    assert re.search(r'<param name="limit_window_s">0.08</param>', real)
    assert 'max_joint_speed' not in mock
    assert 'limit_window_s' not in mock
    assert real == _robot_description(dict(HW_MAPPINGS, max_joint_speed='0.75'))


def test_driver_step_limit_does_not_clip_moveit_at_the_default_scaling():
    real = _robot_description(HW_MAPPINGS)
    speed = float(re.search(r'name="max_joint_speed">([0-9.]+)<', real).group(1))
    window = float(re.search(r'name="limit_window_s">([0-9.]+)<', real).group(1))
    limits = _yaml(MOVEIT_CONFIG / 'joint_limits.yaml')
    max_velocity = max(v['max_velocity'] for v in limits['joint_limits'].values())
    scaling = limits['default_velocity_scaling_factor']
    update_rate = CONTROLLERS['arm_controller']['ros__parameters']['update_rate']
    worst = 1.2 * max_velocity * scaling * 2.0 / update_rate  # 2 JTC updates, 1.2 margin
    assert speed * window >= worst
    assert speed * window <= 0.1  # keep the last guard tight (the design value is 0.06 rad)


def test_drift_rule_with_hysteresis():
    assert SM.choose_drift(0.0) == 'rx'
    assert SM.choose_drift(1.2) == 'rx'
    assert SM.choose_drift(-1.2) == 'rx'
    assert SM.choose_drift(1.3) == 'ry'
    assert SM.choose_drift(1.22, current='rx') == 'rx'
    assert SM.choose_drift(1.26, current='rx') == 'ry'
    assert SM.choose_drift(1.18, current='ry') == 'ry'
    assert SM.choose_drift(1.14, current='ry') == 'rx'
    assert SM.drift_flags('rx') == [False, False, False, True, False, False]
    assert SM.drift_flags('ry') == [False, False, False, False, True, False]


def _top_down_q(pose, pan):
    return [pan, pose[0], pose[1], pose[2], 0.0]


def test_top_down_poses_point_straight_down():
    for pose in TOP_DOWN_POSES:
        _, _, rotation = SM.jacobian(CHAIN, _top_down_q(pose, 0.4))
        assert rotation[2, 2] < -0.999  # the approach axis (gripper z) is straight down


def test_unreachable_direction_is_a_rotation_about_the_pan_dependent_axis():
    for pose in TOP_DOWN_POSES:
        for pan in TOP_DOWN_PANS:
            jac, _, _ = SM.jacobian(CHAIN, _top_down_q(pose, pan))
            u = SM.left_null(jac)
            assert np.linalg.norm(u[:3]) < 1e-3  # a pure rotation
            axis = np.array([np.cos(pan), -np.sin(pan), 0.0])
            assert abs(u[3:] @ axis) > 0.999


def test_projected_twist_is_reproduced_exactly_after_dropping_rx_or_ry():
    twist = np.array([0.3, -0.5, 0.2, 0.4, -0.3, 0.6])
    for pose in TOP_DOWN_POSES:
        for pan in TOP_DOWN_PANS:
            jac, _, _ = SM.jacobian(CHAIN, _top_down_q(pose, pan))
            projected = SM.project_twist(jac, twist)
            assert abs(SM.left_null(jac) @ projected) < 1e-12
            for row in ('rx', 'ry'):
                if SM.reduced_condition(jac, row) > 1e3:
                    continue  # ry is singular at pan = 0 (not used there)
                index = SM.ROWS[row]
                reduced = np.delete(jac, index, axis=0)
                qdot = np.linalg.solve(reduced, np.delete(projected, index))
                assert np.allclose(jac @ qdot, projected, atol=1e-9)


def test_reduced_jacobian_is_well_conditioned_with_the_chosen_drift():
    for pose in TOP_DOWN_POSES:
        for pan in TOP_DOWN_PANS + [1.4, -1.5]:
            jac, _, _ = SM.jacobian(CHAIN, _top_down_q(pose, pan))
            row = SM.choose_drift(pan)
            assert SM.reduced_condition(jac, row) < 100.0


def test_dropping_yaw_is_singular_for_the_top_down_poses():
    for pose in TOP_DOWN_POSES:
        jac, _, _ = SM.jacobian(CHAIN, _top_down_q(pose, 0.4))
        assert SM.reduced_condition(jac, 'rz') > 1e4
