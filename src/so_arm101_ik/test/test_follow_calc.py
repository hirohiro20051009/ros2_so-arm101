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

"""The calculation of the VR follow node (follow_calc.py): no ROS, no clock."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik import follow_calc as fc
from so_arm101_ik.urdf_kinematics import UrdfChain

# poses [pan, lift, elbow, wrist_flex, roll] (rad) well above the floor (z about 0.08 to 0.19 m)
POSES = {
    'stretched': [0.0, 0.3, -0.3, 0.2, 0.0],
    'pan and roll': [0.4, 0.2, -0.2, 0.3, 0.5],
    'elbow positive': [-0.5, 0.0, 0.4, -0.2, -0.6],
    'low wrist': [0.2, -0.3, 0.6, 0.5, 1.0],
}


def _urdf_xml():
    share = Path(get_package_share_directory('so_arm101_description'))
    return (share / 'urdf' / 'so101_new_calib.urdf').read_text()


def _exact_urdf_xml():
    xml = _urdf_xml()
    return xml.replace('1.5708', repr(math.pi / 2.0)).replace('3.14159', repr(math.pi))


def _random_poses(count=12):
    rng = np.random.default_rng(1)
    return [rng.uniform(ik.Q_MIN, ik.Q_MAX) for _ in range(count)]


# ------------------------------------------------------------------------------ rotations

def test_rotation_helpers_are_rotations():
    for r in (fc.rot_x(0.7), fc.rot_y(-1.1), fc.rot_z(2.0)):
        assert np.allclose(r @ r.T, np.eye(3)) and np.isclose(np.linalg.det(r), 1.0)
    assert np.allclose(fc.rot_z(math.pi / 2.0) @ [1, 0, 0], [0, 1, 0])
    assert np.allclose(fc.rot_x(math.pi / 2.0) @ [0, 1, 0], [0, 0, 1])
    assert np.allclose(fc.rot_y(math.pi / 2.0) @ [0, 0, 1], [1, 0, 0])


def test_tip_rotation_matches_the_urdf_chain_exactly():
    chain = UrdfChain(_exact_urdf_xml())
    for q in list(map(np.array, POSES.values())) + _random_poses() + [np.zeros(5)]:
        expected = chain.fk(q)[1]
        assert np.allclose(fc.tip_rotation(q), expected, atol=1e-9), q


def test_tip_rotation_matches_the_installed_urdf():
    chain = UrdfChain(_urdf_xml())
    for q in _random_poses():
        assert fc.rotation_angle(chain.fk(q)[1].T @ fc.tip_rotation(q)) < 1e-4


def test_tip_rotation_is_a_rotation():
    for q in _random_poses():
        r = fc.tip_rotation(q)
        assert np.allclose(r @ r.T, np.eye(3)) and np.isclose(np.linalg.det(r), 1.0)


def test_rotation_angle():
    assert fc.rotation_angle(np.eye(3)) == pytest.approx(0.0, abs=1e-7)
    assert fc.rotation_angle(fc.rot_x(0.4)) == pytest.approx(0.4)
    assert fc.rotation_angle(fc.rot_z(-2.5)) == pytest.approx(2.5)
    assert fc.rotation_angle(fc.rot_y(math.pi)) == pytest.approx(math.pi)


def test_quaternion_roundtrip():
    for r in (np.eye(3), fc.rot_x(0.5) @ fc.rot_z(1.2), fc.rot_y(3.0), fc.rot_z(math.pi),
              fc.tip_rotation(POSES['pan and roll'])):
        x, y, z, w = fc.matrix_to_quaternion(r)
        assert w >= 0.0
        assert np.allclose(fc.quaternion_to_matrix(x, y, z, w), r, atol=1e-9)


def test_quaternion_known_values():
    assert np.allclose(fc.quaternion_to_matrix(0, 0, 0, 1), np.eye(3))
    s = math.sqrt(0.5)
    assert np.allclose(fc.quaternion_to_matrix(0, 0, s, s), fc.rot_z(math.pi / 2.0))
    assert np.allclose(fc.quaternion_to_matrix(0, 0, 0, 2.0 - 1.0), np.eye(3))


@pytest.mark.parametrize('quaternion', [(0, 0, 0, 0), (0, 0, 0, 3.0), (math.nan, 0, 0, 1),
                                        (0, 0, math.inf, 1)])
def test_quaternion_rejects_bad_values(quaternion):
    with pytest.raises(ValueError):
        fc.quaternion_to_matrix(*quaternion)


# ----------------------------------------------------------------------------- projection

def test_projection_returns_what_the_arm_made():
    for q in list(map(np.array, POSES.values())) + _random_poses():
        flex_sum, q5 = fc.project_orientation(fc.tip_rotation(q), -q[0], q[1] + q[2] + q[3],
                                              q[4])
        assert flex_sum == pytest.approx(q[1] + q[2] + q[3], abs=1e-9), q
        assert q5 == pytest.approx(q[4], abs=1e-9), q


def test_projection_takes_the_angles_nearest_to_the_reference():
    q = np.array(POSES['pan and roll'])
    flex = q[1] + q[2] + q[3]
    flex_sum, q5 = fc.project_orientation(fc.tip_rotation(q), -q[0], flex + 2.0 * math.pi - 0.1,
                                          q[4] - 2.0 * math.pi + 0.1)
    assert flex_sum == pytest.approx(flex + 2.0 * math.pi, abs=1e-9)
    assert q5 == pytest.approx(q[4] - 2.0 * math.pi, abs=1e-9)


def test_projection_refuses_a_tool_axis_perpendicular_to_the_pan_plane():
    theta = 0.3
    r = fc.rot_z(theta) @ fc._M1 @ np.eye(3) @ fc._RY_PI
    with pytest.raises(fc.TargetError) as error:
        fc.project_orientation(r, theta, 0.0, 0.0)
    assert error.value.reason == 'ORIENT'


def test_projection_angle_is_zero_for_a_made_orientation():
    q = np.array(POSES['low wrist'])
    assert fc.projection_angle(fc.tip_rotation(q), q) == pytest.approx(0.0, abs=1e-6)
    r = fc.rot_z(0.2) @ fc.tip_rotation(q)
    assert 0.0 < fc.projection_angle(r, q) < 0.5


# --------------------------------------------------------------------------- the IK solve

@pytest.mark.parametrize('name', sorted(POSES))
def test_solve_target_returns_q_from_its_own_pose(name):
    q = np.array(POSES[name])
    p = ik.fk(q)[0]
    config = fc.FollowConfig()
    solved = fc.solve_target(p, fc.tip_rotation(q), q + 0.03, config)
    assert np.allclose(solved, q, atol=1e-6), (solved, q)


def test_solve_target_follows_a_small_move_without_changing_the_orientation():
    q = np.array(POSES['pan and roll'])
    p = ik.fk(q)[0] + np.array([0.01, -0.01, 0.01])
    r = fc.tip_rotation(q)
    solved = fc.solve_target(p, r, q, fc.FollowConfig())
    assert np.allclose(ik.fk(solved)[0], p, atol=1e-6)
    assert fc.rotation_angle(fc.tip_rotation(solved).T @ r) < 0.02


def test_solve_target_takes_the_branch_nearest_to_the_previous_solution():
    q = np.array(POSES['stretched'])  # elbow negative
    p, pitch, roll = ik.fk(q)
    solved = fc.solve_target(p, fc.tip_rotation(q), q, fc.FollowConfig())
    assert solved[2] < 0.0
    assert np.allclose(solved, q, atol=1e-6)


def test_solve_target_projects_an_orientation_the_arm_cannot_make():
    q = np.array(POSES['stretched'])
    p = ik.fk(q)[0]
    r = fc.rot_z(math.radians(10.0)) @ fc.tip_rotation(q)  # a yaw: only the pan can make it
    solved = fc.solve_target(p, r, q, fc.FollowConfig())
    assert np.allclose(ik.fk(solved)[0], p, atol=1e-6)
    assert fc.rotation_angle(fc.tip_rotation(solved).T @ r) < math.radians(20.0)


def test_solve_target_errors_have_reasons():
    q = np.array(POSES['stretched'])
    r = fc.tip_rotation(q)
    config = fc.FollowConfig()
    with pytest.raises(fc.TargetError) as error:
        fc.solve_target([0.2, 0.0, 0.040], r, q, config)
    assert error.value.reason == 'FLOOR'
    with pytest.raises(fc.TargetError) as error:
        fc.solve_target([1.0, 0.0, 0.2], r, q, config)
    assert error.value.reason == 'UNREACHABLE'
    with pytest.raises(fc.TargetError) as error:
        fc.solve_target([0.2, math.nan, 0.2], r, q, config)
    assert error.value.reason == 'UNREACHABLE'
    q_roll = np.array([0.0, 0.3, -0.3, 0.2, 3.0])  # the roll is beyond its limit (2.84 rad)
    with pytest.raises(fc.TargetError) as error:
        fc.solve_target(ik.fk(q_roll)[0], fc.tip_rotation(q_roll), q, config)
    assert error.value.reason == 'JOINT_RANGE'


def test_solve_target_stops_the_elbow_at_the_configured_angle():
    q = np.array([0.0, -1.5, 1.62, 0.2, 0.0])  # the elbow is at 92.8 deg (the URDF allows 96.8)
    p, r = ik.fk(q)[0], fc.tip_rotation(q)
    assert p[2] > 0.06
    with pytest.raises(fc.TargetError) as error:
        fc.solve_target(p, r, q, fc.FollowConfig())
    assert error.value.reason == 'ELBOW_LIMIT'
    solved = fc.solve_target(p, r, q, fc.FollowConfig(elbow_max_deg=95.0))
    assert solved[2] == pytest.approx(1.62, abs=1e-6)


# ---------------------------------------------------------------------------- the target

def test_relative_target_without_a_move_is_the_reference():
    p_ref, r_ref = np.array([0.3, 0.1, 0.2]), fc.rot_y(0.4)
    p_in0, r_in0 = np.array([1.0, 2.0, 3.0]), fc.rot_z(0.7)
    p, r = fc.relative_target(p_ref, r_ref, p_in0, r_in0, p_in0, r_in0, 1.0)
    assert np.allclose(p, p_ref) and np.allclose(r, r_ref)


def test_relative_target_scales_the_position_only():
    p_ref, r_ref = np.array([0.3, 0.1, 0.2]), np.eye(3)
    p_in0 = np.array([1.0, 2.0, 3.0])
    for scale in (0.5, 1.0, 2.0):
        p, r = fc.relative_target(p_ref, r_ref, p_in0, np.eye(3), p_in0 + [0.01, -0.02, 0.03],
                                  fc.rot_x(0.3), scale)
        assert np.allclose(p, p_ref + scale * np.array([0.01, -0.02, 0.03]))
        assert fc.rotation_angle(r) == pytest.approx(0.3)  # the rotation is not scaled


def test_relative_target_rotation_is_a_difference_in_base_link():
    r_ref, r_in0 = fc.rot_y(0.5), fc.rot_z(0.9)
    r_in = fc.rot_x(0.2) @ r_in0
    _, r = fc.relative_target(np.zeros(3), r_ref, np.zeros(3), r_in0, np.zeros(3), r_in, 1.0)
    assert np.allclose(r, fc.rot_x(0.2) @ r_ref)


def test_relative_target_with_the_input_rotation():
    r_fix = fc.FollowConfig(input_rotation_rpy_deg=(0.0, 0.0, 90.0)).r_fix
    p, r = fc.relative_target(np.zeros(3), np.eye(3), np.zeros(3), np.eye(3),
                              np.array([0.1, 0.0, 0.0]), fc.rot_z(0.3), 1.0, r_fix)
    assert np.allclose(p, [0.0, 0.1, 0.0], atol=1e-12)  # the x of the VR is the y of base_link
    assert np.allclose(r, fc.rot_z(0.3))  # a rotation about z stays about z
    _, r = fc.relative_target(np.zeros(3), np.eye(3), np.zeros(3), np.eye(3), np.zeros(3),
                              fc.rot_x(0.3), 1.0, r_fix)
    assert np.allclose(r, fc.rot_y(0.3), atol=1e-12)  # the x axis of the VR is the y of base_link


def test_target_difference_and_the_two_judgements():
    config = fc.FollowConfig()
    distance, angle = fc.target_difference(np.zeros(3), np.eye(3), [0.03, 0.04, 0.0],
                                           fc.rot_z(0.2))
    assert distance == pytest.approx(0.05) and angle == pytest.approx(0.2)
    assert fc.is_jump(0.05, 0.0, config) and not fc.is_jump(0.0499, 0.0, config)
    assert fc.is_jump(0.0, math.radians(30.0), config)
    assert not fc.is_jump(0.0, math.radians(29.9), config)
    assert fc.is_near(0.02, math.radians(10.0), config)
    assert not fc.is_near(0.0201, 0.0, config) and not fc.is_near(0.0, math.radians(10.1), config)
    assert not fc.is_near(0.0, math.radians(30.0), config)


# ------------------------------------------------------------------------ speed limit

def test_limit_step_passes_a_small_step():
    q, limited = fc.limit_step([0.0] * 5, [0.01, -0.02, 0.0, 0.0, 0.01], 0.05, 0.5)
    assert not limited and np.allclose(q, [0.01, -0.02, 0.0, 0.0, 0.01])


def test_limit_step_shortens_the_whole_step_by_one_ratio():
    q_prev = np.array([0.1, 0.2, 0.3, 0.4, 0.5])
    goal = q_prev + np.array([0.2, -0.1, 0.0, 0.05, -0.4])
    q, limited = fc.limit_step(q_prev, goal, 0.05, 0.5)
    assert limited
    step = q - q_prev
    assert np.max(np.abs(step)) == pytest.approx(0.025)  # the largest joint is at the limit
    ratio = 0.025 / 0.4
    assert np.allclose(step, (goal - q_prev) * ratio)  # the direction is kept
    assert np.all(np.abs(step) <= 0.025 + 1e-12)


def test_limit_step_exactly_at_the_limit_is_not_limited():
    _, limited = fc.limit_step([0.0] * 5, [0.025, 0, 0, 0, 0], 0.05, 0.5)
    assert not limited


def test_limit_step_does_not_modify_its_arguments():
    q_prev, goal = np.zeros(5), np.ones(5)
    fc.limit_step(q_prev, goal, 0.05, 0.5)
    assert np.all(q_prev == 0.0) and np.all(goal == 1.0)


# ------------------------------------------------------------------------- configuration

def test_config_defaults_are_the_values_of_the_plan():
    c = fc.FollowConfig()
    assert (c.scale, c.rate_hz, c.point_time_s, c.max_joint_speed) == (1.0, 10.0, 0.08, 0.5)
    assert (c.jump_pos, c.jump_ang_deg, c.resume_pos, c.resume_ang_deg) == (0.05, 30.0, 0.02, 10.0)
    assert (c.stale_s, c.joy_timeout_s, c.floor_z, c.elbow_max_deg) == (0.5, 1.0, 0.045, 91.0)
    assert (c.clutch_windup, c.ik_jump, c.max_lag, c.max_tracking_error) == (0.06, 0.35, 0.15, 0.2)
    assert c.clutch_mode == 'hold' and not c.gripper_requires_clutch
    assert (c.gripper_open_rad, c.gripper_closed_rad) == (1.0, 0.05)
    assert c.period == pytest.approx(0.1)


def test_config_q_max_cuts_only_the_elbow():
    q_max = fc.FollowConfig().q_max
    assert q_max[2] == pytest.approx(math.radians(91.0))
    assert np.allclose(np.delete(q_max, 2), np.delete(ik.Q_MAX, 2))
    assert ik.Q_MAX[2] == 1.69  # the module value is not modified


@pytest.mark.parametrize('change', [
    {'scale': 0.0}, {'scale': -1.0}, {'scale': 3.1}, {'scale': math.nan}, {'scale': True},
    {'rate_hz': 0.5}, {'rate_hz': 101.0}, {'max_joint_speed': 0.0}, {'max_joint_speed': 1.1},
    {'point_time_s': 0.0}, {'point_time_s': 0.6}, {'point_time_s': math.nan},
    {'jump_pos': 0.01}, {'jump_pos': 0.3}, {'resume_pos': 0.0}, {'resume_ang_deg': 40.0},
    {'jump_ang_deg': 100.0}, {'stale_s': 0.0}, {'stale_s': 6.0}, {'joy_timeout_s': -1.0},
    {'floor_z': 0.009}, {'floor_z': -0.1}, {'elbow_max_deg': 0.0}, {'elbow_max_deg': 97.0},
    {'clutch_windup': 0.0}, {'clutch_windup': 0.09}, {'ik_jump': -0.1}, {'ik_jump': 1.5},
    {'max_lag': -0.1}, {'max_lag': 0.6}, {'max_tracking_error': 1.5},
    {'clutch_mode': 'press'}, {'gripper_requires_clutch': 1},
    {'gripper_open_rad': 2.0}, {'gripper_closed_rad': -0.1},
    {'gripper_open_rad': 0.5, 'gripper_closed_rad': 0.55},
    {'input_rotation_rpy_deg': (0.0, 0.0)}, {'input_rotation_rpy_deg': (0.0, 0.0, math.nan)},
    {'input_rotation_rpy_deg': (0.0, 0.0, 400.0)}])
def test_config_refuses_bad_values(change):
    with pytest.raises(ValueError):
        fc.FollowConfig(**change)


@pytest.mark.parametrize('change', [
    {'ik_jump': 0.0}, {'max_lag': 0.0}, {'max_tracking_error': 0.0}, {'joy_timeout_s': 0.0},
    {'floor_z': 0.02}, {'floor_z': 0.01},
    {'clutch_mode': 'toggle'}, {'scale': 3.0}, {'input_rotation_rpy_deg': [10, 20, 30]}])
def test_config_accepts_the_values_that_disable_a_check_and_the_edges(change):
    fc.FollowConfig(**change)


def test_config_replace_checks_again():
    c = fc.FollowConfig()
    assert c.replace(scale=0.5).scale == 0.5 and c.scale == 1.0
    with pytest.raises(ValueError):
        c.replace(scale=0.0)


def test_input_rotation_matrix():
    assert np.allclose(fc.FollowConfig().r_fix, np.eye(3))
    r = fc.FollowConfig(input_rotation_rpy_deg=(0.0, 0.0, 90.0)).r_fix
    assert np.allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-12)


# --------------------------------------------------------------------------- the gripper

def test_gripper_joint_is_linear_between_the_open_and_the_closed_value():
    c = fc.FollowConfig()
    assert fc.gripper_joint(0.0, c) == pytest.approx(1.0)
    assert fc.gripper_joint(1.0, c) == pytest.approx(0.05)
    assert fc.gripper_joint(0.5, c) == pytest.approx(0.525)
    c2 = fc.FollowConfig(gripper_open_rad=0.2, gripper_closed_rad=1.5)  # the other direction
    assert fc.gripper_joint(0.0, c2) == pytest.approx(0.2)
    assert fc.gripper_joint(0.25, c2) == pytest.approx(0.525)


@pytest.mark.parametrize('amount', [-0.01, 1.01, math.nan, math.inf, True, None, '0.5'])
def test_gripper_joint_refuses_a_bad_amount(amount):
    with pytest.raises(ValueError):
        fc.gripper_joint(amount, fc.FollowConfig())


def test_rising_edge():
    assert fc.rising(False, True)
    assert not fc.rising(True, True) and not fc.rising(True, False)
    assert not fc.rising(False, False)
    assert not fc.rising(None, True)  # not known: no edge
