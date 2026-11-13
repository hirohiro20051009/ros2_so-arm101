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

"""The URDF chain (urdf_kinematics) and the closed-form forward kinematics (analytic_ik)."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_ik import analytic_ik
from so_arm101_ik.urdf_kinematics import (
    ARM_JOINTS, axis_angle_matrix, derive_constants, rpy_matrix, UrdfChain)

INSTALLED_TOLERANCE_MM = 0.01  # installed URDF (angles rounded to 1.5708 / 3.14159) vs the model
EXACT_TOLERANCE_MM = 1e-5  # URDF with pi/2 and pi vs the model derived from it
TF_TOLERANCE_MM = 1.0  # the recorded tf of the real arm has a 1 mm resolution

# named poses [pan, lift, elbow, wrist_flex, roll] (rad)
NAMED_POSES = {
    'zero': [0.0, 0.0, 0.0, 0.0, 0.0],
    'start (C3)': [-0.09820, -1.48295, 1.69009, 0.50327, -0.04833],
    'T (W1)': [0.0031, -0.3813, 0.5501, 1.4454, -0.0069],
    'T (target)': [0.0, math.radians(-22), math.radians(30), math.radians(83), 0.0],
    'home': [0.0, -1.57, 1.57, 0.8, 0.0],
    'stretched': [0.0, 0.3, -0.3, 0.2, 0.0],
}


def _urdf_xml():
    share = Path(get_package_share_directory('so_arm101_description'))
    return (share / 'urdf' / 'so101_new_calib.urdf').read_text()


def _exact_urdf_xml():
    """Return the URDF with the rounded angles replaced by pi/2 and pi (the text is replaced)."""
    xml = _urdf_xml()
    assert '1.5708' in xml and '3.14159' in xml
    return xml.replace('1.5708', repr(math.pi / 2.0)).replace('3.14159', repr(math.pi))


def _random_poses(count=200):
    rng = np.random.default_rng(20261113)
    return [rng.uniform(analytic_ik.Q_MIN, analytic_ik.Q_MAX) for _ in range(count)]


def _roll_sweep():
    base = NAMED_POSES['T (W1)']
    return [base[:4] + [roll] for roll in np.arange(-2.5, 2.5001, 0.25)]


def _worst_difference_mm(chain, constants, poses):
    worst, where = 0.0, None
    for q in poses:
        diff = np.linalg.norm(chain.fk(q)[0] - analytic_ik.fk(q, constants)[0]) * 1000.0
        if diff > worst:
            worst, where = diff, q
    return worst, where


def test_rotation_helpers():
    r = rpy_matrix(0.3, -0.5, 1.1)
    assert np.allclose(r @ r.T, np.eye(3)) and abs(np.linalg.det(r) - 1.0) < 1e-12
    assert np.allclose(rpy_matrix(0.0, 0.0, math.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    # URDF convention R = Rz(yaw) Ry(pitch) Rx(roll)
    expected = (axis_angle_matrix([0, 0, 1], 1.1) @ axis_angle_matrix([0, 1, 0], -0.5)
                @ axis_angle_matrix([1, 0, 0], 0.3))
    assert np.allclose(r, expected, atol=1e-12)
    assert np.allclose(axis_angle_matrix([0, 0, 2], math.pi / 2) @ [1, 0, 0], [0, 1, 0],
                       atol=1e-12)
    assert np.allclose(axis_angle_matrix([1, 1, 0], 0.4) @ axis_angle_matrix([1, 1, 0], -0.4),
                       np.eye(3), atol=1e-12)


def test_chain_structure():
    chain = UrdfChain(_urdf_xml())
    assert tuple(j.name for j in chain.joints if j.moving) == ARM_JOINTS
    assert chain.joints[-1].name == 'gripper_frame_joint' and not chain.joints[-1].moving
    position, rotation = chain.fk(np.zeros(5))
    assert position.shape == (3,) and np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    with pytest.raises(ValueError):
        chain.fk(np.zeros(4))
    with pytest.raises(ValueError):
        UrdfChain(_urdf_xml(), tip_link='no_such_link')


def test_pan_rotates_the_tip_about_the_vertical_axis_of_the_pan_joint():
    chain = UrdfChain(_urdf_xml())
    q = np.array(NAMED_POSES['T (W1)'])
    pan = chain.joint('shoulder_pan').xyz
    base = chain.fk(q)[0] - pan
    for angle in (-1.0, 0.5, 1.9):
        turned = chain.fk(np.concatenate([[angle], q[1:]]))[0] - pan
        # the rpy of the pan joint holds the rounded 3.14159, so its axis is tilted by 2.7e-6 rad
        assert abs(turned[2] - base[2]) < 2e-6
        assert abs(math.hypot(*turned[:2]) - math.hypot(*base[:2])) < 2e-6
        # the model says q1 > 0 turns the arm clockwise seen from above (S1 = -1)
        assert abs(math.atan2(turned[1], turned[0]) - math.atan2(base[1], base[0])
                   + (angle - q[0])) < 1e-5


def test_model_fk_equals_the_chain_for_random_poses():
    worst, where = _worst_difference_mm(UrdfChain(_urdf_xml()), analytic_ik.CONSTANTS,
                                        _random_poses())
    assert worst <= INSTALLED_TOLERANCE_MM, f'worst {worst:.5f} mm at q = {where}'


@pytest.mark.parametrize('name', sorted(NAMED_POSES))
def test_model_fk_equals_the_chain_for_named_poses(name):
    q = NAMED_POSES[name]
    chain = UrdfChain(_urdf_xml())
    diff = np.linalg.norm(chain.fk(q)[0] - analytic_ik.fk(q)[0]) * 1000.0
    assert diff <= INSTALLED_TOLERANCE_MM, f'{name}: {diff:.5f} mm'


def test_model_fk_equals_the_chain_for_a_roll_sweep():
    worst, where = _worst_difference_mm(UrdfChain(_urdf_xml()), analytic_ik.CONSTANTS,
                                        _roll_sweep())
    assert worst <= INSTALLED_TOLERANCE_MM, f'worst {worst:.5f} mm at q = {where}'


def test_roll_moves_the_tip_because_the_tcp_is_off_the_roll_axis():
    chain = UrdfChain(_urdf_xml())
    q0 = np.array(NAMED_POSES['T (W1)'][:4] + [0.0])
    q1 = q0 + np.array([0, 0, 0, 0, 1.0])
    moved = np.linalg.norm(chain.fk(q1)[0] - chain.fk(q0)[0]) * 1000.0
    assert 7.0 < moved < 8.5, moved  # about 7.6 mm
    model_moved = np.linalg.norm(analytic_ik.fk(q1)[0] - analytic_ik.fk(q0)[0]) * 1000.0
    assert abs(model_moved - moved) < 0.01


def test_model_derived_from_a_pi_exact_urdf_is_exact():
    # an independent formulation: the chain of the URDF with exact pi/2 and pi, and the closed-form
    # model with the constants derived from that same URDF; a wrong rpy order or axis direction
    # in the chain (or a wrong model) does not agree to 1e-5 mm
    xml = _exact_urdf_xml()
    worst, where = _worst_difference_mm(UrdfChain(xml), derive_constants(xml),
                                        _random_poses() + _roll_sweep()
                                        + list(map(list, NAMED_POSES.values())))
    assert worst <= EXACT_TOLERANCE_MM, f'worst {worst:.3e} mm at q = {where}'


def test_fk_at_the_recorded_pose_of_the_real_arm():
    # tf_echo of the real arm at the pose T (W1), 1 mm resolution
    q = NAMED_POSES['T (W1)']
    recorded = np.array([0.172, 0.000, 0.055])
    for position in (UrdfChain(_urdf_xml()).fk(q)[0], analytic_ik.fk(q)[0]):
        assert np.linalg.norm(position - recorded) * 1000.0 <= TF_TOLERANCE_MM, position
