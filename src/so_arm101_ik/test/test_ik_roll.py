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

"""Inverse kinematics with the roll (P2): closed form, no iteration, rejections."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik.urdf_kinematics import UrdfChain

JOINT_TOLERANCE = 1e-9  # rad, IK(FK(q)) = q
MODEL_TOLERANCE_MM = 0.001  # FK(IK(p)) = p with the same model
CHAIN_TOLERANCE_MM = 0.01  # URDF chain FK(IK(p)) = p (model vs chain difference)
REGRESSION_TOLERANCE = 1e-12  # rad, the roll = 0 results of P1

# conditions that are excluded from the round trips (as in test_ik_roundtrip.py)
NEAR_STRAIGHT = 1e-5
MIN_RADIAL = 0.01
LIMIT_MARGIN = 1e-3
BRANCH_DISTANCE = 0.01

ROLL_SWEEP = [0.25 * n for n in range(-10, 11)]  # -2.5 .. 2.5 rad
T_POSE = [0.0031, -0.3813, 0.5501, 1.4454, 0.0]

# P1 results (roll = 0), written here as numbers (the test reads no git or external file).
# How they were made: analytic_ik.py, urdf_kinematics.py and __init__.py of commit 4aadc04 (the P1
# commit; the package is identical at b1b51f5) were extracted with "git show" into a scratch
# directory outside the repository and imported there; for each pose q, the target is
# FK(q).position + (0.004, -0.003, 0.002) m and the entry is (q, target, pitch of FK(q),
# solve_ik(target, current_q = q, pitch = pitch, roll = 0.0)), printed with repr() (17 digits).
P1_RESULTS = (
    (T_POSE, [0.1763866479117904, -0.003423222662237125, 0.05723324648075191],
     -1.6141934944464,
     [0.0248147836131804, -0.3478648630313713, 0.5052232698528742, 1.4568415931784973, 0.0]),
    ([0.0, -1.57, 1.57, 0.8, 0.0],
     [0.20085417101095987, -0.003009212113274826, 0.03216225025965494], -0.7999934944464,
     [0.01851423614516575, -1.4529534097032886, 1.5340948677229327, 0.7188585419803561, 0.0]),
    ([0.2, -0.8, 0.8, 0.9, 0.0],
     [0.23575237011471312, -0.0421156254276278, 0.09267598231236682], -0.8999934944464,
     [0.21065480830633718, -0.7548981567420169, 0.7597988349134563, 0.8950993218285606, 0.0]),
)


def _urdf_chain():
    share = Path(get_package_share_directory('so_arm101_description'))
    return UrdfChain((share / 'urdf' / 'so101_new_calib.urdf').read_text())


def _signed_radial(q, xyz):
    theta = ik.S1 * q[0]
    c = ik.CONSTANTS
    return (xyz[0] - c.pan_x) * math.cos(theta) + (xyz[1] - c.pan_y) * math.sin(theta)


def _is_usable(q):
    """Return False for the poses that are excluded from the round trips (rounding issues)."""
    q = np.asarray(q, dtype=float)
    if np.any(q - ik.Q_MIN < LIMIT_MARGIN) or np.any(ik.Q_MAX - q < LIMIT_MARGIN):
        return False
    xyz, pitch, roll = ik.fk(q)
    if _signed_radial(q, xyz) < MIN_RADIAL:
        return False
    try:
        geometry = ik.ik_geometry(xyz[0], xyz[1], xyz[2], pitch, roll)
    except ik.IKError:
        return False
    if abs(geometry.cos_elbow) > 1.0 - NEAR_STRAIGHT:
        return False
    return bool(np.linalg.norm(geometry.candidates[0] - geometry.candidates[1]) >= BRANCH_DISTANCE)


def _usable_poses():
    rng = np.random.default_rng(20261114)
    poses = [rng.uniform(ik.Q_MIN, ik.Q_MAX) for _ in range(3000)]
    for roll in ROLL_SWEEP:  # the named pose with the roll swept
        q = np.array(T_POSE)
        q[4] = roll
        poses.append(q)
    return [q for q in poses if _is_usable(q)]


USABLE = _usable_poses()


def test_the_roll_round_trips_are_not_vacuous():
    assert len(USABLE) >= 500, len(USABLE)
    assert len({round(float(q[4]), 1) for q in USABLE}) >= 40  # the roll is really spread


def test_joint_space_round_trip_with_roll():
    worst, where = 0.0, None
    for q in USABLE:
        xyz, pitch, roll = ik.fk(q)
        solved = ik.solve_ik(xyz[0], xyz[1], xyz[2], q, pitch=pitch, roll=roll)
        diff = float(np.max(np.abs(solved - q)))
        if diff > worst:
            worst, where = diff, q
    assert worst <= JOINT_TOLERANCE, f'worst {worst:.3e} rad at q = {where}'


def test_position_round_trip_with_roll_for_moved_targets():
    rng = np.random.default_rng(11)
    worst, solved_count = 0.0, 0
    for q in USABLE:
        xyz, pitch, roll = ik.fk(q)
        target = xyz + rng.uniform(-0.01, 0.01, 3)
        try:
            solved = ik.solve_ik(target[0], target[1], target[2], q, pitch=pitch, roll=roll)
        except ik.IKError:
            continue
        solved_count += 1
        worst = max(worst, np.linalg.norm(ik.fk(solved)[0] - target) * 1000.0)
        assert abs(ik.fk(solved)[1] - pitch) < 1e-9 and solved[4] == roll
    assert solved_count >= 300, solved_count
    assert worst <= MODEL_TOLERANCE_MM, f'worst {worst:.3e} mm'


def test_round_trip_with_roll_through_the_urdf_chain():
    chain = _urdf_chain()
    worst, where = 0.0, None
    for q in USABLE:
        target = chain.fk(q)[0]
        _, pitch, roll = ik.fk(q)
        solved = ik.solve_ik(target[0], target[1], target[2], q, pitch=pitch, roll=roll)
        diff = np.linalg.norm(chain.fk(solved)[0] - target) * 1000.0
        if diff > worst:
            worst, where = diff, q
    assert worst <= CHAIN_TOLERANCE_MM, f'worst {worst:.5f} mm at q = {where}'


def test_the_roll_moves_the_tip_in_the_same_direction_as_the_chain():
    # a roll that is solved for the same target moves the arm, not the target: the lateral
    # offset b of the model must follow the chain (sign of the roll, S5)
    chain = _urdf_chain()
    q = np.array(T_POSE)
    for roll in (-1.0, 1.0):
        moved = q.copy()
        moved[4] = roll
        model_shift = ik.fk(moved)[0] - ik.fk(q)[0]
        chain_shift = chain.fk(moved)[0] - chain.fk(q)[0]
        assert np.linalg.norm(model_shift - chain_shift) * 1000.0 <= CHAIN_TOLERANCE_MM
        assert abs(chain_shift[1]) > 0.003  # the shift is really there (several mm)


def test_roll_zero_reproduces_the_p1_results():
    for q, target, pitch, expected in P1_RESULTS:
        solved = ik.solve_ik(target[0], target[1], target[2], np.array(q), pitch=pitch, roll=0.0)
        assert np.max(np.abs(solved - expected)) <= REGRESSION_TOLERANCE, (q, solved)


def test_the_two_branches_share_the_pan_and_the_roll():
    both = 0
    for q in USABLE:
        xyz, pitch, roll = ik.fk(q)
        first, second = ik.ik_geometry(xyz[0], xyz[1], xyz[2], pitch, roll).candidates
        assert first[0] == second[0] and first[4] == second[4] == roll
        both += 1
    assert both >= 500


def test_the_nearest_branch_does_not_depend_on_the_roll():
    both = 0
    for q in USABLE:
        xyz, pitch, roll = ik.fk(q)
        candidates = ik.ik(xyz[0], xyz[1], xyz[2], pitch, roll)
        if len(candidates) != 2:
            continue
        both += 1
        for candidate in candidates:
            solved = ik.solve_ik(xyz[0], xyz[1], xyz[2], candidate, pitch=pitch, roll=roll)
            assert np.max(np.abs(solved - candidate)) < JOINT_TOLERANCE
    assert both >= 20, both


def test_the_pan_axis_neighbourhood_depends_on_the_roll():
    # the lateral offset b of the TCP changes with the roll: a target 3 mm from the pan axis is
    # inside |b| for a roll with b = -6.7 mm (roll 1.0) and not rejected for this reason at b = 0
    c = ik.CONSTANTS
    assert abs(ik.tcp_offsets(1.0)[1]) > 0.006
    with pytest.raises(ik.IKUnreachableError, match='lateral offset'):
        ik.ik(c.pan_x + 0.003, c.pan_y, 0.2, -1.0, 1.0)
    zero_b_roll = -c.psi0  # b = y0 + k sin(0) = y0 = -0.18 mm
    assert abs(ik.tcp_offsets(zero_b_roll)[1]) < 0.0003
    try:
        ik.ik(c.pan_x + 0.003, c.pan_y, 0.2, -1.0, zero_b_roll)
    except ik.IKError as error:
        assert 'lateral offset' not in str(error)


def test_a_roll_outside_the_joint_limits_is_rejected_and_never_wrapped():
    xyz, pitch, _ = ik.fk(T_POSE)
    for roll in (3.0, -2.8, 0.1 + 2.0 * math.pi, 0.1 - 2.0 * math.pi):
        with pytest.raises(ik.IKJointLimitError):
            ik.ik(xyz[0], xyz[1], xyz[2], pitch, roll)
    for roll in (ik.Q_MIN[4], ik.Q_MAX[4]):  # the limits themselves are inclusive
        assert ik.ik(xyz[0], xyz[1], xyz[2], pitch, roll)


def test_non_finite_roll_is_rejected():
    for bad in (float('nan'), float('inf'), float('-inf')):
        with pytest.raises(ValueError):
            ik.ik(0.2, 0.0, 0.1, -1.0, bad)
        with pytest.raises(ValueError):
            ik.solve_ik(0.2, 0.0, 0.1, np.zeros(5), pitch=-1.0, roll=bad)


def test_solve_ik_keeps_the_roll_of_the_current_pose_by_default():
    q = np.array(T_POSE)
    q[4] = -1.3
    xyz, pitch, roll = ik.fk(q)
    solved = ik.solve_ik(xyz[0] + 0.004, xyz[1], xyz[2], q)
    assert solved[4] == -1.3 and abs(ik.fk(solved)[2] - roll) < 1e-12
    assert abs(ik.fk(solved)[1] - pitch) < JOINT_TOLERANCE
