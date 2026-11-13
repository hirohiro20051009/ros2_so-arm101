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

"""Round trips of the analytic inverse kinematics (roll = 0) and the excluded conditions."""

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

# conditions that are excluded from the round trips (rounding; tested separately below)
NEAR_STRAIGHT = 1e-5  # |D| > 1 - this: the arm is almost fully stretched or folded
MIN_RADIAL = 0.01  # m: the TCP is almost above the pan axis (or behind it)
LIMIT_MARGIN = 1e-3  # rad: a joint is almost at its limit
BRANCH_DISTANCE = 0.01  # rad: the two elbow branches (before the limits) are this close

NAMED_POSES = {
    'zero': [0.0, 0.0, 0.0, 0.0, 0.0],
    'T (W1, roll 0)': [0.0031, -0.3813, 0.5501, 1.4454, 0.0],
    'T (target)': [0.0, math.radians(-22), math.radians(30), math.radians(83), 0.0],
    'home': [0.0, -1.57, 1.57, 0.8, 0.0],
    'stretched': [0.0, 0.3, -0.3, 0.2, 0.0],
}


def _urdf_chain():
    share = Path(get_package_share_directory('so_arm101_description'))
    return UrdfChain((share / 'urdf' / 'so101_new_calib.urdf').read_text())


def _signed_radial(q, xyz):
    theta = ik.S1 * q[0]
    c = ik.CONSTANTS
    return (xyz[0] - c.pan_x) * math.cos(theta) + (xyz[1] - c.pan_y) * math.sin(theta)


def _exclusion(q):
    """Return the reason why the pose q (roll 0) is excluded from the round trips, or None."""
    q = np.asarray(q, dtype=float)
    xyz, pitch, roll = ik.fk(q)
    if np.any(q - ik.Q_MIN < LIMIT_MARGIN) or np.any(ik.Q_MAX - q < LIMIT_MARGIN):
        return 'joint at its limit'
    if _signed_radial(q, xyz) < MIN_RADIAL:
        return 'above or behind the pan axis'
    try:
        geometry = ik.ik_geometry(xyz[0], xyz[1], xyz[2], pitch, roll)
    except ik.IKError:
        return 'unreachable'
    if abs(geometry.cos_elbow) > 1.0 - NEAR_STRAIGHT:
        return 'stretched or folded'
    first, second = geometry.candidates
    if np.linalg.norm(first - second) < BRANCH_DISTANCE:
        return 'branches are close'
    return None


def _random_poses(count, seed):
    rng = np.random.default_rng(seed)
    poses = []
    for _ in range(count):
        q = rng.uniform(ik.Q_MIN, ik.Q_MAX)
        q[4] = 0.0
        poses.append(q)
    return poses


def _usable_poses():
    poses = [np.array(q) for q in NAMED_POSES.values()] + _random_poses(3000, 20261113)
    return [q for q in poses if _exclusion(q) is None]


USABLE = _usable_poses()


def _excluded_poses():
    """Poses that are excluded from the round trips, one family per reason."""
    c = ik.CONSTANTS
    # the elbow angle q3 for which the two links are in line (fully stretched, D = 1)
    straight = (c.d2 + math.pi / 2.0 - c.d3) / ik.S3
    poses = [[0.2, -0.3, straight + eps, 0.5, 0.0] for eps in (0.0, 1e-4, -1e-4, 1e-3, -1e-3)]
    # joints at or just inside/outside their limits
    for index in range(4):
        for edge, sign in ((ik.Q_MIN, -1.0), (ik.Q_MAX, 1.0)):
            for eps in (0.0, 1e-6, 5e-4, -1e-3):
                q = np.array([0.1, -0.5, 0.8, 0.6, 0.0])
                q[index] = edge[index] - sign * eps
                poses.append(q)
    # the TCP is above or behind the pan axis (signed radial < MIN_RADIAL): the first ones found
    behind = [q for q in _random_poses(300, 5)
              if _exclusion(q) == 'above or behind the pan axis']
    return [np.array(q) for q in poses] + behind[:10]


def _solutions_are_finite_and_exact(q):
    """Return the number of solutions; they must be finite and reproduce the target position."""
    xyz, pitch, roll = ik.fk(q)
    try:
        solutions = ik.ik(xyz[0], xyz[1], xyz[2], pitch, roll)
    except ik.IKError:
        return 0
    for solution in solutions:
        assert np.all(np.isfinite(solution)), (q, solution)
        assert ik.within_limits(solution)
        diff = np.linalg.norm(ik.fk(solution)[0] - xyz) * 1000.0
        assert diff <= MODEL_TOLERANCE_MM, f'q = {q}: {diff} mm'
        assert abs(ik.fk(solution)[1] - pitch) < 1e-9
    return len(solutions)


def test_the_round_trips_are_not_vacuous():
    assert len(USABLE) >= 500, len(USABLE)
    assert len(NAMED_POSES) >= 4


def test_joint_space_round_trip():
    worst, where = 0.0, None
    for q in USABLE:
        xyz, pitch, roll = ik.fk(q)
        solved = ik.solve_ik(xyz[0], xyz[1], xyz[2], q, pitch=pitch, roll=roll)
        diff = float(np.max(np.abs(solved - q)))
        if diff > worst:
            worst, where = diff, q
    assert worst <= JOINT_TOLERANCE, f'worst {worst:.3e} rad at q = {where}'


def test_position_round_trip_for_moved_targets():
    # not only the FK of a joint vector: targets moved by up to 10 mm (the solution is new)
    rng = np.random.default_rng(7)
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
        assert abs(ik.fk(solved)[1] - pitch) < 1e-9 and solved[4] == 0.0
    assert solved_count >= 300, solved_count
    assert worst <= MODEL_TOLERANCE_MM, f'worst {worst:.3e} mm'


def test_round_trip_through_the_urdf_chain():
    chain = _urdf_chain()
    worst, where = 0.0, None
    for q in USABLE:
        target = chain.fk(q)[0]
        pitch = ik.fk(q)[1]
        solved = ik.solve_ik(target[0], target[1], target[2], q, pitch=pitch, roll=0.0)
        diff = np.linalg.norm(chain.fk(solved)[0] - target) * 1000.0
        if diff > worst:
            worst, where = diff, q
    assert worst <= CHAIN_TOLERANCE_MM, f'worst {worst:.5f} mm at q = {where}'


def test_nearest_branch_is_selected():
    both = 0
    for q in USABLE:
        xyz, pitch, roll = ik.fk(q)
        candidates = ik.ik(xyz[0], xyz[1], xyz[2], pitch, roll)
        if len(candidates) != 2:
            continue
        both += 1
        for candidate in candidates:
            solved = ik.solve_ik(xyz[0], xyz[1], xyz[2], candidate, pitch=pitch, roll=roll)
            assert np.max(np.abs(solved - candidate)) < 1e-9
    assert both >= 20, both


def test_solve_ik_keeps_the_current_orientation_by_default():
    q = np.array(NAMED_POSES['T (W1, roll 0)'])
    xyz, pitch, _ = ik.fk(q)
    solved = ik.solve_ik(xyz[0] + 0.005, xyz[1], xyz[2], q)
    assert abs(ik.fk(solved)[1] - pitch) < 1e-9
    moved = xyz + [0.005, 0.0, 0.0]
    assert np.linalg.norm(ik.fk(solved)[0] - moved) * 1000.0 <= MODEL_TOLERANCE_MM


@pytest.mark.parametrize('index', range(12))
def test_excluded_conditions_give_finite_values_or_an_explicit_error(index):
    poses = _excluded_poses()
    chunk = poses[index::12]
    assert chunk
    for q in chunk:
        _solutions_are_finite_and_exact(q)


def test_the_excluded_families_are_really_excluded():
    reasons = {_exclusion(q) for q in _excluded_poses()}
    assert {'stretched or folded', 'joint at its limit', 'above or behind the pan axis'} <= reasons


def test_above_the_pan_axis():
    c = ik.CONSTANTS
    for dx, dy in ((0.0, 0.0), (1e-6, 0.0), (0.0, -1e-3), (0.005, 0.005)):
        try:
            solutions = ik.ik(c.pan_x + dx, c.pan_y + dy, 0.2, -1.0, 0.0)
        except ik.IKError:
            continue
        for solution in solutions:
            assert np.all(np.isfinite(solution))


def test_the_pan_axis_itself_is_unreachable_for_the_tcp():
    # the TCP is off the approach axis by the lateral offset, so r' is imaginary
    with pytest.raises(ik.IKUnreachableError):
        ik.ik(ik.CONSTANTS.pan_x, ik.CONSTANTS.pan_y, 0.2, -1.0, 0.0)


def test_unreachable_target_is_rejected():
    with pytest.raises(ik.IKUnreachableError):
        ik.solve_ik(0.6, 0.0, 0.05, np.zeros(5), pitch=-1.0, roll=0.0)
    with pytest.raises(ik.IKUnreachableError):
        ik.solve_ik(0.2, 0.0, 1.0, np.zeros(5), pitch=0.0, roll=0.0)


def test_target_outside_the_joint_limits_is_rejected():
    # the target is the FK of a pose with one joint beyond its limit, so the geometry is solvable
    # but every candidate is outside (wrist_flex 1.9 > 1.658; pan 2.2 > 1.920)
    for q in ([0.1, -0.5, 0.8, 1.9, 0.0], [2.2, -0.5, 0.8, 0.6, 0.0]):
        xyz, pitch, _ = ik.fk(q)
        assert np.all(np.isfinite(ik.ik_geometry(xyz[0], xyz[1], xyz[2], pitch, 0.0).candidates))
        with pytest.raises(ik.IKJointLimitError):
            ik.solve_ik(xyz[0], xyz[1], xyz[2], np.zeros(5), pitch=pitch, roll=0.0)


def test_roll_other_than_zero_is_not_supported_at_this_stage():
    with pytest.raises(ValueError, match='roll'):
        ik.solve_ik(0.2, 0.0, 0.1, np.zeros(5), pitch=-1.0, roll=0.1)
    with pytest.raises(ValueError, match='roll'):
        ik.solve_ik(0.2, 0.0, 0.1, np.array([0, 0, 0, 0, 0.5]), pitch=-1.0)


def test_non_finite_input_is_rejected():
    for bad in (float('nan'), float('inf')):
        with pytest.raises(ValueError):
            ik.ik(bad, 0.0, 0.1, -1.0, 0.0)
        with pytest.raises(ValueError):
            ik.ik(0.2, 0.0, 0.1, bad, 0.0)


def test_limits_are_inclusive_and_checked_for_every_joint():
    assert ik.within_limits(ik.Q_MIN) and ik.within_limits(ik.Q_MAX)
    for index in range(5):
        for edge, sign in ((ik.Q_MIN, -1.0), (ik.Q_MAX, 1.0)):
            q = np.zeros(5)
            q[index] = edge[index] + sign * 1e-6
            assert not ik.within_limits(q)
