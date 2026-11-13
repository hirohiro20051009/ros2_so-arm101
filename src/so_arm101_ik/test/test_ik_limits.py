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

"""Joint limits as arguments of the inverse kinematics, and the start pose (P2, 7.9.2)."""

import math

import numpy as np
import pytest
from so_arm101_ik import analytic_ik as ik

# start pose (W1 2026-11-13 07:54): the elbow is 0.0001 rad above its upper limit
START = np.array([-0.0967, -1.4968, 1.6901, 0.4204, -0.0453])
T_TARGET = np.radians([0.0, -22.0, 30.0, 83.0, 0.0])
EPSILON = 0.02  # rad, the margin that a read-only tool passes (not used for sending)
WIDE_MIN = ik.Q_MIN - EPSILON
WIDE_MAX = ik.Q_MAX + EPSILON


def _solve_start(delta, q=START):
    xyz, pitch, roll = ik.fk(q)
    return ik.solve_ik(xyz[0] + delta[0], xyz[1] + delta[1], xyz[2] + delta[2], q,
                       pitch=pitch, roll=roll, q_min=WIDE_MIN, q_max=WIDE_MAX)


def test_the_start_pose_is_out_of_range_and_below_the_floor():
    assert START[2] - ik.Q_MAX[2] == pytest.approx(1e-4, abs=1e-9)
    assert not ik.within_limits(START)
    xyz, pitch, roll = ik.fk(START)
    assert xyz == pytest.approx([0.21719, 0.01765, 0.03389], abs=5e-5)  # m
    assert math.degrees(pitch) == pytest.approx(-35.16, abs=0.01)
    assert roll == -0.0453
    assert xyz[2] < 0.045 - 0.010  # more than 10 mm below the floor (45 mm)
    assert abs(ik.ik_geometry(*xyz, pitch, roll).cos_elbow) == pytest.approx(0.987, abs=1e-3)


def test_strict_limits_reject_the_start_pose_by_default():
    xyz, pitch, roll = ik.fk(START)
    with pytest.raises(ik.IKJointLimitError):
        ik.ik(xyz[0], xyz[1], xyz[2], pitch, roll)
    with pytest.raises(ik.IKJointLimitError):
        ik.solve_ik(xyz[0], xyz[1], xyz[2], START, pitch=pitch, roll=roll)
    # the geometry itself is solvable (only the limits reject it)
    assert np.all(np.isfinite(ik.ik_geometry(xyz[0], xyz[1], xyz[2], pitch, roll).candidates))


def test_the_margin_recognises_the_start_pose_and_keeps_the_other_branch_out():
    xyz, pitch, roll = ik.fk(START)
    solutions = ik.ik(xyz[0], xyz[1], xyz[2], pitch, roll, q_min=WIDE_MIN, q_max=WIDE_MAX)
    assert len(solutions) == 1  # the other branch (elbow about -4.27) is far outside
    assert np.max(np.abs(solutions[0] - START)) < 1e-9
    solved = _solve_start([0.0, 0.0, 0.0])
    assert np.max(np.abs(solved - START)) < 1e-9


def test_a_margin_smaller_than_the_overrun_does_not_recognise_it():
    xyz, pitch, roll = ik.fk(START)
    with pytest.raises(ik.IKJointLimitError):
        ik.ik(xyz[0], xyz[1], xyz[2], pitch, roll, q_min=ik.Q_MIN - 5e-5, q_max=ik.Q_MAX + 5e-5)
    assert ik.ik(xyz[0], xyz[1], xyz[2], pitch, roll, q_min=ik.Q_MIN - 2e-4, q_max=ik.Q_MAX + 2e-4)


def test_the_margin_does_not_cover_a_calibration_sized_overrun():
    q = START.copy()
    q[2] = ik.Q_MAX[2] + 0.03  # 0.03 rad over, more than the 0.02 margin
    xyz, pitch, roll = ik.fk(q)
    with pytest.raises(ik.IKJointLimitError):
        ik.ik(xyz[0], xyz[1], xyz[2], pitch, roll, q_min=WIDE_MIN, q_max=WIDE_MAX)


def test_the_default_limits_are_strict_and_within_limits_is_unchanged():
    assert ik.within_limits(ik.Q_MIN) and ik.within_limits(ik.Q_MAX)
    assert not ik.within_limits(ik.Q_MAX + 1e-6)
    assert ik.within_limits(ik.Q_MAX + 1e-6, ik.Q_MIN, ik.Q_MAX + 1e-3)
    rng = np.random.default_rng(3)
    returned = 0
    for _ in range(1500):
        q = rng.uniform(ik.Q_MIN - 0.3, ik.Q_MAX + 0.3)
        xyz, pitch, roll = ik.fk(q)
        try:
            solutions = ik.ik(xyz[0], xyz[1], xyz[2], pitch, roll)
        except ik.IKError:
            continue
        for solution in solutions:  # nothing outside the strict limits is ever returned
            assert ik.within_limits(solution)
            returned += 1
    assert returned >= 300, returned


def test_the_start_pose_is_not_a_usable_start_of_the_differential_mode():
    # 3 mm in x: the largest joint change is 0.14 rad, more than the 0.05 rad step limit (2.4);
    # T (|D| = 0.24) needs 0.02 .. 0.03 rad for the same 3 mm
    start_step = np.max(np.abs(_solve_start([0.003, 0.0, 0.0]) - START))
    assert start_step > 0.1 and start_step == pytest.approx(0.1435, abs=0.005)
    for delta in ([0.003, 0.0, 0.0], [0.0, 0.003, 0.0], [0.0, 0.0, 0.003]):
        t_step = np.max(np.abs(_solve_start(delta, T_TARGET) - T_TARGET))
        assert 0.015 < t_step < 0.03, (delta, t_step)
    xyz, pitch, roll = ik.fk(T_TARGET)
    assert abs(ik.ik_geometry(xyz[0], xyz[1], xyz[2], pitch, roll).cos_elbow) == pytest.approx(
        0.239, abs=1e-3)
