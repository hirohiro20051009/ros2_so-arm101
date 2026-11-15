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

"""Safety of the differential mode (P4): the last check, random walks, numbers of 7.12.4."""

import ast
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik import diff_mode as dm
from so_arm101_ik.diff_mode import DiffMode, DiffModeConfig, Status, StartRefused, verify_command
from so_arm101_ik.urdf_kinematics import UrdfChain

T_Q = np.radians([0.0, -22.0, 30.0, 83.0, 0.0])
T_XYZ, T_PITCH, T_ROLL = ik.fk(T_Q)
T_MEASURED = np.array([0.0031, -0.3813, 0.5501, 1.4454, -0.0069])  # T (W1)
CONFIG = DiffModeConfig()
DIRECTIONS = {'+x': (1, 0, 0), '-x': (-1, 0, 0), '+y': (0, 1, 0), '-y': (0, -1, 0),
              '+z': (0, 0, 1), '-z': (0, 0, -1)}


def _chain():
    share = Path(get_package_share_directory('so_arm101_description'))
    return UrdfChain((share / 'urdf' / 'so101_new_calib.urdf').read_text())


def _at_height(z):
    return ik.solve_ik(T_XYZ[0], T_XYZ[1], z, T_Q, pitch=T_PITCH, roll=T_ROLL)


def _dq_at_t(direction, millimetres):
    target = T_XYZ + np.array(direction, dtype=float) * millimetres / 1000.0
    q = ik.solve_ik(*target, T_Q, pitch=T_PITCH, roll=T_ROLL)
    return dm.wrapped_norm(q, T_Q)


# ----------------------------------------------------------------------- verify_command


def test_verify_command_rejects_each_bad_command():
    low = _at_height(0.047)
    high = _at_height(0.0495)
    sagged = _at_height(0.0445)
    over = T_Q.copy()
    over[3] = ik.Q_MAX[3] + 1e-6
    d_t = dm.abs_d_of_q(T_Q)
    bad_d = DiffModeConfig(max_abs_d=d_t - 1e-6)
    shifted = T_Q + np.array([0.07, 0.0, 0.0, 0.0, 0.0])
    lifted = T_Q + np.array([0.0, 0.06, 0.0, 0.0, 0.0])
    nan = T_Q.copy()
    nan[1] = np.nan
    cases = [
        ('good', (T_Q, T_Q, T_Q, 1, CONFIG, None), None),
        ('nan', (nan, T_Q, T_Q, 1, CONFIG, None), 'NOT_FINITE'),
        ('short', (T_Q, T_Q, [0.0, 0.0], 1, CONFIG, None), 'NOT_FINITE'),
        ('range', (over, over, over, 1, CONFIG, None), 'JOINT_LIMIT'),
        ('floor of the command', (low, low, low, 1, CONFIG, None), 'FLOOR_CMD'),
        ('dq', (T_Q, lifted, T_Q, 1, CONFIG, None), 'DQ_TOO_BIG'),
        ('wind-up', (T_Q, T_Q, shifted, 1, CONFIG, None), 'FINAL_CHECK'),
        ('floor of the measurement', (high, high, sagged, 1, CONFIG, None), 'FLOOR_MEASURED'),
        ('abs(D)', (T_Q, T_Q, T_Q, 1, bad_d, None), 'FINAL_CHECK'),
        ('target', (T_Q, T_Q, T_Q, 1, CONFIG, T_XYZ + [0.001, 0.0, 0.0]), 'FINAL_CHECK'),
        ('target (matching)', (T_Q, T_Q, T_Q, 1, CONFIG, T_XYZ), None),
    ]
    for name, arguments, reason in cases:
        verdict = verify_command(*arguments)
        if reason is None:
            assert verdict is None, name
        else:
            assert verdict is not None and verdict[0] == reason, name
    # exactly at the strict range is accepted; 1e-6 rad beyond it is not (every joint)
    for joint in range(5):
        for limit, outside in ((ik.Q_MAX[joint], 1e-6), (ik.Q_MIN[joint], -1e-6)):
            q = T_Q.copy()
            q[joint] = limit + outside
            verdict = verify_command(q, q, q, 1, CONFIG)
            assert verdict is not None and verdict[0] == 'JOINT_LIMIT'


def test_verify_command_detects_the_other_branch():
    assert dm.branch_of_q(T_Q) == 1
    assert verify_command(T_Q, T_Q, T_Q, 1, CONFIG) is None
    verdict = verify_command(T_Q, T_Q, T_Q, 0, CONFIG)  # the command is on branch 1, not 0
    assert verdict is not None and verdict[0] == 'BRANCH_SWITCH'


def test_abs_d_and_the_branch_from_q3_agree_with_the_ik():
    rng = np.random.default_rng(29)
    count = 0
    for _ in range(6000):
        q = rng.uniform(ik.Q_MIN, ik.Q_MAX)
        xyz, pitch, roll = ik.fk(q)
        try:
            geometry = ik.ik_geometry(*xyz, pitch, roll)
        except ik.IKError:
            continue
        match = [i for i, c in enumerate(geometry.candidates)
                 if float(np.max(np.abs(c - q))) < 1e-6]
        if len(match) != 1:
            continue
        assert dm.abs_d_of_q(q) == pytest.approx(abs(geometry.cos_elbow), abs=1e-9)
        assert dm.branch_of_q(q) == match[0]
        count += 1
    assert count >= 500


# ------------------------------------------------------------------------- random walks


def _check_command(chain, mode, previous):
    q = mode.state.q_cmd
    assert np.all(np.isfinite(q))
    assert ik.within_limits(q)  # the strict range, never the margin
    assert chain.fk(q)[0][2] >= 0.048 - 1e-4  # the URDF chain, independent of the model
    assert dm.branch_of_q(q) == mode.state.branch
    assert dm.wrapped_norm(q, previous) <= 0.05 + 1e-9
    assert np.linalg.norm(mode.state.target - mode.state.target0) <= 0.03 + 1e-9


def test_random_walk_with_suggestions_only_sends_safe_commands():
    chain = _chain()
    rng = np.random.default_rng(31)
    sent = 0
    statuses = set()
    config = DiffModeConfig(max_reversals=None)
    for _ in range(60):
        mode = DiffMode.start(T_Q, T_Q, config)
        direction = rng.normal(size=3)
        direction /= np.linalg.norm(direction)
        goal = T_XYZ + direction * rng.uniform(0.01, 0.08)  # often beyond a boundary
        for _ in range(9):
            previous = mode.state.q_cmd
            measured = previous + rng.uniform(-0.003, 0.003, 5)
            suggestion = mode.suggest_delta(goal, measured)
            statuses.add(suggestion.status)
            assert suggestion.status is not Status.REFUSED, suggestion.reason
            if suggestion.status in (Status.OK, Status.AT_LIMIT):
                result = mode.step(suggestion.delta, measured, suggestion.requested_norm)
                statuses.add(result.status)
                assert result.status is not Status.REFUSED, result.reason
                if result.status in (Status.OK, Status.AT_LIMIT):
                    sent += 1
                    _check_command(chain, mode, previous)
            elif suggestion.status is Status.LIMITED_DONE:
                break
    assert sent >= 100
    assert {Status.OK, Status.AT_LIMIT, Status.HOLD} <= statuses


def test_random_walk_with_direct_steps_only_sends_safe_commands():
    chain = _chain()
    rng = np.random.default_rng(37)
    sent = 0
    refusals = set()
    config = DiffModeConfig(max_reversals=None)
    for _ in range(60):
        mode = DiffMode.start(T_Q, T_Q, config)
        for _ in range(9):
            previous = mode.state.q_cmd
            delta = rng.normal(size=3)
            delta *= rng.uniform(0.0005, 0.005) / np.linalg.norm(delta)
            result = mode.step(delta, previous + rng.uniform(-0.003, 0.003, 5))
            if result.status in (Status.OK, Status.AT_LIMIT):
                sent += 1
                _check_command(chain, mode, previous)
            elif result.status is Status.REFUSED:
                refusals.add(result.reason)
                assert mode.state.halted
                assert mode.state.q_cmd == pytest.approx(previous, abs=1e-15)
                break
    assert sent >= 100
    # only the stops that a direct step can reach (a large difference, a reversal)
    assert refusals <= {'DQ_TOO_BIG', 'REVERSAL_GROWTH', 'STUCK'}


def test_the_mode_has_no_clock_no_ros_and_no_io():
    path = Path(dm.__file__)
    tree = ast.parse(path.read_text())
    modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            modules.add(('.' * node.level) + (node.module or ''))
    assert modules <= {'dataclasses', 'enum', 'math', 'typing', 'numpy', '.'}
    called = {node.func.id for node in ast.walk(tree)
              if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
    assert not called & {'open', 'exec', 'eval', '__import__', 'input', 'print'}
    attributes = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    assert not attributes & {'time', 'sleep', 'monotonic', 'system', 'popen', 'run'}


# -------------------------------------------------------------------- the numbers of 7.12.4


def test_the_dq_at_t_is_the_recorded_table():
    # [rad] for 1, 3 and 5 mm in +x, -x, +y, -y, +z, -z (2-norm; the plan 7.12.4)
    table = {
        1: [0.0118, 0.0118, 0.0073, 0.0073, 0.0101, 0.0101],
        3: [0.0355, 0.0354, 0.0218, 0.0218, 0.0304, 0.0302],
        5: [0.0592, 0.0589, 0.0363, 0.0363, 0.0508, 0.0503],
    }
    for millimetres, expected in table.items():
        values = [_dq_at_t(direction, millimetres) for direction in DIRECTIONS.values()]
        assert values == pytest.approx(expected, abs=2e-4)
    rng = np.random.default_rng(1)
    values = []
    for _ in range(2000):
        v = rng.normal(size=3)
        values.append(_dq_at_t(v / np.linalg.norm(v), 3.0))
    assert 0.0210 < min(values) < 0.0225 and 0.037 < max(values) < 0.0395


def test_the_largest_move_inside_each_dq_limit_is_the_recorded_table():
    expected = {
        0.02: [1.69, 1.70, 2.76, 2.76, 1.97, 1.98],
        0.03: [2.54, 2.54, 4.13, 4.13, 2.96, 2.98],
        0.05: [4.22, 4.24, 6.89, 6.89, 4.92, 4.97],
    }
    for limit, values in expected.items():
        found = []
        for direction in DIRECTIONS.values():
            low, high = 0.0, 20.0
            for _ in range(40):
                middle = 0.5 * (low + high)
                if _dq_at_t(direction, middle) <= limit:
                    low = middle
                else:
                    high = middle
            found.append(low)
        assert found == pytest.approx(values, abs=0.02)  # mm


def test_the_two_branches_are_at_least_2_acos_abs_d_apart():
    rng = np.random.default_rng(41)
    count = 0
    floor = 2.0 * math.acos(dm.HARD_MAX_ABS_D)
    for _ in range(20000):
        q = rng.uniform(ik.Q_MIN, ik.Q_MAX)
        xyz, pitch, roll = ik.fk(q)
        try:
            geometry = ik.ik_geometry(*xyz, pitch, roll)
        except ik.IKError:
            continue
        d = abs(geometry.cos_elbow)
        if geometry.radial < 0.05 or d > dm.HARD_MAX_ABS_D:
            continue
        distance = dm.wrapped_norm(*geometry.candidates)
        assert distance >= 2.0 * math.acos(d) - 1e-9
        assert distance >= floor - 1e-9 and distance > 5.6 * dm.HARD_MAX_DQ
        count += 1
    assert count >= 1000
    at_t = ik.ik_geometry(*T_XYZ, T_PITCH, T_ROLL).candidates
    assert dm.wrapped_norm(*at_t) == pytest.approx(3.703, abs=1e-3)  # the wrapped 2-norm
    assert float(np.linalg.norm(at_t[0] - at_t[1])) == pytest.approx(4.447, abs=1e-3)  # plain
    q3 = at_t[0][2] - at_t[1][2]
    assert abs(q3) == pytest.approx(3.625, abs=1e-3)  # the plain difference of q3
    assert abs(ik.normalize_angle(q3)) == pytest.approx(2.0 * math.acos(0.23895), abs=1e-3)


def test_abs_d_is_the_same_limit_at_entry_and_at_the_last_check():
    d_t = dm.abs_d_of_q(T_Q)
    assert d_t == pytest.approx(0.239, abs=1e-3)
    assert d_t == pytest.approx(abs(ik.ik_geometry(*T_XYZ, T_PITCH, T_ROLL).cos_elbow), abs=1e-12)
    for offset, allowed in ((-1e-6, False), (1e-6, True)):
        config = DiffModeConfig(max_abs_d=d_t + offset)
        verdict = verify_command(T_Q, T_Q, T_Q, 1, config)
        assert (verdict is None) == allowed
        if allowed:
            DiffMode.start(T_Q, T_Q, config)
        else:
            with pytest.raises(StartRefused) as error:
                DiffMode.start(T_Q, T_Q, config)
            assert 'SINGULAR' in error.value.reasons
    # the guarantee that a branch switch exceeds the dq limit: 8.0 times at 0.98, 5.7 at 0.99
    assert 2.0 * math.acos(0.98) / dm.HARD_MAX_DQ == pytest.approx(8.01, abs=0.01)
    assert 2.0 * math.acos(0.99) / dm.HARD_MAX_DQ == pytest.approx(5.66, abs=0.01)


def test_the_tip_of_t_from_the_target_joints_and_from_the_measurement():
    measured, _, _ = ik.fk(T_MEASURED)
    tf_record = np.array([0.172, 0.000, 0.055])  # tf2_echo of the real arm, 1 mm resolution
    assert T_XYZ * 1000 == pytest.approx([176.66, 0.0, 58.77], abs=0.02)
    assert measured * 1000 == pytest.approx([172.39, -0.37, 55.23], abs=0.02)
    assert np.max(np.abs(measured - tf_record)) < 0.0005  # the measurement agrees with the tf
    assert (T_XYZ - tf_record) * 1000 == pytest.approx([4.66, 0.0, 3.77], abs=0.02)
    # the x of the distance limit (0.05 m): from the command and from the measurement
    assert T_XYZ[0] + 0.05 == pytest.approx(0.2267, abs=5e-5)
    assert measured[0] + 0.05 == pytest.approx(0.222, abs=5e-4)
    # the difference of the joints is the elbow (about 17 ticks), the cause of the difference
    assert (T_MEASURED - T_Q)[2] / (2.0 * math.pi / 4096.0) == pytest.approx(17.3, abs=0.5)


def test_the_reach_from_t_in_six_directions():
    mode = DiffMode.start(T_Q, T_Q, CONFIG)
    reach = {}
    kinds = {}
    for name, direction in DIRECTIONS.items():
        delta = np.array(direction, dtype=float) * 0.05  # larger than max_delta: internal plan
        s, limits, _ = mode._plan(delta, with_dq=False)
        reach[name] = s * 0.05 * 1000.0  # mm
        kinds[name] = limits
    for name in ('+x', '-x', '+y', '-y'):
        assert reach[name] == pytest.approx(29.999, abs=0.01)  # the distance (30 mm - 1 um)
        assert kinds[name] == ('DISTANCE',)
    assert reach['-z'] == pytest.approx((T_XYZ[2] - 0.048 - 1e-6) * 1000.0, abs=0.01)  # 10.77 mm
    assert kinds['-z'] == ('FLOOR',)
    assert 25.3 < reach['+z'] < 25.7  # wrist_flex 0.02 rad inside its range
    assert kinds['+z'] == ('JOINT_RANGE',)
    # without the joint margin the range is reached 2.6 mm later (28.2 mm)
    plain = DiffMode.start(T_Q, T_Q, DiffModeConfig(margin_joint=0.0))
    s, limits, _ = plain._plan(np.array([0.0, 0.0, 0.05]), with_dq=False)
    assert 28.0 < s * 50.0 < 28.4 and limits == ('JOINT_RANGE',)
    assert s * 50.0 - reach['+z'] == pytest.approx(2.6, abs=0.2)
