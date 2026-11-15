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

"""The differential mode (P4): start, step, boundaries, suggestions, spin guard, outcome."""

import math

import numpy as np
import pytest
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik import diff_mode as dm
from so_arm101_ik.diff_mode import DiffMode, DiffModeConfig, Status, StartRefused

T_Q = np.radians([0.0, -22.0, 30.0, 83.0, 0.0])  # the pose T (the target joint values)
T_XYZ, T_PITCH, T_ROLL = ik.fk(T_Q)
START = np.array([-0.0967, -1.4968, 1.6901, 0.4204, -0.0453])  # the start pose (W1 07:54)
INSIDE = 0.02  # the joint margin (rad)

# Poses with one joint 0.01 rad inside one end of its range (joint index, end: 0 lower, 1 upper).
# Found with a fixed seed (every pose is accepted by start(), the other joints are more than
# 0.05 rad inside, abs(D) <= 0.989, radial >= 0.07 m, z >= 0.06 m).
BAND_POSES = {
    (0, 0): [-1.90986, 0.132739, -0.575507, 0.951815, -1.032428],
    (0, 1): [1.90986, 0.132739, -0.575507, 0.951815, -1.032428],
    (1, 0): [0.061061, -1.73533, 0.417395, 0.913054, 0.671518],
    (1, 1): [-1.677995, 1.73533, -1.532718, -1.424248, -2.260316],
    (2, 0): [-0.291359, 1.140405, -1.68, 0.163659, -2.548425],
    (2, 1): [-0.176708, -1.273535, 1.68, -0.978598, -1.257277],
    (3, 0): [-0.291359, 1.140405, -0.306907, -1.64806, -2.548425],
    (3, 1): [0.96335, 0.132739, -0.575507, 1.64806, -1.032428],
    (4, 0): [0.96335, 0.132739, -0.575507, 0.951815, -2.73385],
    (4, 1): [0.96335, 0.132739, -0.575507, 0.951815, 2.83121],
}


def _mode(q=None, **kwargs):
    q = T_Q if q is None else q
    return DiffMode.start(q, q, DiffModeConfig(**kwargs))


def _at_height(z):
    """Return the joint angles of the pose T lowered or raised to the height z."""
    return ik.solve_ik(T_XYZ[0], T_XYZ[1], z, T_Q, pitch=T_PITCH, roll=T_ROLL)


def _snapshot(mode):
    s = mode.state
    return (s.target.tolist(), s.target0.tolist(), s.q_cmd.tolist(), s.steps, s.reversals,
            s.holds, s.hold_run, s.at_limit_run)


def _send(mode, delta, q_meas=None):
    return mode.step(delta, mode.state.q_cmd if q_meas is None else q_meas)


# ------------------------------------------------------------------------------------- start


def test_start_accepts_the_pose_t_and_records_the_references():
    mode = _mode()
    s = mode.state
    assert s.target == pytest.approx(T_XYZ, abs=1e-12)
    assert s.target0 == pytest.approx(T_XYZ, abs=1e-12)
    assert s.q_cmd == pytest.approx(T_Q, abs=1e-12)
    assert s.pitch_ref == T_PITCH and s.roll_ref == T_ROLL
    assert s.branch == dm.branch_of_q(T_Q) == 1
    assert (s.steps, s.reversals, s.holds) == (0, 0, 0)
    assert not s.halted and not s.finished
    # the tip of T computed from the target joint values (7.12.4)
    assert T_XYZ == pytest.approx([0.17666, 0.0, 0.05877], abs=5e-5)


def test_the_first_zero_step_keeps_q_desired():
    mode = _mode()
    for delta in ([0.0, 0.0, 0.0], [1e-10, 0.0, 0.0]):
        result = mode.step(delta, T_Q)
        assert result.status is Status.OK
        assert result.q_cmd == pytest.approx(T_Q, abs=1e-12)
    assert mode.state.steps == 0  # a zero difference is not counted
    assert mode.state.q_cmd == pytest.approx(T_Q, abs=1e-12)


def test_start_rejects_the_start_pose_and_lists_every_reason():
    with pytest.raises(StartRefused) as error:
        DiffMode.start(START, START, DiffModeConfig())
    reasons = error.value.reasons
    assert 'JOINT_LIMIT' in reasons and 'FLOOR' in reasons and 'FLOOR_MEASURED' in reasons
    assert 'SINGULAR' not in reasons  # abs(D) = 0.987 is not above 0.99
    assert abs(dm.abs_d_of_q(START)) == pytest.approx(0.987, abs=1e-3)
    text = ' '.join(error.value.details)
    assert 'elbow_flex' in text and 'neighbourhood of the pose T' in text


def test_start_rejects_invalid_inputs():
    bad = np.array([0.0, np.nan, 0.0, 0.0, 0.0])
    cases = [
        ('no desired', None, T_Q, 'NO_DESIRED'),
        ('nan desired', bad, T_Q, 'NOT_FINITE'),
        ('short desired', [0.0, 0.0], T_Q, 'NOT_FINITE'),
        ('no measurement', T_Q, None, 'NO_MEASUREMENT'),
        ('nan measurement', T_Q, bad, 'NOT_FINITE'),
    ]
    for name, desired, measured, reason in cases:
        with pytest.raises(StartRefused) as error:
            DiffMode.start(desired, measured, DiffModeConfig())
        assert reason in error.value.reasons, name


def test_start_uses_the_strict_joint_range():
    # inside the margin band (0.01 rad from the end) is accepted: the margin is not used here
    for (joint, end), pose in BAND_POSES.items():
        q = np.array(pose)
        limit = ik.Q_MAX[joint] if end else ik.Q_MIN[joint]
        assert abs(q[joint] - limit) == pytest.approx(0.01, abs=2e-6)
        DiffMode.start(q, q, DiffModeConfig())
    # outside the strict range by 1e-6 rad is refused (every joint, both ends)
    for joint in range(5):
        for end, limit in ((1, ik.Q_MAX[joint]), (0, ik.Q_MIN[joint])):
            q = np.array(BAND_POSES[(joint, end)])
            q[joint] = limit + (1e-6 if end else -1e-6)
            with pytest.raises(StartRefused) as error:
                DiffMode.start(q, q, DiffModeConfig())
            assert 'JOINT_LIMIT' in error.value.reasons


def test_start_checks_the_entry_conditions():
    low = _at_height(0.047)
    high = _at_height(0.0495)
    sagged = _at_height(0.0445)
    shifted = T_Q + np.array([0.07, 0.0, 0.0, 0.0, 0.0])
    d_t = dm.abs_d_of_q(T_Q)
    cases = [
        ('floor', low, low, {}, 'FLOOR'),
        ('radial', T_Q, T_Q, {'min_radial': 0.14}, 'RADIAL'),
        ('abs(D) above', T_Q, T_Q, {'max_abs_d': d_t - 1e-6}, 'SINGULAR'),
        ('not settled', T_Q, shifted, {}, 'NOT_SETTLED'),
        ('measured floor', high, sagged, {}, 'FLOOR_MEASURED'),
    ]
    for name, desired, measured, kwargs, reason in cases:
        with pytest.raises(StartRefused) as error:
            DiffMode.start(desired, measured, DiffModeConfig(**kwargs))
        assert reason in error.value.reasons, name
    DiffMode.start(T_Q, T_Q, DiffModeConfig(max_abs_d=d_t + 1e-6))  # abs(D) just inside
    DiffMode.start(high, high, DiffModeConfig())  # 49.5 mm is above the floor


def test_start_requires_a_unique_branch():
    rng = np.random.default_rng(3)
    found = 0
    for _ in range(4000):
        q = rng.uniform(ik.Q_MIN, ik.Q_MAX)
        try:
            DiffMode.start(q, q, DiffModeConfig())
        except StartRefused as error:
            if 'NO_BRANCH' in error.reasons:
                found += 1
    assert found > 0  # an arm folded back over the pan axis is not reproduced by the IK


# ------------------------------------------------------------------------------------- step


def test_the_origin_is_the_previous_command_not_the_measurement():
    measured = T_Q + np.array([0.0, 0.02, -0.02, 0.01, 0.0])
    mode = DiffMode.start(T_Q, measured, DiffModeConfig())
    assert mode.state.target == pytest.approx(T_XYZ, abs=1e-12)  # not FK(q_meas)
    assert mode.state.q_cmd == pytest.approx(T_Q, abs=1e-12)
    rng = np.random.default_rng(5)
    deltas = [rng.uniform(0.0, 0.002, 3) for _ in range(6)]  # no reversal
    reference = _mode()
    perturbed = _mode()
    for delta in deltas:
        a = reference.step(delta, reference.state.q_cmd)
        offset = rng.uniform(-0.02, 0.02, 5)
        b = perturbed.step(delta, perturbed.state.q_cmd + offset)
        assert a.status is b.status is Status.OK
        assert a.q_cmd == pytest.approx(b.q_cmd, abs=1e-12)  # q_meas never enters the origin


def test_a_refusal_halts_without_changing_the_state():
    mode = _mode()
    _send(mode, [0.002, 0.0, 0.0])
    before = _snapshot(mode)
    result = mode.step([np.nan, 0.0, 0.0], mode.state.q_cmd)
    assert result.status is Status.REFUSED and result.reason == 'NOT_FINITE'
    assert _snapshot(mode) == before
    assert mode.state.halted and mode.state.reason == 'NOT_FINITE'
    after = _send(mode, [0.001, 0.0, 0.0])
    assert after.status is Status.REFUSED and after.reason == 'HALTED'
    assert _snapshot(mode) == before


def test_the_contract_violations_are_refused():
    good = T_Q
    cases = [
        ('nan delta', [np.nan, 0, 0], good, None, 'NOT_FINITE'),
        ('short delta', [0.001, 0], good, None, 'BAD_REQUEST'),
        ('no measurement', [0.001, 0, 0], None, None, 'NO_MEASUREMENT'),
        ('nan measurement', [0.001, 0, 0], np.full(5, np.nan), None, 'NOT_FINITE'),
        ('short measurement', [0.001, 0, 0], [0.0, 0.0], None, 'BAD_REQUEST'),
        ('delta too big', [0.006, 0, 0], good, None, 'DELTA_TOO_BIG'),
        ('requested below delta', [0.002, 0, 0], good, 0.001, 'BAD_REQUEST'),
        ('requested above the limit', [0.002, 0, 0], good, 0.006, 'BAD_REQUEST'),
        ('requested nan', [0.002, 0, 0], good, float('nan'), 'BAD_REQUEST'),
    ]
    for name, delta, measured, requested, reason in cases:
        mode = _mode()
        result = mode.step(delta, measured, requested)
        assert result.status is Status.REFUSED and result.reason == reason, name
        assert mode.state.halted, name
        assert mode.state.q_cmd == pytest.approx(T_Q, abs=1e-12), name


def test_a_measured_position_below_the_floor_stops():
    high = _at_height(0.0495)
    sagged = _at_height(0.0445)
    mode = DiffMode.start(high, high, DiffModeConfig())
    result = mode.step([0.001, 0.0, 0.0], sagged)
    assert result.status is Status.REFUSED and result.reason == 'FLOOR_MEASURED'
    assert mode.state.q_cmd == pytest.approx(high, abs=1e-12)


def test_a_difference_above_the_dq_limit_stops_without_shortening():
    mode = _mode()
    result = mode.step([0.005, 0.0, 0.0], T_Q)  # 5 mm in x is 0.059 rad at T
    assert result.status is Status.REFUSED and result.reason == 'DQ_TOO_BIG'
    assert mode.state.q_cmd == pytest.approx(T_Q, abs=1e-12) and mode.state.steps == 0


def test_the_windup_clip_moves_the_target_back_to_the_command():
    measured = T_Q + np.array([0.0, 0.0, 0.05, 0.0, 0.0])
    mode = _mode()
    result = mode.step([0.003, 0.0, 0.0], measured)
    assert result.status is Status.OK
    q = mode.state.q_cmd
    assert q[2] == pytest.approx(measured[2] - 0.06, abs=1e-12)  # clipped to the box
    assert np.max(np.abs(q - measured)) <= 0.06 + 1e-12
    assert mode.state.target == pytest.approx(ik.fk(q)[0], abs=1e-12)  # target := FK(q_cmd)
    assert mode.state.pitch_ref == T_PITCH  # the references are not changed


def test_no_progress_after_the_clip_is_stuck():
    delta = np.array([0.002, 0.0, 0.0])
    free = ik.solve_ik(*(T_XYZ + delta), T_Q, pitch=T_PITCH, roll=T_ROLL)
    sign = np.sign(free - T_Q) * (np.abs(free - T_Q) > 1e-6)
    measured = T_Q - 0.06 * sign  # the command is at the edge of the box in every moving joint
    mode = _mode()
    result = mode.step(delta, measured)
    assert result.status is Status.REFUSED and result.reason == 'STUCK'
    assert mode.state.q_cmd == pytest.approx(T_Q, abs=1e-12)


def test_the_other_branch_is_never_sent():
    rng = np.random.default_rng(11)
    cases = []
    for _ in range(600000):
        q = np.array([rng.uniform(-1.0, 1.0), rng.uniform(-1.7, 1.7), rng.uniform(-1.69, 1.69),
                      rng.uniform(-1.65, 1.65), rng.uniform(-1.0, 1.0)])
        xyz, pitch, roll = ik.fk(q)
        if xyz[2] < 0.048:
            continue
        try:
            geometry = ik.ik_geometry(*xyz, pitch, roll)
            if geometry.radial < 0.05 or abs(geometry.cos_elbow) > 0.99:
                continue
            branch = dm.branch_of_q(q)
            if float(np.max(np.abs(geometry.candidates[branch] - q))) > 1e-6:
                continue
            delta = rng.normal(size=3)
            delta *= 0.005 / np.linalg.norm(delta)
            moved = ik.ik_geometry(*(xyz + delta), pitch, roll)
        except ik.IKError:
            continue
        if (not ik.within_limits(moved.candidates[branch])
                and ik.within_limits(moved.candidates[1 - branch])):
            cases.append((q, delta))
        if len(cases) == 6:
            break
    assert len(cases) == 6  # such targets exist (about 2% of the 5 mm moves)
    started = 0
    for q, delta in cases:
        try:
            mode = DiffMode.start(q, q, DiffModeConfig())
        except StartRefused:
            continue
        started += 1
        result = mode.step(delta, q)
        assert result.status in (Status.AT_LIMIT, Status.HOLD, Status.REFUSED)
        assert dm.branch_of_q(mode.state.q_cmd) == mode.state.branch
        assert ik.within_limits(mode.state.q_cmd)
        assert np.all(mode.state.q_cmd <= np.maximum(ik.Q_MAX - INSIDE, q) + 1e-9)
        assert np.all(mode.state.q_cmd >= np.minimum(ik.Q_MIN + INSIDE, q) - 1e-9)
    assert started >= 2  # at least some of them are valid starts


def test_accepted_steps_equal_the_plain_ik_solution():
    rng = np.random.default_rng(13)
    mode = _mode(max_reversals=None)
    accepted = 0
    for count in range(150):
        if count % 8 == 7:
            mode = _mode(max_reversals=None)  # the budget of one mode is 10 steps
        before = mode.state
        delta = rng.normal(size=3)
        delta *= 0.002 / np.linalg.norm(delta)  # the same size: a reversal is not larger
        result = mode.step(delta, before.q_cmd)
        if result.status is Status.OK:
            expected = ik.solve_ik(*(before.target + delta), before.q_cmd,
                                   pitch=before.pitch_ref, roll=before.roll_ref)
            assert mode.state.q_cmd == pytest.approx(expected, abs=1e-9)
            accepted += 1
        elif result.status in (Status.REFUSED, Status.LIMITED_DONE):
            mode = _mode(max_reversals=None)
    assert accepted >= 100


# -------------------------------------------------------------------------------- boundaries


def _walk(mode, delta, count):
    """Send the same difference up to count times; return the first status that is not OK."""
    for _ in range(count):
        result = _send(mode, delta)
        if result.status is not Status.OK:
            return result
    return None


def test_each_boundary_shortens_the_difference_to_its_inside_limit():
    cases = [
        ('FLOOR', {}, [0.0, 0.0, -0.004]),
        ('DISTANCE', {}, [0.0, 0.005, 0.0]),
        ('JOINT_RANGE', {}, [0.0, 0.0, 0.003]),
        ('SINGULAR', {'max_abs_d': 0.30}, [-0.002, 0.0, 0.0]),
        ('RADIAL', {'min_radial': 0.13}, [-0.002, 0.0, 0.0]),
    ]
    for kind, kwargs, delta in cases:
        mode = _mode(**kwargs)
        result = _walk(mode, delta, 30)
        assert result is not None and kind in result.limits, kind
        if kind in ('FLOOR', 'DISTANCE', 'JOINT_RANGE'):
            assert result.status is Status.AT_LIMIT, kind  # the room left is above 0.5 mm
        assert result.status in (Status.AT_LIMIT, Status.HOLD), kind
        s = mode.state
        geometry = ik.ik_geometry(*s.target, s.pitch_ref, s.roll_ref)
        cfg = mode.config
        assert s.target[2] >= cfg.floor_z + cfg.margin_floor - 1e-9
        inside = cfg.max_distance - cfg.margin_distance + 1e-9
        assert np.linalg.norm(s.target - s.target0) <= inside
        assert abs(geometry.cos_elbow) <= cfg.max_abs_d - cfg.margin_abs_d + 1e-9
        assert geometry.radial >= cfg.min_radial + cfg.margin_radial - 1e-9
        assert np.all(s.q_cmd <= ik.Q_MAX - INSIDE + 1e-9)
        assert np.all(s.q_cmd >= ik.Q_MIN + INSIDE - 1e-9)
        # the boundary is reached (nothing but a 1 um margin or a 1e-3 margin is left) and holds
        again = _send(mode, delta)
        assert again.status is Status.HOLD and kind in again.limits, kind
        assert not mode.state.halted
    # UNREACHABLE: the IK has no solution (a target on the pan axis); it is a boundary too
    mode = _mode()
    axis = np.array([ik.CONSTANTS.pan_x, 0.0, 0.10])
    base = mode._excess(mode.state.target)
    assert 'UNREACHABLE' in mode._violated(axis, base)


def test_the_suggestion_predicts_what_step_sends():
    mode = _mode()
    for _ in range(2):
        _send(mode, [0.0, 0.0, -0.004])
    goal = mode.state.target + np.array([0.0, 0.0, -0.02])
    q = mode.state.q_cmd
    suggestion = mode.suggest_delta(goal, q)
    assert suggestion.status is Status.AT_LIMIT and 'FLOOR' in suggestion.limits
    assert suggestion.shrunk and suggestion.requested_norm == pytest.approx(0.005)
    result = mode.step(suggestion.delta, q, suggestion.requested_norm)
    assert result.status is Status.AT_LIMIT
    assert result.delta == pytest.approx(suggestion.delta, abs=1e-12)
    assert mode.state.target[2] == pytest.approx(0.048 + 1e-6, abs=1e-8)


def test_shortening_keeps_the_direction_and_never_overshoots():
    rng = np.random.default_rng(17)
    mode = _mode(max_distance=0.012, max_reversals=None)
    checked = 0
    for count in range(200):
        if count % 8 == 7:
            mode = _mode(max_distance=0.012, max_reversals=None)
        delta = rng.normal(size=3)
        delta *= 0.003 / np.linalg.norm(delta)
        result = _send(mode, delta)
        if result.status in (Status.REFUSED, Status.LIMITED_DONE):
            mode = _mode(max_distance=0.012, max_reversals=None)
            continue
        if result.status is Status.AT_LIMIT:
            sent = result.delta
            assert float(np.dot(sent, delta)) > 0.0
            assert np.linalg.norm(np.cross(sent, delta)) <= 1e-12
            assert np.linalg.norm(sent) <= np.linalg.norm(delta) + 1e-15
            checked += 1
        s = mode.state
        assert np.linalg.norm(s.target - s.target0) <= 0.012 - 1e-6 + 1e-9
        assert s.target[2] >= 0.048 + 1e-6 - 1e-9
    assert checked > 3


def test_no_room_inside_the_boundary_holds_without_halting():
    mode = _mode()
    result = _walk(mode, [0.0, 0.0, -0.004], 10)
    assert result.status is Status.AT_LIMIT
    before = _snapshot(mode)
    held = _send(mode, [0.0, 0.0, -0.004])
    assert held.status is Status.HOLD and held.reason == 'DELTA_TOO_SMALL'
    assert 'FLOOR' in held.limits
    assert _snapshot(mode) == before  # nothing sent, nothing counted
    assert not mode.state.halted and mode.state.steps == before[3]
    assert _send(mode, [0.0, 0.0, 0.002]).status is Status.OK  # it can move away


def test_several_boundaries_use_the_largest_fraction_that_satisfies_all():
    z_room = T_XYZ[2] - 0.048  # the floor boundary: 10.77 mm below T (inside limit + 1 um)
    tie = _mode(max_distance=z_room)  # the distance boundary is at the same place
    result = _walk(tie, [0.0, 0.0, -0.004], 10)
    assert result.status is Status.AT_LIMIT
    assert set(result.limits) == {'FLOOR', 'DISTANCE'}
    assert tie.state.target[2] == pytest.approx(0.048 + 1e-6, abs=1e-8)
    near = _mode(max_distance=0.008)  # the distance (8 mm) comes before the floor (10.77 mm)
    result = _walk(near, [0.0, 0.0, -0.004], 10)
    assert result.status is Status.AT_LIMIT and result.limits == ('DISTANCE',)
    assert T_XYZ[2] - near.state.target[2] == pytest.approx(0.008 - 1e-6, abs=1e-8)
    far = _mode(max_distance=0.05)  # only the floor
    result = _walk(far, [0.0, 0.0, -0.004], 10)
    assert result.limits == ('FLOOR',)


def test_a_joint_in_the_margin_band_does_not_approach_its_limit_and_the_others_move():
    rng = np.random.default_rng(23)
    directions = []
    while len(directions) < 300:
        v = rng.normal(size=3)
        directions.append(v / np.linalg.norm(v))  # every pose is 12 mm or more above the floor
    for (joint, end), pose in sorted(BAND_POSES.items()):
        q = np.array(pose)
        toward = 1.0 if end else -1.0  # the sign of a change that approaches the end
        xyz, pitch, roll = ik.fk(q)
        moves = []
        for v in directions:
            target = xyz + 0.0015 * v
            try:
                candidate = ik.ik_geometry(*target, pitch, roll).candidates[dm.branch_of_q(q)]
            except ik.IKError:
                continue
            change = toward * (candidate[joint] - q[joint])
            if dm.wrapped_norm(candidate, q) < 0.02:
                moves.append((change, 0.0015 * v))
        assert moves, (joint, end)
        if joint == 4:
            # the roll is an input of the IK: it never changes, so its band never blocks a move
            mode = DiffMode.start(q, q, DiffModeConfig())
            result = mode.step(moves[0][1], q)
            assert result.status is Status.OK and 'JOINT_RANGE' not in result.limits
            continue
        away = [m for m in moves if m[0] < -1e-5]
        closer = [m for m in moves if m[0] > 1e-5]
        assert away and closer, (joint, end)
        mode = DiffMode.start(q, q, DiffModeConfig())
        result = mode.step(away[0][1], q)  # away from the end: normal
        assert result.status is Status.OK, (joint, end, result.reason, result.limits)
        assert toward * (mode.state.q_cmd[joint] - q[joint]) < 0.0
        mode = DiffMode.start(q, q, DiffModeConfig())
        result = mode.step(closer[0][1], q)  # towards the end: it does not get closer
        assert result.status in (Status.HOLD, Status.AT_LIMIT), (joint, end)
        assert 'JOINT_RANGE' in result.limits
        assert toward * (mode.state.q_cmd[joint] - q[joint]) <= 1e-9
        assert ik.within_limits(mode.state.q_cmd)


def test_max_fraction_finds_the_boundary_and_can_skip_a_thin_band():
    s, high = dm.max_fraction(lambda x: x <= 0.3)
    assert s == pytest.approx(0.3, abs=1e-8) and high is not None and high - s < 1e-8
    assert dm.max_fraction(lambda x: True) == (1.0, None)
    s, high = dm.max_fraction(lambda x: False)
    assert s == 0.0 and high is not None and high < 1e-8
    s, high = dm.max_fraction(lambda x: not 0.40 <= x <= 0.50)
    assert s == pytest.approx(0.4, abs=1e-8)  # a wide band is found
    # a band thinner than 1 / 16 is skipped by the scan (the sent point is re-checked anyway)
    assert dm.max_fraction(lambda x: not 0.40 <= x <= 0.41) == (1.0, None)


# ------------------------------------------------------------------------------ suggestions


def test_the_dead_band_has_a_hysteresis():
    mode = _mode()
    goal = T_XYZ + np.array([0.002, 0.0, 0.0])
    inside = mode.suggest_delta(goal, T_Q)
    assert inside.status is Status.IN_BAND and inside.outcome.kind is dm.Kind.CONVERGED
    assert inside.outcome.within_tolerance and inside.delta is None
    near = mode.suggest_delta(T_XYZ + np.array([0.004, 0.0, 0.0]), T_Q)
    assert near.status is Status.IN_BAND  # 4 mm: it was in the band, the exit is 5 mm
    out = mode.suggest_delta(T_XYZ + np.array([0.006, 0.0, 0.0]), T_Q)
    assert out.status is Status.OK
    again = mode.suggest_delta(T_XYZ + np.array([0.004, 0.0, 0.0]), T_Q)
    assert again.status is not Status.IN_BAND  # 4 mm is outside after it left the band
    fresh = _mode().suggest_delta(T_XYZ + np.array([0.004, 0.0, 0.0]), T_Q)
    assert fresh.status is not Status.IN_BAND  # never in the band: the entry is 3 mm


def test_the_difference_is_clipped_to_max_delta_and_shortened_by_dq():
    mode = _mode(max_dq=0.03)
    q = mode.state.q_cmd
    goal = T_XYZ + np.array([0.008, 0.0, 0.0])
    suggestion = mode.suggest_delta(goal, q)
    assert suggestion.status is Status.OK and suggestion.shrunk
    assert suggestion.requested_norm == pytest.approx(0.005)  # 8 mm is clipped to 5 mm
    assert suggestion.delta[1:] == pytest.approx([0.0, 0.0], abs=1e-12)  # the direction is kept
    assert suggestion.delta[0] == pytest.approx(0.002283, abs=2e-5)  # 0.9 x 0.03 rad
    result = mode.step(suggestion.delta, q, suggestion.requested_norm)
    assert result.status is Status.OK
    assert dm.wrapped_norm(result.q_cmd, q) == pytest.approx(0.027, abs=1e-6)
    z_goal = T_XYZ + np.array([0.0, 0.0, 0.008])
    z = _mode(max_dq=0.03).suggest_delta(z_goal, q)
    assert z.status is Status.OK and z.shrunk and z.delta[2] == pytest.approx(0.00267, abs=3e-5)
    plain = _mode().suggest_delta(T_XYZ + np.array([0.0, 0.004, 0.0]), q)
    assert plain.status is Status.OK and not plain.shrunk
    assert plain.delta == pytest.approx([0.0, 0.004, 0.0], abs=1e-12)


def test_a_tiny_shortened_difference_holds_and_continues():
    mode = _mode(max_dq=0.03, min_delta=0.0029)
    q = mode.state.q_cmd
    before = _snapshot(mode)
    suggestion = mode.suggest_delta(T_XYZ + np.array([0.008, 0.0, 0.0]), q)
    assert suggestion.status is Status.HOLD and suggestion.reason == 'DELTA_TOO_SMALL'
    assert suggestion.delta is None
    assert not mode.state.halted  # a hold is not a stop
    after = _snapshot(mode)
    assert after[2] == before[2] and after[3] == before[3]  # nothing was sent
    assert mode.state.holds == 1 and mode.state.hold_run == 1


def test_a_different_direction_can_move_after_a_hold():
    mode = _mode(max_dq=0.03, min_delta=0.0029)
    q = mode.state.q_cmd
    assert mode.suggest_delta(T_XYZ + [0.008, 0.0, 0.0], q).status is Status.HOLD
    other = mode.suggest_delta(T_XYZ + [0.0, 0.008, 0.0], q)  # y: 0.0073 rad/mm
    assert other.status is Status.OK and other.shrunk and other.delta[1] > 0.0029
    result = mode.step(other.delta, q, other.requested_norm)
    assert result.status is Status.OK and mode.state.hold_run == 0
    assert not mode.state.halted


def test_the_trial_solves_do_not_change_the_state():
    mode = _mode(max_dq=0.03)
    q = mode.state.q_cmd
    before = _snapshot(mode)
    for goal in (T_XYZ + [0.008, 0.0, 0.0], T_XYZ + [0.0, 0.0, -0.02], T_XYZ + [0.0, 0.01, 0.0]):
        suggestion = mode.suggest_delta(goal, q)
        assert suggestion.status in (Status.OK, Status.AT_LIMIT)
        assert _snapshot(mode) == before
    assert not mode.state.halted


def test_refused_suggestions_halt_the_mode():
    cases = [
        ('nan goal', {'goal_xyz': [np.nan, 0.0, 0.0], 'q_meas': T_Q}, 'NOT_FINITE'),
        ('no measurement', {'goal_xyz': T_XYZ, 'q_meas': None}, 'NO_MEASUREMENT'),
        ('nan measurement', {'goal_xyz': T_XYZ, 'q_meas': np.full(5, np.nan)}, 'NOT_FINITE'),
        ('zero gain', {'goal_xyz': T_XYZ, 'q_meas': T_Q, 'gain': 0.0}, 'BAD_REQUEST'),
    ]
    for name, kwargs, reason in cases:
        mode = _mode()
        suggestion = mode.suggest_delta(**kwargs)
        assert suggestion.status is Status.REFUSED and suggestion.reason == reason, name
        assert mode.state.halted, name
        assert mode.suggest_delta(T_XYZ, T_Q).reason == 'HALTED'


# ----------------------------------------------------------------------- spin and the budget


def _hold_mode(**kwargs):
    return _mode(max_dq=0.03, min_delta=0.0029, **kwargs)


def test_repeated_holds_end_the_mode_normally():
    mode = _hold_mode()
    q = mode.state.q_cmd
    goal = T_XYZ + [0.008, 0.0, 0.0]
    assert mode.suggest_delta(goal, q).status is Status.HOLD
    ended = mode.suggest_delta(goal, q)
    assert ended.status is Status.LIMITED_DONE and ended.reason == 'NO_ROOM'
    outcome = ended.outcome
    assert outcome.kind is dm.Kind.LIMITED and outcome.ok and not outcome.within_tolerance
    assert outcome.limit_reason == 'NO_ROOM'
    assert mode.state.finished and not mode.state.halted and mode.state.steps == 0


def test_repeated_boundary_steps_end_the_mode():
    mode = _mode(max_at_limit_run=1)
    goal = T_XYZ + [0.0, 0.0, -0.02]
    for _ in range(10):
        q = mode.state.q_cmd
        suggestion = mode.suggest_delta(goal, q)
        if suggestion.status is Status.LIMITED_DONE:
            break
        assert suggestion.status in (Status.OK, Status.AT_LIMIT)
        result = mode.step(suggestion.delta, q, suggestion.requested_norm)
        assert result.status in (Status.OK, Status.AT_LIMIT)
    assert suggestion.status is Status.LIMITED_DONE and suggestion.reason == 'SPIN'
    assert suggestion.outcome.limit_reason == 'SPIN' and 'FLOOR' in suggestion.outcome.limits
    assert mode.state.at_limit_run == 1 and not mode.state.halted


def test_holds_in_total_end_the_mode():
    mode = _hold_mode(max_hold_run=None, max_holds=2)
    plus = ik.solve_ik(*(T_XYZ + [0.008, 0.0, 0.0]), T_Q, pitch=T_PITCH, roll=T_ROLL)
    minus = ik.solve_ik(*(T_XYZ - [0.008, 0.0, 0.0]), T_Q, pitch=T_PITCH, roll=T_ROLL)
    # the same goal; the measured position is on either side, so the directions alternate
    assert mode.suggest_delta(T_XYZ, minus).status is Status.HOLD
    ended = mode.suggest_delta(T_XYZ, plus)
    assert ended.status is Status.LIMITED_DONE
    assert mode.state.hold_run == 1 and mode.state.holds == 2


def test_a_new_goal_or_a_different_direction_releases_the_spin():
    mode = _hold_mode()
    q = mode.state.q_cmd
    goal = T_XYZ + [0.008, 0.0, 0.0]
    mode.suggest_delta(goal, q)
    assert mode.suggest_delta(goal, q).status is Status.LIMITED_DONE
    new_goal = mode.suggest_delta(T_XYZ + [0.0, 0.008, 0.0], q)  # more than 5 mm away
    assert new_goal.status is Status.OK and mode.state.holds == 0
    mode = _hold_mode()
    plus = ik.solve_ik(*(T_XYZ + [0.008, 0.0, 0.0]), T_Q, pitch=T_PITCH, roll=T_ROLL)
    minus = ik.solve_ik(*(T_XYZ - [0.008, 0.0, 0.0]), T_Q, pitch=T_PITCH, roll=T_ROLL)
    mode.suggest_delta(T_XYZ, minus)
    assert mode.suggest_delta(T_XYZ, minus).status is Status.LIMITED_DONE
    released = mode.suggest_delta(T_XYZ, plus)  # the same goal, the opposite direction
    assert released.status is Status.HOLD and mode.state.hold_run == 1 and not mode.state.finished


def test_a_finished_mode_does_nothing_and_repeats_its_result():
    mode = _hold_mode()
    q = mode.state.q_cmd
    goal = T_XYZ + [0.008, 0.0, 0.0]
    mode.suggest_delta(goal, q)
    first = mode.suggest_delta(goal, q)
    snapshot = _snapshot(mode)
    for _ in range(3):
        again = mode.suggest_delta(goal, q)
        assert again.status is Status.LIMITED_DONE
        assert again.outcome.residual == pytest.approx(first.outcome.residual, abs=1e-15)
        assert _snapshot(mode) == snapshot  # the counters do not move


def test_holds_are_not_counted_as_steps():
    mode = _hold_mode(max_hold_run=None, max_holds=None)
    q = mode.state.q_cmd
    for _ in range(8):
        assert mode.suggest_delta(T_XYZ + [0.008, 0.0, 0.0], q).status is Status.HOLD
    assert mode.state.steps == 0 and mode.state.holds == 8


def test_the_step_budget_ends_the_mode_and_a_new_goal_does_not_release_it():
    mode = _mode(max_steps=2)
    assert _send(mode, [0.0, 0.002, 0.0]).status is Status.OK
    assert _send(mode, [0.0, 0.002, 0.0]).status is Status.OK
    held = mode.state.q_cmd.copy()
    done = _send(mode, [0.0, 0.002, 0.0])
    assert done.status is Status.LIMITED_DONE and done.reason == 'MAX_STEPS'
    assert mode.state.finished and not mode.state.halted
    for goal in (T_XYZ + [0.0, 0.02, 0.0], T_XYZ + [0.05, 0.0, 0.0]):  # a new goal
        suggestion = mode.suggest_delta(goal, mode.state.q_cmd)
        assert suggestion.status is Status.LIMITED_DONE and suggestion.reason == 'MAX_STEPS'
        assert suggestion.outcome.limit_reason == 'MAX_STEPS'
    assert mode.state.q_cmd == pytest.approx(held, abs=1e-15) and mode.state.steps == 2


def test_the_reversal_budget_ends_the_mode_without_sending_the_reversing_step():
    mode = _mode(max_reversals=2)
    assert _send(mode, [0.0, 0.002, 0.0]).status is Status.OK
    assert _send(mode, [0.0, -0.002, 0.0]).status is Status.OK  # the first reversal
    held = mode.state.q_cmd.copy()
    done = _send(mode, [0.0, 0.002, 0.0])  # the second reversal: not sent
    assert done.status is Status.LIMITED_DONE and done.reason == 'REVERSAL'
    assert mode.state.reversals == 1 and mode.state.steps == 2
    assert mode.state.q_cmd == pytest.approx(held, abs=1e-15) and not mode.state.halted
    assert _send(mode, [0.0, 0.001, 0.0]).status is Status.LIMITED_DONE


# --------------------------------------------------------------------------------- outcome


def test_the_outcome_of_a_converged_goal():
    mode = _mode()
    goal = T_XYZ + [0.002, 0.0, 0.0]
    measured = _at_height(T_XYZ[2] + 0.001)
    outcome = mode.outcome(goal, measured)
    assert outcome.kind is dm.Kind.CONVERGED and outcome.ok and outcome.within_tolerance
    assert outcome.limit_reason is None
    assert outcome.reached_xyz == pytest.approx(ik.fk(measured)[0], abs=1e-15)
    assert outcome.residual == pytest.approx(goal - ik.fk(measured)[0], abs=1e-15)
    assert outcome.residual_norm == pytest.approx(np.linalg.norm(outcome.residual))
    assert outcome.commanded_xyz == pytest.approx(T_XYZ, abs=1e-12)


def test_the_outcome_of_a_goal_beyond_a_boundary_splits_the_error():
    mode = _mode()
    _walk(mode, [0.0, 0.0, -0.004], 10)  # the floor boundary
    goal = T_XYZ + [0.0, 0.0, -0.025]  # 25 mm below T, 14 mm below the floor boundary
    q_cmd = mode.state.q_cmd
    behind = _at_height(mode.state.target[2] + 0.002)  # the measured tip is 2 mm behind
    outcome = mode.outcome(goal, behind)
    assert outcome.kind is dm.Kind.LIMITED and outcome.limit_reason == 'FLOOR'
    assert outcome.limits == ('FLOOR',)
    assert outcome.limit_gap == pytest.approx(goal - mode.state.target, abs=1e-15)
    assert outcome.track_gap == pytest.approx(mode.state.target - ik.fk(behind)[0], abs=1e-15)
    assert outcome.residual == pytest.approx(outcome.limit_gap + outcome.track_gap, abs=1e-15)
    assert outcome.limit_gap[2] == pytest.approx(goal[2] - (0.048 + 1e-6), abs=1e-8)
    assert outcome.track_gap[2] == pytest.approx(-0.002, abs=1e-6)
    assert mode.state.q_cmd == pytest.approx(q_cmd)  # reading the outcome changes nothing


def test_a_limited_end_is_a_result_not_an_error():
    mode = _hold_mode()
    q = mode.state.q_cmd
    goal = T_XYZ + [0.008, 0.0, 0.0]
    mode.suggest_delta(goal, q)
    ended = mode.suggest_delta(goal, q)
    assert ended.status is Status.LIMITED_DONE and ended.status is not Status.REFUSED
    assert ended.outcome.kind is dm.Kind.LIMITED and ended.outcome.ok
    assert not mode.state.halted and mode.state.reason is None
    assert ended.outcome.residual_norm == pytest.approx(0.008, abs=1e-9)


# ------------------------------------------------------------------- reversal and splitting


def test_a_reversal_is_counted_by_the_direction_of_the_sent_difference():
    mode = _mode()
    for delta in ([0.0, 0.002, 0.0], [0.0, 0.002, 0.0], [0.0, 0.001, 0.001]):
        _send(mode, delta)
    assert mode.state.reversals == 0
    assert _send(mode, [0.0, -0.001, 0.0]).status is Status.OK
    assert mode.state.reversals == 1


def test_a_reversing_step_larger_than_the_previous_one_is_refused():
    mode = _mode()
    _send(mode, [0.0, 0.002, 0.0])
    result = _send(mode, [0.0, -0.003, 0.0])
    assert result.status is Status.REFUSED and result.reason == 'REVERSAL_GROWTH'
    assert mode.state.halted


def test_unlimited_reversals_do_not_end_the_mode():
    mode = _mode(max_reversals=None)
    for i in range(8):
        sign = 1.0 if i % 2 == 0 else -1.0
        assert _send(mode, [0.0, sign * 0.001, 0.0]).status is Status.OK
    assert mode.state.reversals == 7 and mode.state.steps == 8


def test_a_split_goal_is_sent_in_pieces_that_are_neither_reversals_nor_free():
    mode = _mode(max_dq=0.03)
    goal = T_XYZ + [0.008, 0.0, 0.0]
    pieces = []
    for _ in range(10):
        q = mode.state.q_cmd
        suggestion = mode.suggest_delta(goal, q)
        if suggestion.status is Status.IN_BAND:
            break
        assert suggestion.status is Status.OK and suggestion.shrunk
        assert float(np.dot(suggestion.delta, goal - mode.state.target)) > 0.0
        result = mode.step(suggestion.delta, q, suggestion.requested_norm)
        assert result.status is Status.OK
        pieces.append(result.delta)
    assert len(pieces) >= 3
    assert mode.state.steps == len(pieces) and mode.state.reversals == 0
    assert mode.state.holds == 0  # nothing was held


def test_a_reversal_after_a_shortened_step_is_compared_with_the_size_before_shortening():
    def run(requested):
        mode = _mode()
        mode.step([0.0, 0.002, 0.0], T_Q, requested)
        return mode.step([0.0, -0.004, 0.0], mode.state.q_cmd)

    ok = run(0.005)  # the previous step was shortened from 5 mm: a 4 mm reversal is allowed
    assert ok.status is Status.OK
    refused = run(None)  # the previous step was 2 mm as requested: 4 mm is larger
    assert refused.status is Status.REFUSED and refused.reason == 'REVERSAL_GROWTH'


# ------------------------------------------------------------------------------------ config


def test_the_config_checks_the_hard_ceilings_and_the_relations():
    bad = [
        {'max_dq': 0.06}, {'max_dq': 0.0}, {'shrink_ratio': 0.96}, {'max_abs_d': 0.995},
        {'max_windup': 0.09}, {'max_distance': 0.06}, {'max_steps': 13}, {'max_steps': 0},
        {'max_steps': True}, {'floor_z': 0.044}, {'measured_floor_z': 0.044},
        {'min_radial': 0.04}, {'margin_joint': 0.06}, {'margin_abs_d': 0.02},
        {'margin_radial': 0.02}, {'margin_floor': 0.01}, {'margin_distance': 0.04},
        {'min_delta': 0.003}, {'deadband_exit': 0.002}, {'max_reversals': 0},
        {'max_dq': float('nan')}, {'hold_cos': 0.96}, {'hold_cos': -1.0}, {'max_delta': 0.0},
    ]
    for kwargs in bad:
        with pytest.raises(ValueError):
            DiffModeConfig(**kwargs)
    # the hard ceilings themselves are accepted
    DiffModeConfig(max_dq=0.05, shrink_ratio=0.95, max_abs_d=0.99, max_windup=0.08,
                   max_distance=0.05, max_steps=12, floor_z=0.045, measured_floor_z=0.045,
                   min_radial=0.05, margin_joint=0.05, margin_abs_d=0.01, margin_radial=0.01,
                   margin_floor=1e-3, margin_distance=1e-3)
    # a branch switch always exceeds the dq limit at the ceilings (5.6 times or more)
    assert 2.0 * math.acos(dm.HARD_MAX_ABS_D) > 5.6 * dm.HARD_MAX_DQ


def test_the_spin_and_reversal_limits_may_be_none_but_not_above_their_ceilings():
    DiffModeConfig(max_hold_run=None, max_at_limit_run=None, max_holds=None, max_reversals=None)
    for kwargs in ({'max_hold_run': 4}, {'max_at_limit_run': 5}, {'max_holds': 9},
                   {'max_hold_run': 0}, {'max_holds': 1.5}):
        with pytest.raises(ValueError):
            DiffModeConfig(**kwargs)
    DiffModeConfig(max_hold_run=3, max_at_limit_run=4, max_holds=8)
