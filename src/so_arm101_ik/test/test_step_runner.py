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

"""The pure helpers of step_runner (P5): stop, freshness, plans, the goal for MoveIt, options."""

import dataclasses
import inspect

import numpy as np
import pytest
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik import diff_mode as dm
from so_arm101_ik import step_runner as sr

NAMES = list(sr.JOINT_NAMES)
Q = np.radians([0.0, -22.0, 30.0, 83.0, 0.0])


def _points(q_first, q_last, duration=2.0, velocities=None, count=5):
    q_first = np.asarray(q_first, float)
    q_last = np.asarray(q_last, float)
    out = []
    for i in range(count):
        a = i / (count - 1)
        v = None if velocities is None else tuple(velocities)
        out.append(sr.TrajPoint(duration * a, tuple(q_first + a * (q_last - q_first)), v))
    return out


def _check(points=None, names=None, cmd=None, desired=None, meas=None, **kwargs):
    cmd = Q + 0.02 if cmd is None else cmd
    desired = Q if desired is None else desired
    meas = desired if meas is None else meas
    points = _points(desired, cmd) if points is None else points
    return sr.check_planned_trajectory(points, NAMES if names is None else names, cmd, desired,
                                       meas, **kwargs)


# ------------------------------------------------------------------------------ small helpers

def test_is_stopped_uses_the_tolerance_of_command_a():
    a = Q.copy()
    assert sr.is_stopped(a, Q + np.array([0, 0, 0.00299, 0, 0]))  # the limit is 0.003 rad
    assert not sr.is_stopped(a, Q + np.array([0, 0, 0.0031, 0, 0]))
    assert not sr.is_stopped(a, [0.0, 0.0, 0.0, 0.0])
    assert not sr.is_stopped(a, Q + np.array([np.nan, 0, 0, 0, 0]))


def test_positions_by_name_keeps_the_order_and_rejects_missing_or_bad_values():
    names = ['gripper', 'wrist_roll', 'elbow_flex', 'shoulder_pan', 'wrist_flex', 'shoulder_lift']
    values = [9.0, 5.0, 3.0, 1.0, 4.0, 2.0]
    assert list(sr.positions_by_name(names, values)) == [1.0, 2.0, 3.0, 4.0, 5.0]
    assert sr.positions_by_name(names[:-1], values[:-1]) is None  # shoulder_lift is missing
    assert sr.positions_by_name(names, values[:-1]) is None
    assert sr.positions_by_name(names, [9.0, 5.0, float('nan'), 1.0, 4.0, 2.0]) is None


def test_freshness_of_an_input():
    assert sr.is_fresh(10.0, 10.29) and not sr.is_fresh(10.0, 10.31)  # the limit is 0.3 s
    assert not sr.is_fresh(None, 10.0) and not sr.is_fresh(10.0, 9.0)  # never from the future
    assert not sr.is_fresh(float('nan'), 10.0)


def test_the_tolerance_of_desired_depends_on_the_route():
    assert sr.desired_tolerance('direct') == pytest.approx(1e-4)
    assert sr.desired_tolerance('moveit') == pytest.approx(0.002)  # twice the 0.001 default
    assert sr.desired_tolerance('moveit', 0.004) == pytest.approx(0.008)
    with pytest.raises(ValueError):
        sr.desired_tolerance('servo')
    shifted = Q + np.array([0, 0, 0, 1.5e-4, 0])
    assert not sr.desired_matches(shifted, Q, 'direct')
    assert sr.desired_matches(shifted, Q, 'moveit')
    assert not sr.desired_matches(Q + np.array([0, 0, 0, 0.0025, 0]), Q, 'moveit')
    assert not sr.desired_matches(None, Q, 'direct')


def test_the_origin_sphere():
    tip = ik.fk(Q)[0]
    assert sr.check_origin(None, Q) is None  # no origin: this run's start is the origin
    assert sr.check_origin(tip + [0.049, 0, 0], Q, 0.05) is None
    assert sr.check_origin(tip + [0.051, 0, 0], Q, 0.05) is not None
    assert sr.check_origin(tip, [1.0, 2.0], 0.05) is not None


def test_the_speed_check_of_a_direct_move():
    a = np.zeros(5)
    assert sr.check_direct_speed(a, a + [0, 0, 0.2, 0, 0], 2.0) is None  # exactly 0.1 rad/s
    assert sr.check_direct_speed(a, a + [0, 0, 0.21, 0, 0], 2.0) is not None
    assert sr.check_direct_speed(a, a + 0.01, 0.0) is not None
    assert sr.direct_speed(a, a + [0, 0.03, 0, 0, 0], 2.0) == pytest.approx(0.015)


# ------------------------------------------------------------------------------ planned trajectory

def test_the_speeds_of_a_trajectory_come_from_the_positions_and_from_the_velocities():
    pos, vel = sr.trajectory_speeds(_points(Q, Q + [0, 0.1, 0, 0, 0], duration=1.0))
    assert pos == pytest.approx(0.1) and vel == 0.0
    pos, vel = sr.trajectory_speeds(_points(Q, Q, velocities=[0, 0.3, 0, 0, -0.4]))
    assert pos == 0.0 and vel == pytest.approx(0.4)


def test_a_good_plan_passes_and_its_numbers_are_reported():
    check = _check()
    assert check.ok and check.reasons == ()
    assert check.points == 5 and check.duration == pytest.approx(2.0)
    assert check.peak_pos_speed == pytest.approx(0.02 / 2.0)
    assert check.first_vs_desired == pytest.approx(0.0) and check.last_vs_cmd == pytest.approx(0.0)


def test_each_check_of_a_plan_can_forbid_the_execution():
    cmd = Q + 0.02
    cases = {
        'FIRST_DESIRED over': (dict(points=_points(Q + [0, 0, 0.0051, 0, 0], cmd)),
                               'FIRST_DESIRED'),
        'FIRST_MEASURED over': (dict(points=_points(Q + [0, 0, 0.0101, 0, 0], cmd),
                                     desired=Q + np.array([0, 0, 0.0101, 0, 0]),
                                     meas=Q), 'FIRST_MEASURED'),
        'LAST': (dict(points=_points(Q, cmd + [0, 0, 0.0011, 0, 0])), 'LAST'),
        'SPEED by positions': (dict(points=_points(Q, cmd, duration=0.01)), 'SPEED'),
        'SPEED by velocities': (dict(points=_points(Q, cmd, velocities=[0.6, 0, 0, 0, 0])),
                                'SPEED'),
        'NAMES': (dict(names=NAMES[::-1]), 'NAMES'),
        'NOT_FINITE point': (dict(points=_points(Q, cmd + [np.nan, 0, 0, 0, 0])), 'NOT_FINITE'),
        'NOT_FINITE cmd': (dict(cmd=Q + [np.nan, 0, 0, 0, 0]), 'NOT_FINITE'),
        'EMPTY': (dict(points=[]), 'EMPTY'),
        'ONE POINT': (dict(points=_points(Q, cmd)[:1]), 'EMPTY'),
        'ZERO LENGTH': (dict(points=_points(Q, cmd, duration=0.0)), 'EMPTY'),
    }
    for name, (kwargs, reason) in cases.items():
        check = _check(**kwargs)
        assert not check.ok and reason in check.reasons, name


def test_the_first_point_is_compared_with_the_desired_and_the_numbers_are_kept():
    # the plan starts at the measured position, 0.0265 rad (17 ticks) away from the desired,
    # as at T of the real arm: it is not executed, and the size of the jump is reported
    desired = Q
    meas = Q + [0, 0, -0.0265, 0, 0]
    check = _check(points=_points(meas, Q + 0.02), desired=desired, meas=meas)
    assert not check.ok and check.reasons == ('FIRST_DESIRED',)
    assert check.first_vs_desired == pytest.approx(0.0265)
    assert check.first_vs_meas == pytest.approx(0.0)


def test_the_two_first_point_checks_hold_together_only_within_0_015_rad():
    for gap, expected in ((0.014, True), (0.0149, True), (0.0152, False), (0.03, False)):
        desired = Q
        meas = Q + [0, 0, gap, 0, 0]
        ok = False
        for fraction in np.linspace(0.0, 1.0, 301):  # every possible first point on the segment
            first = desired + fraction * (meas - desired)
            check = _check(points=_points(first, Q + 0.02), desired=desired, meas=meas)
            ok = ok or check.ok
        assert ok == expected, gap


def test_the_first_point_tolerances_are_the_decided_ones():
    assert sr.FIRST_POINT_DESIRED_TOL == 0.005 and sr.FIRST_POINT_MEAS_TOL == 0.01
    assert sr.MOVEIT_MAX_SPEED == 0.5 and sr.DIRECT_MAX_SPEED == 0.1
    assert sr.MOVEIT_MAX_SPEED < 0.75  # below L1 (max_joint_speed 0.75 rad/s)
    assert sr.MOVEIT_SCALING == 0.1 and sr.CONFIRM_TIMEOUT == 30.0
    assert sr.ORIGIN_RADIUS == sr.ORIGIN_RADIUS_MAX == dm.HARD_MAX_DISTANCE == 0.05


# ------------------------------------------------------------------------------ goal for MoveIt

def test_the_goal_for_move_action_only_plans():
    spec = sr.build_plan_goal_spec(Q, NAMES, 0.001)
    assert spec.plan_only is True
    assert spec.scaling == 0.1 and spec.is_diff is True and spec.group == 'arm'
    assert spec.joint_names == tuple(NAMES) and spec.tolerance == 0.001
    assert np.allclose(spec.positions, Q)
    # plan_only cannot be asked for: no argument, no field, no way to set it
    assert 'plan_only' not in inspect.signature(sr.build_plan_goal_spec).parameters
    assert 'plan_only' not in [f.name for f in dataclasses.fields(sr.PlanGoalSpec)]
    with pytest.raises(dataclasses.FrozenInstanceError):
        spec.scaling = 0.5
    with pytest.raises((AttributeError, dataclasses.FrozenInstanceError)):
        spec.plan_only = False
    assert 'scaling' not in inspect.signature(sr.build_plan_goal_spec).parameters


def test_the_goal_for_move_action_rejects_bad_input():
    bad = [dict(q_cmd=Q[:4]), dict(q_cmd=Q + [np.nan, 0, 0, 0, 0]),
           dict(joint_names=NAMES[::-1]), dict(joint_names=NAMES + ['gripper']),
           dict(tolerance=0.0), dict(tolerance=-0.001), dict(tolerance=float('nan')),
           dict(tolerance=0.011)]
    for kwargs in bad:
        args = dict(q_cmd=Q, joint_names=NAMES, tolerance=0.001)
        args.update(kwargs)
        with pytest.raises(ValueError):
            sr.build_plan_goal_spec(**args)


def test_the_end_of_a_goal_is_not_an_arrival():
    assert sr.classify_goal_result('direct', 4, 0)[0]
    assert sr.classify_goal_result('moveit', 4, 1)[0]
    assert 'not a proof' in sr.classify_goal_result('direct', 4, 0)[1]
    for kind, status, code in (('direct', 6, 0), ('direct', 5, 0), ('direct', 4, -4),
                               ('moveit', 4, 0), ('moveit', 4, -1), ('moveit', 6, 1)):
        assert not sr.classify_goal_result(kind, status, code)[0], (kind, status, code)
    with pytest.raises(ValueError):
        sr.classify_goal_result('servo', 4, 0)


# ------------------------------------------------------------------------------ options

def test_the_combinations_of_the_options():
    ok = dict(route='direct', offset=(0.001, 0, 0))
    config = sr.validate_run_options(**ok)
    assert config.max_dq == 0.03 and config.max_steps == 10 and config.max_abs_d == 0.99
    bad = {
        'no route': dict(ok, route=None),
        'unknown route': dict(ok, route='servo'),
        'assume without dry-run': dict(ok, assume_desired_is_measured=True),
        'goal and offset': dict(ok, goal=(0.1, 0, 0.1)),
        'neither goal nor offset': dict(route='direct'),
        'goal with 2 numbers': dict(route='direct', goal=(0.1, 0.1)),
        'offset with nan': dict(route='direct', offset=(np.nan, 0, 0)),
        'auto 0': dict(ok, auto=0),
        'auto over max_steps': dict(ok, auto=11),
        'auto with dry-run': dict(ok, auto=3, dry_run=True),
        'auto is a float': dict(ok, auto=2.5),
        'origin radius 0.06': dict(ok, origin_radius=0.06),
        'origin radius 0': dict(ok, origin_radius=0.0),
        'origin with 2 numbers': dict(ok, origin=(0.1, 0.1)),
        'max_dq over the hard ceiling': dict(ok, max_dq=0.06),
        'max_abs_d over the hard ceiling': dict(ok, max_abs_d=0.995),
        'max_steps over the hard ceiling': dict(ok, max_steps=13),
        'gain 0': dict(ok, gain=0.0), 'gain 1.5': dict(ok, gain=1.5),
        'moveit tolerance 0': dict(ok, moveit_tolerance=0.0),
        'moveit tolerance 0.02': dict(ok, moveit_tolerance=0.02),
        'move seconds 0': dict(ok, move_seconds=0.0),
    }
    for name, kwargs in bad.items():
        try:
            sr.validate_run_options(**kwargs)
        except ValueError:
            continue
        raise AssertionError('the options were accepted: ' + name)
    # the combinations that are accepted
    sr.validate_run_options('direct', dry_run=True, assume_desired_is_measured=True,
                            offset=(0.001, 0, 0))
    sr.validate_run_options('moveit', goal=(0.1, 0.0, 0.1), auto=10, origin=(0.1, 0, 0.1),
                            origin_radius=0.05)


def test_the_rows_of_the_csv():
    row = sr.step_record(step=1, status='OK', q_cmd=Q, plan_peak_pos_speed=0.2, limits=('A', 'B'))
    assert list(row) == list(sr.CSV_FIELDS)
    assert row['plan_peak_pos_speed'] == '0.2' and row['limits'] == 'A B'
    assert row['q_cmd'].startswith('0.00000 ')
    for name in ('plan_peak_pos_speed', 'plan_peak_vel_speed', 'first_vs_desired', 'first_vs_meas',
                 'last_vs_cmd', 'abs_d', 'cmd_speed', 'track_gap', 'limit_gap', 'windup_after',
                 'tip_move_over_delta'):
        assert name in sr.CSV_FIELDS
    with pytest.raises(ValueError):
        sr.step_record(speed=1.0)


# ------------------------------------------------------------------------------ confirmation

class _Typing:
    """A fake terminal: the chunks that arrive, the time that passes at every poll."""

    def __init__(self, chunks, step=0.05):
        self.chunks = list(chunks)
        self.t = 0.0
        self.step = step
        self.polls = 0

    def now(self):
        return self.t

    def poll(self):
        self.polls += 1
        self.t += self.step
        return self.chunks.pop(0) if self.chunks else None


def _ask(chunks, reader=None, timeout=30.0):
    typing = _Typing(chunks)
    reader = sr.LineReader() if reader is None else reader
    return sr.confirm_answer(reader, typing.poll, typing.now, timeout), typing, reader


def test_only_an_empty_line_or_yes_confirms():
    for text in ('\n', 'y\n', 'yes\n', 'Y\n', ' YES \n'):
        assert _ask([text])[0] == 'yes', repr(text)
    for text in ('n\n', 'no\n', 'x\n', 'hello\n', 'yess\n', '0\n', ' no\n'):
        assert _ask([text])[0] == 'no', repr(text)  # anything but an empty line or yes: not sent


def test_the_end_of_the_input_does_not_confirm():
    assert _ask([''])[0] == 'no'  # EOF with nothing typed
    answer, _, reader = _ask(['n\n', ''])
    assert answer == 'no'
    reader = sr.LineReader()
    reader.feed('')
    assert sr.confirm_answer(reader, lambda: None, lambda: 0.0) == 'no'  # at once, no waiting
    assert _ask(['y', ''])[0] == 'yes'  # the last line without a newline is still a line


def test_no_answer_in_30_seconds_does_not_confirm():
    answer, typing, _ = _ask([])
    assert answer == 'timeout'
    assert 29.9 <= typing.t <= 30.2  # it waited the 30 s of CONFIRM_TIMEOUT, not longer
    assert _ask(['partial typing without an Enter'])[0] == 'timeout'
    assert sr.CONFIRM_TIMEOUT == 30.0
    assert _ask([], timeout=1.0)[1].t <= 1.1


def test_one_enter_confirms_only_one_confirmation_even_when_lines_arrive_together():
    reader = sr.LineReader()
    typing = _Typing(['\n\n\n'])  # three Enter keys in one chunk (a pipe, or typing ahead)
    first = sr.confirm_answer(reader, typing.poll, typing.now)
    assert first == 'yes'
    assert reader.take() == 'yes' and reader.take() == 'yes'  # the other two lines are still there
    assert reader.take() is None  # nothing left: the next confirmation has to wait
    reader = sr.LineReader()
    reader.feed('\n')
    assert sr.confirm_answer(reader, lambda: None, lambda: 0.0) == 'yes'
    assert sr.confirm_answer(reader, lambda: None, iter([0.0, 31.0, 31.0, 31.0]).__next__,
                             30.0) == 'timeout'  # the same single Enter does not answer twice
