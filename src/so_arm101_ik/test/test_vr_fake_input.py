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

"""The real-arm entrance of the fake VR input (examples/vr_fake_input.py): no node is run."""

import importlib.util
from pathlib import Path
import types

import numpy as np
import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'examples' / 'vr_fake_input.py'


@pytest.fixture(scope='module')
def fake():
    pytest.importorskip('rclpy')
    spec = importlib.util.spec_from_file_location('vr_fake_input', SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def args(**changes):
    values = dict(mock=False, real=True, scenarios=['line'], amplitude_mm=10.0, speed_mm_s=10.0,
                  gripper_cycles=1, arm_read_only=False, expect='normal')
    values.update(changes)
    return types.SimpleNamespace(**values)


# --------------------------------------------------------------------- the arguments

def test_the_allowed_real_arguments(fake):
    assert fake.check_args(args(), None) is None
    assert fake.check_args(args(), '5') is None
    for scenario in ('line', 'stale', 'jump', 'gripper'):
        assert fake.check_args(args(scenarios=[scenario]), None) is None
    assert fake.check_args(args(amplitude_mm=30.0, speed_mm_s=20.0, gripper_cycles=3),
                           None) is None
    assert fake.check_args(args(arm_read_only=True, amplitude_mm=60.0, expect='blocked'),
                           None) is None
    assert fake.check_args(args(arm_read_only=True, expect='stationary'), None) is None


@pytest.mark.parametrize('changes, domain', [
    ({'mock': True}, None),  # both
    ({'real': False}, '87'),  # neither
    ({'real': True}, '87'),  # real in the mock domain
    ({'scenarios': None}, None),
    ({'scenarios': []}, None),
    ({'scenarios': ['range']}, None),
    ({'scenarios': ['rotation']}, None),
    ({'scenarios': ['busy']}, None),
    ({'scenarios': ['line', 'range']}, None),
    ({'amplitude_mm': 30.1}, None),
    ({'amplitude_mm': 60.0}, None),  # 60 only for a read_only arm
    ({'amplitude_mm': 0.5}, None),
    ({'amplitude_mm': 60.1, 'arm_read_only': True}, None),
    ({'speed_mm_s': 20.1}, None),
    ({'speed_mm_s': 0.5}, None),
    ({'gripper_cycles': 0}, None),
    ({'gripper_cycles': 4}, None),
    ({'expect': 'blocked'}, None),
    ({'expect': 'stationary'}, None),
])
def test_the_refused_arguments(fake, changes, domain):
    reason = fake.check_args(args(**changes), domain)
    assert reason and isinstance(reason, str)


def test_the_mock_arguments(fake):
    assert fake.check_args(args(mock=True, real=False, scenarios=None), '87') is None
    assert fake.check_args(args(mock=True, real=False), None)  # not the mock domain
    assert fake.check_args(args(mock=True, real=False), '5')


def test_the_limits_are_the_ones_of_the_plan(fake):
    assert fake.REAL_SCENARIOS == ('line', 'stale', 'jump', 'gripper')
    assert fake.REAL_MAX_AMPLITUDE_MM == 30.0 and fake.READ_ONLY_MAX_AMPLITUDE_MM == 60.0
    assert fake.REAL_MAX_SPEED_MM_S == 20.0 and fake.REAL_JUMP_MM == 60.0


# ------------------------------------------------------------------------- the plans

def target_y(plan, scale, t):
    return float(plan.hand(t)[0][1] - fake_base(plan)[1]) * scale


def fake_base(plan):
    return plan.hand(0.0)[0]


@pytest.mark.parametrize('scale', [0.5, 1.0, 2.0])
@pytest.mark.parametrize('amplitude', [0.010, 0.030, 0.060])
@pytest.mark.parametrize('name', ['line', 'stale'])
def test_the_moves_of_the_target_stay_in_the_limits(fake, name, amplitude, scale):
    speed = 0.010
    plan = fake.make_real_plan(name, amplitude, speed, scale)
    times = np.arange(0.0, plan.duration, 0.005)
    ys = np.array([target_y(plan, scale, t) for t in times])
    assert np.max(np.abs(ys)) <= amplitude + 1e-9
    assert np.max(np.abs(np.diff(ys) / 0.005)) <= speed + 1e-6
    assert abs(ys[-1]) < 1e-9  # it ends where it began
    for t in times[::40]:  # only y moves; x and z of the hand stay
        position = plan.hand(t)[0]
        assert position[0] == pytest.approx(0.30) and position[2] == pytest.approx(0.30)


def test_the_line_goes_to_both_sides(fake):
    plan = fake.make_real_plan('line', 0.010, 0.010, 0.5)
    ys = [target_y(plan, 0.5, t) for t in np.arange(0.0, plan.duration, 0.01)]
    assert max(ys) == pytest.approx(0.010, abs=1e-4) and min(ys) == pytest.approx(-0.010, abs=1e-4)
    assert plan.clutch and plan.clutch[0][0] >= 1.0 and plan.clutch[-1][1] < plan.duration
    assert not plan.opens and not plan.closes and not plan.spikes


def test_the_hand_moves_target_over_scale(fake):
    full = fake.make_real_plan('line', 0.010, 0.010, 1.0)
    half = fake.make_real_plan('line', 0.010, 0.010, 0.5)
    t = 1.0 + 0.5
    assert (half.hand(t)[0][1] - 0.0) == pytest.approx(2.0 * (full.hand(t)[0][1] - 0.0))


def test_the_stale_plan_stops_the_pose_longer_than_the_node_waits(fake):
    plan = fake.make_real_plan('stale', 0.010, 0.010, 1.0)
    (start, end), = plan.pose_off
    assert end - start >= 1.0 > 0.5  # the node goes STALE after 0.5 s
    assert target_y(plan, 1.0, start) == pytest.approx(target_y(plan, 1.0, end - 0.001), abs=1e-6)
    assert target_y(plan, 1.0, start) == pytest.approx(0.010, abs=1e-6)  # comes back to the place


@pytest.mark.parametrize('scale', [0.5, 1.0, 2.0])
def test_the_jump_plan_makes_one_step_of_60_mm_of_the_target(fake, scale):
    plan = fake.make_real_plan('jump', 0.010, 0.010, scale)
    times = np.arange(0.0, plan.duration, 0.0001)
    ys = np.array([target_y(plan, scale, t) for t in times])
    assert np.max(ys) == pytest.approx(0.060, abs=1e-6)
    assert ys[-1] == pytest.approx(0.0, abs=1e-6)  # back where the target stopped (within 20 mm)
    assert np.max(ys) - ys[-1] > 0.05  # the way back is a jump of its own: the node is stopped
    assert not plan.opens and not plan.closes


def test_the_gripper_plan_has_only_buttons_that_never_overlap(fake):
    for cycles in (1, 2, 3):
        plan = fake.make_real_plan('gripper', 0.010, 0.010, 0.5, cycles)
        assert not plan.clutch and not plan.disp and not plan.pose_off
        assert len(plan.closes) == cycles and len(plan.opens) == cycles
        for t in np.arange(0.0, plan.duration, 0.01):
            clutch, opened, closed = plan.buttons(t)
            assert not clutch and not (opened and closed)
        assert plan.buttons(plan.closes[0][0] + 0.1) == (False, False, True)
        assert plan.buttons(plan.opens[0][0] + 0.1) == (False, True, False)


def test_an_unknown_real_scenario_is_refused(fake):
    for name in ('range', 'rotation', 'busy', 'nothing'):
        with pytest.raises(ValueError):
            fake.make_real_plan(name, 0.01, 0.01, 1.0)


# ------------------------------------------------------------------- the real checks

def record(states, points=(), amounts=(), goals=(), samples=None, refs=None):
    status = [(t, state, reason, {'jump_mm': 60.0} if state == 'HOLD_JUMP' else {})
              for t, state, reason in states]
    tips = samples if samples is not None else [(t * 0.1, np.array([0.18, 0.0, 0.06]), 0.0, 0.5,
                                                 0.03, np.zeros(5)) for t in range(60)]
    return {'status': status, 'points': list(points), 'amounts': list(amounts),
            'goals': list(goals), 'samples': tips, 'refs': refs or [], 'busy_sent': None}


def results(checks):
    return {label: ok for label, ok, _ in checks if not label.startswith('INFO')}


def test_the_blocked_expectation(fake):
    states = [(0.0, 'IDLE', 'START_WAIT'), (1.0, 'TRACKING', 'OK'),
              (2.0, 'BLOCKED', 'TRACKING_ERROR_0.201_rad'), (4.0, 'IDLE', 'RELEASED')]
    points = [(1.2, [0.0] * 5), (1.5, [0.1] * 5)]
    ok = results(fake.real_checks('line', record(states, points), 'blocked', 0.03))
    assert all(ok.values()), ok
    moved = [(t * 0.1, np.array([0.18 + 0.01 * (t > 30), 0.0, 0.06]), 0.0, 0.5, 0.03, np.zeros(5))
             for t in range(60)]
    bad = results(fake.real_checks('line', record(states, points, samples=moved), 'blocked',
                                   0.03))
    assert not all(bad.values())  # the arm moved: a failed check
    no_block = [(0.0, 'IDLE', 'START_WAIT'), (1.0, 'TRACKING', 'OK'), (4.0, 'IDLE', 'RELEASED')]
    assert not all(results(fake.real_checks('line', record(no_block, points), 'blocked',
                                            0.03)).values())


def test_the_stationary_expectation(fake):
    states = [(0.0, 'IDLE', 'START_WAIT'), (1.0, 'TRACKING', 'OK'), (4.0, 'IDLE', 'RELEASED')]
    points = [(1.5, [0.0] * 5)]
    assert all(results(fake.real_checks('line', record(states, points), 'stationary',
                                        0.03)).values())
    blocked = states[:2] + [(2.0, 'BLOCKED', 'TRACKING_ERROR_0.2_rad')] + states[2:]
    assert not all(results(fake.real_checks('line', record(blocked, []), 'stationary',
                                            0.03)).values())


def test_a_point_outside_tracking_fails_every_check(fake):
    states = [(0.0, 'IDLE', 'START_WAIT'), (1.0, 'TRACKING', 'OK'),
              (2.0, 'HOLD_JUMP', 'TARGET_JUMP'), (4.0, 'IDLE', 'RELEASED')]
    out = results(fake.real_checks('jump', record(states, [(3.0, [0.0] * 5)]), 'normal', 0.01))
    assert not out['no point while the state is not TRACKING (1 points)']


def test_the_jump_check_wants_the_arm_to_stay(fake):
    states = [(0.0, 'IDLE', 'START_WAIT'), (1.0, 'TRACKING', 'OK'),
              (2.0, 'HOLD_JUMP', 'TARGET_JUMP'), (3.0, 'TRACKING', 'OK'),
              (4.0, 'IDLE', 'RELEASED')]
    assert all(results(fake.real_checks('jump', record(states), 'normal', 0.01)).values())
    followed = [(t * 0.1, np.array([0.18, 0.0 + 0.01 * (t > 30), 0.06]), 0.0, 0.5, 0.03,
                 np.zeros(5)) for t in range(60)]
    assert not all(results(fake.real_checks('jump', record(states, samples=followed), 'normal',
                                            0.01)).values())


def test_the_gripper_check_counts_the_commands(fake):
    states = [(0.0, 'IDLE', 'START_WAIT')]
    good = record(states, amounts=[(1.2, 1.0), (3.2, 0.0)], goals=[(1.2, 'a'), (3.2, 'b')])
    assert all(results(fake.real_checks('gripper', good, 'normal', 0.01, 1)).values())
    twice = record(states, amounts=[(1.2, 1.0), (3.2, 0.0), (3.3, 0.0)],
                   goals=[(1.2, 'a'), (3.2, 'b'), (3.3, 'c')])
    assert not all(results(fake.real_checks('gripper', twice, 'normal', 0.01, 1)).values())


# ------------------------------------------------------------ the source (no publisher)

def test_the_real_entrance_never_creates_a_publisher_or_a_client_for_the_arm():
    text = SOURCE.read_text()
    init = text[text.index('    def __init__(self, buttons, real=False):'):text.index(
        '    def _on_status')]
    for creation in ('self.create_publisher(JointTrajectory', 'ActionClient(self, '
                     'FollowJointTrajectory'):
        at = init.index(creation)
        guard = init.rindex('if not real:', 0, at)
        assert 0 < at - guard < 400, creation  # directly under the guard
        assert init[guard:at].count('\n') <= 2
    assert text.count('create_publisher(JointTrajectory') == 1
    assert text.count('send_trajectory(') >= 1


def test_the_real_branch_does_not_prepare_the_arm():
    text = SOURCE.read_text()
    main = text[text.index('def main():'):]
    assert main.count('prepare(') == 1
    assert '    if args.mock:  # only the mock arm is moved' in main
    guard = main.index('    if args.mock:')
    assert main.index('prepare(') - guard < 200 and main.index('prepare(') > guard
    assert 'send_trajectory' not in main
    assert 'confirm(describe(' in main  # every real scenario waits for Enter
    assert 'real_like = args.real or args.real_plans' in main


def test_real_plans_in_the_mock_follow_the_same_limits(fake):
    mock_plans = args(mock=True, real=False, real_plans=True)
    assert fake.check_args(mock_plans, '87') is None
    assert fake.check_args(args(mock=True, real=False, real_plans=True, scenarios=['range']),
                           '87')
    assert fake.check_args(args(mock=True, real=False, real_plans=True, amplitude_mm=31.0),
                           '87')
    assert fake.check_args(args(mock=True, real=False, real_plans=True), None)
    assert fake.check_args(args(real=True, real_plans=True), None)  # only for the mock


@pytest.mark.parametrize('text, expected', [
    ('<param name="read_only">True</param>', True),  # the form of the real robot_description
    ('<param name="read_only">true</param>', True),
    ('<param name="read_only">TRUE</param>', True),
    ('<param name="read_only"> True </param>', True),
    ('<param name="read_only">False</param>', False),
    ('<param name="read_only">false</param>', False),
    ('<param name="mock">True</param>', False),
    ('', False),
])
def test_the_read_only_check_of_the_fake_input(fake, text, expected):
    assert fake.description_is_read_only(text) is expected
