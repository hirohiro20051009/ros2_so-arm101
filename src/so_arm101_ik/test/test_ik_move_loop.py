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

"""MoveLoop (the large move) with a fake io, a fake sender and a fake arm."""

import numpy as np
import pytest
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik import ik_move as mv
from so_arm101_ik import step_runner as sr

T_MEAS = np.array([0.0031, -0.3813, 0.5501, 1.4454, -0.0069])
REST = np.array([-0.0951, -1.4952, 1.6901, 0.4219, -0.0453])
M2 = (0.200, 0.000, 0.060)
NAMES = sr.JOINT_NAMES


class FakeArm:
    """A fake arm: the desired follows the command, the measurement may stop short of it."""

    def __init__(self, q, short=None, frozen=False):
        self.meas = np.array(q, float)
        self.desired = np.array(q, float)
        self.short = np.zeros(5) if short is None else np.array(short, float)
        self.frozen = frozen

    def command(self, q):
        self.desired = np.array(q, float)
        if not self.frozen:
            self.meas = self.desired - self.short


class FakeIO:
    """Inputs, time, confirmation and status of the actions, all under the control of a test."""

    def __init__(self, arm, answers=None, active=None, on_confirm=None, stale=None):
        self.arm = arm
        self.t = 100.0
        self.answers = list(answers or [])
        self.active = active
        self.on_confirm = on_confirm
        self.stale = stale
        self.said = []
        self.rows = []
        self.prompts = []

    def now(self):
        return self.t

    def wait(self, seconds):
        self.t += seconds
        assert self.t < 1100.0, 'the loop does not end'

    def read_inputs(self):
        desired, desired_at = self.arm.desired, self.t
        if self.stale == 'no_desired':
            desired, desired_at = None, None
        meas_at = self.t - 5.0 if self.stale == 'meas' else self.t
        return sr.Inputs(desired, desired_at, self.arm.meas.copy(), meas_at)

    def other_active_goals(self):
        return list(self.active or [])

    def confirm(self, text, timeout):
        self.prompts.append(text)
        if self.on_confirm is not None:
            self.on_confirm(len(self.prompts), self)
        answer = self.answers.pop(0) if self.answers else 'yes'
        if isinstance(answer, BaseException):
            raise answer
        return answer

    def say(self, text):
        self.said.append(text)

    def log_row(self, row):
        self.rows.append(row)


class FakeSender:
    """A sender that plans a straight line and moves the fake arm when it executes."""

    def __init__(self, arm, seconds=6.0, plan_ok=True, build=None, exec_ok=True, on_plan=None,
                 on_execute=None):
        self.arm = arm
        self.seconds = seconds
        self.plan_ok = plan_ok
        self.build = build
        self.exec_ok = exec_ok
        self.on_plan = on_plan
        self.on_execute = on_execute
        self.planned = []
        self.executed = []

    def plan(self, q_goal, tolerance):
        self.planned.append(np.array(q_goal))
        if self.on_plan is not None:
            self.on_plan()
        if not self.plan_ok:
            return mv.PlanResult(False, 'planning error -1')
        if self.build is not None:
            points = self.build(self.arm.meas, q_goal)
        else:
            q0 = self.arm.meas
            points = [sr.TrajPoint(self.seconds * k / 10.0, tuple(q0 + (q_goal - q0) * k / 10.0))
                      for k in range(11)]
        return mv.PlanResult(True, '', tuple(points), tuple(NAMES), np.array(q_goal))

    def execute(self, planned, seconds):
        if self.on_execute is not None:
            self.on_execute()
        self.executed.append(planned.payload)
        if not self.exec_ok:
            return sr.ExecResult(False, 'rejected')
        self.arm.command(planned.payload)
        return sr.ExecResult(True, 'SUCCEEDED')


def _loop(arm=None, io=None, sender=None, goal=M2, dry_run=False, **kwargs):
    arm = FakeArm(T_MEAS) if arm is None else arm
    io = FakeIO(arm) if io is None else io
    if sender is None and not dry_run:
        sender = FakeSender(arm)
    options = dict(goal=goal) if kwargs.get('offset') is None and kwargs.get(
        'joints_deg') is None else {}
    options.update(kwargs)
    return mv.MoveLoop(io, sender, dry_run=dry_run, **options), arm, io, sender


# ------------------------------------------------------------------------------- dry run

def test_a_dry_run_needs_no_desired_no_sender_and_sends_nothing():
    arm = FakeArm(T_MEAS)
    loop, _, io, _ = _loop(arm=arm, io=FakeIO(arm, stale='no_desired'), dry_run=True)
    result = loop.run()
    assert (result.exit_code, result.reason, result.sent) == (0, 'DRY_RUN', 0)
    assert io.prompts == [] and io.rows == []


def test_a_dry_run_at_the_start_pose_is_refused_by_the_measured_floor():
    arm = FakeArm(REST)
    loop, _, io, _ = _loop(arm=arm, dry_run=True)
    result = loop.run()
    assert (result.exit_code, result.reason) == (sr.EXIT_START_REFUSED, 'START_REFUSED')
    assert 'START_FLOOR_MEASURED' in result.detail
    assert io.prompts == []


def test_a_refused_goal_never_reaches_the_sender():
    loop, arm, io, sender = _loop(goal=None, offset=(0.0, 0.16, 0.0))
    result = loop.run()
    assert result.reason == 'START_REFUSED' and 'MOVE_TIP_TOO_FAR' in result.detail
    assert sender.planned == [] and sender.executed == [] and io.prompts == []


# ------------------------------------------------------------------------------ a full run

def test_a_move_is_planned_checked_confirmed_twice_and_executed_once():
    loop, arm, io, sender = _loop()
    result = loop.run()
    assert (result.exit_code, result.reason, result.sent) == (0, 'REACHED', 1)
    assert len(sender.planned) == 1 and len(sender.executed) == 1
    assert len(io.prompts) == 2 and 'Start' in io.prompts[0] and 'Send' in io.prompts[1]
    assert np.linalg.norm(ik.fk(arm.meas)[0] - np.array(M2)) < 1e-6
    assert len(io.rows) == 1 and io.rows[0]['label'] == 'REACHED'
    assert list(io.rows[0]) == list(mv.MOVE_CSV_FIELDS)


def test_an_arm_that_stops_short_is_reported_short_with_code_4():
    # the elbow stops 0.1 rad short of the command (more than on the real arm: about 0.04)
    arm = FakeArm(T_MEAS, short=[0.0, 0.0, 0.1, 0.0, 0.0])
    loop, _, io, _ = _loop(arm=arm)
    result = loop.run()
    assert result.reason == 'SHORT' and result.exit_code == mv.EXIT_SHORT
    assert result.outcome.error_mm > mv.REACHED_MM
    assert io.rows[0]['label'] == 'SHORT' and io.rows[0]['stuck'] != ''


def test_succeeded_is_not_an_arrival_when_the_arm_did_not_move():
    arm = FakeArm(T_MEAS, frozen=True)
    loop, _, io, sender = _loop(arm=arm)
    result = loop.run()
    assert len(sender.executed) == 1  # the goal ended SUCCEEDED
    assert result.reason == 'SHORT' and result.exit_code == mv.EXIT_SHORT


def test_the_arm_that_stopped_short_of_the_desired_can_still_be_moved_again():
    arm = FakeArm(T_MEAS)
    arm.desired = T_MEAS + np.array([0.0, 0.0, -0.034, 0.0, 0.0])  # 0.034 rad of windup
    loop, _, io, sender = _loop(arm=arm)
    result = loop.run()
    assert result.reason == 'REACHED'
    assert 'windup before' in io.said[0]
    assert abs(float(io.rows[0]['first_vs_desired']) - 0.034) < 1e-3


def test_a_windup_over_the_limit_forbids_the_execution():
    arm = FakeArm(T_MEAS)
    arm.desired = T_MEAS + np.array([0.0, 0.0, -0.07, 0.0, 0.0])
    loop, _, io, sender = _loop(arm=arm)
    result = loop.run()
    assert result.reason == 'PLAN_REFUSED' and 'FIRST_DESIRED' in result.detail
    assert sender.executed == [] and len(io.prompts) == 1


# ----------------------------------------------------------------------- confirmations

@pytest.mark.parametrize('answer', ['no', 'timeout'])
def test_no_start_confirmation_no_plan(answer):
    arm = FakeArm(T_MEAS)
    loop, _, io, sender = _loop(arm=arm, io=FakeIO(arm, answers=[answer]))
    result = loop.run()
    assert result.reason == 'NOT_CONFIRMED' and sender.planned == [] and sender.executed == []


@pytest.mark.parametrize('answer', ['no', 'timeout'])
def test_no_send_confirmation_no_execution(answer):
    arm = FakeArm(T_MEAS)
    loop, _, io, sender = _loop(arm=arm, io=FakeIO(arm, answers=['yes', answer]))
    result = loop.run()
    assert result.reason == 'NOT_CONFIRMED' and len(sender.planned) == 1
    assert sender.executed == [] and result.sent == 0


def test_ctrl_c_in_a_confirmation_sends_nothing_and_exits_with_130():
    for position in (0, 1):
        arm = FakeArm(T_MEAS)
        answers = ['yes'] * position + [KeyboardInterrupt()]
        loop, _, io, sender = _loop(arm=arm, io=FakeIO(arm, answers=answers))
        result = loop.run()
        assert result.exit_code == sr.EXIT_INTERRUPTED and sender.executed == []
        assert 'no goal was in flight' in io.said[-1]


def test_ctrl_c_during_the_execution_says_that_the_goal_goes_on():
    arm = FakeArm(T_MEAS)

    def interrupt():
        raise KeyboardInterrupt()
    loop, _, io, sender = _loop(arm=arm, sender=FakeSender(arm, on_execute=interrupt))
    result = loop.run()
    assert result.exit_code == sr.EXIT_INTERRUPTED
    assert 'a goal was in flight' in io.said[-1] and 'not cancelled' in io.said[-1]


# ------------------------------------------------------------------------ the plan checks

def test_a_failed_plan_is_not_executed():
    arm = FakeArm(T_MEAS)
    loop, _, io, sender = _loop(arm=arm, sender=FakeSender(arm, plan_ok=False))
    result = loop.run()
    assert result.reason == 'PLAN_FAILED' and sender.executed == []
    assert result.exit_code == sr.EXIT_STOPPED


def test_a_plan_that_is_too_long_too_fast_or_through_the_floor_is_not_executed():
    low = np.array([0.0, 0.2, 0.0, 0.0, 0.0])

    def through_the_floor(q0, q_goal):
        mid = np.radians([0.0, -22.0, 30.0, 83.0, 0.0]) + low
        path = [q0, mid, q_goal]
        return [sr.TrajPoint(2.0 * i, tuple(q)) for i, q in enumerate(path)]
    for sender_args, reason in (({'seconds': 21.0}, 'DURATION'), ({'seconds': 0.05}, 'SPEED'),
                                ({'build': through_the_floor}, 'PATH_FLOOR')):
        arm = FakeArm(T_MEAS)
        loop, _, io, sender = _loop(arm=arm, sender=FakeSender(arm, **sender_args))
        result = loop.run()
        assert result.reason == 'PLAN_REFUSED' and reason in result.detail, reason
        assert sender.executed == [] and len(io.prompts) == 1
        assert result.exit_code == sr.EXIT_STOPPED


def test_a_plan_that_does_not_end_at_the_goal_is_not_executed():
    def wrong_end(q0, q_goal):
        return [sr.TrajPoint(0.0, tuple(q0)), sr.TrajPoint(5.0, tuple(q_goal + 0.01))]
    arm = FakeArm(T_MEAS)
    loop, _, io, sender = _loop(arm=arm, sender=FakeSender(arm, build=wrong_end))
    result = loop.run()
    assert result.reason == 'PLAN_REFUSED' and 'LAST' in result.detail and sender.executed == []


def test_an_arm_that_moved_while_planning_is_not_executed():
    arm = FakeArm(T_MEAS)

    def push():
        arm.meas = arm.meas + 0.01
    loop, _, io, sender = _loop(arm=arm, sender=FakeSender(arm, on_plan=push))
    result = loop.run()
    assert result.reason == 'ARM_MOVED' and sender.executed == []


# ---------------------------------------------------------- the check right before the send

def test_an_arm_moved_during_the_confirmation_is_not_sent_to():
    arm = FakeArm(T_MEAS)

    def move(count, io):
        if count == 2:
            arm.meas = arm.meas + 0.01
    loop, _, io, sender = _loop(arm=arm, io=FakeIO(arm, on_confirm=move))
    result = loop.run()
    assert result.reason == 'ARM_MOVED' and sender.executed == [] and result.sent == 0


def test_a_changed_desired_during_the_confirmation_is_not_sent():
    arm = FakeArm(T_MEAS)

    def change(count, io):
        if count == 2:
            arm.desired = arm.desired + 0.001
    loop, _, io, sender = _loop(arm=arm, io=FakeIO(arm, on_confirm=change))
    result = loop.run()
    assert result.reason == 'DESIRED_CHANGED' and sender.executed == []


def test_another_active_goal_is_not_sent_over():
    arm = FakeArm(T_MEAS)
    loop, _, io, sender = _loop(arm=arm, io=FakeIO(arm, active=['/arm: 1 active goal']))
    result = loop.run()
    assert result.reason == 'OTHER_GOAL' and sender.executed == []


def test_a_run_that_takes_too_long_is_not_sent():
    arm = FakeArm(T_MEAS)

    def slow(count, io):
        if count == 2:
            io.t += 130.0
    loop, _, io, sender = _loop(arm=arm, io=FakeIO(arm, on_confirm=slow))
    result = loop.run()
    assert result.reason == 'RUN_TIMEOUT' and sender.executed == []


def test_stale_inputs_stop_before_anything_else():
    arm = FakeArm(T_MEAS)
    loop, _, io, sender = _loop(arm=arm, io=FakeIO(arm, stale='meas'))
    result = loop.run()
    assert result.reason == 'STALE_INPUT' and sender.planned == [] and io.prompts == []
    arm = FakeArm(T_MEAS)
    loop, _, io, sender = _loop(arm=arm, io=FakeIO(arm, stale='no_desired'))
    assert loop.run().reason == 'STALE_INPUT'


def test_a_rejected_execution_is_a_stop_and_is_logged():
    arm = FakeArm(T_MEAS)
    loop, _, io, sender = _loop(arm=arm, sender=FakeSender(arm, exec_ok=False))
    result = loop.run()
    assert result.reason == 'SEND_FAILED' and result.sent == 0
    assert len(io.rows) == 1 and 'SEND_FAILED' in io.rows[0]['note']


def test_an_arm_that_does_not_stop_is_not_judged():
    arm = FakeArm(T_MEAS)

    class Drifting(FakeIO):
        def wait(self, seconds):
            super().wait(seconds)
            arm.meas = arm.meas + 0.01  # still moving after every wait
    loop, _, io, sender = _loop(arm=arm, io=Drifting(arm))
    result = loop.run()
    assert result.reason == 'NOT_SETTLED' and result.exit_code == sr.EXIT_STOPPED
    assert len(sender.executed) == 1


def test_the_options_are_checked_when_the_loop_is_made():
    arm = FakeArm(T_MEAS)
    with pytest.raises(ValueError):
        mv.MoveLoop(FakeIO(arm), FakeSender(arm), goal=M2, offset=(0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        mv.MoveLoop(FakeIO(arm), FakeSender(arm), joints_deg=(0, 0, 0, 0, 181))


def test_a_joint_goal_moves_the_arm_to_the_integer_degrees_of_t():
    arm = FakeArm(T_MEAS)
    loop, _, io, sender = _loop(arm=arm, goal=None, joints_deg=(0, -22, 30, 83, 0))
    result = loop.run()
    assert result.reason == 'REACHED'
    assert np.allclose(arm.meas, np.radians([0, -22, 30, 83, 0]), atol=1e-12)
