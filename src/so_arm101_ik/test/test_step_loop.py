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

"""StepLoop (P5): "move, stop, measure" with a fake io, a fake sender and a fake arm."""

import numpy as np
import pytest
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik import step_runner as sr
from so_arm101_ik.diff_mode import DiffModeConfig

T_Q = np.radians([0.0, -22.0, 30.0, 83.0, 0.0])
T_XYZ = ik.fk(T_Q)[0]
START = np.array([-0.0951, -1.4952, 1.6901, 0.4219, -0.0453])


class FakeArm:
    """A fake arm: the desired follows the command, the measurement sticks (friction)."""

    def __init__(self, q, stick=0.0, undershoot=0.0, frozen=False, drift=0.0):
        self.desired = np.array(q, float)
        self.meas = np.array(q, float)
        self.stick = stick
        self.undershoot = undershoot
        self.frozen = frozen
        self.drift = drift
        self.reads = 0

    def command(self, q):
        self.desired = np.array(q, float)
        if self.frozen:
            return
        gap = self.desired - self.meas
        move = np.abs(gap) > self.stick
        self.meas = np.where(move, self.desired - np.sign(gap) * self.undershoot, self.meas)


class FakeIO:
    """Inputs, time, confirmation and status of the actions, all under the control of a test."""

    def __init__(self, arm, answers=None, active=None, on_confirm=None, stale=None):
        self.arm = arm
        self.t = 100.0
        self.answers = list(answers or [])
        self.active = list(active or [])
        self.on_confirm = on_confirm
        self.stale = stale  # None, 'desired' or 'meas' or 'no_desired'
        self.said = []
        self.rows = []
        self.prompts = []
        self.active_calls = 0
        self.read_calls = 0

    def now(self):
        return self.t

    def wait(self, seconds):
        self.t += seconds
        assert self.t < 1100.0, 'the loop does not end (the fake time ran far past the limits)'
        self.arm.meas = self.arm.meas + self.arm.drift

    def read_inputs(self):
        self.read_calls += 1
        desired_at = self.t
        desired = self.arm.desired
        if self.stale == 'desired':
            desired_at = self.t - 5.0
        if self.stale == 'no_desired':
            desired, desired_at = None, None
        meas_at = self.t - 5.0 if self.stale == 'meas' else self.t
        return sr.Inputs(desired, desired_at, self.arm.meas.copy(), meas_at)

    def other_active_goals(self):
        self.active_calls += 1
        if self.active and self.active_calls >= self.active[0]:
            return ['/move_action: 1 active goal']
        return []

    def confirm(self, text, timeout):
        self.prompts.append(text)
        if self.on_confirm is not None:
            self.on_confirm(len(self.prompts), text)
        answer = self.answers.pop(0) if self.answers else 'yes'
        if isinstance(answer, BaseException):
            raise answer
        return answer

    def say(self, text):
        self.said.append(text)

    def log_row(self, row):
        self.rows.append(row)


class FakeSender:
    """A sender that moves the fake arm; it can refuse to prepare or fail to execute."""

    def __init__(self, arm, prepared_ok=True, exec_ok=True, check=None):
        self.arm = arm
        self.prepared_ok = prepared_ok
        self.exec_ok = exec_ok
        self.check = check
        self.prepared = []
        self.executed = []
        self.meas_at_prepare = []

    def prepare(self, q_cmd, q_desired, q_meas):
        self.prepared.append(np.array(q_cmd))
        self.meas_at_prepare.append(np.array(q_meas))
        reasons = () if self.prepared_ok else ('FIRST_DESIRED',)
        return sr.Prepared(self.prepared_ok, reasons, self.check, 0.015, 2.0, np.array(q_cmd))

    def execute(self, prepared):
        self.executed.append(prepared.payload)
        if not self.exec_ok:
            return sr.ExecResult(False, 'rejected')
        self.arm.command(prepared.payload)
        return sr.ExecResult(True, 'SUCCEEDED')


def _loop(q=None, goal_offset=(0.006, 0.0, 0.0), route='direct', arm=None, io=None, sender=None,
          config=None, **kwargs):
    q = T_Q if q is None else q
    arm = arm or FakeArm(q)
    io = io or FakeIO(arm)
    sender = sender or FakeSender(arm)
    config = config or DiffModeConfig(max_dq=0.03)
    loop = sr.StepLoop(io, sender, config, route, offset=goal_offset, **kwargs)
    return loop, io, sender, arm


# ------------------------------------------------------------------------------ reaching goals

def test_a_goal_is_reached_without_friction():
    loop, io, sender, arm = _loop()
    result = loop.run()
    assert result.exit_code == sr.EXIT_OK
    assert result.outcome.kind.value == 'CONVERGED'
    assert result.sent == len(sender.executed) >= 2  # 6 mm at 0.03 rad needs several steps
    assert result.outcome.residual_norm <= 0.003
    assert len(io.rows) == result.sent


def test_a_goal_is_reached_with_a_stuck_arm_by_accumulation():
    arm = FakeArm(T_Q, stick=0.02, undershoot=0.004)
    loop, io, sender, arm = _loop(arm=arm, config=DiffModeConfig(max_dq=0.03, max_steps=12))
    result = loop.run()
    assert result.exit_code == sr.EXIT_OK
    assert result.outcome.kind.value == 'CONVERGED', result.outcome
    assert result.outcome.track_gap is not None


def test_succeeded_is_not_an_arrival_when_the_arm_does_not_move():
    arm = FakeArm(T_Q, frozen=True)  # the goal ends normally, the measurement never changes
    loop, io, sender, arm = _loop(arm=arm)
    result = loop.run()
    assert result.outcome is None or result.outcome.kind.value != 'CONVERGED'
    assert result.reason != 'CONVERGED'
    assert result.exit_code == sr.EXIT_STOPPED  # stopped by the wind-up guard, not "reached"
    assert np.max(np.abs(sender.executed[-1] - arm.meas)) <= 0.06 + 1e-9


def test_every_command_that_is_sent_is_inside_the_range_and_above_the_floor():
    for dx, dy, dz in [(0.008, 0, 0), (-0.008, 0, 0), (0, 0.006, 0), (0, 0, 0.02), (0, 0, -0.02)]:
        arm = FakeArm(T_Q)
        loop, io, sender, arm = _loop(arm=arm, goal_offset=(dx, dy, dz),
                                      config=DiffModeConfig(max_dq=0.03, max_steps=12), auto=3)
        loop.run()
        for q, meas in zip(sender.prepared, sender.meas_at_prepare):
            assert ik.within_limits(q)
            assert ik.fk(q)[0][2] >= 0.048 - 1e-9
            assert float(np.max(np.abs(q - meas))) <= 0.08


# ------------------------------------------------------------------------------ start

def test_a_refused_start_sends_nothing_and_asks_nothing():
    arm = FakeArm(START)
    loop, io, sender, arm = _loop(q=START, arm=arm)
    result = loop.run()
    assert result.exit_code == sr.EXIT_START_REFUSED
    assert result.reason == 'START_REFUSED'
    assert not sender.prepared and not io.prompts


def test_missing_or_old_inputs_do_not_start():
    for stale in ('no_desired', 'desired', 'meas'):
        arm = FakeArm(T_Q)
        loop, io, sender, arm = _loop(arm=arm, io=FakeIO(arm, stale=stale))
        result = loop.run()
        assert result.reason == 'STALE_INPUT', stale
        assert not sender.prepared and not io.prompts, stale


def test_the_start_outside_the_origin_sphere_is_refused():
    origin = T_XYZ + np.array([0.04, 0.0, 0.0])
    loop, io, sender, arm = _loop(origin=origin, origin_radius=0.03)
    assert loop.run().reason == 'ORIGIN'
    assert not sender.prepared


def test_without_an_origin_the_start_position_is_the_origin():
    loop, io, sender, arm = _loop()
    assert loop.origin is None
    assert loop.run().exit_code == sr.EXIT_OK


def test_no_confirmation_at_the_start_sends_nothing():
    for answer in ('no', 'timeout'):
        arm = FakeArm(T_Q)
        loop, io, sender, arm = _loop(arm=arm, io=FakeIO(arm, answers=[answer]))
        result = loop.run()
        assert result.reason == 'NOT_CONFIRMED', answer
        assert not sender.prepared, answer


# ---------------------------------------------------------------------- confirmation and Ctrl-C

def test_no_confirmation_for_a_step_sends_nothing():
    for answer in ('no', 'timeout'):
        arm = FakeArm(T_Q)
        loop, io, sender, arm = _loop(arm=arm, io=FakeIO(arm, answers=['yes', answer]))
        result = loop.run()
        assert result.reason == 'NOT_CONFIRMED', answer
        assert not sender.prepared and result.sent == 0, answer


def test_ctrl_c_while_waiting_for_the_confirmation_sends_and_cancels_nothing():
    arm = FakeArm(T_Q)
    loop, io, sender, arm = _loop(arm=arm, io=FakeIO(arm, answers=['yes', KeyboardInterrupt()]))
    result = loop.run()
    assert result.exit_code == sr.EXIT_INTERRUPTED
    assert not sender.prepared and not sender.executed
    assert not hasattr(io, 'cancel') and not hasattr(sender, 'cancel')


def test_ctrl_c_while_a_goal_is_in_flight_is_reported_and_nothing_is_cancelled():
    arm = FakeArm(T_Q)
    sender = FakeSender(arm)

    def interrupted(prepared):
        raise KeyboardInterrupt()

    sender.execute = interrupted
    loop, io, sender, arm = _loop(arm=arm, sender=sender)
    result = loop.run()
    assert result.exit_code == sr.EXIT_INTERRUPTED
    assert 'in flight' in result.detail and 'not cancelled' in result.detail
    assert any('not cancelled' in text for text in io.said)
    assert not hasattr(io, 'cancel') and not hasattr(sender, 'cancel')


def test_every_step_is_confirmed_by_default_and_auto_skips_the_confirmation_after_the_first():
    loop, io, sender, arm = _loop()
    result = loop.run()
    assert len(io.prompts) == 1 + result.sent  # the start and every step
    loop, io, sender, arm = _loop(auto=5, goal_offset=(0.012, 0.0, 0.0))
    result = loop.run()
    assert result.sent >= 3
    assert len(io.prompts) == 2  # the start and the first step only


def test_auto_goes_back_to_the_confirmation_at_a_limit_and_at_a_hold():
    arm = FakeArm(T_Q)
    loop, io, sender, arm = _loop(arm=arm, goal_offset=(0, 0, -0.02), auto=6,
                                  config=DiffModeConfig(max_dq=0.03, max_steps=12))
    result = loop.run()
    at_limit = [r for r in io.rows if r['status'] == 'AT_LIMIT']
    assert at_limit, 'the floor must be a boundary for this goal'
    send_prompts = [p for p in io.prompts if p.startswith('Send')]
    assert len(send_prompts) == 1 + len(at_limit)  # the first step and every AT_LIMIT step only
    assert result.sent > len(send_prompts)  # the steps in between were sent without Enter
    assert sum(p.startswith('HOLD') for p in io.prompts) >= 1  # a HOLD asks for the Enter too


# --------------------------------------------------------------------- the checks before a send

def test_the_checks_are_done_before_every_send_not_only_at_the_start():
    loop, io, sender, arm = _loop()
    result = loop.run()
    assert result.sent >= 2
    assert io.active_calls == len(sender.prepared) == result.sent
    # the inputs are read again after the Enter of every step (more reads than steps)
    assert io.read_calls > 3 * result.sent


def test_the_desired_tolerance_depends_on_the_route():
    # direct: 1e-4 rad; moveit: twice the moveit tolerance (0.001 by default, so 0.002 rad)
    for route, gap, stops in (('direct', 5e-5, False), ('direct', 2e-4, True),
                              ('moveit', 0.0015, False), ('moveit', 0.003, True)):
        arm = FakeArm(T_Q)

        def disturb(count, text, arm=arm, gap=gap):
            if count == 3:  # the Enter of the second step
                arm.desired = arm.desired + np.array([0, 0, 0, gap, 0])

        loop, io, sender, arm = _loop(route=route, arm=arm, io=FakeIO(arm, on_confirm=disturb))
        result = loop.run()
        case = (route, gap)
        if stops:
            assert result.reason == 'DESIRED_MISMATCH', case
            assert result.sent == 1, case
        else:
            assert result.reason != 'DESIRED_MISMATCH', case


def test_an_active_goal_of_another_action_stops_before_the_second_send():
    arm = FakeArm(T_Q)
    loop, io, sender, arm = _loop(arm=arm, io=FakeIO(arm, active=[2]))
    result = loop.run()
    assert result.reason == 'OTHER_GOAL'
    assert result.sent == 1 and len(sender.prepared) == 1


def test_the_arm_moved_while_waiting_for_the_enter_is_not_sent():
    arm = FakeArm(T_Q)

    def move(count, text):
        if count == 2:
            arm.meas = arm.meas + np.array([0, 0.01, 0, 0, 0])

    loop, io, sender, arm = _loop(arm=arm, io=FakeIO(arm, on_confirm=move))
    result = loop.run()
    assert result.reason == 'ARM_MOVED'
    assert not sender.prepared


def test_inputs_that_become_old_while_waiting_are_not_sent():
    arm = FakeArm(T_Q)
    io = FakeIO(arm)

    def go_stale(count, text):
        if count == 2:
            io.stale = 'desired'

    io.on_confirm = go_stale
    loop, io, sender, arm = _loop(arm=arm, io=io)
    assert loop.run().reason == 'STALE_INPUT'
    assert not sender.prepared


def test_verify_command_is_run_again_with_the_new_measurement(monkeypatch):
    calls = []
    real = sr.verify_command

    def spy(q_new, q_prev, q_meas, branch, config, target=None):
        calls.append(np.array(q_meas))
        return ('FINAL_CHECK', 'spy')

    monkeypatch.setattr(sr, 'verify_command', spy)
    loop, io, sender, arm = _loop()
    result = loop.run()
    assert real is not spy and len(calls) == 1
    assert result.reason == 'VERIFY'
    assert not sender.prepared
    assert np.allclose(calls[0], arm.meas)


# ------------------------------------------------------------------------- not executed / failed

def test_a_plan_that_failed_its_check_is_not_executed_and_ends_the_run():
    check = sr.PlanCheck(False, ('FIRST_DESIRED',), 0.2, 0.2, 1.0, 5, 0.026, 0.0, 0.0)
    arm = FakeArm(T_Q)
    sender = FakeSender(arm, False, True, check)
    loop, io, sender, arm = _loop(route='moveit', arm=arm, sender=sender)
    result = loop.run()
    assert result.reason == 'NOT_EXECUTED'
    assert not sender.executed and len(sender.prepared) == 1
    assert io.rows[-1]['plan_ok'] is False
    assert io.rows[-1]['first_vs_desired'] == '0.026'  # recorded although it was not executed


def test_the_peak_speed_of_the_plan_is_recorded_for_every_step():
    check = sr.PlanCheck(True, (), 0.2, 0.25, 1.0, 5, 0.001, 0.0, 0.0005)
    arm = FakeArm(T_Q)
    sender = FakeSender(arm, True, True, check)
    loop, io, sender, arm = _loop(route='moveit', arm=arm, sender=sender)
    result = loop.run()
    assert result.sent >= 1
    for row in io.rows:
        assert row['plan_peak_pos_speed'] == '0.2' and row['plan_peak_vel_speed'] == '0.25'
        assert row['plan_duration'] == '1' and row['cmd_speed'] == '0.015'


def test_a_failed_send_ends_the_run_and_the_mode_is_not_stepped_again():
    arm = FakeArm(T_Q)
    loop, io, sender, arm = _loop(arm=arm, sender=FakeSender(arm, True, False))
    result = loop.run()
    assert result.reason == 'SEND_FAILED'
    assert len(sender.prepared) == 1 and len(sender.executed) == 1
    assert loop.mode.state.steps == 1


def test_an_arm_that_does_not_stop_ends_the_run():
    arm = FakeArm(T_Q, drift=0.01)
    loop, io, sender, arm = _loop(arm=arm)
    assert loop.run().reason == 'NOT_SETTLED'


def test_a_run_that_takes_too_long_ends():
    loop, io, sender, arm = _loop(run_timeout=2.0)
    assert loop.run().reason == 'RUN_TIMEOUT'


# ------------------------------------------------------------------------------ dry run

def test_a_dry_run_sends_nothing_asks_nothing_and_needs_no_sender():
    loop, io, sender, arm = _loop(dry_run=True)
    result = loop.run()
    assert result.reason == 'DRY_RUN' and result.exit_code == sr.EXIT_OK
    assert not sender.prepared and not io.prompts
    assert any('DRY-RUN step' in text for text in io.said)


def test_a_dry_run_with_assumed_desired_uses_the_measurement_and_still_refuses_the_start_pose():
    arm = FakeArm(START)
    io = FakeIO(arm, stale='no_desired')
    loop, io, sender, arm = _loop(q=START, arm=arm, io=io, dry_run=True,
                                  assume_desired_is_measured=True)
    assert loop.run().exit_code == sr.EXIT_START_REFUSED


def test_the_assumption_is_not_accepted_for_a_real_run():
    arm = FakeArm(T_Q)
    with pytest.raises(ValueError):
        sr.StepLoop(FakeIO(arm), FakeSender(arm), DiffModeConfig(), 'direct',
                    offset=(0.001, 0, 0), assume_desired_is_measured=True)


def test_two_lines_typed_together_do_not_send_two_steps_with_one_enter():
    # '\n' for the start and '\n' for the first step arrive in one chunk, then nothing
    arm = FakeArm(T_Q)

    class Terminal(FakeIO):
        def __init__(self, arm, text):
            super().__init__(arm)
            self.reader = sr.LineReader()
            self.chunks = [text]

        def confirm(self, text, timeout):
            self.prompts.append(text)

            def poll():
                self.wait(0.05)
                return self.chunks.pop(0) if self.chunks else None

            return sr.confirm_answer(self.reader, poll, self.now, timeout)

    for text, steps in (('\n\n', 1), ('\n\n\n', 2), ('\n\n\nn\n', 2)):
        arm = FakeArm(T_Q)
        loop, io, sender, arm = _loop(arm=arm, io=Terminal(arm, text), goal_offset=(0.02, 0, 0))
        result = loop.run()
        assert result.sent == steps, text  # one line for the start, one for every step sent
        assert result.reason == 'NOT_CONFIRMED', text  # the next confirmation got no line


def test_the_windup_after_every_step_is_recorded():
    arm = FakeArm(T_Q, stick=0.02, undershoot=0.01)
    loop, io, sender, arm = _loop(arm=arm, config=DiffModeConfig(max_dq=0.03, max_steps=12))
    loop.run()
    assert io.rows
    for row in io.rows:
        assert float(row['windup_after']) >= 0.0 and row['windup_after'] != ''
    assert max(float(r['windup_after']) for r in io.rows) > 0.0  # the stuck arm leaves a gap
