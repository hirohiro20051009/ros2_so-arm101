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

"""The state machine of the VR follow node (follow_machine.py): no ROS, no clock."""

import math

import numpy as np
import pytest
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik import follow_calc as fc
from so_arm101_ik import follow_machine as fm

Q0 = np.array([0.0, 0.0, 0.8, -0.6, 0.0])  # the tip: z about 0.10 m, middle of the reach
P_IN0 = np.array([0.4, 0.1, 0.5])
PERIOD = 0.05


class Rig:
    """A fake world: the arm follows every point at once (q_meas = the last point)."""

    def __init__(self, follow_meas=True, **config):
        # the test moves are larger than a real hand: the check of the solution (C1) is off unless
        # a test asks for it
        config.setdefault('ik_jump', 0.0)
        self.f = fm.Follower(fc.FollowConfig(**config))
        self.t = 100.0
        self.p = P_IN0.copy()
        self.r = np.eye(3)
        self.clutch = self.open = self.close = False
        self.q_meas = Q0.copy()
        self.q_desired = Q0.copy()
        self.follow_meas = follow_meas
        self.busy = ()
        self.pose_age = self.joy_age = self.meas_age = self.desired_age = 0.0
        self.no_pose = False
        self.out = None

    def tick(self, dt=PERIOD, **changes):
        for name, value in changes.items():
            setattr(self, name, value)
        self.t += dt
        inp = fm.Inputs(
            pose=None if self.no_pose else (self.p.copy(), self.r.copy()),
            pose_time=self.t - self.pose_age,
            joy=fm.JoyState(self.clutch, self.open, self.close), joy_time=self.t - self.joy_age,
            q_meas=self.q_meas.copy(), q_meas_time=self.t - self.meas_age,
            q_desired=self.q_desired.copy(), q_desired_time=self.t - self.desired_age,
            busy=self.busy)
        self.out = self.f.update(self.t, inp)
        if self.out.point is not None and self.follow_meas:
            self.q_meas = self.out.point.copy()
            self.q_desired = self.out.point.copy()  # open loop: the reference is the command
        return self.out

    def start(self):
        self.tick(clutch=False)
        out = self.tick(clutch=True)
        assert out.state == fm.TRACKING, out.reason
        return out

    def run(self, count, **changes):
        out = self.tick(**changes)
        for _ in range(count - 1):
            out = self.tick()
        return out

    @property
    def tip(self):
        return ik.fk(self.q_meas)[0]


def _p_ref():
    return ik.fk(Q0)[0]


# ------------------------------------------------------------------------- idle and start

def test_idle_sends_nothing_without_the_clutch():
    rig = Rig()
    for _ in range(5):
        out = rig.tick()
        assert out.state == fm.IDLE and out.point is None and out.gripper is None


def test_the_clutch_starts_tracking_with_the_first_point_on_the_next_tick():
    rig = Rig()
    rig.tick()
    out = rig.tick(clutch=True)
    assert out.state == fm.TRACKING and out.point is None
    out = rig.tick()
    assert out.state == fm.TRACKING and out.reason == 'OK'
    assert np.allclose(out.point, Q0, atol=1e-6)  # the hand did not move: hold the reference


def test_a_clutch_that_is_already_pressed_at_the_first_joy_does_not_start():
    rig = Rig()
    for _ in range(3):
        assert rig.tick(clutch=True).state == fm.IDLE
    rig.tick(clutch=False)
    assert rig.tick(clutch=True).state == fm.TRACKING


def test_releasing_the_clutch_stops_at_once_and_sends_nothing():
    rig = Rig()
    rig.start()
    rig.tick()
    out = rig.tick(clutch=False)
    assert out.state == fm.IDLE and out.reason == 'RELEASED' and out.point is None
    for _ in range(3):
        assert rig.tick().point is None


def test_every_clutch_takes_the_bases_again():
    rig = Rig()
    rig.start()
    rig.run(3, p=P_IN0 + [0.01, 0.0, 0.0])
    rig.tick(clutch=False)
    rig.tick(p=P_IN0 + [0.2, 0.2, 0.2])  # the hand went somewhere else
    out = rig.tick(clutch=True)
    assert out.state == fm.TRACKING
    out = rig.tick()
    assert out.state == fm.TRACKING
    assert np.allclose(ik.fk(out.point)[0], ik.fk(rig.q_desired)[0], atol=1e-6)


def test_the_start_is_refused_with_every_reason():
    rig = Rig(floor_z=0.25)  # the tip (about 0.19 m) is under this floor
    rig.tick()
    rig.q_desired = Q0 + [0.0, 0.1, 0.0, 0.0, 0.0]
    out = rig.tick(clutch=True)
    assert out.state == fm.IDLE and out.reason.startswith('REFUSED')
    assert 'WINDUP' in out.reason and 'FLOOR' in out.reason


Q_ROLL_OUT = Q0 + [0.0, 0.0, 0.0, 0.0, 3.0]  # the roll is beyond its limit

START_REFUSALS = [
    ({'no_pose': True}, 'POSE'),
    ({'pose_age': 0.6}, 'POSE'),
    ({'meas_age': 0.6}, 'JOINT_STATE'),
    ({'desired_age': 0.6}, 'CONTROLLER_STATE'),
    ({'busy': ('arm goal',)}, 'BUSY'),
    ({'q_desired': Q0 + [0.0, 0.07, 0.0, 0.0, 0.0]}, 'WINDUP'),
    ({'q_desired': Q_ROLL_OUT, 'q_meas': Q_ROLL_OUT}, 'RANGE'),
]


@pytest.mark.parametrize('change, reason', START_REFUSALS)
def test_the_start_is_refused(change, reason):
    rig = Rig()
    rig.tick()
    for name, value in change.items():
        setattr(rig, name, value)
    out = rig.tick(clutch=True)
    assert out.state == fm.IDLE and reason in out.reason and out.point is None


def test_the_windup_limit_is_a_parameter():
    rig = Rig(clutch_windup=0.08)
    rig.tick()
    rig.q_desired = Q0 + [0.0, 0.07, 0.0, 0.0, 0.0]
    assert rig.tick(clutch=True).state == fm.TRACKING


def test_a_refused_start_needs_a_new_press():
    rig = Rig()
    rig.tick()
    rig.tick(clutch=True, busy=('x',))
    assert rig.tick(busy=()).state == fm.IDLE
    rig.tick(clutch=False)
    assert rig.tick(clutch=True).state == fm.TRACKING


def test_the_elbow_above_the_limit_at_the_start_is_refused():
    q = np.array([0.0, -1.5, 1.62, 0.2, 0.0])  # 92.8 deg
    rig = Rig()
    rig.tick()
    rig.q_meas, rig.q_desired = q.copy(), q.copy()
    assert 'RANGE' in rig.tick(clutch=True).reason


# ------------------------------------------------------------------------------ tracking

def test_the_tip_follows_the_hand_with_the_scale():
    for scale in (1.0, 0.5):
        rig = Rig(scale=scale)
        rig.start()
        out = rig.run(40, p=P_IN0 + [0.02, -0.01, 0.015])
        assert out.state == fm.TRACKING
        assert np.allclose(rig.tip, _p_ref() + scale * np.array([0.02, -0.01, 0.015]), atol=1e-4)
        assert np.allclose(out.target_p, _p_ref() + scale * np.array([0.02, -0.01, 0.015]))


def test_the_target_pose_is_after_the_scale_and_the_projection():
    rig = Rig()
    rig.start()
    out = rig.tick(p=P_IN0 + [0.01, 0.0, 0.0])
    assert out.target_p is not None and out.target_r.shape == (3, 3)
    assert np.allclose(out.target_r @ out.target_r.T, np.eye(3))
    assert out.detail['proj_deg'] < 1.0  # a move without a rotation: almost nothing is lost


def test_a_rotation_of_the_hand_turns_the_tip_and_the_projection_is_reported():
    rig = Rig()
    rig.start()
    out = rig.run(40, r=fc.rot_x(math.radians(15.0)))
    assert out.state == fm.TRACKING
    assert out.detail['proj_deg'] >= 0.0
    turned = fc.rotation_angle(fc.tip_rotation(Q0).T @ fc.tip_rotation(rig.q_meas))
    assert 0.05 < turned < math.radians(30.0)


def test_the_speed_of_every_joint_is_limited_and_reported():
    rig = Rig()
    rig.start()
    rig.tick()
    previous = rig.q_meas.copy()
    out = rig.tick(p=P_IN0 + [0.04, 0.0, 0.0])
    assert out.reason == 'SPEED_LIMITED'
    assert np.max(np.abs(out.point - previous)) <= 0.5 * PERIOD + 1e-9
    assert np.max(np.abs(out.point - previous)) == pytest.approx(0.5 * PERIOD)


def test_the_step_uses_the_real_elapsed_time_up_to_two_periods():
    rig = Rig()
    rig.start()
    rig.tick()
    previous = rig.q_meas.copy()
    out = rig.tick(dt=1.0, p=P_IN0 + [0.04, 0.0, 0.0])  # a long gap: at most two periods
    assert np.max(np.abs(out.point - previous)) <= 0.5 * 2 * PERIOD + 1e-9
    previous = rig.q_meas.copy()
    out = rig.tick(dt=0.07)
    assert np.max(np.abs(out.point - previous)) == pytest.approx(0.5 * 0.07)


def test_a_slow_hand_passes_the_default_check_of_the_solution():
    rig = Rig(ik_jump=0.35)  # the default value
    rig.start()
    for i in range(30):
        out = rig.tick(p=P_IN0 + [0.0, 0.005 * (i + 1), 0.0])  # 100 mm/s
        assert out.state == fm.TRACKING, (i, out.reason)


def test_a_point_is_sent_on_every_tracking_tick():
    rig = Rig()
    rig.start()
    for _ in range(5):
        assert rig.tick().point is not None


def test_the_target_does_not_drift_when_the_hand_is_still():
    rig = Rig()
    rig.start()
    first = rig.tick().point
    for _ in range(30):
        out = rig.tick()
    assert np.allclose(out.point, first, atol=1e-9)


# -------------------------------------------------------------------------------- jump

def test_a_jump_of_the_target_stops_and_sends_nothing():
    rig = Rig()
    rig.start()
    rig.run(3, p=P_IN0 + [0.01, 0.0, 0.0])
    out = rig.tick(p=P_IN0 + [0.07, 0.0, 0.0])
    assert out.state == fm.HOLD_JUMP and out.point is None and out.reason == 'TARGET_JUMP'
    assert rig.tick().point is None


def test_the_jump_is_judged_after_the_scale():
    rig = Rig(scale=0.5)
    rig.start()
    assert rig.tick(p=P_IN0 + [0.08, 0.0, 0.0]).state == fm.TRACKING  # 40 mm of the target
    assert rig.tick(p=P_IN0 + [0.19, 0.0, 0.0]).state == fm.HOLD_JUMP  # 55 mm more
    rig2 = Rig(scale=2.0)
    rig2.start()
    assert rig2.tick(p=P_IN0 + [0.026, 0.0, 0.0]).state == fm.HOLD_JUMP  # 52 mm of the target


def test_an_angular_jump_stops():
    rig = Rig()
    rig.start()
    assert rig.tick(r=fc.rot_x(math.radians(25.0))).state == fm.TRACKING
    out = rig.tick(r=fc.rot_x(math.radians(60.0)))  # 35 deg from the last accepted target
    assert out.state == fm.HOLD_JUMP


def test_the_resume_needs_the_hand_back_within_20_mm_and_10_deg():
    rig = Rig()
    rig.start()
    rig.run(3, p=P_IN0 + [0.01, 0.0, 0.0])
    rig.tick(p=P_IN0 + [0.07, 0.0, 0.0])
    assert rig.tick(p=P_IN0 + [0.04, 0.0, 0.0]).state == fm.HOLD_JUMP  # 30 mm: not yet
    assert rig.tick(p=P_IN0 + [0.01, 0.0, 0.03]).state == fm.HOLD_JUMP
    rig.tick(p=P_IN0 + [0.01, 0.0, 0.0], r=fc.rot_x(math.radians(12.0)))
    assert rig.f.state == fm.HOLD_JUMP  # the angle is 12 deg
    out = rig.tick(p=P_IN0 + [0.025, 0.0, 0.0], r=np.eye(3))  # 15 mm and 0 deg
    assert out.state == fm.TRACKING and out.point is not None


def test_the_resume_keeps_the_bases():
    rig = Rig()
    rig.start()
    rig.run(40, p=P_IN0 + [0.02, 0.0, 0.0])
    rig.tick(p=P_IN0 + [0.09, 0.0, 0.0])
    rig.run(40, p=P_IN0 + [0.025, 0.0, 0.0])
    assert rig.f.state == fm.TRACKING
    assert np.allclose(rig.tip, _p_ref() + [0.025, 0.0, 0.0], atol=1e-4)


# ------------------------------------------------------------------------------- stale

def test_an_old_pose_stops_and_the_hand_back_resumes():
    rig = Rig()
    rig.start()
    rig.run(3, p=P_IN0 + [0.01, 0.0, 0.0])
    out = rig.tick(pose_age=0.6)
    assert out.state == fm.STALE and out.reason == 'POSE' and out.point is None
    out = rig.tick(pose_age=0.0, p=P_IN0 + [0.012, 0.0, 0.0])
    assert out.state == fm.TRACKING and out.point is not None


def test_after_stale_a_pose_far_away_does_not_resume():
    rig = Rig()
    rig.start()
    rig.tick(pose_age=0.6)
    out = rig.tick(pose_age=0.0, p=P_IN0 + [0.2, 0.0, 0.0])
    assert out.state == fm.STALE and out.reason == 'WAIT_RETURN' and out.point is None
    rig.tick(clutch=False)
    assert rig.f.state == fm.IDLE


def test_an_old_pose_is_judged_by_the_age_not_by_the_gap_of_the_ticks():
    rig = Rig()
    rig.start()
    assert rig.tick(pose_age=0.49).state == fm.TRACKING
    assert rig.tick(pose_age=0.51).state == fm.STALE


def test_a_missing_pose_stops():
    rig = Rig()
    rig.start()
    assert rig.tick(no_pose=True).state == fm.STALE


def test_old_joint_states_stop():
    rig = Rig()
    rig.start()
    out = rig.tick(meas_age=0.6)
    assert out.state == fm.STALE and out.reason == 'JOINT_STATE' and out.point is None
    assert rig.tick(meas_age=0.0).state == fm.TRACKING


# ---------------------------------------------------------------------------- joy timeout

def test_a_joy_that_stops_for_a_second_ends_the_tracking():
    rig = Rig()
    rig.start()
    assert rig.tick(joy_age=0.9).state == fm.TRACKING
    out = rig.tick(joy_age=1.1)
    assert out.state == fm.IDLE and out.reason == 'JOY_TIMEOUT' and out.point is None


def test_the_joy_timeout_can_be_disabled():
    rig = Rig(joy_timeout_s=0.0)
    rig.start()
    assert rig.tick(joy_age=50.0).state == fm.TRACKING


def test_after_a_joy_timeout_a_held_button_is_not_an_edge():
    rig = Rig()
    rig.start()
    rig.tick(joy_age=2.0)
    assert rig.tick(joy_age=0.0).state == fm.IDLE  # still pressed: no new start
    rig.tick(clutch=False)
    assert rig.tick(clutch=True).state == fm.TRACKING


# ---------------------------------------------------------------------------- range stops

def test_the_floor_stops_the_arm_in_place():
    z = _p_ref()[2]
    rig = Rig(floor_z=max(0.045, z - 0.02), max_lag=0.0)
    rig.start()
    rig.tick(p=P_IN0 + [0.0, 0.0, -0.01])
    out = rig.tick(p=P_IN0 + [0.0, 0.0, -0.03])
    assert out.state == fm.HOLD_RANGE and out.reason == 'FLOOR' and out.point is None
    assert out.detail['message']


def test_a_far_target_is_refused_and_the_command_does_not_move():
    rig = Rig(max_lag=0.0, max_tracking_error=0.0, ik_jump=0.0)
    rig.start()
    out = None
    for i in range(60):
        out = rig.tick(p=P_IN0 + [0.04 * (i + 1), 0.0, 0.0])
        if out.state != fm.TRACKING:
            break
    assert out.state == fm.HOLD_RANGE
    assert out.reason in ('UNREACHABLE', 'JOINT_RANGE', 'ELBOW_LIMIT', 'ORIENT')
    assert out.point is None and out.target_p is None
    again = rig.tick(p=P_IN0 + [0.04 * 61, 0.0, 0.0])
    assert again.point is None


def test_a_range_stop_resumes_when_the_hand_comes_back():
    rig = Rig(max_lag=0.0, max_tracking_error=0.0, ik_jump=0.0)
    rig.start()
    last = P_IN0.copy()
    for i in range(60):
        candidate = P_IN0 + [0.04 * (i + 1), 0.0, 0.0]
        out = rig.tick(p=candidate)
        if out.state != fm.TRACKING:
            break
        last = candidate
    assert out.state == fm.HOLD_RANGE
    assert rig.tick(p=last + [-0.01, 0.0, 0.0]).state == fm.TRACKING


def test_a_jump_of_the_solution_is_refused_and_can_be_disabled():
    rig = Rig(ik_jump=0.001)
    rig.start()
    out = rig.tick(p=P_IN0 + [0.02, 0.0, 0.0])
    assert out.state == fm.HOLD_RANGE and out.reason == 'IK_JUMP' and out.point is None
    rig = Rig(ik_jump=0.0)
    rig.start()
    assert rig.tick(p=P_IN0 + [0.02, 0.0, 0.0]).state == fm.TRACKING


# ------------------------------------------------------------------------------- C2 lag

def test_a_target_far_ahead_of_the_commanded_tip_stops_and_resumes_when_close():
    rig = Rig(max_lag=0.02, max_tracking_error=0.0)
    rig.start()
    out = rig.tick(p=P_IN0 + [0.045, 0.0, 0.0])  # 45 mm ahead of the tip: not a jump, a lag
    assert out.state == fm.HOLD_LAG and out.point is None
    assert out.detail['lag_mm'] > 20.0
    out = rig.tick(p=P_IN0 + [0.008, 0.0, 0.0])  # the hand came back: lag 8 mm, under the half
    assert out.state == fm.TRACKING and out.point is not None


def test_the_lag_stop_needs_the_lag_under_half_of_the_limit():
    rig = Rig(max_lag=0.02, max_tracking_error=0.0)
    rig.start()
    rig.tick(p=P_IN0 + [0.045, 0.0, 0.0])
    assert rig.tick(p=P_IN0 + [0.03, 0.0, 0.0]).state == fm.HOLD_LAG  # 30 mm
    assert rig.tick(p=P_IN0 + [0.012, 0.0, 0.0]).state == fm.HOLD_LAG  # 12 mm: over the half
    assert rig.tick(p=P_IN0 + [0.009, 0.0, 0.0]).state == fm.TRACKING


def test_the_lag_stop_judges_a_jump_from_frame_to_frame():
    rig = Rig(max_lag=0.02, max_tracking_error=0.0)
    rig.start()
    rig.tick(p=P_IN0 + [0.045, 0.0, 0.0])
    assert rig.tick(p=P_IN0 + [0.10, 0.0, 0.0]).state == fm.HOLD_JUMP  # 55 mm from the last


def test_the_lag_check_can_be_disabled():
    rig = Rig(max_lag=0.0, max_tracking_error=0.0)
    rig.start()
    assert rig.tick(p=P_IN0 + [0.045, 0.0, 0.0]).state == fm.TRACKING


# -------------------------------------------------------------------------- C3 blocked

def test_a_command_far_from_the_measurement_is_blocked_until_a_new_clutch():
    rig = Rig(follow_meas=False, max_tracking_error=0.05, max_lag=0.0)  # the arm does not move
    rig.start()
    out = None
    for i in range(40):
        out = rig.tick(p=P_IN0 + [0.0, 0.003 * (i + 1), 0.0])
        if out.state == fm.BLOCKED:
            break
    assert out.state == fm.BLOCKED and out.point is None and 'TRACKING_ERROR' in out.reason
    assert rig.tick(p=P_IN0).state == fm.BLOCKED  # the hand back: still blocked
    rig.tick(clutch=False)
    assert rig.f.state == fm.IDLE


def test_the_command_runs_away_from_a_still_arm_until_the_limit_c0():
    rig = Rig(follow_meas=False, max_lag=0.0)  # the real arm in read-only mode (c0)
    rig.start()
    advanced = 0.0
    for i in range(100):
        out = rig.tick(p=P_IN0 + [0.0, 0.003 * (i + 1), 0.0])
        if out.state == fm.BLOCKED:
            break
        advanced = float(np.max(np.abs(rig.f._q_cmd - Q0)))
    assert out.state == fm.BLOCKED and 'TRACKING_ERROR' in out.reason
    assert 0.19 < advanced <= 0.23  # TRACKING -> BLOCKED where the command is 0.2 rad ahead


def test_the_blocked_check_can_be_disabled():
    rig = Rig(follow_meas=False, max_tracking_error=0.0, max_lag=0.0)
    rig.start()
    for i in range(60):
        out = rig.tick(p=P_IN0 + [0.0, 0.003 * (i + 1), 0.0])
    assert out.state == fm.TRACKING
    assert np.max(np.abs(rig.f._q_cmd - Q0)) > 0.3  # it went far from the (still) measurement


# ------------------------------------------------------------------------------- busy

def test_another_active_goal_stops_the_arm_until_a_new_clutch():
    rig = Rig()
    rig.start()
    out = rig.tick(busy=('/arm_controller/follow_joint_trajectory: 1 active goal(s)',))
    assert out.state == fm.BUSY and out.point is None and 'active goal' in out.reason
    assert rig.tick(busy=()).state == fm.BUSY
    rig.tick(clutch=False)
    assert rig.f.state == fm.IDLE
    rig.tick(clutch=False)
    assert rig.tick(clutch=True).state == fm.TRACKING


# ------------------------------------------------------------------------------ toggle

def test_the_toggle_mode_starts_and_stops_on_the_edges():
    rig = Rig(clutch_mode='toggle')
    rig.tick()
    assert rig.tick(clutch=True).state == fm.TRACKING
    assert rig.tick(clutch=True).state == fm.TRACKING  # still pressed
    assert rig.tick(clutch=False).state == fm.TRACKING  # released: it keeps tracking
    assert rig.tick().point is not None
    out = rig.tick(clutch=True)
    assert out.state == fm.IDLE and out.reason == 'RELEASED'
    assert rig.tick(clutch=False).point is None


def test_the_toggle_mode_also_stops_when_the_joy_stops():
    rig = Rig(clutch_mode='toggle')
    rig.tick()
    rig.tick(clutch=True)
    rig.tick(clutch=False)
    assert rig.tick(joy_age=2.0).reason == 'JOY_TIMEOUT'


# ------------------------------------------------------------------------------- scale

def test_changing_the_scale_while_tracking_does_not_move_the_target():
    rig = Rig()
    rig.start()
    rig.run(40, p=P_IN0 + [0.03, 0.0, 0.0])
    before = rig.q_meas.copy()
    rig.f.set_scale(0.5)
    out = rig.tick()
    assert out.state == fm.TRACKING and np.allclose(out.point, before, atol=1e-6)
    rig.run(40, p=P_IN0 + [0.05, 0.0, 0.0])  # 20 mm of the hand: 10 mm of the target
    assert np.allclose(rig.tip, _p_ref() + [0.04, 0.0, 0.0], atol=1e-4)


def test_an_invalid_scale_is_refused_and_the_old_one_kept():
    f = fm.Follower()
    for scale in (0.0, -1.0, 4.0, math.nan):
        with pytest.raises(ValueError):
            f.set_scale(scale)
    assert f.scale == 1.0
    f.set_scale(2.0)
    assert f.scale == 2.0


# ---------------------------------------------------------------------------- gripper

def test_the_buttons_give_one_command_per_edge():
    rig = Rig()
    rig.tick()
    assert rig.tick(open=True).gripper == 0.0
    assert rig.tick().gripper is None  # held: nothing more
    rig.tick(open=False)
    assert rig.tick(close=True).gripper == 1.0
    assert rig.tick().gripper is None
    assert rig.tick(close=False).gripper is None


def test_the_gripper_works_without_the_clutch_by_default():
    rig = Rig()
    rig.tick()
    out = rig.tick(close=True)
    assert out.state == fm.IDLE and out.gripper == 1.0


def test_the_gripper_can_be_tied_to_the_clutch():
    rig = Rig(gripper_requires_clutch=True)
    rig.tick()
    assert rig.tick(close=True).gripper is None
    rig.tick(close=False)
    rig.tick(clutch=True)
    assert rig.tick(close=True).gripper == 1.0


def test_both_buttons_at_once_give_nothing():
    rig = Rig()
    rig.tick()
    assert rig.tick(open=True, close=True).gripper is None
    assert rig.tick().gripper is None


def test_a_button_held_at_the_first_joy_or_after_a_gap_is_not_an_edge():
    rig = Rig()
    assert rig.tick(open=True).gripper is None  # held at the first joy
    rig.tick(open=False)
    rig.tick(open=True, joy_age=2.0)  # joy is stale: nothing
    assert rig.tick(open=True, joy_age=0.0).gripper is None  # back and held: no edge
    rig.tick(open=False)
    assert rig.tick(open=True).gripper == 0.0


@pytest.mark.parametrize('change, state', [
    ({'p': P_IN0 + [0.07, 0.0, 0.0]}, fm.HOLD_JUMP),
    ({'pose_age': 0.6}, fm.STALE),
    ({'busy': ('x',)}, fm.BUSY),
    ({'joy_age': 2.0}, fm.IDLE),
])
def test_the_gripper_is_not_moved_by_the_stop_conditions(change, state):
    rig = Rig()
    rig.start()
    out = rig.tick(**change)
    assert out.state == state and out.gripper is None
    assert rig.tick().gripper is None
    assert rig.tick(joy_age=0.0).gripper is None


def test_a_gripper_edge_comes_with_the_arm_output_of_the_same_tick():
    rig = Rig()
    rig.start()
    rig.tick()
    out = rig.tick(close=True, p=P_IN0 + [0.01, 0.0, 0.0])
    assert out.gripper == 1.0 and out.point is not None and out.state == fm.TRACKING


# --------------------------------------------------------------------------- the inputs

def test_the_inputs_are_not_modified():
    f = fm.Follower()
    inp = fm.Inputs(pose=(P_IN0.copy(), np.eye(3)), pose_time=1.0, joy=fm.JoyState(False),
                    joy_time=1.0, q_meas=Q0.copy(), q_meas_time=1.0, q_desired=Q0.copy(),
                    q_desired_time=1.0)
    f.update(1.0, inp)
    inp.joy = fm.JoyState(True)
    f.update(1.05, inp)
    inp.pose = (P_IN0 + [0.01, 0.0, 0.0], np.eye(3))
    out = f.update(1.1, inp)
    assert out.state == fm.TRACKING
    assert np.all(inp.q_meas == Q0) and np.all(inp.q_desired == Q0)
    assert np.all(inp.pose[1] == np.eye(3))
    out.point[:] = 0.0  # the output is the caller's: it must not be the state of the machine
    out.target_p[:] = 0.0
    again = f.update(1.15, inp)
    assert again.point is not None and np.max(np.abs(again.point)) > 0.1
    assert np.allclose(again.target_p, P_IN0 * 0 + _p_ref() + [0.01, 0.0, 0.0])


def test_the_input_rotation_turns_the_move_of_the_hand():
    rig = Rig(input_rotation_rpy_deg=(0.0, 0.0, 90.0))
    rig.start()
    rig.run(40, p=P_IN0 + [0.02, 0.0, 0.0])  # the x of the VR is the y of base_link
    assert np.allclose(rig.tip, _p_ref() + [0.0, 0.02, 0.0], atol=1e-4)
