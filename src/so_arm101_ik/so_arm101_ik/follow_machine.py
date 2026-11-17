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

"""
State machine of the VR follow node (P6; docs/p6_plan.md).

No ROS, no clock of its own and no I/O: the node calls Follower.update() once per tick with the
time and the latest inputs, and sends what the returned Output says. The machine only decides.

States (Output.state) and their reasons:
  IDLE       nothing is sent. reason: START_WAIT, RELEASED, JOY_TIMEOUT, REFUSED a,b,...
  TRACKING   one point per tick. reason: OK, SPEED_LIMITED
  HOLD_JUMP  a pose message whose target is 50 mm / 30 deg or more from the target of the message
             before it (judged for every message, not once per tick; nothing is
             sent). Resumes when the target is back within 20 mm and 10 deg of the last accepted
             target.
  HOLD_RANGE the target cannot be made: UNREACHABLE, JOINT_RANGE, ELBOW_LIMIT, FLOOR, ORIENT,
             IK_JUMP (C1: the IK solution of a pose message is ik_jump rad or more from the
             solution of the message before it, judged for every message). Resumes like HOLD_JUMP.
  HOLD_LAG   the target is too far ahead of the commanded tip (C2). The accepted target follows
             the hand; resumes when the lag is back under half of the limit.
  STALE      the pose or the joint states are older than stale_s. Resumes like HOLD_JUMP.
  BUSY       another action goal is active (needs a new clutch).
  BLOCKED    the measured joints are far from the command (C3; needs a new clutch).

Every distance and angle of the judgements is in the space of the target of the arm in base_link
(after the scale and the projection). While the state is not TRACKING nothing is sent: the
trajectory controller keeps the last point. The gripper is independent of all of this: it only
reacts to the edges of the buttons.
"""

import dataclasses
from typing import Optional, Tuple

import numpy as np

from . import analytic_ik as ik
from . import follow_calc as fc

MAX_SOLVES_PER_TICK = 100  # pose messages of one tick whose IK solution is compared (C1)

IDLE = 'IDLE'
TRACKING = 'TRACKING'
HOLD_JUMP = 'HOLD_JUMP'
HOLD_RANGE = 'HOLD_RANGE'
HOLD_LAG = 'HOLD_LAG'
STALE = 'STALE'
BUSY = 'BUSY'
BLOCKED = 'BLOCKED'
RESUMABLE = (HOLD_JUMP, HOLD_RANGE, STALE)
NEEDS_NEW_CLUTCH = (BUSY, BLOCKED)


@dataclasses.dataclass(frozen=True)
class JoyState:
    """Levels of the three buttons (True while pressed)."""

    clutch: bool = False
    open: bool = False
    close: bool = False


@dataclasses.dataclass
class Inputs:
    """The latest inputs with the time they were received (the clock of update())."""

    pose: Optional[Tuple[np.ndarray, np.ndarray]] = None  # (position, rotation), base_link
    pose_time: Optional[float] = None
    poses: Optional[list] = None  # all pose messages since the last tick [(p, R), ...]
    joy: Optional[JoyState] = None
    joy_time: Optional[float] = None
    q_meas: Optional[np.ndarray] = None
    q_meas_time: Optional[float] = None
    q_desired: Optional[np.ndarray] = None  # the reference of the trajectory controller
    q_desired_time: Optional[float] = None
    busy: tuple = ()  # reasons (texts) why another action goal is active


@dataclasses.dataclass(frozen=True, eq=False)
class Output:
    """What the node must do: send point (if not None) and the gripper, publish the state."""

    state: str
    reason: str
    point: Optional[np.ndarray] = None  # joint angles of the one point to send
    target_p: Optional[np.ndarray] = None  # the commanded target (after scale and projection)
    target_r: Optional[np.ndarray] = None
    gripper: Optional[float] = None  # amount 0 (open) .. 1 (closed), only at a button edge
    detail: dict = dataclasses.field(default_factory=dict)


@dataclasses.dataclass(frozen=True)
class _Joy:
    fresh: bool
    level: bool
    clutch_edge: bool
    open_edge: bool
    close_edge: bool


class Follower:
    """Decides, once per tick, what to send to the arm and to the gripper."""

    def __init__(self, config=None):
        """Create the machine in IDLE."""
        self.config = fc.FollowConfig() if config is None else config
        self.state = IDLE
        self.reason = 'START_WAIT'
        self._last_time = None
        self._prev_buttons = None
        self._rebase = False
        self._clear()

    def _clear(self):
        self._p_ref = self._r_ref = self._p_in0 = self._r_in0 = None
        self._acc_p = self._acc_r = None
        self._prev_msg = None  # the target of the previous pose message
        self._prev_msg_q = None  # the IK solution of that target (None: not known)
        self._q_cmd = self._q_ik = None

    @property
    def scale(self):
        """Return the scale now in use."""
        return self.config.scale

    def set_scale(self, scale):
        """Change the scale (ValueError if invalid); while active the bases are taken again."""
        self.config = self.config.replace(scale=scale)
        if self.state != IDLE:
            self._rebase = True

    # ------------------------------------------------------------------------ inputs

    def _dt(self, now):
        period = self.config.period
        if self._last_time is None:
            dt = period
        else:
            dt = min(max(now - self._last_time, 1e-3), 2.0 * period)
        self._last_time = now
        return dt

    def _fresh(self, value, stamp, now):
        return value is not None and stamp is not None and now - stamp <= self.config.stale_s

    def _read_joy(self, now, inp):
        timeout = self.config.joy_timeout_s
        fresh = (inp.joy is not None and inp.joy_time is not None
                 and (timeout == 0.0 or now - inp.joy_time <= timeout))
        if not fresh:
            self._prev_buttons = None  # no edge when the buttons come back (re-synchronize)
            return _Joy(False, False, False, False, False)
        now_buttons = (inp.joy.clutch, inp.joy.open, inp.joy.close)
        previous = self._prev_buttons or (None, None, None)
        self._prev_buttons = now_buttons
        edges = [fc.rising(p, n) for p, n in zip(previous, now_buttons)]
        return _Joy(True, inp.joy.clutch, *edges)

    # ------------------------------------------------------------------------ update

    def update(self, now, inp):
        """Return the Output for this tick."""
        dt = self._dt(now)
        joy = self._read_joy(now, inp)
        gripper = None
        if joy.open_edge != joy.close_edge and (
                not self.config.gripper_requires_clutch or self.state != IDLE):
            gripper = 0.0 if joy.open_edge else 1.0
        out = self._run(now, dt, inp, joy)
        return dataclasses.replace(out, gripper=gripper)

    def _out(self, point=None, target_p=None, target_r=None, detail=None):
        return Output(self.state, self.reason, point, target_p, target_r, None, detail or {})

    def _set(self, state, reason):
        self.state, self.reason = state, reason

    def _to_idle(self, reason):
        self._set(IDLE, reason)
        self._clear()
        self._rebase = False

    def _run(self, now, dt, inp, joy):
        toggle = self.config.clutch_mode == 'toggle'
        if self.state == IDLE:
            if joy.clutch_edge:
                return self._try_start(now, inp)
            return self._out()
        if not joy.fresh:
            self._to_idle('JOY_TIMEOUT')
            return self._out()
        if (toggle and joy.clutch_edge) or (not toggle and not joy.level):
            self._to_idle('RELEASED')
            return self._out()
        if self.state in NEEDS_NEW_CLUTCH:
            return self._out()
        if inp.busy:
            self._set(BUSY, '; '.join(inp.busy))
            return self._out()
        if not self._fresh(inp.q_meas, inp.q_meas_time, now):
            self._set(STALE, 'JOINT_STATE')
            return self._out()
        if not self._fresh(inp.pose, inp.pose_time, now):
            self._set(STALE, 'POSE')
            return self._out()
        return self._follow(dt, inp)

    # ------------------------------------------------------------------------- start

    def _try_start(self, now, inp):
        cfg = self.config
        reasons = []
        if not self._fresh(inp.pose, inp.pose_time, now):
            reasons.append('POSE')
        if not self._fresh(inp.q_meas, inp.q_meas_time, now):
            reasons.append('JOINT_STATE')
        if not self._fresh(inp.q_desired, inp.q_desired_time, now):
            reasons.append('CONTROLLER_STATE')
        if inp.busy:
            reasons.append('BUSY')
        detail = {}
        if not reasons:
            q_meas = np.asarray(inp.q_meas, dtype=float)
            q_desired = np.asarray(inp.q_desired, dtype=float)
            windup = float(np.max(np.abs(q_desired - q_meas)))
            detail['windup'] = windup
            if windup > cfg.clutch_windup:
                reasons.append('WINDUP')
            if ik.fk(q_meas)[0][2] < cfg.floor_z:
                reasons.append('FLOOR')
            if not ik.within_limits(q_desired, q_max=cfg.q_max):
                reasons.append('RANGE')
        if reasons:
            self._set(IDLE, 'REFUSED ' + ','.join(reasons))
            return self._out(detail=detail)
        q_desired = np.asarray(inp.q_desired, dtype=float)
        self._p_ref = ik.fk(q_desired)[0]
        self._r_ref = fc.tip_rotation(q_desired)
        self._p_in0 = np.asarray(inp.pose[0], dtype=float)
        self._r_in0 = np.asarray(inp.pose[1], dtype=float)
        self._acc_p, self._acc_r = self._p_ref.copy(), self._r_ref.copy()
        self._prev_msg = (self._p_ref.copy(), self._r_ref.copy())
        self._prev_msg_q = q_desired.copy()
        self._q_cmd = q_desired.copy()
        self._q_ik = q_desired.copy()
        self._rebase = False
        self._set(TRACKING, 'OK')
        return self._out(detail=detail)

    # ------------------------------------------------------------------------ follow

    def _hold(self, state, reason, detail=None):
        self._set(state, reason)
        return self._out(detail=detail)

    def _messages(self, inp):
        """Return the pose messages of this tick (the latest one if the caller gave no list)."""
        if inp.poses:
            return [(np.asarray(p, dtype=float), np.asarray(r, dtype=float))
                    for p, r in inp.poses]
        return [(np.asarray(inp.pose[0], dtype=float), np.asarray(inp.pose[1], dtype=float))]

    def _follow(self, dt, inp):
        cfg = self.config
        messages = self._messages(inp)
        if self._rebase:
            p_in, r_in = messages[-1]
            self._p_ref, self._r_ref = self._acc_p.copy(), self._acc_r.copy()
            self._p_in0, self._r_in0 = p_in.copy(), r_in.copy()
            self._prev_msg = (self._acc_p.copy(), self._acc_r.copy())
            self._prev_msg_q = self._q_ik.copy()
            self._rebase = False
            messages = messages[-1:]
        was = self.state
        running = was in (TRACKING, HOLD_LAG)
        # The jumps are judged for every message of the input (not once per tick): a message
        # whose target is 50 mm / 30 deg or more from the target of the message before it, or
        # whose IK solution is ik_jump rad or more from the solution of the message before it
        # (C1: a real change of the branch). The analytic IK is cheap, so every message is solved.
        jumped = None
        ik_jumped = None
        ik_largest = 0.0
        first_solved = max(0, len(messages) - MAX_SOLVES_PER_TICK)
        for index, (p_in, r_in) in enumerate(messages):
            msg_p, msg_r = fc.relative_target(self._p_ref, self._r_ref, self._p_in0, self._r_in0,
                                              p_in, r_in, cfg.scale, cfg.r_fix)
            if running and jumped is None:
                d, a = fc.target_difference(self._prev_msg[0], self._prev_msg[1], msg_p, msg_r)
                if fc.is_jump(d, a, cfg):
                    jumped = (d, a)
            self._prev_msg = (msg_p, msg_r)
            if cfg.ik_jump <= 0.0:
                continue
            if index < first_solved:
                self._prev_msg_q = None  # too many messages: the chain of solutions starts again
                continue
            start = self._q_ik if self._prev_msg_q is None else self._prev_msg_q
            try:
                q_msg = fc.solve_target(msg_p, msg_r, start, cfg)
            except fc.TargetError:
                q_msg = None  # the range is judged below, for the target of the tick
            if q_msg is not None and self._prev_msg_q is not None:
                jump = float(np.max(np.abs(q_msg - self._prev_msg_q)))
                ik_largest = max(ik_largest, jump)
                if running and ik_jumped is None and jump >= cfg.ik_jump:
                    ik_jumped = (jump, index)
            self._prev_msg_q = q_msg
        cand_p, cand_r = self._prev_msg
        distance, angle = fc.target_difference(self._acc_p, self._acc_r, cand_p, cand_r)
        detail = {'dist_mm': distance * 1000.0, 'angle_deg': float(np.degrees(angle)),
                  'scale': cfg.scale, 'messages': len(messages)}
        if cfg.ik_jump > 0.0:
            detail['ik_jump'] = ik_largest
        if jumped is not None:
            detail['jump_mm'] = jumped[0] * 1000.0
            detail['jump_deg'] = float(np.degrees(jumped[1]))
            return self._hold(HOLD_JUMP, 'TARGET_JUMP', detail)
        if ik_jumped is not None:
            detail['message'] = ('the IK solution moved %.3f rad between two messages (message %d '
                                 'of %d)' % (ik_jumped[0], ik_jumped[1] + 1, len(messages)))
            return self._hold(HOLD_RANGE, 'IK_JUMP', detail)
        if not running and not fc.is_near(distance, angle, cfg):
            # keep waiting for the hand to return
            return self._hold(was, 'WAIT_RETURN' if was == STALE else self.reason, detail)
        q_cmd = self._q_cmd
        error = float(np.max(np.abs(np.asarray(inp.q_meas, dtype=float) - q_cmd)))
        detail['track_err'] = error
        if was != HOLD_LAG and cfg.max_tracking_error > 0.0 and error > cfg.max_tracking_error:
            return self._hold(BLOCKED, 'TRACKING_ERROR %.3f rad' % error, detail)
        if cfg.ik_jump > 0.0 and self._prev_msg_q is not None:
            q_ik = self._prev_msg_q.copy()  # the solution of the last message
        else:
            try:
                q_ik = fc.solve_target(cand_p, cand_r, self._q_ik, cfg)
            except fc.TargetError as target_error:
                detail['message'] = str(target_error)
                return self._hold(HOLD_RANGE, target_error.reason, detail)
        if not running and cfg.ik_jump > 0.0:
            # a resume: the solution must be near the last accepted one (not another branch)
            jump = float(np.max(np.abs(q_ik - self._q_ik)))
            if jump >= cfg.ik_jump:
                detail['message'] = 'the solution is %.3f rad from the last accepted one' % jump
                return self._hold(HOLD_RANGE, 'IK_JUMP', detail)
        lag = float(np.linalg.norm(cand_p - fc.tip_position(q_cmd)))
        detail['lag_mm'] = lag * 1000.0
        limit = cfg.max_lag / 2.0 if was == HOLD_LAG else cfg.max_lag
        if cfg.max_lag > 0.0 and lag > limit:
            self._acc_p, self._acc_r, self._q_ik = cand_p, cand_r, q_ik  # follow the hand
            return self._hold(HOLD_LAG, 'LAG', detail)
        q_next, limited = fc.limit_step(q_cmd, q_ik, dt, cfg.max_joint_speed)
        if fc.tip_position(q_next)[2] < cfg.floor_z:
            detail['message'] = 'the next point is below the floor'
            return self._hold(HOLD_RANGE, 'FLOOR', detail)
        self._acc_p, self._acc_r = cand_p, cand_r
        self._q_ik = q_ik
        self._q_cmd = q_next
        self._set(TRACKING, 'SPEED_LIMITED' if limited else 'OK')
        target_r = fc.tip_rotation(q_ik)
        detail['proj_deg'] = float(np.degrees(fc.rotation_angle(target_r.T @ cand_r)))
        return self._out(q_next.copy(), cand_p.copy(), target_r, detail)
