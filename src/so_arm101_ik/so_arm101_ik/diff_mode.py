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
Pure computation of the analytic differential mode (P4; docs/analytic_diff_plan.md 7.12).

The mode accumulates differences on the target of the tip (the integral term), solves the
analytic IK on the same elbow branch, and returns the next absolute joint command q_cmd.
It has no clock, no ROS and no I/O. The caller (P5, P6) owns the destination and the timing.

Units: m and rad. Joint order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll.

Principle. A check of the TARGET that fails is a boundary: the difference is shortened (same
direction) to the inside of the boundary and the mode continues (AT_LIMIT, or HOLD when nothing
is left to send). A check of the command q that is sent, or of the measured position, that fails
stops the mode (REFUSED). Running out of the budget of the mode (MAX_STEPS, REVERSAL) or
spinning at a limit (SPIN) ends the mode normally (LIMITED_DONE, never an error).

The origin of the accumulation is the previous command (the desired of the trajectory
controller), never a measured position. The measured joint angles q_meas are used only for
checks (wind-up, floor), for the error e = goal - FK(q_meas), and for reporting. The caller must
pass a q_meas measured after the arm has stopped (the "stop, then measure" way of use); the mode
has no clock and cannot check this, and it must not be called while the arm is moving.
"""

import dataclasses
import enum
import math
from typing import Optional, Tuple

import numpy as np

from . import analytic_ik as ik

JOINT_NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')

# Search of the largest fraction s of a difference that stays inside every boundary.
SCAN_POINTS = 16
BISECT_ITERATIONS = 30

# Tolerances of the checks that must not be loosened (rounding only).
NO_PROGRESS = 1e-7  # rad
TARGET_CONSISTENCY = 1e-6  # m
FLOOR_ROUNDING = 1e-9  # m
LIMIT_ROUNDING = 1e-9  # rad (dq and wind-up checks)
NOOP_DELTA = 1e-9  # m
TOUCH = 1e-6  # a difference touches a boundary if 1 + TOUCH times it would cross it
BRANCH_MATCH = 1e-6  # rad (q_desired against a candidate of the IK)

# Hard ceilings of the configuration (7.12.7 item 23).
HARD_MAX_DQ = 0.05
HARD_SHRINK_RATIO = 0.95
HARD_MAX_ABS_D = 0.99
HARD_MAX_WINDUP = 0.08
HARD_MAX_DISTANCE = 0.05
HARD_MAX_STEPS = 12
HARD_MIN_FLOOR = 0.045
HARD_MIN_RADIAL = 0.05
HARD_MAX_HOLD_RUN = 3
HARD_MAX_AT_LIMIT_RUN = 4
HARD_MAX_HOLDS = 8
HARD_HOLD_COS = 0.95
HARD_MARGIN_ABS_D = 0.01
HARD_MARGIN_RADIAL = 0.01
HARD_MARGIN_JOINT = 0.05
HARD_MARGIN_LENGTH = 1e-3

BOUNDARIES = ('UNREACHABLE', 'SINGULAR', 'RADIAL', 'JOINT_RANGE', 'FLOOR', 'DISTANCE')


class Status(enum.Enum):
    """Result of a step or of a suggestion."""

    OK = 'OK'
    AT_LIMIT = 'AT_LIMIT'
    HOLD = 'HOLD'
    IN_BAND = 'IN_BAND'
    LIMITED_DONE = 'LIMITED_DONE'
    REFUSED = 'REFUSED'


class Kind(enum.Enum):
    """Kind of the final result."""

    CONVERGED = 'CONVERGED'
    LIMITED = 'LIMITED'


class StartRefused(Exception):
    """The mode cannot start. reasons is the list of the codes, details the messages."""

    def __init__(self, reasons, details):
        super().__init__('; '.join(details))
        self.reasons = list(reasons)
        self.details = list(details)


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _is_count(value):
    return isinstance(value, int) and not isinstance(value, bool)


@dataclasses.dataclass(frozen=True)
class DiffModeConfig:
    """Values of the guards. The constructor checks the hard ceilings and the relations."""

    floor_z: float = 0.048
    measured_floor_z: float = 0.045
    min_radial: float = 0.05
    max_abs_d: float = 0.99
    max_dq: float = 0.05
    shrink_ratio: float = 0.9
    min_delta: float = 0.0005
    max_delta: float = 0.005
    max_windup: float = 0.06
    max_distance: float = 0.03
    max_steps: int = 10
    max_reversals: Optional[int] = 2
    deadband_enter: float = 0.003
    deadband_exit: float = 0.005
    margin_abs_d: float = 0.001
    margin_radial: float = 0.001
    margin_joint: float = 0.02
    margin_floor: float = 1e-6
    margin_distance: float = 1e-6
    max_hold_run: Optional[int] = 2
    max_at_limit_run: Optional[int] = 3
    max_holds: Optional[int] = 5
    hold_cos: float = 0.8

    def __post_init__(self):
        numbers = (
            self.floor_z, self.measured_floor_z, self.min_radial, self.max_abs_d, self.max_dq,
            self.shrink_ratio, self.min_delta, self.max_delta, self.max_windup,
            self.max_distance, self.deadband_enter, self.deadband_exit, self.margin_abs_d,
            self.margin_radial, self.margin_joint, self.margin_floor, self.margin_distance,
            self.hold_cos)
        _require(all(math.isfinite(v) for v in numbers), 'every value must be a finite number')
        _require(0.0 < self.max_dq <= HARD_MAX_DQ, 'max_dq must be in (0, %g]' % HARD_MAX_DQ)
        _require(0.0 < self.shrink_ratio <= HARD_SHRINK_RATIO,
                 'shrink_ratio must be in (0, %g]' % HARD_SHRINK_RATIO)
        _require(0.0 < self.max_abs_d <= HARD_MAX_ABS_D,
                 'max_abs_d must be in (0, %g]' % HARD_MAX_ABS_D)
        _require(self.max_dq < 2.0 * math.acos(self.max_abs_d),
                 'max_dq must be smaller than 2 acos(max_abs_d) (a branch switch must exceed it)')
        _require(0.0 < self.min_delta < self.deadband_enter <= self.deadband_exit,
                 'the limits must satisfy 0 < min_delta < deadband_enter <= deadband_exit')
        _require(self.max_delta > 0.0, 'max_delta must be positive')
        _require(0.0 < self.max_windup <= HARD_MAX_WINDUP,
                 'max_windup must be in (0, %g]' % HARD_MAX_WINDUP)
        _require(0.0 < self.max_distance <= HARD_MAX_DISTANCE,
                 'max_distance must be in (0, %g]' % HARD_MAX_DISTANCE)
        _require(_is_count(self.max_steps) and 1 <= self.max_steps <= HARD_MAX_STEPS,
                 'max_steps must be an integer in [1, %d]' % HARD_MAX_STEPS)
        _require(self.max_reversals is None
                 or (_is_count(self.max_reversals) and self.max_reversals >= 1),
                 'max_reversals must be None or an integer >= 1')
        _require(self.floor_z >= HARD_MIN_FLOOR and self.measured_floor_z >= HARD_MIN_FLOOR,
                 'the floors must be at least %g m' % HARD_MIN_FLOOR)
        _require(self.min_radial >= HARD_MIN_RADIAL,
                 'min_radial must be at least %g m' % HARD_MIN_RADIAL)
        _require(0.0 <= self.margin_abs_d <= HARD_MARGIN_ABS_D, 'margin_abs_d out of range')
        _require(0.0 <= self.margin_radial <= HARD_MARGIN_RADIAL, 'margin_radial out of range')
        _require(0.0 <= self.margin_joint <= HARD_MARGIN_JOINT, 'margin_joint out of range')
        _require(0.0 <= self.margin_floor <= HARD_MARGIN_LENGTH, 'margin_floor out of range')
        _require(0.0 <= self.margin_distance < self.max_distance
                 and self.margin_distance <= HARD_MARGIN_LENGTH,
                 'margin_distance out of range')
        for name, value, ceiling in (('max_hold_run', self.max_hold_run, HARD_MAX_HOLD_RUN),
                                     ('max_at_limit_run', self.max_at_limit_run,
                                      HARD_MAX_AT_LIMIT_RUN),
                                     ('max_holds', self.max_holds, HARD_MAX_HOLDS)):
            _require(value is None or (_is_count(value) and 1 <= value <= ceiling),
                     '%s must be None or an integer in [1, %d]' % (name, ceiling))
        _require(-1.0 < self.hold_cos <= HARD_HOLD_COS,
                 'hold_cos must be in (-1, %g]' % HARD_HOLD_COS)


@dataclasses.dataclass(frozen=True, eq=False)
class StepResult:
    """Result of step(). q_cmd is the command to send (the held one if nothing is sent)."""

    status: Status
    q_cmd: np.ndarray
    reason: Optional[str] = None
    limits: Tuple[str, ...] = ()
    detail: str = ''
    delta: Optional[np.ndarray] = None


@dataclasses.dataclass(frozen=True, eq=False)
class Outcome:
    """Final result: the best position that was reached and the remaining error."""

    kind: Kind
    goal: np.ndarray
    reached_xyz: np.ndarray
    commanded_xyz: np.ndarray
    residual: np.ndarray
    residual_norm: float
    limit_gap: np.ndarray
    track_gap: np.ndarray
    limit_reason: Optional[str]
    limits: Tuple[str, ...]
    steps: int
    holds: int
    within_tolerance: bool
    ok: bool = True


@dataclasses.dataclass(frozen=True, eq=False)
class Suggestion:
    """Result of suggest_delta(): the difference to pass to step()."""

    status: Status
    delta: Optional[np.ndarray] = None
    requested_norm: float = 0.0
    shrunk: bool = False
    limits: Tuple[str, ...] = ()
    reason: Optional[str] = None
    outcome: Optional[Outcome] = None


@dataclasses.dataclass(frozen=True, eq=False)
class DiffModeState:
    """Read-only copy of the state of a DiffMode."""

    target: np.ndarray
    target0: np.ndarray
    pitch_ref: float
    roll_ref: float
    q_cmd: np.ndarray
    q_start: np.ndarray
    branch: int
    steps: int
    reversals: int
    holds: int
    hold_run: int
    at_limit_run: int
    finished: bool
    finished_reason: Optional[str]
    goal: Optional[np.ndarray]
    halted: bool
    reason: Optional[str]


def branch_of_q(q):
    """Return the elbow branch (0: beta3 > 0, 1: beta3 < 0) of q, from q3 only."""
    c = ik.CONSTANTS
    beta = ik.normalize_angle(ik.S3 * float(q[2]) - math.pi / 2.0 - c.d2 + c.d3)
    return 0 if beta > 0.0 else 1


def abs_d_of_q(q):
    """Return abs(D) of the law of cosines, computed from q3 only (not through the IK)."""
    c = ik.CONSTANTS
    return abs(math.cos(ik.S3 * float(q[2]) - math.pi / 2.0 - c.d2 + c.d3))


def wrapped_norm(a, b):
    """Return the 2-norm of the joint differences a - b wrapped to (-pi, pi]."""
    return float(np.linalg.norm([ik.normalize_angle(x - y) for x, y in zip(a, b)]))


def max_fraction(is_feasible, points=SCAN_POINTS, iterations=BISECT_ITERATIONS):
    """
    Return (s, hi): the largest feasible fraction s of 0..1 and the first infeasible bound.

    is_feasible(s) is evaluated at s = k / points (k = 1..points); the first infeasible sample is
    narrowed down by bisection. s = 0 is assumed feasible. hi is None if every sample is
    feasible (then s = 1). A band thinner than 1 / points can be skipped by the scan.
    """
    previous = 0.0
    for k in range(1, points + 1):
        s = k / points
        if not is_feasible(s):
            low, high = previous, s
            for _ in range(iterations):
                middle = 0.5 * (low + high)
                if is_feasible(middle):
                    low = middle
                else:
                    high = middle
            return low, high
        previous = s
    return 1.0, None


def verify_command(q_new, q_prev, q_meas, branch, config, target=None):
    """
    Check the command that is about to be sent, from q only (independent of the solver).

    Return None if it may be sent, else (reason, detail). The checks, in this order: finite
    (NOT_FINITE), strict joint range (JOINT_LIMIT), floor of FK(q_new) (FLOOR_CMD), dq
    (DQ_TOO_BIG), wind-up (FINAL_CHECK), floor of FK(q_meas) (FLOOR_MEASURED), abs(D) from q3
    (FINAL_CHECK), elbow branch from q3 (BRANCH_SWITCH) and, if the target is given, FK(q_new)
    against the target (FINAL_CHECK). These checks are never loosened.
    """
    arrays = []
    for value in (q_new, q_prev, q_meas):
        try:
            array = np.asarray(value, dtype=float)
        except (TypeError, ValueError):
            return 'NOT_FINITE', 'a joint vector is not numeric'
        if array.shape != (5,) or not np.all(np.isfinite(array)):
            return 'NOT_FINITE', 'a joint vector is not 5 finite numbers'
        arrays.append(array)
    q_new, q_prev, q_meas = arrays
    if not ik.within_limits(q_new):
        return 'JOINT_LIMIT', 'the command is outside the strict joint range'
    xyz, _, _ = ik.fk(q_new)
    if xyz[2] < config.floor_z - FLOOR_ROUNDING:
        return 'FLOOR_CMD', 'the command puts the tip at z = %.4f m' % xyz[2]
    dq = wrapped_norm(q_new, q_prev)
    if dq > config.max_dq + LIMIT_ROUNDING:
        return 'DQ_TOO_BIG', 'the command changes the joints by %.4f rad' % dq
    if np.any(np.abs(q_new - q_meas) > config.max_windup + LIMIT_ROUNDING):
        return 'FINAL_CHECK', 'the command is too far from the measured joint angles'
    if ik.fk(q_meas)[0][2] < config.measured_floor_z - FLOOR_ROUNDING:
        return 'FLOOR_MEASURED', 'the measured tip is below the floor'
    if abs_d_of_q(q_new) > config.max_abs_d:
        return 'FINAL_CHECK', 'abs(D) of the command is above the limit'
    if branch_of_q(q_new) != branch:
        return 'BRANCH_SWITCH', 'the command is on the other elbow branch'
    if target is not None:
        if float(np.linalg.norm(xyz - np.asarray(target, dtype=float))) > TARGET_CONSISTENCY:
            return 'FINAL_CHECK', 'the command does not reach the target'
    return None


def _vector(value, length):
    """Return value as a float vector of the given length, or None if it is not one."""
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    return array if array.shape == (length,) else None


class DiffMode:
    """Accumulates differences of the tip target and returns the next joint command."""

    def __init__(self, config, q_desired, branch):
        """Create the state; use start(), which also checks the entry conditions."""
        self.config = config
        xyz, pitch, roll = ik.fk(q_desired)
        self._target = xyz.copy()
        self._target0 = xyz.copy()
        self._pitch_ref = pitch
        self._roll_ref = roll
        self._q_cmd = np.array(q_desired, dtype=float)
        self._q_start = self._q_cmd.copy()
        self._branch = branch
        self._steps = 0
        self._reversals = 0
        self._last_delta = None
        self._last_requested = 0.0
        self._holds = 0
        self._hold_run = 0
        self._hold_dir = None
        self._at_limit_run = 0
        self._spin_done = False
        self._spin_cause = None
        self._exhausted = None
        self._goal = None
        self._in_band = False
        self._last_limits = ()
        self._halted = False
        self._reason = None

    # ------------------------------------------------------------------ start

    @classmethod
    def start(cls, q_desired, q_meas, config=None):
        """Check the entry conditions and create the mode (StartRefused with every reason)."""
        config = DiffModeConfig() if config is None else config
        reasons, details = [], []

        def refuse(code, message):
            reasons.append(code)
            details.append(message)

        if q_desired is None:
            raise StartRefused(['NO_DESIRED'], ['the previous command (desired) is not available'])
        desired = _vector(q_desired, 5)
        if desired is None or not np.all(np.isfinite(desired)):
            raise StartRefused(['NOT_FINITE'], ['q_desired must be 5 finite numbers'])
        measured = None
        if q_meas is None:
            refuse('NO_MEASUREMENT', 'the measured joint angles are not available')
        else:
            measured = _vector(q_meas, 5)
            if measured is None or not np.all(np.isfinite(measured)):
                refuse('NOT_FINITE', 'q_meas must be 5 finite numbers')
                measured = None

        for i, name in enumerate(JOINT_NAMES):
            if desired[i] > ik.Q_MAX[i]:
                refuse('JOINT_LIMIT', '%s is %.4f rad above its upper limit' %
                       (name, desired[i] - ik.Q_MAX[i]))
            elif desired[i] < ik.Q_MIN[i]:
                refuse('JOINT_LIMIT', '%s is %.4f rad below its lower limit' %
                       (name, ik.Q_MIN[i] - desired[i]))

        xyz, pitch, roll = ik.fk(desired)
        if xyz[2] < config.floor_z:
            refuse('FLOOR', 'the tip is at z = %.4f m (floor %.3f m)' % (xyz[2], config.floor_z))
        branch = None
        try:
            geometry = ik.ik_geometry(xyz[0], xyz[1], xyz[2], pitch, roll)
        except ik.IKUnreachableError:
            geometry = None
            refuse('NO_BRANCH', 'the pose is not reproduced by the analytic IK')
        if geometry is not None:
            if geometry.radial < config.min_radial:
                refuse('RADIAL', 'the tip is %.3f m from the pan axis (minimum %.3f m)' %
                       (geometry.radial, config.min_radial))
            if abs(geometry.cos_elbow) > config.max_abs_d:
                refuse('SINGULAR', 'abs(D) = %.3f is above %.3f' %
                       (abs(geometry.cos_elbow), config.max_abs_d))
            matches = [i for i, c in enumerate(geometry.candidates)
                       if float(np.max(np.abs(c - desired))) <= BRANCH_MATCH]
            if len(matches) == 1:
                branch = matches[0]
            else:
                refuse('NO_BRANCH', 'q_desired is not on a unique branch of the analytic IK')
        if measured is not None:
            if np.any(np.abs(desired - measured) > config.max_windup):
                refuse('NOT_SETTLED', 'q_desired and q_meas differ by more than %.3f rad' %
                       config.max_windup)
            if ik.fk(measured)[0][2] < config.measured_floor_z:
                refuse('FLOOR_MEASURED', 'the measured tip is below %.3f m' %
                       config.measured_floor_z)
        if reasons:
            details.append('start from the neighbourhood of the pose T '
                           '(pan 0, lift -22, elbow 30, wrist_flex 83, roll 0 deg), not from '
                           'the start pose')
            raise StartRefused(reasons, details)
        return cls(config, desired, branch)

    # ------------------------------------------------------------------ state

    @property
    def state(self):
        """Return a read-only copy of the state."""
        return DiffModeState(
            target=self._target.copy(), target0=self._target0.copy(),
            pitch_ref=self._pitch_ref, roll_ref=self._roll_ref, q_cmd=self._q_cmd.copy(),
            q_start=self._q_start.copy(), branch=self._branch, steps=self._steps,
            reversals=self._reversals, holds=self._holds, hold_run=self._hold_run,
            at_limit_run=self._at_limit_run,
            finished=self._exhausted is not None or self._spin_done,
            finished_reason=self._end_reason(),
            goal=None if self._goal is None else self._goal.copy(), halted=self._halted,
            reason=self._reason)

    def _end_reason(self):
        if self._exhausted is not None:
            return self._exhausted
        if self._spin_done:
            return 'NO_ROOM' if self._spin_cause == 'hold_run' else 'SPIN'
        return None

    # ------------------------------------------------------------- boundaries

    def _excess(self, target):
        """
        Return {boundary: [excess, ...]} of a target; an excess above 0 is outside the limit.

        The limit is the inside limit (limit value minus margin). The joint range is checked on
        the fixed elbow branch only. UNREACHABLE appears only when the IK has no solution.
        """
        cfg = self.config
        excess = {}
        try:
            geometry = ik.ik_geometry(target[0], target[1], target[2],
                                      self._pitch_ref, self._roll_ref)
        except ik.IKUnreachableError:
            geometry = None
            excess['UNREACHABLE'] = [1.0]
        if geometry is not None:
            q = geometry.candidates[self._branch]
            excess['SINGULAR'] = [abs(geometry.cos_elbow) - (cfg.max_abs_d - cfg.margin_abs_d)]
            excess['RADIAL'] = [(cfg.min_radial + cfg.margin_radial) - geometry.radial]
            upper = q - (ik.Q_MAX - cfg.margin_joint)
            lower = (ik.Q_MIN + cfg.margin_joint) - q
            excess['JOINT_RANGE'] = [float(v) for v in upper] + [float(v) for v in lower]
        excess['FLOOR'] = [(cfg.floor_z + cfg.margin_floor) - float(target[2])]
        excess['DISTANCE'] = [float(np.linalg.norm(target - self._target0))
                              - (cfg.max_distance - cfg.margin_distance)]
        return excess

    def _violated(self, target, base):
        """Return the boundaries that the target violates (no worse than the start is allowed)."""
        violated = []
        for kind, values in self._excess(target).items():
            if kind == 'UNREACHABLE':
                violated.append(kind)
                continue
            start = base.get(kind)
            for i, value in enumerate(values):
                allowed = 0.0 if start is None else max(0.0, start[i])
                if value > allowed:
                    violated.append(kind)
                    break
        return violated

    def _dq(self, target):
        """Return the joint change to a target on the fixed branch (inf if unreachable)."""
        try:
            geometry = ik.ik_geometry(target[0], target[1], target[2],
                                      self._pitch_ref, self._roll_ref)
        except ik.IKUnreachableError:
            return math.inf
        return wrapped_norm(geometry.candidates[self._branch], self._q_cmd)

    def _plan(self, delta, with_dq):
        """
        Return (s, limits, dq_bound): the largest fraction of delta inside every boundary.

        limits are the boundaries that stop it (or that it touches when it fits entirely).

        Every boundary (and, with with_dq, the shrunk dq limit) is one predicate; s is found
        with one scan and one bisection of the product, so that all boundaries are satisfied.
        """
        base = self._excess(self._target)
        dq_limit = self.config.shrink_ratio * self.config.max_dq

        def feasible(s):
            point = self._target + s * delta
            if self._violated(point, base):
                return False
            return not (with_dq and self._dq(point) > dq_limit)

        s, high = max_fraction(feasible)
        if high is None:
            # the whole difference fits; it still touches a boundary if a hair more would not
            beyond = self._target + (1.0 + TOUCH) * delta
            touching = []
            for kind in self._violated(beyond, base):
                if kind not in touching:
                    touching.append(kind)
            return s, tuple(touching), False
        point = self._target + high * delta
        limits = []
        for kind in self._violated(point, base):
            if kind not in limits:
                limits.append(kind)
        return s, tuple(limits), with_dq and self._dq(point) > dq_limit

    # ------------------------------------------------------------------- step

    def _refuse(self, reason, detail=''):
        self._halted = True
        self._reason = reason
        return StepResult(Status.REFUSED, self._q_cmd.copy(), reason, (), detail)

    def _done(self, reason):
        return StepResult(Status.LIMITED_DONE, self._q_cmd.copy(), reason)

    def step(self, delta_xyz, q_meas, requested_norm=None):
        """
        Add a difference to the target and return the next command (see the module docstring).

        requested_norm is the size of the difference before it was shortened (default: its
        own size). It is compared with the size of a difference that reverses the direction.
        A refusal does not change the state (the mode is only marked as halted).
        """
        cfg = self.config
        if self._halted:
            return StepResult(Status.REFUSED, self._q_cmd.copy(), 'HALTED')
        if self._exhausted is not None:
            return self._done(self._exhausted)
        delta = _vector(delta_xyz, 3)
        if delta is None:
            return self._refuse('BAD_REQUEST', 'delta must be 3 numbers')
        if not np.all(np.isfinite(delta)):
            return self._refuse('NOT_FINITE', 'delta is not finite')
        if q_meas is None:
            return self._refuse('NO_MEASUREMENT', 'q_meas is required')
        measured = _vector(q_meas, 5)
        if measured is None:
            return self._refuse('BAD_REQUEST', 'q_meas must be 5 numbers')
        if not np.all(np.isfinite(measured)):
            return self._refuse('NOT_FINITE', 'q_meas is not finite')
        size = float(np.linalg.norm(delta))
        if size > cfg.max_delta + 1e-12:
            return self._refuse('DELTA_TOO_BIG', 'delta is %.4f m' % size)
        requested = size if requested_norm is None else requested_norm
        if (not isinstance(requested, (int, float)) or not math.isfinite(requested)
                or requested < size - 1e-12 or requested > cfg.max_delta + 1e-12):
            return self._refuse('BAD_REQUEST', 'requested_norm is not consistent with delta')
        if self._steps >= cfg.max_steps:
            self._exhausted = 'MAX_STEPS'
            return self._done('MAX_STEPS')
        if size <= NOOP_DELTA:
            return StepResult(Status.OK, self._q_cmd.copy(), None, (), 'no change',
                              np.zeros(3))

        reverse = (self._last_delta is not None
                   and float(np.dot(delta, self._last_delta)) < 0.0)
        if reverse:
            if requested > self._last_requested + 1e-12:
                return self._refuse('REVERSAL_GROWTH',
                                    'a reversing step must not be larger than the previous one')
            if cfg.max_reversals is not None and self._reversals + 1 >= cfg.max_reversals:
                self._exhausted = 'REVERSAL'
                return self._done('REVERSAL')

        s, limits, _ = self._plan(delta, with_dq=False)
        sent = s * delta
        status = Status.OK
        if limits:
            self._last_limits = limits
        if s < 1.0 and float(np.linalg.norm(sent)) < cfg.min_delta:
            return StepResult(Status.HOLD, self._q_cmd.copy(), 'DELTA_TOO_SMALL', limits,
                              'no room inside the boundary', np.zeros(3))
        if limits:
            status = Status.AT_LIMIT

        target_new = self._target + sent
        try:
            geometry = ik.ik_geometry(target_new[0], target_new[1], target_new[2],
                                      self._pitch_ref, self._roll_ref)
        except ik.IKUnreachableError:
            return self._refuse('FINAL_CHECK', 'the shortened target is unreachable')
        q_new = geometry.candidates[self._branch].copy()

        if wrapped_norm(q_new, self._q_cmd) > cfg.max_dq + LIMIT_ROUNDING:
            return self._refuse('DQ_TOO_BIG', 'the joints would change by %.4f rad' %
                                wrapped_norm(q_new, self._q_cmd))
        clipped = np.clip(q_new, measured - cfg.max_windup, measured + cfg.max_windup)
        if np.any(clipped != q_new):
            q_new = clipped
            target_new = ik.fk(q_new)[0]
            if wrapped_norm(q_new, self._q_cmd) <= NO_PROGRESS:
                return self._refuse('STUCK', 'the wind-up clip leaves no progress')

        verdict = verify_command(q_new, self._q_cmd, measured, self._branch, cfg,
                                 target=target_new)
        if verdict is not None:
            return self._refuse(verdict[0], verdict[1])

        self._q_cmd = q_new
        self._target = target_new
        self._steps += 1
        if reverse:
            self._reversals += 1
        self._last_delta = sent.copy()
        self._last_requested = requested
        self._hold_run = 0
        self._hold_dir = None
        if status is Status.AT_LIMIT:
            self._at_limit_run += 1
            if cfg.max_at_limit_run is not None and self._at_limit_run >= cfg.max_at_limit_run:
                self._spin_done = True
                self._spin_cause = 'at_limit_run'
        else:
            self._at_limit_run = 0
        return StepResult(status, q_new.copy(), None, limits if status is Status.AT_LIMIT else (),
                          '', sent.copy())

    # --------------------------------------------------------------- outcome

    def outcome(self, goal_xyz, q_meas):
        """Return the final result for a goal and a measurement (read only)."""
        goal = np.asarray(goal_xyz, dtype=float)
        reached = ik.fk(np.asarray(q_meas, dtype=float))[0]
        commanded = self._target.copy()
        residual = goal - reached
        norm = float(np.linalg.norm(residual))
        converged = norm <= self.config.deadband_enter
        reason = None
        if not converged:
            reason = self._end_reason()
            if reason is None and self._last_limits:
                reason = self._last_limits[0]
        return Outcome(
            kind=Kind.CONVERGED if converged else Kind.LIMITED, goal=goal.copy(),
            reached_xyz=reached.copy(), commanded_xyz=commanded, residual=residual,
            residual_norm=norm, limit_gap=goal - commanded, track_gap=commanded - reached,
            limit_reason=reason, limits=tuple(self._last_limits), steps=self._steps,
            holds=self._holds, within_tolerance=converged)

    # --------------------------------------------------------------- suggest

    def _refuse_suggestion(self, reason):
        self._halted = True
        self._reason = reason
        return Suggestion(Status.REFUSED, reason=reason)

    def _hold(self, direction, requested, limits):
        """Count a HOLD; return the Suggestion (LIMITED_DONE when it has gone on too long)."""
        cfg = self.config
        self._holds += 1
        if (self._hold_dir is not None
                and float(np.dot(direction, self._hold_dir)) >= cfg.hold_cos):
            self._hold_run += 1
        else:
            self._hold_run = 1
        self._hold_dir = direction
        self._last_limits = limits
        cause = None
        if cfg.max_hold_run is not None and self._hold_run >= cfg.max_hold_run:
            cause = 'hold_run'
        elif cfg.max_holds is not None and self._holds >= cfg.max_holds:
            cause = 'holds'
        if cause is not None:
            self._spin_done = True
            self._spin_cause = cause
            return Suggestion(Status.LIMITED_DONE, None, requested, False, limits,
                              self._end_reason())
        return Suggestion(Status.HOLD, None, requested, False, limits, 'DELTA_TOO_SMALL')

    def suggest_delta(self, goal_xyz, q_meas, gain=1.0):
        """
        Return the difference to send for a goal: the error, shortened to the boundaries and dq.

        status IN_BAND: the error is within the dead band (the outcome is attached).
        status OK / AT_LIMIT: send delta with step(delta, q_meas, requested_norm).
        status HOLD: send nothing (the mode continues). LIMITED_DONE: nothing is sent and the
        mode has ended normally (the outcome is attached). REFUSED: the mode is halted.
        Nothing but the dead band, the spin counters and the goal is changed.
        """
        cfg = self.config
        if self._halted:
            return Suggestion(Status.REFUSED, reason='HALTED')
        goal = _vector(goal_xyz, 3)
        if goal is None or not np.all(np.isfinite(goal)):
            return self._refuse_suggestion('NOT_FINITE')
        if q_meas is None:
            return self._refuse_suggestion('NO_MEASUREMENT')
        measured = _vector(q_meas, 5)
        if measured is None or not np.all(np.isfinite(measured)):
            return self._refuse_suggestion('NOT_FINITE')
        if not (isinstance(gain, (int, float)) and math.isfinite(gain) and gain > 0.0):
            return self._refuse_suggestion('BAD_REQUEST')

        if (self._goal is not None
                and float(np.linalg.norm(goal - self._goal)) > cfg.deadband_exit):
            self._holds = 0
            self._hold_run = 0
            self._hold_dir = None
            self._at_limit_run = 0
            self._spin_done = False
        self._goal = goal.copy()

        error = goal - ik.fk(measured)[0]
        norm = float(np.linalg.norm(error))
        if self._exhausted is not None:
            return Suggestion(Status.LIMITED_DONE, reason=self._exhausted,
                              outcome=self.outcome(goal, measured))
        if norm <= cfg.deadband_enter or (self._in_band and norm <= cfg.deadband_exit):
            self._in_band = True
            return Suggestion(Status.IN_BAND, outcome=self.outcome(goal, measured))
        self._in_band = False

        delta0 = gain * error
        size0 = float(np.linalg.norm(delta0))
        if size0 > cfg.max_delta:
            delta0 = delta0 * (cfg.max_delta / size0)
            size0 = cfg.max_delta
        direction = delta0 / size0
        if self._spin_done:
            if (self._spin_cause == 'hold_run' and self._hold_dir is not None
                    and float(np.dot(direction, self._hold_dir)) < cfg.hold_cos):
                self._spin_done = False
                self._hold_run = 0
            else:
                return Suggestion(Status.LIMITED_DONE, None, size0, False, self._last_limits,
                                  self._end_reason(), self.outcome(goal, measured))
        if self._steps >= cfg.max_steps:
            self._exhausted = 'MAX_STEPS'
            return Suggestion(Status.LIMITED_DONE, reason='MAX_STEPS',
                              outcome=self.outcome(goal, measured))

        s, limits, _ = self._plan(delta0, with_dq=True)
        delta = s * delta0
        if s < 1.0 and float(np.linalg.norm(delta)) < cfg.min_delta:
            result = self._hold(direction, size0, limits)
            if result.status is Status.LIMITED_DONE:
                return dataclasses.replace(result, outcome=self.outcome(goal, measured))
            return result
        if limits:
            return Suggestion(Status.AT_LIMIT, delta, size0, s < 1.0, limits)
        return Suggestion(Status.OK, delta, size0, s < 1.0)
