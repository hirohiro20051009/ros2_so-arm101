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
Steps of "move, stop, measure" around the differential mode (P5; docs/analytic_diff_plan.md 7.14).

This module has no ROS, no clock of its own and no I/O: the caller supplies an `io` object (the
inputs, the time, the confirmation, the status of the actions) and a `sender` (the destination:
the trajectory controller directly, or MoveIt). StepLoop only decides; it never sends anything
itself. Units: m, rad, s. Joint order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll.

Rules that this module keeps (docs 7.14.1 to 7.14.3):
- Before EVERY send (not only at the start): the inputs are read again and must be fresh, the
  measured angles must not have moved, the desired of the controller must equal the previous
  command (tolerance per route), no other action has an active goal, and verify_command() of the
  differential mode is run again with the new measurement.
- A goal is never cancelled (a cancel makes the controller hold the MEASURED position).
- The status SUCCEEDED of the controller is not a proof of arrival (open_loop_control: true):
  the result is judged from the measurement after the arm has stopped.
- The goal sent to /move_action is built by build_plan_goal_spec(), whose plan_only is always
  true: MoveIt only plans, and the trajectory that was checked is what gets executed.
"""

import dataclasses
import math
from typing import Optional, Tuple

import numpy as np

from . import analytic_ik as ik
from . import diff_mode as dm
from .diff_mode import DiffMode, DiffModeConfig, Status, StartRefused, verify_command

JOINT_NAMES = dm.JOINT_NAMES
ROUTES = ('direct', 'moveit')

# Inputs and waiting (docs 7.14.2, 7.14.3). Values marked (guess) have no measurement behind them.
MAX_AGE = 0.3  # s, age of an input (guess)
INPUT_WAIT = 5.0  # s, wait for the first inputs (send_joint_delta.py)
CONFIRM_TIMEOUT = 30.0  # s, wait for the Enter of the user
STOP_TOL = 0.003  # rad, command A of servo_real_check.md
STOP_INTERVAL = 0.6  # s, command A
SETTLE_MIN = 1.5  # s, so101_diffik.py (old code)
SETTLE_TIMEOUT = 8.0  # s (guess)
ACCEPT_TIMEOUT = 5.0  # s (guess)
RESULT_MARGIN = 3.0  # s (guess)
PLAN_TIMEOUT = 5.0  # s, allowed_planning_time of ros_bridge.py (old code)
RUN_TIMEOUT = 120.0  # s, docs 2.5 (guess)
MOVE_SECONDS = 2.0  # s, time_from_start of a direct goal (old code: STEP_T)

# Speeds (docs 7.14.3)
DIRECT_MAX_SPEED = 0.1  # rad/s, MAX_SPEED of send_joint_delta.py
MOVEIT_MAX_SPEED = 0.5  # rad/s, max_velocity 5.0 x scaling 0.1 (the upper limit by calculation)

# desired against the previous command (docs 7.14.1)
DESIRED_TOL_DIRECT = 1e-4  # rad (guess)
DESIRED_TOL_MOVEIT_FACTOR = 2.0  # times moveit_tolerance
MOVEIT_TOLERANCE = 0.001  # rad, JointConstraint tolerance (ros_bridge.py used 0.01)
MOVEIT_TOLERANCE_MAX = 0.01  # rad
MOVEIT_SCALING = 0.1  # fixed, no argument
MOVEIT_GROUP = 'arm'
MOVEIT_PLANNING_ATTEMPTS = 10  # ros_bridge.py (old code)

# first point of a planned trajectory (docs 7.14.3, decision 19)
FIRST_POINT_DESIRED_TOL = 0.005  # rad: about 3 ticks, the dead band when closing (measured)
FIRST_POINT_MEAS_TOL = 0.01  # rad: allowed_start_tolerance of MoveIt (servo_real_check.md W6)

ORIGIN_RADIUS = 0.05  # m, HARD_MAX_DISTANCE of the differential mode
ORIGIN_RADIUS_MAX = dm.HARD_MAX_DISTANCE
DEFAULT_MAX_DQ = 0.03  # rad, the first value on the real arm (docs 7.14.4)

ACTIVE_STATUSES = (1, 2, 3)  # action_msgs GoalStatus: ACCEPTED, EXECUTING, CANCELING

EXIT_OK = 0
EXIT_STOPPED = 2
EXIT_START_REFUSED = 3
EXIT_INTERRUPTED = 130

CSV_FIELDS = (
    'step', 'status', 'limits', 'route', 'q_meas_before', 'q_meas_after', 'q_cmd',
    'q_desired_before', 'q_desired_after', 'tip_before', 'tip_after', 'tip_move_over_delta',
    'delta', 'residual_norm', 'limit_gap', 'track_gap', 'abs_d', 'cmd_speed',
    'plan_peak_pos_speed', 'plan_peak_vel_speed', 'plan_duration', 'plan_points',
    'first_vs_desired', 'first_vs_meas', 'last_vs_cmd', 'plan_ok', 'plan_reasons',
    'windup_after', 'wait_settle', 'note')


# --------------------------------------------------------------------------- small helpers

def is_stopped(first, second, tol=STOP_TOL):
    """Return True if every joint moved by at most tol between two readings (command A)."""
    a = _five(first)
    b = _five(second)
    if a is None or b is None:
        return False
    return bool(np.max(np.abs(a - b)) <= tol)


def positions_by_name(names, positions, wanted=JOINT_NAMES):
    """Return the positions of the wanted joints in their order, or None if one is missing."""
    try:
        names = list(names)
        positions = list(positions)
    except TypeError:
        return None
    if len(names) != len(positions):
        return None
    table = dict(zip(names, positions))
    try:
        values = [float(table[name]) for name in wanted]
    except (KeyError, TypeError, ValueError):
        return None
    array = np.array(values, dtype=float)
    return array if np.all(np.isfinite(array)) else None


def is_fresh(received_at, now, max_age=MAX_AGE):
    """Return True if an input received at received_at is not older than max_age at now."""
    if received_at is None or now is None:
        return False
    age = now - received_at
    return bool(math.isfinite(age) and 0.0 <= age <= max_age)


def desired_tolerance(route, moveit_tolerance=MOVEIT_TOLERANCE):
    """Return the tolerance of desired against the previous command for a route."""
    if route == 'direct':
        return DESIRED_TOL_DIRECT
    if route == 'moveit':
        return DESIRED_TOL_MOVEIT_FACTOR * moveit_tolerance
    raise ValueError('route must be one of %s' % (ROUTES,))


def desired_matches(q_desired, q_cmd, route, moveit_tolerance=MOVEIT_TOLERANCE):
    """Return True if the desired of the controller equals the previous command (all joints)."""
    a = _five(q_desired)
    b = _five(q_cmd)
    if a is None or b is None:
        return False
    return bool(np.max(np.abs(a - b)) <= desired_tolerance(route, moveit_tolerance))


def check_origin(origin, q_desired, radius=ORIGIN_RADIUS):
    """Return None if the start is inside the sphere around origin (origin None: no check)."""
    if origin is None:
        return None
    desired = _five(q_desired)
    if desired is None:
        return 'q_desired is not 5 finite numbers'
    distance = float(np.linalg.norm(ik.fk(desired)[0] - np.asarray(origin, dtype=float)))
    if distance > radius:
        return ('the start is %.4f m from the origin (the radius is %.4f m)' % (distance, radius))
    return None


def check_direct_speed(q_from, q_to, seconds, max_speed=DIRECT_MAX_SPEED):
    """Return None if moving q_from to q_to in seconds is slow enough, else the reason."""
    a = _five(q_from)
    b = _five(q_to)
    if a is None or b is None:
        return 'a joint vector is not 5 finite numbers'
    if not (isinstance(seconds, (int, float)) and math.isfinite(seconds) and seconds > 0.0):
        return 'the time of the move must be positive'
    speed = float(np.max(np.abs(b - a))) / seconds
    if speed > max_speed:
        return 'the speed %.3f rad/s is above %.3f rad/s' % (speed, max_speed)
    return None


def direct_speed(q_from, q_to, seconds):
    """Return the largest joint speed [rad/s] of a move (for the record)."""
    return float(np.max(np.abs(np.asarray(q_to, float) - np.asarray(q_from, float)))) / seconds


def _five(value):
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if array.shape != (5,) or not np.all(np.isfinite(array)):
        return None
    return array


# ------------------------------------------------------------------ planned trajectory (moveit)

@dataclasses.dataclass(frozen=True)
class TrajPoint:
    """One point of a trajectory: the time from the start [s], positions, velocities or None."""

    time: float
    positions: Tuple[float, ...]
    velocities: Optional[Tuple[float, ...]] = None


@dataclasses.dataclass(frozen=True)
class PlanCheck:
    """Result of check_planned_trajectory(): ok, the reasons if not, and the numbers to record."""

    ok: bool
    reasons: Tuple[str, ...]
    peak_pos_speed: float
    peak_vel_speed: float
    duration: float
    points: int
    first_vs_desired: float
    first_vs_meas: float
    last_vs_cmd: float


def trajectory_speeds(points):
    """Return (peak speed from the position differences, peak speed from the velocities)."""
    peak_pos = 0.0
    peak_vel = 0.0
    previous = None
    for point in points:
        if point.velocities is not None:
            peak_vel = max(peak_vel, float(np.max(np.abs(np.asarray(point.velocities, float)))))
        if previous is not None:
            dt = point.time - previous.time
            if dt > 0.0:
                step = np.abs(np.asarray(point.positions, float)
                              - np.asarray(previous.positions, float))
                peak_pos = max(peak_pos, float(np.max(step)) / dt)
        previous = point
    return peak_pos, peak_vel


def check_planned_trajectory(points, joint_names, q_cmd, q_desired, q_meas,
                             tolerance=MOVEIT_TOLERANCE, max_speed=MOVEIT_MAX_SPEED,
                             first_desired_tol=FIRST_POINT_DESIRED_TOL,
                             first_meas_tol=FIRST_POINT_MEAS_TOL):
    """
    Check a trajectory that MoveIt planned, before it is executed (docs 7.14.3).

    The first point is compared with q_desired (the reference of the controller), not with the
    measured position: a plan that starts at the measured position moves the target of the
    controller there at the instant it is executed. It is also compared with q_meas (the
    allowed_start_tolerance of MoveIt). The last point must reach q_cmd within the tolerance, the
    speed must not exceed max_speed, the joints must be exactly the five joints in order and
    every value must be finite. Any failed check forbids the execution.
    """
    reasons = []
    nan = float('nan')
    points = list(points) if points is not None else []
    if list(joint_names) != list(JOINT_NAMES):
        reasons.append('NAMES')
    vectors = [_five(q_cmd), _five(q_desired), _five(q_meas)]
    if any(v is None for v in vectors):
        reasons.append('NOT_FINITE')
    finite = True
    for point in points:
        if _five(point.positions) is None or not math.isfinite(point.time):
            finite = False
        if point.velocities is not None and _five(point.velocities) is None:
            finite = False
    if not finite:
        reasons.append('NOT_FINITE')
    if len(points) < 2 or not finite:
        if len(points) < 2:
            reasons.append('EMPTY')
        reasons = tuple(dict.fromkeys(reasons))
        return PlanCheck(False, reasons, nan, nan, 0.0, len(points), nan, nan, nan)
    duration = float(points[-1].time - points[0].time)
    if not duration > 0.0:
        reasons.append('EMPTY')
    peak_pos, peak_vel = trajectory_speeds(points)
    first = np.asarray(points[0].positions, float)
    last = np.asarray(points[-1].positions, float)
    first_vs_desired = nan
    first_vs_meas = nan
    last_vs_cmd = nan
    if vectors[0] is not None:
        last_vs_cmd = float(np.max(np.abs(last - vectors[0])))
        if not last_vs_cmd <= tolerance:
            reasons.append('LAST')
    if vectors[1] is not None:
        first_vs_desired = float(np.max(np.abs(first - vectors[1])))
        if not first_vs_desired <= first_desired_tol:
            reasons.append('FIRST_DESIRED')
    if vectors[2] is not None:
        first_vs_meas = float(np.max(np.abs(first - vectors[2])))
        if not first_vs_meas <= first_meas_tol:
            reasons.append('FIRST_MEASURED')
    if not (peak_pos <= max_speed and peak_vel <= max_speed):
        reasons.append('SPEED')
    reasons = tuple(dict.fromkeys(reasons))
    return PlanCheck(not reasons, reasons, peak_pos, peak_vel, duration, len(points),
                     first_vs_desired, first_vs_meas, last_vs_cmd)


@dataclasses.dataclass(frozen=True)
class PlanGoalSpec:
    """Content of the goal for /move_action. It only plans: plan_only cannot be changed."""

    group: str
    joint_names: Tuple[str, ...]
    positions: Tuple[float, ...]
    tolerance: float
    scaling: float = MOVEIT_SCALING
    is_diff: bool = True
    planning_attempts: int = MOVEIT_PLANNING_ATTEMPTS
    planning_time: float = PLAN_TIMEOUT

    @property
    def plan_only(self):
        """Return True, always (MoveIt plans; the checked trajectory is executed afterwards)."""
        return True


def build_plan_goal_spec(q_cmd, joint_names, tolerance=MOVEIT_TOLERANCE):
    """Build the goal for /move_action (the scaling is fixed and plan_only is always true)."""
    q = _five(q_cmd)
    if q is None:
        raise ValueError('q_cmd must be 5 finite numbers')
    names = tuple(joint_names)
    if names != tuple(JOINT_NAMES):
        raise ValueError('joint_names must be %s' % (JOINT_NAMES,))
    if not (isinstance(tolerance, (int, float)) and math.isfinite(tolerance)
            and 0.0 < tolerance <= MOVEIT_TOLERANCE_MAX):
        raise ValueError('tolerance must be in (0, %g] rad' % MOVEIT_TOLERANCE_MAX)
    return PlanGoalSpec(MOVEIT_GROUP, names, tuple(float(v) for v in q), float(tolerance))


# ------------------------------------------------------------------------ results and options

def classify_goal_result(kind, status, error_code):
    """
    Return (ok, text) for the end of a goal. SUCCEEDED is not an arrival (open loop control).

    kind is 'direct' (FollowJointTrajectory: error_code 0 is success) or 'moveit'
    (ExecuteTrajectory: error_code 1 is success). ok only means "accepted and ended normally".
    """
    names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
    if status != 4:
        return False, 'the goal ended with status %s' % names.get(status, status)
    success = {'direct': 0, 'moveit': 1}.get(kind)
    if success is None:
        raise ValueError('kind must be one of %s' % (ROUTES,))
    if error_code != success:
        return False, 'SUCCEEDED but error_code is %s (expected %s)' % (error_code, success)
    return True, 'SUCCEEDED (this is not a proof of arrival; the measurement decides)'


def validate_run_options(route, dry_run=False, assume_desired_is_measured=False, goal=None,
                         offset=None, auto=None, origin=None, origin_radius=ORIGIN_RADIUS,
                         max_dq=DEFAULT_MAX_DQ, max_steps=10, max_abs_d=0.99, gain=1.0,
                         moveit_tolerance=MOVEIT_TOLERANCE, move_seconds=MOVE_SECONDS):
    """
    Check the combination of the options and return the DiffModeConfig (ValueError if wrong).

    Called right after the arguments are read and before any node or client is made.
    """
    if route not in ROUTES:
        raise ValueError('--route is required and must be one of %s' % (ROUTES,))
    if assume_desired_is_measured and not dry_run:
        raise ValueError('--assume-desired-is-measured is only accepted together with --dry-run')
    if (goal is None) == (offset is None):
        raise ValueError('give exactly one of --goal and --offset')
    for name, value in (('--goal', goal), ('--offset', offset), ('--origin', origin)):
        if value is not None and _three(value) is None:
            raise ValueError('%s needs 3 finite numbers' % name)
    if auto is not None:
        if dry_run:
            raise ValueError('--auto cannot be used with --dry-run')
        if not (isinstance(auto, int) and not isinstance(auto, bool) and 1 <= auto <= max_steps):
            raise ValueError('--auto must be an integer from 1 to max-steps (%s)' % max_steps)
    if not (isinstance(origin_radius, (int, float)) and math.isfinite(origin_radius)
            and 0.0 < origin_radius <= ORIGIN_RADIUS_MAX):
        raise ValueError('--origin-radius must be in (0, %g] m' % ORIGIN_RADIUS_MAX)
    if not (isinstance(gain, (int, float)) and math.isfinite(gain) and 0.0 < gain <= 1.0):
        raise ValueError('--gain must be in (0, 1]')
    if not (isinstance(moveit_tolerance, (int, float)) and math.isfinite(moveit_tolerance)
            and 0.0 < moveit_tolerance <= MOVEIT_TOLERANCE_MAX):
        raise ValueError('--moveit-tolerance must be in (0, %g] rad' % MOVEIT_TOLERANCE_MAX)
    if not (isinstance(move_seconds, (int, float)) and math.isfinite(move_seconds)
            and move_seconds > 0.0):
        raise ValueError('--move-seconds must be positive')
    # the hard ceilings of the differential mode (raises ValueError)
    return DiffModeConfig(max_dq=max_dq, max_steps=max_steps, max_abs_d=max_abs_d)


def _three(value):
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        return None
    return array


def _text(array):
    if array is None:
        return ''
    return ' '.join('%.5f' % float(v) for v in np.ravel(array))


def step_record(**fields):
    """Return one row of the CSV (every column of CSV_FIELDS; unknown names are an error)."""
    unknown = set(fields) - set(CSV_FIELDS)
    if unknown:
        raise ValueError('unknown columns: %s' % sorted(unknown))
    row = {}
    for name in CSV_FIELDS:
        value = fields.get(name, '')
        if isinstance(value, np.ndarray):
            value = _text(value)
        elif isinstance(value, (tuple, list)):
            value = ' '.join(str(v) for v in value)
        elif isinstance(value, float):
            value = '%.6g' % value
        row[name] = value
    return row


# ----------------------------------------------------------------------------- confirmation

class LineReader:
    """
    Cut the text typed by the user into lines; one confirmation uses exactly one line.

    Text that arrives in one chunk (two Enter keys at once, a pipe) stays in the buffer and
    answers the NEXT confirmation, never the current one as well.
    """

    def __init__(self):
        self._buffer = ''
        self.eof = False

    def feed(self, chunk):
        """Add a chunk of text; the empty text means the end of the input (EOF)."""
        if chunk == '':
            self.eof = True
        else:
            self._buffer += chunk

    def take(self):
        """Return 'yes' / 'no' for the next line, or None if there is no line yet."""
        if '\n' in self._buffer or (self.eof and self._buffer):
            line, _, self._buffer = self._buffer.partition('\n')
            answer = line.strip().lower()
            return 'yes' if answer in ('', 'y', 'yes') else 'no'
        if self.eof:
            return 'no'
        return None


def confirm_answer(reader, poll, now, timeout=CONFIRM_TIMEOUT):
    """
    Wait for one answer: 'yes' (an empty line, y, yes), 'no' (anything else, EOF) or 'timeout'.

    poll() returns the text that arrived (str), '' at the end of the input, or None if nothing
    has arrived yet; it may wait a little. Nothing is ever sent by this function.
    """
    end = now() + timeout
    while True:
        answer = reader.take()
        if answer is not None:
            return answer
        if now() >= end:
            return 'timeout'
        chunk = poll()
        if chunk is not None:
            reader.feed(chunk)


# ---------------------------------------------------------------------------------- the loop

@dataclasses.dataclass(frozen=True)
class Inputs:
    """The latest inputs and when they were received (None if never)."""

    q_desired: Optional[np.ndarray] = None
    desired_at: Optional[float] = None
    q_meas: Optional[np.ndarray] = None
    meas_at: Optional[float] = None


@dataclasses.dataclass(frozen=True)
class Prepared:
    """What a sender made ready to be executed, or why it cannot be (nothing has moved yet)."""

    ok: bool
    reasons: Tuple[str, ...] = ()
    check: Optional[PlanCheck] = None
    cmd_speed: Optional[float] = None
    duration: float = 0.0
    payload: object = None
    note: str = ''


@dataclasses.dataclass(frozen=True)
class ExecResult:
    """Result of executing a prepared command: ok means it was accepted and ended normally."""

    ok: bool
    note: str = ''


@dataclasses.dataclass(frozen=True)
class RunResult:
    """Result of a run: the exit code, the reason it ended, the final outcome and counts."""

    exit_code: int
    reason: str
    outcome: object = None
    sent: int = 0
    detail: str = ''


class _Stop(Exception):
    def __init__(self, reason, detail='', exit_code=EXIT_STOPPED):
        super().__init__(reason)
        self.reason = reason
        self.detail = detail
        self.exit_code = exit_code


class StepLoop:
    """
    Run "move, stop, measure" with DiffMode, one step at a time (docs 7.14.2).

    io: now(), wait(seconds), read_inputs() -> Inputs, other_active_goals() -> list of text,
    confirm(text, timeout) -> 'yes' / 'no' / 'timeout' (KeyboardInterrupt for Ctrl-C), say(text),
    log_row(dict). sender: prepare(q_cmd, q_desired, q_meas) -> Prepared and
    execute(prepared) -> ExecResult. Nothing is ever cancelled.
    """

    def __init__(self, io, sender, config, route, goal=None, offset=None, gain=1.0, auto=None,
                 origin=None, origin_radius=ORIGIN_RADIUS, moveit_tolerance=MOVEIT_TOLERANCE,
                 dry_run=False, assume_desired_is_measured=False, run_timeout=RUN_TIMEOUT):
        validate_run_options(route, dry_run, assume_desired_is_measured, goal, offset, auto,
                             origin, origin_radius, config.max_dq, config.max_steps,
                             config.max_abs_d, gain, moveit_tolerance)
        self.io = io
        self.sender = sender
        self.config = config
        self.route = route
        self.goal = None if goal is None else _three(goal)
        self.offset = None if offset is None else _three(offset)
        self.gain = gain
        self.auto_left = auto
        self.origin = None if origin is None else _three(origin)
        self.origin_radius = origin_radius
        self.moveit_tolerance = moveit_tolerance
        self.dry_run = dry_run
        self.assume = assume_desired_is_measured
        self.run_timeout = run_timeout
        self.mode = None
        self.sent = 0
        self.last_meas = None
        self.in_flight = False

    # ------------------------------------------------------------------------------ helpers

    def _say(self, text):
        self.io.say(text)

    def _read(self, wait=INPUT_WAIT):
        """Return fresh (q_desired, q_meas); wait up to `wait` s for them (_Stop if not)."""
        end = self.io.now() + wait
        while True:
            now = self.io.now()
            inputs = self.io.read_inputs()
            meas_ok = inputs.q_meas is not None and is_fresh(inputs.meas_at, now)
            desired_ok = inputs.q_desired is not None and is_fresh(inputs.desired_at, now)
            if self.assume and self.dry_run and meas_ok:
                return np.array(inputs.q_meas, float), np.array(inputs.q_meas, float)
            if meas_ok and desired_ok:
                return np.array(inputs.q_desired, float), np.array(inputs.q_meas, float)
            if now >= end:
                missing = []
                if not desired_ok:
                    missing.append('q_desired (controller_state)')
                if not meas_ok:
                    missing.append('q_meas (joint_states)')
                raise _Stop('STALE_INPUT', 'no fresh input: ' + ', '.join(missing))
            self.io.wait(0.1)

    def _settle(self):
        """Wait until the arm has stopped (command A) and return the measured angles."""
        self.io.wait(SETTLE_MIN)
        started = self.io.now()
        while True:
            first = self._read()[1]
            self.io.wait(STOP_INTERVAL)
            second = self._read()[1]
            if is_stopped(first, second):
                return second, self.io.now() - started + SETTLE_MIN
            if self.io.now() - started > SETTLE_TIMEOUT:
                raise _Stop('NOT_SETTLED', 'the arm did not stop within %.1f s' % SETTLE_TIMEOUT)

    def _confirm(self, text):
        answer = self.io.confirm(text, CONFIRM_TIMEOUT)
        if answer != 'yes':
            raise _Stop('NOT_CONFIRMED', 'answer: %s (nothing was sent)' % answer)

    def _check_run_time(self, t0):
        if self.io.now() - t0 > self.run_timeout:
            raise _Stop('RUN_TIMEOUT', 'the run is longer than %.0f s' % self.run_timeout)

    # -------------------------------------------------------------------------------- run

    def run(self):
        """Run until the goal is reached, a limit ends the mode, or something stops it."""
        t0 = self.io.now()
        try:
            return self._run(t0)
        except _Stop as stop:
            self._say('STOP %s: %s' % (stop.reason, stop.detail))
            return RunResult(stop.exit_code, stop.reason, None, self.sent, stop.detail)
        except KeyboardInterrupt:
            detail = ('a goal was in flight; it is not cancelled and goes on to its command'
                      if self.in_flight else 'no goal was in flight')
            self._say('INTERRUPTED: nothing is sent or cancelled any more (%s)' % detail)
            return RunResult(EXIT_INTERRUPTED, 'INTERRUPTED', None, self.sent, detail)

    def _run(self, t0):
        q_desired, q_meas = self._read()
        if self.offset is not None:
            self.goal = ik.fk(q_meas)[0] + self.offset
        problem = check_origin(self.origin, q_desired, self.origin_radius)
        if problem is not None:
            raise _Stop('ORIGIN', problem, EXIT_START_REFUSED)
        try:
            self.mode = DiffMode.start(q_desired, q_meas, self.config)
        except StartRefused as refused:
            raise _Stop('START_REFUSED', ', '.join(refused.reasons) + ': ' + ' / '.join(
                refused.details), EXIT_START_REFUSED)
        state = self.mode.state
        self._say('START q_desired=%s tip=%s goal=%s branch=%s' % (
            _text(q_desired), _text(ik.fk(q_desired)[0]), _text(self.goal), state.branch))
        if self.dry_run:
            return self._dry_run(q_meas)
        self._confirm('Start from this state? [Enter = yes, n = no]')
        while True:
            self._check_run_time(t0)
            q_meas = self._one_step(q_meas)
            if q_meas is None:
                return self._finish(self.last_meas)

    def _finish(self, q_meas):
        outcome = self.mode.outcome(self.goal, q_meas)
        self._say('OUTCOME %s residual=%.2f mm limit_gap=%s track_gap=%s reason=%s steps=%d '
                  'holds=%d' % (outcome.kind.value, outcome.residual_norm * 1000.0,
                                _text(outcome.limit_gap * 1000.0),
                                _text(outcome.track_gap * 1000.0), outcome.limit_reason,
                                outcome.steps, outcome.holds))
        return RunResult(EXIT_OK, outcome.kind.value, outcome, self.sent)

    def _dry_run(self, q_meas):
        """Show the first suggestion and step. Nothing is sent and no client exists."""
        sug = self.mode.suggest_delta(self.goal, q_meas, self.gain)
        self._say('DRY-RUN suggestion: %s %s' % (sug.status.value, sug.reason or ''))
        if sug.status in (Status.OK, Status.AT_LIMIT):
            res = self.mode.step(sug.delta, q_meas, sug.requested_norm)
            self._say('DRY-RUN step: %s q_cmd=%s dq_max=%.4f' % (
                res.status.value, _text(res.q_cmd),
                float(np.max(np.abs(res.q_cmd - self.mode.state.q_start)))))
        elif sug.status is Status.REFUSED:
            raise _Stop('REFUSED', str(sug.reason))
        return RunResult(EXIT_OK, 'DRY_RUN', None, 0)

    def _one_step(self, q_meas):
        """Do one step. Return the new measured angles, or None when the run has ended."""
        mode = self.mode
        self.last_meas = q_meas
        sug = mode.suggest_delta(self.goal, q_meas, self.gain)
        if sug.status in (Status.IN_BAND, Status.LIMITED_DONE):
            return None
        if sug.status is Status.REFUSED:
            raise _Stop('REFUSED', str(sug.reason))
        if sug.status is Status.HOLD:
            self._say('HOLD %s limits=%s: nothing to send' % (sug.reason, ','.join(sug.limits)))
            self._confirm('HOLD: nothing is sent. Try again? [Enter = yes, n = end]')
            return q_meas
        q_prev = mode.state.q_cmd.copy()
        res = mode.step(sug.delta, q_meas, sug.requested_norm)
        if res.status is Status.LIMITED_DONE:
            return None
        if res.status is Status.REFUSED:
            raise _Stop('REFUSED', '%s %s' % (res.reason, res.detail))
        if res.status is Status.HOLD:
            self._say('HOLD %s: nothing to send' % res.reason)
            self._confirm('HOLD: nothing is sent. Try again? [Enter = yes, n = end]')
            return q_meas
        state = mode.state
        # Confirmation: always for the first step and for AT_LIMIT; otherwise unless --auto.
        if self.sent == 0 or self.auto_left is None or self.auto_left <= 0 \
                or res.status is Status.AT_LIMIT:
            self._say(self._table(q_meas, q_prev, res, state))
            self._confirm('Send this command? [Enter = yes, n = end]')
        else:
            self.auto_left -= 1
            self._say('AUTO ' + self._table(q_meas, q_prev, res, state))
        q_desired_now, q_meas_now = self._recheck(q_meas, q_prev, res, state)
        prepared = self.sender.prepare(res.q_cmd, q_desired_now, q_meas_now)
        row = dict(step=self.sent + 1, status=res.status.value, limits=res.limits,
                   route=self.route, q_meas_before=q_meas_now, q_cmd=res.q_cmd,
                   q_desired_before=q_desired_now, tip_before=ik.fk(q_meas_now)[0],
                   delta=res.delta if res.delta is not None else sug.delta,
                   abs_d=dm.abs_d_of_q(res.q_cmd), cmd_speed=prepared.cmd_speed,
                   plan_ok=prepared.ok, plan_reasons=prepared.reasons, note=prepared.note)
        if prepared.check is not None:
            c = prepared.check
            row.update(plan_peak_pos_speed=c.peak_pos_speed, plan_peak_vel_speed=c.peak_vel_speed,
                       plan_duration=c.duration, plan_points=c.points,
                       first_vs_desired=c.first_vs_desired, first_vs_meas=c.first_vs_meas,
                       last_vs_cmd=c.last_vs_cmd)
        if not prepared.ok:
            self.io.log_row(step_record(**row))
            raise _Stop('NOT_EXECUTED', 'nothing was executed (%s); the mode has advanced, '
                        'start again' % ','.join(prepared.reasons))
        self.in_flight = True  # a KeyboardInterrupt in execute() leaves a goal that goes on
        done = self.sender.execute(prepared)
        self.in_flight = False
        if not done.ok:
            row['note'] = done.note
            self.io.log_row(step_record(**row))
            raise _Stop('SEND_FAILED', done.note)
        self.sent += 1
        after, waited = self._settle()
        outcome = mode.outcome(self.goal, after)
        tip_before = ik.fk(q_meas_now)[0]
        tip_after = ik.fk(after)[0]
        delta_size = float(np.linalg.norm(row['delta']))
        ratio = float(np.linalg.norm(tip_after - tip_before)) / delta_size if delta_size else 0.0
        windup_after = float(np.max(np.abs(res.q_cmd - after)))
        row.update(q_meas_after=after, tip_after=tip_after, tip_move_over_delta=ratio,
                   windup_after=windup_after,
                   residual_norm=outcome.residual_norm, limit_gap=outcome.limit_gap,
                   track_gap=outcome.track_gap, wait_settle=waited, note=done.note)
        row['q_desired_after'] = self._read()[0]
        self.io.log_row(step_record(**row))
        self.last_meas = after
        return after

    def _recheck(self, q_meas_used, q_prev, res, state):
        """Re-read and re-check everything just before sending (every step, also in --auto)."""
        q_desired_now, q_meas_now = self._read()
        if float(np.max(np.abs(q_meas_now - q_meas_used))) > STOP_TOL:
            raise _Stop('ARM_MOVED', 'the measured angles changed while waiting (nothing sent)')
        if not desired_matches(q_desired_now, q_prev, self.route, self.moveit_tolerance):
            raise _Stop('DESIRED_MISMATCH', 'desired differs from the previous command by '
                        '%.5f rad (tolerance %.5f); somebody else moved the arm?' % (
                            float(np.max(np.abs(q_desired_now - q_prev))),
                            desired_tolerance(self.route, self.moveit_tolerance)))
        active = self.io.other_active_goals()
        if active:
            raise _Stop('OTHER_GOAL', 'an action has an active goal: ' + '; '.join(active))
        bad = verify_command(res.q_cmd, q_prev, q_meas_now, state.branch, self.config,
                             state.target)
        if bad is not None:
            raise _Stop('VERIFY', '%s: %s' % bad)
        return q_desired_now, q_meas_now

    @staticmethod
    def _table(q_meas, q_prev, res, state):
        dq = res.q_cmd - q_prev
        return ('STEP %s limits=%s\n  q_meas=%s\n  q_cmd =%s\n  dq=%s (max %.4f rad)\n'
                '  target=%s FK(q_cmd).z=%.4f abs(D)=%.3f\n  q_cmd-q_meas=%s' % (
                    res.status.value, ','.join(res.limits), _text(q_meas), _text(res.q_cmd),
                    _text(dq), float(np.max(np.abs(dq))), _text(state.target),
                    ik.fk(res.q_cmd)[0][2], dm.abs_d_of_q(res.q_cmd), _text(res.q_cmd - q_meas)))
