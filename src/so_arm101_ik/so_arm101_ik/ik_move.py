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
Large move (docs/large_move_plan.md): the analytic IK gives a joint goal, MoveIt only plans it.

This module has no ROS, no clock of its own and no I/O: the caller supplies an `io` object (the
inputs, the time, the confirmation, the status of the actions) and a `sender` (MoveIt: plan only,
then execute the checked trajectory). MoveLoop only decides; it never sends anything itself.
Units: m, rad, s (the command line uses degrees and the module converts at the entrance).
Joint order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll.

Rules that this module keeps (docs/large_move_plan.md):
- The goal is checked from q only: the strict joint range and a margin from its ends, the floor
  (0.048 m), the radial distance, abs(D), the elbow branch of the measured pose, and the size of
  the move (the tip, every joint). The measured tip must be above the measured floor (0.045 m).
- The plan is made with plan_only (MoveIt moves nothing). Its trajectory is checked (names, finite,
  first point against the measurement and the desired of the controller, last point against the
  goal, speed, duration, every point against the joint range and the floor) and the trajectory
  that was checked is what gets executed.
- There are two confirmations (start, send). Right before the send everything is read and checked
  again. A goal is never cancelled. SUCCEEDED is not a proof of arrival (open_loop_control: true):
  the result is judged from the measurement after the arm has stopped.
"""

import dataclasses
import math
from typing import Optional, Tuple

import numpy as np

from . import analytic_ik as ik
from . import diff_mode as dm
from . import step_runner as sr

JOINT_NAMES = dm.JOINT_NAMES
_CONFIG = dm.DiffModeConfig()

# Size of a move (docs/large_move_plan.md 3.4; guesses: they stop a mistyped target)
MAX_TIP_MOVE = 0.15  # m, abs(FK(q_goal) - FK(q_meas))
MAX_JOINT_MOVE = 1.0  # rad, abs(q_goal - q_meas) of every joint
MAX_ANGLE_DEG = 180.0  # the limit of an angle that is typed in degrees

# Floors (the values of the differential mode)
GOAL_FLOOR = _CONFIG.floor_z  # m, the goal (diff_mode.py floor_z)
PATH_FLOOR = _CONFIG.measured_floor_z  # m, the start and every point of the plan

# The plan (docs/large_move_plan.md 3.5, 4, 8)
FIRST_DESIRED_TOL = _CONFIG.max_windup  # rad: the bound of the wind-up of the differential mode
MAX_PLAN_DURATION = 20.0  # s (guess)
PATH_STEP = 0.01  # rad (guess): the spacing of the points where a plan is checked
LIMIT_ROUNDING = dm.LIMIT_ROUNDING  # rad

# Right before the send (docs/large_move_plan.md 5)
RECHECK_MEAS_TOL = sr.STOP_TOL  # rad
RECHECK_DESIRED_TOL = sr.DESIRED_TOL_DIRECT  # rad

# Arrival (docs/large_move_plan.md 6)
REACHED_MM = 10.0  # mm, the policy of the user (about 1 cm)
BAND_MM = _CONFIG.deadband_enter * 1000.0  # mm, the dead band of the differential mode
EXIT_SHORT = 4

MOVE_CSV_FIELDS = (
    'input', 'moveit_tolerance', 'q_meas_before', 'q_desired_before', 'windup_before',
    'tip_before', 'target', 'q_goal', 'branch', 'abs_d', 'tip_move', 'joint_move_max',
    'plan_ok', 'plan_reasons', 'plan_duration', 'plan_points', 'plan_peak_pos_speed',
    'plan_peak_vel_speed', 'first_vs_desired', 'first_vs_meas', 'last_vs_cmd', 'path_min_z',
    'q_meas_after', 'q_desired_after', 'joint_error', 'tip_error_mm', 'tip_error_vec_mm',
    'ref_error_mm', 'stuck', 'label', 'wait_settle', 'note')


# ------------------------------------------------------------------------------- options

def _vector(value, length):
    """Return value as a finite float vector of the given length, or None."""
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if array.shape != (length,) or not np.all(np.isfinite(array)):
        return None
    return array


def _number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def validate_move_options(goal=None, offset=None, joints_deg=None, pitch_deg=None, roll_deg=None,
                          moveit_tolerance=sr.MOVEIT_TOLERANCE):
    """Check the combination of the options (ValueError if wrong), before any node is made."""
    given = [value is not None for value in (goal, offset, joints_deg)]
    if sum(given) != 1:
        raise ValueError('give exactly one of --goal, --offset and --joints-deg')
    for name, value, count in (('--goal', goal, 3), ('--offset', offset, 3),
                               ('--joints-deg', joints_deg, 5)):
        if value is not None and _vector(value, count) is None:
            raise ValueError('%s needs %d finite numbers' % (name, count))
    if joints_deg is not None and (pitch_deg is not None or roll_deg is not None):
        raise ValueError('--pitch-deg and --roll-deg cannot be used with --joints-deg')
    angles = [('--pitch-deg', pitch_deg), ('--roll-deg', roll_deg)]
    if joints_deg is not None:
        angles += [('--joints-deg', v) for v in joints_deg]
    for name, value in angles:
        if value is None:
            continue
        if not _number(value):
            raise ValueError('%s needs finite numbers' % name)
        if abs(value) > MAX_ANGLE_DEG:
            raise ValueError('%s %g is outside +-%g degrees (degrees, not radians)' %
                             (name, value, MAX_ANGLE_DEG))
    if not (_number(moveit_tolerance) and 0.0 < moveit_tolerance <= sr.MOVEIT_TOLERANCE_MAX):
        raise ValueError('--moveit-tolerance must be in (0, %g] rad' % sr.MOVEIT_TOLERANCE_MAX)


# --------------------------------------------------------------------------------- goal

@dataclasses.dataclass(frozen=True, eq=False)
class GoalPlan:
    """Result of solve_goal(): the goal (None if there is none) and every reason to refuse."""

    ok: bool
    reasons: Tuple[str, ...]
    details: Tuple[str, ...]
    q_goal: Optional[np.ndarray]
    target: Optional[np.ndarray]
    branch: Optional[int]
    abs_d: Optional[float]
    tip_move: Optional[float]
    joint_move: Optional[np.ndarray]
    candidates: int


def goal_checks(q_goal, q_meas, config=_CONFIG):
    """
    Return [(reason, detail), ...] for a joint goal, from q only (independent of the solver).

    The checks: the strict joint range (JOINT_LIMIT), a margin from its ends (JOINT_RANGE), the
    floor (GOAL_FLOOR), abs(D) (SINGULAR), the radial distance (RADIAL), everything that
    DiffMode.start() needs of the goal (NO_BRANCH), the elbow branch of the measured pose
    (BRANCH_SWITCH) and the size of the move (MOVE_TIP_TOO_FAR, MOVE_JOINT_TOO_FAR).
    """
    found = []
    goal = _vector(q_goal, 5)
    meas = _vector(q_meas, 5)
    if goal is None or meas is None:
        return [('NOT_FINITE', 'a joint vector is not 5 finite numbers')]
    if not ik.within_limits(goal):
        found.append(('JOINT_LIMIT', 'the goal is outside the strict joint range'))
    for i, name in enumerate(JOINT_NAMES):
        lower = ik.Q_MIN[i] + config.margin_joint
        upper = ik.Q_MAX[i] - config.margin_joint
        if goal[i] < lower or goal[i] > upper:
            found.append(('JOINT_RANGE', '%s %.4f rad is within %.3f rad of its limit' % (
                name, goal[i], config.margin_joint)))
    xyz, pitch, roll = ik.fk(goal)
    if xyz[2] < config.floor_z - dm.FLOOR_ROUNDING:
        found.append(('GOAL_FLOOR', 'the goal tip is at z = %.4f m (floor %.3f m)' % (
            xyz[2], config.floor_z)))
    abs_d = dm.abs_d_of_q(goal)
    if abs_d > config.max_abs_d - config.margin_abs_d:
        found.append(('SINGULAR', 'abs(D) = %.3f is above %.3f' % (
            abs_d, config.max_abs_d - config.margin_abs_d)))
    try:
        radial = ik.ik_geometry(xyz[0], xyz[1], xyz[2], pitch, roll).radial
        if radial < config.min_radial + config.margin_radial:
            found.append(('RADIAL', 'the tip is %.3f m from the pan axis (minimum %.3f m)' % (
                radial, config.min_radial + config.margin_radial)))
    except ik.IKUnreachableError:
        found.append(('UNREACHABLE', 'the pose of the goal is not reproduced by the analytic IK'))
    try:
        dm.DiffMode.start(goal, goal, config)
    except dm.StartRefused as refused:
        for code in refused.reasons:
            if code == 'NO_BRANCH':
                found.append((code, 'the goal is not on a unique branch of the analytic IK'))
    if dm.branch_of_q(goal) != dm.branch_of_q(meas):
        found.append(('BRANCH_SWITCH', 'the goal is on the other elbow branch than the arm'))
    tip_move = float(np.linalg.norm(xyz - ik.fk(meas)[0]))
    if tip_move > MAX_TIP_MOVE:
        found.append(('MOVE_TIP_TOO_FAR', 'the tip moves %.3f m (the limit is %.3f m)' % (
            tip_move, MAX_TIP_MOVE)))
    for i, name in enumerate(JOINT_NAMES):
        move = abs(goal[i] - meas[i])
        if move > MAX_JOINT_MOVE:
            found.append(('MOVE_JOINT_TOO_FAR', '%s moves %.3f rad (the limit is %.1f rad)' % (
                name, move, MAX_JOINT_MOVE)))
    return found


def start_checks(q_meas, config=_CONFIG):
    """Return [(reason, detail), ...] for the pose the arm is in (the measured floor)."""
    meas = _vector(q_meas, 5)
    if meas is None:
        return [('NOT_FINITE', 'q_meas is not 5 finite numbers')]
    z = ik.fk(meas)[0][2]
    if z < config.measured_floor_z - dm.FLOOR_ROUNDING:
        return [('START_FLOOR_MEASURED', 'the measured tip is at z = %.4f m (minimum %.3f m)' % (
            z, config.measured_floor_z))]
    return []


def solve_goal(q_meas, goal=None, offset=None, joints_deg=None, pitch_deg=None, roll_deg=None,
               config=_CONFIG):
    """
    Return the GoalPlan for one input: a target (goal or offset) or a joint goal in degrees.

    The elbow branch that is nearest to q_meas is taken. Every reason to refuse is collected
    (never only the first). A refused GoalPlan has ok = False and q_goal may be None.
    """
    found = []
    meas = _vector(q_meas, 5)
    if meas is None:
        found.append(('NOT_FINITE', 'q_meas is not 5 finite numbers'))
        return _plan(found, None, None, meas, 0)
    found += start_checks(meas, config)
    q_goal = None
    target = None
    candidates = 0
    if joints_deg is not None:
        q_goal = np.radians(np.asarray(joints_deg, dtype=float))
        target = ik.fk(q_goal)[0]
    else:
        xyz0, pitch0, roll0 = ik.fk(meas)
        target = (np.asarray(goal, dtype=float) if goal is not None
                  else xyz0 + np.asarray(offset, dtype=float))
        pitch = pitch0 if pitch_deg is None else math.radians(pitch_deg)
        roll = roll0 if roll_deg is None else math.radians(roll_deg)
        if target[2] < config.floor_z - dm.FLOOR_ROUNDING:
            found.append(('GOAL_FLOOR', 'the target is at z = %.4f m (floor %.3f m)' % (
                target[2], config.floor_z)))
        try:
            geometry = ik.ik_geometry(target[0], target[1], target[2], pitch, roll)
            valid = [q for q in geometry.candidates if ik.within_limits(q)]
            candidates = len(valid)
            if valid:
                q_goal = ik.select_nearest_solution(valid, meas)
            else:
                found.append(('JOINT_LIMIT', 'every IK candidate is outside the joint range'))
        except ik.IKUnreachableError as error:
            found.append(('UNREACHABLE', str(error)))
        except ValueError as error:
            found.append(('NOT_FINITE', str(error)))
    if q_goal is not None:
        found += goal_checks(q_goal, meas, config)
    return _plan(found, q_goal, target, meas, candidates)


def _plan(found, q_goal, target, meas, candidates):
    reasons = tuple(dict.fromkeys(code for code, _ in found))
    details = tuple('%s: %s' % item for item in found)
    branch = abs_d = tip_move = joint_move = None
    if q_goal is not None:
        branch = dm.branch_of_q(q_goal)
        abs_d = dm.abs_d_of_q(q_goal)
        if meas is not None:
            tip_move = float(np.linalg.norm(ik.fk(q_goal)[0] - ik.fk(meas)[0]))
            joint_move = np.abs(q_goal - meas)
    return GoalPlan(not reasons, reasons, details, q_goal, target, branch, abs_d, tip_move,
                    joint_move, candidates)


# ---------------------------------------------------------------------------------- plan

@dataclasses.dataclass(frozen=True)
class PathCheck:
    """Result of check_path(): ok, the reasons, the lowest z of the tip and the points checked."""

    ok: bool
    reasons: Tuple[str, ...]
    min_z: float
    samples: int


def check_path(positions, floor=PATH_FLOOR, step=PATH_STEP):
    """
    Check every point of a trajectory, and the points between them, against range and floor.

    The joints are interpolated linearly between two points of the plan, with at most `step` rad
    between the points that are checked (the plan has a point every 0.1 s only). Each point must
    be inside the strict joint range (PATH_RANGE) and have z of the tip above the floor
    (PATH_FLOOR). Anything that is not 5 finite numbers is PATH_NOT_FINITE.
    """
    nan = float('nan')
    points = []
    try:
        for position in positions:
            vector = _vector(position, 5)
            if vector is None:
                return PathCheck(False, ('PATH_NOT_FINITE',), nan, 0)
            points.append(vector)
    except TypeError:
        return PathCheck(False, ('PATH_NOT_FINITE',), nan, 0)
    if not points:
        return PathCheck(False, ('PATH_NOT_FINITE',), nan, 0)
    reasons = []
    min_z = float('inf')
    samples = 0
    lower = ik.Q_MIN - LIMIT_ROUNDING
    upper = ik.Q_MAX + LIMIT_ROUNDING
    for index, point in enumerate(points):
        if index == 0:
            checked = [point]
        else:
            previous = points[index - 1]
            gap = float(np.max(np.abs(point - previous)))
            count = max(1, int(math.ceil(gap / step - 1e-9)))
            checked = [previous + (point - previous) * (k / count) for k in range(1, count + 1)]
        for q in checked:
            samples += 1
            if not ik.within_limits(q, lower, upper):
                reasons.append('PATH_RANGE')
            min_z = min(min_z, float(ik.fk(q)[0][2]))
    if min_z < floor - dm.FLOOR_ROUNDING:
        reasons.append('PATH_FLOOR')
    reasons = tuple(dict.fromkeys(reasons))
    return PathCheck(not reasons, reasons, min_z, samples)


@dataclasses.dataclass(frozen=True)
class MovePlanCheck:
    """Result of check_plan(): ok, the reasons, the numbers of the plan and of the path."""

    ok: bool
    reasons: Tuple[str, ...]
    plan: sr.PlanCheck
    path: PathCheck


def check_plan(points, joint_names, q_goal, q_desired, q_meas, tolerance=sr.MOVEIT_TOLERANCE,
               max_duration=MAX_PLAN_DURATION):
    """
    Check a trajectory that MoveIt planned, before it is executed (docs/large_move_plan.md 4).

    It is check_planned_trajectory() of the differential mode with the first point compared with
    the desired at FIRST_DESIRED_TOL (the arm stops short of the desired; 0.005 would refuse it
    every time), plus the duration and the path (range and floor of every point).
    """
    plan = sr.check_planned_trajectory(points, joint_names, q_goal, q_desired, q_meas, tolerance,
                                       first_desired_tol=FIRST_DESIRED_TOL)
    reasons = list(plan.reasons)
    path = check_path([p.positions for p in points] if points else [])
    if not plan.duration <= max_duration:
        reasons.append('DURATION')
    reasons += list(path.reasons)
    reasons = tuple(dict.fromkeys(reasons))
    return MovePlanCheck(not reasons, reasons, plan, path)


def recheck_before_send(q_meas_planned, q_desired_planned, q_meas_now, q_desired_now, active,
                        points, joint_names, q_goal, tolerance=sr.MOVEIT_TOLERANCE):
    """
    Check everything again right before the send; return None or (reason, detail).

    The measured angles and the desired of the controller must be what they were when the plan
    was made, no other action has an active goal, the goal passes goal_checks() against the new
    measurement and the trajectory passes check_plan() again.
    """
    planned_meas = _vector(q_meas_planned, 5)
    planned_desired = _vector(q_desired_planned, 5)
    meas = _vector(q_meas_now, 5)
    desired = _vector(q_desired_now, 5)
    if any(v is None for v in (planned_meas, planned_desired, meas, desired)):
        return 'NOT_FINITE', 'an input is not 5 finite numbers'
    moved = float(np.max(np.abs(meas - planned_meas)))
    if not moved <= RECHECK_MEAS_TOL:
        return 'ARM_MOVED', 'the measured angles changed by %.4f rad since the plan' % moved
    changed = float(np.max(np.abs(desired - planned_desired)))
    if not changed <= RECHECK_DESIRED_TOL:
        return 'DESIRED_CHANGED', 'desired changed by %.5f rad since the plan' % changed
    if active:
        return 'OTHER_GOAL', 'an action has an active goal: ' + '; '.join(active)
    bad = goal_checks(q_goal, meas) + start_checks(meas)
    if bad:
        return bad[0]
    check = check_plan(points, joint_names, q_goal, desired, meas, tolerance)
    if not check.ok:
        return 'PLAN_RECHECK', ','.join(check.reasons)
    return None


# -------------------------------------------------------------------------------- arrival

@dataclasses.dataclass(frozen=True, eq=False)
class Arrival:
    """Result of judge_arrival(): the errors after the arm has stopped and the label."""

    label: str
    exit_code: int
    joint_error: np.ndarray  # q_meas - q_goal [rad]
    tip_error: np.ndarray  # FK(q_meas) - target [m]
    error_mm: float
    ref_error_mm: float  # FK(q_desired) - target; nan if there is no desired
    stuck: Optional[np.ndarray]  # q_desired - q_meas [rad]
    in_band: bool


def judge_arrival(q_goal, target, q_meas_after, q_desired_after=None):
    """Judge an arrival from the measurement after the arm has stopped (never from SUCCEEDED)."""
    goal = _vector(q_goal, 5)
    meas = _vector(q_meas_after, 5)
    aim = _vector(target, 3)
    if goal is None or meas is None or aim is None:
        raise ValueError('q_goal, q_meas_after and target must be finite')
    tip_error = ik.fk(meas)[0] - aim
    error_mm = float(np.linalg.norm(tip_error)) * 1000.0
    desired = None if q_desired_after is None else _vector(q_desired_after, 5)
    ref_mm = float('nan') if desired is None else float(
        np.linalg.norm(ik.fk(desired)[0] - aim)) * 1000.0
    stuck = None if desired is None else desired - meas
    if error_mm <= REACHED_MM:
        label, code = 'REACHED', sr.EXIT_OK
    else:
        label, code = 'SHORT', EXIT_SHORT
    return Arrival(label, code, meas - goal, tip_error, error_mm, ref_mm, stuck,
                   error_mm <= BAND_MM)


# ------------------------------------------------------------------------------- display

def _text(array, scale=1.0):
    if array is None:
        return ''
    return ' '.join('%.5f' % (float(v) * scale) for v in np.ravel(array))


def goal_lines(plan, q_meas, q_desired):
    """Return the text that shows the goal before the first confirmation."""
    lines = []
    if plan.q_goal is not None:
        lines.append('GOAL branch=%s abs(D)=%.3f candidates=%d tip move=%.1f mm' % (
            plan.branch, plan.abs_d, plan.candidates, (plan.tip_move or 0.0) * 1000.0))
        lines.append('  q_meas =%s' % _text(q_meas))
        lines.append('  q_goal =%s' % _text(plan.q_goal))
        lines.append('  q_goal [deg] =%s' % _text(np.degrees(plan.q_goal)))
        lines.append('  dq     =%s (max %.4f rad)' % (
            _text(plan.q_goal - q_meas), float(np.max(np.abs(plan.q_goal - q_meas)))))
        lines.append('  target =%s FK(q_goal)=%s' % (
            _text(plan.target), _text(ik.fk(plan.q_goal)[0])))
    if q_desired is not None:
        lines.append('  windup before (desired - meas) max %.4f rad (the plan starts from the '
                     'measurement: this is dropped)' % float(np.max(np.abs(q_desired - q_meas))))
    return '\n'.join(lines)


def plan_lines(check):
    """Return the text that shows the plan before the second confirmation."""
    plan, path = check.plan, check.path
    return ('PLAN %s duration=%.2f s points=%d peak speed=%.3f rad/s (vel %.3f)\n'
            '  first vs desired=%.4f rad, first vs meas=%.4f rad, last vs goal=%.5f rad\n'
            '  lowest z of the path=%.4f m (%d points checked)' % (
                'OK' if check.ok else 'REFUSED ' + ','.join(check.reasons), plan.duration,
                plan.points, plan.peak_pos_speed, plan.peak_vel_speed, plan.first_vs_desired,
                plan.first_vs_meas, plan.last_vs_cmd, path.min_z, path.samples))


def arrival_lines(arrival):
    """Return the text of the result (joint errors, tip error in mm, the label)."""
    lines = ['ARRIVAL %s error=%.2f mm (reached <= %.0f mm)' % (
        arrival.label, arrival.error_mm, REACHED_MM),
        '  q_meas - q_goal [rad] =%s' % _text(arrival.joint_error),
        '  FK(q_meas) - target [mm] =%s' % _text(arrival.tip_error, 1000.0)]
    if not math.isnan(arrival.ref_error_mm):
        lines.append('  FK(desired) - target = %.2f mm' % arrival.ref_error_mm)
    if arrival.stuck is not None:
        lines.append('  desired - q_meas [rad] =%s' % _text(arrival.stuck))
    if arrival.label == 'SHORT':
        lines.append('  SHORT: more than %.0f mm from the target (not a failure of the tool)'
                     % REACHED_MM)
    elif not arrival.in_band:
        lines.append('  more than %.0f mm: the differential mode (camera stage) can correct it'
                     % BAND_MM)
    return '\n'.join(lines)


def move_record(**fields):
    """Return one row of the CSV (every column of MOVE_CSV_FIELDS; unknown names are an error)."""
    unknown = set(fields) - set(MOVE_CSV_FIELDS)
    if unknown:
        raise ValueError('unknown columns: %s' % sorted(unknown))
    row = {}
    for name in MOVE_CSV_FIELDS:
        value = fields.get(name, '')
        if isinstance(value, np.ndarray):
            value = _text(value)
        elif isinstance(value, (tuple, list)):
            value = ' '.join(str(v) for v in value)
        elif isinstance(value, float):
            value = '%.6g' % value
        row[name] = value
    return row


# ------------------------------------------------------------------------------- the loop

@dataclasses.dataclass(frozen=True)
class PlanResult:
    """What a sender returns for a plan: ok, a note, the points of the trajectory, the payload."""

    ok: bool
    note: str = ''
    points: Tuple[sr.TrajPoint, ...] = ()
    joint_names: Tuple[str, ...] = ()
    payload: object = None


class _Stop(Exception):
    def __init__(self, reason, detail='', exit_code=sr.EXIT_STOPPED):
        super().__init__(reason)
        self.reason = reason
        self.detail = detail
        self.exit_code = exit_code


class MoveLoop:
    """
    Plan, confirm, check again, execute, settle and judge: one large move.

    io: now(), wait(seconds), read_inputs() -> sr.Inputs, other_active_goals() -> list of text,
    confirm(text, timeout) -> 'yes' / 'no' / 'timeout' (KeyboardInterrupt for Ctrl-C), say(text),
    log_row(dict). sender: plan(q_goal, tolerance) -> PlanResult and execute(plan_result, seconds)
    -> sr.ExecResult. Nothing is ever cancelled.
    """

    def __init__(self, io, sender, goal=None, offset=None, joints_deg=None, pitch_deg=None,
                 roll_deg=None, moveit_tolerance=sr.MOVEIT_TOLERANCE, dry_run=False,
                 run_timeout=sr.RUN_TIMEOUT):
        validate_move_options(goal, offset, joints_deg, pitch_deg, roll_deg, moveit_tolerance)
        self.io = io
        self.sender = sender
        self.goal = goal
        self.offset = offset
        self.joints_deg = joints_deg
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.moveit_tolerance = moveit_tolerance
        self.dry_run = dry_run
        self.run_timeout = run_timeout
        self.sent = 0
        self.in_flight = False
        self.row = {}

    # ------------------------------------------------------------------------------ helpers

    def _say(self, text):
        self.io.say(text)

    def _read(self, need_desired=True, wait=sr.INPUT_WAIT):
        """Return fresh (q_desired or None, q_meas); wait up to `wait` s (_Stop if not)."""
        end = self.io.now() + wait
        while True:
            now = self.io.now()
            inputs = self.io.read_inputs()
            meas_ok = inputs.q_meas is not None and sr.is_fresh(inputs.meas_at, now)
            desired_ok = inputs.q_desired is not None and sr.is_fresh(inputs.desired_at, now)
            if meas_ok and (desired_ok or not need_desired):
                desired = np.array(inputs.q_desired, float) if desired_ok else None
                return desired, np.array(inputs.q_meas, float)
            if now >= end:
                missing = []
                if need_desired and not desired_ok:
                    missing.append('q_desired (controller_state)')
                if not meas_ok:
                    missing.append('q_meas (joint_states)')
                raise _Stop('STALE_INPUT', 'no fresh input: ' + ', '.join(missing))
            self.io.wait(0.1)

    def _settle(self):
        """Wait until the arm has stopped (command A) and return (angles, seconds)."""
        self.io.wait(sr.SETTLE_MIN)
        started = self.io.now()
        while True:
            first = self._read(need_desired=False)[1]
            self.io.wait(sr.STOP_INTERVAL)
            second = self._read(need_desired=False)[1]
            if sr.is_stopped(first, second):
                return second, self.io.now() - started + sr.SETTLE_MIN
            if self.io.now() - started > sr.SETTLE_TIMEOUT:
                raise _Stop('NOT_SETTLED',
                            'the arm did not stop within %.1f s' % sr.SETTLE_TIMEOUT)

    def _confirm(self, text):
        answer = self.io.confirm(text, sr.CONFIRM_TIMEOUT)
        if answer != 'yes':
            raise _Stop('NOT_CONFIRMED', 'answer: %s (nothing was sent)' % answer)

    def _check_run_time(self, t0):
        if self.io.now() - t0 > self.run_timeout:
            raise _Stop('RUN_TIMEOUT', 'the run is longer than %.0f s' % self.run_timeout)

    def _log(self):
        self.io.log_row(move_record(**self.row))

    # -------------------------------------------------------------------------------- run

    def run(self):
        """Run the move, or stop; return a step_runner.RunResult (outcome is an Arrival)."""
        t0 = self.io.now()
        try:
            return self._run(t0)
        except _Stop as stop:
            self._say('STOP %s: %s' % (stop.reason, stop.detail))
            self.row.setdefault('note', '%s: %s' % (stop.reason, stop.detail))
            if self.row and not self.dry_run:
                self._log()
            return sr.RunResult(stop.exit_code, stop.reason, None, self.sent, stop.detail)
        except KeyboardInterrupt:
            detail = ('a goal was in flight; it is not cancelled and goes on to its end'
                      if self.in_flight else 'no goal was in flight')
            self._say('INTERRUPTED: nothing is sent or cancelled any more (%s)' % detail)
            return sr.RunResult(sr.EXIT_INTERRUPTED, 'INTERRUPTED', None, self.sent, detail)

    def _input_name(self):
        if self.joints_deg is not None:
            return 'joints-deg'
        return 'goal' if self.goal is not None else 'offset'

    def _run(self, t0):
        q_desired, q_meas = self._read(need_desired=not self.dry_run)
        plan = solve_goal(q_meas, self.goal, self.offset, self.joints_deg, self.pitch_deg,
                          self.roll_deg)
        self._say(goal_lines(plan, q_meas, q_desired) or 'GOAL (none)')
        windup = (float(np.max(np.abs(q_desired - q_meas))) if q_desired is not None
                  else float('nan'))
        self.row = dict(input=self._input_name(), moveit_tolerance=self.moveit_tolerance,
                        q_meas_before=q_meas, q_desired_before=q_desired, windup_before=windup,
                        tip_before=ik.fk(q_meas)[0], target=plan.target, q_goal=plan.q_goal,
                        branch=plan.branch, abs_d=plan.abs_d, tip_move=plan.tip_move,
                        joint_move_max=(None if plan.joint_move is None
                                        else float(np.max(plan.joint_move))))
        if not plan.ok:
            raise _Stop('START_REFUSED', ' / '.join(plan.details), sr.EXIT_START_REFUSED)
        if self.dry_run:
            self._say('DRY-RUN: the goal passes every check. Nothing was planned or sent.')
            return sr.RunResult(sr.EXIT_OK, 'DRY_RUN', None, 0)
        self._confirm('Start from this state? Plan the move (nothing moves yet). '
                      '[Enter = yes, n = no]')
        planned = self.sender.plan(plan.q_goal, self.moveit_tolerance)
        if not planned.ok:
            self.row.update(plan_ok=False, note=planned.note)
            raise _Stop('PLAN_FAILED', planned.note)
        q_desired_p, q_meas_p = self._read()
        if not float(np.max(np.abs(q_meas_p - q_meas))) <= RECHECK_MEAS_TOL:
            raise _Stop('ARM_MOVED', 'the measured angles changed while planning')
        check = check_plan(planned.points, planned.joint_names, plan.q_goal, q_desired_p, q_meas_p,
                           self.moveit_tolerance)
        self._say(plan_lines(check))
        c = check.plan
        self.row.update(plan_ok=check.ok, plan_reasons=check.reasons, plan_duration=c.duration,
                        plan_points=c.points, plan_peak_pos_speed=c.peak_pos_speed,
                        plan_peak_vel_speed=c.peak_vel_speed, first_vs_desired=c.first_vs_desired,
                        first_vs_meas=c.first_vs_meas, last_vs_cmd=c.last_vs_cmd,
                        path_min_z=check.path.min_z)
        if not check.ok:
            raise _Stop('PLAN_REFUSED', 'nothing was executed (%s)' % ','.join(check.reasons))
        self._confirm('Send this trajectory? [Enter = yes, n = end]')
        self._check_run_time(t0)
        q_desired_n, q_meas_n = self._read()
        bad = recheck_before_send(q_meas_p, q_desired_p, q_meas_n, q_desired_n,
                                  self.io.other_active_goals(), planned.points,
                                  planned.joint_names, plan.q_goal, self.moveit_tolerance)
        if bad is not None:
            raise _Stop(bad[0], bad[1])
        self.in_flight = True  # a KeyboardInterrupt in execute() leaves a goal that goes on
        done = self.sender.execute(planned, c.duration)
        self.in_flight = False
        if not done.ok:
            raise _Stop('SEND_FAILED', done.note)
        self.sent += 1
        after, waited = self._settle()
        desired_after = self._read(need_desired=False)[0]
        arrival = judge_arrival(plan.q_goal, plan.target, after, desired_after)
        self._say(arrival_lines(arrival))
        self.row.update(q_meas_after=after, q_desired_after=desired_after,
                        joint_error=arrival.joint_error, tip_error_mm=arrival.error_mm,
                        tip_error_vec_mm=arrival.tip_error * 1000.0,
                        ref_error_mm=arrival.ref_error_mm, stuck=arrival.stuck,
                        label=arrival.label, wait_settle=waited, note=done.note)
        self._log()
        return sr.RunResult(arrival.exit_code, arrival.label, arrival, self.sent)
