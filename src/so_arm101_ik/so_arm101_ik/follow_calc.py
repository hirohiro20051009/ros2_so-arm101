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
Pure computation of the VR follow node (P6; docs/p6_plan.md).

No ROS, no clock, no I/O. Units: m, rad, s (the configuration of angles that people type is in
degrees: *_deg). Joint order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll.

Spaces. Every judgement about the target (the jump, the resume, the lag) is made in the space
of the TARGET OF THE ARM in base_link, after the scale and the projection (not in the space of
the VR hand). The target is

    p_target = p_ref + scale * R_fix (p_in - p_in0)
    R_target = (R_fix R_in R_in0^T R_fix^T) R_ref        (the rotation is relative, scale 1)

where (p_ref, R_ref) is the tip of the arm and (p_in0, R_in0) the hand at the moment of the clutch.
R_fix (input_rotation_rpy_deg) is the rotation that fits the axes of the VR to base_link.

Orientation. A 5 joint arm can make only the orientations Rz(theta) * (pitch, roll): the yaw is
decided by the pan (the position). project_orientation() takes the part that the arm can make;
the angle that is thrown away is reported (see Projection).
"""

import dataclasses
import math
from typing import Optional

import numpy as np

from . import analytic_ik as ik
from .urdf_kinematics import rpy_matrix

JOINT_NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')

# URDF: the tilt of the roll joint origin (rpy 1.5708 0.0486795 3.14159). It is a rotation about
# the roll axis itself, so it only shifts the zero of the roll.
ROLL_TILT = 0.0486795

# The rotation of the lift frame in base_link at pan 0 (R0 Ry(-pi/2) Rx(-pi/2); see tip_rotation).
_M1 = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])
_RY_PI = np.diag([-1.0, 1.0, -1.0])

GRIPPER_LOWER = 0.0  # rad. The driver clamps below the calibrated minimum (raw min = 0 %)
GRIPPER_UPPER = 1.74533  # rad (URDF upper limit)

SCALE_MAX = 3.0
SINGULAR_EPS = 1e-3  # the in-plane part of the tool axis below which the pitch is undefined
THETA_ITERATIONS = 4


class TargetError(Exception):
    """The target cannot be made. reason is one of the codes of the state topic."""

    def __init__(self, reason, detail=''):
        super().__init__('%s %s' % (reason, detail) if detail else reason)
        self.reason = reason
        self.detail = detail


# ------------------------------------------------------------------------------ rotations

def rot_x(a):
    """Return the rotation by a about x."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    """Return the rotation by a about y."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    """Return the rotation by a about z."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotation_angle(r):
    """Return the angle (0..pi) of the rotation matrix r."""
    return math.acos(max(-1.0, min(1.0, (float(np.trace(r)) - 1.0) / 2.0)))


def quaternion_to_matrix(x, y, z, w):
    """Return the rotation matrix of a quaternion (ValueError if it is not a rotation)."""
    values = (x, y, z, w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('the quaternion must be finite')
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-6 or abs(norm - 1.0) > 0.1:
        raise ValueError('the quaternion is not normalized (norm %g)' % norm)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quaternion(r):
    """Return (x, y, z, w) of the rotation matrix r (w >= 0)."""
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x, y, z = (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w, x = (r[2, 1] - r[1, 2]) / s, 0.25 * s
        y, z = (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w, y = (r[0, 2] - r[2, 0]) / s, 0.25 * s
        x, z = (r[0, 1] + r[1, 0]) / s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w, z = (r[1, 0] - r[0, 1]) / s, 0.25 * s
        x, y = (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    return x, y, z, w


def tip_rotation(q):
    """
    Return the rotation of gripper_frame_link in base_link for q (closed form of the URDF).

    From the joint origins of the URDF (every joint axis is z of its own frame):
    R = Rz(-q1) M1 Rz(S + pi) Rx(pi/2) Rz(q5 - ROLL_TILT) Ry(pi), with S = q2 + q3 + q4. The
    wrist_flex pitch of the model (analytic_ik.fk) is pitch = -S + d4.
    """
    q = np.asarray(q, dtype=float)
    s = float(q[1] + q[2] + q[3])
    return (rot_z(-float(q[0])) @ _M1 @ rot_z(s + math.pi) @ rot_x(math.pi / 2.0)
            @ rot_z(float(q[4]) - ROLL_TILT) @ _RY_PI)


def _wrap_near(value, reference):
    """Return value + 2 pi k that is nearest to reference."""
    return reference + ik.normalize_angle(value - reference)


def project_orientation(r, theta, flex_ref, roll_ref):
    """
    Return (flex_sum, q5) that make the orientation nearest to r for the pan angle theta.

    flex_sum = q2 + q3 + q4. The tool axis of r is projected on the pan plane (this fixes
    flex_sum), then the rotation about the tool axis is read from the first row (this fixes q5).
    The result is exact if r can be made. The angles are taken nearest to flex_ref and
    roll_ref (the previous solution). TargetError('ORIENT') if the tool axis is nearly
    perpendicular to the pan plane (the pitch is undefined).
    """
    # tip_rotation is Rz(theta) M1 Rz(S + pi) Rx(pi/2) Rz(q5 - ROLL_TILT) Ry(pi), theta = -q1
    a_matrix = _M1.T @ rot_z(-theta) @ r @ _RY_PI.T
    c0, c1 = a_matrix[0, 2], a_matrix[1, 2]
    h = math.hypot(c0, c1)
    if h < SINGULAR_EPS:
        raise TargetError('ORIENT', 'the tool axis is perpendicular to the pan plane')
    a = math.atan2(c0 / h, -c1 / h)
    b_matrix = rot_z(-a) @ a_matrix
    b = math.atan2(-b_matrix[0, 1], b_matrix[0, 0])
    flex_sum = _wrap_near(a - math.pi, flex_ref)
    q5 = _wrap_near(b + ROLL_TILT, roll_ref)
    return flex_sum, q5


def projection_angle(r, q):
    """Return the angle (rad) between r and the orientation that the arm makes at q."""
    return rotation_angle(tip_rotation(q).T @ r)


# ------------------------------------------------------------------------ the target

@dataclasses.dataclass(frozen=True)
class FollowConfig:
    """Values of the follower (every value is provisional; the checks of the constructor)."""

    scale: float = 1.0
    rate_hz: float = 20.0
    max_joint_speed: float = 0.5  # rad/s of the tick (the real limit; the driver cap is a net)
    jump_pos: float = 0.05  # m, in the space of the target
    jump_ang_deg: float = 30.0
    resume_pos: float = 0.02  # m
    resume_ang_deg: float = 10.0
    stale_s: float = 0.5  # the pose and the joint states
    joy_timeout_s: float = 1.0  # C4 (0 disables)
    floor_z: float = 0.045  # m
    elbow_max_deg: float = 91.0
    clutch_windup: float = 0.06  # rad, |q_desired - q_meas| allowed at the clutch
    ik_jump: float = 0.35  # rad, C1 (0 disables)
    max_lag: float = 0.15  # m, C2 (0 disables)
    max_tracking_error: float = 0.2  # rad, C3 (0 disables)
    clutch_mode: str = 'hold'  # 'hold' or 'toggle'
    gripper_requires_clutch: bool = False
    gripper_open_rad: float = 1.0
    gripper_closed_rad: float = 0.05
    input_rotation_rpy_deg: tuple = (0.0, 0.0, 0.0)

    def __post_init__(self):
        def need(condition, message):
            if not condition:
                raise ValueError(message)

        numbers = (self.scale, self.rate_hz, self.max_joint_speed, self.jump_pos,
                   self.jump_ang_deg, self.resume_pos, self.resume_ang_deg, self.stale_s,
                   self.joy_timeout_s, self.floor_z, self.elbow_max_deg, self.clutch_windup,
                   self.ik_jump, self.max_lag, self.max_tracking_error, self.gripper_open_rad,
                   self.gripper_closed_rad)
        need(all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                 for v in numbers), 'every value must be a finite number')
        need(0.0 < self.scale <= SCALE_MAX, 'scale must be in (0, %g]' % SCALE_MAX)
        need(1.0 <= self.rate_hz <= 100.0, 'rate_hz must be in [1, 100]')
        need(0.0 < self.max_joint_speed <= 1.0, 'max_joint_speed must be in (0, 1] rad/s')
        need(0.0 < self.resume_pos < self.jump_pos <= 0.2, 'need 0 < resume_pos < jump_pos <= 0.2')
        need(0.0 < self.resume_ang_deg < self.jump_ang_deg <= 90.0,
             'need 0 < resume_ang_deg < jump_ang_deg <= 90')
        need(0.0 < self.stale_s <= 5.0, 'stale_s must be in (0, 5]')
        need(0.0 <= self.joy_timeout_s <= 10.0, 'joy_timeout_s must be in [0, 10]')
        need(self.floor_z >= 0.045, 'floor_z must be at least 0.045 m')
        need(0.0 < self.elbow_max_deg <= math.degrees(ik.Q_MAX[2]),
             'elbow_max_deg must be in (0, %.2f]' % math.degrees(ik.Q_MAX[2]))
        need(0.0 < self.clutch_windup <= 0.08, 'clutch_windup must be in (0, 0.08] rad')
        need(0.0 <= self.ik_jump <= 1.0, 'ik_jump must be in [0, 1] rad')
        need(0.0 <= self.max_lag <= 0.5, 'max_lag must be in [0, 0.5] m')
        need(0.0 <= self.max_tracking_error <= 1.0, 'max_tracking_error must be in [0, 1] rad')
        need(self.clutch_mode in ('hold', 'toggle'), "clutch_mode must be 'hold' or 'toggle'")
        need(isinstance(self.gripper_requires_clutch, bool), 'gripper_requires_clutch is a bool')
        for name, value in (('gripper_open_rad', self.gripper_open_rad),
                            ('gripper_closed_rad', self.gripper_closed_rad)):
            need(GRIPPER_LOWER <= value <= GRIPPER_UPPER,
                 '%s must be in [%g, %g]' % (name, GRIPPER_LOWER, GRIPPER_UPPER))
        need(abs(self.gripper_open_rad - self.gripper_closed_rad) > 0.1,
             'gripper_open_rad and gripper_closed_rad must differ by more than 0.1 rad')
        rpy = self.input_rotation_rpy_deg
        need(isinstance(rpy, (tuple, list)) and len(rpy) == 3
             and all(isinstance(v, (int, float)) and math.isfinite(v) and abs(v) <= 360.0
                     for v in rpy), 'input_rotation_rpy_deg needs 3 numbers (degrees)')

    @property
    def period(self):
        """Return the nominal period of a tick in s."""
        return 1.0 / self.rate_hz

    @property
    def q_max(self):
        """Return the upper joint limits (the URDF's, the elbow cut at elbow_max_deg)."""
        q_max = ik.Q_MAX.copy()
        q_max[2] = min(q_max[2], math.radians(self.elbow_max_deg))
        return q_max

    @property
    def r_fix(self):
        """Return the rotation that fits the axes of the VR to base_link."""
        roll, pitch, yaw = (math.radians(v) for v in self.input_rotation_rpy_deg)
        return rpy_matrix(roll, pitch, yaw)

    def replace(self, **changes):
        """Return a copy with the changes (checked again)."""
        return dataclasses.replace(self, **changes)


def relative_target(p_ref, r_ref, p_in0, r_in0, p_in, r_in, scale, r_fix=None):
    """Return (p, R) of the target of the arm for the hand (p_in, r_in); see the module text."""
    r_fix = np.eye(3) if r_fix is None else r_fix
    p = np.asarray(p_ref, dtype=float) + scale * (
        r_fix @ (np.asarray(p_in, dtype=float) - np.asarray(p_in0, dtype=float)))
    r = r_fix @ (r_in @ r_in0.T) @ r_fix.T @ r_ref
    return p, r


def target_difference(p_a, r_a, p_b, r_b):
    """Return (distance in m, angle in rad) between two targets."""
    return (float(np.linalg.norm(np.asarray(p_a, dtype=float) - np.asarray(p_b, dtype=float))),
            rotation_angle(np.asarray(r_a).T @ np.asarray(r_b)))


def is_jump(distance, angle, config):
    """Return True if a new target is too far from the last accepted one (50 mm or 30 deg)."""
    return distance >= config.jump_pos or angle >= math.radians(config.jump_ang_deg)


def is_near(distance, angle, config):
    """Return True if the input is back near the stopped target (within 20 mm and 10 deg)."""
    return distance <= config.resume_pos and angle <= math.radians(config.resume_ang_deg)


# ------------------------------------------------------------------------------ the IK

def solve_target(p, r, q_prev, config):
    """
    Return the joint angles (the branch nearest to q_prev) for the target (p, r).

    TargetError with the reason: UNREACHABLE, JOINT_RANGE, ELBOW_LIMIT, ORIENT, FLOOR.
    The orientation is projected on what the five joints can make (project_orientation); theta
    and the roll depend on each other through the lateral offset of the TCP, so they are
    iterated (a contraction of about 0.04 per pass).
    """
    p = np.asarray(p, dtype=float)
    q_prev = np.asarray(q_prev, dtype=float)
    if not (np.all(np.isfinite(p)) and np.all(np.isfinite(r))):
        raise TargetError('UNREACHABLE', 'the target is not finite')
    if p[2] < config.floor_z:
        raise TargetError('FLOOR', 'z %.4f m is below the floor %.4f m' % (p[2], config.floor_z))
    c = ik.CONSTANTS
    flex_ref = float(q_prev[1] + q_prev[2] + q_prev[3])
    roll = float(q_prev[4])
    dx, dy = p[0] - c.pan_x, p[1] - c.pan_y
    flex_sum = flex_ref
    for _ in range(THETA_ITERATIONS):
        _, lateral = ik.tcp_offsets(roll)
        r_squared = dx * dx + dy * dy - lateral * lateral
        if r_squared < 0.0:
            raise TargetError('UNREACHABLE', 'closer to the pan axis than the lateral offset')
        theta = math.atan2(dy, dx) - math.atan2(lateral, math.sqrt(r_squared))
        flex_sum, roll = project_orientation(r, theta, flex_ref, float(q_prev[4]))
    pitch = -flex_sum + c.d4
    try:
        candidates = ik.ik(p[0], p[1], p[2], pitch, roll, q_max=config.q_max)
    except ik.IKUnreachableError as error:
        raise TargetError('UNREACHABLE', str(error)) from None
    except ik.IKJointLimitError as error:
        try:
            ik.ik(p[0], p[1], p[2], pitch, roll)
        except ik.IKError:
            raise TargetError('JOINT_RANGE', str(error)) from None
        raise TargetError('ELBOW_LIMIT', 'the elbow would pass +%g deg' % config.elbow_max_deg
                          ) from None
    except ValueError as error:
        raise TargetError('UNREACHABLE', str(error)) from None
    return ik.select_nearest_solution(candidates, q_prev)


def limit_step(q_prev, q_goal, dt, max_speed):
    """
    Return (q_next, limited): the step from q_prev to q_goal with every joint at most max_speed.

    If the largest joint would be faster than max_speed, the whole step is shortened by the same
    ratio (the direction in the joint space is kept, so the tip stays on the straight joint path).
    """
    q_prev = np.asarray(q_prev, dtype=float)
    delta = np.asarray(q_goal, dtype=float) - q_prev
    allowed = max_speed * dt
    largest = float(np.max(np.abs(delta)))
    if largest <= allowed:
        return np.asarray(q_goal, dtype=float).copy(), False
    return q_prev + delta * (allowed / largest), True


def tip_position(q):
    """Return the position of the tip (analytic FK) for q."""
    return ik.fk(q)[0]


# --------------------------------------------------------------------------- the gripper

def gripper_joint(amount, config):
    """Return the joint (rad) for the amount 0 (open) .. 1 (closed); ValueError if outside."""
    if not (isinstance(amount, (int, float)) and not isinstance(amount, bool)
            and math.isfinite(amount) and 0.0 <= amount <= 1.0):
        raise ValueError('the gripper amount must be a number in [0, 1]')
    return config.gripper_open_rad + amount * (config.gripper_closed_rad - config.gripper_open_rad)


def rising(previous: Optional[bool], now: bool):
    """Return True only for a change from released to pressed (None: not known, no edge)."""
    return previous is False and bool(now)
