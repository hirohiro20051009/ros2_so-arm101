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
Closed-form forward kinematics and analytic inverse kinematics of the SO-ARM101 arm.

Lengths in m, angles in rad. Joint order: shoulder_pan, shoulder_lift, elbow_flex,
wrist_flex, wrist_roll (the gripper is not handled).

Ported from the author's so101_ik.py. The constants are written by hand, are checked against
the values derived from the URDF (urdf_kinematics.derive_constants) by the tests, and replace
the zero-point offsets and lengths that did not match the URDF.

Model. With theta = S1 q1, alpha1 = S2 q2 + D2 + pi/2, alpha2 = S2 q2 + S3 q3 + D3,
pitch = S2 q2 + S3 q3 + S4 q4 + D4 (direction of the approach axis in the pan plane) and
psi = S5 q5 (roll):

    radial = AP + L1 cos(alpha1) + L2 cos(alpha2) + LT cos(pitch) - k cos(psi + psi0) sin(pitch)
    z      = Z0 + HP + L1 sin(alpha1) + L2 sin(alpha2) + LT sin(pitch)
             + k cos(psi + psi0) cos(pitch)
    lat    = y0 + k sin(psi + psi0)
    x      = X0 + radial cos(theta) - lat sin(theta)
    y      = Y0 + radial sin(theta) + lat cos(theta)

The roll psi is an input of the inverse kinematics, so the TCP offsets (a, b) = tcp_offsets(psi)
are known constants and the problem is solved in closed form (no iteration). psi is the joint
angle S5 q5 itself (not the roll of a standard RPY). It is used as given: it is never wrapped by
2 pi (a wrapped value could command a full turn of the wrist), and a value outside the joint
limits raises IKJointLimitError. The two elbow branches share q1 and q5.

The joint limits are arguments (q_min, q_max) of ik() and solve_ik(); the default is the strict
limits of the URDF. A caller that only reads (never sends) may pass wider limits.
"""

import math
from typing import NamedTuple

import numpy as np

from .urdf_kinematics import ModelConstants

# Constants written by hand (values derived from the URDF TheRobotStudio/SO-ARM100 385e8d7).
CONSTANTS = ModelConstants(
    pan_x=0.0388353, pan_y=-8.97657e-09, pan_z=0.0624,
    ap=0.0303992485, hp=0.0541999515,
    l1=0.1160000211, l2=0.1350001852, lt=0.1592272838,
    d2=-0.24378322, d3=0.0385279969, d4=6.5055536e-06,
    k=0.0079031382, psi0=3.1204982576, y0=-0.0001759027)

# Positive directions of the joints (signs of the model angles).
S1, S2, S3, S4, S5 = -1.0, -1.0, -1.0, -1.0, 1.0

# Joint limits (the same as the <limit> of the URDF; checked by the tests).
Q_MIN = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385])
Q_MAX = np.array([1.91986, 1.74533, 1.69, 1.65806, 2.84121])

ROOT_TOLERANCE = 1e-9  # |D| up to 1 + this is accepted as reachable (rounding)


class IKError(RuntimeError):
    """Base class of the errors of the inverse kinematics."""


class IKUnreachableError(IKError):
    """The target is geometrically unreachable."""


class IKJointLimitError(IKError):
    """A geometric solution exists, but all candidates exceed the joint limits."""


class Geometry(NamedTuple):
    """Intermediate result of the inverse kinematics (before the joint limits are applied)."""

    radial: float  # radial distance of the TCP from the pan axis (>= 0)
    cos_elbow: float  # D of the law of cosines (-1..1)
    candidates: tuple  # the two joint vectors (elbow +, elbow -), not limited


def normalize_angle(angle):
    """Return the angle wrapped to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def within_limits(q, q_min=Q_MIN, q_max=Q_MAX):
    """Return True if every joint is inside its limit (inclusive; the default is the URDF's)."""
    q = np.asarray(q, dtype=float)
    return bool(np.all(q >= q_min) and np.all(q <= q_max))


def tcp_offsets(roll, c=CONSTANTS):
    """Return (in-plane offset a, lateral offset b) of the TCP from the approach axis."""
    return c.k * math.cos(roll + c.psi0), c.y0 + c.k * math.sin(roll + c.psi0)


def fk(q, c=CONSTANTS):
    """Return (xyz, pitch, roll) of the TCP in base_link for q = [q1..q5] (roll included)."""
    q1, q2, q3, q4, q5 = (float(v) for v in q)
    theta = S1 * q1
    alpha1 = S2 * q2 + c.d2 + math.pi / 2.0
    alpha2 = S2 * q2 + S3 * q3 + c.d3
    pitch = S2 * q2 + S3 * q3 + S4 * q4 + c.d4
    roll = S5 * q5
    a, b = tcp_offsets(roll, c)

    radial = (c.ap + c.l1 * math.cos(alpha1) + c.l2 * math.cos(alpha2)
              + c.lt * math.cos(pitch) - a * math.sin(pitch))
    z = (c.pan_z + c.hp + c.l1 * math.sin(alpha1) + c.l2 * math.sin(alpha2)
         + c.lt * math.sin(pitch) + a * math.cos(pitch))
    x = c.pan_x + radial * math.cos(theta) - b * math.sin(theta)
    y = c.pan_y + radial * math.sin(theta) + b * math.cos(theta)
    return np.array([x, y, z]), pitch, roll


def ik_geometry(x, y, z, pitch, roll, c=CONSTANTS):
    """Solve the geometry for XYZ + pitch + roll (both elbow branches, limits not applied)."""
    values = (x, y, z, pitch, roll)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('the target and the orientation must be finite numbers')
    a, b = tcp_offsets(roll, c)

    dx, dy = x - c.pan_x, y - c.pan_y
    r_squared = dx * dx + dy * dy - b * b
    if r_squared < 0.0:
        raise IKUnreachableError('the target is closer to the pan axis than the lateral offset')
    radial = math.sqrt(r_squared)
    theta = math.atan2(dy, dx) - math.atan2(b, radial)
    q1 = theta / S1

    # TCP -> point on the approach axis -> wrist_flex axis (in the pan plane)
    rw = radial + a * math.sin(pitch) - c.ap - c.lt * math.cos(pitch)
    zw = z - a * math.cos(pitch) - c.pan_z - c.hp - c.lt * math.sin(pitch)
    cos_elbow = (rw * rw + zw * zw - c.l1 * c.l1 - c.l2 * c.l2) / (2.0 * c.l1 * c.l2)
    if abs(cos_elbow) > 1.0 + ROOT_TOLERANCE:
        raise IKUnreachableError('the target is geometrically unreachable')
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    sin_elbow = math.sqrt(max(0.0, 1.0 - cos_elbow * cos_elbow))

    candidates = []
    for sign in (1.0, -1.0):
        beta3 = math.atan2(sign * sin_elbow, cos_elbow)
        alpha1 = (math.atan2(zw, rw)
                  - math.atan2(c.l2 * math.sin(beta3), c.l1 + c.l2 * math.cos(beta3)))
        alpha2 = alpha1 + beta3
        q2 = (alpha1 - math.pi / 2.0 - c.d2) / S2
        q3 = (alpha2 - c.d3 - S2 * q2) / S3
        q4 = (pitch - c.d4 - S2 * q2 - S3 * q3) / S4
        candidates.append(np.array([q1, q2, q3, q4, roll / S5]))
    return Geometry(radial, cos_elbow, tuple(candidates))


def ik(x, y, z, pitch, roll, c=CONSTANTS, q_min=Q_MIN, q_max=Q_MAX):
    """Return the candidates (elbow +, -) inside the joint limits (default: the URDF's)."""
    valid = [q for q in ik_geometry(x, y, z, pitch, roll, c).candidates
             if within_limits(q, q_min, q_max)]
    if not valid:
        raise IKJointLimitError('a geometric solution exists, but every candidate is out of range')
    return valid


def select_nearest_solution(solutions, current_q):
    """Return the candidate nearest to the current joint angles (angle differences wrapped)."""
    current_q = np.asarray(current_q, dtype=float)

    def distance(q):
        return np.linalg.norm([normalize_angle(a - b) for a, b in zip(q, current_q)])
    return min(solutions, key=distance)


def solve_ik(x, y, z, current_q, pitch=None, roll=None, c=CONSTANTS, q_min=Q_MIN, q_max=Q_MAX):
    """Return the joint angles for XYZ; pitch and roll default to those of current_q."""
    _, current_pitch, current_roll = fk(current_q, c)
    pitch = current_pitch if pitch is None else pitch
    roll = current_roll if roll is None else roll
    return select_nearest_solution(ik(x, y, z, pitch, roll, c, q_min, q_max), current_q)
