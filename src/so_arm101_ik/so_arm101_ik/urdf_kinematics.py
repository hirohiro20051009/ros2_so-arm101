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

"""Forward kinematics of the SO-ARM101 arm read from the URDF text (numpy only, no ROS)."""

import math
from typing import NamedTuple
from xml.etree import ElementTree

import numpy as np

ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
BASE_LINK = 'base_link'
TIP_LINK = 'gripper_frame_link'


class ModelConstants(NamedTuple):
    """
    Constants of the closed-form model of the arm (lengths in m, angles in rad).

    pan_x, pan_y, pan_z : origin of the shoulder_pan joint in base_link
    ap, hp : shoulder_pan axis -> shoulder_lift axis (radial, up) in the pan plane
    l1, l2 : shoulder_lift -> elbow_flex, elbow_flex -> wrist_flex
    lt     : wrist_flex axis -> the plane of the TCP, along the approach axis
    d2, d3 : zero-point offsets of the directions of l1 (= pi/2 + d2) and l2 (= d3)
    d4     : zero-point offset of the pitch of the approach axis
    k, psi0, y0 : the TCP is k away from the wrist_roll axis (at angle psi0 for roll 0),
                  y0 is the constant lateral offset of the arm plane
    """

    pan_x: float
    pan_y: float
    pan_z: float
    ap: float
    hp: float
    l1: float
    l2: float
    lt: float
    d2: float
    d3: float
    d4: float
    k: float
    psi0: float
    y0: float


def rpy_matrix(roll, pitch, yaw):
    """Return the URDF rotation Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def axis_angle_matrix(axis, angle):
    """Return the rotation by angle about the (not necessarily unit) axis (Rodrigues)."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * (cross @ cross)


def _floats(text, count, default):
    if text is None:
        return list(default)
    values = [float(v) for v in text.split()]
    if len(values) != count:
        raise ValueError(f'expected {count} numbers, got {text!r}')
    return values


class _Joint:
    def __init__(self, element):
        self.name = element.get('name')
        self.kind = element.get('type')
        origin = element.find('origin')
        xyz = _floats(None if origin is None else origin.get('xyz'), 3, (0.0, 0.0, 0.0))
        rpy = _floats(None if origin is None else origin.get('rpy'), 3, (0.0, 0.0, 0.0))
        self.xyz = np.array(xyz)
        self.rotation = rpy_matrix(*rpy)
        axis = element.find('axis')
        self.axis = np.array(
            _floats(None if axis is None else axis.get('xyz'), 3, (1.0, 0.0, 0.0)))
        limit = element.find('limit')
        if limit is not None and limit.get('lower') is not None:
            self.limits = (float(limit.get('lower')), float(limit.get('upper')))
        else:
            self.limits = None
        self.moving = self.kind in ('revolute', 'continuous')


class UrdfChain:
    """Serial chain base_link -> tip_link built from the joint elements of a URDF string."""

    def __init__(self, urdf_xml, base_link=BASE_LINK, tip_link=TIP_LINK):
        root = ElementTree.fromstring(urdf_xml)
        by_child = {}
        for element in root.findall('joint'):
            child = element.find('child').get('link')
            by_child[child] = (element.find('parent').get('link'), _Joint(element))
        chain = []
        link = tip_link
        while link != base_link:
            if link not in by_child:
                raise ValueError(f'no joint chain from {base_link} to {tip_link}')
            link, joint = by_child[link]
            chain.append(joint)
        self.joints = chain[::-1]
        moving = tuple(j.name for j in self.joints if j.moving)
        if moving != ARM_JOINTS:
            raise ValueError(f'moving joints of the chain are {moving}, expected {ARM_JOINTS}')

    def joint_limits(self):
        """Return the (lower, upper) limits of the five arm joints as two arrays."""
        lower = [j.limits[0] for j in self.joints if j.moving]
        upper = [j.limits[1] for j in self.joints if j.moving]
        return np.array(lower), np.array(upper)

    def frames(self, q):
        """Return {joint name: (position, rotation)} of the frame after each joint in base_link."""
        q = np.asarray(q, dtype=float)
        if q.shape != (len(ARM_JOINTS),):
            raise ValueError(f'q must have {len(ARM_JOINTS)} elements')
        position = np.zeros(3)
        rotation = np.eye(3)
        result = {}
        index = 0
        for joint in self.joints:
            position = position + rotation @ joint.xyz
            rotation = rotation @ joint.rotation
            if joint.moving:
                rotation = rotation @ axis_angle_matrix(joint.axis, q[index])
                index += 1
            result[joint.name] = (position, rotation)
        return result

    def fk(self, q):
        """Return (position, rotation) of the tip link in base_link for the five arm joints."""
        return self.frames(q)[self.joints[-1].name]

    def joint(self, name):
        """Return the joint with the given name (its origin and axis are in the parent frame)."""
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise KeyError(name)


def derive_constants(urdf_xml):
    """
    Derive the constants of the closed-form model from the joint origins of the URDF.

    The arm is evaluated at q = 0 (pan plane = x-z plane of base_link) and with the
    wrist_roll joint at pi/2. The sign convention of the model is S1..S4 = -1 and S5 = +1
    (see analytic_ik); a different convention shows up as a mismatch in the FK tests.
    """
    chain = UrdfChain(urdf_xml)
    zero = np.zeros(len(ARM_JOINTS))
    rolled = zero.copy()
    rolled[4] = math.pi / 2.0
    frames = chain.frames(zero)
    tcp0 = chain.fk(zero)[0]
    tcp1 = chain.fk(rolled)[0]

    pan = chain.joint('shoulder_pan').xyz
    lift = frames['shoulder_lift'][0]
    elbow = frames['elbow_flex'][0]
    flex = frames['wrist_flex'][0]

    def plane(point):
        return np.array([point[0] - pan[0], point[2] - pan[2]])

    ap, hp = plane(lift)
    v1 = plane(elbow) - plane(lift)
    v2 = plane(flex) - plane(elbow)
    d2 = math.atan2(v1[1], v1[0]) - math.pi / 2.0
    d3 = math.atan2(v2[1], v2[0])

    # approach axis = axis of wrist_roll, directed from the wrist towards the TCP
    roll_joint = chain.joint('wrist_roll')
    axis = frames['wrist_roll'][1] @ roll_joint.axis
    axis = axis / np.linalg.norm(axis)
    if float(np.dot(tcp0 - frames['wrist_roll'][0], axis)) < 0.0:
        axis = -axis
    d4 = math.atan2(axis[2], axis[0])
    along = np.array([math.cos(d4), math.sin(d4)])
    across = np.array([-math.sin(d4), math.cos(d4)])

    t0 = plane(tcp0) - plane(flex)
    t1 = plane(tcp1) - plane(flex)
    lt = float(np.dot(t0, along))
    a0 = float(np.dot(t0, across))  # k cos(psi0)
    a1 = float(np.dot(t1, across))  # -k sin(psi0)
    k_cos, k_sin = a0, -a1
    y0 = float(tcp0[1] - pan[1]) - k_sin
    return ModelConstants(
        pan_x=float(pan[0]), pan_y=float(pan[1]), pan_z=float(pan[2]),
        ap=float(ap), hp=float(hp),
        l1=float(np.linalg.norm(v1)), l2=float(np.linalg.norm(v2)), lt=lt,
        d2=d2, d3=d3, d4=d4,
        k=math.hypot(k_cos, k_sin), psi0=math.atan2(k_sin, k_cos), y0=y0)
