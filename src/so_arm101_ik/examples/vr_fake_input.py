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
Fake VR input for the VR follow node (P6 b3): a Joy and a PoseStamped in place of the VR.

Two modes (exactly one of --mock and --real):
  --mock: the MOCK arm only. It refuses to run unless ROS_DOMAIN_ID is 87 (the mock domain).
  --real: the REAL arm (docs/p6_real_check.md). It refuses to run in domain 87, never gives
          the arm a trajectory (no publisher to /arm_controller/joint_trajectory, no action
          client), keeps every move of the TARGET of the arm within 30 mm and 20 mm/s, plays
          only line, stale, jump and gripper, and waits for Enter before each scenario.
          --arm-read-only (c0) raises the limit to 60 mm, only if the robot_description
          says read_only; --expect blocked or stationary is for such an arm.

Mock: start the mock arm first (ros2 launch so_arm101_bringup bringup.launch.py mock:=true
use_rviz:=false), then ros2 launch so_arm101_ik vr_follow.launch.py, then this script.

In the mock it moves the arm to a working pose (a trajectory to the arm_controller that does not
go through the follow node), plays the scenarios line, jump, stale, range, rotation, gripper and
busy, watches /vr_follow/status, /arm_controller/joint_trajectory, the gripper topic and the
gripper action, and prints what it checked. The exit code is 0 only if every check passed.

  ROS_DOMAIN_ID=87 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 vr_fake_input.py --mock \
      --out DIR [--scenarios line jump ...]
"""

import argparse
import csv
import os
import sys
import time

from action_msgs.msg import GoalStatusArray
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
import numpy as np
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik import follow_calc as fc
from so_arm101_ik.urdf_kinematics import rpy_matrix
from so_arm101_ik.vr_follow_node import description_is_read_only
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

MOCK_DOMAIN = '87'
JOINTS = fc.JOINT_NAMES
WORK_POSE = np.array([0.0, 0.0, 0.8, -0.6, 0.0])  # the tip is at z about 0.10 m
HAND_BASE = np.array([0.30, 0.0, 0.30])  # where the fake hand is (only the moves matter)
JOY_HZ = 50.0
POSE_HZ = 90.0
MARGIN = 0.08  # s: a point this close to a change of the state is not judged
SCENARIOS = ('line', 'jump', 'stale', 'range', 'rotation', 'gripper', 'busy')
RANGE_REASONS = ('UNREACHABLE', 'JOINT_RANGE', 'ELBOW_LIMIT', 'FLOOR', 'ORIENT', 'IK_JUMP')
GRIPPER_STATUS = '/gripper_controller/gripper_cmd/_action/status'
JTC_ACTION = '/arm_controller/follow_joint_trajectory'
CONTROLLER_STATE = '/arm_controller/controller_state'

# The real arm (--real; docs/p6_real_check.md). The limits are for the moves of the TARGET of the
# arm (after the scale): the hand moves target / scale.
REAL_SCENARIOS = ('line', 'stale', 'jump', 'gripper')
REAL_MAX_AMPLITUDE_MM = 30.0
READ_ONLY_MAX_AMPLITUDE_MM = 60.0  # only if the robot_description says read_only (c0)
REAL_MAX_SPEED_MM_S = 20.0
REAL_JUMP_MM = 60.0  # the step of the jump scenario: over the 50 mm limit, so the node must stop
REAL_JUMP_MAX_TIP_MM = 5.0  # the arm must not follow the jump
STATIONARY_MM = 3.0  # a read-only arm does not move
EXPECTATIONS = ('normal', 'blocked', 'stationary')


# ------------------------------------------------------------------------------ the plan

def interpolate(keys, t):
    """Return the value at t of keyframes [(time, vector), ...] (linear; before/after: ends)."""
    if not keys:
        return np.zeros(3)
    if t <= keys[0][0]:
        return np.array(keys[0][1], dtype=float)
    for (t0, v0), (t1, v1) in zip(keys, keys[1:]):
        if t0 <= t < t1:
            ratio = (t - t0) / (t1 - t0)
            return np.array(v0, dtype=float) * (1.0 - ratio) + np.array(v1, dtype=float) * ratio
    return np.array(keys[-1][1], dtype=float)


def inside(windows, t):
    return any(a <= t < b for a, b in windows)


class Plan:
    """What the fake VR does as a function of the time since the start of a scenario."""

    def __init__(self, duration, disp=(), rpy_deg=(), clutch=(), opens=(), closes=(),
                 pose_off=(), joy_off=(), spikes=(), busy_at=None):
        self.duration = duration
        self.disp = list(disp)  # keyframes (t, (dx, dy, dz)) of the hand in m
        self.rpy_deg = list(rpy_deg)  # keyframes (t, (roll, pitch, yaw)) in degrees
        self.clutch = list(clutch)  # windows (start, end) with the clutch pressed
        self.opens = list(opens)
        self.closes = list(closes)
        self.pose_off = list(pose_off)  # windows without pose messages
        self.joy_off = list(joy_off)  # windows without joy messages
        self.spikes = list(spikes)  # (t, (dx, dy, dz)): one message with this offset
        self.busy_at = busy_at  # time of a goal sent to the arm_controller

    def hand(self, t):
        position = HAND_BASE + interpolate(self.disp, t)
        roll, pitch, yaw = np.radians(interpolate(self.rpy_deg, t)) if self.rpy_deg else (0, 0, 0)
        return position, rpy_matrix(roll, pitch, yaw)

    def buttons(self, t):
        return (inside(self.clutch, t), inside(self.opens, t),
                inside(self.closes, t))


def make_plans():
    plans = {}
    plans['line'] = Plan(
        8.0, disp=[(1.5, (0, 0, 0)), (3.5, (0, 0.04, 0)), (4.5, (0, 0.04, 0)), (6.5, (0, 0, 0))],
        clutch=[(1.0, 7.0)])
    plans['jump'] = Plan(
        9.0, disp=[(1.0, (0, 0, 0)), (3.0, (0, 0.01, 0)), (3.001, (0, 0.09, 0)),
                   (4.5, (0, 0.09, 0)), (4.501, (0, 0.012, 0))],
        clutch=[(1.0, 8.0)], spikes=[(6.0, (0, 0.08, 0))])
    plans['stale'] = Plan(
        14.0, disp=[(1.0, (0, 0, 0)), (2.5, (0, 0.01, 0)), (6.2, (0, 0.01, 0)),
                    (6.201, (0, 0.12, 0))],
        clutch=[(1.0, 7.5), (8.5, 12.5)], pose_off=[(3.0, 4.0), (5.0, 6.2)],
        joy_off=[(10.0, 11.5)])
    plans['range'] = Plan(
        15.0, disp=[(1.0, (0, 0, 0)), (4.0, (0, 0, -0.09)), (5.0, (0, 0, -0.09)),
                    (7.0, (0, 0, 0)), (8.0, (0, 0, 0)), (12.0, (-0.20, 0, 0))],
        clutch=[(1.0, 14.0)])
    plans['rotation'] = Plan(
        10.0, rpy_deg=[(1.0, (0, 0, 0)), (3.0, (20, 0, 0)), (4.0, (20, 0, 0)), (6.0, (0, 0, 0)),
                       (7.5, (0, 0, 15)), (8.5, (0, 0, 15))],
        clutch=[(1.0, 9.0)])
    plans['gripper'] = Plan(
        12.5, disp=[(5.5, (0, 0, 0)), (7.0, (0, 0.01, 0))], clutch=[(5.5, 11.0)],
        opens=[(2.5, 2.9), (4.0, 4.4)], closes=[(1.0, 1.4), (4.0, 4.4)],
        pose_off=[(8.5, 9.5)], spikes=[(7.0, (0, 0.08, 0))])
    plans['busy'] = Plan(
        13.0, disp=[(1.0, (0, 0, 0)), (3.0, (0, 0.01, 0))],
        clutch=[(1.0, 9.0), (10.0, 12.0)], busy_at=3.0)
    return plans


def check_args(args, domain):
    """Return the reason to refuse the arguments (a text), or None if they are allowed."""
    real_plans = bool(getattr(args, 'real_plans', False))
    if bool(args.mock) == bool(args.real):
        return 'give exactly one of --mock and --real'
    if args.mock:
        if domain != MOCK_DOMAIN:
            return '--mock needs ROS_DOMAIN_ID=%s (the mock domain)' % MOCK_DOMAIN
        if not real_plans:
            return None
    else:
        if real_plans:
            return '--real-plans is only for --mock'
        if domain == MOCK_DOMAIN:
            return '--real must not run in the mock domain (ROS_DOMAIN_ID=%s)' % MOCK_DOMAIN
    if not args.scenarios:
        return '--real needs --scenarios (%s)' % ' '.join(REAL_SCENARIOS)
    others = [s for s in args.scenarios if s not in REAL_SCENARIOS]
    if others:
        return 'the scenarios %s are not allowed with --real (only %s)' % (
            ' '.join(others), ' '.join(REAL_SCENARIOS))
    cap = READ_ONLY_MAX_AMPLITUDE_MM if args.arm_read_only else REAL_MAX_AMPLITUDE_MM
    if not 1.0 <= args.amplitude_mm <= cap:
        return '--amplitude-mm must be in [1, %g]%s' % (
            cap, '' if args.arm_read_only else ' (60 only with --arm-read-only)')
    if not 1.0 <= args.speed_mm_s <= REAL_MAX_SPEED_MM_S:
        return '--speed-mm-s must be in [1, %g]' % REAL_MAX_SPEED_MM_S
    if not 1 <= args.gripper_cycles <= 3:
        return '--gripper-cycles must be 1, 2 or 3'
    if args.expect != 'normal' and not args.arm_read_only:
        return '--expect %s needs --arm-read-only' % args.expect
    return None


def make_real_plan(name, amplitude, speed, scale, cycles=1):
    """
    Return the Plan of a real-arm scenario; amplitude (m) and speed (m/s) are of the TARGET.

    The hand moves target / scale. line: 0 -> +A -> -A -> 0 along y. stale: +A, the pose stops for
    1 s and comes back to the same place. jump: a step of REAL_JUMP_MM of the target that the node
    must refuse. gripper: only buttons (CLOSE then OPEN), no clutch.
    """
    k = 1.0 / scale
    t1 = amplitude / speed
    if name == 'line':
        keys = [(1.0, 0.0), (1.0 + t1, amplitude), (2.0 + t1, amplitude),
                (2.0 + 3 * t1, -amplitude), (3.0 + 3 * t1, -amplitude), (3.0 + 4 * t1, 0.0)]
        end = 3.0 + 4 * t1 + 0.5
        return Plan(end + 1.0, disp=[(t, (0.0, y * k, 0.0)) for t, y in keys],
                    clutch=[(1.0, end)])
    if name == 'stale':
        keys = [(1.0, 0.0), (1.0 + t1, amplitude), (3.5 + t1, amplitude),
                (3.5 + 2 * t1, 0.0)]
        end = 3.5 + 2 * t1 + 0.5
        return Plan(end + 1.0, disp=[(t, (0.0, y * k, 0.0)) for t, y in keys],
                    clutch=[(1.0, end)], pose_off=[(1.0 + t1 + 0.5, 1.0 + t1 + 1.5)])
    if name == 'jump':
        jump = REAL_JUMP_MM / 1000.0
        keys = [(1.0, 0.0), (2.5, 0.0), (2.5001, jump), (4.0, jump), (4.0001, 0.0), (5.5, 0.0)]
        return Plan(6.5, disp=[(t, (0.0, y * k, 0.0)) for t, y in keys], clutch=[(1.0, 5.5)])
    if name == 'gripper':
        closes = [(1.0 + 4.0 * c, 1.4 + 4.0 * c) for c in range(cycles)]
        opens = [(3.0 + 4.0 * c, 3.4 + 4.0 * c) for c in range(cycles)]
        return Plan(1.0 + 4.0 * cycles + 0.5, closes=closes, opens=opens)
    raise ValueError('no real scenario %s' % name)


# ---------------------------------------------------------------------------- the monitor

class Monitor(Node):
    """Publishes the fake inputs and records the state, the points and the gripper."""

    def __init__(self, buttons, real=False):
        super().__init__('vr_fake_input')
        self.buttons = buttons  # (clutch, open, close) indices
        self.real = real
        self.joy_pub = self.create_publisher(Joy, '/vr/right/joy', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vr/right/pose', 10)
        if not real:  # the real arm is never given a trajectory by this script
            self.traj_pub = self.create_publisher(JointTrajectory,
                                                  '/arm_controller/joint_trajectory', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/vr_follow/status', self._on_status, latched)
        self.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory',
                                 self._on_point, 100)
        self.create_subscription(Float64, '/vr_follow/gripper', self._on_gripper_amount, latched)
        self.create_subscription(GoalStatusArray, GRIPPER_STATUS, self._on_gripper_status, latched)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 50)
        self.create_subscription(JointTrajectoryControllerState, CONTROLLER_STATE,
                                 self._on_controller_state, 50)
        if not real:
            self.jtc_client = ActionClient(self, FollowJointTrajectory, JTC_ACTION)
        self.param_client = self.create_client(GetParameters, '/vr_follow/get_parameters')
        self.rsp_client = self.create_client(GetParameters,
                                             '/robot_state_publisher/get_parameters')
        self.started = time.monotonic()
        self.status = []  # (t, state, reason, detail)
        self.points = []  # (t, positions)
        self.amounts = []  # (t, amount)
        self.goal_ids = set()
        self.goal_times = []  # (t, id) of the new gripper goals
        self.q = None
        self.gripper = None
        self.samples = []  # (t, tip xyz, q5, elbow, gripper, q)
        self.refs = []  # (t, reference of the arm_controller)
        self.blocked_seen = False

    def _on_status(self, msg):
        parts = msg.data.split()
        detail = {}
        for part in parts[2:]:
            if '=' in part:
                key, value = part.split('=', 1)
                try:
                    detail[key] = float(value)
                except ValueError:
                    detail[key] = value
        state = parts[0]
        self.blocked_seen = self.blocked_seen or state == 'BLOCKED'
        self.status.append((time.monotonic(), state, parts[1] if len(parts) > 1 else '', detail))

    def _on_controller_state(self, msg):
        names = list(msg.joint_names)
        if all(j in names for j in JOINTS):
            reference = np.array([msg.reference.positions[names.index(j)] for j in JOINTS])
            self.refs.append((time.monotonic(), reference))

    def _on_point(self, msg):
        if msg.points:
            self.points.append((time.monotonic(), list(msg.points[0].positions)))

    def _on_gripper_amount(self, msg):
        if time.monotonic() - self.started > 1.0:  # not the latched value of an earlier run
            self.amounts.append((time.monotonic(), msg.data))

    def _on_gripper_status(self, msg):
        for status in msg.status_list:
            goal = bytes(status.goal_info.goal_id.uuid).hex()
            if goal not in self.goal_ids:
                self.goal_ids.add(goal)
                self.goal_times.append((time.monotonic(), goal))

    def _on_joint_state(self, msg):
        names = list(msg.name)
        if not all(j in names for j in JOINTS):
            return
        self.q = np.array([msg.position[names.index(j)] for j in JOINTS])
        if 'gripper' in names:
            self.gripper = msg.position[names.index('gripper')]
        tip = ik.fk(self.q)[0]
        self.samples.append((time.monotonic(), tip, self.q[4], self.q[2], self.gripper,
                             self.q.copy()))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.002)

    def last_state(self):
        return self.status[-1][1] if self.status else None

    # --- publishing
    def publish_joy(self, clutch, open_, close):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        buttons = [0] * (max(self.buttons) + 1)
        buttons[self.buttons[0]] = int(clutch)
        buttons[self.buttons[1]] = int(open_)
        buttons[self.buttons[2]] = int(close)
        msg.buttons = buttons
        self.joy_pub.publish(msg)

    def publish_pose(self, position, rotation):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        pos = msg.pose.position
        pos.x, pos.y, pos.z = (float(v) for v in position)
        x, y, z, w = fc.matrix_to_quaternion(rotation)
        msg.pose.orientation.x, msg.pose.orientation.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = z, w
        self.pose_pub.publish(msg)

    def send_trajectory(self, q, seconds):
        msg = JointTrajectory()
        msg.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        msg.points = [point]
        self.traj_pub.publish(msg)


def wait_for(monitor, condition, timeout, text):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(monitor, timeout_sec=0.05)
        if condition():
            return
    sys.exit('timeout: %s' % text)


def prepare(monitor):
    """Move the mock arm to the working pose (a trajectory that does not go through vr_follow)."""
    wait_for(monitor, lambda: monitor.q is not None and monitor.status, 20.0,
             'no /joint_states or /vr_follow/status: are the mock arm and vr_follow running?')
    if np.max(np.abs(monitor.q - WORK_POSE)) > 0.02:
        monitor.send_trajectory(WORK_POSE, 4.0)
        wait_for(monitor, lambda: np.max(np.abs(monitor.q - WORK_POSE)) < 0.02, 12.0,
                 'the arm did not reach the working pose')
    monitor.spin(0.5)


def send_busy_goal(monitor):
    """Send a goal that holds the present joints for 2 s to the arm_controller (a user of it)."""
    if not monitor.jtc_client.wait_for_server(timeout_sec=3.0):
        return False
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(JOINTS)
    point = JointTrajectoryPoint()
    point.positions = [float(v) for v in monitor.q]
    point.time_from_start = Duration(sec=2)
    goal.trajectory.points = [point]
    monitor.jtc_client.send_goal_async(goal)
    return True


def play(monitor, plan):
    """Play a plan and return the records of it (times from the start of the plan)."""
    # return to IDLE with the buttons released, then synchronize the buttons
    for _ in range(int(0.8 * JOY_HZ)):
        monitor.publish_joy(False, False, False)
        monitor.publish_pose(*plan.hand(0.0))
        monitor.spin(1.0 / JOY_HZ)
    wait_for(monitor, lambda: monitor.last_state() == 'IDLE', 5.0, 'the node did not go IDLE')
    t0 = time.monotonic()
    marks = (len(monitor.status), len(monitor.points), len(monitor.amounts),
             len(monitor.goal_times), len(monitor.samples), len(monitor.refs))
    next_joy = next_pose = 0.0
    spikes = sorted(plan.spikes)
    busy_sent = None
    while True:
        t = time.monotonic() - t0
        if t >= plan.duration:
            break
        if t >= next_joy:
            next_joy += 1.0 / JOY_HZ
            if not inside(plan.joy_off, t):
                monitor.publish_joy(*plan.buttons(t))
        if t >= next_pose:
            next_pose += 1.0 / POSE_HZ
            if not inside(plan.pose_off, t):
                position, rotation = plan.hand(t)
                if spikes and t >= spikes[0][0]:
                    position = position + np.array(spikes.pop(0)[1])
                monitor.publish_pose(position, rotation)
        if plan.busy_at is not None and busy_sent is None and t >= plan.busy_at:
            busy_sent = send_busy_goal(monitor)
        rclpy.spin_once(monitor, timeout_sec=0.001)
    for _ in range(int(0.5 * JOY_HZ)):  # release everything at the end
        monitor.publish_joy(False, False, False)
        monitor.spin(1.0 / JOY_HZ)
    monitor.spin(0.5)

    def rel(items):
        return [(i[0] - t0,) + tuple(i[1:]) for i in items]

    return {
        'status': rel(monitor.status[marks[0]:]), 'points': rel(monitor.points[marks[1]:]),
        'amounts': rel(monitor.amounts[marks[2]:]), 'goals': rel(monitor.goal_times[marks[3]:]),
        'samples': rel(monitor.samples[marks[4]:]), 'refs': rel(monitor.refs[marks[5]:]),
        'busy_sent': busy_sent,
    }


# ------------------------------------------------------------------------------- checks

def transitions(status):
    """Return [(t, state, reason)] where the (state, reason) changes."""
    result = []
    for t, state, reason, _ in status:
        if not result or result[-1][1:] != (state, reason):
            result.append((t, state, reason))
    return result


def states_of(status):
    result = []
    for _, state, _ in transitions(status):
        if not result or result[-1] != state:
            result.append(state)
    return result


def is_subsequence(wanted, seen):
    iterator = iter(seen)
    return all(item in iterator for item in wanted)


def points_outside_tracking(record):
    """Return the times of the points that came while the state was not TRACKING."""
    status = record['status']
    bad = []
    for t, _ in record['points']:
        before = [s for s in status if s[0] <= t - MARGIN]
        after = [s for s in status if s[0] <= t + MARGIN]
        if before and after and before[-1][1] != 'TRACKING' and after[-1][1] != 'TRACKING':
            bad.append(t)
    return bad


def window(record, a, b):
    return [s for s in record['samples'] if a <= s[0] <= b]


def common_checks(record):
    outside = points_outside_tracking(record)
    return [('no point while the state is not TRACKING (%d points)' % len(record['points']),
             not outside, 'points at %s' % ['%.2f' % t for t in outside[:5]])]


def check_line(r):
    checks = common_checks(r)
    states = states_of(r['status'])
    only_one = states in (['IDLE', 'TRACKING', 'IDLE'], ['TRACKING', 'IDLE'])
    checks.append(('states IDLE, TRACKING, IDLE only', only_one, str(states)))
    checks.append(('points came (>= 30)', len(r['points']) >= 30, str(len(r['points']))))
    peak = window(r, 3.9, 4.6)
    start = window(r, 0.0, 0.9)
    if peak and start:
        moved = float(np.mean([s[1][1] for s in peak]) - np.mean([s[1][1] for s in start]))
        checks.append(('the tip followed +40 mm in y (scale 1.0, within 6 mm)',
                       abs(moved - 0.04) < 0.006, '%.1f mm' % (moved * 1000)))
        end = window(r, 7.3, 7.9)
        back = float(np.mean([s[1][1] for s in end]) - np.mean([s[1][1] for s in start]))
        checks.append(('the tip came back (within 6 mm)', abs(back) < 0.006,
                       '%.1f mm' % (back * 1000)))
    return checks


def check_jump(r):
    checks = common_checks(r)
    states = states_of(r['status'])
    checks.append(('IDLE, TRACKING, HOLD_JUMP, TRACKING, HOLD_JUMP, TRACKING, IDLE',
                   is_subsequence(['TRACKING', 'HOLD_JUMP', 'TRACKING', 'HOLD_JUMP', 'TRACKING',
                                   'IDLE'], states), str(states)))
    reasons = {t[2] for t in transitions(r['status']) if t[1] == 'HOLD_JUMP'}
    checks.append(('reason TARGET_JUMP', reasons == {'TARGET_JUMP'}, str(reasons)))
    ys = [s[1][1] for s in r['samples'] if 1.0 <= s[0] <= 8.0]
    start = np.mean([s[1][1] for s in window(r, 0.0, 0.9)])
    checks.append(('the tip never followed the 80 mm jump (peak under +30 mm)',
                   max(ys) - start < 0.03, '%.1f mm' % ((max(ys) - start) * 1000)))
    return checks


def check_stale(r):
    checks = common_checks(r)
    states = states_of(r['status'])
    checks.append(('TRACKING, STALE, TRACKING, STALE, IDLE, TRACKING, IDLE',
                   is_subsequence(['TRACKING', 'STALE', 'TRACKING', 'STALE', 'IDLE', 'TRACKING',
                                   'IDLE'], states), str(states)))
    reasons = [t[2] for t in transitions(r['status'])]
    for wanted in ('POSE', 'WAIT_RETURN', 'RELEASED', 'JOY_TIMEOUT'):
        checks.append(('reason %s appeared' % wanted, wanted in reasons, str(reasons)))
    timeout = [t[0] for t in transitions(r['status']) if t[2] == 'JOY_TIMEOUT']
    tracking = [t[0] for t in transitions(r['status']) if t[1] == 'TRACKING']
    checks.append(('no restart after the JOY_TIMEOUT while the clutch stays pressed',
                   bool(timeout) and all(t < timeout[0] for t in tracking), '%s %s' % (
                       timeout, tracking)))
    return checks


def check_range(r):
    checks = common_checks(r)
    states = states_of(r['status'])
    checks.append(('TRACKING, HOLD_RANGE, TRACKING, HOLD_RANGE, IDLE',
                   is_subsequence(['TRACKING', 'HOLD_RANGE', 'TRACKING', 'HOLD_RANGE', 'IDLE'],
                                  states), str(states)))
    reasons = [t[2] for t in transitions(r['status']) if t[1] == 'HOLD_RANGE']
    checks.append(('the first stop is FLOOR', bool(reasons) and reasons[0] == 'FLOOR',
                   str(reasons)))
    checks.append(('every HOLD_RANGE reason is one of the codes',
                   all(x in RANGE_REASONS for x in reasons), str(reasons)))
    checks.append(('a second stop from moving toward the base',
                   len(reasons) >= 2 and reasons[-1] != 'FLOOR', str(reasons)))
    z = min(s[1][2] for s in r['samples'])
    elbow = max(s[3] for s in r['samples'])
    checks.append(('the tip stayed above 0.043 m', z > 0.043, '%.4f m' % z))
    checks.append(('the elbow stayed under +91 deg (1.588 rad)', elbow < 1.588 + 0.01,
                   '%.3f rad' % elbow))
    return checks


def check_rotation(r):
    checks = common_checks(r)
    states = states_of(r['status'])
    only_one = states in (['IDLE', 'TRACKING', 'IDLE'], ['TRACKING', 'IDLE'])
    checks.append(('IDLE, TRACKING, IDLE only', only_one, str(states)))
    before = np.mean([s[2] for s in window(r, 0.0, 0.9)])
    held = np.mean([s[2] for s in window(r, 3.5, 4.0)])
    checks.append(('the roll followed a 20 deg rotation of the hand (>= 0.2 rad)',
                   abs(held - before) >= 0.2, '%.3f rad' % (held - before)))
    projections = [(t, d.get('proj_deg')) for t, st, _, d in r['status'] if st == 'TRACKING']
    roll = [p for t, p in projections if 3.2 <= t <= 4.0 and p is not None]
    yaw = [p for t, p in projections if 8.0 <= t <= 8.5 and p is not None]
    # a rotation about the x axis of base_link is not a pure roll when the tool axis is 11 deg
    # off that axis: about 4 deg of it leaves the pan plane and is reported
    checks.append(('proj_deg is small while only the roll turns (< 6 deg)',
                   bool(roll) and max(roll) < 6.0, str(roll[-3:])))
    checks.append(('proj_deg reports the yaw that the arm cannot make (> 5 deg)',
                   bool(yaw) and max(yaw) > 5.0, str(yaw[-3:])))
    return checks


def check_gripper(r):
    checks = common_checks(r)
    amounts = [a for _, a in r['amounts']]
    checks.append(('the gripper topic: 1.0 (close) then 0.0 (open), nothing else',
                   amounts == [1.0, 0.0], str(amounts)))
    checks.append(('two goals to the gripper action (not for both buttons, not for the stops)',
                   len(r['goals']) == 2, str(len(r['goals']))))
    states = states_of(r['status'])
    checks.append(('the arm went TRACKING, HOLD_JUMP, TRACKING, STALE, TRACKING, IDLE',
                   is_subsequence(['TRACKING', 'HOLD_JUMP', 'TRACKING', 'STALE', 'TRACKING',
                                   'IDLE'], states), str(states)))
    closed = [s[4] for s in window(r, 2.2, 2.45) if s[4] is not None]
    opened = [s[4] for s in window(r, 3.8, 3.95) if s[4] is not None]
    checks.append(('the mock gripper closed (position < 0.3 rad)', bool(closed) and
                   np.mean(closed) < 0.3, str(np.mean(closed) if closed else None)))
    checks.append(('the mock gripper opened (position > 0.8 rad)', bool(opened) and
                   np.mean(opened) > 0.8, str(np.mean(opened) if opened else None)))
    later = [s[4] for s in window(r, 4.6, 12.0) if s[4] is not None]
    checks.append(('the gripper did not move during the stops (range < 0.02 rad)',
                   bool(later) and max(later) - min(later) < 0.02,
                   '%.3f' % (max(later) - min(later) if later else -1)))
    return checks


def check_busy(r):
    checks = common_checks(r)
    states = states_of(r['status'])
    checks.append(('TRACKING, BUSY, IDLE, TRACKING, IDLE',
                   is_subsequence(['TRACKING', 'BUSY', 'IDLE', 'TRACKING', 'IDLE'], states),
                   str(states)))
    checks.append(('the goal was sent', bool(r['busy_sent']), str(r['busy_sent'])))
    busy = [t for t in transitions(r['status']) if t[1] == 'BUSY']
    checks.append(('the BUSY reason names the active goal',
                   bool(busy) and 'active_goal' in busy[0][2], str(busy[:1])))
    after = [t for t, _ in r['points'] if busy and 3.0 <= busy[0][0] < t < 9.0]
    checks.append(('no point after BUSY until the new clutch', not after, str(after[:3])))
    return checks


CHECKS = {
    'line': check_line, 'jump': check_jump, 'stale': check_stale, 'range': check_range,
    'rotation': check_rotation, 'gripper': check_gripper, 'busy': check_busy,
}


# --------------------------------------------------------------------------------- main

# ------------------------------------------------------ the real arm: checks and the inputs

def tip_excursion(record):
    """Return the largest distance (m) of the tip from its first position (None: no samples)."""
    tips = [s[1] for s in record['samples']]
    if not tips:
        return None
    return max(float(np.linalg.norm(tip - tips[0])) for tip in tips)


def tracking_stats(record):
    """Return (max |measured - reference|, max |sent point - measured|) in rad (None: no data)."""
    refs, samples, points = record['refs'], record['samples'], record['points']
    error = advance = None
    if refs and samples:
        times = np.array([r[0] for r in refs])
        worst = 0.0
        for sample in samples:
            i = int(np.searchsorted(times, sample[0], side='right')) - 1
            if i >= 0:
                worst = max(worst, float(np.max(np.abs(sample[5] - refs[i][1]))))
        error = worst
    if points and samples:
        times = np.array([s[0] for s in samples])
        worst = 0.0
        for t, positions in points:
            i = int(np.searchsorted(times, t, side='right')) - 1
            if i >= 0:
                worst = max(worst, float(np.max(np.abs(np.array(positions) - samples[i][5]))))
        advance = worst
    return error, advance


def real_checks(name, record, expect, amplitude, cycles=1):
    """Return the checks of a scenario on the real arm (expect: normal, blocked, stationary)."""
    checks = common_checks(record)
    states = states_of(record['status'])
    transition_list = transitions(record['status'])
    excursion = tip_excursion(record)
    error, advance = tracking_stats(record)
    still = STATIONARY_MM / 1000.0

    def mm(value):
        return None if value is None else '%.1f mm' % (value * 1000.0)

    if expect == 'blocked':
        blocked = [t for t in transition_list if t[1] == 'BLOCKED']
        checks.append(('TRACKING, then BLOCKED (TRACKING_ERROR)',
                       is_subsequence(['TRACKING', 'BLOCKED'], states) and bool(blocked)
                       and blocked[0][2].startswith('TRACKING_ERROR'), str(transition_list)))
        checks.append(('the arm did not move (tip < %g mm)' % STATIONARY_MM,
                       excursion is not None and excursion < still, mm(excursion)))
    elif expect == 'stationary':
        only = states in (['IDLE', 'TRACKING', 'IDLE'], ['TRACKING', 'IDLE'])
        checks.append(('IDLE, TRACKING, IDLE only (no BLOCKED)', only, str(states)))
        checks.append(('the arm did not move (tip < %g mm)' % STATIONARY_MM,
                       excursion is not None and excursion < still, mm(excursion)))
    else:
        bad = [s for s in states if s in ('BLOCKED', 'HOLD_RANGE', 'HOLD_LAG', 'BUSY')]
        checks.append(('no BLOCKED, HOLD_RANGE, HOLD_LAG or BUSY', not bad, str(states)))
        if name == 'line':
            only = states in (['IDLE', 'TRACKING', 'IDLE'], ['TRACKING', 'IDLE'])
            checks.append(('IDLE, TRACKING, IDLE only', only, str(states)))
            checks.append(('the tip moved about the amplitude (0.4 to 1.6 times)',
                           excursion is not None
                           and 0.4 * amplitude <= excursion <= 1.6 * amplitude
                           + 0.002, '%s for %s' % (mm(excursion), mm(amplitude))))
        elif name == 'stale':
            checks.append(('TRACKING, STALE, TRACKING, IDLE',
                           is_subsequence(['TRACKING', 'STALE', 'TRACKING', 'IDLE'], states),
                           str(states)))
            reasons = {t[2] for t in transition_list if t[1] == 'STALE'}
            checks.append(('the STALE reason is POSE', reasons == {'POSE'}, str(reasons)))
        elif name == 'jump':
            checks.append(('TRACKING, HOLD_JUMP, TRACKING, IDLE',
                           is_subsequence(['TRACKING', 'HOLD_JUMP', 'TRACKING', 'IDLE'], states),
                           str(states)))
            jumps = [d.get('jump_mm', 0.0) for _, s, _, d in record['status'] if s == 'HOLD_JUMP']
            checks.append(('the jump was seen as 50 mm or more',
                           bool(jumps) and max(jumps) >= 50.0,
                           str(jumps[-1:])))
            checks.append(('the arm did not follow the jump (tip < %g mm)' % REAL_JUMP_MAX_TIP_MM,
                           excursion is not None and excursion < REAL_JUMP_MAX_TIP_MM / 1000.0,
                           mm(excursion)))
        elif name == 'gripper':
            amounts = [a for _, a in record['amounts']]
            checks.append(('the gripper topic: CLOSE 1.0 then OPEN 0.0, %d time(s)' % cycles,
                           amounts == [1.0, 0.0] * cycles, str(amounts)))
            checks.append(('%d goal(s) to the gripper action' % (2 * cycles),
                           len(record['goals']) == 2 * cycles, str(len(record['goals']))))
            checks.append(('the state stayed IDLE (no clutch)', set(states) <= {'IDLE'},
                           str(states)))
            checks.append(('the arm did not move (tip < 2 mm)',
                           excursion is not None and excursion < 0.002, mm(excursion)))
    checks.append(('INFO max abs(measured - reference)', True,
                   'n/a' if error is None else '%.4f rad' % error))
    checks.append(('INFO max abs(sent point - measured)', True,
                   'n/a' if advance is None else '%.4f rad' % advance))
    checks.append(('INFO tip excursion', True, str(mm(excursion))))
    if name == 'gripper':
        for c in range(cycles):
            for label, a, b in (('after CLOSE', 2.4 + 4.0 * c, 2.9 + 4.0 * c),
                                ('after OPEN', 4.4 + 4.0 * c, 4.9 + 4.0 * c)):
                values = [s[4] for s in window(record, a, b) if s[4] is not None]
                checks.append(('INFO gripper %s (cycle %d)' % (label, c + 1), True,
                               'n/a' if not values else '%.4f rad' % float(np.mean(values))))
    return checks


def call_get_parameters(monitor, client, names, timeout=5.0):
    """Return the values of the parameters of a node (None if the service does not answer)."""
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    request = GetParameters.Request()
    request.names = list(names)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(monitor, future, timeout_sec=timeout)
    return future.result().values if future.done() else None


def read_scale(monitor):
    """Return the scale of /vr_follow (None if it cannot be read)."""
    values = call_get_parameters(monitor, monitor.param_client, ['scale'])
    if not values or values[0].type != 3:  # PARAMETER_DOUBLE
        return None
    return values[0].double_value


def arm_is_read_only(monitor):
    """Return True if the robot_description says that the arm driver is read_only."""
    values = call_get_parameters(monitor, monitor.rsp_client, ['robot_description'])
    if not values:
        return False
    return description_is_read_only(values[0].string_value)


def confirm(text):
    """Show text and return True only if the user presses Enter."""
    print(text, flush=True)
    try:
        answer = input('Press Enter to start (n and Enter to stop here): ')
    except EOFError:
        return False
    return answer.strip().lower() not in ('n', 'no', 'q')


def wait_ready(monitor):
    wait_for(monitor, lambda: monitor.q is not None and monitor.status, 20.0,
             'no /joint_states or /vr_follow/status: are the arm and vr_follow running?')
    monitor.spin(0.5)


# --------------------------------------------------------------------------------- main

def _parse():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--mock', action='store_true', help='the mock arm (ROS_DOMAIN_ID=87)')
    parser.add_argument('--real-plans', action='store_true',
                        help='mock only: play the scenarios of --real (same limits, Enter)')
    parser.add_argument('--real', action='store_true',
                        help='the real arm: no preparation move, small limits, Enter to start')
    parser.add_argument('--out', required=True, help='directory for the timelines (CSV)')
    parser.add_argument('--scenarios', nargs='+', default=None,
                        help='mock: %s (default all); real: %s (required)' % (
                            ' '.join(SCENARIOS), ' '.join(REAL_SCENARIOS)))
    parser.add_argument('--buttons', type=int, nargs=3, default=[0, 1, 2],
                        metavar=('CLUTCH', 'OPEN', 'CLOSE'))
    parser.add_argument('--amplitude-mm', type=float, default=10.0,
                        help='real: amplitude of the TARGET of the arm in mm (30 at most)')
    parser.add_argument('--speed-mm-s', type=float, default=10.0,
                        help='real: speed of the target in mm/s (20 at most)')
    parser.add_argument('--gripper-cycles', type=int, default=1, help='real: CLOSE and OPEN')
    parser.add_argument('--arm-read-only', action='store_true',
                        help='real (c0): the arm is read_only (checked in the robot_description)')
    parser.add_argument('--expect', choices=EXPECTATIONS, default='normal',
                        help='real, read_only arm: blocked (C3 stops it) or stationary')
    args = parser.parse_args()
    reason = check_args(args, os.environ.get('ROS_DOMAIN_ID'))
    if reason:
        sys.exit('refused: %s' % reason)
    if args.mock:
        args.scenarios = args.scenarios or list(SCENARIOS)
        unknown = [s for s in args.scenarios if s not in SCENARIOS]
        if unknown:
            sys.exit('refused: unknown scenarios %s' % ' '.join(unknown))
    os.makedirs(args.out, exist_ok=True)
    return args


def write_csv(path, record):
    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['kind', 't', 'a', 'b', 'c'])
        for t, state, reason, detail in record['status']:
            writer.writerow(['status', '%.3f' % t, state, reason, detail.get('proj_deg', '')])
        for t, positions in record['points']:
            writer.writerow(['point', '%.3f' % t, ' '.join('%.4f' % p for p in positions), '', ''])
        for t, amount in record['amounts']:
            writer.writerow(['gripper_amount', '%.3f' % t, amount, '', ''])
        for t, goal in record['goals']:
            writer.writerow(['gripper_goal', '%.3f' % t, goal, '', ''])
        for t, reference in record['refs'][::2]:
            writer.writerow(['reference', '%.3f' % t, ' '.join('%.4f' % v for v in reference),
                             '', ''])
        for t, tip, q5, elbow, gripper, q in record['samples'][::5]:
            writer.writerow(['sample', '%.3f' % t, ' '.join('%.4f' % v for v in tip),
                             ' '.join('%.4f' % v for v in q),
                             '' if gripper is None else '%.4f' % gripper])


def describe(name, plan, scale, args):
    hand = args.amplitude_mm / scale
    return ('--- %s on the REAL arm (scale %.2f): target amplitude %g mm (hand %g mm), speed %g '
            'mm/s, %.1f s. Support the arm and keep a hand near the 12 V switch.' % (
                name, scale, args.amplitude_mm, hand, args.speed_mm_s, plan.duration))


def main():
    args = _parse()
    rclpy.init()
    monitor = Monitor(tuple(args.buttons), real=args.real)
    plans = {}
    if args.mock:  # only the mock arm is moved to the working pose (never the real arm)
        prepare(monitor)
    real_like = args.real or args.real_plans
    if real_like:
        wait_ready(monitor)
        scale = read_scale(monitor)
        if scale is None:
            sys.exit('refused: cannot read the scale of /vr_follow (is vr_follow running?)')
        if args.arm_read_only and not arm_is_read_only(monitor):
            sys.exit('refused: the robot_description does not say read_only')
        for name in args.scenarios:
            plans[name] = make_real_plan(name, args.amplitude_mm / 1000.0,
                                         args.speed_mm_s / 1000.0, scale, args.gripper_cycles)
    else:
        plans = make_plans()
    failures = 0
    for name in args.scenarios:
        print('=== %s' % name, flush=True)
        if real_like and not confirm(describe(name, plans[name], scale, args)):
            sys.exit('stopped before %s (nothing was sent)' % name)
        record = play(monitor, plans[name])
        write_csv(os.path.join(args.out, '%s.csv' % name), record)
        print('  transitions: %s' % ' > '.join(
            '%s%s@%.1f' % (s, '(%s)' % r if r not in ('OK', '-') else '', t)
            for t, s, r in transitions(record['status'])), flush=True)
        if real_like:
            checks = real_checks(name, record, args.expect, args.amplitude_mm / 1000.0,
                                 args.gripper_cycles)
        else:
            checks = CHECKS[name](record)
        for label, ok, text in checks:
            if label.startswith('INFO'):
                print('  INFO  %s: %s' % (label[5:], text), flush=True)
                continue
            failures += 0 if ok else 1
            print('  %s  %s%s' % ('PASS' if ok else 'FAIL', label,
                                  '' if ok else '  [%s]' % text), flush=True)
    print('BLOCKED (C3) seen: %s' % monitor.blocked_seen, flush=True)
    verdict = 'all checks passed' if failures == 0 else '%d check(s) FAILED' % failures
    print('RESULT: %s' % verdict)
    monitor.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
