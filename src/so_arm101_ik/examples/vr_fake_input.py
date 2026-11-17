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

MOCK ONLY. It refuses to run unless ROS_DOMAIN_ID is 87 (the mock domain) and --mock is given.
Start the mock arm first (ros2 launch so_arm101_bringup bringup.launch.py mock:=true
use_rviz:=false), then ros2 launch so_arm101_ik vr_follow.launch.py, then this script.

It moves the mock arm to a working pose (a trajectory to the arm_controller that does not go
through the follow node), plays the scenarios line, jump, stale, range, rotation, gripper and
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
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik import follow_calc as fc
from so_arm101_ik.urdf_kinematics import rpy_matrix
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


# ---------------------------------------------------------------------------- the monitor

class Monitor(Node):
    """Publishes the fake inputs and records the state, the points and the gripper."""

    def __init__(self, buttons):
        super().__init__('vr_fake_input')
        self.buttons = buttons  # (clutch, open, close) indices
        self.joy_pub = self.create_publisher(Joy, '/vr/right/joy', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vr/right/pose', 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory',
                                              10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/vr_follow/status', self._on_status, latched)
        self.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory',
                                 self._on_point, 100)
        self.create_subscription(Float64, '/vr_follow/gripper', self._on_gripper_amount, latched)
        self.create_subscription(GoalStatusArray, GRIPPER_STATUS, self._on_gripper_status, latched)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 50)
        self.jtc_client = ActionClient(self, FollowJointTrajectory, JTC_ACTION)
        self.started = time.monotonic()
        self.status = []  # (t, state, reason, detail)
        self.points = []  # (t, positions)
        self.amounts = []  # (t, amount)
        self.goal_ids = set()
        self.goal_times = []  # (t, id) of the new gripper goals
        self.q = None
        self.gripper = None
        self.samples = []  # (t, tip xyz, q5, elbow, gripper)
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
        self.samples.append((time.monotonic(), tip, self.q[4], self.q[2], self.gripper))

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
             len(monitor.goal_times), len(monitor.samples))
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
        'samples': rel(monitor.samples[marks[4]:]), 'busy_sent': busy_sent,
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

def _parse():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--mock', action='store_true', help='required: this is for the mock only')
    parser.add_argument('--out', required=True, help='directory for the timelines (CSV)')
    parser.add_argument('--scenarios', nargs='+', default=list(SCENARIOS), choices=SCENARIOS)
    parser.add_argument('--buttons', type=int, nargs=3, default=[0, 1, 2],
                        metavar=('CLUTCH', 'OPEN', 'CLOSE'))
    args = parser.parse_args()
    if not args.mock or os.environ.get('ROS_DOMAIN_ID') != MOCK_DOMAIN:
        sys.exit('refused: --mock and ROS_DOMAIN_ID=%s are required' % MOCK_DOMAIN)
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
        for t, tip, q5, elbow, gripper in record['samples'][::5]:
            writer.writerow(['sample', '%.3f' % t, ' '.join('%.4f' % v for v in tip),
                             '%.4f %.4f' % (q5, elbow),
                             '' if gripper is None else '%.4f' % gripper])


def main():
    args = _parse()
    rclpy.init()
    monitor = Monitor(tuple(args.buttons))
    prepare(monitor)
    plans = make_plans()
    failures = 0
    for name in args.scenarios:
        print('=== %s' % name, flush=True)
        record = play(monitor, plans[name])
        write_csv(os.path.join(args.out, '%s.csv' % name), record)
        print('  transitions: %s' % ' > '.join(
            '%s%s@%.1f' % (s, '(%s)' % r if r not in ('OK', '-') else '', t)
            for t, s, r in transitions(record['status'])), flush=True)
        for label, ok, text in CHECKS[name](record):
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
