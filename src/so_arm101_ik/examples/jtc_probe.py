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
Measure how the arm_controller (JTC) follows a stream of one-point trajectories (P6 b0).

MOCK ONLY. It moves shoulder_pan at a constant speed by publishing one point per tick to
/arm_controller/joint_trajectory (the same message as the VR follow node) and records the
reference of the controller (/arm_controller/controller_state). It refuses to run unless
ROS_DOMAIN_ID is 87 (the mock domain) and --mock is given. It never opens a serial port.

For every (rate, time_from_start) pair it prints the speed that the controller really made, the
lag of its reference behind the target, and how many of its updates did not move.

  ROS_DOMAIN_ID=87 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 jtc_probe.py --mock --out p.csv
"""

import argparse
import csv
import os
import sys
import time

from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
TOPIC = '/arm_controller/joint_trajectory'
STATE = '/arm_controller/controller_state'
MOCK_DOMAIN = '87'
PAN_LIMIT = 1.8  # rad; the probe stays inside +-1.8 (the URDF limit is 1.92)


def _parse():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--mock', action='store_true', help='required: this is for the mock only')
    parser.add_argument('--rates', type=float, nargs='+', default=[20.0, 12.5, 10.0])
    parser.add_argument('--times', type=float, nargs='+', default=[0.05, 0.10, 0.12])
    parser.add_argument('--speed', type=float, default=0.4, help='rad/s of the target (<= 0.5)')
    parser.add_argument('--duration', type=float, default=3.0, help='s of one ramp')
    parser.add_argument('--out', required=True, help='CSV of the summary of every run')
    parser.add_argument('--raw', help='CSV of every state message of the reference')
    args = parser.parse_args()
    if not args.mock or os.environ.get('ROS_DOMAIN_ID') != MOCK_DOMAIN:
        sys.exit('refused: --mock and ROS_DOMAIN_ID=%s are required' % MOCK_DOMAIN)
    if not 0.0 < args.speed <= 0.5 or not 1.0 <= args.duration <= 5.0:
        sys.exit('refused: --speed must be in (0, 0.5], --duration in [1, 5]')
    if any(r <= 0 or r > 50 for r in args.rates) or any(t <= 0 or t > 1 for t in args.times):
        sys.exit('refused: rates in (0, 50] Hz, times in (0, 1] s')
    return args


class Probe(Node):
    def __init__(self):
        super().__init__('jtc_probe')
        self.pub = self.create_publisher(JointTrajectory, TOPIC, 10)
        self.create_subscription(JointTrajectoryControllerState, STATE, self._on_state, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.reference = None
        self.measured = None
        self.log = []  # (time, reference of the pan) of every state message

    def _on_state(self, msg):
        names = list(msg.joint_names)
        if all(j in names for j in JOINTS):
            self.reference = np.array([msg.reference.positions[names.index(j)] for j in JOINTS])
            self.log.append((time.monotonic(), float(self.reference[0])))

    def _on_joint_state(self, msg):
        names = list(msg.name)
        if all(j in names for j in JOINTS):
            self.measured = np.array([msg.position[names.index(j)] for j in JOINTS])

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def send(self, q, seconds):
        msg = JointTrajectory()
        msg.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        whole = int(seconds)
        point.time_from_start = Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))
        msg.points = [point]
        self.pub.publish(msg)


def _wait_for_state(node, timeout=20.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and (node.reference is None or node.measured is None):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.reference is None or node.measured is None:
        sys.exit('no controller_state / joint_states: is the mock bringup running?')


def run_one(node, rate, seconds, speed, duration, raw=None):
    """Send one ramp of the pan and return the metrics (the controller reference is measured)."""
    start = node.reference.copy()
    direction = 1.0 if start[0] < 0.5 and start[0] + speed * duration < PAN_LIMIT else -1.0
    if abs(start[0] + direction * speed * duration) > PAN_LIMIT:
        sys.exit('the pan is too near the limit for the probe')
    first = len(node.log)
    t0 = time.monotonic()
    period = 1.0 / rate
    tick = 0
    while True:
        elapsed = time.monotonic() - t0
        if elapsed >= duration:
            break
        q = start.copy()
        q[0] = start[0] + direction * speed * elapsed
        node.send(q, seconds)
        tick += 1
        wait = t0 + tick * period - time.monotonic()
        node.spin(max(0.0, wait))
    ramp = [(t - t0, p) for t, p in node.log[first:] if 0.5 <= t - t0 <= duration]
    if raw is not None:
        for t, p in node.log[first:]:
            raw.writerow([rate, seconds, '%.4f' % (t - t0), '%.6f' % p])
    node.spin(1.0)  # let the controller finish the last point
    if len(ramp) < 10:
        return None
    times = np.array([r[0] for r in ramp])
    ref = np.array([r[1] for r in ramp])
    slope, intercept = np.polyfit(times, ref, 1)
    target = start[0] + direction * speed * times
    lag = np.abs(target - ref)
    steps = np.diff(ref)
    return {
        'rate_hz': rate, 'time_from_start_s': seconds, 'target_speed': speed,
        'made_speed': abs(slope), 'ratio': abs(slope) / speed,
        'lag_mean_rad': float(np.mean(lag)), 'lag_max_rad': float(np.max(lag)),
        'still_fraction': float(np.mean(np.abs(steps) < 1e-7)),
        'step_std': float(np.std(steps)), 'step_max': float(np.max(np.abs(steps))),
        'residual_std': float(np.std(ref - (slope * times + intercept))),
        'updates': len(ramp), 'sent': tick,
    }


def main():
    args = _parse()
    rclpy.init()
    node = Probe()
    _wait_for_state(node)
    rows = []
    raw_file = open(args.raw, 'w', newline='') if args.raw else None
    raw = csv.writer(raw_file) if raw_file else None
    if raw:
        raw.writerow(['rate_hz', 'time_from_start_s', 't', 'reference_pan'])
    for rate in args.rates:
        for seconds in args.times:
            result = run_one(node, rate, seconds, args.speed, args.duration, raw)
            if result is None:
                print('rate %g time %g: no data' % (rate, seconds), flush=True)
                continue
            rows.append(result)
            print('rate %5.1f Hz  T %.2f s  made %.3f rad/s (%.0f%% of %.2f)  lag mean %.3f max '
                  '%.3f rad  still %.0f%%  step std %.4f max %.4f' % (
                      rate, seconds, result['made_speed'], 100 * result['ratio'], args.speed,
                      result['lag_mean_rad'], result['lag_max_rad'],
                      100 * result['still_fraction'], result['step_std'], result['step_max']),
                  flush=True)
    if rows:
        with open(args.out, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    if raw_file:
        raw_file.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
