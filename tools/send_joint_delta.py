#!/usr/bin/env python3
"""1 つの関節だけを、今の位置から delta [rad] だけ、ゆっくり動かす（docs/feetech_write_check.md の W1〜W3 用）。

  python3 tools/send_joint_delta.py wrist_roll 0.2 --seconds 4
  python3 tools/send_joint_delta.py gripper 0.3 --seconds 3

【実機を動かす。ユーザーだけが実行する】腕は /joint_states の現在位置から始まり、他の関節は現在位置のまま。
安全のため: |delta| は既定で 0.5 rad まで、速度（|delta|/seconds）は 0.1 rad/s まで。超える指定は拒否する
（--allow-fast で速度だけ緩められる。0.3 rad/s が上限）。送る前に目標を表示して、Enter を待つ。
arm の 5 関節は /arm_controller/follow_joint_trajectory、gripper は /gripper_controller/gripper_cmd。
"""
import argparse
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
GRIPPER = 'gripper'
MAX_DELTA = 0.5
MAX_SPEED = 0.1
MAX_SPEED_FAST = 0.3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('joint', choices=ARM + [GRIPPER])
    ap.add_argument('delta', type=float, help='現在位置からの相対 [rad]')
    ap.add_argument('--seconds', type=float, required=True, help='かける時間 [s]')
    ap.add_argument('--allow-fast', action='store_true', help='速度の上限を 0.1 から 0.3 rad/s に緩める')
    ap.add_argument('--yes', action='store_true', help='Enter の確認を省く')
    args = ap.parse_args()

    limit = MAX_SPEED_FAST if args.allow_fast else MAX_SPEED
    if abs(args.delta) > MAX_DELTA:
        sys.exit(f'refused: |delta| {abs(args.delta):.3f} rad > {MAX_DELTA} rad')
    if args.seconds <= 0 or abs(args.delta) / args.seconds > limit:
        sys.exit(f'refused: speed {abs(args.delta) / max(args.seconds, 1e-9):.3f} rad/s > {limit} rad/s')

    rclpy.init()
    node = Node('send_joint_delta')
    state = {}
    node.create_subscription(JointState, '/joint_states', lambda m: state.update(zip(m.name, m.position)), 10)
    end = time.time() + 5
    while rclpy.ok() and time.time() < end and not all(j in state for j in ARM + [GRIPPER]):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not all(j in state for j in ARM + [GRIPPER]):
        sys.exit('no /joint_states (is the launch running?)')
    if any(v != v for v in state.values()):
        sys.exit('/joint_states contains NaN: the driver is not healthy. Not sending anything.')

    cur = state[args.joint]
    target = cur + args.delta
    print(f'{args.joint}: {cur:+.4f} -> {target:+.4f} rad over {args.seconds:.1f} s '
          f'({abs(args.delta) / args.seconds:.3f} rad/s)')
    if not args.yes:
        input('Support the arm and keep a hand near the servo power. Press Enter to send (Ctrl-C to cancel) ')

    if args.joint == GRIPPER:
        client = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')
        if not client.wait_for_server(timeout_sec=5):
            sys.exit('gripper action server not available')
        goal = GripperCommand.Goal()
        goal.command.position = float(target)
        goal.command.max_effort = 0.0
        fut = client.send_goal_async(goal)
    else:
        client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=5):
            sys.exit('arm action server not available')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint()
        pt.positions = [float(target if j == args.joint else state[j]) for j in ARM]
        pt.time_from_start.sec = int(args.seconds)
        pt.time_from_start.nanosec = int((args.seconds - int(args.seconds)) * 1e9)
        goal.trajectory.points = [pt]
        fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    handle = fut.result()
    if not handle.accepted:
        sys.exit('goal rejected')
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res)
    print('result status:', res.result().status)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
