#!/usr/bin/env python3
"""/joint_states を指定秒数記録して CSV に保存する（rclpy）。

  python3 tools/record_joint_states.py --seconds 5 --out lerobot_P1.csv

CSV: 1 行目はコメント(#)、次にヘッダ `t,<joint>...`、以降 1 メッセージ 1 行（t は記録開始からの秒、値は rad）。
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Recorder(Node):
    def __init__(self, topic):
        super().__init__('record_joint_states')
        self.rows = []
        self.names = None
        self.t0 = None
        self.create_subscription(JointState, topic, self.cb, 50)

    def cb(self, msg):
        now = time.monotonic()
        if self.names is None:
            self.names = list(msg.name)
            self.t0 = now
        pos = dict(zip(msg.name, msg.position))
        self.rows.append([now - self.t0] + [pos.get(n, float('nan')) for n in self.names])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--out', required=True)
    ap.add_argument('--topic', default='/joint_states')
    ap.add_argument('--wait', type=float, default=10.0, help='最初のメッセージを待つ最大秒数')
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.topic)
    t_wait = time.monotonic() + args.wait
    while rclpy.ok() and node.names is None and time.monotonic() < t_wait:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.names is None:
        sys.exit(f'{args.topic} のメッセージが来ない')
    t_end = time.monotonic() + args.seconds
    while rclpy.ok() and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    with open(args.out, 'w') as f:
        f.write(f'# topic={args.topic} seconds={args.seconds} samples={len(node.rows)} recorded_at={time.strftime("%Y-%m-%dT%H:%M:%S")}\n')
        f.write('t,' + ','.join(node.names) + '\n')
        for r in node.rows:
            f.write(','.join(repr(v) for v in r) + '\n')
    print(f'{len(node.rows)} samples -> {args.out}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
