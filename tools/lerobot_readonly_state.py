#!/usr/bin/env python3
"""【比較専用】LeRobot の bus だけを読み取り専用で開き、関節角を CSV に記録する（実機のポートを開く）。

  ~/miniforge3/envs/ros2_lerobot/bin/python tools/lerobot_readonly_state.py --seconds 5 --out lerobot_P2.csv

目的: bridge は接続時にトルクを ON にする（configure()）ため手で姿勢を作れない。このスクリプトは
  FeetechMotorsBus.connect()（ping とレジスタ読み出しのみ）と sync_read("Present_Position") だけを使い、
  configure() やトルク操作をしない。出力は record_joint_states.py と同じ CSV（rad）。
安全策:
  - bus の write / sync_write / enable_torque / disable_torque / write_calibration を例外にして、書き込みを不可能にしている。
  - 終了は disconnect(disable_torque=False)。既定の disconnect() はトルクを OFF にする書き込みをするので使わない。
  - 他のプロセス（bridge / feetech ドライバ）が同じポートを使っていないこと。
  - so_arm101 のドライバはこのスクリプトに依存しない（比較の基準を作るためだけに lerobot を使う）。
変換: LeRobot 0.4.2 の MotorsBus._normalize（Apache-2.0）→ bridge と同じ math.radians(値)。
"""
import argparse
import json
import math
import os
import time

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

JOINTS = [('shoulder_pan', 1, MotorNormMode.DEGREES), ('shoulder_lift', 2, MotorNormMode.DEGREES),
          ('elbow_flex', 3, MotorNormMode.DEGREES), ('wrist_flex', 4, MotorNormMode.DEGREES),
          ('wrist_roll', 5, MotorNormMode.DEGREES), ('gripper', 6, MotorNormMode.RANGE_0_100)]


def _forbidden(*_a, **_k):
    raise RuntimeError('このスクリプトはサーボに書き込まない')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--json', default='~/.cache/huggingface/lerobot/calibration/robots/so101_follower/so101_follower.json')
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--rate', type=float, default=30.0)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    cal = json.load(open(os.path.expanduser(args.json)))
    calibration = {n: MotorCalibration(id=c['id'], drive_mode=c['drive_mode'], homing_offset=c['homing_offset'],
                                       range_min=c['range_min'], range_max=c['range_max']) for n, c in cal.items()}
    bus = FeetechMotorsBus(port=args.port, motors={n: Motor(i, 'sts3215', m) for n, i, m in JOINTS},
                           calibration=calibration)
    for name in ('write', 'sync_write', 'enable_torque', 'disable_torque', 'write_calibration', 'configure_motors'):
        setattr(bus, name, _forbidden)
    bus.connect()  # ping + 読み出しのみ
    rows = []
    try:
        t0 = time.monotonic()
        period = 1.0 / args.rate
        while time.monotonic() - t0 < args.seconds:
            t = time.monotonic() - t0
            vals = bus.sync_read('Present_Position')
            rows.append([t] + [math.radians(vals[n]) for n, _, _ in JOINTS])
            time.sleep(max(0.0, period - (time.monotonic() - t0 - t)))
    finally:
        bus.disconnect(disable_torque=False)  # トルクに触れない
    with open(args.out, 'w') as f:
        f.write(f'# source=lerobot_readonly port={args.port} json={args.json} samples={len(rows)}\n')
        f.write('t,' + ','.join(n for n, _, _ in JOINTS) + '\n')
        for r in rows:
            f.write(','.join(repr(v) for v in r) + '\n')
    print(f'{len(rows)} samples -> {args.out}')


if __name__ == '__main__':
    main()
