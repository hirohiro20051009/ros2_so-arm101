#!/usr/bin/env python3
"""【比較専用】LeRobot の bus だけを読み取り専用で開き、関節角を CSV に記録する（実機のポートを開く）。

  ~/miniforge3/envs/ros2_lerobot/bin/python tools/lerobot_readonly_state.py --seconds 5 --out lerobot_P2.csv

目的: bridge は接続時にトルクを ON にする（configure()）ため手で姿勢を作れない。このスクリプトは
  FeetechMotorsBus.connect()（ping とレジスタ読み出しのみ）と sync_read("Present_Position") だけを使い、
  configure() やトルク操作をしない。出力は record_joint_states.py と同じ CSV（rad）。
安全策（二重）:
  1. 送信バイト列の検査（最終防衛線）: port_handler.writePort をラップし、送信パケットの命令コードが
     PING(0x01) / READ(0x02) / SYNC_READ(0x82) 以外なら例外にする。どの経路（公開メソッド、_write、_sync_write、
     _disable_torque、SDK 直呼び）でも、書き込み命令は 1 バイトもシリアルに出ない。
  2. LeRobot の公開メソッド write / sync_write / enable_torque / disable_torque / write_calibration /
     configure_motors を例外にする（呼んだ箇所が分かるように早く失敗させる）。
  - 終了は disconnect(disable_torque=False)。既定の disconnect() はトルクを OFF にする書き込みをするので使わない
    （上の 2 により、既定のまま呼んでも例外になり書き込まれない）。
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


READ_ONLY_INSTRUCTIONS = {0x01: 'PING', 0x02: 'READ', 0x82: 'SYNC_READ'}


def _forbidden(*_a, **_k):
    raise RuntimeError('このスクリプトはサーボに書き込まない')


def install_write_guards(bus):
    """bus を読み取り専用にする。bus.connect() より前に呼ぶこと。"""
    # 1. 送信バイト列の検査: FF FF <ID> <LEN> <INST> ... の INST が読み取り系でなければ送らない
    port = bus.port_handler
    original_write_port = port.writePort

    def guarded_write_port(packet):
        inst = packet[4] if len(packet) > 4 else None
        if len(packet) < 6 or packet[0] != 0xFF or packet[1] != 0xFF or inst not in READ_ONLY_INSTRUCTIONS:
            port.is_using = False  # SDK の txPacket が立てたフラグを戻す（以降の読み取りを妨げない）
            raise RuntimeError(f'書き込み/不明な命令を送ろうとした: {bytes(packet).hex()} (INST={inst})')
        return original_write_port(packet)

    port.writePort = guarded_write_port
    # 2. 公開メソッドを例外にする
    for name in ('write', 'sync_write', 'enable_torque', 'disable_torque', 'write_calibration', 'configure_motors'):
        setattr(bus, name, _forbidden)


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
    install_write_guards(bus)
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
