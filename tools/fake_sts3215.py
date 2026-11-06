#!/usr/bin/env python3
"""STS3215 の偽サーボ（pty）。ドライバの結合確認用。実機・実ポートは開かない。

  python3 tools/fake_sts3215.py --calibration <yaml> --state-dir <dir>
  → 標準出力に pty スレーブのパスを 1 行出す。ドライバの serial_port にそのパスを渡す。

<dir>/positions.json  {"1": 2097, ...}  各 ID の Present_Position（毎回読み直す。無ければ中心/中点）
<dir>/mode            ok | silent | corrupt   応答の挙動（毎回読み直す）
--homing-offset ID=値 : その ID の EEPROM Homing_Offset を yaml と違う値にする（診断の MISMATCH 確認用）
対応する命令は PING(0x01)・READ(0x02)・SYNC_READ(0x82) のみ。受けた命令の回数は <dir>/instructions.json に書く。それ以外が来たら "UNEXPECTED INSTRUCTION" と標準エラーに出す
（ドライバが書き込み命令を送っていないことの確認）。
"""
import argparse
import json
import os
import pty
import sys
import time
import tty

import yaml


def chk(body):
    return (~sum(body)) & 0xFF


def sign_mag(v, bit):
    return ((1 << bit) | -v) if v < 0 else v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--calibration', required=True)
    ap.add_argument('--state-dir', required=True)
    ap.add_argument('--homing-offset', action='append', default=[])
    args = ap.parse_args()
    cal = yaml.safe_load(open(args.calibration))
    joints = {j['id']: j for j in cal['joints'].values()}
    override = {int(k): int(v) for k, v in (s.split('=') for s in args.homing_offset)}
    os.makedirs(args.state_dir, exist_ok=True)
    pos_file = os.path.join(args.state_dir, 'positions.json')
    mode_file = os.path.join(args.state_dir, 'mode')

    master, slave = pty.openpty()
    tty.setraw(master)
    print(os.ttyname(slave), flush=True)

    def mode():
        try:
            return open(mode_file).read().strip() or 'ok'
        except OSError:
            return 'ok'

    def regs(id_):
        j = joints[id_]
        try:
            pos = json.load(open(pos_file)).get(str(id_))
        except (OSError, ValueError):
            pos = None
        if pos is None:
            pos = int(j.get('center', (j['min'] + j['max']) / 2))
        e = j['eeprom']
        mem = {}

        def put(addr, val, n):
            for i in range(n):
                mem[addr + i] = (val >> (8 * i)) & 0xFF
        put(3, 777, 2)
        put(7, 0, 1)
        put(9, e['min_limit'], 2)
        put(11, e['max_limit'], 2)
        put(31, sign_mag(override.get(id_, e['homing_offset']), 11), 2)
        put(56, sign_mag(pos, 15), 2)
        return mem

    def respond(id_, addr, n):
        m = mode()
        if m == 'silent' or id_ not in joints:
            return b''
        mem = regs(id_)
        body = [id_, n + 2, 0] + [mem.get(addr + i, 0) for i in range(n)]
        c = chk(body)
        if m == 'corrupt':
            c ^= 0x55
        return bytes([0xFF, 0xFF] + body + [c])

    buf = bytearray()
    counts = {}
    while True:
        try:
            buf += os.read(master, 256)
        except OSError:
            time.sleep(0.01)
            continue
        while True:
            while len(buf) >= 2 and not (buf[0] == 0xFF and buf[1] == 0xFF):
                del buf[0]
            if len(buf) < 4 or len(buf) < buf[3] + 4:
                break
            pkt = bytes(buf[:buf[3] + 4])
            del buf[:buf[3] + 4]
            id_, inst = pkt[2], pkt[4]
            counts[inst] = counts.get(inst, 0) + 1
            with open(os.path.join(args.state_dir, 'instructions.json'), 'w') as f:
                json.dump({hex(k): v for k, v in counts.items()}, f)
            if inst == 0x01:  # PING: ERR=0 のステータス（LeRobot の ping 用）
                body = [id_, 2, 0]
                if id_ in joints and mode() == 'ok':
                    os.write(master, bytes([0xFF, 0xFF] + body + [chk(body)]))
            elif inst == 0x02:
                os.write(master, respond(id_, pkt[5], pkt[6]))
            elif inst == 0x82:
                for sid in pkt[7:-1]:
                    os.write(master, respond(sid, pkt[5], pkt[6]))
            else:
                print(f'UNEXPECTED INSTRUCTION 0x{inst:02X}', file=sys.stderr, flush=True)


if __name__ == '__main__':
    main()
