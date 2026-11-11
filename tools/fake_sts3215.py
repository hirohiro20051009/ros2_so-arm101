#!/usr/bin/env python3
"""STS3215 の偽サーボ（pty）。ドライバの結合確認用。実機・実ポートは開かない。

  python3 tools/fake_sts3215.py --calibration <yaml> --state-dir <dir>
  → 標準出力に pty スレーブのパスを 1 行出す。ドライバの serial_port にそのパスを渡す。

<dir>/positions.json  {"1": 2097, ...}  各 ID の Present_Position（毎回読み直す。無ければ中心/中点）
<dir>/mode            ok | silent | corrupt   応答の挙動（毎回読み直す）
--reg ID:ADDR:LEN=値 : 任意のレジスタを上書き（診断の設定比較の確認用。例: --reg 2:21:1=32 は ID 2 の P_Coefficient を 32 に）
--homing-offset ID=値 : その ID の EEPROM Homing_Offset を yaml と違う値にする（診断の MISMATCH 確認用）
対応する命令は PING(0x01)・READ(0x02)・SYNC_READ(0x82)・WRITE(0x03)・SYNC_WRITE(0x83)。受けた命令の回数は <dir>/instructions.json に、
書き込み（命令・ID・アドレス・データ）は <dir>/writes.jsonl に順番に書く。それ以外の命令が来たら "UNEXPECTED INSTRUCTION" と標準エラーに出す。
--strict-readonly を付けると、書き込み系（0x03 / 0x83）も "UNEXPECTED INSTRUCTION" にして**書かない**（read_only:=true で
ドライバが書き込み命令を送っていないことの確認）。
サーボは電源投入直後の状態から始める（段階2b-3a の実機値）: Acceleration=0、Lock=1、Torque_Enable=0、Goal_Position=0、
Torque_Limit=Max_Torque_Limit。EEPROM（アドレス < 40）への書き込みは Lock=1 のとき無視する。
<dir>/reset というファイルを作ると、次の要求の前にサーボが電源を入れ直した状態（SRAM が上の値）に戻り、ファイルは消える。
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
    ap.add_argument('--reg', action='append', default=[])
    ap.add_argument('--strict-readonly', action='store_true')
    args = ap.parse_args()
    cal = yaml.safe_load(open(args.calibration))
    joints = {j['id']: j for j in cal['joints'].values()}
    override = {int(k): int(v) for k, v in (s.split('=') for s in args.homing_offset)}
    reg_override = {}
    for spec in args.reg:
        key, val = spec.split('=')
        id_, addr, length = (int(x) for x in key.split(':'))
        reg_override[(id_, addr)] = (int(val), length)
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

    store = {}

    def sram_defaults(mem):
        def put(addr, val, n):
            for i in range(n):
                mem[addr + i] = (val >> (8 * i)) & 0xFF
        put(40, 0, 1)    # Torque_Enable
        put(41, 0, 1)    # Acceleration（電源投入時は 0）
        put(42, 0, 2)    # Goal_Position（不明。わざと 0）
        put(46, 0, 2)    # Goal_Velocity
        put(48, mem[16] | (mem[17] << 8), 2)  # Torque_Limit = Max_Torque_Limit
        put(55, 1, 1)    # Lock

    def init_mem(id_):
        j = joints[id_]
        e = j['eeprom']
        mem = {}

        def put(addr, val, n):
            for i in range(n):
                mem[addr + i] = (val >> (8 * i)) & 0xFF
        put(3, 777, 2)
        put(5, id_, 1)   # ID
        put(7, 0, 1)
        put(9, e['min_limit'], 2)
        put(11, e['max_limit'], 2)
        put(31, sign_mag(override.get(id_, e['homing_offset']), 11), 2)
        # LeRobot の configure() 後の設定。gripper だけトルク系を絞る
        is_gripper = j['kind'] == 'linear_percent'
        put(16, 500 if is_gripper else 1000, 2)   # Max_Torque_Limit
        put(21, 16, 1)                            # P
        put(22, 32, 1)                            # D
        put(23, 0, 1)                             # I
        put(28, 250 if is_gripper else 500, 2)    # Protection_Current
        put(33, 0, 1)                             # Operating_Mode
        put(36, 25 if is_gripper else 80, 1)      # Overload_Torque
        put(62, 119, 1)                           # 11.9 V
        put(63, 30, 1)                            # 30 C
        put(65, 0, 1)                             # Status
        put(85, 254, 1)                           # Maximum_Acceleration
        for (rid, addr), (val, length) in reg_override.items():
            if rid == id_:
                put(addr, val, length)
        sram_defaults(mem)
        return mem

    reset_file = os.path.join(args.state_dir, 'reset')
    writes_file = os.path.join(args.state_dir, 'writes.jsonl')
    open(writes_file, 'w').close()

    def regs(id_):
        if os.path.exists(reset_file):
            for m in store.values():
                sram_defaults(m)
            try:
                os.remove(reset_file)
            except OSError:
                pass
        if id_ not in store:
            store[id_] = init_mem(id_)
        j = joints[id_]
        try:
            pos = json.load(open(pos_file)).get(str(id_))
        except (OSError, ValueError):
            pos = None
        if pos is None:
            pos = int(j.get('center', (j['min'] + j['max']) / 2))
        mem = store[id_]
        enc = sign_mag(pos, 15)
        mem[56] = enc & 0xFF
        mem[57] = (enc >> 8) & 0xFF
        return mem

    def do_write(inst, id_, addr, data):
        mem = regs(id_)
        with open(writes_file, 'a') as f:
            f.write(json.dumps({'inst': inst, 'id': id_, 'addr': addr, 'data': list(data)}) + '\n')
        locked = mem[55] == 1
        for i, b in enumerate(data):
            a = addr + i
            if a < 40 and locked:
                continue
            mem[a] = b

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
            elif inst in (0x03, 0x83) and not args.strict_readonly:
                if inst == 0x03:
                    if id_ in joints:
                        do_write(inst, id_, pkt[5], pkt[6:-1])
                        if mode() == 'ok':
                            body = [id_, 2, 0]
                            os.write(master, bytes([0xFF, 0xFF] + body + [chk(body)]))
                else:
                    addr, ln = pkt[5], pkt[6]
                    i = 7
                    while i + ln < len(pkt):
                        if pkt[i] in joints:
                            do_write(inst, pkt[i], addr, pkt[i + 1:i + 1 + ln])
                        i += ln + 1
            else:
                print(f'UNEXPECTED INSTRUCTION 0x{inst:02X}', file=sys.stderr, flush=True)


if __name__ == '__main__':
    main()
