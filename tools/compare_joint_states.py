#!/usr/bin/env python3
"""record_joint_states.py などで作った 2 つの CSV を関節ごとに比較する。

  python3 tools/compare_joint_states.py lerobot_P1.csv feetech_P1.csv

tick 換算（キャリブレーション yaml から）:
  angle 関節     : 1 tick = 2π / angle_ticks rad（angle_ticks=4095）
  gripper(linear): 1 tick = radians(100 / (max − min)) rad（max/min は yaml の range_max/range_min）
指標（関節ごと）:
  平均差 = |mean(A) − mean(B)|
  最大差 = max(|min(A) − max(B)|, |max(A) − min(B)|)   （A と B の全サンプルの組合せで最悪の差）
  標準偏差 = A, B それぞれ（静止しているかの確認用）
合格基準: 平均差 <= 1 tick かつ 最大差 <= 2 tick（--mean-tol / --max-tol で変更可）。
標準偏差が 1 tick を超えるときは「静止していない可能性」と警告する（合否には含めない）。
NaN を含む関節は不合格。終了コード: 全関節合格なら 0、それ以外は 1。
"""
import argparse
import math
import os
import statistics
import sys

import yaml

DEFAULT_CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'so_arm101_hardware',
                           'config', 'follower_calibration.yaml')


def read_csv(path):
    names, cols = None, None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if names is None:
                names = parts[1:]
                cols = {n: [] for n in names}
                continue
            for n, v in zip(names, parts[1:]):
                cols[n].append(float(v))
    if not names:
        sys.exit(f'{path}: データが無い')
    return cols


def tick_rad(joint, cal):
    if joint['kind'] == 'angle':
        return 2.0 * math.pi / cal['conversion']['angle_ticks']
    return math.radians(100.0 / (joint['max'] - joint['min']))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('a')
    ap.add_argument('b')
    ap.add_argument('--calibration', default=DEFAULT_CAL)
    ap.add_argument('--mean-tol', type=float, default=1.0, help='平均差の許容 [tick]')
    ap.add_argument('--max-tol', type=float, default=2.0, help='最大差の許容 [tick]')
    args = ap.parse_args()

    cal = yaml.safe_load(open(args.calibration))
    A, B = read_csv(args.a), read_csv(args.b)
    joints = [n for n in cal['joints'] if n in A and n in B]
    missing = [n for n in cal['joints'] if n not in joints]
    if missing:
        sys.exit(f'CSV に無い関節: {missing}')
    print(f'A = {args.a} ({len(next(iter(A.values())))} samples)')
    print(f'B = {args.b} ({len(next(iter(B.values())))} samples)')
    print(f'合格基準: 平均差 <= {args.mean_tol} tick, 最大差 <= {args.max_tol} tick\n')
    hdr = (f'{"joint":14s} {"mean A [rad]":>13s} {"mean B [rad]":>13s} {"mean diff":>16s} {"max diff":>16s} '
           f'{"std A":>10s} {"std B":>10s}  result')
    print(hdr)
    print(f'{"":14s} {"":>13s} {"":>13s} {"rad (tick)":>16s} {"rad (tick)":>16s} {"tick":>10s} {"tick":>10s}')
    all_ok = True
    notes = []
    for n in joints:
        a, b = A[n], B[n]
        tk = tick_rad(cal['joints'][n], cal)
        if any(math.isnan(x) for x in a + b):
            print(f'{n:14s}  NaN を含む  FAIL')
            all_ok = False
            continue
        ma, mb = statistics.fmean(a), statistics.fmean(b)
        sa = statistics.pstdev(a) / tk
        sb = statistics.pstdev(b) / tk
        mean_diff = abs(ma - mb)
        max_diff = max(abs(min(a) - max(b)), abs(max(a) - min(b)))
        eps = 1e-9  # 丁度 2 tick が浮動小数の丸めで不合格にならないように
        ok = mean_diff / tk <= args.mean_tol + eps and max_diff / tk <= args.max_tol + eps
        all_ok &= ok
        print(f'{n:14s} {ma:13.6f} {mb:13.6f} {mean_diff:8.5f} ({mean_diff / tk:5.2f}) {max_diff:8.5f} ({max_diff / tk:5.2f}) '
              f'{sa:10.2f} {sb:10.2f}  {"PASS" if ok else "FAIL"}')
        if max(sa, sb) > 1.0:
            notes.append(f'{n}: 標準偏差が 1 tick を超える（A={sa:.2f}, B={sb:.2f}）→ 静止していない可能性')
    for s in notes:
        print('注意:', s)
    print('\n総合:', 'PASS' if all_ok else 'FAIL')
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
