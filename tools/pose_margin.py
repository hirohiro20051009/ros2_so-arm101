#!/usr/bin/env python3
"""姿勢と校正範囲が、URDF の関節リミットまでどれだけ余裕があるかを表示する（段階2c の C3 で使う）。

オフライン。シリアルポートは開かない。URDF・SRDF は読むだけ（変更しない）。
  python3 tools/pose_margin.py --pose start=-0.0982,-1.4830,1.6901,0.5033,-0.0483,0.0152 \
      [--pose home=...] [--calibration follower_calibration.yaml] [--threshold 0.05]

--pose NAME=v1,...,v6 : 6 関節の値（rad）。順序は shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper。
  C3 で読み取り専用の起動をしたときの /joint_states の値を使う。
表示: 各関節で「URDF の下限・上限までの残り（rad）」。負なら範囲の外（OUT）、threshold 未満なら余裕が小さい（LOW）。
      校正で記録した範囲の両端（rad。yaml の min/max と center から換算）も、URDF の範囲と比べる。
式は so_arm101_hardware の raw_to_rad と同じ（angle: (raw − center)·360/angle_ticks 度、gripper: 0..100% を度として rad）。
"""
import argparse
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def urdf_limits(xacro_path):
    xml = subprocess.run(['xacro', xacro_path, 'use_fake_hardware:=true'], check=True, capture_output=True,
                         text=True).stdout
    lim = {}
    for j in ET.fromstring(xml).iter('joint'):
        n = j.get('name')
        l = j.find('limit')
        if n in JOINTS and l is not None and l.get('lower') is not None:
            lim[n] = (float(l.get('lower')), float(l.get('upper')))
    return lim


def raw_to_rad(j, angle_ticks, raw):
    if j['kind'] == 'angle':
        return math.radians((raw - j['center']) * 360.0 / angle_ticks)
    pct = (min(max(raw, j['min']), j['max']) - j['min']) / (j['max'] - j['min']) * 100.0
    return math.radians(pct)


def flag(margin, thr):
    return 'OUT' if margin < 0 else ('LOW' if margin < thr else 'ok')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', action='append', default=[], help='NAME=v1,...,v6 (rad)')
    ap.add_argument('--calibration', default=os.path.join(ROOT, 'src/so_arm101_hardware/config/follower_calibration.yaml'))
    ap.add_argument('--xacro', default=os.path.join(ROOT, 'src/so_arm101_moveit_config/config/so101_new_calib.urdf.xacro'))
    ap.add_argument('--threshold', type=float, default=0.05, help='余裕が小さいとみなす rad（報告のための仮の値）')
    args = ap.parse_args()
    lim = urdf_limits(args.xacro)
    cal = yaml.safe_load(open(args.calibration))
    ticks = cal['conversion']['angle_ticks']
    thr = args.threshold
    print(f'URDF の関節リミット（{args.xacro}）と校正（{args.calibration}）。LOW < {thr} rad、OUT < 0')
    print('\n== 校正で記録した範囲の両端と、URDF の範囲（rad）')
    print(f'{"joint":15s} {"calib_min":>10s} {"calib_max":>10s} {"urdf_lower":>11s} {"urdf_upper":>11s} '
          f'{"lower_margin":>13s} {"upper_margin":>13s}')
    for n in JOINTS:
        j = cal['joints'][n]
        lo, hi = raw_to_rad(j, ticks, j['min']), raw_to_rad(j, ticks, j['max'])
        ul, uu = lim[n]
        lm, um = lo - ul, uu - hi  # 校正の端が URDF の内側なら正（URDF の範囲の方が広い）
        print(f'{n:15s} {lo:10.4f} {hi:10.4f} {ul:11.4f} {uu:11.4f} {lm:+9.4f} {flag(lm, thr):>3s} {um:+9.4f} {flag(um, thr):>3s}')
    print('  （margin が負 = 校正の範囲が URDF の範囲より広い。サーボは URDF の外まで動ける）')
    worst = []
    for p in args.pose:
        name, vals = p.split('=')
        q = [float(v) for v in vals.split(',')]
        if len(q) != 6:
            sys.exit(f'--pose {name}: 6 つの値が必要')
        print(f'\n== 姿勢 {name}: URDF の下限・上限までの残り（rad）')
        print(f'{"joint":15s} {"q":>9s} {"to_lower":>9s} {"to_upper":>9s}  min_margin')
        for n, v in zip(JOINTS, q):
            ul, uu = lim[n]
            m = min(v - ul, uu - v)
            f = flag(m, thr)
            if f != 'ok':
                worst.append((name, n, m, f))
            print(f'{n:15s} {v:+9.4f} {v - ul:+9.4f} {uu - v:+9.4f}  {m:+9.4f} {f}')
    if worst:
        print('\n要報告（範囲の外、または余裕が小さい）:')
        for name, n, m, f in worst:
            print(f'  {name}: {n} 残り {m:+.4f} rad [{f}]')


if __name__ == '__main__':
    main()
