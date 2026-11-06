#!/usr/bin/env python3
"""LeRobot の so101_follower.json から so_arm101 の自前キャリブレーション yaml を作る（一回だけ使う）。

- json を標準の json モジュールで読むだけ。lerobot は import しない。シリアルも開かない。
- 変換元は変更しない。
- yaml の形式は docs/inventory.md「段階2b-1」のとおり（schema_version 1）。
  段階2c では同じ形式の中身（center / min / max / eeprom）を差し替える。

意味:
  angle           : rad = direction * (raw - center) * 2*pi / angle_ticks   (clamp なし)
  linear_percent  : pct = (clamp(raw, min, max) - min) / (max - min) * 100  (direction=-1 なら 100-pct)
                    rad = radians(pct)
  eeprom.*        : 変換には使わない。起動時診断（サーボの EEPROM 実値との比較）専用。
                    homing_offset = json の homing_offset、min_limit/max_limit = json の range_min/range_max
                    （LeRobot の write_calibration が EEPROM に書く値）。
"""
import argparse
import datetime
import json
import os
import sys

JOINTS = [  # (名前, kind)  順序は json に依存しない
    ('shoulder_pan', 'angle'),
    ('shoulder_lift', 'angle'),
    ('elbow_flex', 'angle'),
    ('wrist_flex', 'angle'),
    ('wrist_roll', 'angle'),
    ('gripper', 'linear_percent'),
]
DEFAULT_SRC = '~/.cache/huggingface/lerobot/calibration/robots/so101_follower/so101_follower.json'


def fmt(x):
    return repr(float(x)) if isinstance(x, float) and not float(x).is_integer() else str(int(x))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--json', default=DEFAULT_SRC)
    ap.add_argument('--out', required=True)
    ap.add_argument('--force', action='store_true', help='既存の出力を上書きする')
    args = ap.parse_args()

    src = os.path.expanduser(args.json)
    if os.path.exists(args.out) and not args.force:
        sys.exit(f'{args.out} は既にある。上書きするなら --force')
    with open(src) as f:
        cal = json.load(f)
    file_date = datetime.datetime.fromtimestamp(os.path.getmtime(src)).strftime('%Y-%m-%d')
    today = datetime.date.today().isoformat()

    lines = [
        '# so_arm101 サーボキャリブレーション（フォロワー）。スキーマと意味は docs/inventory.md「段階2b-1」。',
        '# 段階2c では同じ形式で中身を差し替える。eeprom.* は変換に使わず、起動時診断専用。',
        'schema_version: 1',
        'source:',
        '  kind: converted_from_lerobot_json',
        f'  file: {args.json}',
        f'  file_date: {file_date}',
        f'  converted_at: {today}',
        '  converted_by: tools/convert_lerobot_calibration.py',
        'bus: {baudrate: 1000000, protocol: 0}',
        '# LeRobot 等価（分母 4095）。物理的には 4096 の可能性があり、段階2c で再検討する。',
        'conversion: {angle_ticks: 4095}',
        'joints:',
    ]
    for name, kind in JOINTS:
        c = cal[name]
        if c['drive_mode'] != 0:
            sys.exit(f'{name}: drive_mode={c["drive_mode"]} は未対応（direction への対応を確認してから）')
        mn, mx = c['range_min'], c['range_max']
        eeprom = f'eeprom: {{homing_offset: {c["homing_offset"]}, min_limit: {mn}, max_limit: {mx}}}'
        if kind == 'angle':
            center = (mn + mx) / 2
            lines.append(f'  {name}: {{id: {c["id"]}, kind: angle, direction: 1, center: {fmt(center)}, '
                         f'min: {mn}, max: {mx}, {eeprom}}}')
        else:
            lines.append(f'  {name}: {{id: {c["id"]}, kind: linear_percent, direction: 1, '
                         f'min: {mn}, max: {mx}, {eeprom}}}')
    with open(args.out, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'wrote {args.out} (source {src}, file date {file_date})')


if __name__ == '__main__':
    main()
