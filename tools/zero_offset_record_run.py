#!/usr/bin/env python3
"""オフセットキャリブ（zero-offset calibration）の記録: 先端を机に当てた姿勢の関節角を CSV に追記する。

センターキャリブ（so_arm101_calib measure・apply）ではない。読み取り専用で、購読するのは
/joint_states だけ。腕・サーボ・カメラには何も送らず、シリアルポートも開かない。
腕はトルク OFF の読み取り専用の起動（ユーザーが実施。docs/zero_offset_calib_plan.md 7.5）で、
手で動かして先端を机にそっと当てる。

  python3 tools/zero_offset_record_run.py --csv ~/so_arm101_check/zero_offset_NEW/zero.csv \
      --expected-offset elbow_flex=-6.3

目標の姿勢（P1〜P4）は実物（真の角度）の傾きと x。yaml に δ がまだ無い間は、--expected-offset で
「実物 = 読んだ角度 + オフセット」と見込んだ値で案内する（表示・目標の判定・45° の下限だけ）。
記録する角度と fit には使わない。yaml に δ を入れた後は渡さない。

端末: Enter で今の姿勢を記録（静止していて、仰角 45° 以上のときだけ）。P1〜P4 で目標の姿勢を選ぶ。
y で、目標の範囲の外の記録を確認する。q で終了。CSV と meta を書くのは
so_arm101_calibration.zero_offset_record の write_point だけで、この道具は open を使わない。
終了コード: 0 正常、2 拒否（起動の検査）、130 Ctrl-C。
"""

import argparse
import datetime
import os
from pathlib import Path
import select
import sys
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from so_arm101_calibration import zero_offset as zo
from so_arm101_calibration import zero_offset_record as zr
from so_arm101_ik.urdf_kinematics import UrdfChain

JOINT_STATES = '/joint_states'
DISPLAY_SECONDS = 0.2  # the display is updated about 5 times a second
WINDOW_SECONDS = 1.0  # the samples of the last second are averaged (up to zr.SAMPLES)
MIN_SAMPLES = 5  # fewer samples than this are not a record (推測)
FRESH_SECONDS = 1.0  # the newest sample may be this old (推測)
EXIT_REFUSED = zo.EXIT_REJECTED
EXIT_INTERRUPTED = 130


class RunNode(Node):
    """Receives /joint_states; nothing is published."""

    def __init__(self):
        super().__init__('zero_offset_record_run')
        self._samples = []
        self.create_subscription(JointState, JOINT_STATES, self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        table = dict(zip(msg.name, msg.position))
        try:
            values = tuple(float(table[name]) for name in zr.STATE_JOINTS)
        except (KeyError, ValueError):
            return
        now = time.monotonic()
        self._samples.append((now, values))
        self._samples = [s for s in self._samples if now - s[0] <= 5.0]

    def spin(self, seconds):
        rclpy.spin_once(self, timeout_sec=seconds)

    def recent(self, seconds=WINDOW_SECONDS, limit=zr.SAMPLES):
        """Return the newest samples of the last seconds (at most limit), as an array (n, 6)."""
        now = time.monotonic()
        taken = [s[1] for s in self._samples if now - s[0] <= seconds][-limit:]
        return np.array(taken, dtype=float).reshape(-1, len(zr.STATE_JOINTS))

    def newest_age(self):
        return None if not self._samples else time.monotonic() - self._samples[-1][0]


def status_line(label, node, chain, jaw, options, offsets=None):
    """Return the one-line display of the present state against the target of the label."""
    samples = node.recent()
    if len(samples) == 0:
        return f'{label}: no /joint_states yet'
    mean, span = zr.window_stats(samples)
    arm = mean[:len(zr.ARM_JOINTS)]
    _, eps_read = zo.tool_state(chain, arm)
    x_mm, eps = zo.tool_state(chain, zr.expected_state(arm, offsets))
    dx, de, inside = zr.target_delta(label.split('#')[0], x_mm, eps)
    height = zo.height_above_desk_mm(chain, jaw, arm, options)
    still = 'still' if zr.is_still(span) else 'MOVING'
    return (f'{label}: x {x_mm:6.1f} mm ({dx:+6.1f})  elevation {eps:5.1f} deg ({de:+5.1f}) '
            f'[read {eps_read:5.1f}]  {"in range" if inside else "OUT of range"}  '
            f'moving {np.max(span[:5]):4.2f} deg {still}  model height {height:+6.1f} mm')


def provisional_fit(csv_path, chain, jaw, identity, options):
    """Return the lines of the display-only fit of the records so far (nothing is written)."""
    touches = zr.read_touches(csv_path)
    if len(touches) < 6:
        return []
    solution = zo.fit(chain, jaw, touches, identity['zero_offset_deg'], options)
    return ['provisional (display only, not a result): ' + ', '.join(
        f'{j} {solution.delta_deg[i]:+.2f} (sd {solution.sd_deg[i]:.2f}) deg'
        for i, j in enumerate(zo.FIT_JOINTS)) + f'; rms {solution.rms_mm:.2f} mm']


def record(node, label, args, chain, jaw, options, identity, state):
    """Record the present state as label.  Return True when a row was written."""
    samples = node.recent()
    age = node.newest_age()
    if len(samples) < MIN_SAMPLES or age is None or age > FRESH_SECONDS:
        print('\nnot recorded: no fresh /joint_states')
        return False
    mean, span = zr.window_stats(samples)
    try:
        row = zr.make_row(label, time.time(), mean, span, len(samples), chain, jaw, options,
                          args.expected_offsets)
        dx, de, inside = zr.target_delta(row['pose'], float(row['x_expected_mm']),
                                         float(row['eps_expected_deg']))
        if not inside and state.get('confirmed') != label:
            state['ask'] = label
            print(f'\n{label}: outside the target (dx {dx:+.1f} mm, d-elevation {de:+.1f} deg). '
                  'Type y and Enter to record it anyway, or move the arm.')
            return False
        meta = zr.build_meta(identity, options, datetime.datetime.now(
            datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            expected_offset_deg=args.expected_offsets)
        idx = zr.write_point(args.csv, label, identity, meta, row)
    except zr.SessionError as error:
        print(f'\nnot recorded: {error}')
        return False
    print(f'\nrecorded {label} (row {idx}): x {row["x_expected_mm"]} mm, elevation '
          f'{row["eps_expected_deg"]} deg (read {row["eps_deg"]}), '
          f'model height {row["model_height_mm"]} mm {row["warn"]}')
    for line in provisional_fit(args.csv, chain, jaw, identity, options):
        print(line)
    return True


def loop(node, args, chain, jaw, options, identity):
    """Show the state and record on Enter until q.  Nothing is sent to the arm."""
    existing = list(zr.existing_labels(args.csv))
    label = zr.next_label(existing)
    if args.target:
        label = next((f'{args.target}#{n}' for n in range(1, zr.REPEATS + 1)
                      if f'{args.target}#{n}' not in existing), label)
    state = {}
    buffer = ''
    last = 0.0
    print('Enter = record, P1..P4 = choose the target, y = confirm, q = quit')
    while rclpy.ok():
        node.spin(0.05)
        if time.monotonic() - last >= DISPLAY_SECONDS:
            last = time.monotonic()
            text = (status_line(label, node, chain, jaw, options, args.expected_offsets)
                    if label else 'all poses recorded')
            print(f'\r{text}   ', end='', flush=True)
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            continue
        chunk = os.read(sys.stdin.fileno(), 4096).decode(errors='replace')
        if chunk == '':
            return 0
        buffer += chunk
        while '\n' in buffer:
            line, _, buffer = buffer.partition('\n')
            command, argument = zr.parse_command(line)
            if command == 'quit':
                print()
                return 0
            if command == 'target':
                label = next((f'{argument}#{n}' for n in range(1, zr.REPEATS + 1)
                              if f'{argument}#{n}' not in existing), f'{argument}#1')
            elif command == 'confirm' and state.get('ask'):
                state['confirmed'] = state.pop('ask')
                if record(node, label, args, chain, jaw, options, identity, state):
                    existing.append(label)
                    label = zr.next_label(existing)
            elif command == 'record' and label:
                if record(node, label, args, chain, jaw, options, identity, state):
                    existing.append(label)
                    label = zr.next_label(existing)
            else:
                print(f'\nunknown input: {argument!r}' if command == 'unknown' else '')
    return 0


def parse_args(argv):
    description = os.path.join(get_package_share_directory('so_arm101_description'))
    ap = argparse.ArgumentParser(
        description='zero-offset calibration: record touch poses (read-only; not the center '
                    'calibration)')
    ap.add_argument('--csv', required=True, metavar='FILE', help='the CSV to append to')
    ap.add_argument('--calibration-yaml', default=os.path.join(
        get_package_share_directory('so_arm101_hardware'), 'config', 'follower_calibration.yaml'),
        help='the yaml the driver reads (its sha256 and zero_offset_deg are the identity)')
    ap.add_argument('--urdf', default=os.path.join(description, 'urdf', 'so101_new_calib.urdf'))
    ap.add_argument('--mesh-dir', default=os.path.join(description, 'meshes'))
    ap.add_argument('--h-prior-mm', type=float, default=zo.H_PRIOR_MM)
    ap.add_argument('--h-sigma-mm', type=float, default=zo.H_SIGMA_MM)
    ap.add_argument('--target', choices=sorted(zr.POSES), help='start with this pose')
    ap.add_argument('--expected-offset', action='append', default=[], metavar='NAME=DEG',
                    help='where the arm really is = the angle read + DEG (e.g. elbow_flex=-6.3). '
                         'A guide for the hand only (the display, the target and the 45 deg '
                         'limit); the recorded angles and the fit do not use it. Not for a '
                         'yaml that already has zero_offset_deg')
    args = ap.parse_args(argv)
    try:
        args.expected_offsets = zr.parse_expected_offsets(args.expected_offset)
    except ValueError as error:
        ap.error(str(error))
    return args


def main(argv):
    print(zo.RECORD_BANNER, flush=True)
    args = parse_args(argv)
    try:  # every check that needs no node comes before rclpy.init
        options = zo.Options(h_prior_mm=args.h_prior_mm, h_sigma_mm=args.h_sigma_mm)
        urdf_xml = Path(args.urdf).read_text(encoding='utf-8')
        chain = UrdfChain(urdf_xml)
        jaw = zo.FixedJaw.from_urdf(urdf_xml, args.mesh_dir)
        identity = zr.identity_from_files(args.calibration_yaml, args.urdf, args.mesh_dir)
        zr.check_file(args.csv, identity, expected_offset_deg=args.expected_offsets)
    except (zr.SessionError, OSError, ValueError) as error:
        print(f'refused: {error}', file=sys.stderr)
        return EXIT_REFUSED
    print('expected offset: ' + zr.expected_text(args.expected_offsets)
          + ' (a guide for the hand only; the recorded angles and the fit do not use it)',
          flush=True)
    rclpy.init()
    node = RunNode()
    code = 0
    try:
        code = loop(node, args, chain, jaw, options, identity)
    except KeyboardInterrupt:
        print('\ninterrupted: nothing was sent by this tool')
        code = EXIT_INTERRUPTED
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
