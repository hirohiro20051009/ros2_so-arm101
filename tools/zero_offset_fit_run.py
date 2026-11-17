#!/usr/bin/env python3
"""オフセットキャリブ（zero-offset calibration）の当てはめ: CSV から lift・elbow・wrist_flex の zero_offset_deg を出す。

センターキャリブ（so_arm101_calib measure・apply）ではない。表示だけで、何も書かない
（yaml・CSV・meta・サーボのどれも）。合格したときに表示される「貼る行」を、ユーザーの承認のあとで
手で yaml に入れる（docs/zero_offset_calib_plan.md 6.1）。ROS は使わない。

  python3 tools/zero_offset_fit_run.py zero_offset_YYYYMMDD.csv [--exclude P3#2]

終了コード: 0 合格（貼る行を表示）、2 不合格または拒否（貼る行は出さない）、1 引数・読み込みの誤り。
"""

import argparse
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from so_arm101_calibration import zero_offset as zo
from so_arm101_calibration import zero_offset_record as zr
from so_arm101_ik.urdf_kinematics import UrdfChain

EXIT_ERROR = zo.EXIT_ERROR
EXIT_REFUSED = zo.EXIT_REJECTED


class Parser(argparse.ArgumentParser):
    """An argument parser whose usage error is exit code 1 (2 is the rejection of the data)."""

    def error(self, message):
        self.print_usage(sys.stderr)
        print(f'{self.prog}: error: {message}', file=sys.stderr)
        sys.exit(EXIT_ERROR)


def parse_args(argv):
    description = get_package_share_directory('so_arm101_description')
    ap = Parser(description='zero-offset calibration: fit zero_offset_deg from the records '
                            '(display only; the yaml is not written)')
    ap.add_argument('csv', metavar='CSV', help='the records of zero_offset_record_run.py')
    ap.add_argument('--meta', metavar='FILE', help='the meta file (default: CSV.meta.yaml)')
    ap.add_argument('--exclude', action='append', default=[], metavar='LABEL',
                    help='leave this record out (can be given more than once)')
    ap.add_argument('--sigma-contact', type=float, metavar='MM', help='scatter of one touch (1.0)')
    ap.add_argument('--sigma-90-extra', type=float, metavar='MM',
                    help='added in quadrature for a vertical touch (1.0)')
    ap.add_argument('--h-prior-mm', type=float, help='the desk height (default: from the meta)')
    ap.add_argument('--h-sigma-mm', type=float, help='its sigma (default: from the meta)')
    ap.add_argument('--calibration-yaml', default=os.path.join(
        get_package_share_directory('so_arm101_hardware'), 'config', 'follower_calibration.yaml'),
        help='the yaml the driver read (read only: its sha256 and zero_offset_deg are compared)')
    ap.add_argument('--urdf', default=os.path.join(description, 'urdf', 'so101_new_calib.urdf'))
    ap.add_argument('--mesh-dir', default=os.path.join(description, 'meshes'))
    return ap.parse_args(argv)


def options_of(args, meta):
    settings = meta.get('settings') or {}
    pick = zo.Options()
    return zo.Options(
        h_prior_mm=_first(args.h_prior_mm, settings.get('h_prior_mm'), pick.h_prior_mm),
        h_sigma_mm=_first(args.h_sigma_mm, settings.get('h_sigma_mm'), pick.h_sigma_mm),
        sigma_contact_mm=_first(args.sigma_contact, settings.get('sigma_contact_mm'),
                                pick.sigma_contact_mm),
        sigma_90_extra_mm=_first(args.sigma_90_extra, settings.get('sigma_90_extra_mm'),
                                 pick.sigma_90_extra_mm))


def _first(*values):
    return next(v for v in values if v is not None)


def main(argv):
    print(zo.FIT_BANNER, flush=True)
    args = parse_args(argv)
    try:
        urdf_xml = Path(args.urdf).read_text(encoding='utf-8')
        chain = UrdfChain(urdf_xml)
        jaw = zo.FixedJaw.from_urdf(urdf_xml, args.mesh_dir)
        identity = zr.identity_from_files(args.calibration_yaml, args.urdf, args.mesh_dir)
        touches = None
        if not Path(args.csv).exists():
            raise OSError(f'{args.csv} does not exist')
    except (OSError, ValueError) as error:
        print(f'cannot read: {error}', file=sys.stderr)
        return EXIT_ERROR
    try:
        zr.check_file(args.csv, identity, args.meta)
        meta = zr.read_meta(args.csv, args.meta)
        touches = zr.read_touches(args.csv)
    except zr.SessionError as error:
        print(f'refused: {error}', file=sys.stderr)
        return EXIT_REFUSED
    labels = {t.label for t in touches}
    unknown = sorted(set(args.exclude) - labels)
    if unknown:
        print(f'cannot read: no such record to exclude: {", ".join(unknown)}', file=sys.stderr)
        return EXIT_ERROR
    touches = [t for t in touches if t.label not in set(args.exclude)]
    if args.exclude:
        print('left out: ' + ', '.join(args.exclude))
    report = zo.evaluate(chain, jaw, touches, identity['zero_offset_deg'], options_of(args, meta))
    for line in zo.report_lines(report):
        print(line)
    return report.exit_code


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
