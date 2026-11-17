#!/usr/bin/env python3
"""C3: C2 の CSV から base_link → camera_link を Kabsch で出し、camera_extrinsics.yaml を作る。

  # 提案の表示だけ（何も書かない）
  python3 tools/camera_extrinsics_run.py c2_points.csv
  # 合格したときだけ src の config に書く（既存のファイルは --force が無いと上書きしない）
  python3 tools/camera_extrinsics_run.py c2_points.csv --write
  # 腕の 0 のずれ（q_true = q_meas + δ。度）を当てて fk_tag を q から計算し直す（CSV は変えない）
  python3 tools/camera_extrinsics_run.py c2_points.csv --joint-offset elbow_flex=-6.3

ROS は使わない（CSV と meta だけから計算する）。計算は so_arm101_camera.extrinsics。
--joint-offset NAME=DEG（複数可。gripper は不可。絶対値 10 度以内）: CSV の q_ 列に DEG を足した角度で
fk_tag を計算し直す。--urdf は meta の urdf_sha256 と同じ sha256 のものだけ受け付ける（既定は
so_arm101_description の share の urdf/so101_new_calib.urdf）。meta に零のずれの記録が無い古い
CSV は --joint-offset と一緒のときだけ使える（ずれなしは NAME=0）。meta に 0 でない δ が記録された
CSV（ドライバが補正済み）に --joint-offset を重ねたら拒否する。使った δ は sidecar に残す。
書くのは本体（translation_m と quaternion_xyzw の 2 キーだけ）と、記録の
camera_extrinsics.yaml.meta.yaml。不合格のときは残差・LOO・スケール・広がりをすべて表示して
から、--write と --force があっても何も書かず、終了コード 2 にする。
終了コード: 0 合格（--write なら書いた）、2 不合格または拒否（何も書かない）。
"""

import argparse
from pathlib import Path
import sys

from so_arm101_camera import camera_config as cc
from so_arm101_camera import extrinsics as ex

ROOT = Path(__file__).resolve().parents[1]


def default_output_path():
    """Return the extrinsics file in the src tree of this repository (never install)."""
    return ROOT / 'src' / 'so_arm101_camera' / 'config' / 'camera_extrinsics.yaml'


def default_urdf_path():
    """Return the URDF of the description package (the install copy that record used)."""
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('so_arm101_description')) / 'urdf' / (
        'so101_new_calib.urdf')


def build_parser():
    """Return the argument parser."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('csv', help='C2 の CSV（tools/camera_record_run.py の出力）')
    ap.add_argument('--meta', help='meta ファイル（既定は CSV の名前に .meta.yaml を足したもの）')
    ap.add_argument('--output', default=str(default_output_path()),
                    help='書き先（既定は src の config/camera_extrinsics.yaml）')
    ap.add_argument('--exclude', action='append', default=[], metavar='LABEL',
                    help='使わない点の label（複数可。やり直した点の古い行など）')
    ap.add_argument('--joint-offset', action='append', default=[], metavar='NAME=DEG',
                    help='q_true = q_meas + DEG（度）。複数可。例 elbow_flex=-6.3')
    ap.add_argument('--urdf', help='fk_tag の計算し直しに使う URDF（meta の urdf_sha256 と同じ'
                    'ものだけ。--joint-offset のときだけ。既定は description の share）')
    ap.add_argument('--write', action='store_true', help='合格したときだけ書く')
    ap.add_argument('--force', action='store_true', help='既存のファイルを置き換える')
    return ap


def main(argv=None):
    """Run the tool and return the exit code."""
    args = build_parser().parse_args(argv)
    meta_path = args.meta or str(rec_meta_path(args.csv))
    try:
        requested = ex.parse_joint_offsets(args.joint_offset)
        if args.urdf and not requested:
            raise ex.InputError('--urdf is used only together with --joint-offset')
        rows = ex.read_csv_rows(args.csv)
        meta, optical = ex.read_meta(meta_path)
        plan = ex.plan_offsets(meta, requested)
        if requested:
            rows, plan = ex.recompute_fk_tag(rows, meta, args.urdf or default_urdf_path(), plan)
        points, rejected = ex.select_rows(rows, tuple(args.exclude))
    except ex.InputError as exc:
        print(f'refused: {exc}')
        return ex.EXIT_FAIL
    for line in ex.offset_lines(plan, len(rows)):
        print(line)
    solution = ex.solve(points, optical, ex.tag_size_of(meta))
    for line in ex.report_lines(solution, rejected):
        print(line)
    if solution.failures:
        print('nothing is written')
        return ex.EXIT_FAIL
    main_text = ex.extrinsics_text(solution.link_translation, solution.quaternion)
    print('--- ' + str(args.output))
    print(main_text, end='')
    if not args.write:
        print('not written (add --write to write it)')
        return ex.EXIT_OK
    side_text = ex.sidecar_text(solution, rejected, args.csv, meta_path, meta, optical,
                                main_text, ex.utc_now_text(), tuple(args.exclude), plan)
    if args.force and Path(args.output).exists():
        print('replacing: ' + ex.describe_existing(args.output))
    try:
        ex.write_outputs(args.output, main_text, side_text, solution.link_translation,
                         solution.quaternion, args.force)
    except ex.WriteRefused as exc:
        print(f'refused: {exc}')
        return ex.EXIT_FAIL
    print(f'written: {args.output} and {ex.sidecar_path_for(args.output)}')
    print('the install copy is made by: colcon build --packages-select so_arm101_camera '
          f'(or try it with extrinsics_file:={args.output}; default {cc.TF_PARENT_FRAME} to '
          f'{cc.TF_CHILD_FRAME})')
    return ex.EXIT_OK


def rec_meta_path(csv_path):
    """Return the meta file that belongs to a CSV file."""
    return Path(str(csv_path) + '.meta.yaml')


if __name__ == '__main__':
    sys.exit(main())
