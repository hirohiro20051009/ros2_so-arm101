#!/usr/bin/env python3
"""C4 の独立した確認（docs/camera_stage_plan.md 9.2 と 2026-11-19 の注記、inventory.md の C4-0）。

腕には何も送らない。シリアルもカメラも開かない。購読するのは /joint_states と
/arm_controller/controller_state だけ（TF は tf2_ros の listener で読む）。CSV は so_arm101_camera.record の
write_verify_row だけが書く（この道具は open を使わない）。

  # 手順書用: 定規で測った印から hover の目標と ik_move_run.py のコマンドを出す（ROS は使わない）
  python3 tools/camera_verify_run.py --verify-plan --mark-rear 0.221 0 --desk-minus-mount 0 \
      --thickness 0.0005 --offset 0 0.062 --offset 0.02 0.062
  # (a) 腕を退けた状態で、定規で測った ID 1 と TF の tfb_tag1 を比べる（カメラ・腕は別の端末で起動済み）
  python3 tools/camera_verify_run.py --verify-survey --mark-rear 0.221 0 --desk-minus-mount 0 \
      --thickness 0.0005 --csv c4_verify.csv --label S01
  # 1 点: ik_move_run.py が止まってから、定規で測った値（相対 X Y と高さ）を入れる
  python3 tools/camera_verify_run.py --verify-record --mark-rear 0.221 0 --desk-minus-mount 0 \
      --thickness 0.0005 --offset 0 0.062 --ruler 0.003 -0.001 0.0556 --csv c4_verify.csv \
      --label V01 --move-csv ik_move_xxx.csv
  # オフラインの報告: A・B の判定と δ の参考値
  python3 tools/camera_verify_run.py --verify-report --csv c4_verify.csv

単位はすべて m。Δ（--desk-minus-mount）は机の面 − 取り付け面で、机の面が高いとき正。--ruler は X・Y が
ID 1 の中心から固定あごの先端（パッドの前の縁の中央）への水平の差（+X は前、+Y は左）、H が ID 1 の面から先端
までの高さ。判定に tfb_tag1（カメラ）は使わない（hover の 4 点では ID 1 があごに隠れる）。
FK には /joint_states の値をそのまま入れる（ドライバが zero_offset_deg をかけた後の値。δ は二重にかけない）。
終了コード: 0 成功、2 拒否、3 腕が動いている・タグが古い、5 入力なし、130 Ctrl-C。
"""

import argparse
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from so_arm101_camera import record as rec
from so_arm101_camera import tag_mount as tm
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik import ik_move as im
from so_arm101_ik import step_runner as sr
from tf2_ros import Buffer, TransformException, TransformListener

JOINT_STATES = '/joint_states'
CONTROLLER_STATE = '/arm_controller/controller_state'
BASE_LINK = 'base_link'
TAG_ID = 1  # the tag on the desk
JOINT_SECONDS = 1.0  # s, the joint states that are averaged (推測)
JOINT_MIN = 5  # the least number of joint states in that time (推測)
INPUT_WAIT = 3.0  # s, wait for the first inputs
TF_WAIT = 8.0  # s, wait for the tag of the survey (推測)
TF_SAMPLES = 15  # transforms (distinct stamps) averaged by the survey (推測)
TF_MAX_AGE = 1.0  # s, a tag TF older than this is not "seen" (推測)
SURVEY_STD_MM = 1.0  # mm, a survey whose spread is larger is reported as unstable (推測)
T_DEG = (0.0, -22.0, 30.0, 83.0, 0.0)  # the pose the hover targets are reached from


def parse_args(argv):
    config = os.path.join(get_package_share_directory('so_arm101_camera'), 'config')
    urdf = os.path.join(get_package_share_directory('so_arm101_description'), 'urdf',
                        'so101_new_calib.urdf')
    ap = argparse.ArgumentParser(description='C4 の独立した確認（腕に何も送らない）')
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--verify-plan', action='store_true', help='hover の目標とコマンドを出す')
    mode.add_argument('--verify-survey', action='store_true', help='(a) 定規の ID 1 と TF の比較')
    mode.add_argument('--verify-record', action='store_true', help='1 点を記録する')
    mode.add_argument('--verify-report', action='store_true', help='A・B の判定と δ の参考値')
    ap.add_argument('--mark-rear', type=float, nargs=2, metavar=('X', 'Y'),
                    help='ID 1 の中心。X は後ろの辺から前へ、Y は後ろの 2 つの角の中点の線から左へ [m]')
    ap.add_argument('--desk-minus-mount', type=float, metavar='M',
                    help='机の面 − 取り付け面 [m]（机の面が高いとき正）')
    ap.add_argument('--thickness', type=float, metavar='M', help='ID 1 の厚さ [m]')
    ap.add_argument('--offset', type=float, nargs=2, action='append', default=[],
                    metavar=('DX', 'H'),
                    help='印からの前後のずれ DX と、TCP の ID 1 の面からの高さ H [m]（繰り返せる）')
    ap.add_argument('--pitch-deg', type=float, default=-91.0)
    ap.add_argument('--roll-deg', type=float, default=0.0)
    ap.add_argument('--start-deg', type=float, nargs=5, default=list(T_DEG),
                    help='hover の目標へ着く前の姿勢（5 関節の度）。既定は T')
    ap.add_argument('--csv', nargs='+', metavar='FILE',
                    help='検証の CSV（記録は 1 つ。報告は複数）')
    ap.add_argument('--label', help='点の名前（記録と survey）')
    ap.add_argument('--move-csv', metavar='FILE', help='ik_move_run.py の CSV（q_goal を読む）')
    ap.add_argument('--ruler', type=float, nargs=3, metavar=('X', 'Y', 'H'),
                    help='定規の読み [m]: 相対 X・Y と高さ H')
    ap.add_argument('--ruler-mm', type=float, default=2.0, help='定規の誤差（点ごと）[mm]')
    ap.add_argument('--common-mm', type=float, nargs=3, default=[1.5, 1.5, 1.5],
                    metavar=('X', 'Y', 'Z'), help='全点に共通のずれ（x・y・z）[mm]')
    ap.add_argument('--tag-mount', default=os.path.join(config, 'tag_mount.yaml'))
    ap.add_argument('--apriltag-yaml', default=os.path.join(config, 'apriltag.yaml'))
    ap.add_argument('--urdf', default=urdf)
    ap.add_argument('--calibration-yaml', default=os.path.join(
        get_package_share_directory('so_arm101_hardware'), 'config',
        'follower_calibration.yaml'))
    args = ap.parse_args(argv)
    problems = check_args(args)
    if problems:
        ap.error('; '.join(problems))
    return args


def check_args(args):
    """Return the list of what is missing or wrong in the arguments (before any node)."""
    problems = []
    if not args.verify_report:
        if args.mark_rear is None or args.desk_minus_mount is None or args.thickness is None:
            problems.append('--mark-rear, --desk-minus-mount and --thickness are required '
                            '(measured with the ruler; there is no default)')
        elif not all(math.isfinite(v) for v in (*args.mark_rear, args.desk_minus_mount,
                                                args.thickness)):
            problems.append('the mark values must be finite')
    if args.verify_plan and not args.offset:
        problems.append('--verify-plan needs at least one --offset DX H')
    if args.verify_record:
        if len(args.offset) != 1:
            problems.append('--verify-record needs exactly one --offset DX H (the point)')
        if args.ruler is None:
            problems.append('--verify-record needs --ruler X Y H')
        if not args.csv or len(args.csv) != 1 or not args.label:
            problems.append('--verify-record needs one --csv and --label')
    if args.verify_survey and bool(args.csv) != bool(args.label):
        problems.append('--csv and --label go together')
    if args.verify_survey and args.csv and len(args.csv) != 1:
        problems.append('--verify-survey takes one --csv')
    if args.verify_report and not args.csv:
        problems.append('--verify-report needs --csv')
    return problems


def load_model(args):
    """Return (chain, reference point, zero offsets of the driver, mark in base_link)."""
    mount = tm.load_tag_mount(args.tag_mount, args.apriltag_yaml)
    chain = rec.load_chain(args.urdf)
    zero = rec.read_zero_offsets(args.calibration_yaml)
    mark = None
    if args.mark_rear is not None:
        mark = rec.mark_in_base(args.mark_rear[0], args.mark_rear[1], args.desk_minus_mount,
                                args.thickness)
    return chain, mount.reference_point, zero, mark


def mark_lines(args, mark):
    return ['mark (ID 1 centre): rear frame (%.4f, %+.4f) m -> base_link (%.4f, %+.4f), the '
            'face z = %.4f m' % (args.mark_rear[0], args.mark_rear[1], mark[0], mark[1], mark[2]),
            'desk - mounting plane = %+.4f m (positive: the desk is higher), thickness %.4f m'
            % (args.desk_minus_mount, args.thickness)]


# ------------------------------------------------------------------------------- the plan
def run_plan(args):
    try:
        chain, reference, _, mark = load_model(args)
    except (tm.TagMountError, OSError, ValueError) as error:
        print('refused: %s' % error, file=sys.stderr)
        return rec.EXIT_REFUSED
    start = np.radians(args.start_deg)
    print('\n'.join(mark_lines(args, mark)))
    print('start of every move (for the checks only): joints %s deg' % list(args.start_deg))
    code = rec.EXIT_OK
    for number, (dx, h) in enumerate(args.offset, 1):
        hover = rec.hover_target(mark, dx, h)
        print('\n[%d] offset dx %+.4f m, TCP height above the face %.4f m' % (number, dx, h))
        print('    TCP target (base_link) %s  height above the mounting plane H %.4f m, above the '
              'ID 1 face %.4f m' % (np.round(hover.target, 4), hover.h_mount, hover.h_face))
        if not hover.ok:
            print('    REFUSED: %s' % hover.reason)
            code = rec.EXIT_REFUSED
            continue
        plan = im.solve_goal(start, goal=list(hover.target), pitch_deg=args.pitch_deg,
                             roll_deg=args.roll_deg)
        if plan.q_goal is None or not plan.ok:
            print('    ik_move would refuse: %s' % '; '.join(plan.details))
            code = rec.EXIT_REFUSED
            continue
        q = np.asarray(plan.q_goal)
        edge = min((q - ik.Q_MIN).min(), (ik.Q_MAX - q).min())
        tip = rec.jaw_tip(chain, q, reference)
        print('    q_goal %s deg, the nearest joint end %.3f rad, tip move %.1f mm' % (
            np.round(np.degrees(q), 2), edge, plan.tip_move * 1000.0))
        print('    jaw tip at the goal: z %.4f m, H %.4f m, above the face %.4f m' % (
            tip[2], rec.height_above_mount(tip[2]), rec.height_above_face(tip[2], mark[2])))
        print('    ' + rec.ik_move_command(hover.target, args.pitch_deg, args.roll_deg, True))
        print('    ' + rec.ik_move_command(hover.target, args.pitch_deg, args.roll_deg))
    return code


# --------------------------------------------------------------------------------- the node
class VerifyNode(Node):
    """Reads /joint_states, the controller state and the TF tree; publishes nothing."""

    def __init__(self):
        super().__init__('camera_verify_run')
        self._q = []
        self._ref = None
        self._ref_at = None
        self._collect = False
        self.create_subscription(JointState, JOINT_STATES, self._on_joint_state, 10)
        self.create_subscription(JointTrajectoryControllerState, CONTROLLER_STATE,
                                 self._on_controller_state, 10)
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

    def _on_joint_state(self, msg):
        q = sr.positions_by_name(msg.name, msg.position)
        if q is not None and self._collect:
            self._q.append(np.asarray(q, dtype=float))

    def _on_controller_state(self, msg):
        # Humble: reference is the command (desired is a deprecated alias)
        q = sr.positions_by_name(msg.joint_names, msg.reference.positions)
        if q is not None:
            self._ref = np.asarray(q, dtype=float)
            self._ref_at = time.monotonic()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def joints(self, seconds=JOINT_SECONDS):
        """Return the joint states received in `seconds`."""
        self._q = []
        self._collect = True
        self.spin_for(seconds)
        self._collect = False
        return list(self._q)

    def reference(self, wait=INPUT_WAIT):
        """Return the latest reference of the controller if it is fresh, else None."""
        end = time.monotonic() + wait
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._ref is not None and sr.is_fresh(self._ref_at, time.monotonic()):
                return self._ref
        return None

    def tag(self, frame):
        """Return (position, quaternion, stamp_ns, age_s) of base_link to frame, or None."""
        try:
            found = self._buffer.lookup_transform(BASE_LINK, frame, Time())
        except TransformException:
            return None
        t, q = found.transform.translation, found.transform.rotation
        stamp = Time.from_msg(found.header.stamp).nanoseconds
        age = rec.tf_age_s(self.get_clock().now().nanoseconds, stamp)
        return (np.array([t.x, t.y, t.z]), (q.x, q.y, q.z, q.w), stamp, age)


def run_survey(args):
    try:
        _, _, zero, mark = load_model(args)
        frame = tm.tag_frame_name(args.apriltag_yaml, TAG_ID)
        if args.csv and os.path.exists(args.csv[0]):
            rec.read_verify_rows(args.csv[0])  # the header is checked before anything is read
    except (tm.TagMountError, rec.SessionError, OSError, ValueError) as error:
        print('refused: %s' % error, file=sys.stderr)
        return rec.EXIT_REFUSED
    rclpy.init()
    node = VerifyNode()
    code = rec.EXIT_NO_INPUT
    try:
        seen = {}
        end = time.monotonic() + TF_WAIT
        while rclpy.ok() and time.monotonic() < end and len(seen) < TF_SAMPLES:
            node.spin_for(0.1)
            found = node.tag(frame)
            if found is not None and found[3] <= TF_MAX_AGE:
                seen[found[2]] = found
        if len(seen) < TF_SAMPLES // 2:
            print('the tag %s was not seen (%d fresh transforms in %.0f s): nothing was written. '
                  'Is the camera launch with the extrinsics running, and is ID 1 in view with '
                  'the arm out of the way?' % (frame, len(seen), TF_WAIT), file=sys.stderr)
            return rec.EXIT_NOT_SEEN
        positions = np.array([v[0] for v in seen.values()])
        last = max(seen.values(), key=lambda v: v[2])
        mean = positions.mean(axis=0)
        spread = positions.std(axis=0) * 1000.0
        diff = mean - np.asarray(mark)
        tilt = rec.tag_tilt_deg(last[1])
        print('\n'.join(mark_lines(args, mark)))
        print('(a) camera (tfb_tag1, %d transforms, age of the last %.2f s): %s m, spread %s mm'
              % (len(seen), last[3], np.round(mean, 4), np.round(spread, 2)))
        print('    tfb_tag1 - ruler = %s mm: horizontal %.1f mm, vertical %+.1f mm' % (
            np.round(diff * 1000.0, 1), np.hypot(diff[0], diff[1]) * 1000.0, diff[2] * 1000.0))
        print('    tilt of the ID 1 face %.1f deg (more than %.0f: put it down again)' % (
            tilt, rec.TILT_MAX_DEG))
        if spread.max() > SURVEY_STD_MM:
            print('    WARNING: the spread is larger than %.1f mm (the tag or the camera moved?)'
                  % SURVEY_STD_MM)
        if tilt > rec.TILT_MAX_DEG:
            print('    WARNING: the tag is tilted')
        if args.csv:
            row = rec.build_verify_row(time.time(), args.mark_rear, args.desk_minus_mount,
                                       args.thickness, tfb=(*mean, last[3]), zero_offset=zero)
            idx = rec.write_verify_row(args.csv[0], args.label, row)
            print('recorded %s as row %d of %s (a survey row: no joint angles)' % (
                args.label, idx, args.csv[0]))
        code = rec.EXIT_OK
    except rec.SessionError as error:
        print('refused: %s' % error, file=sys.stderr)
        code = rec.EXIT_REFUSED
    except KeyboardInterrupt:
        print('interrupted: nothing was sent by this tool')
        code = sr.EXIT_INTERRUPTED
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


# -------------------------------------------------------------------------------- one point
def run_record(args):
    try:
        chain, reference, zero, mark = load_model(args)
        frame = tm.tag_frame_name(args.apriltag_yaml, TAG_ID)
        q_goal = rec.read_move_goal(args.move_csv) if args.move_csv else None
        if os.path.exists(args.csv[0]):
            rec.read_verify_rows(args.csv[0])
    except (tm.TagMountError, rec.SessionError, OSError, ValueError) as error:
        print('refused: %s' % error, file=sys.stderr)
        return rec.EXIT_REFUSED
    rclpy.init()
    node = VerifyNode()
    code = rec.EXIT_NO_INPUT
    try:
        samples = node.joints()
        if len(samples) < JOINT_MIN:
            print('no fresh /joint_states (%d in %.1f s): nothing was written' % (
                len(samples), JOINT_SECONDS), file=sys.stderr)
            return rec.EXIT_NO_INPUT
        stack = np.array(samples)
        q_read = stack.mean(axis=0)
        span = float((stack.max(axis=0) - stack.min(axis=0)).max())
        if span > sr.STOP_TOL:
            print('the arm is moving (the joint angles vary by %.4f rad > %.4f): nothing was '
                  'written' % (span, sr.STOP_TOL), file=sys.stderr)
            return rec.EXIT_NOT_SEEN
        q_ref = node.reference()
        if q_ref is None:
            print('no fresh %s: nothing was written (FK(reference) needs it)' % CONTROLLER_STATE,
                  file=sys.stderr)
            return rec.EXIT_NO_INPUT
        fk_read = rec.jaw_tip(chain, q_read, reference)
        fk_ref = rec.jaw_tip(chain, q_ref, reference)
        phys = rec.physical_jaw_tip(mark, *args.ruler)
        errors = rec.point_errors(phys, fk_read, fk_ref)
        found = node.tag(frame)  # recorded only: ID 1 is hidden by the jaw at the hover points
        tfb = None if found is None else (*found[0], found[3])
        row = rec.build_verify_row(time.time(), args.mark_rear, args.desk_minus_mount,
                                   args.thickness, offset=args.offset[0], q_read=q_read,
                                   q_ref=q_ref, q_goal=q_goal, ruler=args.ruler, fk_read=fk_read,
                                   fk_ref=fk_ref, phys=phys, errors=errors, tfb=tfb,
                                   zero_offset=zero, still=True)
        idx = rec.write_verify_row(args.csv[0], args.label, row)
        print('\n'.join(mark_lines(args, mark)))
        print('q_read (deg, as /joint_states says; the driver has applied zero_offset_deg %s): %s'
              % (zero, np.round(np.degrees(q_read), 2)))
        print('q_reference (deg): %s' % np.round(np.degrees(q_ref), 2))
        _, largest = rec.goal_difference(q_ref, q_goal)
        if largest is not None:
            print('reference - q_goal of ik_move: largest %.4f rad' % largest)
        print('jaw tip (base_link, m): FK(read) %s  FK(reference) %s  ruler %s' % (
            np.round(fk_read, 4), np.round(fk_ref, 4), np.round(phys, 4)))
        print('heights above the mounting plane H / above the ID 1 face: FK(read) %.4f / %.4f, '
              'FK(reference) %.4f / %.4f, ruler %.4f / %.4f m' % (
                  rec.height_above_mount(fk_read[2]), rec.height_above_face(fk_read[2], mark[2]),
                  rec.height_above_mount(fk_ref[2]), rec.height_above_face(fk_ref[2], mark[2]),
                  rec.height_above_mount(phys[2]), rec.height_above_face(phys[2], mark[2])))
        print('|FK(read) - FK(reference)| = %.1f mm; ruler - FK(read) = %s mm; ruler - '
              'FK(reference) = %s mm' % (errors['d'] * 1000.0,
                                         np.round(errors['e_read'] * 1000.0, 1),
                                         np.round(errors['e_ref'] * 1000.0, 1)))
        if errors['fraction'] is not None:
            print('fraction (0 = at FK(read) = A, 1 = at FK(reference) = B): %.2f, off the line '
                  '%.1f mm%s' % (errors['fraction'], errors['perp'] * 1000.0,
                                 '' if errors['d'] >= rec.AB_GATE_M else
                                 '  (d < %.0f mm: this point does not decide)'
                                 % (rec.AB_GATE_M * 1000.0)))
        print('tfb_tag1 (not used): %s' % ('not visible' if tfb is None else
                                           '%s m, age %.2f s' % (np.round(found[0], 4), found[3])))
        print('recorded %s as row %d of %s' % (args.label, idx, args.csv[0]))
        code = rec.EXIT_OK
    except rec.SessionError as error:
        print('refused: %s' % error, file=sys.stderr)
        code = rec.EXIT_REFUSED
    except KeyboardInterrupt:
        print('interrupted: nothing was sent by this tool')
        code = sr.EXIT_INTERRUPTED
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


# ------------------------------------------------------------------------------- the report
def _floats(row, names):
    try:
        values = [float(row[n]) for n in names]
    except ValueError:
        return None
    return np.array(values) if all(math.isfinite(v) for v in values) else None


def run_report(args):
    try:
        chain, reference, _, _ = load_model(args)
        rows = []
        for path in args.csv:
            rows += rec.read_verify_rows(path)
    except (tm.TagMountError, rec.SessionError, OSError, ValueError) as error:
        print('refused: %s' % error, file=sys.stderr)
        return rec.EXIT_REFUSED
    q_names = ('q_read_pan', 'q_read_lift', 'q_read_elbow', 'q_read_wflex', 'q_read_roll')
    points, errors = [], []
    print('rows: %d' % len(rows))
    for row in rows:
        mark = rec.mark_in_base(float(row['mark_rear_x']), float(row['mark_rear_y']),
                                float(row['desk_minus_mount']), float(row['thickness']))
        tfb = _floats(row, ('tfb_tag1_x', 'tfb_tag1_y', 'tfb_tag1_z'))
        q_read = _floats(row, q_names)
        if q_read is None:  # a survey row: (a) the camera against the ruler
            if tfb is not None:
                diff = (tfb - np.asarray(mark)) * 1000.0
                print('%s  (a) tfb_tag1 - ruler = %s mm: horizontal %.1f mm, vertical %+.1f mm, '
                      'age %s s' % (row['label'], np.round(diff, 1), np.hypot(diff[0], diff[1]),
                                    diff[2], row['tfb_tag1_age_s']))
            continue
        phys = _floats(row, ('phys_x', 'phys_y', 'phys_z'))
        fk_read = _floats(row, ('fk_read_x', 'fk_read_y', 'fk_read_z'))
        fk_ref = _floats(row, ('fk_ref_x', 'fk_ref_y', 'fk_ref_z'))
        if phys is None or fk_read is None or fk_ref is None:
            print('%s  skipped (a value is missing)' % row['label'])
            continue
        error = rec.point_errors(phys, fk_read, fk_ref)
        errors.append(error)
        points.append((q_read, phys))
        fraction = error['fraction']
        print('%s  offset (%s, %s)  |FK(read) - FK(ref)| %.1f mm  fraction %s  '
              'off the line %s mm  ruler - FK(read) %s mm  ruler - FK(ref) %s mm  '
              'reference - q_goal %s rad' % (
                  row['label'], row['offset_dx'], row['offset_h'], error['d'] * 1000.0,
                  'n/a' if fraction is None else '%.2f' % fraction,
                  'n/a' if error['perp'] is None else '%.1f' % (error['perp'] * 1000.0),
                  np.round(error['e_read'] * 1000.0, 1), np.round(error['e_ref'] * 1000.0, 1),
                  row['ref_minus_goal_max'] or 'n/a'))
    verdict, mean, std, used = rec.decide_ab(errors)
    print('\nA or B (the points with |FK(read) - FK(reference)| >= %.0f mm; the camera is not an '
          'input): %s' % (rec.AB_GATE_M * 1000.0, verdict))
    if mean is not None:
        print('  mean fraction %.2f, standard deviation %.2f, %d points used (0 = the reading is '
              'the physical angle: A; 1 = the reference is: B)' % (mean, std, used))
    else:
        print('  %d points used: the decision needs two or more' % used)
    common = [v / 1000.0 for v in args.common_mm]
    estimate = rec.estimate_delta(chain, points, reference, args.ruler_mm / 1000.0, common)
    print('\nREFERENCE VALUE (not a result): delta (true = read + delta), degrees')
    if not estimate.ok:
        print('  no estimate: %s' % estimate.reason)
    else:
        print('  delta_lift %+.2f, delta_elbow %+.2f deg (fit rms %.2f mm, condition %.1f)' % (
            *estimate.delta_deg, estimate.rms_m * 1000.0, estimate.condition))
        print('  uncertainty: ruler %.2f / %.2f deg (each point %.1f mm, averages out), common '
              'shift %.2f / %.2f deg (x %.1f, y %.1f, z %.1f mm: does not average out), total '
              '%.2f / %.2f deg (lift / elbow)' % (
                  *estimate.sd_ruler_deg, args.ruler_mm, *estimate.sd_common_deg, *args.common_mm,
                  *estimate.sd_total_deg))
        print('  delta is not written anywhere by this tool (C4 does not rewrite the yaml)')
    return rec.EXIT_OK


def main(argv):
    args = parse_args(argv)
    if args.verify_plan:
        return run_plan(args)
    if args.verify_survey:
        return run_survey(args)
    if args.verify_record:
        return run_record(args)
    return run_report(args)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
