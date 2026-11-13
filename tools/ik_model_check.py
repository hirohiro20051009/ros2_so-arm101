#!/usr/bin/env python3
"""解析 IK・FK の模型と MoveIt の FK の差を表（mm）にする。腕を動かさない（読み取りだけ）。

  python3 tools/ik_model_check.py                       # 今の姿勢・±10 mm の 6 方向・roll の掃引
  python3 tools/ik_model_check.py --cases now           # 今の姿勢だけ
  python3 tools/ik_model_check.py --target 172 0 55     # 目標の先端 [mm] への往復（IK -> /compute_fk）
  python3 tools/ik_model_check.py --csv table.csv       # 同じ表をファイルにも
  python3 tools/ik_model_check.py --record-fixture src/so_arm101_ik/test/data/moveit_fk.csv

前提: `source setup_so101.sh`（または install/setup.bash）の後。`move_group` が動いていること（/compute_fk）。
モックでも、実機の読み取り専用（bringup.launch.py mock:=false read_only:=true）でも使える。

【読み取りだけ】購読するのは /joint_states、呼ぶのは /compute_fk（GetPositionFK）だけ。--record-fixture のときだけ
/move_group/get_parameters（robot_description の読み取り）も呼ぶ。パブリッシャー・action クライアント・
パラメータの書き込みは持たない（tests の test_ik_model_check_static.py が AST で確かめる）。
/compute_fk は渡した関節角から計算するだけで、コントローラにも move_group の現在の状態にも触らない。

表: model/target は解析 FK（IK の行では目標）、MoveIt FK は /compute_fk、d = model - MoveIt。
chain-MoveIt は URDF の鎖（自前）と MoveIt の差、dpitch は先端の接近軸の pitch の差 [deg]（模型 - MoveIt）、
orient は鎖と MoveIt の姿勢の差 [deg]。解析モデルの roll は関節角 q5 そのもので、姿勢からは取り出さない
（roll の違いは orient に出る）。許容を超えた行は * が付く。--ik-range-margin は IK の関節範囲を広げる
（読み取りだけなので安全。出発点の姿勢が URDF の範囲を少し超えていても今の姿勢を確認できる）。
"""
import argparse
import csv
import datetime
import math
import os
import subprocess
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
import numpy as np
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from so_arm101_ik import analytic_ik, model_check
from so_arm101_ik.model_check import CaseRow
from so_arm101_ik.urdf_kinematics import ARM_JOINTS, UrdfChain

FK_SERVICE = '/compute_fk'
PARAMETER_SERVICE = '/move_group/get_parameters'
ALLOWED_PUBLISHED_TOPICS = {'/parameter_events', '/rosout'}  # created by rclpy itself
SERVICE_TIMEOUT = 10.0
OFFSET_MM = 10.0
NOW_TOLERANCE_MM = 1.0
IK_TOLERANCE_MM = 0.5


def installed_urdf():
    share = Path(get_package_share_directory('so_arm101_description'))
    return (share / 'urdf' / 'so101_new_calib.urdf').read_text()


class Checker(Node):
    """Reads /joint_states and calls /compute_fk. Nothing else."""

    def __init__(self, joint_states_topic):
        super().__init__('ik_model_check')
        self._joint_state = None
        self.create_subscription(JointState, joint_states_topic, self._on_joint_state, 10)
        self._fk = self.create_client(GetPositionFK, FK_SERVICE)
        self._parameters = None

    def _on_joint_state(self, msg):
        self._joint_state = msg

    def assert_nothing_is_sent(self):
        """Stop if this node publishes anything but what rclpy creates itself."""
        topics = {name for name, _ in self.get_publisher_names_and_types_by_node(
            self.get_name(), self.get_namespace())}
        extra = topics - ALLOWED_PUBLISHED_TOPICS
        if extra:
            raise RuntimeError(f'this node publishes {sorted(extra)}: refusing to run')

    def _wait(self, future, what, timeout=SERVICE_TIMEOUT):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f'timeout ({timeout} s) waiting for {what}')
        return future.result()

    def wait_for_fk_service(self):
        if not self._fk.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            raise RuntimeError(f'{FK_SERVICE} is not available (is move_group running?)')

    def arm_joint_angles(self, timeout=SERVICE_TIMEOUT):
        """Return q of the five arm joints from the next /joint_states."""
        self._joint_state = None
        end = time.monotonic() + timeout
        while self._joint_state is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._joint_state is None:
            raise RuntimeError('no /joint_states received')
        by_name = dict(zip(self._joint_state.name, self._joint_state.position))
        missing = [j for j in ARM_JOINTS if j not in by_name]
        if missing:
            raise RuntimeError(f'/joint_states has no {missing}')
        return np.array([by_name[j] for j in ARM_JOINTS])

    def moveit_fk(self, q):
        """Return (position [m], quaternion x y z w) of the tip in base_link from MoveIt."""
        request = GetPositionFK.Request()
        request.header.frame_id = model_check.FIXTURE_FRAME
        request.fk_link_names = [model_check.FIXTURE_LINK]
        request.robot_state = RobotState()
        request.robot_state.joint_state.name = list(ARM_JOINTS)
        request.robot_state.joint_state.position = [float(v) for v in q]
        response = self._wait(self._fk.call_async(request), FK_SERVICE)
        if response.error_code.val != 1:
            raise RuntimeError(f'{FK_SERVICE} failed: error_code {response.error_code.val}')
        stamped = response.pose_stamped[0]
        if stamped.header.frame_id != model_check.FIXTURE_FRAME:
            raise RuntimeError(f'the pose is in {stamped.header.frame_id!r}, not '
                               f'{model_check.FIXTURE_FRAME!r}')
        p, o = stamped.pose.position, stamped.pose.orientation
        return np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w])

    def robot_description(self):
        """Return the robot_description parameter of move_group (read only)."""
        if self._parameters is None:
            self._parameters = self.create_client(GetParameters, PARAMETER_SERVICE)
        if not self._parameters.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            raise RuntimeError(f'{PARAMETER_SERVICE} is not available')
        request = GetParameters.Request()
        request.names = ['robot_description']
        response = self._wait(self._parameters.call_async(request), PARAMETER_SERVICE)
        return response.values[0].string_value


def run_text(command):
    """Run a read-only command (git, dpkg-query) and return its output, or 'unknown'."""
    try:
        done = subprocess.run(command, capture_output=True, text=True, check=True, timeout=20)
        return done.stdout.strip() or 'unknown'
    except (OSError, subprocess.SubprocessError):
        return 'unknown'


def record_fixture(checker, path, overwrite, argv):
    out = Path(path)
    if out.exists() and not overwrite:
        raise RuntimeError(f'{out} exists (use --overwrite to record it again)')
    urdf_text = installed_urdf()
    chain = UrdfChain(urdf_text)
    lower, upper = chain.joint_limits()
    poses = model_check.fixture_poses(lower, upper)

    # MoveIt must have loaded the same kinematics as the installed URDF.
    loaded = UrdfChain(checker.robot_description())
    for name, q in poses:
        a, b = chain.fk(q), loaded.fk(q)
        if np.linalg.norm(a[0] - b[0]) > 1e-9 or model_check.rotation_angle(a[1], b[1]) > 1e-9:
            raise RuntimeError(f'the robot_description of move_group differs from the installed '
                               f'URDF (pose {name}): not recording')
    print(f'robot_description of move_group = installed URDF ({len(poses)} poses): ok')

    rows = []
    for name, q in poses:
        position, quaternion = checker.moveit_fk(q)
        rows.append(model_check.FixtureRow(name, q, position, quaternion))
    status = run_text(['git', 'status', '--porcelain'])
    meta = {
        'urdf_file': 'share/so_arm101_description/urdf/so101_new_calib.urdf (installed)',
        'urdf_sha256': model_check.urdf_sha256(urdf_text),
        'moveit_version': 'ros-humble-moveit-core ' + run_text(
            ['dpkg-query', '-W', '-f=${Version}', 'ros-humble-moveit-core']),
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        'recorded_utc': datetime.datetime.now(datetime.timezone.utc).strftime(
            '%Y-%m-%dT%H:%M:%SZ'),
        'git_commit': run_text(['git', 'rev-parse', 'HEAD']),
        'git_dirty': 'unknown' if status == 'unknown' else str(bool(status)).lower(),
        'tool_sha256': model_check.urdf_sha256(Path(__file__).read_bytes()),
        'seed': model_check.FIXTURE_SEED,
        'frame_id': model_check.FIXTURE_FRAME,
        'link': model_check.FIXTURE_LINK,
        'rows': len(rows),
        'command': ' '.join(argv),
    }
    model_check.write_fixture(out, meta, rows)
    print(f'wrote {out} ({len(rows)} poses)')


class Comparison:
    """Builds the rows of the table: the model, the chain of the URDF and MoveIt."""

    def __init__(self, checker, chain, q_min, q_max):
        self.checker, self.chain, self.q_min, self.q_max = checker, chain, q_min, q_max
        self.axis = model_check.approach_axis_in_tip(chain.fk(np.zeros(5))[1])

    def fk_row(self, case, q, tolerance):
        position, quaternion = self.checker.moveit_fk(q)
        model_xyz, model_pitch, _ = analytic_ik.fk(q)
        chain_xyz, chain_rotation = self.chain.fk(q)
        rotation = model_check.quaternion_to_matrix(quaternion)
        pitch = model_check.approach_pitch(rotation, q[0], self.axis)
        pitch_diff = analytic_ik.normalize_angle(model_pitch - pitch)
        return CaseRow(case, q, model_xyz, position, chain_xyz, math.degrees(pitch_diff),
                       math.degrees(model_check.rotation_angle(chain_rotation, rotation)),
                       tolerance)

    def ik_row(self, case, q_now, target, pitch=None, roll=None, tolerance=IK_TOLERANCE_MM):
        try:
            q = analytic_ik.solve_ik(*target, q_now, pitch=pitch, roll=roll,
                                     q_min=self.q_min, q_max=self.q_max)
        except analytic_ik.IKError as error:
            return CaseRow(case, note=f'IK failed: {type(error).__name__}: {error}')
        position, quaternion = self.checker.moveit_fk(q)
        chain_xyz, chain_rotation = self.chain.fk(q)
        orient = model_check.rotation_angle(
            chain_rotation, model_check.quaternion_to_matrix(quaternion))
        return CaseRow(case, q, np.asarray(target), position, chain_xyz, None,
                       math.degrees(orient), tolerance)


def build_rows(checker, args):
    chain = UrdfChain(installed_urdf())
    lower, upper = chain.joint_limits()
    margin = args.ik_range_margin
    compare = Comparison(checker, chain, lower - margin, upper + margin)
    q_now = checker.arm_joint_angles()
    cases = set(args.cases.split(','))
    rows = []
    if 'now' in cases:
        rows.append(compare.fk_row('now', q_now, args.tolerance))
    base, _, _ = analytic_ik.fk(q_now)
    if args.target is not None:
        rows.append(compare.ik_row('target', q_now, np.array(args.target) / 1000.0,
                                   args.pitch, args.roll, args.ik_tolerance))
    elif 'offset' in cases:
        rows.append(compare.ik_row('now (IK)', q_now, base, args.pitch, args.roll,
                                   args.ik_tolerance))
        for axis, label in enumerate('xyz'):
            for sign in (1, -1):
                offset = np.zeros(3)
                offset[axis] = sign * OFFSET_MM / 1000.0
                name = f'target{sign * OFFSET_MM:+.0f}{label}'
                rows.append(compare.ik_row(name, q_now, base + offset, args.pitch, args.roll,
                                           args.ik_tolerance))
    if 'roll' in cases:
        start, stop, step = args.roll_sweep
        for roll in np.arange(start, stop + step / 2.0, step):
            q = np.array(q_now)
            q[4] = roll
            rows.append(compare.fk_row(f'roll={roll:+.2f}', q, args.tolerance))
            xyz, pitch, _ = analytic_ik.fk(q)
            rows.append(compare.ik_row(f'roll={roll:+.2f} (IK)', q_now, xyz, pitch, roll,
                                       args.ik_tolerance))
    return rows


def parse_args(argv):
    ap = argparse.ArgumentParser(description='解析 FK・IK と MoveIt の FK の差の表（読み取りだけ）')
    ap.add_argument('--joint-states-topic', default='/joint_states')
    ap.add_argument('--cases', default='now,offset,roll', help='now,offset,roll のうち実行するもの')
    ap.add_argument('--target', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                    help='目標の先端 [mm]（base_link）。指定すると ±10 mm の 6 方向の代わりになる')
    ap.add_argument('--pitch', type=float, help='IK の pitch [rad]（既定: 今の姿勢のまま）')
    ap.add_argument('--roll', type=float, help='IK の roll [rad]（既定: 今の姿勢のまま）')
    ap.add_argument('--roll-sweep', type=float, nargs=3, default=(-2.0, 2.0, 0.5),
                    metavar=('START', 'STOP', 'STEP'), help='roll の掃引 [rad]')
    ap.add_argument('--ik-range-margin', type=float, default=0.02, metavar='RAD',
                    help='IK の関節範囲を URDF の範囲から広げる量（既定 0.02 rad）')
    ap.add_argument('--tolerance', type=float, default=NOW_TOLERANCE_MM, help='FK の差 [mm]')
    ap.add_argument('--ik-tolerance', type=float, default=IK_TOLERANCE_MM, help='往復の差 [mm]')
    ap.add_argument('--csv', metavar='FILE', help='表を CSV でも書く')
    ap.add_argument('--record-fixture', metavar='FILE',
                    help='MoveIt の FK を 105 姿勢で記録する（モックで。試験の fixture）')
    ap.add_argument('--overwrite', action='store_true', help='--record-fixture で既存のファイルを上書き')
    args = ap.parse_args(argv)
    if not set(args.cases.split(',')) <= {'now', 'offset', 'roll'}:
        ap.error('--cases is a comma separated list of now, offset, roll')
    if args.roll_sweep[2] <= 0.0:
        ap.error('the step of --roll-sweep must be positive')
    return args


def main(argv):
    args = parse_args(argv[1:])
    rclpy.init()
    checker = Checker(args.joint_states_topic)
    code = 0
    try:
        checker.assert_nothing_is_sent()
        print('読み取りだけ: /joint_states を購読し、/compute_fk を呼びます。何も送りません。')
        checker.wait_for_fk_service()
        if args.record_fixture:
            record_fixture(checker, args.record_fixture, args.overwrite, argv)
        else:
            rows = build_rows(checker, args)
            print('\n'.join(model_check.table_lines(rows)))
            if args.csv:
                with open(args.csv, 'w', newline='') as stream:
                    csv.writer(stream).writerows(model_check.table_csv_rows(rows))
            if any(model_check.is_over(r) for r in rows):
                code = 2
    except RuntimeError as error:
        print(f'error: {error}', file=sys.stderr)
        code = 1
    finally:
        checker.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main(sys.argv))
