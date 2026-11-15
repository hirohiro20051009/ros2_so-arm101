#!/usr/bin/env python3
"""解析 IK の差分モードを、1 ステップずつ「動かす → 止まる → 測る」で回す（docs/analytic_diff_plan.md 7.14）。

  # 実機: V1・V2 の確認（何も送らない。クライアントも作らない）
  python3 tools/diff_mode_run.py --route direct --dry-run --assume-desired-is-measured --offset 0.002 0 0
  python3 tools/diff_mode_run.py --route direct --dry-run --offset 0.002 0 0
  # 実機: V3 以降（1 ステップごとに Enter で確認する）
  python3 tools/diff_mode_run.py --route direct --max-steps 1 --offset 0.002 0 0
  python3 tools/diff_mode_run.py --route moveit --goal 0.180 0.0 0.055

前提: `source install/setup.bash` の後。--route は必須（既定なし。direct は JTC に直接、moveit は MoveIt が計画し、
検査した軌道を実行する）。--offset は今の先端（測った位置の FK）からの差 [m]、--goal は base_link の座標 [m]。

【実機を動かす。ユーザーだけが実行する（Claude Code は実機に対して実行しない）】
- 送る直前に毎回: 入力の読み直しと古さ・測った位置が動いていないこと・desired と前回の指令の一致・他のアクションの
  アクティブなゴール・verify_command をやり直す。Enter の待ちは 30 秒。--auto N は最初のステップのあと N ステップを
  Enter なしで送る（AT_LIMIT と HOLD は Enter に戻る）。--yes は無い。
- ゴールをキャンセルしない（キャンセルは測った位置で保持して垂れる）。Ctrl-C は何も送らず、何も取り消さず、終了する
  （終了コード 130。進行中のゴールは指令まで進む）。急ぐときは手順書の止め方 S1〜S6。
- SUCCEEDED は到達の根拠にしない（open_loop_control: true）。止まってから測った関節角で判定する。
- moveit は /move_action に plan_only で計画させ、その軌道を検査してから /execute_trajectory で実行する。
  軌道の最初の点が q_desired から 0.005 rad を超えてずれるときは実行しない。Scaling は 0.1 固定。
- CSV（--csv。既定は diff_mode_run_<日時>.csv）に 1 ステップ 1 行を書く。
"""
import argparse
import csv
import datetime
import os
import select
import sys
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from so_arm101_ik import step_runner as sr
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_STATES = '/joint_states'
CONTROLLER_STATE = '/arm_controller/controller_state'
JTC_ACTION = '/arm_controller/follow_joint_trajectory'
MOVE_ACTION = 'move_action'
EXECUTE_ACTION = 'execute_trajectory'
# 他のアクションのアクティブなゴールを見る（送る直前に毎回。自分のゴールは終わっている）
STATUS_TOPICS = (
    JTC_ACTION + '/_action/status',
    '/' + MOVE_ACTION + '/_action/status',
    '/' + EXECUTE_ACTION + '/_action/status',
)


class RunNode(Node):
    """入力（/joint_states・controller_state・各 _action/status）と、端末の確認を受け持つ（io）。"""

    def __init__(self, csv_writer, csv_file):
        super().__init__('diff_mode_run')
        self._csv = csv_writer
        self._csv_file = csv_file
        self._q_meas = None
        self._meas_at = None
        self._q_desired = None
        self._desired_at = None
        self._status = {}
        self.said = []
        self._reader = sr.LineReader()  # 標準入力の未処理の分（1 回の確認で 1 行だけ使う）
        self.create_subscription(JointState, JOINT_STATES, self._on_joint_state, 10)
        self.create_subscription(JointTrajectoryControllerState, CONTROLLER_STATE,
                                 self._on_controller_state, 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in STATUS_TOPICS:
            self.create_subscription(
                GoalStatusArray, topic, lambda msg, topic=topic: self._status.update({topic: msg}),
                qos)

    def _on_joint_state(self, msg):
        self._q_meas = sr.positions_by_name(msg.name, msg.position)
        self._meas_at = time.monotonic() if self._q_meas is not None else None

    def _on_controller_state(self, msg):
        # Humble: reference が正（desired は非推奨の別名）
        self._q_desired = sr.positions_by_name(msg.joint_names, msg.reference.positions)
        self._desired_at = time.monotonic() if self._q_desired is not None else None

    # --- io（StepLoop が使う）
    def now(self):
        return time.monotonic()

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def read_inputs(self):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.0)
        return sr.Inputs(self._q_desired, self._desired_at, self._q_meas, self._meas_at)

    def other_active_goals(self):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.0)
        active = []
        for topic in STATUS_TOPICS:
            msg = self._status.get(topic)
            if msg is None:
                continue
            count = sum(1 for s in msg.status_list if s.status in sr.ACTIVE_STATUSES)
            if count:
                active.append('%s: %d active goal(s)' % (topic, count))
        return active

    def _poll(self):
        """標準入力から届いた分を返す（なければ None。EOF は空文字）。待つ間も入力を受け取り続ける。"""
        rclpy.spin_once(self, timeout_sec=0.0)
        ready, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not ready:
            return None
        return os.read(sys.stdin.fileno(), 4096).decode(errors='replace')

    def confirm(self, text, timeout):
        print(text, flush=True)
        return sr.confirm_answer(self._reader, self._poll, time.monotonic, timeout)

    def say(self, text):
        self.said.append(text)
        print(text, flush=True)

    def log_row(self, row):
        if self._csv is not None:
            self._csv.writerow(row)
            self._csv_file.flush()


def _spin_until(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.done()


class DirectSender:
    """JTC へ直接、q_cmd の 1 点の軌道を送る（FollowJointTrajectory）。"""

    def __init__(self, node, seconds):
        self.node = node
        self.seconds = seconds
        self.client = ActionClient(node, FollowJointTrajectory, JTC_ACTION)

    def prepare(self, q_cmd, q_desired, q_meas):
        problem = sr.check_direct_speed(q_desired, q_cmd, self.seconds)
        if problem is not None:
            return sr.Prepared(False, ('SPEED',), note=problem)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(sr.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q_cmd]
        point.time_from_start.sec = int(self.seconds)
        point.time_from_start.nanosec = int((self.seconds % 1.0) * 1e9)
        goal.trajectory.points = [point]
        return sr.Prepared(True, (), None, sr.direct_speed(q_desired, q_cmd, self.seconds),
                           self.seconds, goal)

    def execute(self, prepared):
        if not self.client.wait_for_server(timeout_sec=sr.ACCEPT_TIMEOUT):
            return sr.ExecResult(False, 'no action server ' + JTC_ACTION)
        future = self.client.send_goal_async(prepared.payload)
        if not _spin_until(self.node, future, sr.ACCEPT_TIMEOUT):
            return sr.ExecResult(False, 'the goal was not accepted in time')
        handle = future.result()
        if not handle.accepted:
            return sr.ExecResult(False, 'the goal was rejected')
        result_future = handle.get_result_async()
        if not _spin_until(self.node, result_future, prepared.duration + sr.RESULT_MARGIN):
            return sr.ExecResult(False, 'no result in time (the goal is NOT cancelled)')
        wrapped = result_future.result()
        ok, text = sr.classify_goal_result('direct', wrapped.status, wrapped.result.error_code)
        return sr.ExecResult(ok, text)


class MoveItSender:
    """MoveIt に plan_only で計画させ（動かさない）、検査した軌道を ExecuteTrajectory で実行する。"""

    def __init__(self, node, tolerance):
        self.node = node
        self.tolerance = tolerance
        self.move_client = ActionClient(node, MoveGroup, MOVE_ACTION)
        self.execute_client = ActionClient(node, ExecuteTrajectory, EXECUTE_ACTION)

    def prepare(self, q_cmd, q_desired, q_meas):
        spec = sr.build_plan_goal_spec(q_cmd, sr.JOINT_NAMES, self.tolerance)
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = spec.group
        request.num_planning_attempts = spec.planning_attempts
        request.allowed_planning_time = spec.planning_time
        request.max_velocity_scaling_factor = spec.scaling
        request.max_acceleration_scaling_factor = spec.scaling
        request.start_state.is_diff = spec.is_diff
        constraints = Constraints()
        for name, position in zip(spec.joint_names, spec.positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = spec.tolerance
            joint.tolerance_below = spec.tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = spec.plan_only
        if not self.move_client.wait_for_server(timeout_sec=sr.ACCEPT_TIMEOUT):
            return sr.Prepared(False, ('NO_SERVER',), note='no action server ' + MOVE_ACTION)
        future = self.move_client.send_goal_async(goal)
        if not _spin_until(self.node, future, sr.ACCEPT_TIMEOUT):
            return sr.Prepared(False, ('PLAN_TIMEOUT',), note='the plan was not accepted in time')
        handle = future.result()
        if not handle.accepted:
            return sr.Prepared(False, ('PLAN_REJECTED',), note='the plan request was rejected')
        result_future = handle.get_result_async()
        if not _spin_until(self.node, result_future, spec.planning_time + sr.RESULT_MARGIN):
            return sr.Prepared(False, ('PLAN_TIMEOUT',), note='no plan in time')
        planned = result_future.result().result
        if planned.error_code.val != 1:
            return sr.Prepared(False, ('PLAN_FAILED',), note='planning error %d' %
                               planned.error_code.val)
        trajectory = planned.planned_trajectory
        points = []
        for p in trajectory.joint_trajectory.points:
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            velocities = tuple(p.velocities) if len(p.velocities) == 5 else None
            points.append(sr.TrajPoint(t, tuple(p.positions), velocities))
        check = sr.check_planned_trajectory(points, trajectory.joint_trajectory.joint_names,
                                            q_cmd, q_desired, q_meas, self.tolerance)
        return sr.Prepared(check.ok, check.reasons, check, None, check.duration, trajectory)

    def execute(self, prepared):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = prepared.payload
        if not self.execute_client.wait_for_server(timeout_sec=sr.ACCEPT_TIMEOUT):
            return sr.ExecResult(False, 'no action server ' + EXECUTE_ACTION)
        future = self.execute_client.send_goal_async(goal)
        if not _spin_until(self.node, future, sr.ACCEPT_TIMEOUT):
            return sr.ExecResult(False, 'the execution was not accepted in time')
        handle = future.result()
        if not handle.accepted:
            return sr.ExecResult(False, 'the execution was rejected')
        result_future = handle.get_result_async()
        if not _spin_until(self.node, result_future, prepared.duration + sr.RESULT_MARGIN):
            return sr.ExecResult(False, 'no result in time (the goal is NOT cancelled)')
        wrapped = result_future.result()
        ok, text = sr.classify_goal_result('moveit', wrapped.status, wrapped.result.error_code.val)
        return sr.ExecResult(ok, text)


def make_sender(route, node, args):
    """--route で選んだ送り先を 1 つだけ作る（もう一方のクライアントは作らない）。"""
    if route == 'direct':
        return DirectSender(node, args.move_seconds)
    return MoveItSender(node, args.moveit_tolerance)


def parse_args(argv):
    ap = argparse.ArgumentParser(description='解析 IK の差分モードを 1 ステップずつ回す（実機を動かす）')
    ap.add_argument('--route', required=True, choices=sr.ROUTES,
                    help='direct（JTC に直接）か moveit（MoveIt が計画。検査した軌道を実行）。既定なし')
    ap.add_argument('--goal', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                    help='目標の先端（base_link、m）')
    ap.add_argument('--offset', type=float, nargs=3, metavar=('DX', 'DY', 'DZ'),
                    help='今の先端（測った位置の FK）からの差 [m]')
    ap.add_argument('--gain', type=float, default=1.0, help='誤差に掛ける倍率（0 より大きく 1 以下）')
    ap.add_argument('--max-dq', type=float, default=sr.DEFAULT_MAX_DQ, help='1 回の関節の変化 [rad]')
    ap.add_argument('--max-steps', type=int, default=10)
    ap.add_argument('--max-abs-d', type=float, default=0.99)
    ap.add_argument('--auto', type=int, metavar='N',
                    help='最初のステップのあと N ステップを Enter なしで送る（AT_LIMIT・HOLD は Enter に戻る）')
    ap.add_argument('--origin', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                    help='起点（指令側の座標）。省略すると、この実行の最初の start の位置')
    ap.add_argument('--origin-radius', type=float, default=sr.ORIGIN_RADIUS,
                    help='起点からの半径 [m]（既定・上限 0.05）')
    ap.add_argument('--move-seconds', type=float, default=sr.MOVE_SECONDS,
                    help='direct の time_from_start [s]（速度は 0.1 rad/s まで）')
    ap.add_argument('--moveit-tolerance', type=float, default=sr.MOVEIT_TOLERANCE,
                    help='moveit の JointConstraint の許容 [rad]')
    ap.add_argument('--dry-run', action='store_true', help='何も送らない（クライアントも作らない）')
    ap.add_argument('--assume-desired-is-measured', action='store_true',
                    help='--dry-run のときだけ: q_desired を測った位置とみなす（読み取り専用の起動用）')
    ap.add_argument('--csv', metavar='FILE', help='1 ステップ 1 行の記録（既定 diff_mode_run_<日時>.csv）')
    return ap.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    try:  # ノードやクライアントを作る前に、引数の組み合わせを検査する
        config = sr.validate_run_options(
            args.route, args.dry_run, args.assume_desired_is_measured, args.goal, args.offset,
            args.auto, args.origin, args.origin_radius, args.max_dq, args.max_steps,
            args.max_abs_d, args.gain, args.moveit_tolerance, args.move_seconds)
    except ValueError as error:
        print('refused: %s' % error, file=sys.stderr)
        return sr.EXIT_STOPPED
    csv_file = csv_writer = None
    if not args.dry_run:
        name = args.csv or datetime.datetime.now(datetime.timezone.utc).strftime(
            'diff_mode_run_%Y%m%dT%H%M%SZ.csv')
        csv_file = open(name, 'w', newline='')
        csv_writer = csv.DictWriter(csv_file, fieldnames=list(sr.CSV_FIELDS))
        csv_writer.writeheader()
        print('record: %s' % name)
    rclpy.init()
    node = RunNode(csv_writer, csv_file)
    code = sr.EXIT_STOPPED
    try:
        sender = None if args.dry_run else make_sender(args.route, node, args)
        loop = sr.StepLoop(node, sender, config, args.route, goal=args.goal, offset=args.offset,
                           gain=args.gain, auto=args.auto, origin=args.origin,
                           origin_radius=args.origin_radius,
                           moveit_tolerance=args.moveit_tolerance, dry_run=args.dry_run,
                           assume_desired_is_measured=args.assume_desired_is_measured)
        result = loop.run()
        print('exit: %s (code %d), steps sent: %d' % (result.reason, result.exit_code, result.sent))
        in_band = any('DRY-RUN suggestion: IN_BAND' in line for line in node.said)
        if (result.reason == 'CONVERGED' and result.sent == 0) or in_band:
            error = result.outcome.residual_norm if result.outcome is not None else float('nan')
            if in_band:  # --dry-run: the outcome is not computed; the error is in the line above
                error = float('nan')
            if error == error:
                print('目標は不感帯の内側（誤差 %.2f mm < 3 mm）。送りません。' % (error * 1000.0))
            else:
                print('目標は不感帯の内側（誤差 3 mm 未満）。送りません。')
        code = result.exit_code
    except KeyboardInterrupt:
        print('interrupted: nothing was sent or cancelled by this tool')
        code = sr.EXIT_INTERRUPTED
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if csv_file is not None:
            csv_file.close()
    return code


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
