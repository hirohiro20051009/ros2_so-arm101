#!/usr/bin/env python3
"""解析 IK で関節の目標を出し、MoveIt に計画だけさせて、検査した軌道を実行する（大きな移動。docs/large_move_plan.md）。

  # 何も送らない（クライアントも作らない）。読み取り専用の起動でも動く（desired は無くてよい）
  python3 tools/ik_move_run.py --dry-run --goal 0.200 0.000 0.060
  # 実機: 開始の確認 → 計画 → 計画の表示 → 送信の確認（それぞれ Enter。30 秒で時間切れ）
  python3 tools/ik_move_run.py --goal 0.200 0.000 0.060
  python3 tools/ik_move_run.py --offset 0.03 0 0
  python3 tools/ik_move_run.py --joints-deg 0 -22 30 83 0

前提: `source install/setup.bash` の後。--goal は base_link の座標 [m]、--offset は今の先端（測った位置の FK）
からの差 [m]、--joints-deg は 5 関節の角度 [度]（--pitch-deg・--roll-deg は --goal と --offset のときだけ。
省略は今の姿勢の値。度であってラジアンではない。各値は ±180 の内側）。

【実機を動かす。ユーザーだけが実行する（Claude Code は実機に対して実行しない）】
- 動かすのは MoveIt が計画した軌道だけ。検査（目標・軌道の最初の点と最後の点・速さ・長さ・全点の関節範囲と床・
  移動量の上限）に通ったものを /execute_trajectory で実行する。Scaling は 0.1 固定（引数なし）。--yes は無い。
- 送る直前に毎回: 入力の読み直し、測った位置と desired が計画のときから動いていないこと、他のアクションの
  アクティブなゴール、目標と軌道の再検査。確認は 2 回（開始、送信）で、待ちは各 30 秒。
- ゴールをキャンセルしない。Ctrl-C は何も送らず、何も取り消さず、終了する（終了コード 130）。進行中の
  ゴールは軌道の最後まで進む（約 6 秒まで）。急ぐときは手順書の止め方 S5・S6（腕が落ちる。支えること）。
- SUCCEEDED は到達の根拠にしない（open_loop_control: true）。止まってから測った関節角で判定する。
- 終了コード: 0 REACHED（先端の誤差 10 mm 以内）、2 停止、3 開始の拒否、4 SHORT（10 mm 超）、130 Ctrl-C。
- CSV（--csv。既定は ik_move_<日時>.csv。--dry-run では作らない）に 1 回の実行 1 行を書く。Ctrl-C（130）や
  停止のときも 1 行書く（stage: 最後の段階、goal_sent・goal_in_flight: ゴールを送ったか・進行中か、
  goal_cancelled: 常に False、q_meas_last: 最後に読んだ関節角）。Ctrl-C では何も読まず、何も待たない。
- すでに目標にいて MoveIt の計画が 1 点だけのときは ALREADY_AT_GOAL（終了コード 2）で止まる。
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
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from so_arm101_ik import ik_move
from so_arm101_ik import step_runner as sr

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
        super().__init__('ik_move_run')
        self._csv = csv_writer
        self._csv_file = csv_file
        self._q_meas = None
        self._meas_at = None
        self._q_desired = None
        self._desired_at = None
        self._status = {}
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

    # --- io（MoveLoop が使う）
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
        print(text, flush=True)

    def log_row(self, row):
        if self._csv is not None:
            self._csv.writerow(row)
            self._csv_file.flush()


def _spin_until(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.done()


class MoveItSender:
    """MoveIt に plan_only で計画させ（動かさない）、検査した軌道を ExecuteTrajectory で実行する。"""

    def __init__(self, node):
        self.node = node
        self.move_client = ActionClient(node, MoveGroup, MOVE_ACTION)
        self.execute_client = ActionClient(node, ExecuteTrajectory, EXECUTE_ACTION)

    def plan(self, q_goal, tolerance):
        spec = sr.build_plan_goal_spec(q_goal, sr.JOINT_NAMES, tolerance)
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
            return ik_move.PlanResult(False, 'no action server ' + MOVE_ACTION)
        future = self.move_client.send_goal_async(goal)
        if not _spin_until(self.node, future, sr.ACCEPT_TIMEOUT):
            return ik_move.PlanResult(False, 'the plan was not accepted in time')
        handle = future.result()
        if not handle.accepted:
            return ik_move.PlanResult(False, 'the plan request was rejected')
        result_future = handle.get_result_async()
        if not _spin_until(self.node, result_future, spec.planning_time + sr.RESULT_MARGIN):
            return ik_move.PlanResult(False, 'no plan in time')
        planned = result_future.result().result
        if planned.error_code.val != 1:
            return ik_move.PlanResult(False, 'planning error %d' % planned.error_code.val)
        trajectory = planned.planned_trajectory
        points = []
        for p in trajectory.joint_trajectory.points:
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            velocities = tuple(p.velocities) if len(p.velocities) == 5 else None
            points.append(sr.TrajPoint(t, tuple(p.positions), velocities))
        return ik_move.PlanResult(True, '', tuple(points),
                                  tuple(trajectory.joint_trajectory.joint_names), trajectory)

    def execute(self, planned, seconds):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = planned.payload
        if not self.execute_client.wait_for_server(timeout_sec=sr.ACCEPT_TIMEOUT):
            return sr.ExecResult(False, 'no action server ' + EXECUTE_ACTION)
        future = self.execute_client.send_goal_async(goal)
        if not _spin_until(self.node, future, sr.ACCEPT_TIMEOUT):
            return sr.ExecResult(False, 'the execution was not accepted in time')
        handle = future.result()
        if not handle.accepted:
            return sr.ExecResult(False, 'the execution was rejected')
        result_future = handle.get_result_async()
        if not _spin_until(self.node, result_future, seconds + sr.RESULT_MARGIN):
            return sr.ExecResult(False, 'no result in time (the goal is NOT cancelled)')
        wrapped = result_future.result()
        ok, text = sr.classify_goal_result('moveit', wrapped.status, wrapped.result.error_code.val)
        return sr.ExecResult(ok, text)


def parse_args(argv):
    ap = argparse.ArgumentParser(description='解析 IK の目標を MoveIt で計画して動かす（実機を動かす）')
    ap.add_argument('--goal', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                    help='目標の先端（base_link、m）')
    ap.add_argument('--offset', type=float, nargs=3, metavar=('DX', 'DY', 'DZ'),
                    help='今の先端（測った位置の FK）からの差 [m]')
    ap.add_argument('--joints-deg', type=float, nargs=5, metavar='DEG',
                    help='5 関節の目標 [度]（pan lift elbow wrist_flex roll。±180 の内側）')
    ap.add_argument('--pitch-deg', type=float, help='目標の pitch [度]（省略は今の姿勢）')
    ap.add_argument('--roll-deg', type=float, help='目標の roll [度]（省略は今の姿勢）')
    ap.add_argument('--moveit-tolerance', type=float, default=sr.MOVEIT_TOLERANCE,
                    help='JointConstraint の許容 [rad]（既定 0.001、上限 0.01）')
    ap.add_argument('--dry-run', action='store_true',
                    help='何も送らない・計画もしない（クライアントも作らない）')
    ap.add_argument('--csv', metavar='FILE', help='1 回の実行 1 行の記録（既定 ik_move_<日時>.csv）')
    return ap.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    try:  # ノードやクライアントを作る前に、引数の組み合わせを検査する
        ik_move.validate_move_options(args.goal, args.offset, args.joints_deg, args.pitch_deg,
                                      args.roll_deg, args.moveit_tolerance)
    except ValueError as error:
        print('refused: %s' % error, file=sys.stderr)
        return sr.EXIT_STOPPED
    csv_file = csv_writer = None
    if not args.dry_run:
        name = args.csv or datetime.datetime.now(datetime.timezone.utc).strftime(
            'ik_move_%Y%m%dT%H%M%SZ.csv')
        csv_file = open(name, 'w', newline='')
        csv_writer = csv.DictWriter(csv_file, fieldnames=list(ik_move.MOVE_CSV_FIELDS))
        csv_writer.writeheader()
        print('record: %s' % name)
    rclpy.init()
    node = RunNode(csv_writer, csv_file)
    code = sr.EXIT_STOPPED
    try:
        sender = None if args.dry_run else MoveItSender(node)
        loop = ik_move.MoveLoop(node, sender, goal=args.goal, offset=args.offset,
                                joints_deg=args.joints_deg, pitch_deg=args.pitch_deg,
                                roll_deg=args.roll_deg, moveit_tolerance=args.moveit_tolerance,
                                dry_run=args.dry_run)
        result = loop.run()
        print('exit: %s (code %d), moves sent: %d' % (result.reason, result.exit_code,
                                                      result.sent))
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
