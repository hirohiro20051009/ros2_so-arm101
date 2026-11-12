#!/usr/bin/env python3
"""一定時間、小さい Twist を Servo に送る（T1。docs/servo_integration_plan.md 5 章、9.1）。

  python3 tools/send_servo_twist.py --linear 0 0 -0.5 --seconds 2          # z を下げる（入力の 0.5 倍）
  python3 tools/send_servo_twist.py --linear 0 0 0.5 --seconds 2 --dry-run   # 計画だけ表示（何も送らない）

【実機を動かす。ユーザーだけが実行する】前提: use_servo:=true で起動済み、ユーザーが手順書
（docs/servo_real_check.md）のとおりに forward_position_controller へ切り替え済み。
このツールは、コントローラの切り替えをしない（start_servo・unpause_servo・change_drift_dimensions は呼ぶ）。

入力は unitless の割合（-1..1）。実速度 = 割合 × scale（servo.yaml。既定: 並進 0.01 m/s、回転 0.05 rad/s）。
安全のため、次を超える指定は拒否する（--allow-fast は成分の上限だけ 0.5 から 1.0 に緩める）:
  成分 0.5、時間 3 秒、先端の総移動量 2 cm、回転の総量 0.1 rad、frame は base_link か gripper_frame_link のみ。
Twist は、現在の関節角から求めた Jacobian の値域に射影してから送る（作れない向きの成分を取り除く）。
射影で捨てる成分が入力の 50 % 以上なら拒否する。drift（rx か ry）は pan から選んで設定する。
Twist の header.stamp は now()。終了時（Ctrl-C を含む）に、ゼロの Twist を 0.3 秒送る。
送る前に、計画を表示して Enter を待つ（--yes で省く）。
"""
import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import servo_math  # noqa: E402

ALLOWED_FRAMES = ('base_link', 'gripper_frame_link')
MAX_COMPONENT = 0.5
MAX_COMPONENT_FAST = 1.0
MAX_SECONDS = 3.0
MAX_DISTANCE = 0.02          # m
MAX_ANGLE = 0.1              # rad
MAX_LOST_FRACTION = 0.5
DEFAULT_SCALE_LINEAR = 0.01  # m/s（servo.yaml と同じ。読めなければこの値）
DEFAULT_SCALE_ROTATIONAL = 0.05


class Refused(Exception):
    """指定が安全の上限を超えている."""


def check_request(linear, angular, seconds, frame, scale_linear, scale_rotational,
                  allow_fast=False):
    """指定が上限の内側であることを確かめる。外側なら Refused."""
    limit = MAX_COMPONENT_FAST if allow_fast else MAX_COMPONENT
    values = list(linear) + list(angular)
    if any(not math.isfinite(v) for v in values) or not math.isfinite(seconds):
        raise Refused('refused: NaN / inf in the request')
    if frame not in ALLOWED_FRAMES:
        raise Refused(f'refused: frame {frame!r} (only {", ".join(ALLOWED_FRAMES)})')
    if any(abs(v) > limit for v in values):
        raise Refused(f'refused: |component| > {limit}')
    if seconds <= 0 or seconds > MAX_SECONDS:
        raise Refused(f'refused: seconds {seconds} (0 < seconds <= {MAX_SECONDS})')
    distance = float(np.linalg.norm(linear)) * scale_linear * seconds
    angle = float(np.linalg.norm(angular)) * scale_rotational * seconds
    if distance > MAX_DISTANCE:
        raise Refused(f'refused: total translation {distance * 1000:.1f} mm > '
                      f'{MAX_DISTANCE * 1000:.0f} mm')
    if angle > MAX_ANGLE:
        raise Refused(f'refused: total rotation {angle:.3f} rad > {MAX_ANGLE} rad')
    if not any(values):
        raise Refused('refused: all components are zero')


def plan_command(chain, q, linear, angular, frame, scale_linear, scale_rotational,
                 seconds):
    """現在の関節角 q から、Servo に送る入力（base_link、射影済み、割合）と計画を求める.

    frame が gripper_frame_link なら、base_link の Twist に直す（射影は base_link 基準）。
    """
    jac, tip, rotation = servo_math.jacobian(chain, q)
    physical = np.concatenate([np.asarray(linear, dtype=float) * scale_linear,
                               np.asarray(angular, dtype=float) * scale_rotational])
    if frame == 'gripper_frame_link':
        physical = np.concatenate([rotation @ physical[:3], rotation @ physical[3:]])
    projected = servo_math.project_twist(jac, physical)
    norm = float(np.linalg.norm(physical))
    lost = float(np.linalg.norm(physical - projected)) / norm if norm > 0 else 0.0
    command = np.concatenate([projected[:3] / scale_linear, projected[3:] / scale_rotational])
    if lost >= MAX_LOST_FRACTION:
        raise Refused(f'refused: {lost * 100:.0f} % of the request is in a direction the arm '
                      f'cannot make (limit {MAX_LOST_FRACTION * 100:.0f} %)')
    return {
        'command': command,                      # base_link、割合（-1..1）
        'drift': servo_math.choose_drift(float(q[0])),
        'lost_fraction': lost,
        'tip': tip,
        'expected_translation': projected[:3] * seconds,   # m
        'expected_rotation': projected[3:] * seconds,      # rad
        'condition': servo_math.reduced_condition(jac, servo_math.choose_drift(float(q[0]))),
    }


def describe(plan, seconds, rate):
    """計画を表示用の文字列にする."""
    c = plan['command']
    return '\n'.join([
        f'tip now (base_link): {np.round(plan["tip"], 4)} m',
        f'drift: {plan["drift"]} (5x5 condition number {plan["condition"]:.1f})',
        f'send for {seconds:.1f} s at {rate:.0f} Hz, frame base_link, unitless '
        f'linear={np.round(c[:3], 3)} angular={np.round(c[3:], 3)}',
        f'dropped by the projection: {plan["lost_fraction"] * 100:.1f} %',
        f'expected tip translation: {np.round(plan["expected_translation"] * 1000, 2)} mm, '
        f'rotation {np.round(plan["expected_rotation"], 4)} rad',
    ])


def _scales():
    """servo.yaml（bringup の config）から scale を読む。読めなければ既定値."""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('so_arm101_bringup'), 'config',
                            'servo.yaml')
        with open(path) as f:
            data = yaml.safe_load(f)
        return float(data['scale']['linear']), float(data['scale']['rotational'])
    except Exception:
        return DEFAULT_SCALE_LINEAR, DEFAULT_SCALE_ROTATIONAL


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--linear', type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=('X', 'Y', 'Z'),
                    help='並進の入力（-1..1 の割合。frame の座標）')
    ap.add_argument('--angular', type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=('RX', 'RY', 'RZ'), help='回転の入力（-1..1 の割合）')
    ap.add_argument('--seconds', type=float, required=True, help='送る時間 [s]（3 秒まで）')
    ap.add_argument('--frame', default='base_link', choices=ALLOWED_FRAMES)
    ap.add_argument('--drift', choices=['auto', 'rx', 'ry'], default='auto')
    ap.add_argument('--rate', type=float, default=20.0, help='送信の周波数 [Hz]（10 以上）')
    ap.add_argument('--allow-fast', action='store_true', help='成分の上限を 0.5 から 1.0 に緩める')
    ap.add_argument('--yes', action='store_true', help='Enter の確認を省く')
    ap.add_argument('--dry-run', action='store_true', help='計画だけ表示（何も送らない・設定しない）')
    args = ap.parse_args()

    scale_linear, scale_rotational = _scales()
    try:
        check_request(args.linear, args.angular, args.seconds, args.frame, scale_linear,
                      scale_rotational, args.allow_fast)
        if args.rate < 10.0:
            raise Refused('refused: rate < 10 Hz (Servo treats the input as stale after 0.1 s)')
    except Refused as e:
        sys.exit(str(e))

    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from geometry_msgs.msg import TwistStamped
    from moveit_msgs.srv import ChangeDriftDimensions
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int8
    from std_srvs.srv import Trigger

    rclpy.init()
    node = Node('send_servo_twist')
    state = {}
    status = {'seen': False, 'last': None}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: state.update(zip(m.name, m.position)), 10)

    def on_status(m):
        status['seen'] = True
        status['last'] = m.data
    node.create_subscription(Int8, '/servo_node/status', on_status, 10)

    def call(client, request, what, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            sys.exit(f'service not available: {what}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            sys.exit(f'no response: {what}')
        return future.result()

    arm = list(servo_math.ARM_JOINTS)
    end = time.time() + 5
    while rclpy.ok() and time.time() < end and not all(j in state for j in arm):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not all(j in state for j in arm):
        sys.exit('no /joint_states (is the launch running?)')
    q = np.array([state[j] for j in arm])
    if not np.all(np.isfinite(q)):
        sys.exit('/joint_states contains NaN: the driver is not healthy. Not sending anything.')

    response = call(node.create_client(GetParameters, '/robot_state_publisher/get_parameters'),
                    GetParameters.Request(names=['robot_description']), 'robot_description')
    chain = servo_math.chain_from_urdf(response.values[0].string_value)
    try:
        plan = plan_command(chain, q, args.linear, args.angular, args.frame, scale_linear,
                            scale_rotational, args.seconds)
    except Refused as e:
        sys.exit(str(e))
    if args.drift != 'auto':
        plan['drift'] = args.drift

    controllers = call(node.create_client(ListControllers, '/controller_manager/list_controllers'),
                       ListControllers.Request(), 'list_controllers')
    active = {c.name for c in controllers.controller if c.state == 'active'}
    print(describe(plan, args.seconds, args.rate))
    print(f'controllers active: {sorted(active)}')
    if 'forward_position_controller' not in active or 'arm_controller' in active:
        sys.exit('refused: forward_position_controller must be active and arm_controller inactive '
                 '(switch by hand: docs/servo_real_check.md)')
    if args.dry_run:
        print('dry-run: nothing was sent or set.')
        return
    if not args.yes:
        input('Enter to send (Ctrl-C to cancel)... ')

    flags = servo_math.drift_flags(plan['drift'])
    drift = ChangeDriftDimensions.Request(
        drift_x_translation=flags[0], drift_y_translation=flags[1], drift_z_translation=flags[2],
        drift_x_rotation=flags[3], drift_y_rotation=flags[4], drift_z_rotation=flags[5])
    if not call(node.create_client(ChangeDriftDimensions, '/servo_node/change_drift_dimensions'),
                drift, 'change_drift_dimensions').success:
        sys.exit('change_drift_dimensions failed. Not sending anything.')
    trigger = Trigger.Request()
    if not status['seen']:
        call(node.create_client(Trigger, '/servo_node/start_servo'), trigger, 'start_servo')
    call(node.create_client(Trigger, '/servo_node/unpause_servo'), trigger, 'unpause_servo')
    publisher = node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
    tip0 = plan['tip']

    def publish(values):
        m = TwistStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (float(v) for v in values[:3])
        m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = (float(v) for v in values[3:])
        publisher.publish(m)

    try:
        stop = time.time() + args.seconds
        while rclpy.ok() and time.time() < stop:
            publish(plan['command'])
            rclpy.spin_once(node, timeout_sec=1.0 / args.rate)
    finally:
        zero = np.zeros(6)
        end = time.time() + 0.3
        while time.time() < end:
            publish(zero)
            rclpy.spin_once(node, timeout_sec=0.02)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    q1 = np.array([state[j] for j in arm])
    _, tip1, _ = servo_math.jacobian(chain, q1)
    print(f'done. Servo status last={status["last"]}. tip moved {np.round((tip1 - tip0) * 1000, 2)} mm '
          f'(expected {np.round(plan["expected_translation"] * 1000, 2)} mm)')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
