# Copyright 2026 hiro
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
VR follow node (P6; docs/p6_plan.md): a thin ROS shell around follow_machine.Follower.

Input : sensor_msgs/Joy (buttons) and geometry_msgs/PoseStamped (the hand, base_link or a frame
        that TF can turn into base_link).
Output: one-point trajectories to /arm_controller/joint_trajectory (only while tracking), the
        target pose, the gripper amount (std_msgs/Float64), the state (std_msgs/String).
The gripper is moved through the action of gripper_controller, only at a button edge.

Nothing is decided here: the machine decides and this node only reads the topics, calls update()
once per tick and sends what the Output says. Only 'scale' can be changed while the node runs.
"""

import dataclasses
import re
import sys
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64, String
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import follow_calc as fc
from . import follow_machine as fm

BASE_FRAME = 'base_link'
JOY_TOPIC = '/vr/right/joy'
POSE_TOPIC = '/vr/right/pose'
TRAJECTORY_TOPIC = '/arm_controller/joint_trajectory'
TARGET_TOPIC = '/vr_follow/target_pose'
GRIPPER_TOPIC = '/vr_follow/gripper'
STATUS_TOPIC = '/vr_follow/status'
GRIPPER_ACTION = '/gripper_controller/gripper_cmd'
JOINT_STATES = '/joint_states'
CONTROLLER_STATE = '/arm_controller/controller_state'
# Goals of these actions (another user of the controller) stop the follow (docs/p6_plan.md 2-4).
STATUS_TOPICS = (
    '/arm_controller/follow_joint_trajectory/_action/status',
    '/move_action/_action/status',
    '/execute_trajectory/_action/status',
)
ACTIVE_STATUSES = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                   GoalStatus.STATUS_CANCELING)
STATUS_RATE_HZ = 20.0
MAX_PENDING_POSES = 2000  # about 20 s of a 100 Hz hand: more than a tick ever collects

# parameters that are not in FollowConfig (names of topics and buttons)
BUTTON_PARAMETERS = {'button_clutch': 0, 'button_open': 1, 'button_close': 2}


def config_fields():
    """Return the names of the FollowConfig values that are parameters of the node."""
    return [f.name for f in dataclasses.fields(fc.FollowConfig)]


def quaternion_message_to_matrix(q):
    """Return the rotation matrix of a geometry_msgs Quaternion (ValueError if invalid)."""
    return fc.quaternion_to_matrix(q.x, q.y, q.z, q.w)


READ_ONLY_PATTERN = re.compile(r'name=["\']read_only["\']\s*>\s*([^<]*?)\s*<', re.IGNORECASE)


def description_is_read_only(text):
    """
    Return True if a robot_description text says that the arm driver is read_only.

    The value is read without regard to case (xacro writes a Python bool: True), spaces
    around it are allowed, and every read_only parameter must say true. Any other value, or
    no read_only parameter, is not read_only.
    """
    values = [v.lower() for v in READ_ONLY_PATTERN.findall(text)]
    return bool(values) and all(v == 'true' for v in values)


def format_status(output):
    """Return the text of the state topic: STATE REASON key=value ..."""
    parts = [output.state, output.reason.replace(' ', '_') if output.reason else '-']
    for key in sorted(output.detail):
        value = output.detail[key]
        if key == 'message':
            continue
        parts.append('%s=%s' % (key, ('%.4g' % value) if isinstance(value, float) else value))
    return ' '.join(parts)


class VrFollowNode(Node):
    """Reads the inputs, runs the Follower once per tick and sends its output."""

    def __init__(self):
        super().__init__('vr_follow')
        defaults = fc.FollowConfig()
        for name in config_fields():
            default = getattr(defaults, name)
            if name == 'input_rotation_rpy_deg':
                default = [float(v) for v in default]
            self.declare_parameter(name, default)
        for name, default in BUTTON_PARAMETERS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('joy_topic', JOY_TOPIC)
        self.declare_parameter('pose_topic', POSE_TOPIC)
        self.config = self._read_config()
        if self.config.floor_z < fc.FLOOR_SAFE:
            # a floor under 0.045 m is only for an arm that cannot move (c0 of p6_real_check.md)
            if not self._arm_is_read_only():
                raise RuntimeError(
                    'floor_z %.3f m is under %.3f m but the arm is not confirmed read_only (the '
                    'robot_description of /robot_state_publisher): refused' % (
                        self.config.floor_z, fc.FLOOR_SAFE))
            self.get_logger().warning(
                'floor_z %.3f m (default %.3f m): the arm is read_only, so it cannot move. Use '
                'the default floor from the writing mode on.' % (
                    self.config.floor_z, fc.FLOOR_SAFE))
        self.buttons = {name: int(self.get_parameter(name).value) for name in BUTTON_PARAMETERS}
        if any(index < 0 for index in self.buttons.values()):
            raise ValueError('the button numbers must be 0 or more')
        self.follower = fm.Follower(self.config)

        self._joy = None
        self._joy_time = None
        self._pose = None
        self._pose_time = None
        self._pending = []
        self._q_meas = self._q_meas_time = None
        self._q_desired = self._q_desired_time = None
        self._status = {}
        self._last_status = None
        self._warned = set()
        self._text = 'IDLE START_WAIT'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Joy, self.get_parameter('joy_topic').value, self._on_joy, 10)
        self.create_subscription(PoseStamped, self.get_parameter('pose_topic').value,
                                 self._on_pose, 100)
        self.create_subscription(JointState, JOINT_STATES, self._on_joint_state, 10)
        self.create_subscription(JointTrajectoryControllerState, CONTROLLER_STATE,
                                 self._on_controller_state, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in STATUS_TOPICS:
            self.create_subscription(
                GoalStatusArray, topic, lambda msg, topic=topic: self._status.update({topic: msg}),
                latched)
        self.trajectory_pub = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)
        self.target_pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 10)
        self.gripper_pub = self.create_publisher(Float64, GRIPPER_TOPIC, latched)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, latched)
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_timer(self.config.period, self._tick)
        self.create_timer(1.0 / STATUS_RATE_HZ, self._publish_status)
        self.get_logger().info(
            'vr_follow started: rate %.1f Hz, point_time %.3f s, scale %.2f, clutch %s '
            '(buttons %s). Nothing is sent until the clutch is pressed.' % (
                self.config.rate_hz, self.config.point_time_s, self.config.scale,
                self.config.clutch_mode, self.buttons))

    def _arm_is_read_only(self):
        """Return True if the robot_description says that the arm driver is read_only."""
        client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            return False
        request = GetParameters.Request()
        request.names = ['robot_description']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or not future.result().values:
            return False
        text = future.result().values[0].string_value
        return description_is_read_only(text)

    # ----------------------------------------------------------------- parameters

    def _read_config(self):
        values = {}
        for name in config_fields():
            value = self.get_parameter(name).value
            if name == 'input_rotation_rpy_deg':
                value = tuple(float(v) for v in value)
            values[name] = value
        return fc.FollowConfig(**values)

    def _on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name != 'scale':
                return SetParametersResult(
                    successful=False, reason='only scale can be changed while the node runs')
            try:
                self.follower.set_scale(float(parameter.value))
            except (TypeError, ValueError) as error:
                return SetParametersResult(successful=False, reason=str(error))
            self.get_logger().info('scale is now %g' % self.follower.scale)
        return SetParametersResult(successful=True)

    # --------------------------------------------------------------------- inputs

    def _warn_once(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warning(text)

    def _on_joy(self, msg):
        def level(name):
            index = self.buttons[name]
            if index >= len(msg.buttons):
                self._warn_once(name, '%s = %d but the Joy has only %d buttons: treated as '
                                'released' % (name, index, len(msg.buttons)))
                return False
            return bool(msg.buttons[index])
        self._joy = fm.JoyState(level('button_clutch'), level('button_open'),
                                level('button_close'))
        self._joy_time = time.monotonic()

    def _on_pose(self, msg):
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        try:
            r = quaternion_message_to_matrix(msg.pose.orientation)
        except ValueError as error:
            self._warn_once('quaternion', 'a pose with a bad orientation is dropped: %s' % error)
            return
        if not np.all(np.isfinite(p)):
            self._warn_once('position', 'a pose with a non-finite position is dropped')
            return
        frame = msg.header.frame_id
        if frame not in ('', BASE_FRAME):
            try:
                transform = self.tf_buffer.lookup_transform(BASE_FRAME, frame, Time())
            except tf2_ros.TransformException as error:
                self._warn_once('tf', 'no transform %s -> %s: the pose is dropped (the follow '
                                'goes STALE): %s' % (frame, BASE_FRAME, error))
                return
            t = transform.transform
            r_frame = quaternion_message_to_matrix(t.rotation)
            p = r_frame @ p + np.array([t.translation.x, t.translation.y, t.translation.z])
            r = r_frame @ r
        self._pose = (p, r)
        self._pose_time = time.monotonic()
        self._pending.append((p, r))
        if len(self._pending) > MAX_PENDING_POSES:
            del self._pending[:len(self._pending) - MAX_PENDING_POSES]

    def _on_joint_state(self, msg):
        q = self._by_name(msg.name, msg.position)
        if q is not None:
            self._q_meas, self._q_meas_time = q, time.monotonic()

    def _on_controller_state(self, msg):
        # Humble: reference is the command of the controller (desired is a deprecated alias)
        q = self._by_name(msg.joint_names, msg.reference.positions)
        if q is not None:
            self._q_desired, self._q_desired_time = q, time.monotonic()

    @staticmethod
    def _by_name(names, positions):
        names = list(names)
        if not all(joint in names for joint in fc.JOINT_NAMES) or len(positions) < len(names):
            return None
        q = np.array([positions[names.index(joint)] for joint in fc.JOINT_NAMES], dtype=float)
        return q if np.all(np.isfinite(q)) else None

    def _busy(self):
        reasons = []
        for topic in STATUS_TOPICS:
            msg = self._status.get(topic)
            if msg is None:
                continue
            count = sum(1 for s in msg.status_list if s.status in ACTIVE_STATUSES)
            if count:
                reasons.append('%s: %d active goal(s)' % (topic, count))
        return tuple(reasons)

    # ----------------------------------------------------------------------- tick

    def _tick(self):
        now = time.monotonic()
        poses, self._pending = self._pending, []
        inp = fm.Inputs(
            pose=self._pose, pose_time=self._pose_time, poses=poses, joy=self._joy,
            joy_time=self._joy_time, q_meas=self._q_meas, q_meas_time=self._q_meas_time,
            q_desired=self._q_desired, q_desired_time=self._q_desired_time, busy=self._busy())
        output = self.follower.update(now, inp)
        if output.point is not None:
            self._send_point(output.point)
        if output.target_p is not None:
            self._send_target(output)
        if output.gripper is not None:
            self._send_gripper(output.gripper)
        text = format_status(output)
        head = (output.state, output.reason)
        if head != self._last_status:
            if output.state in (fm.HOLD_RANGE, fm.BUSY, fm.BLOCKED) or (
                    output.state == fm.IDLE and output.reason.startswith('REFUSED')):
                self.get_logger().error('%s %s' % (text, output.detail.get('message', '')))
            else:
                self.get_logger().info(text)
            self._last_status = head
            self._publish(text)  # a change is published at once (the timer republishes)
        self._text = text

    def _publish_status(self):
        self._publish(self._text)

    def _publish(self, text):
        self.status_pub.publish(String(data=text))

    def _send_point(self, q):
        msg = JointTrajectory()  # stamp 0: the controller starts it at its next update
        msg.joint_names = list(fc.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        seconds = self.config.point_time_s
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))
        msg.points = [point]
        self.trajectory_pub.publish(msg)

    def _send_target(self, output):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(v) for v in output.target_p)
        x, y, z, w = fc.matrix_to_quaternion(output.target_r)
        msg.pose.orientation.x, msg.pose.orientation.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = z, w
        self.target_pub.publish(msg)

    def _send_gripper(self, amount):
        joint = fc.gripper_joint(amount, self.config)
        self.gripper_pub.publish(Float64(data=float(amount)))
        if not self.gripper_client.server_is_ready():
            self.get_logger().error('the gripper action %s is not available: not sent' %
                                    GRIPPER_ACTION)
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(joint)
        goal.command.max_effort = 0.0
        self.get_logger().info('gripper amount %.1f -> position %.3f rad' % (amount, joint))
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self._on_gripper_goal)

    def _on_gripper_goal(self, future):
        handle = future.result()
        if handle is not None and not handle.accepted:
            self.get_logger().error('the gripper goal was rejected')


def main(args=None):
    """Run the node until Ctrl-C (SIGINT). Nothing is sent while the clutch is released."""
    rclpy.init(args=args)
    exit_code = 0
    node = None
    try:
        node = VrFollowNode()
        rclpy.spin(node)
    except (RuntimeError, ValueError) as error:
        print('vr_follow stopped: %s' % error, file=sys.stderr)
        exit_code = 2
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
