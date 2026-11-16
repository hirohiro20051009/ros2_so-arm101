#!/usr/bin/env python3
"""C2 の 1 点を記録する（読み取り専用。腕には何も送らず、シリアルもカメラも開かない）。

  # 見え方の調査だけ（何も書かない。--csv も --label も要らない）
  python3 tools/camera_record_run.py --investigate
  # 1 点の記録: 調査 → Enter → 記録。点ごとに起動し、同じ CSV に 1 行ずつ追記する
  python3 tools/camera_record_run.py --csv c2_20261116.csv --label P01
  python3 tools/camera_record_run.py --csv c2_20261116.csv --label P02

購読するのは /joint_states、/detections、/tf（タグの位置と向き）と、新しい CSV のときだけ
camera_info と /tf_static。CSV と meta を書くのは so_arm101_camera.record の write_point だけで、
この道具は open を使わない。腕を動かすのは tools/ik_move_run.py（docs/camera_stage_plan.md の 8 章）。
終了コード: 0 成功、2 拒否または確認なし、3 タグが見えない、5 入力なし、130 Ctrl-C。
"""

import argparse
import datetime
import os
import select
import sys
import time

from ament_index_python.packages import get_package_share_directory
from apriltag_msgs.msg import AprilTagDetectionArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, JointState
from so_arm101_camera import record as rec
from so_arm101_camera import tag_mount as tm
from so_arm101_ik import step_runner as sr
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

JOINT_STATES = '/joint_states'
DETECTIONS = '/detections'
TF_TOPIC = '/tf'
CAMERA_INFO = '/camera/camera/color/camera_info'
CAMERA_LINK = 'camera_link'
OPTICAL_FRAME = 'camera_color_optical_frame'
EXTRA_TIMEOUT = 5.0  # s, wait for camera_info and the static TF of a new CSV (推測)
FLUSH_SECONDS = 0.3  # s, spin to throw away what arrived before the collecting (推測)
TF_DEPTH = 100  # depth of the /tf subscription (推測: the default depth of the broadcasters)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class RunNode(Node):
    """The io of RecordLoop: subscriptions, the clock, the terminal (nothing is published)."""

    def __init__(self, tag_id, tag_frame):
        super().__init__('camera_record_run')
        self._tag_id = tag_id
        self._tag_frame = tag_frame
        self._q = None
        self._q_at = None
        self._detections = {}
        self._transforms = {}
        self._camera_info = None
        self._warned_parent = False
        self._reader = sr.LineReader()  # unprocessed typed text (one answer uses one line)
        self.create_subscription(JointState, JOINT_STATES, self._on_joint_state, 10)
        # Best effort matches both a reliable and a best-effort publisher.
        self.create_subscription(AprilTagDetectionArray, DETECTIONS, self._on_detections,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, TF_TOPIC, self._on_tf, TF_DEPTH)
        self.create_subscription(CameraInfo, CAMERA_INFO, self._on_camera_info,
                                 qos_profile_sensor_data)

    def _on_joint_state(self, msg):
        self._q = sr.positions_by_name(msg.name, msg.position)
        self._q_at = time.monotonic() if self._q is not None else None

    def _on_detections(self, msg):
        stamp = _stamp_ns(msg.header.stamp)
        for detection in msg.detections:
            if detection.id == self._tag_id:
                self._detections[stamp] = float(detection.decision_margin)

    def _on_tf(self, msg):
        for item in msg.transforms:
            if item.child_frame_id != self._tag_frame:
                continue
            if item.header.frame_id != OPTICAL_FRAME:
                if not self._warned_parent:
                    self._warned_parent = True
                    self.get_logger().warn(
                        'the parent of %s is %s, not %s: these frames are not used'
                        % (self._tag_frame, item.header.frame_id, OPTICAL_FRAME))
                continue
            t, q = item.transform.translation, item.transform.rotation
            stamp = _stamp_ns(item.header.stamp)
            self._transforms[stamp] = ((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

    def _on_camera_info(self, msg):
        self._camera_info = msg

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    # --- io of RecordLoop
    def now(self):
        return time.monotonic()

    def wait(self, seconds):
        self._spin_for(seconds)

    def read_joints(self):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.0)
        return self._q, self._q_at

    def flush(self):
        self._spin_for(FLUSH_SECONDS)
        self._detections = {}
        self._transforms = {}

    def poll(self, seconds):
        self._detections = {}
        self._transforms = {}
        self._spin_for(seconds)
        return dict(self._detections), dict(self._transforms)

    def say(self, text):
        print(text, flush=True)

    def _poll_stdin(self):
        """Return the text typed so far (None if nothing, '' at EOF); keep receiving meanwhile."""
        rclpy.spin_once(self, timeout_sec=0.0)
        ready, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not ready:
            return None
        return os.read(sys.stdin.fileno(), 4096).decode(errors='replace')

    def confirm(self, text):
        print(text + ' (Enter = yes, anything else = no)', flush=True)
        answer = sr.confirm_answer(self._reader, self._poll_stdin, time.monotonic,
                                   sr.CONFIRM_TIMEOUT)
        return answer == 'yes'

    def wall_time(self):
        return time.time()

    def date_text(self):
        return datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')

    # --- for the meta file of a new CSV
    def fetch_camera_extra(self, timeout):
        """Wait for camera_info and camera_link to the optical frame; None if not found."""
        buffer = Buffer()
        listener = TransformListener(buffer, self)  # keeps its subscriptions while it lives
        end = time.monotonic() + timeout
        extra = None
        while extra is None and rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._camera_info is None:
                continue
            try:
                found = buffer.lookup_transform(CAMERA_LINK, OPTICAL_FRAME, Time())
            except TransformException:
                continue
            info = self._camera_info
            t, q = found.transform.translation, found.transform.rotation
            extra = rec.camera_extra(info.k, info.d, info.width, info.height,
                                     (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        del listener
        return extra


def parse_args(argv):
    config = os.path.join(get_package_share_directory('so_arm101_camera'), 'config')
    urdf = os.path.join(get_package_share_directory('so_arm101_description'), 'urdf',
                        'so101_new_calib.urdf')
    ap = argparse.ArgumentParser(description='C2 の 1 点を記録する（読み取り専用）')
    ap.add_argument('--csv', metavar='FILE', help='追記する CSV（既定名なし。記録のときは必須）')
    ap.add_argument('--label', help='点の名前（P01 など。記録のときは必須。同じ名前は拒否）')
    ap.add_argument('--investigate', action='store_true',
                    help='見え方の調査だけ（何も書かず、確認も求めない）')
    ap.add_argument('--tag-mount', default=os.path.join(config, 'tag_mount.yaml'))
    ap.add_argument('--apriltag-yaml', default=os.path.join(config, 'apriltag.yaml'))
    ap.add_argument('--realsense-yaml', default=os.path.join(config, 'realsense.yaml'))
    ap.add_argument('--urdf', default=urdf)
    args = ap.parse_args(argv)
    if not args.investigate and (not args.csv or not args.label):
        ap.error('--csv and --label are required unless --investigate is given')
    return args


def main(argv):
    args = parse_args(argv)
    try:  # every check that needs no node comes before rclpy.init
        mount = tm.load_tag_mount(args.tag_mount, args.apriltag_yaml)
        frame = tm.tag_frame_name(args.apriltag_yaml, mount.tag_id)
        chain = rec.load_chain(args.urdf)
        identity = rec.identity_from_files(args.tag_mount, args.urdf, args.apriltag_yaml,
                                           args.realsense_yaml, mount.tag_id)
        state = None
        if not args.investigate:
            state = rec.check_session(args.csv, args.label, identity)
    except (tm.TagMountError, rec.SessionError, OSError, ValueError) as error:
        print('refused: %s' % error, file=sys.stderr)
        return rec.EXIT_REFUSED
    rclpy.init()
    node = RunNode(mount.tag_id, frame)
    code = rec.EXIT_NO_INPUT
    try:
        extra = {}
        if state == 'new':
            extra = node.fetch_camera_extra(EXTRA_TIMEOUT)
            if extra is None:
                print('no camera_info or no camera_link to optical-frame TF within %.0f s: '
                      'nothing was written' % EXTRA_TIMEOUT, file=sys.stderr)
        if extra is not None:
            loop = rec.RecordLoop(node, chain, mount, args.csv, identity, extra)
            code = loop.run(args.label or 'investigate', investigate_only=args.investigate)
    except KeyboardInterrupt:
        print('interrupted: nothing was sent by this tool')
        code = sr.EXIT_INTERRUPTED
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
