"""SO-ARM101 の Servo（Diff-IK）用の運動学の計算（numpy だけ。ノード・ROS を使わない）.

docs/servo_integration_plan.md 2 章・付録の計算を関数にしたもの。

  - chain_from_urdf(): URDF の文字列から base_link -> gripper_frame_link の関節の鎖を作る
  - jacobian(): 幾何 Jacobian（6x5。base_link 基準、先端 = gripper_frame_link の原点）
  - left_null(): 作れない Twist の向き（左零空間。単位ベクトル）
  - project_twist(): Twist を、5 関節で作れる向き（Jacobian の値域）に射影する（共通の層への要求）
  - reduced_condition(): 1 行（drift）を捨てた 5x5 の条件数
  - choose_drift(): pan から、捨てる行（rx か ry）を選ぶ（真下の接近。ヒステリシスつき）

Twist の並びは Servo と同じ [x, y, z, rx, ry, rz]（base_link 基準）。
"""
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
# 真下の接近で、rx を捨てるのは |pan| がこの値以下のとき（docs 2.3。それを超えたら ry）
DRIFT_PAN_LIMIT = 1.2
DRIFT_PAN_HYSTERESIS = 0.05
ROWS = {'x': 0, 'y': 1, 'z': 2, 'rx': 3, 'ry': 4, 'rz': 5}


def _rotation(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def _axis_rotation(axis, angle):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def chain_from_urdf(urdf_xml, base='base_link', tip='gripper_frame_link'):
    """URDF の文字列から、base から tip までの関節の鎖を作る（親から子の順）."""
    root = ET.fromstring(urdf_xml)
    by_child = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        xyz = origin.get('xyz', '0 0 0') if origin is not None else '0 0 0'
        rpy = origin.get('rpy', '0 0 0') if origin is not None else '0 0 0'
        axis = joint.find('axis')
        limit = joint.find('limit')
        by_child[joint.find('child').get('link')] = {
            'name': joint.get('name'),
            'type': joint.get('type'),
            'parent': joint.find('parent').get('link'),
            'xyz': np.array([float(v) for v in xyz.split()]),
            'rpy': [float(v) for v in rpy.split()],
            'axis': np.array([float(v) for v in axis.get('xyz').split()]) if axis is not None
            else np.zeros(3),
            'velocity': float(limit.get('velocity')) if limit is not None
            and limit.get('velocity') else None,
        }
    chain = []
    link = tip
    while link != base:
        chain.append(by_child[link])
        link = by_child[link]['parent']
    return chain[::-1]


def active_joint_names(chain):
    """鎖の中の、動く関節（revolute）の名前を、親から子の順で返す."""
    return [j['name'] for j in chain if j['type'] == 'revolute']


def jacobian(chain, q):
    """幾何 Jacobian（6x5）、先端の位置、先端の回転行列を返す。q は ARM_JOINTS の順の 5 要素."""
    names = list(ARM_JOINTS)
    rot = np.eye(3)
    pos = np.zeros(3)
    axes, origins = [], []
    for joint in chain:
        pos = pos + rot @ joint['xyz']
        rot = rot @ _rotation(joint['rpy'])
        if joint['type'] == 'revolute':
            axes.append(rot @ joint['axis'])
            origins.append(pos.copy())
            rot = rot @ _axis_rotation(joint['axis'], q[names.index(joint['name'])])
    jac = np.zeros((6, len(axes)))
    for i, (axis, origin) in enumerate(zip(axes, origins)):
        jac[:3, i] = np.cross(axis, pos - origin)
        jac[3:, i] = axis
    return jac, pos, rot


def left_null(jac):
    """6x5 の Jacobian の左零空間（作れない Twist の向き。単位ベクトル）."""
    u, _, _ = np.linalg.svd(jac, full_matrices=True)
    return u[:, 5]


def project_twist(jac, twist):
    """Twist を Jacobian の値域に射影する（作れない向きの成分を取り除く）."""
    u = left_null(jac)
    t = np.asarray(twist, dtype=float)
    return t - (u @ t) * u


def reduced_condition(jac, row):
    """行 row（'rx' など、または 0..5）を捨てた 5x5 の条件数（最大特異値 / 最小特異値）."""
    index = ROWS[row] if isinstance(row, str) else row
    s = np.linalg.svd(np.delete(jac, index, axis=0), compute_uv=False)
    return float(s[0] / s[-1]) if s[-1] > 0 else float('inf')


def choose_drift(pan, current=None):
    """pan から、捨てる行を選ぶ（真下の接近）。'rx'（|pan| <= 1.2）か 'ry'。current はヒステリシス用."""
    limit = DRIFT_PAN_LIMIT
    if current == 'rx':
        limit += DRIFT_PAN_HYSTERESIS
    elif current == 'ry':
        limit -= DRIFT_PAN_HYSTERESIS
    return 'rx' if abs(pan) <= limit else 'ry'


def drift_flags(row):
    """ChangeDriftDimensions の 6 つのフラグ [x, y, z, rx, ry, rz]（row だけ True）."""
    flags = [False] * 6
    flags[ROWS[row]] = True
    return flags
