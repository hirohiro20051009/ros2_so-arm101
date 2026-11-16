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
Tag mount: where the AprilTag sits on the gripper, and the forward kinematics of its centre.

Pure helpers (no ROS). All offsets are in the axes of gripper_frame_link of the URDF. The file
config/tag_mount.yaml holds a reference point (computed from the URDF and the mesh) and the vector
from that point to the tag centre (measured by hand); the offset used for the forward kinematics
is their sum, computed here, so that no sum is ever written by hand.
"""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

import numpy as np

from .camera_config import load_yaml_mapping

KEYS = ('tag_id', 'tag_size_m', 'reference_point_gripper_frame_m', 'reference_point_source',
        'ref_to_tag_gripper_frame_m', 'measured_note')

# Largest allowed length of each vector [m]. A guess: it only stops a value typed in the wrong unit
# (mm instead of m); the real offset is about 0.07 m.
VECTOR_NORM_MAX = 0.2

# Two tag sizes (the file and apriltag.yaml) must agree to this many metres.
TAG_SIZE_TOLERANCE = 1e-6

# Axes of gripper_frame_link seen with the tip pointing straight down (the T pose of the manuals):
# +x is back, +y is left, +z is down (computed from the URDF at all joints 0, see inventory.md).
T_POSE_FRONT = -1.0  # front = -x
T_POSE_UP = -1.0  # up = -z
T_POSE_LEFT = 1.0  # left = +y


class TagMountError(ValueError):
    """tag_mount.yaml (or the tag size in apriltag.yaml) is not usable."""


@dataclass(frozen=True)
class TagMount:
    """The checked content of tag_mount.yaml."""

    tag_id: int
    tag_size_m: float
    reference_point: tuple
    reference_source: str
    ref_to_tag: tuple
    note: str
    sha256: str

    @property
    def offset(self):
        """Return (x, y, z) [m] of the tag centre in gripper_frame_link: reference + measured."""
        return tuple(a + b for a, b in zip(self.reference_point, self.ref_to_tag))


def sha256_file(path):
    """Return the sha256 (hex) of the bytes of a file."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _vector(name, value):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value)):
        raise TagMountError(f'{name} must be a list of 3 numbers')
    values = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in values):
        raise TagMountError(f'{name} must be finite')
    if math.sqrt(sum(v * v for v in values)) > VECTOR_NORM_MAX:
        raise TagMountError(f'{name} is longer than {VECTOR_NORM_MAX} m (is the unit metres?)')
    return values


def _text(name, value):
    if not isinstance(value, str):
        raise TagMountError(f'{name} must be a string')
    return value


def check_tag_size(tag_size_m, tag_id, apriltag_yaml):
    """Raise TagMountError unless apriltag.yaml has tag_id with the same size."""
    try:
        params = load_yaml_mapping(apriltag_yaml)['apriltag']['ros__parameters']['tag']
        ids = list(params['ids'])
        sizes = list(params['sizes'])
    except (ValueError, KeyError, TypeError) as exc:
        raise TagMountError(
            f'cannot read the tag ids and sizes of {apriltag_yaml}: {exc}') from exc
    if tag_id not in ids:
        raise TagMountError(f'tag_id {tag_id} is not in the ids {ids} of {apriltag_yaml}')
    in_file = sizes[ids.index(tag_id)]
    if isinstance(in_file, bool) or not isinstance(in_file, (int, float)):
        raise TagMountError(f'the size of tag {tag_id} in {apriltag_yaml} is not a number')
    if abs(in_file - tag_size_m) > TAG_SIZE_TOLERANCE:
        raise TagMountError(
            f'tag_size_m {tag_size_m} differs from the size {in_file} of tag {tag_id} in '
            f'{apriltag_yaml}')


def load_tag_mount(path, apriltag_yaml):
    """
    Read and check tag_mount.yaml; apriltag_yaml is the file whose tag size must agree.

    The keys must be exactly KEYS. ref_to_tag_gripper_frame_m null means "not measured yet" and
    is refused. Raises TagMountError if anything is not valid.
    """
    try:
        data = load_yaml_mapping(path)
    except ValueError as exc:
        raise TagMountError(str(exc)) from exc
    if set(data) != set(KEYS):
        raise TagMountError(
            f'{path}: the keys must be exactly {sorted(KEYS)}, got {sorted(map(str, data))}')
    tag_id = data['tag_id']
    if isinstance(tag_id, bool) or not isinstance(tag_id, int) or tag_id < 0:
        raise TagMountError('tag_id must be an integer of 0 or more')
    size = data['tag_size_m']
    if (isinstance(size, bool) or not isinstance(size, (int, float))
            or not math.isfinite(size) or size <= 0.0):
        raise TagMountError('tag_size_m must be a positive number')
    reference = _vector('reference_point_gripper_frame_m', data['reference_point_gripper_frame_m'])
    if data['ref_to_tag_gripper_frame_m'] is None:
        raise TagMountError(
            f'{path}: ref_to_tag_gripper_frame_m is null: the tag has not been measured yet')
    measured = _vector('ref_to_tag_gripper_frame_m', data['ref_to_tag_gripper_frame_m'])
    source = _text('reference_point_source', data['reference_point_source'])
    note = _text('measured_note', data['measured_note'])
    check_tag_size(float(size), tag_id, apriltag_yaml)
    return TagMount(tag_id, float(size), reference, source, measured, note, sha256_file(path))


def t_pose_offset_mm(offset):
    """
    Return (front, up, left) in mm of an offset in gripper_frame_link, for the tip pointing down.

    Only the signs of the axes change (+x back, +y left, +z down), so a wrong sign in the yaml
    shows up when this is compared with what the tag looks like on the gripper.
    """
    x, y, z = offset
    return (1000.0 * T_POSE_FRONT * x, 1000.0 * T_POSE_UP * z, 1000.0 * T_POSE_LEFT * y)


def t_pose_lines(mount):
    """Return the two lines printed after tag_mount.yaml is read."""
    front, up, left = t_pose_offset_mm(mount.offset)
    m_front, m_up, m_left = t_pose_offset_mm(mount.ref_to_tag)
    return [
        'tag centre seen from the origin of gripper_frame_link (tip pointing down): '
        f'front {front:.1f} mm, up {up:.1f} mm, left {left:.1f} mm',
        '  of which reference point to tag centre (measured): '
        f'front {m_front:.1f} mm, up {m_up:.1f} mm, left {m_left:.1f} mm',
    ]


def fk_tcp(chain, q):
    """Return the position (base_link) of gripper_frame_link for the five arm joints."""
    return np.asarray(chain.fk(q)[0], dtype=float)


def fk_tag(chain, q, offset):
    """Return the position (base_link) of the tag centre: FK of the tip plus R times the offset."""
    position, rotation = chain.fk(q)
    return np.asarray(position, dtype=float) + np.asarray(rotation, dtype=float) @ np.asarray(
        offset, dtype=float)


def offset_in_base_at_zero(chain, offset):
    """Return the offset in the axes of base_link when all joints are 0 (for the meta file)."""
    rotation = np.asarray(chain.fk(np.zeros(5))[1], dtype=float)
    return rotation @ np.asarray(offset, dtype=float)
