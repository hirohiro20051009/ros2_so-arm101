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

"""Tests of tag_mount.py and config/tag_mount.yaml (no ROS node is started)."""

import hashlib
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_camera import tag_mount as tm
from so_arm101_ik.urdf_kinematics import UrdfChain
import yaml

PKG = Path(__file__).resolve().parents[1]
APRILTAG_YAML = PKG / 'config' / 'apriltag.yaml'
SHIPPED = PKG / 'config' / 'tag_mount.yaml'

# The values of most tests are made here (GOOD), so that they do not depend on the shipped file.
# The shipped file holds the values measured on 2026-11-16 and has its own tests.
GOOD = {
    'tag_id': 0,
    'tag_size_m': 0.06,
    'reference_point_gripper_frame_m': [0.0, 0.0, 0.0063],
    'reference_point_source': 'a test source',
    'ref_to_tag_gripper_frame_m': [-0.038, 0.0, -0.062],
    'measured_note': 'a test note',
}


def _write(tmp_path, data, name='tag_mount.yaml'):
    path = tmp_path / name
    path.write_text(yaml.safe_dump(data))
    return path


def _chain():
    share = Path(get_package_share_directory('so_arm101_description'))
    return UrdfChain((share / 'urdf' / 'so101_new_calib.urdf').read_text())


def _with(**changes):
    data = dict(GOOD)
    data.update(changes)
    return data


def test_good_file_loads_and_the_offset_is_the_sum(tmp_path):
    mount = tm.load_tag_mount(_write(tmp_path, GOOD), APRILTAG_YAML)
    assert mount.tag_id == 0 and mount.tag_size_m == 0.06
    assert mount.reference_point == (0.0, 0.0, 0.0063)
    assert mount.ref_to_tag == (-0.038, 0.0, -0.062)
    assert mount.offset == pytest.approx((-0.038, 0.0, -0.0557))


def test_t_pose_text_front_up_left_from_the_signs_only(tmp_path):
    mount = tm.load_tag_mount(_write(tmp_path, GOOD), APRILTAG_YAML)
    assert tm.t_pose_offset_mm(mount.offset) == pytest.approx((38.0, 55.7, 0.0))
    # +x is back, +y is left, +z is down: a pure change of signs.
    assert tm.t_pose_offset_mm((0.01, 0.02, 0.03)) == pytest.approx((-10.0, -30.0, 20.0))
    lines = tm.t_pose_lines(mount)
    assert 'front 38.0 mm, up 55.7 mm, left 0.0 mm' in lines[0]
    assert 'front 38.0 mm, up 62.0 mm, left 0.0 mm' in lines[1]


def test_the_shipped_file_is_measured_and_loads():
    data = yaml.safe_load(SHIPPED.read_text())
    assert set(data) == set(tm.KEYS)
    assert data['ref_to_tag_gripper_frame_m'] == [-0.038, 0.0, -0.062]
    mount = tm.load_tag_mount(SHIPPED, APRILTAG_YAML)
    assert mount.ref_to_tag == (-0.038, 0.0, -0.062)
    assert mount.offset == pytest.approx((-0.038, 0.0, -0.0557))


def test_the_shipped_file_without_its_measurement_is_refused(tmp_path):
    data = yaml.safe_load(SHIPPED.read_text())
    data['ref_to_tag_gripper_frame_m'] = None
    with pytest.raises(tm.TagMountError, match='not been measured'):
        tm.load_tag_mount(_write(tmp_path, data), APRILTAG_YAML)


def test_null_measurement_is_refused(tmp_path):
    path = _write(tmp_path, _with(ref_to_tag_gripper_frame_m=None))
    with pytest.raises(tm.TagMountError, match='null'):
        tm.load_tag_mount(path, APRILTAG_YAML)


def test_a_missing_key_is_refused(tmp_path):
    data = dict(GOOD)
    del data['measured_note']
    with pytest.raises(tm.TagMountError, match='keys must be exactly'):
        tm.load_tag_mount(_write(tmp_path, data), APRILTAG_YAML)


def test_an_extra_key_is_refused(tmp_path):
    with pytest.raises(tm.TagMountError, match='keys must be exactly'):
        tm.load_tag_mount(_write(tmp_path, _with(offset_gripper_frame_m=[0, 0, 0])),
                          APRILTAG_YAML)


@pytest.mark.parametrize('value', [
    [1.0, 2.0], [0.0, 0.0, 0.0, 0.0], 'abc', 0.1, [True, 0.0, 0.0], [0.0, 'a', 0.0],
    [float('nan'), 0.0, 0.0], [float('inf'), 0.0, 0.0], [0.2, 0.2, 0.2], [38.0, 0.0, -62.0]])
@pytest.mark.parametrize('key', ['reference_point_gripper_frame_m', 'ref_to_tag_gripper_frame_m'])
def test_a_bad_vector_is_refused(tmp_path, key, value):
    with pytest.raises(tm.TagMountError):
        tm.load_tag_mount(_write(tmp_path, _with(**{key: value})), APRILTAG_YAML)


@pytest.mark.parametrize('value', [-1, True, 1.5, '0'])
def test_a_bad_tag_id_is_refused(tmp_path, value):
    with pytest.raises(tm.TagMountError, match='tag_id'):
        tm.load_tag_mount(_write(tmp_path, _with(tag_id=value)), APRILTAG_YAML)


@pytest.mark.parametrize('value', [0, -0.06, True, float('nan'), '0.06', None])
def test_a_bad_tag_size_is_refused(tmp_path, value):
    with pytest.raises(tm.TagMountError, match='tag_size_m'):
        tm.load_tag_mount(_write(tmp_path, _with(tag_size_m=value)), APRILTAG_YAML)


@pytest.mark.parametrize('key', ['reference_point_source', 'measured_note'])
def test_a_text_key_must_be_a_string(tmp_path, key):
    with pytest.raises(tm.TagMountError, match=key):
        tm.load_tag_mount(_write(tmp_path, _with(**{key: 5})), APRILTAG_YAML)


def test_the_tag_size_must_agree_with_apriltag_yaml(tmp_path):
    with pytest.raises(tm.TagMountError, match='differs'):
        tm.load_tag_mount(_write(tmp_path, _with(tag_size_m=0.0601)), APRILTAG_YAML)


def test_the_tag_id_must_be_in_apriltag_yaml(tmp_path):
    with pytest.raises(tm.TagMountError, match='not in the ids'):
        tm.load_tag_mount(_write(tmp_path, _with(tag_id=7)), APRILTAG_YAML)


def test_an_unreadable_yaml_is_refused(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('a: [1, 2')
    with pytest.raises(tm.TagMountError):
        tm.load_tag_mount(path, APRILTAG_YAML)
    with pytest.raises(tm.TagMountError):
        tm.load_tag_mount(tmp_path / 'missing.yaml', APRILTAG_YAML)


def test_the_sha256_is_that_of_the_file_bytes(tmp_path):
    path = _write(tmp_path, GOOD)
    mount = tm.load_tag_mount(path, APRILTAG_YAML)
    assert mount.sha256 == hashlib.sha256(path.read_bytes()).hexdigest()
    assert tm.sha256_file(path) == mount.sha256


def test_fk_tag_is_the_tip_plus_the_rotated_offset():
    chain = _chain()
    q = np.radians([10.0, -22.0, 30.0, 83.0, 15.0])
    position, rotation = chain.fk(q)
    offset = (-0.038, 0.001, -0.0557)
    assert tm.fk_tag(chain, q, offset) == pytest.approx(position + rotation @ np.array(offset))
    assert tm.fk_tcp(chain, q) == pytest.approx(position)
    assert tm.fk_tag(chain, q, (0.0, 0.0, 0.0)) == pytest.approx(position)


def test_the_offset_is_turned_back_to_the_base_axes_at_zero():
    chain = _chain()
    # The old offset in the axes of base_link at all joints 0 and the same offset in the axes of
    # gripper_frame_link (R0 transposed, as printed by ref_point_study.py: -0.03496 0.0017 -0.055).
    in_gripper = (-0.03496, 0.0017, -0.055)
    assert tm.offset_in_base_at_zero(chain, in_gripper) == pytest.approx(
        (-0.055, 0.0, 0.035), abs=2e-5)
    assert math.isclose(float(np.linalg.norm(tm.offset_in_base_at_zero(chain, in_gripper))),
                        float(np.linalg.norm(in_gripper)))


def _apriltag_yaml(tmp_path, ids, frames):
    data = {'apriltag': {'ros__parameters': {'tag': {'ids': ids, 'frames': frames}}}}
    return _write(tmp_path, data, 'apriltag.yaml')


@pytest.mark.parametrize('tag_id, frame', [(0, 'tag36h11_0'), (1, 'tag36h11_1')])
def test_the_frame_of_a_tag_is_read_from_apriltag_yaml(tag_id, frame):
    assert tm.tag_frame_name(APRILTAG_YAML, tag_id) == frame


def test_a_tag_that_is_not_in_apriltag_yaml_has_no_frame():
    with pytest.raises(tm.TagMountError, match='not in the ids'):
        tm.tag_frame_name(APRILTAG_YAML, 5)


def test_ids_and_frames_of_different_lengths_are_refused(tmp_path):
    path = _apriltag_yaml(tmp_path, [0, 1], ['only_one'])
    with pytest.raises(tm.TagMountError, match='different lengths'):
        tm.tag_frame_name(path, 0)


def test_a_frame_that_is_not_a_name_is_refused(tmp_path):
    path = _apriltag_yaml(tmp_path, [0, 1], [5, 'tag_b'])
    with pytest.raises(tm.TagMountError, match='not a name'):
        tm.tag_frame_name(path, 0)
    assert tm.tag_frame_name(path, 1) == 'tag_b'
