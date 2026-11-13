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

"""The chain of the URDF and the closed-form model against the recorded MoveIt FK (the fixture)."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_ik import analytic_ik, model_check
from so_arm101_ik.urdf_kinematics import UrdfChain

FIXTURE = Path(__file__).resolve().parent / 'data' / 'moveit_fk.csv'
RECORD_COMMAND = ('ROS_DOMAIN_ID=87 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 '
                  'tools/ik_model_check.py --record-fixture '
                  'src/so_arm101_ik/test/data/moveit_fk.csv --overwrite  (with the mock launch)')


def _urdf_xml():
    share = Path(get_package_share_directory('so_arm101_description'))
    return (share / 'urdf' / 'so101_new_calib.urdf').read_text()


def _fixture():
    if not FIXTURE.exists():
        pytest.fail(f'{FIXTURE} is missing. Record it: {RECORD_COMMAND}')
    return model_check.read_fixture(FIXTURE)


def _worst_mm(rows, position_of):
    return max((np.linalg.norm(position_of(r) - r.position) * 1000.0, r.name) for r in rows)


def _chain_position(chain):
    return lambda row: chain.fk(row.q)[0]


def _model_position(row):
    return analytic_ik.fk(row.q)[0]


def _named_rows():
    # the names are known without the file, so that the tests are listed even if it is missing
    return sorted(model_check.NAMED_POSES)


def _row(name):
    return next(r for r in _fixture()[1] if r.name == name)


def test_fixture_is_not_stale():
    meta, _ = _fixture()
    message = model_check.fixture_is_stale(meta, _urdf_xml())
    assert message is None, message


def test_fixture_structure():
    meta, rows = _fixture()
    chain = UrdfChain(_urdf_xml())
    lower, upper = chain.joint_limits()
    expected = model_check.fixture_poses(lower, upper)
    assert [r.name for r in rows] == [n for n, _ in expected]
    assert meta['frame_id'] == model_check.FIXTURE_FRAME
    assert meta['link'] == model_check.FIXTURE_LINK
    assert meta['seed'] == str(model_check.FIXTURE_SEED)
    for row, (name, q) in zip(rows, expected):
        if name in model_check.NAMED_POSES:
            assert np.array_equal(row.q, q), name
        else:
            assert np.all(row.q >= lower) and np.all(row.q <= upper), name
        assert np.all(np.isfinite(row.position)) and abs(np.linalg.norm(row.quaternion) - 1) < 1e-9
    for key in ('moveit_version', 'recorded_utc', 'git_commit', 'tool_sha256'):
        assert meta.get(key), key


def test_chain_position_equals_moveit_for_all_poses():
    worst, name = _worst_mm(_fixture()[1], _chain_position(UrdfChain(_urdf_xml())))
    assert worst <= model_check.CHAIN_TOLERANCE_MM, f'{worst:.6f} mm at {name}'


@pytest.mark.parametrize('name', _named_rows())
def test_chain_position_equals_moveit_for_named_poses(name):
    row = _row(name)
    diff = np.linalg.norm(UrdfChain(_urdf_xml()).fk(row.q)[0] - row.position) * 1000.0
    assert diff <= model_check.CHAIN_TOLERANCE_MM, f'{diff:.6f} mm'


def test_chain_rotation_equals_moveit():
    chain = UrdfChain(_urdf_xml())
    worst = max(model_check.rotation_angle(chain.fk(r.q)[1],
                                           model_check.quaternion_to_matrix(r.quaternion))
                for r in _fixture()[1])
    assert worst <= model_check.ROTATION_TOLERANCE_RAD, f'{worst:.2e} rad'


def test_model_position_equals_moveit_for_all_poses():
    worst, name = _worst_mm(_fixture()[1], _model_position)
    assert worst <= model_check.MODEL_TOLERANCE_MM, f'{worst:.6f} mm at {name}'


@pytest.mark.parametrize('name', _named_rows())
def test_model_position_equals_moveit_for_named_poses(name):
    row = _row(name)
    diff = np.linalg.norm(analytic_ik.fk(row.q)[0] - row.position) * 1000.0
    assert diff <= model_check.MODEL_TOLERANCE_MM, f'{diff:.6f} mm'


def test_model_pitch_equals_the_pitch_of_the_moveit_pose():
    rows = _fixture()[1]
    zero = next(r for r in rows if r.name == 'zero')
    axis = model_check.approach_axis_in_tip(model_check.quaternion_to_matrix(zero.quaternion))
    worst = 0.0
    for row in rows:
        pitch = model_check.approach_pitch(
            model_check.quaternion_to_matrix(row.quaternion), row.q[0], axis)
        diff = pitch - analytic_ik.fk(row.q)[1]
        worst = max(worst, abs(math.atan2(math.sin(diff), math.cos(diff))))
    assert worst <= model_check.PITCH_TOLERANCE_RAD, f'{worst:.2e} rad'
