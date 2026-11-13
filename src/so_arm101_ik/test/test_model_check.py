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

"""The pure helpers of tools/ik_model_check.py (fixture poses, CSV, pitch, table)."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_ik import analytic_ik, model_check
from so_arm101_ik.model_check import CaseRow
from so_arm101_ik.urdf_kinematics import UrdfChain


def _chain():
    share = Path(get_package_share_directory('so_arm101_description'))
    return UrdfChain((share / 'urdf' / 'so101_new_calib.urdf').read_text())


def test_fixture_poses_are_named_then_random_inside_the_limits():
    lower, upper = _chain().joint_limits()
    poses = model_check.fixture_poses(lower, upper)
    assert len(poses) == len(model_check.NAMED_POSES) + model_check.FIXTURE_RANDOM_COUNT == 105
    assert [n for n, _ in poses[:5]] == list(model_check.NAMED_POSES)
    assert poses[5][0] == 'random-001' and poses[-1][0] == 'random-100'
    for name, q in poses[5:]:
        assert np.all(q >= lower) and np.all(q <= upper), name
    # the start pose is the measured one of W1 (2026-11-13 07:54), not the old one of C3
    assert model_check.NAMED_POSES['start (W1)'] == (-0.0951, -1.4952, 1.6901, 0.4219, -0.0453)
    again = model_check.fixture_poses(lower, upper)
    assert all(np.array_equal(a[1], b[1]) for a, b in zip(poses, again))  # fixed seed


def test_quaternion_and_rotation_angle():
    rng = np.random.default_rng(1)
    for _ in range(200):
        q = rng.normal(size=4)
        rotation = model_check.quaternion_to_matrix(q)
        assert np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
        back = model_check.quaternion_to_matrix(model_check.matrix_to_quaternion(rotation))
        assert model_check.rotation_angle(rotation, back) < 1e-12
    half_turn = np.diag([1.0, -1.0, -1.0])  # trace < 0
    assert model_check.rotation_angle(
        half_turn, model_check.quaternion_to_matrix(
            model_check.matrix_to_quaternion(half_turn))) < 1e-12
    r = model_check.quaternion_to_matrix([0.0, 0.0, math.sin(5e-9), math.cos(5e-9)])
    assert model_check.rotation_angle(np.eye(3), r) == pytest.approx(1e-8, rel=1e-6)


def test_fixture_file_round_trip_keeps_full_precision(tmp_path):
    rows = [model_check.FixtureRow(
        f'p{i}', np.random.default_rng(i).normal(size=5), np.random.default_rng(i).normal(size=3),
        np.random.default_rng(i).normal(size=4)) for i in range(4)]
    path = tmp_path / 'fixture.csv'
    meta = {'urdf_sha256': 'abc', 'recorded_utc': '2026-11-13T00:00:00Z'}
    model_check.write_fixture(path, meta, rows)
    read_meta, read_rows = model_check.read_fixture(path)
    assert read_meta == meta
    for a, b in zip(rows, read_rows):
        assert a.name == b.name
        assert np.array_equal(a.q, b.q) and np.array_equal(a.position, b.position)
        assert np.array_equal(a.quaternion, b.quaternion)  # repr() is exact
    path.write_text('not a fixture\n')
    with pytest.raises(ValueError):
        model_check.read_fixture(path)
    with pytest.raises(ValueError):
        model_check.write_fixture(path, {'bad': 'a\nb'}, rows)


def test_staleness_follows_the_sha256_of_the_urdf():
    text = '<robot/>'
    fresh = {'urdf_sha256': model_check.urdf_sha256(text)}
    assert model_check.fixture_is_stale(fresh, text) is None
    assert model_check.urdf_sha256(text) == model_check.urdf_sha256(text.encode())
    message = model_check.fixture_is_stale({'urdf_sha256': 'old'}, text)
    assert 'stale' in message and '--record-fixture' in message and '--overwrite' in message


def test_pitch_of_the_approach_axis_equals_the_pitch_of_the_model():
    chain = _chain()
    lower, upper = chain.joint_limits()
    axis = model_check.approach_axis_in_tip(chain.fk(np.zeros(5))[1])
    worst = 0.0
    for _, q in model_check.fixture_poses(lower, upper):
        rotation = model_check.quaternion_to_matrix(
            model_check.matrix_to_quaternion(chain.fk(q)[1]))
        pitch = model_check.approach_pitch(rotation, q[0], axis)
        diff = pitch - analytic_ik.fk(q)[1]
        worst = max(worst, abs(math.atan2(math.sin(diff), math.cos(diff))))
    assert worst < model_check.PITCH_TOLERANCE_RAD, f'worst pitch difference {worst:.2e} rad'


def _rows():
    q = [0.0, -0.4, 0.5, 1.4, 0.0]
    base = np.array([0.1724, -0.0004, 0.0552])
    return [
        CaseRow('now', q, base + 1e-7, base, base + 1e-8, 0.0001, 1e-7, 1.0),
        CaseRow('target+x 10', q, base + [0.0, 0.0, 0.0006], base, None, None, None, 0.5),
        CaseRow('target+y 10', q, note='IKUnreachableError: out of reach'),
    ]


def test_summary_counts_rows_over_the_tolerance_and_failures():
    rows = _rows()
    assert [model_check.is_over(r) for r in rows] == [False, True, False]
    worst, where, mean, over, failed = model_check.summarize(rows)
    assert worst == pytest.approx(0.6) and where == 'target+x 10'
    assert mean == pytest.approx((0.6 + np.linalg.norm([1e-4] * 3)) / 2, rel=1e-6)
    assert (over, failed) == (1, 1)
    assert model_check.summarize([rows[2]]) == (0.0, None, 0.0, 0, 1)
    assert model_check.chain_difference_mm(rows[0]) == pytest.approx(
        np.linalg.norm([1e-5] * 3), rel=1e-6)
    assert model_check.chain_difference_mm(rows[1]) is None


def test_table_has_one_line_per_row_the_failure_reason_and_the_summary():
    lines = model_check.table_lines(_rows())
    assert len(lines) == 1 + 3 + 1
    assert lines[0].startswith('case') and 'MoveIt FK [mm]' in lines[0]
    assert lines[1].startswith('now') and not lines[1].endswith('*')
    assert lines[2].startswith('target+x 10') and lines[2].endswith('*')
    assert 'IKUnreachableError: out of reach' in lines[3]
    assert lines[4].startswith('summary: max |d| = 0.6000 mm (case target+x 10)')
    assert 'over the tolerance (marked *): 1, rows without a result: 1' in lines[4]


def test_table_csv_rows_have_the_same_width_as_the_header():
    out = model_check.table_csv_rows(_rows())
    assert len(out) == 4 and all(len(r) == len(out[0]) for r in out)
    assert out[1][0] == 'now' and float(out[1][6]) == pytest.approx(172.4001, abs=1e-4)
    assert out[3][12:16] == ['', '', '', ''] and out[3][-1].startswith('IKUnreachable')
