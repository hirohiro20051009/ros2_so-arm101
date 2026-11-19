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

"""Tests of record.py (the recording logic; a fake io replaces every ROS part)."""

import csv
import fcntl
import hashlib
import inspect
import math
from pathlib import Path
import threading

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_camera import record as rec
from so_arm101_camera import tag_mount as tm
from so_arm101_ik import step_runner as sr
from so_arm101_ik.urdf_kinematics import UrdfChain
import yaml

MOUNT = tm.TagMount(0, 0.06, (0.0, 0.0, 0.0063), 'a test source', (-0.038, 0.0, -0.062),
                    'a test note', 'e' * 64)
ZERO = {name: 0.0 for name in sr.JOINT_NAMES}
IDENTITY = rec.make_identity('a' * 64, 'b' * 64, 'c' * 64, 'd' * 64, 0, 'f' * 64, ZERO)
Q_T = np.radians([0.0, -22.0, 30.0, 83.0, 0.0])
IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)


def _chain():
    share = Path(get_package_share_directory('so_arm101_description'))
    return UrdfChain((share / 'urdf' / 'so101_new_calib.urdf').read_text())


def _frames(count, start=0, margin=40.0, position=(0.10, 0.05, 0.60), noise=0.0,
            quaternion=IDENTITY_QUATERNION):
    """Return detections and tag poses of count frames with stamps start, start + 1, ..."""
    detections, transforms = {}, {}
    for i in range(count):
        stamp = 1_000_000 + start + i
        detections[stamp] = margin
        transforms[stamp] = ((position[0] + noise * math.sin(i), position[1], position[2]),
                             quaternion)
    return detections, transforms


class FakeIo:
    """A fake io: a clock that only moves when the code waits, and scripted frames."""

    def __init__(self, source=None, q=Q_T, moving=False, stale=False, answers=(True,)):
        self.t = 100.0
        self.waits = []
        self.said = []
        self.questions = []
        self.answers = list(answers)
        self.q = np.asarray(q, dtype=float)
        self.moving = moving
        self.stale = stale
        self.polls = 0
        self.flushes = 0
        self.source = source if source is not None else self._steady

    def _steady(self, index):
        return _frames(1, start=index)

    def now(self):
        return self.t

    def wait(self, seconds):
        self.waits.append(seconds)
        self.t += seconds

    def read_joints(self):
        q = self.q + (0.01 * len(self.waits) if self.moving else 0.0)
        return q, (self.t - 5.0 if self.stale else self.t)

    def flush(self):
        self.flushes += 1

    def poll(self, seconds):
        self.t += seconds
        self.polls += 1
        return self.source(self.polls)

    def say(self, text):
        self.said.append(text)

    def confirm(self, text):
        self.questions.append(text)
        return self.answers.pop(0)

    def wall_time(self):
        return 1_700_000_000.0

    def date_text(self):
        return '2026-11-16'


def _loop(io, tmp_path, name='points.csv'):
    return rec.RecordLoop(io, _chain(), MOUNT, tmp_path / name, IDENTITY, {'note': 'test'})


def _rows(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


# ------------------------------------------------------------------------------- matching

def test_equal_stamps_are_matched_in_the_order_of_the_stamps():
    detections, transforms = _frames(5)
    result = rec.match_frames(dict(reversed(list(detections.items()))), transforms)
    assert [m[0] for m in result.matched] == sorted(detections)
    assert (result.det_n, result.tf_n, len(result.matched)) == (5, 5, 5)
    assert (result.unmatched_det, result.unmatched_tf, result.invalid_n) == (0, 0, 0)


def test_a_frame_with_only_one_side_is_counted_and_dropped():
    detections, transforms = _frames(5)
    detections[7] = 30.0
    transforms[8] = ((0.0, 0.0, 0.5), IDENTITY_QUATERNION)
    transforms[9] = ((0.0, 0.0, 0.5), IDENTITY_QUATERNION)
    result = rec.match_frames(detections, transforms)
    assert (result.det_n, result.tf_n, len(result.matched)) == (6, 7, 5)
    assert (result.unmatched_det, result.unmatched_tf) == (1, 2)


def test_a_stamp_one_nanosecond_apart_is_not_matched():
    detections = {1_000_000: 40.0}
    transforms = {1_000_001: ((0.0, 0.0, 0.5), IDENTITY_QUATERNION)}
    result = rec.match_frames(detections, transforms)
    assert len(result.matched) == 0 and result.unmatched_det == 1 and result.unmatched_tf == 1


@pytest.mark.parametrize('margin, translation, quaternion', [
    (float('nan'), (0.0, 0.0, 0.5), IDENTITY_QUATERNION),
    (40.0, (0.0, float('nan'), 0.5), IDENTITY_QUATERNION),
    (40.0, (0.0, 0.0, 0.5), (0.0, 0.0, 0.0, 0.0)),
    (40.0, (0.0, 0.0, 0.5), (0.0, 0.0, 0.0, float('inf')))])
def test_a_frame_with_an_invalid_number_is_counted_not_used(margin, translation, quaternion):
    result = rec.match_frames({5: margin}, {5: (translation, quaternion)})
    assert len(result.matched) == 0 and result.invalid_n == 1
    assert result.unmatched_det == 0 and result.unmatched_tf == 0


def test_the_tag_normal_is_the_z_axis_of_the_rotation():
    assert rec.tag_normal(IDENTITY_QUATERNION) == pytest.approx((0.0, 0.0, 1.0))
    half = math.sqrt(0.5)
    assert rec.tag_normal((half, 0.0, 0.0, half)) == pytest.approx((0.0, -1.0, 0.0))
    assert rec.tag_normal((0.0, 0.0, 0.0, 2.0)) == pytest.approx((0.0, 0.0, 1.0))


@pytest.mark.parametrize('count, seen', [(0, False), (rec.MATCH_MIN - 1, False),
                                         (rec.MATCH_MIN, True), (rec.MATCH_MIN + 5, True)])
def test_the_tag_is_seen_from_eight_matched_frames_only(count, seen):
    assert rec.is_seen(rec.match_frames(*_frames(count))) is seen


def test_a_large_incidence_does_not_change_the_decision():
    # The tag normal is turned 80 degrees away from the line of sight.
    half = math.radians(80.0) / 2.0
    quaternion = (math.sin(half), 0.0, 0.0, math.cos(half))
    match = rec.match_frames(*_frames(10, quaternion=quaternion))
    window = rec.average_matches(match)
    assert rec.incidence_deg(window.position, window.normal) > 60.0
    assert rec.is_seen(match)


# ------------------------------------------------------------------------------- outliers

def test_a_tight_cluster_keeps_every_frame():
    points = [(0.1 + 0.0001 * i, 0.05, 0.6) for i in range(10)]
    assert rec.keep_mask(points).all()


def test_the_floor_keeps_a_frame_4_mm_off_even_when_the_mad_is_zero():
    points = [(0.1, 0.05, 0.6)] * 9 + [(0.1, 0.05, 0.604)]
    assert rec.keep_mask(points).all()


def test_a_frame_6_mm_off_is_dropped_when_the_mad_is_zero():
    points = [(0.1, 0.05, 0.6)] * 9 + [(0.1, 0.05, 0.606)]
    keep = rec.keep_mask(points)
    assert list(keep) == [True] * 9 + [False]


def test_three_times_the_mad_decides_when_it_is_above_the_floor():
    # The median of z is 0 and the MAD is 4 mm, so the limit is 12 mm (above the 5 mm floor):
    # the frame 10 mm off is kept, the frame 16 mm off is dropped.
    z_mm = [0, 0, 4, -4, 4, -4, 8, -8, 0, 10, 16]
    points = [(0.1, 0.05, 0.6 + v / 1000.0) for v in z_mm]
    keep = rec.keep_mask(points)
    assert keep[9] and not keep[10] and keep[:9].all()


def test_the_limit_is_checked_on_each_axis():
    points = [(0.1, 0.05, 0.6)] * 9 + [(0.106, 0.05, 0.6)]
    assert list(rec.keep_mask(points)) == [True] * 9 + [False]
    assert rec.keep_mask([]).shape == (0,)


# ------------------------------------------------------------------------------- averaging

def test_the_window_has_the_mean_std_ptp_and_the_counts():
    frames = rec.match_frames(*_frames(10, noise=0.0005))
    window = rec.average_matches(frames)
    positions = np.array([m[2] for m in frames.matched])
    assert window.n == 10 and window.dropped == 0
    assert window.position == pytest.approx(positions.mean(axis=0))
    assert window.std == pytest.approx(positions.std(axis=0, ddof=1))
    assert window.ptp == pytest.approx(np.ptp(positions, axis=0))
    assert window.normal == pytest.approx((0.0, 0.0, 1.0))


def test_the_outlier_is_removed_and_counted():
    detections, transforms = _frames(10)
    transforms[1_000_003] = ((0.10, 0.05, 0.62), IDENTITY_QUATERNION)
    window = rec.average_matches(rec.match_frames(detections, transforms))
    assert window.n == 9 and window.dropped == 1
    assert window.position == pytest.approx((0.10, 0.05, 0.60))


def test_the_margin_min_and_mean_are_those_of_the_kept_frames():
    detections, transforms = _frames(10)
    for i, stamp in enumerate(sorted(detections)):
        detections[stamp] = 10.0 + i
    window = rec.average_matches(rec.match_frames(detections, transforms))
    assert window.margin_min == 10.0 and window.margin_mean == pytest.approx(14.5)


def test_nothing_matched_gives_no_window():
    assert rec.average_matches(rec.match_frames({}, {})) is None


def test_stable_is_recorded_but_does_not_remove_frames():
    detections, transforms = {}, {}
    for i in range(10):
        stamp = 1_000_000 + i
        detections[stamp] = 40.0
        transforms[stamp] = ((0.10, 0.05, 0.60 + (0.002 if i >= 5 else 0.0)), IDENTITY_QUATERNION)
    window = rec.average_matches(rec.match_frames(detections, transforms))
    assert window.stable is False and window.n == 10
    steady = rec.average_matches(rec.match_frames(*_frames(10)))
    assert steady.stable is True


def test_the_incidence_is_the_angle_to_the_line_of_sight_whatever_the_sign():
    position = np.array([0.0, 0.0, 0.6])
    assert rec.incidence_deg(position, np.array([0.0, 0.0, 1.0])) == pytest.approx(0.0)
    assert rec.incidence_deg(position, np.array([0.0, 0.0, -1.0])) == pytest.approx(0.0)
    tilted = np.array([math.sin(math.radians(60.0)), 0.0, math.cos(math.radians(60.0))])
    assert rec.incidence_deg(position, tilted) == pytest.approx(60.0)
    assert math.isnan(rec.incidence_deg(np.zeros(3), tilted))


def test_the_lines_show_the_rate_the_margin_and_the_angle():
    match = rec.match_frames(*_frames(10, margin=33.0))
    detections, transforms = _frames(10, margin=33.0)
    detections.update(_frames(10, start=50)[0])
    lines = '\n'.join(rec.investigation_lines(rec.match_frames(detections, transforms),
                                              rec.average_matches(match), True))
    assert 'detection rate (matched / detections): 0.50' in lines
    assert 'decision_margin min 33.0 mean 33.0' in lines
    assert 'shown only' in lines and 'tag seen: yes' in lines


def test_the_rate_is_zero_without_detections():
    lines = '\n'.join(rec.investigation_lines(rec.match_frames({}, {}), None, False))
    assert 'rate (matched / detections): 0.00' in lines and 'tag seen: NO' in lines
    assert 'arm stopped: NO' in lines


# ------------------------------------------------------------------------------- settling

def test_the_settling_uses_the_values_of_so_arm101_ik(tmp_path):
    io = FakeIo()
    assert _loop(io, tmp_path).settle() is True
    assert io.waits[:2] == [sr.SETTLE_MIN, sr.STOP_INTERVAL]
    assert rec.thresholds()['settle_min_s'] is sr.SETTLE_MIN
    assert rec.thresholds()['stop_tol_rad'] is sr.STOP_TOL
    assert rec.thresholds()['stop_interval_s'] is sr.STOP_INTERVAL
    assert rec.thresholds()['settle_timeout_s'] is sr.SETTLE_TIMEOUT


def test_a_moving_arm_is_not_settled_before_the_timeout(tmp_path):
    io = FakeIo(moving=True)
    assert _loop(io, tmp_path).settle() is False
    assert io.t - 100.0 > sr.SETTLE_TIMEOUT


def test_a_stale_joint_state_is_not_settled(tmp_path):
    io = FakeIo(stale=True)
    assert _loop(io, tmp_path).settle() is False


# ------------------------------------------------------------------------------- the session

def _first_point(tmp_path, label='P01', name='points.csv'):
    path = tmp_path / name
    row = rec.build_row(label, 1.0, Q_T, 0.0, (0.1, 0.0, 0.1), (0.2, 0.0, 0.2),
                        rec.match_frames(*_frames(10)),
                        rec.average_matches(rec.match_frames(*_frames(10))), True)
    meta = rec.build_meta(IDENTITY, MOUNT, (-0.055, 0.0, 0.035), '2026-11-16')
    return path, row, meta


def test_a_new_session_needs_neither_file(tmp_path):
    assert rec.check_session(tmp_path / 'points.csv', 'P01', IDENTITY) == 'new'


def test_the_first_point_creates_the_meta_file_and_the_csv(tmp_path):
    path, row, meta = _first_point(tmp_path)
    assert rec.write_point(path, 'P01', IDENTITY, meta, row) == 1
    assert rec.meta_path_for(path).exists()
    rows = _rows(path)
    assert [r['label'] for r in rows] == ['P01'] and rows[0]['idx'] == '1'
    assert list(rows[0]) == list(rec.COLUMNS)


def test_the_second_point_is_appended_and_the_meta_file_stays(tmp_path):
    path, row, meta = _first_point(tmp_path)
    rec.write_point(path, 'P01', IDENTITY, meta, row)
    before_csv = path.read_bytes()
    before_meta = rec.meta_path_for(path).read_bytes()
    assert rec.write_point(path, 'P02', IDENTITY, {'other': 1}, dict(row, label='P02')) == 2
    assert path.read_bytes().startswith(before_csv)
    assert rec.meta_path_for(path).read_bytes() == before_meta
    assert [r['label'] for r in _rows(path)] == ['P01', 'P02']


def test_the_same_label_is_refused_and_nothing_changes(tmp_path):
    path, row, meta = _first_point(tmp_path)
    rec.write_point(path, 'P01', IDENTITY, meta, row)
    before = path.read_bytes()
    with pytest.raises(rec.SessionError, match='already'):
        rec.write_point(path, 'P01', IDENTITY, meta, row)
    assert path.read_bytes() == before


@pytest.mark.parametrize('key, value', [
    ('tag_mount_sha256', 'x' * 64), ('urdf_sha256', 'x' * 64), ('apriltag_yaml_sha256', 'x' * 64),
    ('realsense_yaml_sha256', 'x' * 64), ('tag_id', 1),
    ('follower_calibration_sha256', 'x' * 64),
    ('zero_offset_deg', dict(ZERO, elbow_flex=-6.3))])
def test_a_changed_identity_value_is_refused(tmp_path, key, value):
    path, row, meta = _first_point(tmp_path)
    rec.write_point(path, 'P01', IDENTITY, meta, row)
    before = path.read_bytes()
    changed = dict(IDENTITY, **{key: value})
    with pytest.raises(rec.SessionError, match=key):
        rec.check_session(path, 'P02', changed)
    with pytest.raises(rec.SessionError, match=key):
        rec.write_point(path, 'P02', changed, meta, dict(row, label='P02'))
    assert path.read_bytes() == before


def test_a_meta_file_without_an_identity_value_is_refused(tmp_path):
    path, row, meta = _first_point(tmp_path)
    rec.write_point(path, 'P01', IDENTITY, meta, row)
    broken = yaml.safe_load(rec.meta_path_for(path).read_text())
    del broken['identity']['urdf_sha256']
    rec.meta_path_for(path).write_text(yaml.safe_dump(broken))
    with pytest.raises(rec.SessionError, match='urdf_sha256: missing'):
        rec.check_session(path, 'P02', IDENTITY)


def test_a_leftover_meta_file_without_a_csv_is_refused(tmp_path):
    path, row, meta = _first_point(tmp_path)
    rec.meta_path_for(path).write_text('schema: 1\n')
    with pytest.raises(rec.SessionError, match='leftover'):
        rec.write_point(path, 'P01', IDENTITY, meta, row)
    assert not path.exists()


def test_a_csv_without_a_meta_file_is_refused(tmp_path):
    path, row, meta = _first_point(tmp_path)
    path.write_text('idx\n')
    with pytest.raises(rec.SessionError, match='does not'):
        rec.write_point(path, 'P01', IDENTITY, meta, row)
    assert path.read_text() == 'idx\n' and not rec.meta_path_for(path).exists()


def test_a_csv_with_another_header_is_refused(tmp_path):
    path, row, meta = _first_point(tmp_path)
    rec.write_point(path, 'P01', IDENTITY, meta, row)
    path.write_text('idx,time\n1,2\n')
    with pytest.raises(rec.SessionError, match='header'):
        rec.check_session(path, 'P02', IDENTITY)


@pytest.mark.parametrize('label', ['', 'P 01', 'P,01', 'P01\n', None, 5, '../x'])
def test_a_bad_label_is_refused(tmp_path, label):
    with pytest.raises(rec.SessionError, match='label'):
        rec.check_session(tmp_path / 'points.csv', label, IDENTITY)


def test_the_files_are_created_exclusively_even_if_the_check_was_passed(tmp_path, monkeypatch):
    path, row, meta = _first_point(tmp_path)
    rec.meta_path_for(path).write_text('somebody else\n')
    monkeypatch.setattr(rec, 'check_session', lambda *args: 'new')
    with pytest.raises(rec.SessionError, match='someone else'):
        rec.write_point(path, 'P01', IDENTITY, meta, row)
    assert rec.meta_path_for(path).read_text() == 'somebody else\n' and not path.exists()


def test_the_meta_file_explains_where_the_values_come_from(tmp_path):
    path, row, meta = _first_point(tmp_path)
    rec.write_point(path, 'P01', IDENTITY, meta, row)
    content = yaml.safe_load(rec.meta_path_for(path).read_text())
    assert content['identity'] == IDENTITY
    mount = content['tag_mount']
    assert mount['offset_gripper_frame_m'] == pytest.approx([-0.038, 0.0, -0.0557])
    assert mount['reference_point_gripper_frame_m'] == [0.0, 0.0, 0.0063]
    assert mount['ref_to_tag_gripper_frame_m'] == [-0.038, 0.0, -0.062]
    assert mount['offset_t_pose_mm'] == pytest.approx({'front': 38.0, 'up': 55.7, 'left': 0.0})
    assert mount['offset_base_link_at_zero_pose_m'] == pytest.approx([-0.055, 0.0, 0.035])
    assert content['thresholds']['settle_min_s'] == sr.SETTLE_MIN
    assert content['thresholds']['match_min'] == rec.MATCH_MIN


def test_the_columns_have_the_prefixes_and_no_goal():
    assert len(set(rec.COLUMNS)) == len(rec.COLUMNS)
    assert not [c for c in rec.COLUMNS if 'goal' in c]
    assert {'q_' + name for name in sr.JOINT_NAMES} <= set(rec.COLUMNS)
    assert {'fk_tcp_x', 'fk_tag_z', 'cam_tag_x', 'cam_std_z', 'cam_normal_y', 'cam_margin_min',
            'det_n', 'tf_n', 'matched_n', 'unmatched_det', 'unmatched_tf', 'cam_dropped',
            'cam_n', 'stable', 'still', 'tag_seen', 'incidence_deg'} <= set(rec.COLUMNS)


def test_an_unseen_row_has_no_cam_values_but_has_the_counts():
    match = rec.match_frames(*_frames(3))
    row = rec.build_row('P01', 1.0, Q_T, 0.0, (0.1, 0.0, 0.1), (0.2, 0.0, 0.2), match,
                        rec.average_matches(match), True)
    assert row['tag_seen'] == '0' and row['matched_n'] == '3' and row['fk_tag_x'] == '0.200000'
    assert all(row[c] == '' for c in rec.COLUMNS
               if c.startswith('cam_') or c in ('incidence_deg', 'stable', 'idx'))


# ------------------------------------------------------------------------------- the loop

def test_investigating_writes_nothing_and_asks_nothing(tmp_path):
    io = FakeIo()
    assert _loop(io, tmp_path).run('P01', investigate_only=True) == rec.EXIT_OK
    assert list(tmp_path.iterdir()) == [] and io.questions == []
    assert any('tag seen: yes' in line for line in io.said)


def test_the_t_pose_offset_is_shown_first(tmp_path):
    io = FakeIo()
    _loop(io, tmp_path).run('P01', investigate_only=True)
    assert 'front 38.0 mm, up 55.7 mm, left 0.0 mm' in io.said[0]


def test_an_unseen_tag_stops_without_asking_or_writing(tmp_path):
    io = FakeIo(source=lambda index: _frames(0))
    assert _loop(io, tmp_path).run('P01') == rec.EXIT_NOT_SEEN
    assert list(tmp_path.iterdir()) == [] and io.questions == []
    assert any('Stop and ask' in line for line in io.said)
    assert io.t - 100.0 > rec.COLLECT_TIMEOUT


def test_a_refused_session_does_not_even_measure(tmp_path):
    path, row, meta = _first_point(tmp_path)
    rec.write_point(path, 'P01', IDENTITY, meta, row)
    io = FakeIo()
    assert _loop(io, tmp_path).run('P01') == rec.EXIT_REFUSED
    assert io.waits == [] and io.polls == 0 and io.questions == []


def test_a_no_answer_writes_nothing(tmp_path):
    io = FakeIo(answers=(False,))
    # 2 for a refusal and for no answer, as ik_move_run (sr.EXIT_STOPPED).
    assert _loop(io, tmp_path).run('P01') == rec.EXIT_NOT_CONFIRMED == sr.EXIT_STOPPED == 2
    assert rec.EXIT_REFUSED == 2
    assert list(tmp_path.iterdir()) == [] and len(io.questions) == 1


def test_a_point_is_recorded_with_the_fk_and_the_camera_values(tmp_path):
    io = FakeIo()
    assert _loop(io, tmp_path).run('P01') == rec.EXIT_OK
    row = _rows(tmp_path / 'points.csv')[0]
    chain = _chain()
    assert row['label'] == 'P01' and row['tag_seen'] == '1' and row['still'] == '1'
    assert row['cam_n'] == str(rec.FRAMES_WANTED) and row['matched_n'] == str(rec.FRAMES_WANTED)
    assert float(row['cam_tag_z']) == pytest.approx(0.60)
    assert float(row['cam_normal_z']) == pytest.approx(1.0)
    assert float(row['incidence_deg']) == pytest.approx(
        math.degrees(math.atan2(math.hypot(0.10, 0.05), 0.60)), abs=0.01)
    fk_tag = tm.fk_tag(chain, Q_T, MOUNT.offset)
    assert [float(row['fk_tag_' + a]) for a in 'xyz'] == pytest.approx(fk_tag, abs=1e-6)
    assert [float(row['q_' + n]) for n in sr.JOINT_NAMES] == pytest.approx(Q_T, abs=1e-6)
    assert row['time'] == '1700000000.000' and io.flushes >= 2
    assert any('recorded P01 as row 1' in line for line in io.said)


def test_the_second_point_is_appended_by_a_second_run(tmp_path):
    assert _loop(FakeIo(), tmp_path).run('P01') == rec.EXIT_OK
    meta_before = rec.meta_path_for(tmp_path / 'points.csv').read_bytes()
    assert _loop(FakeIo(), tmp_path).run('P02') == rec.EXIT_OK
    assert [r['idx'] for r in _rows(tmp_path / 'points.csv')] == ['1', '2']
    assert rec.meta_path_for(tmp_path / 'points.csv').read_bytes() == meta_before


def test_a_tag_lost_after_the_confirmation_is_written_as_unseen(tmp_path):
    calls = {'n': 0}

    def source(index):
        calls['n'] += 1
        # The first measurement is steady; the second one (after the answer) sees nothing.
        return _frames(1, start=index) if calls['n'] <= rec.FRAMES_WANTED else _frames(0)

    io = FakeIo(source=source)
    assert _loop(io, tmp_path).run('P01') == rec.EXIT_NOT_SEEN
    row = _rows(tmp_path / 'points.csv')[0]
    assert row['tag_seen'] == '0' and row['cam_tag_x'] == '' and row['fk_tag_x'] != ''


def test_no_fresh_joint_angles_records_nothing(tmp_path):
    io = FakeIo(stale=True)
    assert _loop(io, tmp_path).run('P01') == rec.EXIT_NO_INPUT
    assert list(tmp_path.iterdir()) == []


def test_appending_waits_for_the_lock_of_another_run(tmp_path):
    path, row, meta = _first_point(tmp_path)
    rec.write_point(path, 'P01', IDENTITY, meta, row)
    done = threading.Event()

    def work():
        rec.write_point(path, 'P02', IDENTITY, meta, dict(row, label='P02'))
        done.set()

    thread = threading.Thread(target=work)
    with open(path, 'a+') as held:
        fcntl.flock(held, fcntl.LOCK_EX)
        thread.start()
        assert not done.wait(0.3)
        fcntl.flock(held, fcntl.LOCK_UN)
    thread.join(10)
    assert done.is_set() and [r['label'] for r in _rows(path)] == ['P01', 'P02']


# --- the functions the tool uses (it has no open of its own) ---

def _share_urdf():
    share = Path(get_package_share_directory('so_arm101_description'))
    return share / 'urdf' / 'so101_new_calib.urdf'


def test_load_chain_reads_the_urdf_file():
    chain = rec.load_chain(_share_urdf())
    assert chain.fk(Q_T)[0] == pytest.approx(_chain().fk(Q_T)[0])


def test_load_chain_of_a_missing_file_raises():
    with pytest.raises(OSError):
        rec.load_chain('/nonexistent/robot.urdf')


def _calibration(tmp_path, elbow=None, name='follower_calibration.yaml', gripper=None):
    """Write a follower calibration yaml (the key of elbow_flex only if given)."""
    joints = {n: {'id': i + 1, 'kind': 'angle', 'direction': 1, 'center': 2000.0}
              for i, n in enumerate(sr.JOINT_NAMES)}
    if elbow is not None:
        joints['elbow_flex']['zero_offset_deg'] = elbow
    joints['gripper'] = {'id': 6, 'kind': 'linear_percent', 'direction': 1}
    if gripper is not None:
        joints['gripper']['zero_offset_deg'] = gripper
    path = tmp_path / name
    path.write_text(yaml.safe_dump({'schema_version': 1, 'joints': joints}))
    return path


def test_the_identity_is_made_from_the_sha256_of_the_files(tmp_path):
    paths = []
    for name in ('tag_mount', 'urdf', 'apriltag', 'realsense'):
        path = tmp_path / (name + '.txt')
        path.write_text(name)
        paths.append(path)
    calibration = _calibration(tmp_path, elbow=-6.3)
    identity = rec.identity_from_files(*paths, 0, calibration)
    assert identity == {
        'tag_mount_sha256': hashlib.sha256(b'tag_mount').hexdigest(),
        'urdf_sha256': hashlib.sha256(b'urdf').hexdigest(),
        'apriltag_yaml_sha256': hashlib.sha256(b'apriltag').hexdigest(),
        'realsense_yaml_sha256': hashlib.sha256(b'realsense').hexdigest(),
        'tag_id': 0,
        'follower_calibration_sha256': hashlib.sha256(calibration.read_bytes()).hexdigest(),
        'zero_offset_deg': dict(ZERO, elbow_flex=-6.3)}
    assert tuple(identity) == rec.IDENTITY_KEYS


def test_the_identity_changes_with_the_file_it_comes_from(tmp_path):
    paths = []
    for name in ('a', 'b', 'c', 'd'):
        path = tmp_path / name
        path.write_text(name)
        paths.append(path)
    calibration = _calibration(tmp_path)
    before = rec.identity_from_files(*paths, 0, calibration)
    paths[3].write_text('changed')
    after = rec.identity_from_files(*paths, 0, calibration)
    assert [k for k in before if before[k] != after[k]] == ['realsense_yaml_sha256']
    with pytest.raises(OSError):
        rec.identity_from_files(*paths[:3], tmp_path / 'missing', 0, calibration)


def test_a_changed_calibration_yaml_changes_the_sha256_and_the_offset(tmp_path):
    paths = []
    for name in ('a', 'b', 'c', 'd'):
        path = tmp_path / name
        path.write_text(name)
        paths.append(path)
    before = rec.identity_from_files(*paths, 0, _calibration(tmp_path))
    after = rec.identity_from_files(*paths, 0, _calibration(tmp_path, elbow=-6.3, name='b.yaml'))
    assert [k for k in before if before[k] != after[k]] == [
        'follower_calibration_sha256', 'zero_offset_deg']


def test_zero_offsets_are_zero_where_the_key_is_missing_and_read_where_it_is_given(tmp_path):
    assert rec.read_zero_offsets(_calibration(tmp_path)) == ZERO
    assert rec.read_zero_offsets(_calibration(tmp_path, elbow=-6.3, name='b.yaml')) == dict(
        ZERO, elbow_flex=-6.3)
    assert rec.read_zero_offsets(_calibration(tmp_path, elbow=10, name='c.yaml'))[
        'elbow_flex'] == 10.0


@pytest.mark.parametrize('elbow', [10.001, -10.001, float('nan'), float('inf'), 'x', True])
def test_a_zero_offset_the_driver_would_refuse_is_refused(tmp_path, elbow):
    with pytest.raises(ValueError, match='elbow_flex'):
        rec.read_zero_offsets(_calibration(tmp_path, elbow=elbow))


def test_the_gripper_may_have_no_zero_offset_other_than_0(tmp_path):
    assert rec.read_zero_offsets(_calibration(tmp_path, gripper=0)) == ZERO
    with pytest.raises(ValueError, match='gripper'):
        rec.read_zero_offsets(_calibration(tmp_path, gripper=1.0, name='b.yaml'))


def test_a_calibration_yaml_that_cannot_be_read_is_refused(tmp_path):
    with pytest.raises(ValueError, match='cannot read'):
        rec.read_zero_offsets(tmp_path / 'missing.yaml')
    (tmp_path / 'bad.yaml').write_text('joints: 3\n')
    with pytest.raises(ValueError, match='no joints'):
        rec.read_zero_offsets(tmp_path / 'bad.yaml')


def test_an_older_meta_file_without_the_new_identity_values_is_refused(tmp_path):
    path, row, meta = _first_point(tmp_path)
    rec.write_point(path, 'P01', IDENTITY, meta, row)
    broken = yaml.safe_load(rec.meta_path_for(path).read_text())
    del broken['identity']['zero_offset_deg'], broken['identity']['follower_calibration_sha256']
    rec.meta_path_for(path).write_text(yaml.safe_dump(broken))
    with pytest.raises(rec.SessionError, match='zero_offset_deg: missing'):
        rec.check_session(path, 'P02', IDENTITY)


def test_camera_extra_has_plain_values_that_yaml_can_write():
    extra = rec.camera_extra(np.arange(9.0), np.zeros(5), np.int64(640), 480,
                             np.array([0.0, 0.015, 0.0]), (-0.5, 0.5, -0.5, 0.5))
    assert extra['camera_info']['k'] == [float(i) for i in range(9)]
    assert extra['camera_info']['width'] == 640 and extra['camera_info']['height'] == 480
    link = extra['camera_link_to_optical_frame']
    assert (link['parent'], link['child']) == ('camera_link', 'camera_color_optical_frame')
    assert link['translation_m'] == [0.0, 0.015, 0.0]
    assert yaml.safe_load(yaml.safe_dump(extra)) == extra


def test_camera_extra_goes_into_the_meta_file(tmp_path):
    extra = rec.camera_extra([1.0] * 9, [0.0] * 5, 640, 480, (0.0, 0.0, 0.0), (0, 0, 0, 1))
    meta = rec.build_meta(IDENTITY, MOUNT, (-0.055, 0.0, 0.035), '2026-11-16', extra)
    assert meta['extra']['camera_info']['width'] == 640


def test_investigating_needs_no_csv_but_recording_does(tmp_path):
    io = FakeIo()
    loop = rec.RecordLoop(io, _chain(), MOUNT, None, IDENTITY)
    assert loop.run('investigate', investigate_only=True) == rec.EXIT_OK
    io = FakeIo()
    loop = rec.RecordLoop(io, _chain(), MOUNT, None, IDENTITY)
    assert loop.run('P01') == rec.EXIT_REFUSED
    assert io.waits == [] and list(tmp_path.iterdir()) == []


# ------------------------------------------------------------------ C4: verification (pure)
# The poses are the q_goal of the 4 points of C4-0 (pitch -91 deg, a mark at (0.22, +0.08)).
REF = MOUNT.reference_point
C4_POSES = [np.radians(q) for q in (
    [-26.4, -1.4, 2.8, 89.6, 0.0], [-23.8, 8.0, -3.5, 86.5, 0.0],
    [-23.8, 8.1, -9.1, 92.0, 0.0], [-21.7, 18.2, -17.7, 90.5, 0.0])]


def _truth(delta_deg, chain=None):
    """Return the points (q_read, physical tip) of a arm whose true angle is read + delta."""
    chain = chain or _chain()
    d = np.radians(delta_deg)
    return [(q, rec.jaw_tip(chain, q + np.array([0.0, d[0], d[1], 0.0, 0.0]), REF))
            for q in C4_POSES]


def test_the_rear_edge_frame_becomes_base_link_x():
    assert rec.base_x_from_rear(0.221) == pytest.approx(0.1999, abs=1e-9)
    assert rec.base_x_from_rear(0.0) == pytest.approx(-0.0211)


def test_the_mark_in_base_link_from_the_ruler_measurements():
    x, y, z = rec.mark_in_base(0.221, 0.0, 0.0, 0.0005)
    assert (x, y) == (pytest.approx(0.1999), 0.0) and z == pytest.approx(-0.0024 + 0.0005)


def test_a_desk_higher_than_the_mounting_plane_is_a_positive_difference():
    delta, thickness = 0.003, 0.001  # the desk is 3 mm higher, ID 1 is 1 mm thick
    face = rec.tag_face_z(delta, thickness)
    assert face == pytest.approx(0.0016)
    z = 0.0596
    expected = rec.height_above_mount(z) - delta - thickness
    assert rec.height_above_face(z, face) == pytest.approx(expected)
    assert rec.height_above_face(z, face) == pytest.approx(0.058)


def test_a_desk_lower_than_the_mounting_plane_is_a_negative_difference():
    delta, thickness = -0.002, 0.0005  # the desk is 2 mm lower
    face = rec.tag_face_z(delta, thickness)
    assert face == pytest.approx(-0.0039)
    z = 0.0596
    expected = rec.height_above_mount(z) - delta - thickness
    assert rec.height_above_face(z, face) == pytest.approx(expected)
    assert rec.height_above_face(z, face) == pytest.approx(0.0635)


def test_the_height_above_the_mounting_plane_is_the_height_of_f6():
    assert rec.height_above_mount(0.05249) == pytest.approx(0.05489)  # F6: 54.89 mm


def test_a_hover_target_over_the_mark_with_the_floor_respected():
    mark = rec.mark_in_base(0.221, 0.0, 0.0, 0.0)
    hover = rec.hover_target(mark, 0.0, 0.062)
    assert hover.ok and hover.reason == ''
    assert hover.target == pytest.approx((0.1999, 0.0, 0.0596))
    assert hover.h_mount == pytest.approx(0.062) and hover.h_face == pytest.approx(0.062)


def test_a_hover_target_below_the_floor_is_refused_with_the_reason():
    mark = rec.mark_in_base(0.221, 0.0, 0.0, 0.0)
    hover = rec.hover_target(mark, 0.0, 0.055)  # z = 0.0526 m
    assert not hover.ok and 'below the floor' in hover.reason
    assert hover.target[2] == pytest.approx(0.0526)


def test_a_higher_desk_raises_the_target_for_the_same_height_above_the_face():
    mark = rec.mark_in_base(0.221, 0.0, 0.003, 0.001)
    hover = rec.hover_target(mark, 0.02, 0.058)
    assert hover.ok and hover.target == pytest.approx((0.2199, 0.0, 0.0596))
    assert hover.h_face == pytest.approx(0.058) and hover.h_mount == pytest.approx(0.062)


def test_a_lower_desk_lowers_the_target_for_the_same_height_above_the_face():
    mark = rec.mark_in_base(0.221, 0.0, -0.002, 0.0005)
    assert rec.hover_target(mark, 0.0, 0.057).target[2] == pytest.approx(0.0531)
    assert not rec.hover_target(mark, 0.0, 0.057).ok  # 0.0531 m is below the floor


def test_the_command_of_ik_move_is_the_one_that_is_typed():
    assert rec.ik_move_command((0.2, 0.0, 0.0596), -91.0, 0.0) == (
        'python3 -u tools/ik_move_run.py --goal 0.2000 0.0000 0.0596 --pitch-deg -91.0 '
        '--roll-deg 0.0')
    assert rec.ik_move_command((0.2, 0.0, 0.0596), -91.0, 0.0, dry_run=True).startswith(
        'python3 -u tools/ik_move_run.py --dry-run --goal ')


def test_the_physical_jaw_tip_is_the_mark_plus_the_ruler_reading():
    phys = rec.physical_jaw_tip((0.2, 0.0, -0.0024), 0.01, -0.005, 0.0556)
    assert phys == pytest.approx((0.21, -0.005, 0.0532))


def test_the_fraction_is_zero_at_the_reading_one_at_the_reference_and_half_between():
    read, ref = np.array([0.2, 0.0, 0.05]), np.array([0.2, 0.0, 0.056])
    assert rec.point_errors(read, read, ref)['fraction'] == pytest.approx(0.0)
    assert rec.point_errors(ref, read, ref)['fraction'] == pytest.approx(1.0)
    middle = rec.point_errors((read + ref) / 2.0, read, ref)
    assert middle['fraction'] == pytest.approx(0.5) and middle['d'] == pytest.approx(0.006)


def test_the_part_of_the_tip_off_the_line_is_reported_apart():
    read, ref = np.zeros(3), np.array([0.006, 0.0, 0.0])
    errors = rec.point_errors(np.array([0.003, 0.002, 0.0]), read, ref)
    assert errors['fraction'] == pytest.approx(0.5) and errors['perp'] == pytest.approx(0.002)


def test_two_equal_points_have_no_fraction():
    errors = rec.point_errors(np.ones(3), np.ones(3), np.ones(3))
    assert errors['fraction'] is None and errors['d'] == 0.0


def test_the_jaw_tip_is_the_tcp_plus_the_reference_point_in_the_gripper_axes():
    chain = _chain()
    tcp, rotation = chain.fk(Q_T)
    tip = rec.jaw_tip(chain, Q_T, REF)
    assert tip == pytest.approx(np.asarray(tcp) + np.asarray(rotation) @ np.asarray(REF))
    assert np.linalg.norm(tip - np.asarray(tcp)) == pytest.approx(0.0063)


def test_applying_the_zero_offset_twice_would_move_the_tip_by_about_23_mm():
    chain = _chain()  # F6: the elbow correction of -6.3 deg moves the TCP of T by 23.3 mm
    twice = Q_T + np.array([0.0, 0.0, math.radians(-6.3), 0.0, 0.0])
    moved = np.linalg.norm(rec.jaw_tip(chain, twice, (0.0, 0.0, 0.0))
                           - rec.jaw_tip(chain, Q_T, (0.0, 0.0, 0.0)))
    assert moved == pytest.approx(0.0233, abs=0.0005)


def test_the_age_of_a_transform_is_now_minus_its_stamp():
    assert rec.tf_age_s(2_500_000_000, 1_000_000_000) == pytest.approx(1.5)


def test_the_difference_of_the_reference_and_the_goal():
    vector, largest = rec.goal_difference([0.1, 0.2, 0.3, 0.4, 0.5], [0.1, 0.19, 0.31, 0.4, 0.5])
    assert largest == pytest.approx(0.01) and vector[1] == pytest.approx(0.01)
    assert rec.goal_difference(None, [0.0] * 5) == (None, None)


def test_the_tilt_of_a_tag_face_from_the_horizontal():
    assert rec.tag_tilt_deg((0.0, 0.0, 0.0, 1.0)) == pytest.approx(0.0)
    half = math.radians(10.0) / 2.0
    assert rec.tag_tilt_deg((math.sin(half), 0.0, 0.0, math.cos(half))) == pytest.approx(10.0)
    assert rec.tag_tilt_deg((math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))) == pytest.approx(90.0)


def _errors(fractions, d=0.007):
    read = np.zeros(3)
    ref = np.array([d, 0.0, 0.0])
    return [rec.point_errors(read + f * (ref - read), read, ref) for f in fractions]


def test_points_near_the_reference_decide_b():
    verdict, mean, std, used = rec.decide_ab(_errors([0.9, 1.0, 1.1, 0.8]))
    assert (verdict, used) == ('B', 4) and mean == pytest.approx(0.95)


def test_points_near_the_reading_decide_a():
    verdict, mean, _, used = rec.decide_ab(_errors([0.1, 0.0, -0.1]))
    assert (verdict, used) == ('A', 3) and mean == pytest.approx(0.0)


def test_only_the_points_with_a_difference_of_4_mm_or_more_decide():
    low = _errors([1.0, 1.0], d=0.0039)  # 3.9 mm: not used
    assert rec.decide_ab(low + _errors([0.0, 0.1]))[0] == 'A'
    assert rec.decide_ab(low + _errors([0.0, 0.1]))[3] == 2
    assert rec.decide_ab(_errors([0.0, 0.1], d=0.004))[3] == 2  # 4.0 mm: used


def test_fewer_than_two_points_with_a_difference_cannot_decide():
    errors = _errors([1.0], d=0.007) + _errors([1.0, 1.0], d=0.003)
    verdict, mean, std, used = rec.decide_ab(errors)
    assert (verdict, mean, std, used) == ('UNDECIDED', None, None, 1)


def test_exactly_half_is_not_decided():
    exact = [{'fraction': 0.25, 'd': 0.007}, {'fraction': 0.75, 'd': 0.007}]
    assert rec.decide_ab(exact)[0:2] == ('UNDECIDED', 0.5)


def test_the_deltas_that_made_the_points_are_found_again():
    chain = _chain()
    estimate = rec.estimate_delta(chain, _truth((0.5, -1.5), chain), REF)
    assert estimate.ok and estimate.delta_deg == pytest.approx((0.5, -1.5), abs=0.1)
    assert estimate.rms_m < 5e-4  # a linear fit of a small nonlinear effect


def test_a_positive_delta_means_the_true_angle_is_larger_than_the_reading():
    estimate = rec.estimate_delta(_chain(), _truth((2.0, 0.0)), REF)
    assert estimate.delta_deg[0] > 1.9 and abs(estimate.delta_deg[1]) < 0.2


def test_the_ruler_part_of_the_uncertainty_is_the_formula_of_the_least_squares():
    chain = _chain()
    stacked = np.vstack([rec.jaw_jacobian(chain, q, REF) for q in C4_POSES])
    expected = 0.002 * np.sqrt(np.diag(np.linalg.inv(stacked.T @ stacked))) * 180.0 / math.pi
    estimate = rec.estimate_delta(chain, _truth((0.0, 0.0), chain), REF, ruler_m=0.002)
    assert estimate.sd_ruler_deg == pytest.approx(tuple(expected))
    assert max(estimate.sd_ruler_deg) < 1.0  # C4-0: about 0.5 to 0.6 deg


def test_a_shift_common_to_every_point_moves_the_estimate_as_the_common_part_says():
    chain = _chain()
    base = rec.estimate_delta(chain, _truth((0.0, 0.0), chain), REF, common_m=(0.0, 0.0, 0.001))
    shifted = [(q, p + np.array([0.0, 0.0, 0.001])) for q, p in _truth((0.0, 0.0), chain)]
    moved = rec.estimate_delta(chain, shifted, REF)
    change = np.array(moved.delta_deg) - np.array(base.delta_deg)
    assert np.abs(change) == pytest.approx(np.array(base.sd_common_deg), abs=0.02)
    assert max(base.sd_common_deg) > 0.1  # C4-0: about 0.45 deg per mm of z
    assert base.sd_total_deg[0] >= base.sd_common_deg[0]


def test_deltas_that_cannot_be_told_apart_are_refused(monkeypatch):
    parallel = np.array([[1.0, 2.0], [0.0, 0.0], [1.0, 2.0]])
    monkeypatch.setattr(rec, 'jaw_jacobian', lambda *args, **kwargs: parallel)
    estimate = rec.estimate_delta(_chain(), _truth((0.5, -1.5)), REF)
    assert not estimate.ok and 'not separated' in estimate.reason


def test_fewer_than_two_points_give_no_estimate():
    assert not rec.estimate_delta(_chain(), _truth((0.5, -1.5))[:1], REF).ok


def test_no_function_takes_a_zero_offset_and_the_decisions_do_not_take_the_camera():
    for name in ('jaw_tip', 'jaw_jacobian', 'physical_jaw_tip', 'point_errors', 'decide_ab',
                 'hover_target', 'estimate_delta', 'goal_difference'):
        parameters = set(inspect.signature(getattr(rec, name)).parameters)
        assert not any(word in p for p in parameters for word in ('delta', 'offset', 'zero')), name
        assert not any('tfb' in p or 'tag' in p for p in parameters), name


def test_a_verification_row_is_written_once_per_label(tmp_path):
    path = tmp_path / 'verify.csv'
    row = {'time': '1.0', 'q_read_lift': '0.1', 'tfb_tag1_age_s': '0.5',
           'tfb_tag1_used_for_judgement': '0'}
    assert rec.write_verify_row(path, 'V01', row) == 1
    assert rec.write_verify_row(path, 'V02', row) == 2
    with pytest.raises(rec.SessionError):
        rec.write_verify_row(path, 'V01', row)
    with pytest.raises(rec.SessionError):
        rec.write_verify_row(path, 'bad label', row)
    with pytest.raises(rec.SessionError):
        rec.write_verify_row(path, 'V03', dict(row, nonsense='1'))
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    assert [r['label'] for r in rows] == ['V01', 'V02']
    assert rows[0]['tfb_tag1_used_for_judgement'] == '0' and rows[0]['tfb_tag1_age_s'] == '0.5'
    assert list(rows[0]) == list(rec.VERIFY_COLUMNS)


def test_the_goal_of_the_last_row_of_the_csv_of_ik_move_is_read(tmp_path):
    path = tmp_path / 'move.csv'
    path.write_text('input,q_goal,label\n'
                    'goal,0.1 0.2 0.3 0.4 0.5,a\n'
                    'goal,-0.00307 -0.38742 0.55160 1.44076 0.00000,b\n', encoding='utf-8')
    assert rec.read_move_goal(path) == pytest.approx([-0.00307, -0.38742, 0.5516, 1.44076, 0.0])
    path.write_text('input,q_goal\ngoal,0.1 0.2 0.3 0.4\n', encoding='utf-8')
    with pytest.raises(rec.SessionError):
        rec.read_move_goal(path)


# ----------------------------------------------- C4: tools/camera_verify_run.py (static and run)
VERIFY_TOOL = Path(__file__).resolve().parents[3] / 'tools' / 'camera_verify_run.py'
IK_PACKAGE_XML = Path(__file__).resolve().parents[2] / 'so_arm101_ik' / 'package.xml'
# control_msgs is not declared in the package.xml of so_arm101_camera: it comes through the
# exec_depend of so_arm101_ik (the same as tools/ik_move_run.py). It is handled by the clean-up of
# the exec_depend that follows C4.
VERIFY_IMPORTS = {
    'argparse', 'math', 'os', 'sys', 'time', 'ament_index_python', 'control_msgs', 'numpy',
    'rclpy', 'sensor_msgs', 'so_arm101_camera', 'so_arm101_ik', 'tf2_ros'}
VERIFY_FORBIDDEN = {
    'create_publisher', 'create_client', 'create_service', 'ActionClient', 'ActionServer',
    'publish', 'call_async', 'Serial', 'serial', 'pyrealsense2', 'subprocess', 'Popen', 'socket',
    'TransformBroadcaster', 'StaticTransformBroadcaster', 'sendTransform', 'write_text',
    'write_bytes', 'truncate', 'unlink', 'remove', 'rename', 'replace', 'rmtree', 'chmod',
    'ctypes', 'importlib', 'eval', 'exec', 'fork', 'forkpty', 'killpg', 'system', 'popen', 'kill'}
VERIFY_ARGUMENTS = {
    '--verify-plan', '--verify-survey', '--verify-record', '--verify-report', '--mark-rear',
    '--desk-minus-mount', '--thickness', '--offset', '--pitch-deg', '--roll-deg', '--start-deg',
    '--csv', '--label', '--move-csv', '--ruler', '--ruler-mm', '--common-mm', '--tag-mount',
    '--apriltag-yaml', '--urdf', '--calibration-yaml'}


def _verify_tree():
    import ast
    return ast.parse(VERIFY_TOOL.read_text(encoding='utf-8'))


def _verify_tool():
    import importlib.util
    spec = importlib.util.spec_from_file_location('camera_verify_run', VERIFY_TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_verify_tool_imports_only_the_allowed_modules():
    import ast
    import sys
    imported = set()
    for node in ast.walk(_verify_tree()):
        if isinstance(node, ast.Import):
            imported |= {a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            imported.add(node.module.split('.')[0])
    assert imported <= VERIFY_IMPORTS, imported - VERIFY_IMPORTS
    assert {m for m in imported if m not in sys.stdlib_module_names} <= VERIFY_IMPORTS


def test_control_msgs_comes_through_the_exec_depend_of_so_arm101_ik():
    import xml.etree.ElementTree as ElementTree
    root = ElementTree.parse(IK_PACKAGE_XML).getroot()
    assert 'control_msgs' in {e.text for e in root.findall('exec_depend')}


def test_the_verify_tool_names_nothing_that_sends_or_starts_a_process_or_removes_a_file():
    import ast
    names = set()
    for node in ast.walk(_verify_tree()):
        if isinstance(node, ast.Name):
            names.add(node.id)
        elif isinstance(node, ast.Attribute):
            names.add(node.attr)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            names |= {a.name.split('.')[-1] for a in node.names}
    assert not names & VERIFY_FORBIDDEN, names & VERIFY_FORBIDDEN
    assert not {n for n in names if n.startswith(('exec', 'spawn', 'kill'))}


def test_the_verify_tool_does_not_call_open():
    import ast
    calls = [n.lineno for n in ast.walk(_verify_tree()) if isinstance(n, ast.Call)
             and ((isinstance(n.func, ast.Name) and n.func.id == 'open')
                  or (isinstance(n.func, ast.Attribute) and n.func.attr == 'open'))]
    assert not calls, calls


def test_the_verify_tool_subscribes_to_the_joint_states_and_the_controller_state_only():
    import ast
    tree = _verify_tree()
    constants = {t.id: node.value.value for node in tree.body if isinstance(node, ast.Assign)
                 for t in node.targets if isinstance(t, ast.Name)
                 and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str)}
    topics = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'create_subscription'):
            topic = node.args[1]
            topics.append(topic.value if isinstance(topic, ast.Constant) else constants[topic.id])
    assert sorted(topics) == ['/arm_controller/controller_state', '/joint_states']


def test_the_arguments_of_the_verify_tool_are_exactly_the_expected_ones():
    import ast
    options = set()
    for node in ast.walk(_verify_tree()):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'add_argument'):
            options |= {a.value for a in node.args
                        if isinstance(a, ast.Constant) and str(a.value).startswith('--')}
    assert options == VERIFY_ARGUMENTS


def test_the_decisions_of_the_verify_tool_do_not_take_the_camera():
    import ast
    calls = [n for n in ast.walk(_verify_tree()) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr in ('decide_ab',
                                                                       'estimate_delta')]
    assert len(calls) == 2
    for call in calls:
        assert 'tfb' not in ast.unparse(call) and 'tag' not in ast.unparse(call)


def test_the_plan_needs_no_node_and_the_others_check_before_the_node():
    import ast
    functions = {n.name: n for n in _verify_tree().body if isinstance(n, ast.FunctionDef)}
    plan_names = {n.id for n in ast.walk(functions['run_plan']) if isinstance(n, ast.Name)}
    assert not plan_names & {'rclpy', 'VerifyNode'}
    for name in ('run_survey', 'run_record'):
        first = {}
        for node in ast.walk(functions[name]):
            if isinstance(node, ast.Call):
                func = node.func
                called = func.attr if isinstance(func, ast.Attribute) else getattr(func, 'id', '')
                first[called] = min(first.get(called, node.lineno), node.lineno)
        assert first['load_model'] < first['init'] < first['VerifyNode'], name


def test_no_line_of_the_verify_tool_is_longer_than_99_characters():
    lines = VERIFY_TOOL.read_text(encoding='utf-8').splitlines()
    assert [i + 1 for i, line in enumerate(lines) if len(line) > 99] == []


def test_the_verify_tool_refuses_a_run_without_the_measured_values(capsys):
    tool = _verify_tool()
    with pytest.raises(SystemExit):
        tool.parse_args(['--verify-plan', '--offset', '0', '0.062'])
    with pytest.raises(SystemExit):
        tool.parse_args(['--verify-record', '--mark-rear', '0.221', '0', '--desk-minus-mount',
                         '0', '--thickness', '0', '--offset', '0', '0.062'])
    args = tool.parse_args(['--verify-report', '--csv', 'x.csv'])
    assert tool.check_args(args) == []


def test_the_plan_prints_the_targets_the_commands_and_the_heights(capsys):
    tool = _verify_tool()
    code = tool.main(['--verify-plan', '--mark-rear', '0.221', '0', '--desk-minus-mount', '0',
                      '--thickness', '0', '--offset', '0', '0.062', '--offset', '0.02', '0.062'])
    out = capsys.readouterr().out
    assert code == 0
    assert '--goal 0.1999 0.0000 0.0596 --pitch-deg -91.0 --roll-deg 0.0' in out
    assert '--dry-run --goal 0.2199 0.0000 0.0596' in out
    assert 'H 0.0620 m' in out and 'above the ID 1 face 0.0620 m' in out


def test_the_plan_refuses_a_target_below_the_floor(capsys):
    tool = _verify_tool()
    code = tool.main(['--verify-plan', '--mark-rear', '0.221', '0', '--desk-minus-mount', '0',
                      '--thickness', '0', '--offset', '0', '0.055'])
    out = capsys.readouterr().out
    assert code == rec.EXIT_REFUSED and 'REFUSED' in out and '--goal' not in out


def test_a_higher_desk_changes_the_target_of_the_plan(capsys):
    tool = _verify_tool()
    tool.main(['--verify-plan', '--mark-rear', '0.221', '0', '--desk-minus-mount', '0.003',
               '--thickness', '0.001', '--offset', '0', '0.058'])
    assert '--goal 0.1999 0.0000 0.0596' in capsys.readouterr().out


def _report_csv(tmp_path, physical):
    """Return the path of a CSV of 4 points; physical = 'read' or 'ref' (where the tip is)."""
    chain = _chain()
    path = tmp_path / 'verify.csv'
    mark = rec.mark_in_base(0.221, 0.0, 0.0, 0.0)
    for i, q in enumerate(C4_POSES):
        q_ref = q - np.array([0.0, 0.0, 0.034, 0.0, 0.0])
        fk_read = rec.jaw_tip(chain, q, REF)
        fk_ref = rec.jaw_tip(chain, q_ref, REF)
        phys = fk_read if physical == 'read' else fk_ref
        ruler = (phys[0] - mark[0], phys[1] - mark[1], phys[2] - mark[2])
        errors = rec.point_errors(phys, fk_read, fk_ref)
        row = rec.build_verify_row(
            1.0 + i, (0.221, 0.0), 0.0, 0.0, offset=(0.0, 0.062), q_read=q, q_ref=q_ref,
            ruler=ruler, fk_read=fk_read, fk_ref=fk_ref, phys=phys, errors=errors,
            tfb=(0.1, 0.1, 0.1, 5.0), zero_offset={'elbow_flex': -6.3})
        rec.write_verify_row(path, f'V{i + 1:02d}', row)
    survey = rec.build_verify_row(0.0, (0.221, 0.0), 0.0, 0.0, tfb=(0.2005, 0.0, -0.0024, 0.4))
    rec.write_verify_row(path, 'S01', survey)
    return path


@pytest.mark.parametrize('physical, verdict', [('ref', 'B'), ('read', 'A')])
def test_the_report_decides_from_the_ruler_and_not_the_camera(tmp_path, capsys, physical, verdict):
    tool = _verify_tool()
    path = _report_csv(tmp_path, physical)
    assert tool.main(['--verify-report', '--csv', str(path)]) == 0
    out = capsys.readouterr().out
    assert f'input): {verdict}\n' in out
    assert '4 points used' in out and 'REFERENCE VALUE (not a result)' in out
    assert 'S01  (a) tfb_tag1 - ruler' in out  # the survey row, with the camera
    assert 'delta is not written anywhere' in out


def test_a_verification_row_has_the_heights_in_both_bases_and_never_uses_the_camera():
    chain = _chain()
    fk = rec.jaw_tip(chain, Q_T, REF)
    row = rec.build_verify_row(1.0, (0.221, 0.0), 0.003, 0.001, fk_read=fk, fk_ref=fk,
                               tfb=(0.2, 0.0, 0.0, 0.7))
    assert set(row) == set(rec.VERIFY_COLUMNS)
    assert row['tfb_tag1_used_for_judgement'] == '0' and row['tfb_tag1_age_s'] == '0.700000'
    assert float(row['fk_read_h_mount']) == pytest.approx(fk[2] + 0.0024, abs=1e-6)
    assert float(row['fk_read_h_face']) == pytest.approx(fk[2] - 0.0016, abs=1e-6)  # face z
    assert row['q_read_pan'] == '' and row['zero_offset_deg'] == ''


def test_the_rows_of_a_verification_csv_are_read_back(tmp_path):
    path = tmp_path / 'verify.csv'
    rec.write_verify_row(path, 'V01', rec.build_verify_row(1.0, (0.221, 0.0), 0.0, 0.0,
                                                           q_read=Q_T, still=True))
    rows = rec.read_verify_rows(path)
    assert [r['label'] for r in rows] == ['V01'] and rows[0]['still'] == '1'
    assert float(rows[0]['q_read_elbow']) == pytest.approx(math.radians(30.0), abs=1e-6)
