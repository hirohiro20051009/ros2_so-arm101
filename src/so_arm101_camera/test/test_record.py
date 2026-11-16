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
IDENTITY = rec.make_identity('a' * 64, 'b' * 64, 'c' * 64, 'd' * 64, 0)
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
    ('realsense_yaml_sha256', 'x' * 64), ('tag_id', 1)])
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
