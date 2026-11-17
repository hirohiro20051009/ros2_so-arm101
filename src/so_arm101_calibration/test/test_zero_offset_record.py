# Copyright 2026 so_arm101 authors. MIT License.
"""Tests of zero_offset_record.py: the refusals, the CSV and meta files and the identity."""

import hashlib
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_calibration import zero_offset as zo
from so_arm101_calibration import zero_offset_record as zr
from so_arm101_ik.urdf_kinematics import UrdfChain
import yaml

SHARE = Path(get_package_share_directory('so_arm101_description'))
URDF = SHARE / 'urdf' / 'so101_new_calib.urdf'
URDF_XML = URDF.read_text(encoding='utf-8')
CHAIN = UrdfChain(URDF_XML)
JAW = zo.FixedJaw.from_urdf(URDF_XML, SHARE / 'meshes')
ZERO = {name: 0.0 for name in zr.ARM_JOINTS}
# P3 of the plan (elevation 70, x 150) from the study; a record is the mean of the six joints.
Q_P3 = np.array([0.0, -1.332008, 1.470077, 1.083677, 0.0, 0.0])
STILL = np.zeros(6)
IDENTITY = zr.make_identity('a' * 64, ZERO, 'b' * 64, {'m1.stl': 'c' * 64, 'm2.stl': 'd' * 64})


def row_of(label='P3#1', q=Q_P3, span=STILL):
    return zr.make_row(label, 1000.0, q, span, 20, CHAIN, JAW)


def meta_of(identity=IDENTITY):
    return zr.build_meta(identity, zo.Options(), '2026-11-17T00:00:00Z')


def with_elevation(eps):
    """Return the P3 state with its wrist moved so that the elevation is eps (deg)."""
    q = Q_P3.copy()
    _, now = zo.tool_state(CHAIN, q[:5])
    q[3] += math.radians(eps - now)
    assert zo.tool_state(CHAIN, q[:5])[1] == pytest.approx(eps, abs=0.05)
    return q


# ----------------------------------------------------------------------------- one record

def test_a_still_state_becomes_a_row_with_the_columns_of_the_csv():
    row = row_of()
    assert list(row) == list(zr.COLUMNS)
    assert row['label'] == 'P3#1' and row['pose'] == 'P3' and row['still'] == '1'
    assert float(row['x_mm']) == pytest.approx(150.0, abs=10.0)
    assert float(row['eps_deg']) == pytest.approx(70.0, abs=3.0)
    assert float(row['q_elbow_flex']) == pytest.approx(1.470077, abs=1e-6)


def test_a_state_that_is_not_still_is_not_recorded():
    span = np.array([0.0, 0.0, 0.0, 0.21, 0.0, 0.0])
    with pytest.raises(zr.RecordRefused, match='not still'):
        row_of(span=span)
    assert row_of(span=np.array([0.2, 0.2, 0.2, 0.2, 0.2, 5.0]))['still'] == '1'


@pytest.mark.parametrize('eps, accepted', [(45.05, True), (44.9, False)])
def test_an_elevation_below_45_deg_is_not_recorded(eps, accepted):
    q = with_elevation(eps)
    if accepted:
        assert row_of(q=q)['eps_deg'] != ''
    else:
        with pytest.raises(zr.RecordRefused, match='below 45'):
            row_of(q=q)


# ----------------------------------------------------------------------------- the files

def write(tmp_path, label='P3#1', identity=IDENTITY, name='a.csv'):
    path = tmp_path / name
    zr.write_point(path, label, identity, meta_of(identity), row_of(label))
    return path


@pytest.mark.parametrize('key, value', [
    ('follower_calibration_sha256', 'x' * 64), ('zero_offset_deg', dict(ZERO, elbow_flex=-6.3)),
    ('urdf_sha256', 'x' * 64), ('calibration_kind', 'other')])
def test_a_record_of_another_identity_is_not_appended(tmp_path, key, value):
    path = write(tmp_path)
    before = path.read_bytes()
    with pytest.raises(zr.SessionError, match=key):
        zr.write_point(path, 'P3#2', dict(IDENTITY, **{key: value}), meta_of(), row_of('P3#2'))
    assert path.read_bytes() == before


def test_the_csv_starts_with_the_comment_line_and_the_reading_skips_it(tmp_path):
    path = write(tmp_path)
    assert path.read_text().splitlines()[0] == zr.CSV_COMMENT
    header, rows = zr.read_rows(path)
    assert header == list(zr.COLUMNS) and len(rows) == 1
    touches = zr.read_touches(path)
    assert touches[0].label == 'P3#1'
    assert touches[0].q == pytest.approx(tuple(Q_P3[:5]), abs=1e-6)


def test_a_csv_without_the_comment_line_is_refused(tmp_path):
    path = write(tmp_path)
    lines = path.read_text().splitlines()
    path.write_text('\n'.join(lines[1:]) + '\n')
    with pytest.raises(zr.SessionError, match='comment line'):
        zr.check_file(path, IDENTITY)


def test_the_rows_that_are_already_in_the_csv_are_never_rewritten(tmp_path):
    path = write(tmp_path)
    before = path.read_bytes()
    assert zr.write_point(path, 'P3#2', IDENTITY, meta_of(), row_of('P3#2')) == 2
    assert path.read_bytes().startswith(before)
    assert [r[1] for r in zr.read_rows(path)[1]] == ['P3#1', 'P3#2']


def test_a_label_that_is_used_or_not_a_label_is_refused(tmp_path):
    path = write(tmp_path)
    with pytest.raises(zr.SessionError, match='already'):
        zr.check_session(path, 'P3#1', IDENTITY)
    for bad in ('', 'P3 1', 'P3#', 'P3#x', '../x'):
        with pytest.raises(zr.SessionError, match='label'):
            zr.check_session(path, bad, IDENTITY)


def test_the_meta_file_is_a_zero_offset_one_with_the_settings(tmp_path):
    path = write(tmp_path)
    meta = yaml.safe_load(zr.meta_path_for(path).read_text())
    assert meta['calibration_kind'] == 'zero_offset'
    assert meta['identity'] == yaml.safe_load(yaml.safe_dump(IDENTITY))
    assert meta['settings']['h_prior_mm'] == -2.4 and meta['settings']['eps_min_deg'] == 45.0
    assert meta['settings']['sigma_90_extra_mm'] == 1.0


def test_a_meta_file_of_another_kind_is_not_read(tmp_path):
    path = write(tmp_path)
    meta_path = zr.meta_path_for(path)
    meta = yaml.safe_load(meta_path.read_text())
    meta['calibration_kind'] = 'camera'
    meta_path.write_text(yaml.safe_dump(meta))
    with pytest.raises(zr.SessionError, match='calibration_kind'):
        zr.read_meta(path)
    meta_path.unlink()
    with pytest.raises(zr.SessionError, match='exists but'):
        zr.check_file(path, IDENTITY)


def test_the_next_label_follows_the_poses_and_the_typed_commands_are_parsed():
    assert zr.next_label(()) == 'P1#1'
    assert zr.next_label(('P1#1', 'P1#2', 'P2#1')) == 'P2#2'
    assert zr.next_label(tuple(f'{p}#{n}' for p in zr.POSES for n in (1, 2))) is None
    assert zr.parse_command('\n') == ('record', None)
    assert zr.parse_command('p3') == ('target', 'P3')
    assert zr.parse_command('Q') == ('quit', None)
    assert zr.parse_command('y') == ('confirm', None)
    assert zr.parse_command('zzz')[0] == 'unknown'


def test_the_target_range_is_10_mm_and_5_deg():
    assert zr.target_delta('P3', 160.0, 75.0)[2] is True
    assert zr.target_delta('P3', 160.1, 70.0)[2] is False
    assert zr.target_delta('P3', 150.0, 75.1)[2] is False


def test_the_window_gives_the_mean_and_the_span_in_degrees():
    samples = np.array([[0.0] * 6, [math.radians(0.1)] * 6, [math.radians(0.3)] * 6])
    mean, span = zr.window_stats(samples)
    assert mean[0] == pytest.approx(math.radians(0.4 / 3))
    assert span[0] == pytest.approx(0.3)
    assert zr.is_still(span) is False and zr.is_still(span * 0.5) is True


def test_the_identity_is_made_from_the_files_that_were_used(tmp_path):
    calibration = calibration_yaml(tmp_path, elbow=-6.3)
    identity = zr.identity_from_files(calibration, URDF, SHARE / 'meshes')
    assert identity['follower_calibration_sha256'] == hashlib.sha256(
        calibration.read_bytes()).hexdigest()
    assert identity['urdf_sha256'] == hashlib.sha256(URDF.read_bytes()).hexdigest()
    assert identity['zero_offset_deg'] == dict(ZERO, elbow_flex=-6.3)
    assert sorted(identity['mesh_sha256']) == ['sts3215_03a_v1.stl',
                                               'wrist_roll_follower_so101_v1.stl']
    assert identity['calibration_kind'] == 'zero_offset'


# ------------------------------------------------------------ the same rules as the camera

def calibration_yaml(tmp_path, elbow=None, name='follower_calibration.yaml', gripper=None):
    """Write a follower calibration yaml (the key of elbow_flex only if given)."""
    joints = {n: {'id': i + 1, 'kind': 'angle', 'direction': 1, 'center': 2000.0}
              for i, n in enumerate(zr.ARM_JOINTS)}
    if elbow is not None:
        joints['elbow_flex']['zero_offset_deg'] = elbow
    joints['gripper'] = {'id': 6, 'kind': 'linear_percent', 'direction': 1}
    if gripper is not None:
        joints['gripper']['zero_offset_deg'] = gripper
    path = tmp_path / name
    path.write_text(yaml.safe_dump({'schema_version': 1, 'joints': joints}))
    return path


def test_zero_offsets_are_zero_where_the_key_is_missing_and_read_where_it_is_given(tmp_path):
    assert zr.read_zero_offsets(calibration_yaml(tmp_path)) == ZERO
    assert zr.read_zero_offsets(calibration_yaml(tmp_path, elbow=-6.3, name='b.yaml')) == dict(
        ZERO, elbow_flex=-6.3)
    assert zr.read_zero_offsets(calibration_yaml(tmp_path, elbow=10, name='c.yaml'))[
        'elbow_flex'] == 10.0


@pytest.mark.parametrize('elbow', [10.001, -10.001, float('nan'), float('inf'), 'x', True])
def test_a_zero_offset_the_driver_would_refuse_is_refused(tmp_path, elbow):
    with pytest.raises(ValueError, match='elbow_flex'):
        zr.read_zero_offsets(calibration_yaml(tmp_path, elbow=elbow))


def test_the_gripper_may_have_no_zero_offset_other_than_0(tmp_path):
    assert zr.read_zero_offsets(calibration_yaml(tmp_path, gripper=0)) == ZERO
    with pytest.raises(ValueError, match='gripper'):
        zr.read_zero_offsets(calibration_yaml(tmp_path, gripper=1.0, name='b.yaml'))


def test_a_calibration_yaml_that_cannot_be_read_is_refused(tmp_path):
    with pytest.raises(ValueError, match='cannot read'):
        zr.read_zero_offsets(tmp_path / 'missing.yaml')
    (tmp_path / 'bad.yaml').write_text('joints: 3\n')
    with pytest.raises(ValueError, match='no joints'):
        zr.read_zero_offsets(tmp_path / 'bad.yaml')


def test_a_joint_that_is_not_an_angle_joint_is_refused(tmp_path):
    path = calibration_yaml(tmp_path)
    data = yaml.safe_load(path.read_text())
    data['joints']['wrist_roll']['kind'] = 'linear_percent'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='wrist_roll is not an angle joint'):
        zr.read_zero_offsets(path)


def test_a_changed_calibration_yaml_changes_the_sha256_and_the_offset(tmp_path):
    before = zr.identity_from_files(calibration_yaml(tmp_path), URDF, SHARE / 'meshes')
    after = zr.identity_from_files(calibration_yaml(tmp_path, elbow=-6.3, name='b.yaml'), URDF,
                                   SHARE / 'meshes')
    assert [k for k in before if before[k] != after[k]] == [
        'follower_calibration_sha256', 'zero_offset_deg']
