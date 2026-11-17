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

"""Tests of --joint-offset: q_true = q_meas + delta, the checks, the record, no double count."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_camera import extrinsics as ex
from so_arm101_camera import record as rec
from so_arm101_camera import tag_mount as tm
from test_camera_extrinsics_tool import tool
from test_extrinsics import meta_dict, row_of, TRUTHS, world, write_csv, write_meta
import yaml

URDF = Path(get_package_share_directory('so_arm101_description')) / 'urdf' / (
    'so101_new_calib.urdf')
OFFSET = (-0.038, 0.0, -0.0557)  # the tag offset of the meta file of C2
CHAIN = rec.load_chain(URDF)
NAMES = ex.ANGLE_JOINTS
DELTA_DEG = -6.3

# The 13 measured angles of C2 (rad), copied as values from the q_ columns of
# ~/so_arm101_check/camera_20261116_C2/c2_points.csv (sha256
# 8a4fc4b80bd3192ecd7e0bc9fc10f4da9dba5462457175769e238a201d30a229). The test does not read that
# file. The points are spread enough to pass every gate (the smallest spread was 31.8 mm).
POSES = [
    (0.092061, -1.090160, 0.889159, 0.756437, -0.006905),
    (0.073649, -0.438058, 0.545463, 0.471047, -0.006905),
    (-0.141161, -0.415043, 0.531654, 0.471047, -0.006905),
    (-0.185657, -1.056404, 0.879953, 0.718078, -0.006905),
    (-0.448032, -0.884556, 0.835457, 0.624483, -0.003836),
    (-0.355970, -0.286157, 0.421181, 0.458772, -0.006905),
    (-0.351367, -0.037592, 0.751067, -0.168779, -0.006905),
    (-0.448032, -0.634456, 1.297297, -0.119680, -0.005370),
    (-0.194863, -0.947464, 1.444596, 0.042962, -0.005370),
    (-0.150367, -0.149600, 0.913709, -0.185657, -0.006905),
    (0.070580, -0.144997, 0.921380, -0.187191, -0.006905),
    (0.092061, -1.007304, 1.464542, 0.079786, -0.006905),
    (0.092061, -1.088625, 0.929052, 0.734956, -0.006905)]


def true_q(q, delta_deg=DELTA_DEG):
    """Return q with the elbow corrected: q_true = q_meas + delta."""
    return np.array(q) + np.radians([0.0, 0.0, delta_deg, 0.0, 0.0])


def make_dataset(tmp_path, recorded='zero', delta_deg=DELTA_DEG, name='c2.csv'):
    """
    Write a CSV like C2 and its meta file; return the path of the CSV.

    q is the measured angle, fk_tag is computed from it as record.py does, and the camera sees
    the tag where the true angle puts it (q + delta). recorded: 'zero' (the meta file says all
    offsets are 0), 'legacy' (no record at all) or a dict (the zero_offset_deg it records).
    """
    q_rounded = [np.round(q, 6) for q in POSES]
    base_true = np.array([tm.fk_tag(CHAIN, true_q(q, delta_deg), OFFSET) for q in q_rounded])
    camera, _ = world(TRUTHS[0], box=base_true)
    rows = []
    for i, (q, c) in enumerate(zip(q_rounded, camera)):
        fk = tm.fk_tag(CHAIN, q, OFFSET)
        overrides = {f'q_{n}': f'{v:.6f}' for n, v in zip(NAMES, q)}
        overrides.update({f'fk_tag_{a}': f'{v:.6f}' for a, v in zip('xyz', fk)})
        rows.append(row_of(f'P{i + 1:02d}', c, base_true[i], **overrides))
    csv_path = tmp_path / name
    write_csv(csv_path, rows)
    meta = meta_dict(zero_offsets=recorded != 'legacy')
    meta['identity']['urdf_sha256'] = ex.sha256_file(URDF)
    meta['tag_mount']['offset_gripper_frame_m'] = list(OFFSET)
    if isinstance(recorded, dict):
        meta['identity']['zero_offset_deg'] = recorded
    write_meta(tmp_path / (name + '.meta.yaml'), meta)
    return csv_path


def run(args):
    return tool.main([str(a) for a in args])


def rows_and_meta(csv_path):
    rows = ex.read_csv_rows(csv_path)
    meta, _ = ex.read_meta(str(csv_path) + '.meta.yaml')
    return rows, meta


def recompute(rows, meta, offsets, urdf=URDF):
    """Return (rows, plan) of ex.recompute_fk_tag for the offsets (degrees by joint name)."""
    return ex.recompute_fk_tag(rows, meta, urdf, ex.plan_offsets(meta, offsets))


def corrected_record():
    """Return the zero_offset_deg of a meta file made after the driver corrected the elbow."""
    return dict({n: 0.0 for n in NAMES}, elbow_flex=DELTA_DEG)


# ----------------------------------------------------------------------------- the arguments

def test_the_values_are_read_with_the_sign_of_q_true_minus_q_meas():
    assert ex.parse_joint_offsets(['elbow_flex=-6.3', 'wrist_flex=0.5']) == {
        'elbow_flex': -6.3, 'wrist_flex': 0.5}
    assert ex.parse_joint_offsets([]) == {}


def test_the_limit_of_10_degrees_and_zero_are_accepted():
    assert ex.parse_joint_offsets(['elbow_flex=10', 'wrist_flex=-10', 'wrist_roll=0']) == {
        'elbow_flex': 10.0, 'wrist_flex': -10.0, 'wrist_roll': 0.0}


@pytest.mark.parametrize('item', [
    'elbow_flex', '=1', 'elbow_flex=', 'elbow_flex=abc', 'elbow_flex=nan', 'elbow_flex=inf',
    'elbow_flex=10.01', 'elbow_flex=-10.01', 'wrist_pitch=1', 'gripper=0', 'gripper=1'])
def test_a_bad_argument_is_refused(item):
    with pytest.raises(ex.InputError) as refused:
        ex.parse_joint_offsets([item])
    if item.startswith('gripper'):
        assert 'no zero offset' in str(refused.value)


@pytest.mark.parametrize('items', [['elbow_flex=1', 'elbow_flex=2'],
                                   ['elbow_flex=1', 'elbow_flex=1']])
def test_a_joint_given_twice_is_refused(items):
    with pytest.raises(ex.InputError, match='twice'):
        ex.parse_joint_offsets(items)


# ----------------------------------------------------------------------------- no double count

def test_an_older_meta_file_is_used_only_together_with_joint_offset(tmp_path, capsys):
    csv_path = make_dataset(tmp_path, recorded='legacy', delta_deg=0.0)
    assert run([csv_path, '--output', tmp_path / 'x.yaml']) == 2
    assert 'no record of the zero offsets' in capsys.readouterr().out
    assert run([csv_path, '--output', tmp_path / 'x.yaml', '--urdf', URDF,
                '--joint-offset', 'elbow_flex=0']) == 0


def test_a_meta_file_that_recorded_zero_is_used_with_or_without_joint_offset(tmp_path, capsys):
    csv_path = make_dataset(tmp_path, recorded='zero', delta_deg=0.0)
    assert run([csv_path, '--output', tmp_path / 'x.yaml']) == 0
    assert 'none applied' in capsys.readouterr().out
    csv_path = make_dataset(tmp_path, recorded='zero', name='b.csv')
    assert run([csv_path, '--output', tmp_path / 'x.yaml', '--urdf', URDF,
                '--joint-offset', f'elbow_flex={DELTA_DEG}']) == 0


def test_joint_offset_on_a_meta_file_that_recorded_an_offset_is_refused(tmp_path, capsys):
    csv_path = make_dataset(tmp_path, recorded=corrected_record(), delta_deg=0.0)
    for value in ('-1', '0'):
        assert run([csv_path, '--output', tmp_path / 'x.yaml', '--urdf', URDF,
                    '--joint-offset', f'elbow_flex={value}']) == 2
        assert 'twice' in capsys.readouterr().out


def test_a_meta_file_that_recorded_an_offset_is_used_as_it_is_without_joint_offset(
        tmp_path, capsys):
    csv_path = make_dataset(tmp_path, recorded=corrected_record(), delta_deg=0.0)
    assert run([csv_path, '--output', tmp_path / 'x.yaml']) == 0
    text = capsys.readouterr().out
    assert 'refused' not in text and 'none applied' in text


@pytest.mark.parametrize('bad', [
    {'elbow_flex': 0.0}, dict.fromkeys(NAMES, True), dict.fromkeys(NAMES, 10.5), 'zero'])
def test_a_malformed_record_of_the_offsets_is_refused(bad):
    meta = meta_dict()
    meta['identity']['zero_offset_deg'] = bad
    with pytest.raises(ex.InputError):
        ex.plan_offsets(meta, {})


# ----------------------------------------------------------------------------- the recomputation

def test_the_fk_tag_is_computed_from_q_plus_the_offset(tmp_path):
    rows, meta = rows_and_meta(make_dataset(tmp_path))
    new_rows, plan = recompute(rows, meta, {'elbow_flex': DELTA_DEG})
    for old, new in zip(rows, new_rows):
        q = np.array([float(old['q_' + n]) for n in NAMES])
        expected = tm.fk_tag(CHAIN, q + np.radians([0, 0, DELTA_DEG, 0, 0]), OFFSET)
        got = np.array([float(new['fk_tag_' + a]) for a in 'xyz'])
        before = np.array([float(old['fk_tag_' + a]) for a in 'xyz'])
        assert np.allclose(got, expected, atol=1e-12)
        assert np.linalg.norm(got - before) > 0.005
    assert plan.fk_source == 'recomputed' and plan.baseline_max_m < 5e-6


def test_a_negative_offset_lowers_the_elbow_angle_before_the_fk(tmp_path):
    rows, meta = rows_and_meta(make_dataset(tmp_path))
    new_rows, _ = recompute(rows, meta, {'elbow_flex': -6.3})
    q = np.array([float(rows[0]['q_' + n]) for n in NAMES])
    lowered = q.copy()
    lowered[2] -= math.radians(6.3)
    raised = q.copy()
    raised[2] += math.radians(6.3)
    got = np.array([float(new_rows[0]['fk_tag_' + a]) for a in 'xyz'])
    assert np.allclose(got, tm.fk_tag(CHAIN, lowered, OFFSET), atol=1e-12)
    assert not np.allclose(got, tm.fk_tag(CHAIN, raised, OFFSET), atol=1e-4)


def test_zero_offsets_give_back_the_fk_tag_of_the_csv(tmp_path):
    rows, meta = rows_and_meta(make_dataset(tmp_path, recorded='legacy'))
    new_rows, plan = recompute(rows, meta, {'elbow_flex': 0})
    for old, new in zip(rows, new_rows):
        assert np.allclose([float(new['fk_tag_' + a]) for a in 'xyz'],
                           [float(old['fk_tag_' + a]) for a in 'xyz'], atol=2e-6)
    assert 0.0 <= plan.baseline_max_m < 5e-6


def test_a_joint_that_is_not_given_keeps_its_angle(tmp_path):
    rows, meta = rows_and_meta(make_dataset(tmp_path))
    new_rows, _ = recompute(rows, meta, {'wrist_flex': 2.0})
    q = np.array([float(rows[3]['q_' + n]) for n in NAMES])
    q[3] += math.radians(2.0)
    assert np.allclose([float(new_rows[3]['fk_tag_' + a]) for a in 'xyz'],
                       tm.fk_tag(CHAIN, q, OFFSET), atol=1e-12)


def test_the_rows_that_were_given_are_not_changed(tmp_path):
    rows, meta = rows_and_meta(make_dataset(tmp_path))
    before = [dict(r) for r in rows]
    recompute(rows, meta, {'elbow_flex': DELTA_DEG})
    assert rows == before


def test_a_urdf_that_is_not_the_one_of_the_meta_file_is_refused(tmp_path):
    rows, meta = rows_and_meta(make_dataset(tmp_path))
    other = tmp_path / 'other.urdf'
    other.write_text(URDF.read_text() + '\n<!-- changed -->\n')
    with pytest.raises(ex.InputError, match='urdf_sha256'):
        recompute(rows, meta, {'elbow_flex': 0}, urdf=other)
    with pytest.raises(ex.InputError, match='cannot read'):
        recompute(rows, meta, {}, urdf=tmp_path / 'missing')


@pytest.mark.parametrize('column,value', [('fk_tag_z', 0.0001), ('q_elbow_flex', 0.001)])
def test_a_csv_whose_fk_tag_does_not_follow_from_its_q_is_refused(tmp_path, column, value):
    rows, meta = rows_and_meta(make_dataset(tmp_path))
    rows[4][column] = f'{float(rows[4][column]) + value:.6f}'
    with pytest.raises(ex.InputError, match='P05'):
        recompute(rows, meta, {'elbow_flex': DELTA_DEG})


def test_the_tag_offset_of_the_meta_file_is_used(tmp_path):
    rows, meta = rows_and_meta(make_dataset(tmp_path))
    meta['tag_mount']['offset_gripper_frame_m'] = [-0.038, 0.0, -0.05]
    with pytest.raises(ex.InputError, match='P01'):
        recompute(rows, meta, {'elbow_flex': 0})
    del meta['tag_mount']['offset_gripper_frame_m']
    with pytest.raises(ex.InputError, match='offset_gripper_frame_m'):
        recompute(rows, meta, {'elbow_flex': 0})


# ----------------------------------------------------------------------------- the tool

def test_the_offset_of_the_truth_gives_a_perfect_fit_and_a_wrong_one_does_not(tmp_path, capsys):
    csv_path = make_dataset(tmp_path)
    out = tmp_path / 'x.yaml'
    assert run([csv_path, '--output', out, '--urdf', URDF,
                '--joint-offset', f'elbow_flex={DELTA_DEG}']) == 0
    text = capsys.readouterr().out
    assert 'zero offsets applied' in text and 'elbow_flex -6.3 deg' in text
    assert 'RMS 0.00 mm' in text
    run([csv_path, '--output', out, '--urdf', URDF, '--joint-offset', 'elbow_flex=0'])
    assert 'RMS 0.00 mm' not in capsys.readouterr().out


def test_the_original_csv_and_meta_are_not_written(tmp_path):
    csv_path = make_dataset(tmp_path)
    meta_path = tmp_path / 'c2.csv.meta.yaml'
    before = (csv_path.read_bytes(), meta_path.read_bytes())
    out = tmp_path / 'x.yaml'
    assert run([csv_path, '--output', out, '--urdf', URDF, '--write',
                '--joint-offset', f'elbow_flex={DELTA_DEG}']) == 0
    assert (csv_path.read_bytes(), meta_path.read_bytes()) == before
    assert sorted(p.name for p in tmp_path.iterdir()) == sorted(
        ['c2.csv', 'c2.csv.meta.yaml', 'x.yaml', 'x.yaml.meta.yaml'])


def test_the_record_says_which_offsets_were_used_and_where_they_came_from(tmp_path):
    csv_path = make_dataset(tmp_path, recorded='legacy')
    out = tmp_path / 'x.yaml'
    assert run([csv_path, '--output', out, '--urdf', URDF, '--write',
                '--joint-offset', f'elbow_flex={DELTA_DEG}']) == 0
    record = yaml.safe_load(ex.sidecar_path_for(out).read_text())['joint_offsets']
    assert record['applied_deg'] == {'elbow_flex': DELTA_DEG}
    assert record['source'] == 'command_line' and record['fk_tag_source'] == 'recomputed'
    assert record['meta_zero_offset_deg'] == 'absent'
    assert record['meta_follower_calibration_sha256'] == 'absent'
    assert record['urdf_sha256'] == ex.sha256_file(URDF)
    assert record['tag_offset_gripper_frame_m'] == list(OFFSET)
    assert 0.0 <= record['baseline_check_max_um'] < 5.0
    assert 'q_true = q_meas + delta' in record['sign']


def test_the_record_of_a_run_without_offsets_keeps_what_the_meta_file_said(tmp_path):
    csv_path = make_dataset(tmp_path, recorded=corrected_record(), delta_deg=0.0)
    out = tmp_path / 'x.yaml'
    assert run([csv_path, '--output', out, '--write']) == 0
    record = yaml.safe_load(ex.sidecar_path_for(out).read_text())
    assert record['joint_offsets']['source'] == 'none'
    assert record['joint_offsets']['applied_deg'] == {}
    assert record['joint_offsets']['fk_tag_source'] == 'csv'
    assert record['joint_offsets']['meta_zero_offset_deg'] == corrected_record()
    assert record['joint_offsets']['meta_follower_calibration_sha256'] == 'f' * 64
    assert record['identity']['zero_offset_deg'] == corrected_record()


def test_urdf_without_joint_offset_is_refused(tmp_path, capsys):
    csv_path = make_dataset(tmp_path)
    assert run([csv_path, '--output', tmp_path / 'x.yaml', '--urdf', URDF]) == 2
    assert '--urdf' in capsys.readouterr().out


def test_a_wrong_urdf_is_refused_by_the_tool(tmp_path, capsys):
    csv_path = make_dataset(tmp_path)
    other = tmp_path / 'other.urdf'
    other.write_text('<robot name="x"/>')
    assert run([csv_path, '--output', tmp_path / 'x.yaml', '--urdf', other,
                '--joint-offset', 'elbow_flex=0']) == 2
    assert 'urdf_sha256' in capsys.readouterr().out


def test_a_bad_offset_is_refused_before_anything_is_read(tmp_path, capsys):
    assert run([tmp_path / 'missing.csv', '--joint-offset', 'gripper=1']) == 2
    assert 'gripper' in capsys.readouterr().out


def test_the_default_urdf_is_the_install_copy_of_the_description_package(tmp_path):
    assert tool.default_urdf_path() == URDF
    csv_path = make_dataset(tmp_path)
    assert run([csv_path, '--output', tmp_path / 'x.yaml',
                '--joint-offset', f'elbow_flex={DELTA_DEG}']) == 0


def test_a_row_with_an_angle_that_is_not_a_number_is_not_used(tmp_path, capsys):
    rows, meta = rows_and_meta(make_dataset(tmp_path))
    rows[2]['q_elbow_flex'] = 'nan'
    new_rows, _ = recompute(rows, meta, {'elbow_flex': 0})
    points, rejected = ex.select_rows(new_rows)
    assert len(points) == len(rows) - 1 and rejected[0][0] == 'P03'
