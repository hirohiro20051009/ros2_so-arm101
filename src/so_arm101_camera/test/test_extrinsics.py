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

"""Tests of extrinsics.py (Kabsch, the conversion to camera_link, the gates, the selection)."""

import csv
import math

import numpy as np
import pytest
from so_arm101_camera import camera_config as cc
from so_arm101_camera import extrinsics as ex
from so_arm101_camera import record as rec
import yaml

# The recorded pose of the optical frame in camera_link (c2_points.csv.meta.yaml, real values).
OPTICAL_T = (-4.823565177503042e-05, 0.014808941632509232, 4.587378862197511e-05)
OPTICAL_Q = (-0.49982130563682625, 0.49899555591371714, -0.5004952769026707,
             0.5006861046683616)
OPTICAL = ex.Optical(OPTICAL_T, OPTICAL_Q)


def rotation(axis, degrees):
    """Return the rotation matrix of an angle about an axis (Rodrigues)."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    r = math.radians(degrees)
    return np.eye(3) + math.sin(r) * k + (1 - math.cos(r)) * k @ k


TRUTHS = (
    (rotation([0.3, -0.5, 0.8], 140.0), np.array([0.5, 0.3, 0.6])),
    (rotation([1.0, 0.2, -0.4], 100.0), np.array([-0.2, 0.45, 0.7])),
)

# 12 tag centres like the points of C2 (2 x 3 x 2) and a 13th that is 7 mm from the first.
BOX = np.array([[x, y, z] for z in (0.19, 0.12) for y in (-0.02, 0.04, 0.10)
                for x in (0.215, 0.285)] + [[0.215, -0.02, 0.1969]])


def world(truth, noise=0.0, seed=1, box=BOX):
    """Return (camera points in the optical frame, base_link points) of a known camera_link."""
    r_bl, t_bl = truth
    r_lo = ex.quaternion_to_matrix(OPTICAL_Q)
    in_link = (box - t_bl) @ r_bl
    camera = (in_link - np.asarray(OPTICAL_T)) @ r_lo
    if noise:
        camera = camera + np.random.default_rng(seed).normal(0.0, noise, camera.shape)
    return camera, box


def points_of(camera, base, skip=()):
    return [ex.Point(f'P{i + 1:02d}', tuple(c), tuple(b))
            for i, (c, b) in enumerate(zip(camera, base)) if i not in skip]


def row_of(label, camera, base, **overrides):
    """Return a row dict of the CSV: all columns, the usual good values and the overrides."""
    row = {name: '0' for name in rec.COLUMNS}
    row.update(label=label, tag_seen='1', still='1', stable='1', cam_n='15',
               incidence_deg='20.0', cam_std_x='0.0002', cam_std_y='0.0002', cam_std_z='0.0002')
    for axis, c, b in zip('xyz', camera, base):
        row[f'cam_tag_{axis}'] = repr(float(c))
        row[f'fk_tag_{axis}'] = repr(float(b))
    row.update({k: str(v) for k, v in overrides.items()})
    return row


def write_csv(path, rows, columns=rec.COLUMNS):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(columns)
        for i, row in enumerate(rows):
            writer.writerow([row[c] if c != 'idx' else str(i + 1) for c in columns])


def meta_dict(link=True, zero_offsets=True):
    """Return a meta file; zero_offsets=False makes an older one (no zero offsets recorded)."""
    meta = {'schema': 1, 'identity': {'tag_mount_sha256': 'a' * 64, 'tag_id': 0},
            'tag_mount': {'tag_size_m': 0.06}, 'extra': {}}
    if zero_offsets:
        meta['identity']['follower_calibration_sha256'] = 'f' * 64
        meta['identity']['zero_offset_deg'] = {n: 0.0 for n in ex.ANGLE_JOINTS}
    if link:
        meta['extra']['camera_link_to_optical_frame'] = {
            'parent': 'camera_link', 'child': 'camera_color_optical_frame',
            'translation_m': list(OPTICAL_T), 'quaternion_xyzw': list(OPTICAL_Q)}
    return meta


def write_meta(path, meta):
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f, sort_keys=False)


def dataset(tmp_path, truth=TRUTHS[0], bad_label=None, name='c2.csv', scale=1.0):
    """Write a CSV and its meta file of a known camera_link (scale: the fit finds it)."""
    camera, base = world(truth)
    camera = camera / scale
    if bad_label is not None:
        camera = camera.copy()
        camera[int(bad_label[1:]) - 1, 0] += 0.020
    rows = [row_of(f'P{i + 1:02d}', c, b) for i, (c, b) in enumerate(zip(camera, base))]
    csv_path = tmp_path / name
    write_csv(csv_path, rows)
    write_meta(tmp_path / (name + '.meta.yaml'), meta_dict())
    return csv_path


# ----------------------------------------------------------------------------- Kabsch

@pytest.mark.parametrize('index', [0, 1])
def test_kabsch_recovers_a_known_transform(index):
    r0, t0 = TRUTHS[index][0], TRUTHS[index][1]
    points = np.random.default_rng(3).normal(size=(9, 3)) * 0.05
    r, t, flipped = ex.kabsch(points, points @ r0.T + t0)
    assert np.allclose(r, r0, atol=1e-12) and np.allclose(t, t0, atol=1e-12) and not flipped


def test_kabsch_with_noise_predicts_within_two_millimetres():
    camera, base = world(TRUTHS[0], noise=0.001, seed=20261116)
    clean, _ = world(TRUTHS[0])
    r, t, _ = ex.kabsch(camera, base)
    assert rms(clean @ r.T + t - base) < 0.002


def rms(differences):
    return float(np.sqrt(np.mean(np.sum(np.asarray(differences) ** 2, axis=1))))


def test_a_mirrored_set_is_corrected_to_a_rotation_and_flagged():
    camera, base = world(TRUTHS[0])
    r, _, flipped = ex.kabsch(camera * np.array([1.0, 1.0, -1.0]), base)
    assert flipped and abs(np.linalg.det(r) - 1.0) < 1e-9


def test_points_on_a_line_are_refused():
    line = np.outer(np.arange(5.0), [0.01, 0.0, 0.0])
    with pytest.raises(ex.DegenerateError):
        ex.kabsch(line, line)


def test_residuals_and_rms_agree_with_a_hand_calculation():
    points = np.zeros((2, 3))
    targets = np.array([[0.003, 0.0, 0.0], [0.0, 0.004, 0.0]])
    res = ex.residuals(points, targets, np.eye(3), np.zeros(3))
    assert np.allclose(res, [0.003, 0.004])
    assert ex.rms(res) == pytest.approx(math.sqrt(12.5e-6))


# ----------------------------------------------------------------------------- scale

@pytest.mark.parametrize('factor', [0.98, 1.02])
def test_the_scale_is_recovered(factor):
    camera, base = world(TRUTHS[0])
    assert ex.similarity_scale(camera / factor, base) == pytest.approx(factor, abs=1e-9)


def test_the_implied_tag_size_follows_the_scale():
    camera, base = world(TRUTHS[0])
    solution = ex.solve(points_of(camera / 1.02, base), OPTICAL, 0.06)
    assert solution.tag_size == pytest.approx(0.06 * 1.02, abs=1e-9)


# ----------------------------------------------------------------------------- spread, leave-out

def test_the_spreads_of_a_box_are_its_half_sides():
    corners = np.array([[sx * 0.05, sy * 0.03, sz * 0.02] for sx in (-1, 1) for sy in (-1, 1)
                        for sz in (-1, 1)])
    assert np.allclose(ex.spread(corners), [0.05, 0.03, 0.02])


def test_the_leave_out_error_equals_a_fit_without_the_row():
    camera, base = world(TRUTHS[0], noise=0.002, seed=5)
    groups = [[i] for i in range(len(camera))]
    errors = ex.leave_out_errors(camera, base, groups)
    keep = np.arange(len(camera)) != 4
    r, t, _ = ex.kabsch(camera[keep], base[keep])
    assert errors[4] == pytest.approx(np.linalg.norm(base[4] - (r @ camera[4] + t)))


@pytest.mark.parametrize('distance,grouped', [(0.010, True), (0.0101, False)])
def test_rows_within_ten_millimetres_are_one_group(distance, grouped):
    q = np.array([[0.0, 0.0, 0.0], [distance, 0.0, 0.0], [0.1, 0.0, 0.0]])
    assert ([0, 1] in ex.groups_of(q)) is grouped


def test_a_group_is_left_out_together():
    camera, base = world(TRUTHS[0])
    camera = camera.copy()
    camera[[0, 12]] += np.array([0.02, 0.0, 0.0])  # the near-duplicate pair is off together
    singles = ex.leave_out_errors(camera, base, [[i] for i in range(len(camera))])
    grouped = ex.leave_out_errors(camera, base, [[0, 12]] + [[i] for i in range(1, 12)])
    assert grouped[0] > singles[0] and grouped[12] > singles[12]


# ----------------------------------------------------------------------------- conversion

@pytest.mark.parametrize('index', [0, 1])
def test_camera_link_is_recovered_from_the_points(index):
    camera, base = world(TRUTHS[index])
    solution = ex.solve(points_of(camera, base), OPTICAL, 0.06)
    assert np.allclose(solution.link_rotation, TRUTHS[index][0], atol=1e-9)
    assert np.allclose(solution.link_translation, TRUTHS[index][1], atol=1e-9)
    assert not solution.failures


def test_the_solution_leaves_the_near_duplicate_pair_out_together():
    camera, base = world(TRUTHS[0])
    solution = ex.solve(points_of(camera, base), OPTICAL, 0.06)
    assert (0, 12) in solution.groups and len(solution.groups) == 12


def test_the_wrong_direction_of_the_optical_rotation_is_not_what_is_used():
    camera, base = world(TRUTHS[0])
    solution = ex.solve(points_of(camera, base), OPTICAL, 0.06)
    wrong = solution.rotation @ ex.quaternion_to_matrix(OPTICAL_Q)
    assert np.linalg.norm(wrong - TRUTHS[0][0]) > 0.1


def test_the_conversion_agrees_with_a_hand_calculation():
    r_lo = rotation([0, 0, 1], 90.0)
    r_bl, t_bl = ex.camera_link_pose(np.eye(3), np.array([1.0, 2.0, 3.0]), r_lo,
                                     np.array([0.0, 0.015, 0.0]))
    assert np.allclose(r_bl, r_lo.T) and np.allclose(t_bl, [0.985, 2.0, 3.0])


def test_a_quaternion_that_is_not_unit_is_normalised():
    assert np.allclose(ex.quaternion_to_matrix((0.0, 0.0, 0.0, 2.0)), np.eye(3))


@pytest.mark.parametrize('axis,degrees', [([0.2, 0.5, -0.7], 70.0), ([1.0, 0.0, 0.0], 180.0)])
def test_a_rotation_matrix_goes_through_a_quaternion_and_back(axis, degrees):
    r = rotation(axis, degrees)
    q = ex.matrix_to_quaternion(r)
    assert q[3] >= 0.0 and np.allclose(ex.quaternion_to_matrix(q), r, atol=1e-12)


def test_the_rpy_of_a_turn_about_z():
    assert np.allclose(ex.matrix_to_rpy_deg(rotation([0, 0, 1], 90.0)), [0.0, 0.0, 90.0])


# ----------------------------------------------------------------------------- the judgement

GOOD = dict(count=13, rms_m=0.003, loo_max_m=0.005, scale=1.0, s_min_m=0.03, flipped=False)


def test_a_good_result_and_the_limits_themselves_are_accepted():
    assert ex.judge(**GOOD) == []
    assert ex.judge(count=6, rms_m=0.005, loo_max_m=0.008, scale=1.0499, s_min_m=0.025,
                    flipped=False) == []
    assert ex.scale_warning(GOOD['scale']) is None


@pytest.mark.parametrize('change', [
    dict(count=5), dict(rms_m=0.0051), dict(loo_max_m=0.0081), dict(scale=1.0501),
    dict(s_min_m=0.0249), dict(flipped=True), dict(loo_max_m=math.nan)])
def test_each_gate_fails_by_itself(change):
    assert len(ex.judge(**dict(GOOD, **change))) == 1


@pytest.mark.parametrize('scale,warned', [
    (1.0, False), (1.0099, False), (0.9901, False),
    (1.0101, True), (0.9899, True), (1.0499, True), (0.9501, True)])
def test_the_scale_warning_starts_beyond_one_percent(scale, warned):
    text = ex.scale_warning(scale)
    assert (text is not None) is warned
    if warned:
        assert f'{scale:.4f}' in text and 'warning only' in text


@pytest.mark.parametrize('scale', [1.0499, 0.9501])
def test_a_scale_up_to_five_percent_is_not_a_failure(scale):
    assert ex.judge(**dict(GOOD, scale=scale)) == []


@pytest.mark.parametrize('scale', [1.0501, 0.9499, math.nan, math.inf])
def test_a_scale_beyond_five_percent_or_not_finite_fails_without_a_warning(scale):
    failures = ex.judge(**dict(GOOD, scale=scale))
    assert len(failures) == 1 and 'scale' in failures[0]
    assert ex.scale_warning(scale) is None


@pytest.mark.parametrize('scale,warned,failed', [
    (1.125, False, False), (1.1250001, True, False), (0.875, False, False),
    (0.8749999, True, False), (1.25, True, False), (1.2500001, False, True),
    (0.75, True, False), (0.7499999, False, True)])
def test_the_limits_themselves_with_exact_constants(monkeypatch, scale, warned, failed):
    monkeypatch.setattr(ex, 'SCALE_WARN', 0.125)
    monkeypatch.setattr(ex, 'SCALE_FAIL', 0.25)
    assert (ex.scale_warning(scale) is not None) is warned
    assert bool(ex.judge(**dict(GOOD, scale=scale))) is failed


def test_a_two_percent_scale_is_accepted_with_one_warning():
    camera, base = world(TRUTHS[0])
    solution = ex.solve(points_of(camera / 1.02, base), OPTICAL, 0.06)
    assert not solution.failures and len(solution.warnings) == 1
    assert not ex.solve(points_of(camera, base), OPTICAL, 0.06).warnings


def test_a_six_percent_scale_fails_and_has_no_warning():
    camera, base = world(TRUTHS[0])
    solution = ex.solve(points_of(camera / 1.06, base), OPTICAL, 0.06)
    assert any('scale' in f for f in solution.failures) and not solution.warnings


def test_the_report_tells_a_warning_from_a_failure():
    camera, base = world(TRUTHS[0])
    for factor, last, shown in ((1.0, 'result: accepted', False),
                                (1.02, 'result: accepted (with 1 warning)', True),
                                (1.06, 'result: FAILED', False)):
        lines = ex.report_lines(ex.solve(points_of(camera / factor, base), OPTICAL, 0.06), [])
        assert lines[-1] == last
        assert any(line.startswith('WARNING: ') for line in lines) is shown


def test_the_record_has_the_thresholds_and_the_warning(tmp_path):
    csv_path = dataset(tmp_path, scale=1.02)
    rows = ex.read_csv_rows(csv_path)
    meta, optical = ex.read_meta(str(csv_path) + '.meta.yaml')
    points, rejected = ex.select_rows(rows)
    solution = ex.solve(points, optical, ex.tag_size_of(meta))
    record = yaml.safe_load(ex.sidecar_text(
        solution, rejected, csv_path, str(csv_path) + '.meta.yaml', meta, optical, 'x',
        '2026-11-17T00:00:00Z'))
    assert record['thresholds']['scale_warn'] == 0.01
    assert record['thresholds']['scale_fail'] == 0.05
    assert 'scale_tolerance' not in record['thresholds']
    assert len(record['result']['warnings']) == 1


# ----------------------------------------------------------------------------- the selection

def rows_for(**overrides_of_first):
    camera, base = world(TRUTHS[0])
    rows = [row_of(f'P{i + 1:02d}', c, b) for i, (c, b) in enumerate(zip(camera, base))]
    rows[0].update({k: str(v) for k, v in overrides_of_first.items()})
    return rows


@pytest.mark.parametrize('key,value', [
    ('tag_seen', 0), ('still', 0), ('stable', 0), ('cam_n', 9), ('incidence_deg', 60.5),
    ('cam_std_z', 0.0011), ('fk_tag_x', 'nan')])
def test_a_row_that_breaks_a_condition_is_not_used(key, value):
    points, rejected = ex.select_rows(rows_for(**{key: value}))
    assert len(points) == 12 and rejected[0][0] == 'P01'


def test_the_limits_themselves_are_kept():
    points, rejected = ex.select_rows(rows_for(cam_n=10, incidence_deg=60.0, cam_std_x=0.001))
    assert len(points) == 13 and not rejected


def test_a_label_can_be_excluded_and_an_unknown_one_is_refused():
    points, rejected = ex.select_rows(rows_for(), ('P02',))
    assert len(points) == 12 and rejected == [('P02', 'excluded by --exclude')]
    with pytest.raises(ex.InputError):
        ex.select_rows(rows_for(), ('P99',))


def test_a_label_used_twice_is_refused(tmp_path):
    rows = rows_for()
    rows[1]['label'] = 'P01'
    write_csv(tmp_path / 'a.csv', rows)
    with pytest.raises(ex.InputError):
        ex.read_csv_rows(tmp_path / 'a.csv')


def test_a_header_that_is_not_the_one_of_record_is_refused(tmp_path):
    write_csv(tmp_path / 'a.csv', rows_for(), columns=rec.COLUMNS[:-1])
    with pytest.raises(ex.InputError):
        ex.read_csv_rows(tmp_path / 'a.csv')


def test_a_meta_file_without_the_optical_frame_is_refused(tmp_path):
    write_meta(tmp_path / 'm.yaml', meta_dict(link=False))
    with pytest.raises(ex.InputError):
        ex.read_meta(tmp_path / 'm.yaml')


# ----------------------------------------------------------------------------- the texts

def test_the_extrinsics_text_has_exactly_the_two_keys_and_is_read_back(tmp_path):
    text = ex.extrinsics_text((0.1, -0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
    path = tmp_path / 'camera_extrinsics.yaml'
    path.write_text(text)
    assert set(yaml.safe_load(text)) == {'translation_m', 'quaternion_xyzw'}
    assert cc.load_extrinsics(path) == ([0.1, -0.2, 0.3], [0.0, 0.0, 0.0, 1.0])


def test_the_record_holds_the_hash_of_the_extrinsics_text(tmp_path):
    csv_path = dataset(tmp_path)
    rows = ex.read_csv_rows(csv_path)
    meta, optical = ex.read_meta(str(csv_path) + '.meta.yaml')
    points, rejected = ex.select_rows(rows)
    solution = ex.solve(points, optical, ex.tag_size_of(meta))
    text = ex.extrinsics_text(solution.link_translation, solution.quaternion)
    record = yaml.safe_load(ex.sidecar_text(
        solution, rejected, csv_path, str(csv_path) + '.meta.yaml', meta, optical, text,
        '2026-11-16T00:00:00Z'))
    assert record['extrinsics_sha256'] == ex.sha256_text(text)
    assert record['rows']['used'][0] == 'P01' and record['result']['rms_mm'] < 1e-6
