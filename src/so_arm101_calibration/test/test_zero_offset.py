# Copyright 2026 so_arm101 authors. MIT License.
"""Tests of zero_offset.py: the model, the fit, the checks and the lines to paste (no ROS)."""

import functools
import math
from pathlib import Path
import struct

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_calibration import zero_offset as zo
from so_arm101_ik.urdf_kinematics import UrdfChain

SHARE = Path(get_package_share_directory('so_arm101_description'))
URDF_XML = (SHARE / 'urdf' / 'so101_new_calib.urdf').read_text(encoding='utf-8')
CHAIN = UrdfChain(URDF_XML)
JAW = zo.FixedJaw.from_urdf(URDF_XML, SHARE / 'meshes')
H_TRUE_M = -0.0024
NO_OLD = {}

# (lift, elbow, wrist) in rad of the poses of the plan (from the study; the lift is solved again).
SET_A = {'P1': (-1.203758, 1.381591, 1.392975), 'P2': (0.862913, -0.793725, 1.501615),
         'P3': (-1.332008, 1.470077, 1.083677), 'P4': (-0.331347, 1.205326, -0.001307)}
SET_B = {'Q1': (-0.872809, 1.259139, 1.184476), 'Q2': (0.602249, -0.346325, 1.314878),
         'Q3': (-0.536362, 1.125697, 0.632405), 'Q4': (-0.554848, 1.373897, 0.053624)}
SET_TWO_TILTS = {'P1': SET_A['P1'], 'P2': SET_A['P2'], 'P3': SET_A['P3'],
                 'R4': (-0.536362, 1.125697, 0.632405)}  # 90, 90, 70, 70 deg


def touching_q(lift, elbow, wrist, h_m=H_TRUE_M):
    """Return the true angles whose lowest vertex of the fixed jaw is at h (the lift is solved)."""
    for _ in range(30):
        q = np.array([0.0, lift, elbow, wrist, 0.0])
        z = JAW.lowest_z(CHAIN, q)
        up = JAW.lowest_z(CHAIN, q + np.array([0.0, 1e-5, 0.0, 0.0, 0.0]))
        slope = (up - z) / 1e-5
        lift -= (z - h_m) / slope
    return np.array([0.0, lift, elbow, wrist, 0.0])


@functools.lru_cache(maxsize=None)
def true_poses(name):
    table = {'A': SET_A, 'B': SET_B, 'T': SET_TWO_TILTS}[name]
    return {pose: touching_q(*v) for pose, v in table.items()}


def make_touches(pose_set, delta_true_deg, delta_old_deg=(0.0, 0.0, 0.0), repeats=2,
                 poses=None):
    """Return the records: state = true - delta_true + delta_old (true = state - old + delta)."""
    true = true_poses(pose_set)
    touches = []
    for pose in (poses or list(true)):
        for number in range(1, repeats + 1):
            q = true[pose].copy()
            for k, index in enumerate(zo.FIT_INDEX):
                q[index] += math.radians(delta_old_deg[k] - delta_true_deg[k])
            touches.append(zo.Touch(f'{pose}#{number}', tuple(q)))
    return touches


def old_dict(delta_old_deg):
    return dict(zip(zo.FIT_JOINTS, delta_old_deg))


@functools.lru_cache(maxsize=None)
def evaluated(pose_set, delta_true, old=(0.0, 0.0, 0.0), repeats=2, drop=()):
    touches = make_touches(pose_set, delta_true, old, repeats)
    touches = [t for t in touches if t.label not in drop]
    return zo.evaluate(CHAIN, JAW, touches, old_dict(old))


@functools.lru_cache(maxsize=None)
def fitted(pose_set, delta_true, old=(0.0, 0.0, 0.0), fixed_wrist=False):
    touches = make_touches(pose_set, delta_true, old)
    return zo.fit(CHAIN, JAW, touches, old_dict(old), fixed_wrist=fixed_wrist)


# ----------------------------------------------------------------------------- the mesh

def write_stl(path, triangles):
    data = bytearray(80) + struct.pack('<I', len(triangles))
    for tri in triangles:
        data += struct.pack('<12fH', 0, 0, 0, *np.array(tri, float).reshape(-1), 0)
    Path(path).write_bytes(bytes(data))


def test_the_stl_vertices_are_read_once_each(tmp_path):
    path = tmp_path / 'a.stl'
    write_stl(path, [((0, 0, 0), (1, 0, 0), (0, 1, 0)), ((1, 0, 0), (0, 1, 0), (1, 1, 2))])
    vertices = zo.read_stl_vertices(path)
    assert sorted(map(tuple, vertices)) == sorted([(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 2)])


def test_a_text_file_is_not_a_binary_stl(tmp_path):
    path = tmp_path / 'a.stl'
    path.write_text('solid x\nendsolid x\n')
    with pytest.raises(ValueError, match='not a binary STL'):
        zo.read_stl_vertices(path)


CLOUD = [(0.0, 0.0, -0.03), (0.03, 0.0, 0.0), (-0.03, 0.0, 0.0), (0.0, 0.03, 0.0),
         (0.0, -0.03, 0.0), (0.0, 0.0, 0.05), (0.02, 0.02, 0.04)]


@pytest.mark.parametrize('pose', ['P1', 'P4'])
def test_the_lowest_z_is_the_minimum_over_all_the_vertices(pose):
    cloud = CLOUD
    q = true_poses('A')[pose]
    position, rotation = CHAIN.frames(q)['wrist_roll']
    heights = [(rotation @ np.array(v) + position)[2] for v in cloud]
    assert zo.FixedJaw(cloud).lowest_z(CHAIN, q) == pytest.approx(min(heights), abs=1e-12)


def test_the_lowest_z_is_not_that_of_the_first_vertex_or_of_one_fixed_vertex():
    q = true_poses('A')['P4']
    position, rotation = CHAIN.frames(q)['wrist_roll']
    heights = [(rotation @ np.array(v) + position)[2] for v in CLOUD]
    assert max(heights) - min(heights) > 0.01
    for first in (int(np.argmax(heights)), int(np.argmin(heights))):
        cloud = [CLOUD[first]] + [v for i, v in enumerate(CLOUD) if i != first]
        assert zo.FixedJaw(cloud).lowest_z(CHAIN, q) == pytest.approx(min(heights), abs=1e-12)


def test_the_fixed_jaw_is_the_two_meshes_of_gripper_link():
    names = {name for name, _, _ in zo.link_visuals(URDF_XML)}
    assert names == {'wrist_roll_follower_so101_v1.stl', 'sts3215_03a_v1.stl'}
    assert len(JAW.vertices) > 100


@pytest.mark.parametrize('pose, eps, x', [('P1', 90.0, 100.0), ('P2', 90.0, 300.0),
                                          ('P3', 70.0, 150.0), ('P4', 50.0, 250.0)])
def test_the_elevation_and_x_of_the_plan_poses(pose, eps, x):
    got_x, got_eps = zo.tool_state(CHAIN, true_poses('A')[pose])
    assert got_eps == pytest.approx(eps, abs=3.0)  # a vertical pose touches with the fixed jaw
    assert got_x == pytest.approx(x, abs=10.0)


# ----------------------------------------------------------------------------- the fit

@pytest.mark.parametrize('pose_set, delta', [
    ('A', (0.0, -6.3, 0.0)), ('A', (1.0, -6.3, -3.0)), ('A', (-2.0, 4.0, 2.0)),
    ('B', (1.0, -6.3, -3.0))])
def test_the_deltas_come_back_from_synthetic_touches(pose_set, delta):
    solution = fitted(pose_set, delta)
    assert solution.delta_deg == pytest.approx(delta, abs=0.02)
    assert solution.h_mm < 0
    assert solution.h_mm == pytest.approx(-2.4, abs=0.05)
    assert solution.rms_mm < 0.05


def test_the_sign_of_delta_is_true_minus_measured():
    delta = (0.0, -6.3, 0.0)
    touches = make_touches('A', delta)
    # the measured elbow is the true one minus delta: larger than the true one
    true = true_poses('A')['P1']
    assert touches[0].q[2] - true[2] == pytest.approx(math.radians(6.3))
    assert fitted('A', delta).delta_deg[1] < -6.0


@pytest.mark.parametrize('old', [(0.0, -6.3, 0.0), (0.5, -3.0, 1.0)])
def test_the_delta_is_absolute_when_the_driver_already_applied_one(old):
    delta = (1.0, -6.3, -3.0)
    solution = fitted('A', delta, old)
    assert solution.delta_deg == pytest.approx(delta, abs=0.02)
    assert solution.delta_deg == pytest.approx(fitted('A', delta).delta_deg, abs=0.02)


def test_a_vertical_touch_has_a_larger_sigma():
    options = zo.Options()
    sigmas = {t.label: zo.touch_sigma_mm(CHAIN, t, options) for t in make_touches('A', (0, 0, 0))}
    assert sigmas['P1#1'] == pytest.approx(math.sqrt(2.0))
    assert sigmas['P2#2'] == pytest.approx(math.sqrt(2.0))
    assert sigmas['P3#1'] == pytest.approx(1.0)
    assert sigmas['P4#2'] == pytest.approx(1.0)


def test_the_sigma_of_the_vertical_touches_changes_the_sd():
    touches = make_touches('A', (0.0, -6.3, -3.0))
    plain = zo.fit(CHAIN, JAW, touches, NO_OLD, zo.Options(sigma_90_extra_mm=0.0))
    extra = zo.fit(CHAIN, JAW, touches, NO_OLD)
    assert np.all(extra.sd_model_deg > plain.sd_model_deg)


def test_the_sd_is_the_covariance_of_the_weighted_jacobian():
    touches = make_touches('A', (0.0, -6.3, -3.0))
    solution = fitted('A', (0.0, -6.3, -3.0))
    delta = np.radians(solution.delta_deg)
    rows = []
    for touch in touches:
        raw0 = zo.raw0_of(touch, NO_OLD)
        row = []
        for k in range(3):
            step = np.zeros(3)
            step[k] = 1e-5
            row.append((zo.predicted_z(CHAIN, JAW, raw0, delta + step)
                        - zo.predicted_z(CHAIN, JAW, raw0, delta - step)) / 2e-5)
        rows.append(row + [-1.0])
    sigma = np.array([zo.touch_sigma_mm(CHAIN, t, zo.Options()) for t in touches]) / 1000.0
    jac = np.array(rows) / sigma[:, None]
    jac = np.vstack([jac, [0.0, 0.0, 0.0, 1.0 / 0.001]])
    cov = np.linalg.inv(jac.T @ jac)
    expected = np.degrees(np.sqrt(np.diag(cov)[:3]))
    assert solution.sd_model_deg == pytest.approx(expected, rel=0.02)
    assert solution.dof == 8 + 1 - 4


def test_leaving_one_record_out_is_the_refit_without_it():
    touches = make_touches('A', (0.0, -6.3, -3.0))
    base = fitted('A', (0.0, -6.3, -3.0))
    changes = zo.leave_one_out(CHAIN, JAW, touches, NO_OLD, base)
    assert set(changes) == {t.label for t in touches}
    other = zo.fit(CHAIN, JAW, touches, NO_OLD, exclude=('P3#2',))
    assert changes['P3#2'] == pytest.approx(other.delta_deg - base.delta_deg)
    assert other.labels == tuple(t.label for t in touches if t.label != 'P3#2')


def test_a_bad_record_changes_the_delta_when_it_is_left_out():
    touches = make_touches('A', (0.0, -6.3, -3.0))
    q = list(touches[2].q)
    q[2] += math.radians(2.5)
    bad = touches[:2] + [zo.Touch(touches[2].label, tuple(q))] + touches[3:]
    base = zo.fit(CHAIN, JAW, bad, NO_OLD)
    changes = zo.leave_one_out(CHAIN, JAW, bad, NO_OLD, base)
    assert max(np.max(np.abs(c)) for c in changes.values()) > 0.3
    assert base.max_mm > 3.0


def test_the_wrist_fixed_refit_has_a_zero_wrist_and_shifts_the_others():
    delta = (0.0, -6.3, -3.0)
    fixed = fitted('A', delta, fixed_wrist=True)
    assert fixed.delta_deg[2] == 0.0
    assert fixed.free == (True, True, False)
    assert not np.allclose(fixed.delta_deg[:2], [0.0, -6.3], atol=0.05)
    assert fixed.rms_mm > 0.1


def test_the_wrist_fixed_refit_has_an_sd_for_lift_and_elbow():
    fixed = fitted('A', (0.0, -6.3, 0.0), fixed_wrist=True)
    assert fixed.sd_deg[0] > 0 and fixed.sd_deg[1] > 0
    assert fixed.sd_deg[2] == 0.0
    assert fixed.dof == 8 + 1 - 3


def test_the_height_above_the_desk_follows_the_model():
    q = true_poses('A')['P3']
    assert zo.height_above_desk_mm(CHAIN, JAW, q) == pytest.approx(0.0, abs=0.01)
    lifted = q.copy()
    lifted[2] += 0.1
    assert abs(zo.height_above_desk_mm(CHAIN, JAW, lifted)) > 5.0


# ----------------------------------------------------------------------------- the lines to paste

def test_the_value_to_paste_is_rounded_to_a_tenth_and_has_no_negative_zero():
    assert zo.paste_line('elbow_flex', -6.31).startswith('elbow_flex: zero_offset_deg: -6.3   ')
    assert zo.paste_line('elbow_flex', -6.349).startswith('elbow_flex: zero_offset_deg: -6.3')
    assert zo.paste_line('shoulder_lift', -0.04).startswith('shoulder_lift: zero_offset_deg: 0.0')
    assert '-0.0' not in zo.paste_line('shoulder_lift', -0.04)


def test_the_line_shows_the_yaml_value_and_the_difference():
    line = zo.paste_line('elbow_flex', -6.31, -3.0)
    assert line == 'elbow_flex: zero_offset_deg: -6.3   (yaml now -3.0, difference -3.3)'


# ----------------------------------------------------------------------------- the checks

def names(report, failing=True):
    return {c.name for c in report.checks if (c.failed if failing else c.warned)}


def test_good_touches_of_the_plan_are_accepted():
    report = evaluated('A', (0.5, 0.0, -2.0))
    assert report.accepted and report.exit_code == 0
    assert names(report) == set()
    assert report.used_mode == 'free'
    assert set(report.paste) == set(zo.FIT_JOINTS)
    assert report.paste['wrist_flex'].startswith('wrist_flex: zero_offset_deg: -2.0')


def test_wrist_stays_zero_below_1_5_deg_and_the_others_come_from_the_fixed_fit():
    report = evaluated('A', (0.0, -3.0, 0.0))
    assert report.accepted
    assert report.used_mode == 'wrist_fixed'
    assert 'wrist_flex' not in report.paste
    assert 'wrist は 0 のまま' in report.notes['wrist_flex']
    fixed = fitted('A', (0.0, -3.0, 0.0), fixed_wrist=True)
    assert report.paste['elbow_flex'] == zo.paste_line('elbow_flex', fixed.delta_deg[1])


def test_the_pasted_lift_and_elbow_are_the_wrist_fixed_values_not_the_free_ones():
    report = evaluated('A', (0.0, -1.0, -1.0))
    assert report.used_mode == 'wrist_fixed'
    assert report.accepted, '\n'.join(zo.report_lines(report))
    fixed = fitted('A', (0.0, -1.0, -1.0), fixed_wrist=True)
    free = fitted('A', (0.0, -1.0, -1.0))
    assert report.paste['elbow_flex'] == zo.paste_line('elbow_flex', fixed.delta_deg[1])
    assert abs(fixed.delta_deg[1] - free.delta_deg[1]) > 0.05
    assert report.used.free == (True, True, False)


@pytest.mark.parametrize('delta, sd, expected', [
    (-3.0, 0.69, True), (-1.4, 0.5, False), (-1.5, 0.5, True),
    (-3.0, 1.0, True), (-3.0, 1.01, False)])
def test_the_wrist_line_is_pasted_from_1_5_deg_and_an_sd_of_1_0_deg(delta, sd, expected):
    assert zo.wrist_rule(delta, sd) is expected


def check(report, name):
    return [c for c in report.checks if c.name == name][0]


def test_three_poses_are_refused():
    report = evaluated('A', (0.0, -3.0, 0.0), drop=('P4#1', 'P4#2'))
    assert not report.accepted and report.exit_code == 2
    assert check(report, 'poses').failed
    assert check(report, 'records').failed  # 6 records are too few as well


def test_four_poses_with_one_touch_each_are_refused():
    report = evaluated('A', (0.0, -3.0, 0.0), repeats=1)
    assert check(report, 'poses').ok
    assert check(report, 'records').failed
    assert not report.accepted


def test_two_tilt_groups_are_refused():
    report = evaluated('T', (0.0, -3.0, 0.0))
    assert check(report, 'tilt_groups').failed
    assert not report.accepted


@pytest.mark.parametrize('eps, groups', [([90, 90, 70, 50], 3), ([75, 80, 90, 90], 2),
                                         ([50, 59.9, 69.8, 79.7], 1)])
def test_the_tilt_groups_split_at_gaps_of_10_deg(eps, groups):
    assert zo.tilt_groups(eps) == groups


@pytest.mark.parametrize('spread, ok', [(149.0, False), (150.0, True)])
def test_the_x_spread_must_be_150_mm(spread, ok):
    poses = [('a', 90.0, 0.0, 2), ('b', 70.0, spread, 2), ('c', 50.0, 10.0, 2),
             ('d', 90.0, 20.0, 2)]
    checks = zo.check_poses(poses, [0.0, spread, 10.0, 20.0], [90.0, 70.0, 50.0, 90.0], 8)
    assert [c for c in checks if c.name == 'x_spread'][0].ok is ok


@pytest.mark.parametrize('eps, ok', [(45.0, True), (44.9, False)])
def test_no_elevation_below_45_deg(eps, ok):
    poses = [('a', 90.0, 0.0, 2), ('b', 70.0, 200.0, 2), ('c', eps, 10.0, 2), ('d', 90.0, 20.0, 2)]
    checks = zo.check_poses(poses, [0.0, 200.0, 10.0, 20.0], [90.0, 70.0, eps, 90.0], 8)
    assert [c for c in checks if c.name == 'eps_min'][0].ok is ok


@pytest.mark.parametrize('degrees, failing', [
    (2.5, {'residual_max', 'leave_one_out_shoulder_lift'}),
    (6.0, {'residual_rms', 'residual_max'})])
def test_a_bad_record_fails_the_residual_checks_and_the_leave_one_out_check(degrees, failing):
    touches = make_touches('A', (0.5, 0.0, -2.0))
    q = list(touches[2].q)
    q[2] += math.radians(degrees)
    bad = touches[:2] + [zo.Touch(touches[2].label, tuple(q))] + touches[3:]
    report = zo.evaluate(CHAIN, JAW, bad, NO_OLD)
    assert not report.accepted and report.exit_code == 2
    assert failing <= names(report)
    assert report.paste == {}
    assert all('not written' in note for note in report.notes.values())


def test_a_delta_beyond_10_deg_is_refused():
    report = evaluated('A', (0.0, 12.0, 0.0))
    assert check(report, 'delta_abs_elbow_flex').failed
    assert not report.accepted


@pytest.mark.parametrize('h, severity_ok, failed', [(-2.4 + 3.0, True, False),
                                                    (-2.4 + 3.1, False, False),
                                                    (-2.4 + 5.1, False, True)])
def test_the_estimated_h_warns_beyond_3_mm_and_fails_beyond_5_mm(h, severity_ok, failed):
    result = zo.check_h(h, -2.4)
    assert result.ok is severity_ok
    assert result.failed is failed
    assert result.warned is (not severity_ok and not failed)


def test_a_large_difference_of_the_two_touches_of_a_pose_is_only_a_warning():
    touches = make_touches('A', (0.0, -6.3, -3.0))
    q = list(touches[2].q)
    q[2] += math.radians(0.4)  # P2#1: about 2 mm higher than P2#2
    nudged = touches[:2] + [zo.Touch(touches[2].label, tuple(q))] + touches[3:]
    solution = zo.fit(CHAIN, JAW, nudged, NO_OLD)
    pairs = zo.pair_differences(nudged, solution)
    assert set(pairs) == {'P1', 'P2', 'P3', 'P4'}
    assert pairs['P2'] > pairs['P1']


def test_every_check_is_shown_also_for_a_rejection_and_the_result_line_says_so():
    report = evaluated('A', (0.0, 12.0, 0.0))
    text = '\n'.join(zo.report_lines(report))
    for name in ('poses', 'records', 'tilt_groups', 'x_spread', 'eps_min', 'condition',
                 'residual_rms', 'residual_max', 'pair_difference', 'h_estimate',
                 'sd_shoulder_lift',
                 'leave_one_out_elbow_flex', 'delta_abs_elbow_flex'):
        assert f'] {name}:' in text, name
    assert zo.BANNER_PREFIX + ' result: rejected' in text


def test_warnings_are_counted_and_the_result_is_still_accepted():
    report = evaluated('A', (0.5, 0.0, -2.0))
    changed = zo.Report(**{**report.__dict__, 'checks': report.checks + [
        zo.Check('pair_difference', False, 'warn', 'x')]})
    changed.warnings = 1
    assert changed.accepted
    assert zo.BANNER_PREFIX + ' result: accepted (with 1 warning)' in zo.report_lines(changed)
    assert zo.BANNER_PREFIX + ' result: accepted' in zo.report_lines(report)
