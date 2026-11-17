# Copyright 2026 so_arm101 authors. MIT License.
"""Tests of tools/zero_offset_fit_run.py: what it refuses and shows, and that it writes nothing."""

import ast
import contextlib
import importlib.util
import io
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_calibration import zero_offset as zo
from so_arm101_calibration import zero_offset_record as zr
from so_arm101_ik.urdf_kinematics import UrdfChain
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
TOOL = PACKAGE.parents[1] / 'tools' / 'zero_offset_fit_run.py'
SHARE = Path(get_package_share_directory('so_arm101_description'))
URDF = SHARE / 'urdf' / 'so101_new_calib.urdf'
URDF_XML = URDF.read_text(encoding='utf-8')
CHAIN = UrdfChain(URDF_XML)
JAW = zo.FixedJaw.from_urdf(URDF_XML, SHARE / 'meshes')
H_TRUE_M = -0.0024
# (lift, elbow, wrist) in rad of the four poses of the plan (the lift is solved again).
POSES = {'P1': (-1.203758, 1.381591, 1.392975), 'P2': (0.862913, -0.793725, 1.501615),
         'P3': (-1.332008, 1.470077, 1.083677), 'P4': (-0.331347, 1.205326, -0.001307)}

spec = importlib.util.spec_from_file_location('zero_offset_fit_run', TOOL)
fit_tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fit_tool)


def touching_q(lift, elbow, wrist):
    for _ in range(30):
        q = np.array([0.0, lift, elbow, wrist, 0.0])
        z = JAW.lowest_z(CHAIN, q)
        slope = (JAW.lowest_z(CHAIN, q + np.array([0.0, 1e-5, 0, 0, 0])) - z) / 1e-5
        lift -= (z - H_TRUE_M) / slope
    return np.array([0.0, lift, elbow, wrist, 0.0])


TRUE = {pose: touching_q(*v) for pose, v in POSES.items()}


def calibration_yaml(directory, elbow=None, name='follower_calibration.yaml'):
    joints = {n: {'id': i + 1, 'kind': 'angle', 'direction': 1, 'center': 2000.0}
              for i, n in enumerate(zr.ARM_JOINTS)}
    if elbow is not None:
        joints['elbow_flex']['zero_offset_deg'] = elbow
    joints['gripper'] = {'id': 6, 'kind': 'linear_percent', 'direction': 1}
    path = Path(directory) / name
    path.write_text(yaml.safe_dump({'schema_version': 1, 'joints': joints}))
    return path


def make_csv(directory, delta_true, bad_label=None, name='zero_offset.csv'):
    """Write the records of the four poses (twice each) for the given true deltas (deg)."""
    calibration = calibration_yaml(directory)
    identity = zr.identity_from_files(calibration, URDF, SHARE / 'meshes')
    path = Path(directory) / name
    for pose, q in TRUE.items():
        for number in (1, 2):
            label = f'{pose}#{number}'
            state = q.copy()
            for k, index in enumerate(zo.FIT_INDEX):
                state[index] -= math.radians(delta_true[k])
            if label == bad_label:
                state[2] += math.radians(6.0)
            row = zr.make_row(label, 1000.0, np.append(state, 0.0), np.zeros(6), 20, CHAIN, JAW)
            zr.write_point(path, label, identity,
                           zr.build_meta(identity, zo.Options(), '2026-11-17T00:00:00Z'), row)
    return path, calibration


def run(csv, calibration, *extra):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = fit_tool.main([str(csv), '--calibration-yaml', str(calibration), *extra])
    return code, out.getvalue(), err.getvalue()


@pytest.fixture(scope='module')
def good(tmp_path_factory):
    directory = tmp_path_factory.mktemp('good')
    csv, calibration = make_csv(directory, (0.5, 0.0, -2.0))
    before = {p.name: p.read_bytes() for p in directory.iterdir()}
    result = run(csv, calibration)
    after = {p.name: p.read_bytes() for p in directory.iterdir()}
    return csv, calibration, result, before, after


@pytest.fixture(scope='module')
def wrist_zero(tmp_path_factory):
    directory = tmp_path_factory.mktemp('wrist_zero')
    csv, calibration = make_csv(directory, (0.0, -3.0, 0.0))
    return run(csv, calibration)


# ----------------------------------------------------------------------------- what it shows

def test_the_first_line_says_that_it_is_the_zero_offset_calibration_and_display_only(good):
    first = good[2][1].splitlines()[0]
    assert first == zo.FIT_BANNER
    assert first.startswith('[zero-offset calibration] zero_offset_fit_run:')
    assert 'display only' in first


def test_good_records_are_accepted_and_the_lines_to_paste_are_shown(good):
    code, out, _ = good[2]
    assert code == 0
    assert '[zero-offset calibration] result: accepted' in out
    assert 'paste: shoulder_lift: zero_offset_deg: 0.5' in out
    assert 'paste: wrist_flex: zero_offset_deg: -2.0' in out


def test_every_check_is_shown(good):
    out = good[2][1]
    for name in ('poses', 'records', 'tilt_groups', 'x_spread', 'eps_min', 'condition',
                 'residual_rms', 'residual_max', 'h_estimate', 'leave_one_out_wrist_flex'):
        assert f'] {name}:' in out, name


def test_nothing_is_written(good):
    csv, calibration, _, before, after = good
    assert before == after
    assert set(after) == {'zero_offset.csv', 'zero_offset.csv.meta.yaml', calibration.name}


def test_wrist_stays_zero_when_it_is_small_and_the_free_values_are_shown_for_reference(wrist_zero):
    code, out, _ = wrist_zero
    assert code == 0
    assert 'wrist は 0 のまま' in out
    assert 'for reference, wrist_flex free:' in out
    assert 'paste: elbow_flex: zero_offset_deg: -3.0' in out
    assert 'paste: wrist_flex' not in out


def test_a_bad_record_is_rejected_with_every_item_and_no_line_to_paste(tmp_path):
    csv, calibration = make_csv(tmp_path, (0.5, 0.0, -2.0), bad_label='P2#1')
    code, out, _ = run(csv, calibration)
    assert code == 2
    assert 'result: rejected (nothing to paste)' in out
    assert 'paste:' not in out and '[FAIL]' in out and '] condition:' in out


def test_an_excluded_record_is_not_used(tmp_path):
    csv, calibration = make_csv(tmp_path, (0.5, 0.0, -2.0))
    code, out, _ = run(csv, calibration, '--exclude', 'P1#1')
    assert 'left out: P1#1' in out
    assert 'residual P1#1' not in out and 'residual P1#2' in out
    assert code in (0, 2)


# ----------------------------------------------------------------------------- what it refuses

def test_a_missing_csv_and_an_unknown_record_and_a_bad_argument_are_exit_1(tmp_path):
    csv, calibration = make_csv(tmp_path, (0.5, 0.0, -2.0))
    assert run(tmp_path / 'missing.csv', calibration)[0] == 1
    assert run(csv, calibration, '--exclude', 'P9#1')[0] == 1
    with pytest.raises(SystemExit) as exc:
        fit_tool.main([str(csv), '--no-such-option'])
    assert exc.value.code == 1


def test_records_without_a_meta_file_are_refused(tmp_path):
    csv, calibration = make_csv(tmp_path, (0.5, 0.0, -2.0))
    zr.meta_path_for(csv).unlink()
    code, _, err = run(csv, calibration)
    assert code == 2 and 'refused' in err


def test_a_meta_file_of_another_kind_is_refused(tmp_path):
    csv, calibration = make_csv(tmp_path, (0.5, 0.0, -2.0))
    meta_path = zr.meta_path_for(csv)
    meta = yaml.safe_load(meta_path.read_text())
    meta['calibration_kind'] = 'camera'
    meta_path.write_text(yaml.safe_dump(meta))
    code, _, err = run(csv, calibration)
    assert code == 2 and 'calibration_kind' in err


def test_records_made_with_another_calibration_yaml_are_refused(tmp_path):
    csv, calibration = make_csv(tmp_path, (0.5, 0.0, -2.0))
    other = calibration_yaml(tmp_path, elbow=-6.3, name='other.yaml')
    code, _, err = run(csv, other)
    assert code == 2
    assert 'follower_calibration_sha256' in err and 'zero_offset_deg' in err


def test_records_made_with_another_urdf_are_refused(tmp_path):
    csv, calibration = make_csv(tmp_path, (0.5, 0.0, -2.0))
    other = tmp_path / 'other.urdf'
    other.write_text(URDF_XML + '\n')
    code, _, err = run(csv, calibration, '--urdf', str(other))
    assert code == 2 and 'urdf_sha256' in err


# ----------------------------------------------------------------------------- the source (static)

SOURCE = TOOL.read_text(encoding='utf-8')
TREE = ast.parse(SOURCE)
FORBIDDEN_NAMES = {
    'write_text', 'write_bytes', 'truncate', 'unlink', 'remove', 'rename', 'replace', 'rmtree',
    'chmod', 'safe_dump', 'dump', 'write_point', 'build_meta', 'create_publisher', 'subprocess',
    'Popen', 'socket', 'Serial', 'serial', 'system', 'popen', 'kill', 'rclpy', 'copyfile'}
ARGUMENTS = {'csv', '--meta', '--exclude', '--sigma-contact', '--sigma-90-extra', '--h-prior-mm',
             '--h-sigma-mm', '--calibration-yaml', '--urdf', '--mesh-dir'}


def calls(name):
    return [n for n in ast.walk(TREE) if isinstance(n, ast.Call)
            and (getattr(n.func, 'attr', None) == name or getattr(n.func, 'id', None) == name)]


def test_there_is_no_argument_and_no_call_that_writes_a_yaml_or_a_file():
    arguments = {n.args[0].value for n in calls('add_argument')}
    assert arguments == ARGUMENTS
    assert not arguments & {'--write', '--output', '--joints', '--force'}
    names = set()
    for node in ast.walk(TREE):
        names |= {node.id} if isinstance(node, ast.Name) else set()
        names |= {node.attr} if isinstance(node, ast.Attribute) else set()
    assert names & FORBIDDEN_NAMES == set()
    assert calls('open') == []


def test_the_first_thing_main_does_is_to_print_the_banner_and_no_line_is_too_long():
    main = [n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'main'][0]
    assert ast.unparse(main.body[0].value).startswith('print(zo.FIT_BANNER')
    assert [i + 1 for i, line in enumerate(SOURCE.splitlines()) if len(line) > 99] == []
