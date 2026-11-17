# Copyright 2026 so_arm101 authors. MIT License.
"""tools/zero_offset_record_run.py only listens and writes through one function (by its source)."""

import ast
import importlib.util
from pathlib import Path
import re
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from so_arm101_calibration import zero_offset as zo
from so_arm101_ik.urdf_kinematics import UrdfChain

PACKAGE = Path(__file__).resolve().parents[1]
TOOL = PACKAGE.parents[1] / 'tools' / 'zero_offset_record_run.py'
SOURCE = TOOL.read_text(encoding='utf-8')
TREE = ast.parse(SOURCE)

# Names that would send something, start a process, kill one or write a file.
FORBIDDEN_NAMES = {
    'create_publisher', 'create_client', 'create_service', 'ActionClient', 'ActionServer',
    'publish', 'call_async', 'Serial', 'serial', 'subprocess', 'Popen', 'socket', 'write_text',
    'write_bytes', 'truncate', 'unlink', 'remove', 'rename', 'replace', 'rmtree', 'chmod',
    'ctypes', 'importlib', 'eval', 'exec', 'fork', 'forkpty', 'killpg', 'system', 'popen', 'kill',
    'spawn', 'TransformBroadcaster', 'sendTransform'}
ARGUMENTS = {'--csv', '--calibration-yaml', '--urdf', '--mesh-dir', '--h-prior-mm',
             '--h-sigma-mm', '--target', '--expected-offset'}


def names():
    found = set()
    for node in ast.walk(TREE):
        if isinstance(node, ast.Name):
            found.add(node.id)
        elif isinstance(node, ast.Attribute):
            found.add(node.attr)
    return found


def calls(name):
    return [n for n in ast.walk(TREE) if isinstance(n, ast.Call)
            and (getattr(n.func, 'attr', None) == name or getattr(n.func, 'id', None) == name)]


def test_it_subscribes_to_joint_states_and_nothing_else_and_sends_nothing():
    subscriptions = calls('create_subscription')
    assert len(subscriptions) == 1
    assert ast.unparse(subscriptions[0].args[1]) == 'JOINT_STATES'
    assert "JOINT_STATES = '/joint_states'" in SOURCE
    assert names() & FORBIDDEN_NAMES == set()


def test_there_is_no_call_of_open_at_all():
    assert calls('open') == []


def test_the_first_thing_main_does_is_to_print_the_banner_of_the_zero_offset_calibration():
    main = [n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'main'][0]
    first = main.body[0]
    assert isinstance(first, ast.Expr) and ast.unparse(first.value).startswith(
        'print(zo.RECORD_BANNER')
    assert zo.RECORD_BANNER.startswith('[zero-offset calibration] zero_offset_record_run:')
    assert 'NOT the center calibration' in zo.RECORD_BANNER


def test_the_arguments_are_exactly_the_expected_ones():
    arguments = {n.args[0].value for n in calls('add_argument')}
    assert arguments == ARGUMENTS


def test_every_check_that_needs_no_node_comes_before_rclpy_init():
    init = [n.lineno for n in calls('init') if ast.unparse(n.func) == 'rclpy.init']
    checks = [calls('identity_from_files')[0].lineno, calls('check_file')[0].lineno,
              calls('from_urdf')[0].lineno]
    assert len(init) == 1 and max(checks) < init[0]


def test_no_line_of_the_tool_is_longer_than_99_characters():
    assert [i + 1 for i, line in enumerate(SOURCE.splitlines()) if len(line) > 99] == []


def test_the_expected_offset_is_never_given_to_the_fit():
    for name in ('fit', 'evaluate'):
        for call in calls(name):
            assert 'expected' not in ast.unparse(call), ast.unparse(call)
    assert calls('fit') != []


# ------------------------------------------------- the display uses the expected state only

def load_tool():
    spec = importlib.util.spec_from_file_location('zero_offset_record_run', TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeNode:
    """The few lines of the node that status_line reads (the state of P3 held still)."""

    def recent(self):
        return np.tile([0.0, -1.332008, 1.470077, 1.083677, 0.0, 0.0], (10, 1))


def numbers_of(text):
    pattern = r'elevation\s+(-?[0-9.]+) deg \(\s*([+-][0-9.]+)\)\s+\[read\s+(-?[0-9.]+)'
    return [float(v) for v in re.search(pattern, text).groups()]


def test_the_display_shows_the_expected_elevation_and_the_one_that_was_read():
    tool = load_tool()
    share = Path(get_package_share_directory('so_arm101_description'))
    xml = (share / 'urdf' / 'so101_new_calib.urdf').read_text(encoding='utf-8')
    chain, jaw = UrdfChain(xml), zo.FixedJaw.from_urdf(xml, share / 'meshes')
    plain = numbers_of(tool.status_line('P3#1', FakeNode(), chain, jaw, zo.Options()))
    guided = numbers_of(tool.status_line('P3#1', FakeNode(), chain, jaw, zo.Options(),
                                         {'elbow_flex': -6.3}))
    assert plain[0] == pytest.approx(plain[2]) and plain[2] == pytest.approx(70.0, abs=3.0)
    assert guided[2] == pytest.approx(plain[2])
    assert guided[0] == pytest.approx(plain[2] - 6.3, abs=0.06)
    assert guided[1] == pytest.approx(guided[0] - 70.0, abs=0.06)


class SteadyNode(FakeNode):
    def newest_age(self):
        return 0.1


def test_the_target_check_of_a_record_uses_the_expected_state_and_writes_the_state_read(
        tmp_path, capsys):
    tool = load_tool()
    share = Path(get_package_share_directory('so_arm101_description'))
    xml = (share / 'urdf' / 'so101_new_calib.urdf').read_text(encoding='utf-8')
    chain, jaw = UrdfChain(xml), zo.FixedJaw.from_urdf(xml, share / 'meshes')
    zero = {name: 0.0 for name in tool.zr.ARM_JOINTS}
    identity = tool.zr.make_identity('a' * 64, zero, 'b' * 64, {'m.stl': 'c' * 64})
    outcomes = {}
    for name, offsets in (('plain', {}), ('guided', {'elbow_flex': -6.3})):
        args = SimpleNamespace(csv=tmp_path / f'{name}.csv', expected_offsets=offsets)
        outcomes[name] = tool.record(SteadyNode(), 'P3#1', args, chain, jaw, zo.Options(),
                                     identity, {})
    assert outcomes == {'plain': True, 'guided': False}  # read 70 deg, really 63.7 deg: 6.3 off
    assert 'outside the target' in capsys.readouterr().out
    assert not (tmp_path / 'guided.csv').exists()
