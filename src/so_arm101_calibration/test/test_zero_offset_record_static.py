# Copyright 2026 so_arm101 authors. MIT License.
"""tools/zero_offset_record_run.py only listens and writes through one function (by its source)."""

import ast
from pathlib import Path

from so_arm101_calibration import zero_offset as zo

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
             '--h-sigma-mm', '--target'}


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
