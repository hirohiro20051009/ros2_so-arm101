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

"""tools/camera_record_run.py sends nothing and opens no file: fixed by its source (AST)."""

import ast
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ElementTree

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
TOOL = ROOT / 'tools' / 'camera_record_run.py'
SOURCES = (TOOL, PACKAGE / 'so_arm101_camera' / 'record.py',
           PACKAGE / 'so_arm101_camera' / 'tag_mount.py')

ALLOWED_IMPORTS = {
    'argparse', 'datetime', 'os', 'select', 'sys', 'time', 'ament_index_python', 'apriltag_msgs',
    'rclpy', 'sensor_msgs', 'so_arm101_camera', 'so_arm101_ik', 'tf2_msgs', 'tf2_ros'}

# Names that would send something, start a process, kill one or write a file.
FORBIDDEN_NAMES = {
    'create_publisher', 'create_client', 'create_service', 'ActionClient', 'ActionServer',
    'publish', 'call_async', 'Serial', 'serial', 'pyrealsense2', 'subprocess', 'Popen', 'socket',
    'TransformBroadcaster', 'StaticTransformBroadcaster', 'sendTransform', 'write_text',
    'write_bytes', 'truncate', 'unlink', 'remove', 'rename', 'replace', 'rmtree', 'chmod',
    'ctypes', 'importlib', 'eval', 'exec', 'fork', 'forkpty', 'killpg',
}

# The os functions that start or stop a process: os.system, os.popen, os.exec*, os.spawn*, os.kill
OS_PATTERNS = {'system': r'system', 'popen': r'popen', 'exec': r'exec.*', 'spawn': r'spawn.*',
               'kill': r'kill.*'}

IK_VALUES = ('STOP_TOL', 'STOP_INTERVAL', 'SETTLE_MIN', 'SETTLE_TIMEOUT', 'MAX_AGE')
TOPICS = {'/joint_states', '/detections', '/tf', '/camera/camera/color/camera_info'}
ARGUMENTS = {'--csv', '--label', '--investigate', '--tag-mount', '--apriltag-yaml',
             '--realsense-yaml', '--urdf', '--calibration-yaml'}
IO_METHODS = {'now', 'wait', 'read_joints', 'flush', 'poll', 'say', 'confirm', 'wall_time',
              'date_text'}
# Python module of each import and the package.xml name that provides it.
PACKAGE_XML_NAMES = {
    'numpy': 'python3-numpy', 'yaml': 'python3-yaml', 'rclpy': 'rclpy',
    'sensor_msgs': 'sensor_msgs', 'apriltag_msgs': 'apriltag_msgs', 'tf2_msgs': 'tf2_msgs',
    'tf2_ros': 'tf2_ros_py', 'so_arm101_ik': 'so_arm101_ik',
    'ament_index_python': 'ament_index_python'}


def _tree(path=TOOL):
    return ast.parse(path.read_text(encoding='utf-8'))


def _imports(path):
    found = set()
    for node in ast.walk(_tree(path)):
        if isinstance(node, ast.Import):
            found |= {alias.name.split('.')[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            found.add(node.module.split('.')[0])
    return found


def _declared():
    root = ElementTree.parse(PACKAGE / 'package.xml').getroot()
    return {e.text for e in root.findall('exec_depend')}


def _names(tree):
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            yield node.id
        elif isinstance(node, ast.Attribute):
            yield node.attr
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:  # "from os import kill" names kill without using it
                yield alias.name.split('.')[-1]


def test_only_allowed_modules_are_imported():
    assert _imports(TOOL) <= ALLOWED_IMPORTS, _imports(TOOL) - ALLOWED_IMPORTS


def test_nothing_that_sends_or_starts_a_process_is_named():
    assert not set(_names(_tree())) & FORBIDDEN_NAMES


@pytest.mark.parametrize('kind', sorted(OS_PATTERNS))
def test_no_os_function_that_starts_or_stops_a_process(kind):
    found = {name for name in _names(_tree()) if re.fullmatch(OS_PATTERNS[kind], name)}
    assert not found, found


def test_there_is_no_call_of_open_at_all():
    calls = []
    for node in ast.walk(_tree()):
        if isinstance(node, ast.Call):
            func = node.func
            if (isinstance(func, ast.Name) and func.id == 'open') or (
                    isinstance(func, ast.Attribute) and func.attr == 'open'):
                calls.append(node.lineno)
    assert not calls, calls


def test_it_subscribes_to_the_four_topics_and_nothing_else():
    tree = _tree()
    constants = {t.id: node.value.value for node in tree.body if isinstance(node, ast.Assign)
                 for t in node.targets if isinstance(t, ast.Name)
                 and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str)}
    topics = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'create_subscription'):
            topic = node.args[1]
            topics.append(topic.value if isinstance(topic, ast.Constant)
                          else constants[topic.id])
    assert sorted(topics) == sorted(TOPICS)


def test_the_values_of_so_arm101_ik_are_not_written_again():
    assigned = set()
    for node in ast.walk(_tree()):
        if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for target in targets:
                assigned |= {n.id for n in ast.walk(target) if isinstance(n, ast.Name)}
    assert not assigned & set(IK_VALUES)


def test_the_arguments_are_exactly_the_expected_ones():
    options = set()
    for node in ast.walk(_tree()):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'add_argument'):
            options |= {a.value for a in node.args
                        if isinstance(a, ast.Constant) and str(a.value).startswith('--')}
    assert options == ARGUMENTS


def test_every_check_that_needs_no_node_comes_before_rclpy_init():
    main = [n for n in _tree().body if isinstance(n, ast.FunctionDef) and n.name == 'main'][0]
    first = {}
    for node in ast.walk(main):
        if isinstance(node, ast.Call):
            func = node.func
            name = func.attr if isinstance(func, ast.Attribute) else getattr(func, 'id', '')
            first[name] = min(first.get(name, node.lineno), node.lineno)
    for name in ('load_tag_mount', 'tag_frame_name', 'load_chain', 'identity_from_files',
                 'check_session', 'parse_args'):
        assert name in first and first[name] < first['init'], name
    assert first['init'] < first['RunNode']


def test_the_node_has_every_method_of_the_io_of_the_loop():
    node = [n for n in _tree().body if isinstance(n, ast.ClassDef) and n.name == 'RunNode'][0]
    methods = {n.name for n in node.body if isinstance(n, ast.FunctionDef)}
    assert IO_METHODS <= methods


def test_no_line_is_longer_than_99_characters():
    lines = TOOL.read_text(encoding='utf-8').splitlines()
    assert [i + 1 for i, line in enumerate(lines) if len(line) > 99] == []


@pytest.mark.parametrize('module', sorted(PACKAGE_XML_NAMES))
def test_every_module_is_declared_in_package_xml(module):
    assert PACKAGE_XML_NAMES[module] in _declared()


def test_every_third_party_import_is_one_of_the_mapped_modules():
    imported = set()
    for path in SOURCES:
        imported |= _imports(path)
    third_party = {m for m in imported if m not in sys.stdlib_module_names}
    assert third_party - {'so_arm101_camera'} <= set(PACKAGE_XML_NAMES), third_party


def test_the_hardware_package_of_the_calibration_yaml_is_declared():
    assert 'so_arm101_hardware' in _declared()


def test_the_urdf_of_the_description_package_is_declared():
    assert {'so_arm101_description', 'tf2_ros_py', 'python3-numpy'} <= _declared()
