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

"""record.py and tag_mount.py can send and open nothing: fixed by their source (AST, no ROS)."""

import ast
from pathlib import Path
import re

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / 'so_arm101_camera'
SOURCES = {'record.py': PACKAGE / 'record.py', 'tag_mount.py': PACKAGE / 'tag_mount.py'}

ALLOWED_IMPORTS = {
    'record.py': {'csv', 'dataclasses', 'fcntl', 'math', 'os', 'pathlib', 're', 'numpy',
                  'so_arm101_ik', 'yaml'},
    'tag_mount.py': {'dataclasses', 'hashlib', 'math', 'pathlib', 'numpy'},
}

# Names that would send something, open a port, start a process or touch the camera.
FORBIDDEN_NAMES = {
    'rclpy', 'create_publisher', 'create_client', 'create_service', 'ActionClient', 'publish',
    'call_async', 'Serial', 'serial', 'pyrealsense2', 'subprocess', 'Popen', 'popen', 'system',
    'socket', 'urlopen', 'requests', 'write_text', 'write_bytes', 'truncate', 'unlink', 'remove',
    'rename', 'replace', 'rmtree', 'chmod', 'ctypes', 'importlib', 'eval', 'exec',
}

# Values that come from so_arm101_ik and must not be written again in this package.
IK_VALUES = ('STOP_TOL', 'STOP_INTERVAL', 'SETTLE_MIN', 'SETTLE_TIMEOUT', 'MAX_AGE')


def _tree(name):
    return ast.parse(SOURCES[name].read_text(encoding='utf-8'))


@pytest.mark.parametrize('name', sorted(SOURCES))
def test_only_allowed_modules_are_imported(name):
    imported = set()
    for node in ast.walk(_tree(name)):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split('.')[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            imported.add(node.module.split('.')[0])
    assert imported <= ALLOWED_IMPORTS[name], imported - ALLOWED_IMPORTS[name]


@pytest.mark.parametrize('name', sorted(SOURCES))
def test_nothing_that_sends_opens_a_port_or_removes_a_file_is_named(name):
    found = set()
    for node in ast.walk(_tree(name)):
        if isinstance(node, ast.Name) and node.id in FORBIDDEN_NAMES:
            found.add(node.id)
        elif isinstance(node, ast.Attribute) and node.attr in FORBIDDEN_NAMES:
            found.add(node.attr)
    assert not found, found


@pytest.mark.parametrize('name', sorted(SOURCES))
def test_files_are_opened_only_for_reading_creating_or_appending(name):
    modes = []
    for node in ast.walk(_tree(name)):
        is_call = isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        if is_call and node.func.id == 'open':
            mode = node.args[1] if len(node.args) > 1 else None
            for keyword in node.keywords:
                if keyword.arg == 'mode':
                    mode = keyword.value
            modes.append('r' if mode is None else getattr(mode, 'value', '?'))
    assert set(modes) <= {'r', 'x', 'a+'}, modes
    if name == 'record.py':
        assert {'r', 'x', 'a+'} <= set(modes)


def test_the_values_of_so_arm101_ik_are_imported_not_written_again():
    tree = _tree('record.py')
    assigned = set()
    for node in ast.walk(tree):
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, (ast.AnnAssign, ast.AugAssign)):
            targets = [node.target]
        for target in targets:
            for name in ast.walk(target):
                if isinstance(name, ast.Name):
                    assigned.add(name.id)
    assert not assigned & set(IK_VALUES), assigned & set(IK_VALUES)
    used = {node.attr for node in ast.walk(tree)
            if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
            and node.value.id == 'sr'}
    assert set(IK_VALUES) - {'MAX_AGE'} <= used and {'is_stopped', 'is_fresh'} <= used


def test_every_guessed_value_of_record_py_says_so():
    lines = SOURCES['record.py'].read_text(encoding='utf-8').splitlines()
    names = ('FRAMES_WANTED', 'MATCH_MIN', 'COLLECT_TIMEOUT', 'POLL_SECONDS',
             'OUTLIER_MAD_FACTOR', 'OUTLIER_FLOOR_M', 'STABLE_TOL_M')
    for name in names:
        line = [line for line in lines if re.match(rf'{name} = ', line)]
        assert len(line) == 1 and '推測' in line[0], name
