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

"""Tests of tools/camera_extrinsics_run.py: the guards of the write, and what it must not use."""

import ast
import importlib.util
from pathlib import Path

import numpy as np
import pytest
from so_arm101_camera import camera_config as cc
from so_arm101_camera import extrinsics as ex
from test_extrinsics import dataset, TRUTHS
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
TOOL = ROOT / 'tools' / 'camera_extrinsics_run.py'
MODULE = PACKAGE / 'so_arm101_camera' / 'extrinsics.py'
OLD = '# an older file\ntranslation_m: [9.0, 9.0, 9.0]\nquaternion_xyzw: [0.0, 0.0, 0.0, 1.0]\n'


def load_tool():
    spec = importlib.util.spec_from_file_location('camera_extrinsics_run', TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


tool = load_tool()


def run(args):
    return tool.main([str(a) for a in args])


def sidecar_of(path):
    return ex.sidecar_path_for(path)


# ----------------------------------------------------------------------------- the writes

def test_without_write_the_proposal_is_printed_and_nothing_is_written(tmp_path, capsys):
    out = tmp_path / 'camera_extrinsics.yaml'
    assert run([dataset(tmp_path), '--output', out]) == 0
    assert 'not written' in capsys.readouterr().out
    assert not out.exists() and not sidecar_of(out).exists()


def test_write_makes_the_file_with_two_keys_and_its_record(tmp_path):
    out = tmp_path / 'camera_extrinsics.yaml'
    assert run([dataset(tmp_path), '--output', out, '--write']) == 0
    assert set(yaml.safe_load(out.read_text())) == {'translation_m', 'quaternion_xyzw'}
    translation, quaternion = cc.load_extrinsics(out)
    assert np.allclose(translation, TRUTHS[0][1], atol=1e-9)
    assert np.allclose(ex.quaternion_to_matrix(quaternion), TRUTHS[0][0], atol=1e-9)
    record = yaml.safe_load(sidecar_of(out).read_text())
    assert record['extrinsics_sha256'] == ex.sha256_file(out)
    assert sorted(p.name for p in tmp_path.iterdir()) == sorted(
        ['c2.csv', 'c2.csv.meta.yaml', out.name, sidecar_of(out).name])


def test_an_existing_file_is_not_replaced_without_force(tmp_path, capsys):
    out = tmp_path / 'camera_extrinsics.yaml'
    out.write_text(OLD)
    assert run([dataset(tmp_path), '--output', out, '--write']) == 2
    assert out.read_text() == OLD and not sidecar_of(out).exists()
    assert 'refused' in capsys.readouterr().out


def test_force_replaces_and_shows_the_old_values(tmp_path, capsys):
    out = tmp_path / 'camera_extrinsics.yaml'
    out.write_text(OLD)
    assert run([dataset(tmp_path), '--output', out, '--write', '--force']) == 0
    assert out.read_text() != OLD and '9.0' in capsys.readouterr().out


def test_a_record_left_without_its_file_is_not_overwritten(tmp_path):
    out = tmp_path / 'camera_extrinsics.yaml'
    sidecar_of(out).write_text('old: record\n')
    assert run([dataset(tmp_path), '--output', out, '--write']) == 2
    assert sidecar_of(out).read_text() == 'old: record\n' and not out.exists()


def test_a_missing_directory_is_refused(tmp_path):
    assert run([dataset(tmp_path), '--output', tmp_path / 'none' / 'x.yaml', '--write']) == 2


@pytest.mark.parametrize('existing', [True, False])
def test_a_failed_result_writes_nothing_and_still_shows_every_number(tmp_path, capsys, existing):
    out = tmp_path / 'camera_extrinsics.yaml'
    if existing:
        out.write_text(OLD)
    csv_path = dataset(tmp_path, bad_label='P03')
    assert run([csv_path, '--output', out, '--write', '--force']) == 2
    text = capsys.readouterr().out
    for word in ('RMS', 'P03: residual', 'leave-out', 'scale', 'spreads', 'FAILED'):
        assert word in text
    assert not sidecar_of(out).exists()
    assert (out.read_text() == OLD) if existing else not out.exists()


def test_excluding_the_bad_point_gives_a_result_and_an_unknown_label_is_refused(tmp_path, capsys):
    csv_path = dataset(tmp_path, bad_label='P03')
    assert run([csv_path, '--output', tmp_path / 'x.yaml', '--exclude', 'P03']) == 0
    assert 'excluded by --exclude' in capsys.readouterr().out
    assert run([csv_path, '--output', tmp_path / 'x.yaml', '--exclude', 'P99']) == 2


def test_a_failed_replace_keeps_the_old_file_and_leaves_no_temporary_file(tmp_path, monkeypatch):
    out = tmp_path / 'camera_extrinsics.yaml'
    out.write_text(OLD)
    csv_path = dataset(tmp_path)

    def broken(source, target):
        raise OSError('disk')

    monkeypatch.setattr(ex.os, 'replace', broken)
    with pytest.raises(OSError):
        run([csv_path, '--output', out, '--write', '--force'])
    assert out.read_text() == OLD
    assert sorted(p.name for p in tmp_path.iterdir()) == ['c2.csv', 'c2.csv.meta.yaml', out.name]


def test_the_default_output_is_in_src_and_not_in_install():
    expected = ROOT / 'src' / 'so_arm101_camera' / 'config' / 'camera_extrinsics.yaml'
    assert tool.default_output_path() == expected
    assert 'install' not in tool.default_output_path().relative_to(ROOT).parts


# ----------------------------------------------------------------------------- static

FORBIDDEN_MODULES = {'rclpy', 'serial', 'subprocess', 'socket', 'ctypes', 'importlib', 'shutil',
                     'sensor_msgs', 'tf2_ros'}
FORBIDDEN_NAMES = {'publish', 'create_publisher', 'Serial', 'Popen', 'system', 'kill', 'killpg',
                   'eval', 'exec', 'rmtree', 'chmod'}


def _tree(path):
    return ast.parse(path.read_text())


@pytest.mark.parametrize('path', [TOOL, MODULE], ids=['tool', 'module'])
def test_nothing_that_sends_starts_a_process_or_uses_ros_is_imported_or_named(path):
    tree = _tree(path)
    modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules |= {a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            modules.add(node.module.split('.')[0])
    assert not modules & FORBIDDEN_MODULES
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    names |= {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    assert not names & FORBIDDEN_NAMES


def test_the_write_path_is_one_function_and_open_only_reads():
    tree = _tree(MODULE)
    writer = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                  and n.name == 'write_outputs')
    inside = {id(n) for n in ast.walk(writer)}
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and node.attr in {
                'replace', 'link', 'unlink', 'fdopen', 'mkstemp', 'write_text', 'write_bytes'}:
            if node.attr == 'replace' and not isinstance(node.value, ast.Name):
                continue  # str.replace of a text, not os.replace
            assert id(node) in inside, f'{node.attr} outside write_outputs (line {node.lineno})'
        if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'open':
            assert len(node.args) == 1 and not any(k.arg == 'mode' for k in node.keywords)
    tool_tree = _tree(TOOL)
    assert not any(isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'open'
                   for n in ast.walk(tool_tree))
    assert not any(isinstance(n, ast.Attribute) and n.attr in {
        'write_text', 'write_bytes', 'unlink', 'replace', 'link'} for n in ast.walk(tool_tree))
