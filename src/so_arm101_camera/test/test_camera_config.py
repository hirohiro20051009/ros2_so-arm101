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

"""Tests of camera_config.py, the config files and camera.launch.py (no node is started)."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from launch import LaunchContext
from launch.actions import LogInfo, OpaqueFunction
from launch_ros.actions import Node
import pytest
from so_arm101_camera import camera_config as cc
import yaml

PKG = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PKG / 'launch' / 'camera.launch.py'
REALSENSE_YAML = PKG / 'config' / 'realsense.yaml'
APRILTAG_YAML = PKG / 'config' / 'apriltag.yaml'

GOOD = {'translation_m': [0.1, -0.2, 0.3], 'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0]}
DEFAULT = Path('/nonexistent/config/camera_extrinsics.yaml')


def _write(tmp_path, data, name='extrinsics.yaml'):
    path = tmp_path / name
    path.write_text(yaml.safe_dump(data))
    return path


def _load_launch():
    spec = importlib.util.spec_from_file_location('camera_launch_under_test', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _declared_message(argv):
    with pytest.raises(RuntimeError) as info:
        cc.check_launch_arguments(argv)
    return str(info.value)


# --- realsense.yaml ---------------------------------------------------------------------------

def test_realsense_arguments_are_exactly_the_three_old_values():
    assert cc.realsense_launch_arguments(REALSENSE_YAML) == {
        'depth_module.depth_profile': '640x480x15',
        'rgb_camera.color_profile': '640x480x15',
        'align_depth.enable': 'true',
    }


def test_realsense_keys_are_arguments_of_rs_launch():
    try:
        prefix = Path(get_package_prefix('realsense2_camera'))
    except PackageNotFoundError:
        pytest.skip('realsense2_camera is not installed')
    rs_launch = prefix / 'share' / 'realsense2_camera' / 'launch' / 'rs_launch.py'
    spec = importlib.util.spec_from_file_location('rs_launch_under_test', rs_launch)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    names = {p['name'] for p in module.configurable_parameters}
    assert set(cc.realsense_launch_arguments(REALSENSE_YAML)) <= names


def test_realsense_bool_becomes_lowercase_string(tmp_path):
    path = _write(tmp_path, {'a.b': True, 'c': False, 'd': 5})
    assert cc.realsense_launch_arguments(path) == {'a.b': 'true', 'c': 'false', 'd': '5'}


def test_realsense_rejects_nested_values_and_empty_files(tmp_path):
    with pytest.raises(ValueError):
        cc.realsense_launch_arguments(_write(tmp_path, {'a': {'b': 1}}))
    with pytest.raises(ValueError):
        cc.realsense_launch_arguments(_write(tmp_path, {}, 'empty.yaml'))


# --- undeclared launch arguments --------------------------------------------------------------

def test_no_arguments_are_accepted():
    cc.check_launch_arguments([])
    cc.check_launch_arguments(['/path/camera.launch.py'])


@pytest.mark.parametrize('name', cc.DECLARED_ARGUMENTS)
def test_every_declared_argument_is_accepted(name):
    cc.check_launch_arguments([f'{name}:=x'])


def test_ros2launch_option_arguments_are_accepted():
    cc.check_launch_arguments(['launch-prefix:=gdb', 'launch-prefix-filter:=x'])


def test_undeclared_argument_is_reported_with_the_declared_list():
    message = _declared_message(['bogus:=1'])
    assert 'bogus' in message
    assert 'extrinsics_file, use_apriltag' in message


def test_typo_gets_a_suggestion():
    assert 'did you mean: use_apriltag' in _declared_message(['use_apritag:=true'])


def test_realsense_argument_is_not_accepted_and_points_to_the_yaml():
    message = _declared_message(['depth_module.depth_profile:=1280x720x30'])
    assert 'depth_module.depth_profile' in message
    assert 'realsense.yaml' in message


# --- extrinsics -------------------------------------------------------------------------------

def test_valid_extrinsics_give_the_tf_arguments(tmp_path):
    args = cc.plan_extrinsics_tf(_write(tmp_path, GOOD), DEFAULT)
    assert args == [
        '--x', '0.1', '--y', '-0.2', '--z', '0.3',
        '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
        '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
    ]


def test_missing_extrinsics_file_at_the_default_path_means_no_tf():
    assert cc.plan_extrinsics_tf(DEFAULT, DEFAULT) is None
    assert str(DEFAULT) in cc.no_extrinsics_message(DEFAULT)


def test_missing_extrinsics_file_at_an_explicit_path_is_an_error(tmp_path):
    with pytest.raises(cc.ExtrinsicsError):
        cc.plan_extrinsics_tf(tmp_path / 'typo.yaml', DEFAULT)


def _with(**changes):
    data = dict(GOOD)
    data.update(changes)
    return data


@pytest.mark.parametrize('bad', [
    {'translation_m': [0.1, 0.2, 0.3]},                                  # key missing
    dict(GOOD, extra=1),                                                  # key not allowed
    _with(translation_m=[0.1, 0.2]),                                      # wrong length
    _with(quaternion_xyzw=[0, 0, 1]),                                     # wrong length
    _with(translation_m=[0.1, 0.2, float('nan')]),                        # nan
    _with(translation_m=[0.1, float('inf'), 0.3]),                        # inf
    _with(quaternion_xyzw=[0, 0, 0, float('-inf')]),                      # -inf
    _with(translation_m=[0.1, 'a', 0.3]),                                 # not a number
    _with(translation_m=[0.1, True, 0.3]),                                # bool is not a number
    _with(quaternion_xyzw=[0, 0, 0, 2]),                                  # norm 2
    _with(quaternion_xyzw=[0, 0, 0, 0]),                                  # norm 0
    [1, 2, 3],                                                            # not a mapping
])
def test_invalid_extrinsics_raise(tmp_path, bad):
    with pytest.raises(cc.ExtrinsicsError):
        cc.plan_extrinsics_tf(_write(tmp_path, bad), DEFAULT)


def test_quaternion_norm_tolerance(tmp_path):
    assert cc.QUATERNION_NORM_TOLERANCE == 1e-3
    inside = _with(quaternion_xyzw=[0, 0, 0, 1 + 0.5e-3])
    outside = _with(quaternion_xyzw=[0, 0, 0, 1 + 2e-3])
    assert cc.plan_extrinsics_tf(_write(tmp_path, inside), DEFAULT) is not None
    with pytest.raises(cc.ExtrinsicsError):
        cc.plan_extrinsics_tf(_write(tmp_path, outside, 'out.yaml'), DEFAULT)


def test_unparsable_yaml_raises(tmp_path):
    path = tmp_path / 'broken.yaml'
    path.write_text('translation_m: [1, 2\n')
    with pytest.raises(cc.ExtrinsicsError):
        cc.plan_extrinsics_tf(path, DEFAULT)


# --- camera.launch.py -------------------------------------------------------------------------

def _context(extrinsics_file, use_apriltag='false'):
    context = LaunchContext(argv=[])
    context.launch_configurations['use_apriltag'] = use_apriltag
    context.launch_configurations['extrinsics_file'] = str(extrinsics_file)
    return context


def _tf_nodes(actions):
    return [a for a in actions
            if isinstance(a, Node) and a.node_executable == 'static_transform_publisher']


def _words(node):
    return [getattr(s, 'text', '') for part in node.cmd for s in part]


def test_launch_publishes_exactly_one_tf_with_child_camera_link(tmp_path):
    module = _load_launch()
    ld = module.generate_launch_description()
    start = [e for e in ld.entities if isinstance(e, OpaqueFunction)][0]
    for use_apriltag in ('false', 'true'):
        actions = start.execute(_context(_write(tmp_path, GOOD), use_apriltag))
        nodes = _tf_nodes(actions)
        assert len(nodes) == 1
        words = _words(nodes[0])
        assert words[words.index('--child-frame-id') + 1] == 'camera_link'
        assert words[words.index('--frame-id') + 1] == 'base_link'
    # No file at the default path: no TF and one log line. A file that is not valid, or a
    # missing file at an explicit path: error before any action is returned.
    default = [a for a in ld.get_launch_arguments() if a.name == 'extrinsics_file'][0]
    default_path = default.default_value[0].text
    actions = start.execute(_context(default_path))
    assert _tf_nodes(actions) == []
    assert len([a for a in actions if isinstance(a, LogInfo)]) == 1
    with pytest.raises(cc.ExtrinsicsError):
        start.execute(_context(_write(tmp_path, {'translation_m': 1}, 'bad.yaml')))
    with pytest.raises(cc.ExtrinsicsError):
        start.execute(_context(tmp_path / 'typo.yaml'))


def test_launch_declares_only_the_two_arguments_and_calls_the_check():
    module = _load_launch()
    ld = module.generate_launch_description()
    assert {a.name for a in ld.get_launch_arguments()} == {'use_apriltag', 'extrinsics_file'}
    assert set(cc.DECLARED_ARGUMENTS) == {'use_apriltag', 'extrinsics_file'}
    text = LAUNCH_FILE.read_text()
    assert 'cc.check_launch_arguments(' in text
    assert 'difflib' not in text  # the check is not copied into the launch file


def test_apriltag_yaml_has_the_old_values():
    node = yaml.safe_load(APRILTAG_YAML.read_text())['apriltag']['ros__parameters']
    assert node['family'] == '36h11'
    assert node['size'] == 0.06
    assert node['max_hamming'] == 0
    assert node['image_transport'] == 'raw'
    assert node['detector'] == {
        'threads': 2, 'decimate': 2.0, 'blur': 0.0, 'refine': True, 'sharpening': 0.25}
    assert node['tag'] == {
        'ids': [0, 1], 'frames': ['tag36h11_0', 'tag36h11_1'], 'sizes': [0.06, 0.06]}
    assert dict(cc.APRILTAG_REMAPPINGS) == {
        'image_rect': '/camera/camera/color/image_raw',
        'camera_info': '/camera/camera/color/camera_info',
    }
