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

"""Tests of the undeclared-launch-argument check of bringup.launch.py (no node is started)."""

import importlib.util
from pathlib import Path

from launch import LaunchContext, LaunchDescription, LaunchDescriptionSource, LaunchService
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration,
)
import pytest

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'


def _load():
    spec = importlib.util.spec_from_file_location('bringup_launch_under_test', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MODULE = _load()
LD = MODULE.generate_launch_description()
DECLARED = sorted({a.name for a in LD.get_launch_arguments()})


def _check(argv):
    return MODULE._check_launch_arguments(LaunchContext(argv=argv), DECLARED)


def _message(argv):
    with pytest.raises(RuntimeError) as info:
        _check(argv)
    return str(info.value)


def test_check_is_the_first_action():
    first = LD.entities[0]
    assert isinstance(first, OpaqueFunction)
    assert not first.execute(LaunchContext(argv=['mock:=true']))
    with pytest.raises(RuntimeError, match='foo'):
        first.execute(LaunchContext(argv=['foo:=bar']))


def test_declared_arguments():
    assert len(DECLARED) == 21
    assert 'hardware' not in DECLARED
    for name in ('mock', 'read_only', 'torque_limit', 'use_rviz',
                 'monitor_dynamics', 'rviz_config'):
        assert name in DECLARED


def test_no_arguments():
    assert _check([]) == []


@pytest.mark.parametrize('name', DECLARED)
def test_every_declared_argument_is_accepted(name):
    assert _check([f'{name}:=x']) == []


@pytest.mark.parametrize('name', ['launch-prefix', 'launch-prefix-filter'])
def test_ros2launch_option_arguments_are_accepted(name):
    assert _check([f'{name}:=x']) == []


def test_undeclared_argument_is_reported_with_the_declared_list():
    message = _message(['mock:=true', 'foo:=bar'])
    assert 'foo' in message
    assert 'Declared arguments:' in message
    assert 'mock' in message and 'read_only' in message
    assert 'hardware' not in message


@pytest.mark.parametrize('value', ['feetech', 'lerobot'])
def test_hardware_argument_gets_the_removal_message(value):
    message = _message([f'hardware:={value}'])
    assert "The 'hardware' argument was removed" in message
    assert 'stage2c-done' in message
    assert 'Remove it and run again' in message


@pytest.mark.parametrize('typo, expected', [
    ('read_olny', 'read_only'),
    ('use_rvis', 'use_rviz'),
    ('Mock', 'mock'),
])
def test_typos_get_a_suggestion(typo, expected):
    message = _message([f'{typo}:=true'])
    assert f'{typo} (did you mean: ' in message
    assert expected in message.split('did you mean: ')[1].split('?)')[0]


def _run(argv):
    """Run the check inside a LaunchService, after launch-internal values were set."""
    inner = LaunchDescription([
        DeclareLaunchArgument('inner_arg', default_value='v'),
        OpaqueFunction(
            function=lambda context: [SetLaunchConfiguration('added_by_opaque', '1')]),
    ])
    description = LaunchDescription([
        OpaqueFunction(function=MODULE._check_launch_arguments, args=[DECLARED]),
        SetLaunchConfiguration('internal_x', '1'),
        IncludeLaunchDescription(
            LaunchDescriptionSource(inner), launch_arguments={'forwarded': '1'}.items()),
    ])
    service = LaunchService(argv=argv)
    service.include_launch_description(description)
    return service.run()


def test_launch_internal_values_are_not_reported():
    assert _run([]) == 0
    assert _run(['mock:=true', 'monitor_dynamics:=false']) == 0


def test_undeclared_argument_stops_the_launch_service():
    assert _run(['foo:=bar']) != 0
