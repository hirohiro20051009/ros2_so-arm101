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

"""
Pure helpers of camera.launch.py (no ROS node is started from here).

camera.launch.py only calls these functions, so the code that is tested is the code that runs.
"""

import difflib
import math
from pathlib import Path

import yaml

# Launch arguments declared by camera.launch.py (and nothing else: the arguments of rs_launch.py
# are NOT declared here; the RealSense settings come from config/realsense.yaml).
DECLARED_ARGUMENTS = ('use_apriltag', 'extrinsics_file')

# Names added by "ros2 launch" itself (--launch-prefix / --launch-prefix-filter).
ROS2LAUNCH_OPTION_ARGUMENTS = ('launch-prefix', 'launch-prefix-filter')

# Remapping of apriltag_node (raw colour image; rectification is not used). Same as the old
# start-up.
APRILTAG_REMAPPINGS = (
    ('image_rect', '/camera/camera/color/image_raw'),
    ('camera_info', '/camera/camera/color/camera_info'),
)

# Allowed distance of the quaternion norm from 1. A guess, not taken from a source: 1e-3 is about
# 0.06 degrees after normalisation, far below the 1 cm accuracy aimed at; a file written by C3 in
# double precision is off by about 1e-12.
QUATERNION_NORM_TOLERANCE = 1e-3

TF_PARENT_FRAME = 'base_link'
TF_CHILD_FRAME = 'camera_link'


class ExtrinsicsError(ValueError):
    """The extrinsics file exists but its content is not valid."""


def load_yaml_mapping(path):
    """Read a yaml file whose top level is a mapping."""
    path = Path(path)
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'cannot read {path}: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError(f'{path}: the top level must be a mapping')
    return data


def realsense_launch_arguments(path):
    """
    Read config/realsense.yaml and return the launch arguments of rs_launch.py.

    The file is a flat mapping. Every value is turned into a string (bool: true or false).
    """
    data = load_yaml_mapping(path)
    if not data:
        raise ValueError(f'{path}: no key')
    args = {}
    for key, value in data.items():
        if not isinstance(key, str):
            raise ValueError(f'{path}: key {key!r} is not a string')
        if isinstance(value, bool):
            args[key] = 'true' if value else 'false'
        elif isinstance(value, (int, float, str)):
            args[key] = str(value)
        else:
            raise ValueError(
                f'{path}: the value of {key} must be a scalar, not {type(value).__name__}')
    return args


def parse_bool_argument(name, value):
    """Turn the string of a launch argument (true or false) into a bool."""
    text = str(value).strip().lower()
    if text == 'true':
        return True
    if text == 'false':
        return False
    raise ValueError(f'launch argument {name} must be true or false, not {value!r}')


# The check below is a copy (not an import) of _check_launch_arguments in
# so_arm101_bringup/launch/bringup.launch.py (lines 59-86). bringup is not changed. Differences:
# it takes the typed arguments (context.argv) directly, and only the arguments declared by
# camera.launch.py count as declared. The arguments of rs_launch.py (about 90) are included from
# camera.launch.py, so counting them as declared would let every one of them through and bypass
# config/realsense.yaml.
def check_launch_arguments(argv, declared_names=DECLARED_ARGUMENTS):
    """
    Raise RuntimeError if an argument that is not declared was typed on the command line.

    Only the arguments typed by the user (context.argv) are checked.
    """
    given = [a.split(':=', 1)[0] for a in argv if ':=' in a]
    allowed = set(declared_names) | set(ROS2LAUNCH_OPTION_ARGUMENTS)
    unknown = sorted({name for name in given if name not in allowed})
    if not unknown:
        return
    parts = []
    for name in unknown:
        close = difflib.get_close_matches(name, sorted(allowed), n=2)
        hint = f' (did you mean: {", ".join(close)}?)' if close else ''
        parts.append(f'{name}{hint}')
    message = (
        'Undeclared launch argument(s): ' + ', '.join(parts) + '. '
        'Declared arguments: ' + ', '.join(sorted(declared_names)) + '.'
    )
    if any('.' in name for name in unknown):
        message += (
            ' The settings of the RealSense driver are not launch arguments of this launch file;'
            ' write them in config/realsense.yaml.'
        )
    raise RuntimeError(message)


def _finite_numbers(name, value, count):
    if (not isinstance(value, (list, tuple)) or len(value) != count
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value)):
        raise ExtrinsicsError(f'{name} must be a list of {count} numbers')
    values = [float(v) for v in value]
    if not all(math.isfinite(v) for v in values):
        raise ExtrinsicsError(f'{name} must be finite')
    return values


def load_extrinsics(path):
    """
    Read and check the extrinsics file (base_link to camera_link).

    Keys: translation_m [x, y, z] in metres and quaternion_xyzw [x, y, z, w] (unit length).
    Returns (translation, quaternion). Raises ExtrinsicsError if the content is not valid.
    """
    try:
        data = load_yaml_mapping(path)
    except ValueError as exc:
        raise ExtrinsicsError(str(exc)) from exc
    keys = {'translation_m', 'quaternion_xyzw'}
    if set(data) != keys:
        raise ExtrinsicsError(
            f'{path}: the keys must be exactly {sorted(keys)}, got {sorted(map(str, data))}')
    try:
        translation = _finite_numbers('translation_m', data['translation_m'], 3)
        quaternion = _finite_numbers('quaternion_xyzw', data['quaternion_xyzw'], 4)
    except ExtrinsicsError as exc:
        raise ExtrinsicsError(f'{path}: {exc}') from exc
    norm = math.sqrt(sum(q * q for q in quaternion))
    if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise ExtrinsicsError(f'{path}: quaternion_xyzw must have unit length, its norm is {norm}')
    return translation, quaternion


def static_transform_arguments(translation, quaternion):
    """Return the arguments of tf2_ros static_transform_publisher (named form, Humble)."""
    x, y, z = translation
    qx, qy, qz, qw = quaternion
    values = (('--x', x), ('--y', y), ('--z', z),
              ('--qx', qx), ('--qy', qy), ('--qz', qz), ('--qw', qw))
    args = []
    for flag, value in values:
        args += [flag, repr(float(value))]
    args += ['--frame-id', TF_PARENT_FRAME, '--child-frame-id', TF_CHILD_FRAME]
    return args


def plan_extrinsics_tf(path, default_path):
    """
    Decide what to do with the extrinsics file.

    Returns None only if the file does not exist and path is the default path (no TF is
    published). Raises ExtrinsicsError if the file exists but is not valid, or if a path other
    than the default was given and the file does not exist (a typo must not pass silently).
    Otherwise returns the arguments of static_transform_publisher.
    """
    if not Path(path).exists():
        if Path(path) != Path(default_path):
            raise ExtrinsicsError(f'extrinsics_file {path} does not exist')
        return None
    translation, quaternion = load_extrinsics(path)
    return static_transform_arguments(translation, quaternion)


def no_extrinsics_message(path):
    """Return the one log line printed when the extrinsics file does not exist."""
    return (f'{path} が無いので {TF_PARENT_FRAME} から {TF_CHILD_FRAME} の TF は出さない '
            f'(no extrinsics file: the TF {TF_PARENT_FRAME} to {TF_CHILD_FRAME} is not published)')
