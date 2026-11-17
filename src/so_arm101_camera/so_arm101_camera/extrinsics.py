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
C3: Kabsch of the recorded points and the conversion to base_link to camera_link.

Pure code: no ROS, no clock of its own. It reads the CSV and the meta file written by record.py
and computes everything from them. The only writes are in write_outputs (the extrinsics file and
its record), which never overwrites without force and never writes a file that failed the checks.

Every limit below is a guess (推測) until the first run; see docs/camera_stage_plan.md (9.1).
"""

import csv
import dataclasses
import datetime
import hashlib
import math
import os
from pathlib import Path
import tempfile

import numpy as np
from so_arm101_ik import step_runner as sr
import yaml

from . import camera_config as cc
from . import record as rec
from . import tag_mount as tm

# Row selection (all rows that satisfy every condition are used).
CAM_N_MIN = 10  # frames kept per point (推測: about 2/3 of the 15 asked for)
INCIDENCE_MAX_DEG = 60.0  # same as the study of the points (docs/camera_pose_study.md)
CAM_STD_MAX_M = 0.001  # m, on each axis (推測)
GROUP_DISTANCE_M = 0.010  # m, rows this close are left out together in the leave-one-out (推測)

# Limits of the judgement.
MIN_POINTS = 6  # as the note of the old cal_to_tf.py:171
RMS_MAX_M = 0.005  # the old warning of cal_to_tf.py:169 (推測)
LOO_MAX_M = 0.008  # (推測)
SCALE_TOLERANCE = 0.01  # |scale - 1| (推測)
S_MIN_MIN_M = 0.025  # docs/camera_pose_study.md 3.3 (推測)

EXIT_OK = 0
EXIT_FAIL = 2  # the checks failed or something refused: nothing was written

OPTICAL_PARENT = 'camera_link'
OPTICAL_CHILD = 'camera_color_optical_frame'
SIDECAR_SUFFIX = '.meta.yaml'
SCHEMA = 1


class InputError(ValueError):
    """The CSV or the meta file cannot be used."""


class DegenerateError(ValueError):
    """The points do not determine a rotation (fewer than 3 or on a line)."""


class WriteRefused(RuntimeError):
    """The output files may not be written; nothing was written."""


# ----------------------------------------------------------------------------- mathematics

def kabsch(points, targets):
    """
    Return (R, t, flipped) with targets ~ R points + t (rigid, no scale).

    flipped is True if the best orthogonal matrix was a reflection and the last axis had to be
    corrected. Raises DegenerateError if the points are fewer than 3 or on a line.
    """
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    q = np.asarray(targets, dtype=float).reshape(-1, 3)
    if len(p) != len(q) or len(p) < 3:
        raise DegenerateError('at least 3 points are needed')
    pc, qc = p.mean(axis=0), q.mean(axis=0)
    a, b = p - pc, q - qc
    if np.linalg.svd(a, compute_uv=False)[1] < 1e-9 or np.linalg.svd(
            b, compute_uv=False)[1] < 1e-9:
        raise DegenerateError('the points are on a line')
    u, _, vt = np.linalg.svd(a.T @ b)
    d = 1.0 if np.linalg.det(vt.T @ u.T) >= 0.0 else -1.0
    rotation = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return rotation, qc - rotation @ pc, d < 0.0


def similarity_scale(points, targets):
    """Return the scale s of the best fit targets ~ s R points + t (Umeyama)."""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    q = np.asarray(targets, dtype=float).reshape(-1, 3)
    a, b = p - p.mean(axis=0), q - q.mean(axis=0)
    u, s, vt = np.linalg.svd(a.T @ b)
    d = 1.0 if np.linalg.det(vt.T @ u.T) >= 0.0 else -1.0
    return float((s[0] + s[1] + d * s[2]) / np.sum(a * a))


def residuals(points, targets, rotation, translation):
    """Return the length [m] of targets - (R points + t) for each point."""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    q = np.asarray(targets, dtype=float).reshape(-1, 3)
    return np.linalg.norm(q - (p @ np.asarray(rotation).T + np.asarray(translation)), axis=1)


def rms(values):
    """Return the root mean square of the lengths."""
    v = np.asarray(values, dtype=float)
    return float(math.sqrt(np.mean(v * v)))


def spread(targets):
    """Return the spreads [m] (RMS distance on each principal axis), largest first."""
    q = np.asarray(targets, dtype=float).reshape(-1, 3)
    s = np.linalg.svd(q - q.mean(axis=0), compute_uv=False)
    return s / math.sqrt(len(q))


def groups_of(targets, distance=GROUP_DISTANCE_M):
    """Return lists of row indices; rows closer than (or at) distance are in one list."""
    q = np.asarray(targets, dtype=float).reshape(-1, 3)
    parent = list(range(len(q)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(len(q)):
        for j in range(i + 1, len(q)):
            if np.linalg.norm(q[i] - q[j]) <= distance:
                parent[find(j)] = find(i)
    found = {}
    for i in range(len(q)):
        found.setdefault(find(i), []).append(i)
    return list(found.values())


def leave_out_errors(points, targets, groups):
    """
    Return the error [m] of each row predicted by a fit that left out its whole group.

    nan where the fit cannot be made (fewer than 4 points left, or on a line).
    """
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    q = np.asarray(targets, dtype=float).reshape(-1, 3)
    errors = np.full(len(p), np.nan)
    for group in groups:
        keep = np.array([i not in group for i in range(len(p))])
        if keep.sum() < 4:
            continue
        try:
            rotation, translation, _ = kabsch(p[keep], q[keep])
        except DegenerateError:
            continue
        for k in group:
            errors[k] = np.linalg.norm(q[k] - (rotation @ p[k] + translation))
    return errors


def quaternion_to_matrix(quaternion):
    """Return the rotation matrix of a quaternion (x, y, z, w); it is normalised first."""
    x, y, z, w = np.asarray(quaternion, dtype=float) / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quaternion(r):
    """Return the unit quaternion (x, y, z, w) of a rotation matrix, with w >= 0."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = s / 4.0, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, \
            (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w, x, y, z = (r[2, 1] - r[1, 2]) / s, s / 4.0, (r[0, 1] + r[1, 0]) / s, \
            (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, s / 4.0, \
            (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, \
            (r[1, 2] + r[2, 1]) / s, s / 4.0
    q = np.array([x, y, z, w])
    q = q / np.linalg.norm(q)
    return -q if q[3] < 0.0 else q


def matrix_to_rpy_deg(r):
    """Return (roll, pitch, yaw) [deg] of a rotation matrix (R = Rz Ry Rx)."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    return np.degrees([math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])])


def camera_link_pose(rotation, translation, optical_rotation, optical_translation):
    """
    Return (R_bl, t_bl): the pose of camera_link in base_link.

    rotation and translation map optical-frame points to base_link points (the Kabsch result);
    optical_rotation and optical_translation are the pose of the optical frame in camera_link.
    R_bl = R R_lo^T and t_bl = t - R_bl t_lo.
    """
    r_bl = np.asarray(rotation) @ np.asarray(optical_rotation).T
    return r_bl, np.asarray(translation) - r_bl @ np.asarray(optical_translation)


def judge(count, rms_m, loo_max_m, scale, s_min_m, flipped):
    """Return one line for each failed check (an empty list means the result is accepted)."""
    lines = []
    if count < MIN_POINTS:
        lines.append(f'only {count} points are used, {MIN_POINTS} or more are needed')
    if not math.isfinite(rms_m) or rms_m > RMS_MAX_M:
        lines.append(f'RMS {rms_m * 1000:.2f} mm is over {RMS_MAX_M * 1000:.1f} mm')
    if not math.isfinite(loo_max_m):
        lines.append('the leave-one-out could not be made')
    elif loo_max_m > LOO_MAX_M:
        lines.append(f'the largest leave-one-out error {loo_max_m * 1000:.2f} mm is over '
                     f'{LOO_MAX_M * 1000:.1f} mm')
    if not math.isfinite(scale) or abs(scale - 1.0) > SCALE_TOLERANCE:
        lines.append(f'the scale {scale:.4f} is not within 1 +- {SCALE_TOLERANCE}')
    if not math.isfinite(s_min_m) or s_min_m < S_MIN_MIN_M:
        lines.append(f'the smallest spread {s_min_m * 1000:.1f} mm is under '
                     f'{S_MIN_MIN_M * 1000:.1f} mm')
    if flipped:
        lines.append('the fit needed the correction of a reflection')
    return lines


# ----------------------------------------------------------------------------- the input

@dataclasses.dataclass(frozen=True)
class Point:
    """One used row: the tag centre seen by the camera and computed by the forward kinematics."""

    label: str
    camera: tuple  # optical frame [m]
    fk: tuple  # base_link [m]


@dataclasses.dataclass(frozen=True)
class Optical:
    """The pose of the optical frame in camera_link, as recorded in the meta file."""

    translation: tuple
    quaternion: tuple


def _number(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    return number


def read_csv_rows(csv_path):
    """Return the rows (dicts) of the CSV; the header must be the one of record.py."""
    try:
        with open(csv_path, newline='', encoding='utf-8') as f:
            rows = list(csv.reader(f))
    except OSError as exc:
        raise InputError(f'cannot read {csv_path}: {exc}') from exc
    if not rows or rows[0] != list(rec.COLUMNS):
        raise InputError(f'the header of {csv_path} is not the one of record.py')
    result = []
    for number, row in enumerate(rows[1:], start=2):
        if len(row) != len(rec.COLUMNS):
            raise InputError(f'{csv_path}: line {number} has {len(row)} columns')
        result.append(dict(zip(rec.COLUMNS, row)))
    labels = [r['label'] for r in result]
    if len(set(labels)) != len(labels):
        raise InputError(f'{csv_path}: a label is used twice')
    return result


def _finite_list(name, value, count):
    if (not isinstance(value, (list, tuple)) or len(value) != count
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value)
            or not all(math.isfinite(v) for v in value)):
        raise InputError(f'{name} must be a list of {count} finite numbers')
    return tuple(float(v) for v in value)


def read_meta(meta_path):
    """Return (meta, Optical): the meta file and the pose of the optical frame in camera_link."""
    try:
        with open(meta_path, encoding='utf-8') as f:
            meta = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as exc:
        raise InputError(f'cannot read {meta_path}: {exc}') from exc
    if not isinstance(meta, dict) or meta.get('schema') != 1:
        raise InputError(f'{meta_path}: the schema is not 1')
    extra = meta.get('extra')
    link = extra.get('camera_link_to_optical_frame') if isinstance(extra, dict) else None
    if not isinstance(link, dict):
        raise InputError(f'{meta_path}: extra.camera_link_to_optical_frame is missing')
    if link.get('parent') != OPTICAL_PARENT or link.get('child') != OPTICAL_CHILD:
        raise InputError(f'{meta_path}: the transform is not {OPTICAL_PARENT} to {OPTICAL_CHILD}')
    translation = _finite_list('translation_m', link.get('translation_m'), 3)
    quaternion = _finite_list('quaternion_xyzw', link.get('quaternion_xyzw'), 4)
    if np.linalg.norm(quaternion) < 1e-6:
        raise InputError(f'{meta_path}: the quaternion has no length')
    return meta, Optical(translation, quaternion)


def select_rows(rows, exclude=()):
    """
    Return (points, rejected): the rows that may be used and a list of (label, reason).

    A row is used only if every condition holds (tag_seen, still and stable are 1, cam_n,
    incidence and the standard deviations are within the limits, the numbers are finite).
    Raises InputError for an excluded label that is not in the CSV.
    """
    labels = [r['label'] for r in rows]
    for label in exclude:
        if label not in labels:
            raise InputError(f'--exclude {label}: there is no such label')
    points, rejected = [], []
    for row in rows:
        if row['label'] in exclude:
            rejected.append((row['label'], 'excluded by --exclude'))
            continue
        reasons = []
        for key in ('tag_seen', 'still', 'stable'):
            if row[key] != '1':
                reasons.append(f'{key} is {row[key]}')
        cam_n = _number(row['cam_n'])
        if not cam_n >= CAM_N_MIN:
            reasons.append(f'cam_n {row["cam_n"]} is under {CAM_N_MIN}')
        incidence = _number(row['incidence_deg'])
        if not incidence <= INCIDENCE_MAX_DEG:
            reasons.append(f'incidence {row["incidence_deg"]} deg is over {INCIDENCE_MAX_DEG}')
        for axis in 'xyz':
            if not _number(row[f'cam_std_{axis}']) <= CAM_STD_MAX_M:
                reasons.append(f'cam_std_{axis} {row["cam_std_" + axis]} m is over '
                               f'{CAM_STD_MAX_M}')
        camera = tuple(_number(row[f'cam_tag_{a}']) for a in 'xyz')
        fk = tuple(_number(row[f'fk_tag_{a}']) for a in 'xyz')
        if not all(math.isfinite(v) for v in camera + fk):
            reasons.append('a tag position is not a finite number')
        if reasons:
            rejected.append((row['label'], '; '.join(reasons)))
        else:
            points.append(Point(row['label'], camera, fk))
    return points, rejected


# ----------------------------------------------------------------------------- zero offsets

ANGLE_JOINTS = tuple(sr.JOINT_NAMES)
JOINT_OFFSET_MAX_DEG = 10.0  # the same upper limit as the driver (kMaxZeroOffsetDeg)
BASELINE_TOL_M = 5e-6  # fk_tag recomputed at delta 0 against the CSV (6 digits of q and fk_tag)


@dataclasses.dataclass(frozen=True)
class OffsetPlan:
    """What is known about the zero offsets of a CSV and what is applied to it."""

    applied: dict  # the --joint-offset values (degrees); empty if none was given
    recorded: object  # the zero_offset_deg of the meta file (dict), None for an older meta file
    calibration_sha256: object = None  # follower_calibration_sha256 of the meta file or None
    fk_source: str = 'csv'  # 'csv' (the fk_tag of the CSV) or 'recomputed'
    urdf_sha256: object = None
    tag_offset: object = None
    baseline_max_m: float = math.nan


def parse_joint_offsets(items):
    """
    Return {joint: degrees} of the --joint-offset values NAME=DEG (q_true = q_meas + DEG).

    Raises InputError for a missing "=", a name that is not one of the five angle joints (the
    gripper has no offset), a value that is not a finite number, one over 10 degrees, or a joint
    given twice.
    """
    offsets = {}
    for item in items:
        name, separator, text = str(item).partition('=')
        if not separator:
            raise InputError(f'--joint-offset {item}: the form is NAME=DEG')
        if name == 'gripper':
            raise InputError('--joint-offset gripper: the gripper has no zero offset')
        if name not in ANGLE_JOINTS:
            raise InputError(f'--joint-offset {name}: not one of {", ".join(ANGLE_JOINTS)}')
        try:
            value = float(text)
        except ValueError as exc:
            raise InputError(f'--joint-offset {item}: {text!r} is not a number') from exc
        if not math.isfinite(value) or abs(value) > JOINT_OFFSET_MAX_DEG:
            raise InputError(f'--joint-offset {item}: the value must be finite and within '
                             f'+-{JOINT_OFFSET_MAX_DEG} deg')
        if name in offsets:
            raise InputError(f'--joint-offset {name} is given twice')
        offsets[name] = value
    return offsets


def recorded_offsets(meta):
    """Return the zero_offset_deg recorded in the identity of the meta file, or None if absent."""
    identity = meta.get('identity') if isinstance(meta, dict) else None
    if not isinstance(identity, dict) or 'zero_offset_deg' not in identity:
        return None
    value = identity['zero_offset_deg']
    if (not isinstance(value, dict) or set(value) != set(ANGLE_JOINTS)
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) or abs(v) > JOINT_OFFSET_MAX_DEG
                   for v in value.values())):
        raise InputError('the zero_offset_deg of the meta file is not a number for each of '
                         f'{", ".join(ANGLE_JOINTS)} within +-{JOINT_OFFSET_MAX_DEG} deg')
    return {name: float(value[name]) for name in ANGLE_JOINTS}


def plan_offsets(meta, requested):
    """
    Decide what may be done with the zero offsets. Raise InputError if it is not allowed.

    An older meta file (no zero_offset_deg) is accepted only together with --joint-offset (NAME=0
    says there is none). A meta file that recorded an offset other than 0 was made with a driver
    that had corrected the angles already: --joint-offset on top of it would count it twice.
    """
    recorded = recorded_offsets(meta)
    if recorded is None and not requested:
        raise InputError(
            'the meta file has no record of the zero offsets (it was made before they were '
            'recorded): give --joint-offset NAME=DEG for the correction (NAME=0 for none)')
    if requested and recorded is not None and any(v != 0.0 for v in recorded.values()):
        raise InputError(
            'the meta file recorded zero offsets other than 0 (the angles of the CSV are '
            f'already corrected: {recorded}): --joint-offset would count them twice')
    identity = meta.get('identity') if isinstance(meta, dict) else None
    sha = identity.get('follower_calibration_sha256') if isinstance(identity, dict) else None
    return OffsetPlan(dict(requested), recorded, sha)


def _tag_offset_of(meta):
    mount = meta.get('tag_mount') if isinstance(meta, dict) else None
    value = mount.get('offset_gripper_frame_m') if isinstance(mount, dict) else None
    return _finite_list('tag_mount.offset_gripper_frame_m', value, 3)


def recompute_fk_tag(rows, meta, urdf_path, plan):
    """
    Return (rows, plan) with fk_tag_* computed again from q_* plus the offsets of the plan.

    The URDF must be the one of the meta file (sha256), and the fk_tag computed from the q of
    the CSV without any offset must be the fk_tag of the CSV (within BASELINE_TOL_M); otherwise
    InputError. The rows given are not changed.
    """
    identity = meta.get('identity') if isinstance(meta, dict) else None
    wanted = identity.get('urdf_sha256') if isinstance(identity, dict) else None
    try:
        found = sha256_file(urdf_path)
    except OSError as exc:
        raise InputError(f'cannot read {urdf_path}: {exc}') from exc
    if not isinstance(wanted, str) or found != wanted:
        raise InputError(f'{urdf_path}: its sha256 {found} is not the urdf_sha256 of the meta '
                         f'file ({wanted})')
    offset = _tag_offset_of(meta)
    try:
        chain = rec.load_chain(urdf_path)
    except (OSError, ValueError, KeyError, SyntaxError) as exc:
        raise InputError(f'cannot use {urdf_path}: {exc}') from exc
    delta = np.radians([plan.applied.get(name, 0.0) for name in ANGLE_JOINTS])
    result, baseline = [], 0.0
    for row in rows:
        new = dict(row)
        q = np.array([_number(row['q_' + name]) for name in ANGLE_JOINTS])
        if not np.all(np.isfinite(q)):
            for axis in 'xyz':
                new['fk_tag_' + axis] = 'nan'
            result.append(new)
            continue
        recorded = np.array([_number(row['fk_tag_' + a]) for a in 'xyz'])
        if np.all(np.isfinite(recorded)):
            gap = float(np.linalg.norm(tm.fk_tag(chain, q, offset) - recorded))
            if gap > BASELINE_TOL_M:
                raise InputError(
                    f'{row["label"]}: the fk_tag computed from q differs from the CSV by '
                    f'{gap * 1e6:.1f} um (limit {BASELINE_TOL_M * 1e6:.0f} um): not the same '
                    'URDF, tag offset or angles')
            baseline = max(baseline, gap)
        for axis, value in zip('xyz', tm.fk_tag(chain, q + delta, offset)):
            new['fk_tag_' + axis] = repr(float(value))
        result.append(new)
    return result, OffsetPlan(plan.applied, plan.recorded, plan.calibration_sha256,
                              'recomputed', found, offset, baseline)


def offset_lines(plan, count):
    """Return the lines that say which zero offsets were applied."""
    if plan.fk_source == 'csv':
        return ['zero offsets: none applied; the fk_tag of the CSV is used (the meta file '
                f'recorded {plan.recorded})']
    shown = ', '.join(f'{n} {v:+.4g} deg' for n, v in plan.applied.items())
    return [f'zero offsets applied (q_true = q_meas + delta): {shown}',
            f'fk_tag computed again from q for {count} rows (without the offsets it differs '
            f'from the CSV by at most {plan.baseline_max_m * 1e6:.1f} um)']


def offsets_record(plan):
    """Return the part of the record that says which zero offsets were used."""
    plan = plan or OffsetPlan({}, None)
    return {
        'sign': 'q_true = q_meas + delta (degrees, added to the q_ columns before the FK)',
        'source': 'command_line' if plan.applied else 'none',
        'applied_deg': dict(plan.applied),
        'fk_tag_source': plan.fk_source,
        'meta_zero_offset_deg': plan.recorded if plan.recorded is not None else 'absent',
        'meta_follower_calibration_sha256': plan.calibration_sha256 or 'absent',
        'urdf_sha256': plan.urdf_sha256,
        'tag_offset_gripper_frame_m': list(plan.tag_offset) if plan.tag_offset else None,
        'baseline_check_max_um': (float(plan.baseline_max_m * 1e6)
                                  if math.isfinite(plan.baseline_max_m) else None),
    }


# ----------------------------------------------------------------------------- the solution

@dataclasses.dataclass(frozen=True)
class Solution:
    """Everything computed from the used points; the arrays are None if nothing could be fitted."""

    labels: tuple
    failures: tuple
    rotation: object = None  # optical frame to base_link
    translation: object = None
    flipped: bool = False
    residual: object = None  # per point [m]
    rms: float = math.nan
    leave_out: object = None  # per point [m]
    groups: tuple = ()
    leave_out_max: float = math.nan
    scale: float = math.nan
    tag_size: float = math.nan  # implied side of the tag [m], nan if the meta file has none
    spreads: object = None  # [m], largest first
    s_min: float = math.nan
    link_rotation: object = None  # camera_link in base_link
    link_translation: object = None
    quaternion: object = None
    rpy_deg: object = None


def solve(points, optical, tag_size=None):
    """Fit the points, convert to camera_link and judge. Never raises for bad data."""
    labels = tuple(p.label for p in points)
    if len(points) < 4:
        return Solution(labels, tuple(judge(len(points), math.nan, math.nan, math.nan,
                                            math.nan, False)))
    cam = np.array([p.camera for p in points])
    fk = np.array([p.fk for p in points])
    try:
        rotation, translation, flipped = kabsch(cam, fk)
    except DegenerateError as exc:
        return Solution(labels, (f'the points do not determine a pose: {exc}',))
    residual = residuals(cam, fk, rotation, translation)
    groups = groups_of(fk)
    leave_out = leave_out_errors(cam, fk, groups)
    loo_max = float(np.max(leave_out)) if not np.any(np.isnan(leave_out)) else math.nan
    scale = similarity_scale(cam, fk)
    spreads = spread(fk)
    optical_rotation = quaternion_to_matrix(optical.quaternion)
    link_rotation, link_translation = camera_link_pose(
        rotation, translation, optical_rotation, np.asarray(optical.translation))
    failures = judge(len(points), rms(residual), loo_max, scale, float(spreads[2]), flipped)
    return Solution(
        labels, tuple(failures), rotation, translation, flipped, residual, rms(residual),
        leave_out, tuple(tuple(g) for g in groups), loo_max, scale,
        tag_size * scale if tag_size else math.nan, spreads, float(spreads[2]),
        link_rotation, link_translation, matrix_to_quaternion(link_rotation),
        matrix_to_rpy_deg(link_rotation))


def tag_size_of(meta):
    """Return the side of the tag [m] recorded in the meta file, or None."""
    mount = meta.get('tag_mount') if isinstance(meta, dict) else None
    size = mount.get('tag_size_m') if isinstance(mount, dict) else None
    return float(size) if isinstance(size, (int, float)) and not isinstance(size, bool) else None


def report_lines(solution, rejected):
    """Return the lines of the report (every number that was computed, also when it failed)."""
    lines = [f'rows used: {len(solution.labels)}  rejected: {len(rejected)}']
    for label, reason in rejected:
        lines.append(f'  not used {label}: {reason}')
    if solution.residual is None:
        lines.append('no fit could be made')
    else:
        lines.append(f'RMS {solution.rms * 1000:.2f} mm  largest residual '
                     f'{float(np.max(solution.residual)) * 1000:.2f} mm')
        for i, label in enumerate(solution.labels):
            lines.append(f'  {label}: residual {solution.residual[i] * 1000:.2f} mm  '
                         f'leave-out {solution.leave_out[i] * 1000:.2f} mm')
        lines.append('leave-out groups: ' + ', '.join(
            '+'.join(solution.labels[i] for i in g) for g in solution.groups))
        lines.append(f'largest leave-out error {solution.leave_out_max * 1000:.2f} mm')
        lines.append(f'scale {solution.scale:.4f}' + (
            f'  implied tag side {solution.tag_size * 1000:.2f} mm'
            if math.isfinite(solution.tag_size) else ''))
        lines.append('spreads [mm] ' + ' '.join(f'{v * 1000:.1f}' for v in solution.spreads)
                     + f'  smallest {solution.s_min * 1000:.1f}')
        t, q, rpy = solution.link_translation, solution.quaternion, solution.rpy_deg
        lines.append(f'base_link to camera_link: translation_m {np.round(t, 5).tolist()} '
                     f'quaternion_xyzw {np.round(q, 5).tolist()} rpy_deg '
                     f'{np.round(rpy, 2).tolist()}')
    lines += [f'FAILED: {f}' for f in solution.failures]
    lines.append('result: ' + ('FAILED' if solution.failures else 'accepted'))
    return lines


# ----------------------------------------------------------------------------- the output

def extrinsics_text(translation, quaternion):
    """Return the text of camera_extrinsics.yaml: the two keys of camera_config only."""
    body = yaml.safe_dump(
        {'translation_m': [float(v) for v in translation],
         'quaternion_xyzw': [float(v) for v in quaternion]}, sort_keys=False)
    return ('# Read by: camera.launch.py (static_transform_publisher, base_link to camera_link). '
            'Written by tools/camera_extrinsics_run.py. Do not edit by hand.\n'
            '# The record of the run (inputs, residuals, sha256) is camera_extrinsics.yaml'
            + SIDECAR_SUFFIX + '.\n' + body)


def sha256_text(text):
    """Return the sha256 (hex) of a text."""
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def sha256_file(path):
    """Return the sha256 (hex) of the bytes of a file."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def utc_now_text():
    """Return the present UTC time as text."""
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


def sidecar_text(solution, rejected, csv_path, meta_path, meta, optical, main_text, created,
                 exclude=(), offsets=None):
    """Return the text of the record that is written next to the extrinsics file."""
    labels = solution.labels
    mm = 1000.0
    record = {
        'schema': SCHEMA,
        'created': created,
        'tool': 'tools/camera_extrinsics_run.py',
        'extrinsics_sha256': sha256_text(main_text),
        'input': {'csv': Path(csv_path).name, 'csv_sha256': sha256_file(csv_path),
                  'meta_sha256': sha256_file(meta_path)},
        'identity': rec.to_plain(meta.get('identity', {})),
        'joint_offsets': rec.to_plain(offsets_record(offsets)),
        'rows': {'used': list(labels),
                 'not_used': {label: reason for label, reason in rejected},
                 'excluded': list(exclude)},
        'thresholds': {
            'cam_n_min': CAM_N_MIN, 'incidence_max_deg': INCIDENCE_MAX_DEG,
            'cam_std_max_m': CAM_STD_MAX_M, 'group_distance_m': GROUP_DISTANCE_M,
            'min_points': MIN_POINTS, 'rms_max_m': RMS_MAX_M, 'loo_max_m': LOO_MAX_M,
            'scale_tolerance': SCALE_TOLERANCE, 's_min_min_m': S_MIN_MIN_M},
        'result': {
            'rms_mm': float(solution.rms * mm),
            'residual_mm': {n: float(v * mm) for n, v in zip(labels, solution.residual)},
            'residual_max_mm': float(np.max(solution.residual) * mm),
            'leave_out_mm': {n: float(v * mm) for n, v in zip(labels, solution.leave_out)},
            'leave_out_max_mm': float(solution.leave_out_max * mm),
            'leave_out_groups': [[labels[i] for i in g] for g in solution.groups],
            'scale': float(solution.scale),
            'implied_tag_size_m': (float(solution.tag_size)
                                   if math.isfinite(solution.tag_size) else None),
            'spreads_mm': [float(v * mm) for v in solution.spreads],
            's_min_mm': float(solution.s_min * mm),
            'rpy_deg': [float(v) for v in solution.rpy_deg],
            'reflection_corrected': bool(solution.flipped)},
        'camera_link_to_optical_frame': {
            'translation_m': list(optical.translation),
            'quaternion_xyzw': list(optical.quaternion)},
    }
    return yaml.safe_dump(record, sort_keys=False, allow_unicode=True)


def sidecar_path_for(output):
    """Return the path of the record that belongs to an extrinsics file."""
    return Path(str(output) + SIDECAR_SUFFIX)


def describe_existing(output):
    """Return one line about the file that a forced write replaces."""
    try:
        translation, quaternion = cc.load_extrinsics(output)
    except (cc.ExtrinsicsError, OSError) as exc:
        return f'{output}: the present file is not valid ({exc})'
    return f'{output}: present translation_m {translation} quaternion_xyzw {quaternion}'


def write_outputs(output, main_text, side_text, translation, quaternion, force=False):
    """
    Write the extrinsics file and its record. Raise WriteRefused if nothing may be written.

    Both texts are first written to temporary files next to the output and the main one is read
    back with camera_config.load_extrinsics. A new file is made with os.link (it fails if the
    file appeared meanwhile); a forced write replaces with os.replace. The main file is the last.
    """
    output = Path(output)
    side = sidecar_path_for(output)
    if not output.parent.is_dir():
        raise WriteRefused(f'{output.parent} is not a directory')
    if (output.exists() or side.exists()) and not force:
        raise WriteRefused(f'{output} or {side} exists (use --force to replace)')
    made = []
    try:
        temporary = []
        for text in (side_text, main_text):
            fd, name = tempfile.mkstemp(dir=output.parent, prefix='.camera_extrinsics_')
            made.append(name)
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                f.write(text)
                f.flush()
                os.fsync(f.fileno())
            temporary.append(name)
        try:
            back_t, back_q = cc.load_extrinsics(temporary[1])
        except cc.ExtrinsicsError as exc:
            raise WriteRefused(f'the candidate is not valid: {exc}') from exc
        if (np.max(np.abs(np.array(back_t) - np.asarray(translation))) > 1e-12
                or np.max(np.abs(np.array(back_q) - np.asarray(quaternion))) > 1e-12):
            raise WriteRefused('the candidate does not read back as the computed values')
        for name, final in zip(temporary, (side, output)):
            if force:
                os.replace(name, final)
            else:
                try:
                    os.link(name, final)
                except FileExistsError as exc:
                    raise WriteRefused(f'{final} appeared meanwhile') from exc
    finally:
        for name in made:
            if os.path.exists(name):
                os.unlink(name)
