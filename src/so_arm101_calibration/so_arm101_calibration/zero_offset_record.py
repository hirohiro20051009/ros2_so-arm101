# Copyright 2026 so_arm101 authors. MIT License.
"""
Records of the zero-offset calibration: the CSV, its meta file, the identity and the refusals.

One record is one touch pose: the mean of the last samples of /joint_states (read-only).  A CSV
has one identity (the follower calibration yaml, the delta the driver applied, the URDF and the
meshes); a row of another identity is not appended.  The functions here that decide are pure; the
ROS part is tools/zero_offset_record_run.py.  The rules of the identity are the same as those of
so_arm101_camera.record (read_zero_offsets, identity_mismatches); they are written again here so
that this package does not depend on the camera package (the tests check the same output for
the same input).
"""

import csv
import fcntl
import hashlib
import math
import os
from pathlib import Path
import re

import numpy as np
from so_arm101_ik.urdf_kinematics import ARM_JOINTS
import yaml

from . import zero_offset as zo

TOOL_VERSION = 'zero_offset_record_run 1'
GRIPPER = 'gripper'
STATE_JOINTS = ARM_JOINTS + (GRIPPER,)
LABEL_PATTERN = re.compile(r'[A-Za-z0-9_.-]+(#[0-9]+)?')  # used with fullmatch
CSV_COMMENT = '# zero-offset calibration (floor touch). Not the center calibration.'
ZERO_OFFSET_MAX_DEG = 10.0  # the same upper limit as the driver (kMaxZeroOffsetDeg)
STILL_DEG = 0.2  # the largest span of a joint over the samples that still counts as still (推測)
SAMPLES = 20  # samples averaged for one record (about 1 s; 推測)
TARGET_X_TOL_MM = 10.0
TARGET_EPS_TOL_DEG = 5.0
REPEATS = 2
# The poses of the plan: (target elevation deg, target x mm).  docs/zero_offset_calib_plan.md 5.2.
POSES = {'P1': (90.0, 100.0), 'P2': (90.0, 300.0), 'P3': (70.0, 150.0), 'P4': (50.0, 250.0)}

COLUMNS = (('idx', 'label', 'pose', 'time') + tuple('q_' + n for n in STATE_JOINTS)
           + tuple('span_deg_' + n for n in STATE_JOINTS)
           + ('samples', 'x_mm', 'eps_deg', 'model_height_mm', 'still', 'warn'))
IDENTITY_KEYS = ('calibration_kind', 'follower_calibration_sha256', 'zero_offset_deg',
                 'urdf_sha256', 'mesh_sha256')


class SessionError(RuntimeError):
    """The record or the file may not be used (nothing was written)."""


class RecordRefused(SessionError):
    """The state of the arm may not be recorded (not still, or the elevation is too low)."""


# ----------------------------------------------------------------------------- the identity

def sha256_file(path):
    """Return the sha256 (hex) of a file."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_zero_offsets(calibration_yaml):
    """
    Return {joint: degrees} of zero_offset_deg for the five angle joints (0 where there is none).

    The checks are those of the driver (calibration.cpp): the value is a finite number within
    +-ZERO_OFFSET_MAX_DEG, and the gripper may have none other than 0.  ValueError otherwise.
    """
    try:
        with open(calibration_yaml, encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'cannot read {calibration_yaml}: {exc}') from exc
    joints = data.get('joints') if isinstance(data, dict) else None
    if not isinstance(joints, dict):
        raise ValueError(f'{calibration_yaml}: no joints')
    for name, entry in joints.items():
        value = entry.get('zero_offset_deg') if isinstance(entry, dict) else None
        if name not in ARM_JOINTS and value not in (None, 0, 0.0):
            raise ValueError(f'{calibration_yaml}: {name} may not have a zero_offset_deg')
    offsets = {}
    for name in ARM_JOINTS:
        entry = joints.get(name)
        if not isinstance(entry, dict) or entry.get('kind') != 'angle':
            raise ValueError(f'{calibration_yaml}: {name} is not an angle joint')
        value = entry.get('zero_offset_deg', 0.0)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or abs(value) > ZERO_OFFSET_MAX_DEG):
            raise ValueError(f'{calibration_yaml}: {name} zero_offset_deg must be a finite '
                             f'number within +-{ZERO_OFFSET_MAX_DEG} deg')
        offsets[name] = float(value)
    return offsets


def make_identity(calibration_sha256, zero_offset_deg, urdf_sha256, mesh_sha256):
    """Return the values that must not change between the records of one CSV."""
    return {'calibration_kind': zo.CALIBRATION_KIND,
            'follower_calibration_sha256': calibration_sha256,
            'zero_offset_deg': dict(zero_offset_deg), 'urdf_sha256': urdf_sha256,
            'mesh_sha256': dict(mesh_sha256)}


def identity_from_files(calibration_yaml, urdf_path, mesh_dir):
    """Return the identity of a session from the files that were used."""
    urdf_xml = Path(urdf_path).read_text(encoding='utf-8')
    meshes = {name: sha256_file(Path(mesh_dir) / name)
              for name in sorted({n for n, _, _ in zo.link_visuals(urdf_xml)})}
    return make_identity(sha256_file(calibration_yaml), read_zero_offsets(calibration_yaml),
                         sha256_file(urdf_path), meshes)


def identity_mismatches(recorded, current):
    """Return one line for each identity value that differs (or is missing) in the meta file."""
    lines = []
    for key in IDENTITY_KEYS:
        if not isinstance(recorded, dict) or key not in recorded:
            lines.append(f'{key}: missing in the meta file')
        elif recorded[key] != current.get(key):
            lines.append(f'{key}: recorded {recorded[key]}, now {current.get(key)}')
    return lines


# ----------------------------------------------------------------------------- one record

def meta_path_for(csv_path):
    """Return the path of the meta file that belongs to a CSV file."""
    return Path(str(csv_path) + '.meta.yaml')


def _num(value, digits=6):
    return f'{float(value):.{digits}f}'


def window_stats(samples):
    """Return (mean of the six joints in rad, span of each in deg) of the samples (n, 6)."""
    values = np.asarray(samples, dtype=float)
    return values.mean(axis=0), np.degrees(values.max(axis=0) - values.min(axis=0))


def is_still(span_deg):
    """Return True when no arm joint moved more than STILL_DEG over the samples."""
    return bool(np.max(np.asarray(span_deg, dtype=float)[:len(ARM_JOINTS)]) <= STILL_DEG)


def target_delta(pose, x_mm, eps_deg):
    """Return (delta x mm, delta elevation deg, inside the range) against the target of a pose."""
    eps_target, x_target = POSES[pose]
    dx, de = x_mm - x_target, eps_deg - eps_target
    return dx, de, abs(dx) <= TARGET_X_TOL_MM and abs(de) <= TARGET_EPS_TOL_DEG


def next_label(existing_labels, repeats=REPEATS):
    """Return the next label (P1#1, P1#2, P2#1, ...) that is not in existing_labels, or None."""
    for pose in POSES:
        for number in range(1, repeats + 1):
            label = f'{pose}#{number}'
            if label not in existing_labels:
                return label
    return None


def parse_command(line):
    """Return (command, argument) for a typed line: record, quit, target, confirm or unknown."""
    text = line.strip().lower()
    if text == '':
        return 'record', None
    if text in ('q', 'quit'):
        return 'quit', None
    if text in ('y', 'yes'):
        return 'confirm', None
    if text.upper() in POSES:
        return 'target', text.upper()
    return 'unknown', text


def make_row(label, time_s, q_mean, span_deg, samples, chain, jaw, options=zo.Options()):
    """
    Return the row of one record, or raise RecordRefused (nothing is recorded).

    q_mean is the mean state of the six joints (the five arm joints and the gripper, rad).
    """
    if not isinstance(label, str) or not LABEL_PATTERN.fullmatch(label):
        raise RecordRefused('the label may only have letters, digits, "_", ".", "-" and "#n"')
    q_mean = np.asarray(q_mean, dtype=float)
    span_deg = np.asarray(span_deg, dtype=float)
    if not is_still(span_deg):
        raise RecordRefused(
            f'the arm is not still (largest span {np.max(span_deg[:len(ARM_JOINTS)]):.2f} deg, '
            f'at most {STILL_DEG:g}): nothing was recorded')
    x_mm, eps = zo.tool_state(chain, q_mean[:len(ARM_JOINTS)])
    if eps < zo.EPS_MIN_DEG:
        raise RecordRefused(f'the elevation {eps:.1f} deg is below {zo.EPS_MIN_DEG:g} deg: '
                            'nothing was recorded')
    height = zo.height_above_desk_mm(chain, jaw, q_mean[:len(ARM_JOINTS)], options)
    warnings = []
    if abs(height) > zo.DESK_WARN_MM:
        warnings.append(f'height {height:.0f} mm from the desk (in the air?)')
    row = {name: '' for name in COLUMNS}
    row.update(label=label, pose=label.split('#')[0], time=_num(time_s, 3))
    for name, q, span in zip(STATE_JOINTS, q_mean, span_deg):
        row['q_' + name] = _num(q)
        row['span_deg_' + name] = _num(span, 3)
    row.update(samples=str(int(samples)), x_mm=_num(x_mm, 2), eps_deg=_num(eps, 2),
               model_height_mm=_num(height, 2), still='1', warn='; '.join(warnings))
    return row


# ----------------------------------------------------------------------------- the files

def build_meta(identity, options, created, extra=None):
    """Return the content of the meta file (the values that explain where the CSV comes from)."""
    return {'schema': 1, 'calibration_kind': zo.CALIBRATION_KIND, 'created': created,
            'tool': TOOL_VERSION, 'identity': to_plain(identity),
            'settings': {'h_prior_mm': options.h_prior_mm, 'h_sigma_mm': options.h_sigma_mm,
                         'sigma_contact_mm': options.sigma_contact_mm,
                         'sigma_90_extra_mm': options.sigma_90_extra_mm,
                         'eps_min_deg': zo.EPS_MIN_DEG, 'still_deg': STILL_DEG,
                         'samples': SAMPLES},
            'extra': to_plain(extra or {})}


def to_plain(value):
    """Return value with numpy numbers and tuples turned into plain Python for yaml."""
    if isinstance(value, dict):
        return {str(k): to_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [to_plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def read_rows(csv_path):
    """Return (header, rows) of a CSV; lines that start with '#' are comments and are skipped."""
    with open(csv_path, newline='', encoding='utf-8') as f:
        lines = [line for line in f.read().splitlines() if not line.startswith('#')]
    rows = list(csv.reader(lines))
    if not rows:
        raise SessionError(f'{csv_path} is empty (no header)')
    return rows[0], rows[1:]


def read_meta(csv_path, meta_path=None):
    """Return the meta of a CSV; SessionError when it is missing or is not a zero-offset one."""
    meta_path = Path(meta_path) if meta_path else meta_path_for(csv_path)
    try:
        with open(meta_path, encoding='utf-8') as f:
            meta = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as exc:
        raise SessionError(f'cannot read {meta_path}: {exc}') from exc
    if not isinstance(meta, dict) or meta.get('calibration_kind') != zo.CALIBRATION_KIND:
        raise SessionError(f'{meta_path} is not the meta file of a zero-offset calibration '
                           f'(calibration_kind is not {zo.CALIBRATION_KIND})')
    return meta


def check_file(csv_path, identity, meta_path=None):
    """
    Check the CSV and its meta file against the identity.  Return 'new' or 'append'.

    new: neither the CSV nor the meta file exists.  append: both exist, the first line and the
    header are the present ones and the identity of the meta file is the present one.  Anything
    else raises SessionError; nothing is written.
    """
    csv_path = Path(csv_path)
    meta_path = Path(meta_path) if meta_path else meta_path_for(csv_path)
    has_csv, has_meta = csv_path.exists(), meta_path.exists()
    if not has_csv and not has_meta:
        return 'new'
    if not has_csv:
        raise SessionError(
            f'{meta_path} exists but {csv_path} does not: a leftover of an older run')
    if not has_meta:
        raise SessionError(f'{csv_path} exists but {meta_path} does not')
    meta = read_meta(csv_path, meta_path)
    differences = identity_mismatches(meta.get('identity'), identity)
    if differences:
        raise SessionError(
            'the meta file differs from the present files: ' + '; '.join(differences))
    with open(csv_path, encoding='utf-8') as f:
        first = f.readline().rstrip('\n')
    if first != CSV_COMMENT:
        raise SessionError(f'{csv_path} does not start with the zero-offset comment line')
    header, _ = read_rows(csv_path)
    if header != list(COLUMNS):
        raise SessionError(f'the header of {csv_path} is not the present one')
    return 'append'


def check_session(csv_path, label, identity):
    """Check that the record label may be written to csv_path.  Return 'new' or 'append'."""
    if not isinstance(label, str) or not LABEL_PATTERN.fullmatch(label):
        raise SessionError('the label may only have letters, digits, "_", ".", "-" and "#n"')
    state = check_file(csv_path, identity)
    if state == 'append':
        _, rows = read_rows(csv_path)
        if label in [r[COLUMNS.index('label')] for r in rows if len(r) == len(COLUMNS)]:
            raise SessionError(f'the label {label} is already in {csv_path} (use another label)')
    return state


def write_point(csv_path, label, identity, meta, row):
    """
    Write one record: create the meta file and the CSV (mode 'x') or append a row (mode 'a+').

    The checks of check_session are made again inside the write (under a lock for appending); the
    rows that are already in the CSV are never rewritten.  Returns the idx of the row.
    """
    csv_path = Path(csv_path)
    meta_path = meta_path_for(csv_path)
    state = check_session(csv_path, label, identity)
    if state == 'new':
        try:
            with open(meta_path, 'x', encoding='utf-8') as f:
                yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)
            with open(csv_path, 'x', newline='', encoding='utf-8') as f:
                f.write(CSV_COMMENT + '\n')
                writer = csv.DictWriter(f, COLUMNS, lineterminator='\n')
                writer.writeheader()
                writer.writerow(dict(row, idx='1'))
                f.flush()
                os.fsync(f.fileno())
        except FileExistsError as exc:
            raise SessionError(f'{exc.filename} was created by someone else') from exc
        return 1
    with open(csv_path, 'a+', newline='', encoding='utf-8') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        try:
            if check_session(csv_path, label, identity) != 'append':
                raise SessionError(f'{csv_path} changed while it was checked')
            idx = len(read_rows(csv_path)[1]) + 1
            writer = csv.DictWriter(f, COLUMNS, lineterminator='\n')
            writer.writerow(dict(row, idx=str(idx)))
            f.flush()
            os.fsync(f.fileno())
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)
    return idx


def read_touches(csv_path):
    """Return the records of a CSV as zo.Touch (state angles of the five arm joints)."""
    header, rows = read_rows(csv_path)
    if header != list(COLUMNS):
        raise SessionError(f'the header of {csv_path} is not the present one')
    touches = []
    for row in rows:
        if len(row) != len(COLUMNS):
            raise SessionError(f'{csv_path}: a row has {len(row)} columns, not {len(COLUMNS)}')
        values = dict(zip(COLUMNS, row))
        touches.append(zo.Touch(values['label'],
                                tuple(float(values['q_' + n]) for n in ARM_JOINTS)))
    return touches


def existing_labels(csv_path):
    """Return the labels already in a CSV (an empty tuple when there is no CSV yet)."""
    if not Path(csv_path).exists():
        return ()
    _, rows = read_rows(csv_path)
    return tuple(r[COLUMNS.index('label')] for r in rows if len(r) == len(COLUMNS))
