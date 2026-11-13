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
Pure helpers of tools/ik_model_check.py (numpy and the standard library only, no ROS).

* the poses of the MoveIt FK fixture and its CSV file (test/data/moveit_fk.csv),
* the orientation of the approach axis, to compare the pitch of the model with a MoveIt pose,
* the rows, the summary and the table of the check (values in mm).
"""

import csv
import hashlib
import math
from typing import NamedTuple

import numpy as np

from . import analytic_ik

FIXTURE_SEED = 20261113
FIXTURE_RANDOM_COUNT = 100
FIXTURE_FORMAT = 'so_arm101_ik moveit_fk fixture v1'
FIXTURE_FRAME = 'base_link'
FIXTURE_LINK = 'gripper_frame_link'
FIXTURE_COLUMNS = ('name', 'q1', 'q2', 'q3', 'q4', 'q5', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')

# Named poses [pan, lift, elbow, wrist_flex, roll] (rad). 'start (W1)' is the measured start pose
# of the real arm (W1, 2026-11-13 07:54), not the old one of the C3 record.
NAMED_POSES = {
    'zero': (0.0, 0.0, 0.0, 0.0, 0.0),
    'start (W1)': (-0.0951, -1.4952, 1.6901, 0.4219, -0.0453),
    'T (W1)': (0.0031, -0.3813, 0.5501, 1.4454, -0.0069),
    'home': (0.0, -1.57, 1.57, 0.8, 0.0),
    'stretched': (0.0, 0.3, -0.3, 0.2, 0.0),
}

# Tolerances of the comparison with MoveIt (mm, rad).
CHAIN_TOLERANCE_MM = 0.001  # chain of the URDF vs MoveIt (the same numbers in double precision)
MODEL_TOLERANCE_MM = 0.01  # closed-form model vs MoveIt (as the model vs the chain)
ROTATION_TOLERANCE_RAD = 1e-8  # chain vs MoveIt (quaternion of doubles)
PITCH_TOLERANCE_RAD = 4e-5  # model pitch vs the MoveIt pose (measured max 1.3e-5: URDF rounding)


def fixture_poses(lower, upper, seed=FIXTURE_SEED, count=FIXTURE_RANDOM_COUNT):
    """Return [(name, q)]: the named poses, then count random poses inside [lower, upper]."""
    poses = [(name, np.array(q, dtype=float)) for name, q in NAMED_POSES.items()]
    rng = np.random.default_rng(seed)
    for index in range(count):
        poses.append((f'random-{index + 1:03d}', rng.uniform(lower, upper)))
    return poses


def urdf_sha256(data):
    """Return the sha256 (hex) of the URDF given as str or bytes."""
    if isinstance(data, str):
        data = data.encode()
    return hashlib.sha256(data).hexdigest()


def quaternion_to_matrix(quaternion):
    """Return the rotation matrix of the quaternion (x, y, z, w); it is normalised first."""
    x, y, z, w = np.asarray(quaternion, dtype=float) / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quaternion(rotation):
    """Return the quaternion (x, y, z, w, with w >= 0) of the rotation matrix."""
    r = np.asarray(rotation, dtype=float)
    trace = np.trace(r)
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4.0]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(r[i, i] - r[j, j] - r[k, k] + 1.0)
        q = [0.0, 0.0, 0.0, (r[k, j] - r[j, k]) / s]
        q[i] = s / 4.0
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
    q = np.array(q)
    return q if q[3] >= 0.0 else -q


def rotation_angle(first, second):
    """Return the angle (rad, 0..pi) of the rotation from one rotation matrix to the other."""
    relative = np.asarray(first).T @ np.asarray(second)
    vee = 0.5 * np.array([relative[2, 1] - relative[1, 2], relative[0, 2] - relative[2, 0],
                          relative[1, 0] - relative[0, 1]])
    return math.atan2(float(np.linalg.norm(vee)), 0.5 * (float(np.trace(relative)) - 1.0))


def approach_axis_in_tip(rotation_at_zero, d4=analytic_ik.CONSTANTS.d4):
    """
    Return the approach axis in the frame of the tip link.

    At q = 0 the pan angle is 0 and the approach axis lies in the x-z plane of base_link at the
    pitch d4, so its coordinates in the tip frame follow from the tip rotation at q = 0 (taken
    from the chain of the URDF, or from the 'zero' pose of the MoveIt fixture).
    """
    axis = np.array([math.cos(d4), 0.0, math.sin(d4)])
    return np.asarray(rotation_at_zero).T @ axis


def approach_pitch(rotation, q_pan, axis_in_tip):
    """Return the pitch (rad) of the approach axis in the pan plane, for a tip rotation."""
    axis = np.asarray(rotation) @ axis_in_tip
    theta = analytic_ik.S1 * q_pan
    radial = axis[0] * math.cos(theta) + axis[1] * math.sin(theta)
    return math.atan2(axis[2], radial)


class FixtureRow(NamedTuple):
    """One pose of the fixture: joint angles (rad) and the MoveIt FK of the tip (m, x y z w)."""

    name: str
    q: np.ndarray
    position: np.ndarray
    quaternion: np.ndarray


def write_fixture(path, meta, rows):
    """Write the fixture: '# key: value' lines, then a CSV table (floats in full precision)."""
    with open(path, 'w', newline='') as stream:
        stream.write(f'# {FIXTURE_FORMAT}\n')
        for key, value in meta.items():
            if '\n' in str(value):
                raise ValueError(f'meta {key} has a line break')
            stream.write(f'# {key}: {value}\n')
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(FIXTURE_COLUMNS)
        for row in rows:
            values = [*row.q, *row.position, *row.quaternion]
            writer.writerow([row.name, *(repr(float(v)) for v in values)])


def read_fixture(path):
    """Return (meta dict, [FixtureRow]) of a fixture file written by write_fixture()."""
    meta, table = {}, []
    with open(path, newline='') as stream:
        lines = stream.read().splitlines()
    if not lines or lines[0] != f'# {FIXTURE_FORMAT}':
        raise ValueError(f'{path} is not a "{FIXTURE_FORMAT}" file')
    for line in lines[1:]:
        if line.startswith('# '):
            key, _, value = line[2:].partition(': ')
            meta[key] = value
        elif line:
            table.append(line)
    reader = csv.reader(table)
    if tuple(next(reader)) != FIXTURE_COLUMNS:
        raise ValueError('unexpected columns of the fixture')
    rows = []
    for fields in reader:
        values = [float(v) for v in fields[1:]]
        rows.append(FixtureRow(fields[0], np.array(values[0:5]), np.array(values[5:8]),
                               np.array(values[8:12])))
    return meta, rows


def fixture_is_stale(meta, urdf_text):
    """Return None if the fixture was recorded with this URDF, else a message of what to do."""
    actual = urdf_sha256(urdf_text)
    if meta.get('urdf_sha256') == actual:
        return None
    return (f'the fixture is stale: urdf_sha256 is {meta.get("urdf_sha256")} but the installed '
            f'URDF is {actual}. Record it again (mock launch, see '
            f'docs/analytic_diff_plan.md 7.11): '
            f'python3 tools/ik_model_check.py --record-fixture '
            f'src/so_arm101_ik/test/data/moveit_fk.csv --overwrite')


class CaseRow(NamedTuple):
    """
    One line of the table (positions in m; None where there is no value).

    model is the analytic FK of q (or the target of an IK row), moveit the FK of MoveIt for q,
    chain the FK of the URDF chain. pitch_deg and orient_deg are the differences of the pitch
    of the approach axis (model - MoveIt) and of the orientation (chain - MoveIt).
    """

    case: str
    q: object = None
    model: object = None
    moveit: object = None
    chain: object = None
    pitch_deg: object = None
    orient_deg: object = None
    tolerance_mm: float = 1.0
    note: str = ''


def difference_mm(row):
    """Return (model - MoveIt) in mm as a vector, or None when the row has no numbers."""
    if row.model is None or row.moveit is None:
        return None
    return (np.asarray(row.model) - np.asarray(row.moveit)) * 1000.0


def chain_difference_mm(row):
    """Return |chain - MoveIt| in mm, or None."""
    if row.chain is None or row.moveit is None:
        return None
    return float(np.linalg.norm((np.asarray(row.chain) - np.asarray(row.moveit)) * 1000.0))


def is_over(row):
    """Return True if |model - MoveIt| of the row exceeds its tolerance."""
    diff = difference_mm(row)
    return diff is not None and float(np.linalg.norm(diff)) > row.tolerance_mm


def summarize(rows):
    """Return (max |d| in mm, case of the max, mean |d| in mm, rows over the tolerance, failed)."""
    norms = [(float(np.linalg.norm(difference_mm(r))), r.case)
             for r in rows if difference_mm(r) is not None]
    failed = sum(1 for r in rows if difference_mm(r) is None)
    if not norms:
        return 0.0, None, 0.0, 0, failed
    worst = max(norms)
    mean = sum(n for n, _ in norms) / len(norms)
    return worst[0], worst[1], mean, sum(1 for r in rows if is_over(r)), failed


_HEADER = ('case', 'q [rad]', 'model/target [mm]', 'MoveIt FK [mm]', 'd [mm]', '|d|',
           'chain-MoveIt', 'dpitch', 'orient')


def _vector(values, width, digits):
    return ' '.join(f'{v:{width}.{digits}f}' for v in values)


def table_lines(rows):
    """Return the table as a list of lines (header, one line per row, summary)."""
    q_width = 5 * 7 + 4
    xyz_width = 3 * 9 + 2
    header = (f'{_HEADER[0]:<16} {_HEADER[1]:<{q_width}}  {_HEADER[2]:<{xyz_width}}  '
              f'{_HEADER[3]:<{xyz_width}}  {_HEADER[4]:<{3 * 8 + 2}}  {_HEADER[5]:>7} '
              f'{_HEADER[6]:>12} {_HEADER[7]:>8} {_HEADER[8]:>8}')
    lines = [header]
    for row in rows:
        q = _vector(row.q, 7, 3) if row.q is not None else ''
        diff = difference_mm(row)
        if diff is None:
            lines.append(f'{row.case:<16} {q:<{q_width}}  {row.note}')
            continue
        chain = chain_difference_mm(row)
        flag = ' *' if is_over(row) else ''
        lines.append(
            f'{row.case:<16} {q:<{q_width}}  '
            f'{_vector(np.asarray(row.model) * 1000.0, 9, 3)}  '
            f'{_vector(np.asarray(row.moveit) * 1000.0, 9, 3)}  '
            f'{_vector(diff, 8, 4)}  {np.linalg.norm(diff):7.4f} '
            f'{"" if chain is None else f"{chain:12.5f}"}'
            f'{"" if chain is None else " "}'
            f'{"" if row.pitch_deg is None else f"{row.pitch_deg:8.4f}"}'
            f'{"" if row.orient_deg is None else f" {row.orient_deg:8.5f}"}{flag}')
    worst, where, mean, over, failed = summarize(rows)
    lines.append(f'summary: max |d| = {worst:.4f} mm (case {where}), mean = {mean:.4f} mm, '
                 f'rows over the tolerance (marked *): {over}, rows without a result: {failed}')
    return lines


def table_csv_rows(rows):
    """Return the rows of the table as lists (for csv.writer), SI units converted to mm."""
    out = [['case', 'q1', 'q2', 'q3', 'q4', 'q5', 'model_x', 'model_y', 'model_z', 'moveit_x',
            'moveit_y', 'moveit_z', 'dx', 'dy', 'dz', 'norm', 'chain_vs_moveit', 'dpitch_deg',
            'orient_deg', 'tolerance_mm', 'note']]
    for row in rows:
        diff = difference_mm(row)
        q = [] if row.q is None else [f'{v:.9f}' for v in row.q]
        q += [''] * (5 - len(q))
        model = [''] * 3 if row.model is None else [f'{v * 1000.0:.6f}' for v in row.model]
        moveit = [''] * 3 if row.moveit is None else [f'{v * 1000.0:.6f}' for v in row.moveit]
        delta = [''] * 3 if diff is None else [f'{v:.6f}' for v in diff]
        norm = '' if diff is None else f'{np.linalg.norm(diff):.6f}'
        chain = chain_difference_mm(row)
        out.append([row.case, *q, *model, *moveit, *delta, norm,
                    '' if chain is None else f'{chain:.6f}',
                    '' if row.pitch_deg is None else f'{row.pitch_deg:.6f}',
                    '' if row.orient_deg is None else f'{row.orient_deg:.6f}',
                    row.tolerance_mm, row.note])
    return out
