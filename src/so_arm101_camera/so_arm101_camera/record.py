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
The recording logic of C2: match the detections with the tag poses, average, check, write a row.

Pure code: no ROS, no clock of its own and no input of its own. Everything that touches the world
(time, waiting, /joint_states, /tf, /detections, the keyboard) is an io object, so the logic can
be tested with a fake one. Nothing here sends anything to the arm, opens a serial port or opens
the camera. The files it writes are the CSV and its meta file, and only with the modes 'x' and 'a'.

Values of the settling (STOP_TOL and so on) are imported from so_arm101_ik, not copied.
"""

import csv
import dataclasses
import fcntl
import math
import os
from pathlib import Path
import re

import numpy as np
from so_arm101_ik import step_runner as sr
from so_arm101_ik.urdf_kinematics import UrdfChain
import yaml

from . import tag_mount as tm

# Values of this module. None of them is measured: every one is a guess (推測) until C2 is run.
FRAMES_WANTED = 15  # frames averaged per point (推測; the old collect_cal.py used 10)
MATCH_MIN = 8  # matched frames needed to call a tag seen (推測)
COLLECT_TIMEOUT = 10.0  # s, give up collecting frames (推測)
POLL_SECONDS = 0.1  # s, one step of the collecting loop (推測)
OUTLIER_MAD_FACTOR = 3.0  # times the MAD of an axis (推測)
OUTLIER_FLOOR_M = 0.005  # m, the lower limit of the outlier threshold (推測: C1-2d, 6 mm range)
STABLE_TOL_M = 0.0003  # m, first half against second half; recorded only (推測: collect_cal.py)

EXIT_OK = 0
EXIT_REFUSED = 2  # the file or session checks refused to start (nothing was written)
EXIT_NOT_SEEN = 3  # the tag was not seen: stop and ask
EXIT_NOT_CONFIRMED = EXIT_REFUSED  # the answer was not yes: 2 as in ik_move (EXIT_STOPPED)
EXIT_NO_INPUT = 5  # no fresh /joint_states

LABEL_PATTERN = re.compile(r'[A-Za-z0-9_.-]+')  # used with fullmatch (a "$" would pass "P01\n")
IDENTITY_KEYS = ('tag_mount_sha256', 'urdf_sha256', 'apriltag_yaml_sha256',
                 'realsense_yaml_sha256', 'tag_id', 'follower_calibration_sha256',
                 'zero_offset_deg')
ZERO_OFFSET_MAX_DEG = 10.0  # the same upper limit as the driver (kMaxZeroOffsetDeg)

COLUMNS = (
    ('idx', 'time', 'label')
    + tuple('q_' + name for name in sr.JOINT_NAMES)
    + ('q_span', 'fk_tcp_x', 'fk_tcp_y', 'fk_tcp_z', 'fk_tag_x', 'fk_tag_y', 'fk_tag_z',
       'det_n', 'tf_n', 'matched_n', 'unmatched_det', 'unmatched_tf', 'invalid_n',
       'cam_dropped', 'cam_n',
       'cam_tag_x', 'cam_tag_y', 'cam_tag_z', 'cam_std_x', 'cam_std_y', 'cam_std_z',
       'cam_ptp_x', 'cam_ptp_y', 'cam_ptp_z', 'cam_normal_x', 'cam_normal_y', 'cam_normal_z',
       'cam_margin_min', 'cam_margin_mean', 'incidence_deg', 'tag_seen', 'still', 'stable'))


class SessionError(RuntimeError):
    """The CSV or meta file is not in a state in which a point may be recorded."""


# ----------------------------------------------------------------------------- matching

def tag_normal(quaternion):
    """Return the z axis of a frame given by the quaternion (x, y, z, w): the tag normal."""
    x, y, z, w = np.asarray(quaternion, dtype=float) / np.linalg.norm(quaternion)
    return np.array([2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)])


def _finite(values):
    array = np.asarray(values, dtype=float)
    return bool(np.all(np.isfinite(array)))


@dataclasses.dataclass(frozen=True)
class MatchResult:
    """Frames whose detection and tag pose have the same stamp, and the counts of the rest."""

    det_n: int
    tf_n: int
    matched: tuple  # (stamp, margin, position (3), normal (3)) in the order of the stamps
    unmatched_det: int
    unmatched_tf: int
    invalid_n: int


def match_frames(detections, transforms):
    """
    Match detections {stamp: decision_margin} with tag poses {stamp: (xyz, quaternion xyzw)}.

    Only an equal stamp matches (no interpolation, no tolerance); a frame with a number that is
    not finite (or a zero quaternion) is counted in invalid_n and not used.
    """
    common = sorted(set(detections) & set(transforms))
    matched = []
    invalid = 0
    for stamp in common:
        margin = detections[stamp]
        translation, quaternion = transforms[stamp]
        if (not _finite([margin]) or not _finite(translation) or not _finite(quaternion)
                or np.linalg.norm(np.asarray(quaternion, dtype=float)) < 1e-9):
            invalid += 1
            continue
        matched.append((stamp, float(margin), np.asarray(translation, dtype=float),
                        tag_normal(quaternion)))
    return MatchResult(len(detections), len(transforms), tuple(matched),
                       len(detections) - len(common), len(transforms) - len(common), invalid)


def is_seen(match):
    """Return True if enough frames matched (MATCH_MIN). The viewing angle is not used."""
    return len(match.matched) >= MATCH_MIN


def keep_mask(positions, factor=OUTLIER_MAD_FACTOR, floor=OUTLIER_FLOOR_M):
    """
    Return a boolean array: True for the frames that are not outliers.

    On each axis the limit is max(factor times the MAD, floor) around the median; a frame is kept
    if it is inside the limit on all three axes.
    """
    points = np.asarray(positions, dtype=float).reshape(-1, 3)
    if len(points) == 0:
        return np.zeros(0, dtype=bool)
    median = np.median(points, axis=0)
    spread = np.median(np.abs(points - median), axis=0)
    limit = np.maximum(factor * spread, floor)
    return np.all(np.abs(points - median) <= limit, axis=1)


@dataclasses.dataclass(frozen=True)
class Window:
    """The average of the kept frames of one point (positions in the optical frame, m)."""

    position: np.ndarray
    std: np.ndarray
    ptp: np.ndarray
    n: int
    dropped: int
    normal: np.ndarray
    margin_min: float
    margin_mean: float
    stable: bool


def average_matches(match):
    """Return the Window of the matched frames without the outliers, or None if none is left."""
    if not match.matched:
        return None
    positions = np.array([m[2] for m in match.matched])
    keep = keep_mask(positions)
    kept = [m for m, k in zip(match.matched, keep) if k]
    if not kept:
        return None
    points = np.array([m[2] for m in kept])
    n = len(kept)
    normal = np.mean([m[3] for m in kept], axis=0)
    length = float(np.linalg.norm(normal))
    normal = normal / length if length > 1e-9 else np.zeros(3)
    margins = np.array([m[1] for m in kept])
    stable = False
    if n >= 2:
        first = np.mean(points[:n // 2], axis=0)
        second = np.mean(points[(n + 1) // 2:], axis=0)
        stable = bool(np.linalg.norm(first - second) <= STABLE_TOL_M)
    return Window(np.mean(points, axis=0), np.std(points, axis=0, ddof=1 if n > 1 else 0),
                  np.ptp(points, axis=0), n, len(match.matched) - n, normal,
                  float(np.min(margins)), float(np.mean(margins)), stable)


def incidence_deg(position, normal):
    """
    Return the angle [deg] between the tag normal and the line of sight to the camera.

    The camera is at the origin of the optical frame, so the line of sight is -position. The sign
    of the normal is not assumed (the absolute value of the dot product is used).
    """
    distance = float(np.linalg.norm(position))
    length = float(np.linalg.norm(normal))
    if distance < 1e-9 or length < 1e-9:
        return float('nan')
    cosine = abs(float(np.dot(position, normal))) / (distance * length)
    return math.degrees(math.acos(min(1.0, cosine)))


def investigation_lines(match, window, still):
    """Return the lines shown for a point: counts, rate, margin, angle, scatter, decision."""
    rate = len(match.matched) / match.det_n if match.det_n else 0.0
    lines = [
        f'frames: detections {match.det_n}, tag poses {match.tf_n}, matched {len(match.matched)},'
        f' unmatched detections {match.unmatched_det}, unmatched poses {match.unmatched_tf},'
        f' invalid {match.invalid_n}',
        f'detection rate (matched / detections): {rate:.2f}',
        f'arm stopped: {"yes" if still else "NO"}',
    ]
    if window is not None:
        lines += [
            f'frames used {window.n} (dropped as outliers {window.dropped}), '
            f'decision_margin min {window.margin_min:.1f} mean {window.margin_mean:.1f}',
            f'incidence {incidence_deg(window.position, window.normal):.1f} deg (shown only)',
            'std (mm) ' + ' '.join(f'{1000.0 * v:.2f}' for v in window.std)
            + ', ptp (mm) ' + ' '.join(f'{1000.0 * v:.2f}' for v in window.ptp),
            f'first half against second half: {"within" if window.stable else "OVER"} '
            f'{1000.0 * STABLE_TOL_M:.1f} mm (recorded only)',
        ]
    lines.append(f'tag seen: {"yes" if is_seen(match) else "NO"} '
                 f'(matched frames {len(match.matched)}, at least {MATCH_MIN} needed)')
    return lines


# ----------------------------------------------------------------------------- the CSV row

def _num(value, digits=6):
    return f'{float(value):.{digits}f}'


def build_row(label, time_s, q_mean, q_span, fk_tcp, fk_tag, match, window, still):
    """Return the row (column to text, without idx); the cam_ columns are empty if unseen."""
    seen = is_seen(match) and window is not None
    row = {name: '' for name in COLUMNS}
    row['time'] = _num(time_s, 3)
    row['label'] = label
    for name, value in zip(sr.JOINT_NAMES, q_mean):
        row['q_' + name] = _num(value)
    row['q_span'] = _num(q_span)
    for axis, value in zip('xyz', fk_tcp):
        row['fk_tcp_' + axis] = _num(value)
    for axis, value in zip('xyz', fk_tag):
        row['fk_tag_' + axis] = _num(value)
    row.update(det_n=str(match.det_n), tf_n=str(match.tf_n), matched_n=str(len(match.matched)),
               unmatched_det=str(match.unmatched_det), unmatched_tf=str(match.unmatched_tf),
               invalid_n=str(match.invalid_n), tag_seen='1' if seen else '0',
               still='1' if still else '0')
    if seen:
        row.update(cam_dropped=str(window.dropped), cam_n=str(window.n),
                   cam_margin_min=_num(window.margin_min, 3),
                   cam_margin_mean=_num(window.margin_mean, 3),
                   incidence_deg=_num(incidence_deg(window.position, window.normal), 2),
                   stable='1' if window.stable else '0')
        for axis, pos, std, ptp, normal in zip('xyz', window.position, window.std, window.ptp,
                                               window.normal):
            row['cam_tag_' + axis] = _num(pos)
            row['cam_std_' + axis] = _num(std)
            row['cam_ptp_' + axis] = _num(ptp)
            row['cam_normal_' + axis] = _num(normal)
    return row


# ----------------------------------------------------------------------------- the session

def meta_path_for(csv_path):
    """Return the path of the meta file that belongs to a CSV file."""
    return Path(str(csv_path) + '.meta.yaml')


def make_identity(tag_mount_sha256, urdf_sha256, apriltag_yaml_sha256, realsense_yaml_sha256,
                  tag_id, follower_calibration_sha256, zero_offset_deg):
    """Return the values that must not change between the points of one CSV."""
    return {'tag_mount_sha256': tag_mount_sha256, 'urdf_sha256': urdf_sha256,
            'apriltag_yaml_sha256': apriltag_yaml_sha256,
            'realsense_yaml_sha256': realsense_yaml_sha256, 'tag_id': tag_id,
            'follower_calibration_sha256': follower_calibration_sha256,
            'zero_offset_deg': dict(zero_offset_deg)}


def read_zero_offsets(calibration_yaml):
    """
    Return {joint: degrees} of zero_offset_deg for the five angle joints (0 where there is none).

    The checks are those of the driver (calibration.cpp): the value is a finite number within
    +-ZERO_OFFSET_MAX_DEG, and the gripper may have none other than 0. ValueError otherwise.
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
        if name not in sr.JOINT_NAMES and value not in (None, 0, 0.0):
            raise ValueError(f'{calibration_yaml}: {name} may not have a zero_offset_deg')
    offsets = {}
    for name in sr.JOINT_NAMES:
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


def identity_mismatches(recorded, current):
    """Return one line for each identity value that differs (or is missing) in the meta file."""
    lines = []
    for key in IDENTITY_KEYS:
        if not isinstance(recorded, dict) or key not in recorded:
            lines.append(f'{key}: missing in the meta file')
        elif recorded[key] != current.get(key):
            lines.append(f'{key}: recorded {recorded[key]}, now {current.get(key)}')
    return lines


def _read_csv(csv_path):
    with open(csv_path, newline='', encoding='utf-8') as f:
        rows = list(csv.reader(f))
    if not rows:
        raise SessionError(f'{csv_path} is empty (no header)')
    return rows[0], rows[1:]


def check_session(csv_path, label, identity):
    """
    Check that the point label may be recorded in csv_path. Return 'new' or 'append'.

    new: neither the CSV nor the meta file exists. append: both exist, the header is the current
    one, the identity of the meta file is the current one and the label is not used yet. Anything
    else raises SessionError; nothing is written.
    """
    if not isinstance(label, str) or not LABEL_PATTERN.fullmatch(label):
        raise SessionError('the label may only have letters, digits, "_", "." and "-"')
    csv_path = Path(csv_path)
    meta_path = meta_path_for(csv_path)
    has_csv, has_meta = csv_path.exists(), meta_path.exists()
    if not has_csv and not has_meta:
        return 'new'
    if not has_csv:
        raise SessionError(
            f'{meta_path} exists but {csv_path} does not: a leftover of an older run')
    if not has_meta:
        raise SessionError(f'{csv_path} exists but {meta_path} does not')
    try:
        with open(meta_path, encoding='utf-8') as f:
            meta = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as exc:
        raise SessionError(f'cannot read {meta_path}: {exc}') from exc
    recorded = meta.get('identity') if isinstance(meta, dict) else None
    differences = identity_mismatches(recorded, identity)
    if differences:
        raise SessionError(
            'the meta file differs from the present files: ' + '; '.join(differences))
    header, rows = _read_csv(csv_path)
    if header != list(COLUMNS):
        raise SessionError(f'the header of {csv_path} is not the present one')
    if label in [r[COLUMNS.index('label')] for r in rows if len(r) == len(COLUMNS)]:
        raise SessionError(f'the label {label} is already in {csv_path} (use another label)')
    return 'append'


def to_plain(value):
    """Return value with numpy numbers and tuples turned into plain Python for yaml."""
    if isinstance(value, dict):
        return {str(k): to_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [to_plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def load_chain(urdf_path):
    """Return the UrdfChain (base_link to gripper_frame_link) of a URDF file."""
    return UrdfChain(Path(urdf_path).read_text(encoding='utf-8'))


def identity_from_files(tag_mount_yaml, urdf, apriltag_yaml, realsense_yaml, tag_id,
                        calibration_yaml):
    """
    Return the identity of a session.

    The sha256 of the four files that were used, tag_id, and the sha256 and the zero_offset_deg of
    the follower calibration yaml (the angles of the CSV are the ones the driver corrected by it).
    """
    return make_identity(tm.sha256_file(tag_mount_yaml), tm.sha256_file(urdf),
                         tm.sha256_file(apriltag_yaml), tm.sha256_file(realsense_yaml), tag_id,
                         tm.sha256_file(calibration_yaml), read_zero_offsets(calibration_yaml))


def camera_extra(k, d, width, height, optical_translation, optical_quaternion):
    """Return the camera values written to the meta file (camera_info, camera_link to optical)."""
    return to_plain({
        'camera_info': {'k': list(k), 'd': list(d), 'width': int(width), 'height': int(height)},
        'camera_link_to_optical_frame': {
            'parent': 'camera_link', 'child': 'camera_color_optical_frame',
            'translation_m': list(optical_translation),
            'quaternion_xyzw': list(optical_quaternion)},
    })


def thresholds():
    """Return the values of this module and of so_arm101_ik that decide what is recorded."""
    return {'frames_wanted': FRAMES_WANTED, 'match_min': MATCH_MIN,
            'collect_timeout_s': COLLECT_TIMEOUT, 'outlier_mad_factor': OUTLIER_MAD_FACTOR,
            'outlier_floor_m': OUTLIER_FLOOR_M, 'stable_tol_m': STABLE_TOL_M,
            'settle_min_s': sr.SETTLE_MIN, 'stop_interval_s': sr.STOP_INTERVAL,
            'stop_tol_rad': sr.STOP_TOL, 'settle_timeout_s': sr.SETTLE_TIMEOUT}


def build_meta(identity, mount, offset_base_at_zero, created, extra=None):
    """Return the content of the meta file (the values that explain where the CSV comes from)."""
    front, up, left = tm.t_pose_offset_mm(mount.offset)
    return to_plain({
        'schema': 1,
        'created': created,
        'identity': identity,
        'tag_mount': {
            'tag_id': mount.tag_id,
            'tag_size_m': mount.tag_size_m,
            'reference_point_gripper_frame_m': mount.reference_point,
            'reference_point_source': mount.reference_source,
            'ref_to_tag_gripper_frame_m': mount.ref_to_tag,
            'offset_gripper_frame_m': mount.offset,
            'offset_base_link_at_zero_pose_m': offset_base_at_zero,
            'offset_t_pose_mm': {'front': front, 'up': up, 'left': left},
            'measured_note': mount.note,
        },
        'thresholds': thresholds(),
        'extra': extra or {},
    })


def write_point(csv_path, label, identity, meta, row):
    """
    Write one point: create the meta file and the CSV (mode 'x') or append a row (mode 'a+').

    The checks of check_session are made again inside the write (under a lock for appending), so
    two runs at the same time cannot write the same label. Returns the idx of the row.
    """
    csv_path = Path(csv_path)
    meta_path = meta_path_for(csv_path)
    state = check_session(csv_path, label, identity)
    if state == 'new':
        try:
            with open(meta_path, 'x', encoding='utf-8') as f:
                yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)
            with open(csv_path, 'x', newline='', encoding='utf-8') as f:
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
            idx = len(_read_csv(csv_path)[1]) + 1
            writer = csv.DictWriter(f, COLUMNS, lineterminator='\n')
            writer.writerow(dict(row, idx=str(idx)))
            f.flush()
            os.fsync(f.fileno())
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)
    return idx


# ----------------------------------------------------------------------------- the loop

@dataclasses.dataclass(frozen=True)
class Measurement:
    """What one measurement of a point found."""

    still: bool
    q_mean: np.ndarray
    q_span: float
    match: MatchResult
    window: object


class RecordLoop:
    """
    Settle, collect frames, show what was found and (after a confirmation) write the row.

    io must have: now(), wait(seconds), read_joints() -> (five angles or None, received_at or
    None), flush() (forget what was received before), poll(seconds) -> (detections, transforms)
    with new data only, say(text), confirm(text) -> bool, wall_time() -> float, date_text() -> str.
    """

    def __init__(self, io, chain, mount, csv_path, identity, extra=None):
        self.io = io
        self.chain = chain
        self.mount = mount
        self.csv_path = Path(csv_path) if csv_path else None  # None: only investigating
        self.identity = identity
        self.extra = extra or {}

    def _joint_sample(self):
        q, received = self.io.read_joints()
        if q is None or not sr.is_fresh(received, self.io.now()):
            return None
        return np.asarray(q, dtype=float)

    def settle(self):
        """Wait until the arm has stopped (same rule as ik_move). Return False on the timeout."""
        self.io.wait(sr.SETTLE_MIN)
        started = self.io.now()
        while True:
            first = self._joint_sample()
            self.io.wait(sr.STOP_INTERVAL)
            second = self._joint_sample()
            if sr.is_stopped(first, second):
                return True
            if self.io.now() - started > sr.SETTLE_TIMEOUT:
                return False

    def collect(self):
        """Collect frames until FRAMES_WANTED matched or the timeout. Return (match, samples)."""
        self.io.flush()
        detections, transforms, samples = {}, {}, []
        started = self.io.now()
        while True:
            new_detections, new_transforms = self.io.poll(POLL_SECONDS)
            detections.update(new_detections)
            transforms.update(new_transforms)
            sample = self._joint_sample()
            if sample is not None:
                samples.append(sample)
            match = match_frames(detections, transforms)
            if (len(match.matched) >= FRAMES_WANTED
                    or self.io.now() - started > COLLECT_TIMEOUT):
                return match, samples

    def measure(self):
        """Return a Measurement, or None if no fresh joint angle was read."""
        still = self.settle()
        match, samples = self.collect()
        if not samples:
            return None
        stack = np.array(samples)
        return Measurement(still, np.mean(stack, axis=0), float(np.max(np.ptp(stack, axis=0))),
                           match, average_matches(match))

    def run(self, label, investigate_only=False):
        """Investigate a point and, unless investigate_only, record it. Return an exit code."""
        say = self.io.say
        if not investigate_only:
            if self.csv_path is None:
                say('refused: a point can only be recorded into a CSV file')
                return EXIT_REFUSED
            try:
                check_session(self.csv_path, label, self.identity)
            except SessionError as exc:
                say(f'refused: {exc}')
                return EXIT_REFUSED
        for line in tm.t_pose_lines(self.mount):
            say(line)
        found = self.measure()
        if found is None:
            say('no fresh joint angles: nothing was recorded')
            return EXIT_NO_INPUT
        for line in investigation_lines(found.match, found.window, found.still):
            say(line)
        if not is_seen(found.match):
            say('the tag is not seen: nothing was recorded. Stop and ask.')
            return EXIT_NOT_SEEN
        if investigate_only:
            return EXIT_OK
        if not self.io.confirm(f'Record this point as {label}?'):
            say('not confirmed: nothing was recorded')
            return EXIT_NOT_CONFIRMED
        found = self.measure()
        if found is None:
            say('no fresh joint angles: nothing was recorded')
            return EXIT_NO_INPUT
        row = build_row(label, self.io.wall_time(), found.q_mean, found.q_span,
                        tm.fk_tcp(self.chain, found.q_mean),
                        tm.fk_tag(self.chain, found.q_mean, self.mount.offset),
                        found.match, found.window, found.still)
        meta = build_meta(self.identity, self.mount,
                          tm.offset_in_base_at_zero(self.chain, self.mount.offset),
                          self.io.date_text(), self.extra)
        try:
            idx = write_point(self.csv_path, label, self.identity, meta, row)
        except SessionError as exc:
            say(f'refused: {exc}')
            return EXIT_REFUSED
        for line in investigation_lines(found.match, found.window, found.still):
            say(line)
        say(f'recorded {label} as row {idx} of {self.csv_path}')
        return EXIT_OK if row['tag_seen'] == '1' else EXIT_NOT_SEEN


# ------------------------------------------------------------------ C4: verification (pure)
# docs/camera_stage_plan.md (the notes of 2026-11-19) and docs/inventory.md "C4-0". Nothing here
# touches the world. The angles that go in are the ones of /joint_states, which the driver has
# already corrected by zero_offset_deg: NO function below takes a zero offset (it would be applied
# twice). The only delta is the estimate of estimate_delta(), which is an output.

MOUNT_PLANE_Z = -0.0024  # m, z of the mounting plane in base_link (C4-0: base_so101_v2 bottom)
REAR_EDGE_X = -0.0211  # m, x of the rear edge of the base plate in base_link (C4-0)
AB_GATE_M = 0.004  # m, only points with |FK(read) - FK(reference)| >= this decide A or B
HOVER_FLOOR_Z = 0.058  # m, lowest TCP z of a hover target (0.048 m floor + calibration margin)
TILT_MAX_DEG = 10.0  # deg, an ID 1 that is tilted more is put down again (the old contact test)
COND_MAX = 100.0  # condition number above which the two deltas are not separated (推測)

# The sign convention of the heights, the same in the notes of the plan and the procedure:
#   desk_minus_mount = (height of the desk surface) - (height of the mounting plane), in m;
#   it is POSITIVE when the desk surface is HIGHER than the mounting plane.
#   thickness = the thickness of ID 1 (m).
#   face_z = MOUNT_PLANE_Z + desk_minus_mount + thickness     (z of the ID 1 face in base_link)
#   H (height above the mounting plane, F6's "height from the desk") = z - MOUNT_PLANE_Z
#   height above the ID 1 face = z - face_z = H - desk_minus_mount - thickness

VERIFY_COLUMNS = (
    'idx', 'time', 'label', 'mark_rear_x', 'mark_rear_y', 'desk_minus_mount', 'thickness',
    'offset_dx', 'offset_h',
    'q_read_pan', 'q_read_lift', 'q_read_elbow', 'q_read_wflex', 'q_read_roll',
    'q_ref_pan', 'q_ref_lift', 'q_ref_elbow', 'q_ref_wflex', 'q_ref_roll',
    'q_goal_pan', 'q_goal_lift', 'q_goal_elbow', 'q_goal_wflex', 'q_goal_roll',
    'ref_minus_goal_max', 'ruler_x', 'ruler_y', 'ruler_h',
    'fk_read_x', 'fk_read_y', 'fk_read_z', 'fk_ref_x', 'fk_ref_y', 'fk_ref_z',
    'phys_x', 'phys_y', 'phys_z', 'd_read_ref_mm', 'fraction', 'perp_mm',
    'fk_read_h_mount', 'fk_read_h_face', 'fk_ref_h_mount', 'fk_ref_h_face',
    'tfb_tag1_x', 'tfb_tag1_y', 'tfb_tag1_z', 'tfb_tag1_age_s', 'tfb_tag1_used_for_judgement',
    'zero_offset_deg', 'still')


def base_x_from_rear(x_rear):
    """Return the base_link x of a point that is x_rear in front of the rear edge of the base."""
    return float(x_rear) + REAR_EDGE_X


def tag_face_z(desk_minus_mount, thickness):
    """Return the base_link z of the ID 1 face (desk_minus_mount > 0: the desk is higher)."""
    return MOUNT_PLANE_Z + float(desk_minus_mount) + float(thickness)


def height_above_mount(z):
    """Return H, the height of a base_link z above the mounting plane (F6's "from the desk")."""
    return float(z) - MOUNT_PLANE_Z


def height_above_face(z, face_z):
    """Return the height of a base_link z above the ID 1 face: H - desk_minus_mount - thickness."""
    return float(z) - float(face_z)


def mark_in_base(x_rear, y, desk_minus_mount, thickness):
    """Return (x, y, face_z) in base_link of the centre of ID 1 from the ruler measurements."""
    return (base_x_from_rear(x_rear), float(y), tag_face_z(desk_minus_mount, thickness))


@dataclasses.dataclass(frozen=True)
class HoverTarget:
    """A hover target: ok, the reason if not, the TCP target and its heights."""

    ok: bool
    reason: str
    target: tuple
    h_mount: float
    h_face: float


def hover_target(mark, dx, h_tcp_above_face, dy=0.0, floor=HOVER_FLOOR_Z):
    """
    Return the HoverTarget for the TCP (gripper_frame_link).

    The mark is (x, y, face_z) of ID 1, dx is the forward offset (m) and h_tcp_above_face is the
    TCP height above the ID 1 face. It is refused if the TCP z is below the floor.
    """
    x, y, face_z = (float(v) for v in mark)
    target = (x + float(dx), y + float(dy), face_z + float(h_tcp_above_face))
    h_mount, h_face = height_above_mount(target[2]), height_above_face(target[2], face_z)
    if not all(math.isfinite(v) for v in target):
        return HoverTarget(False, 'the target is not finite', target, h_mount, h_face)
    if target[2] < floor - 1e-9:
        return HoverTarget(False, f'the TCP z {target[2]:.4f} m is below the floor {floor:.3f} m',
                           target, h_mount, h_face)
    return HoverTarget(True, '', target, h_mount, h_face)


def ik_move_command(target, pitch_deg, roll_deg, dry_run=False):
    """Return the command of tools/ik_move_run.py for a TCP target (the one that is typed)."""
    flag = '--dry-run ' if dry_run else ''
    return ('python3 -u tools/ik_move_run.py %s--goal %.4f %.4f %.4f --pitch-deg %.1f '
            '--roll-deg %.1f' % (flag, target[0], target[1], target[2], pitch_deg, roll_deg))


def jaw_tip(chain, q, reference_point):
    """
    Return the base_link position of the jaw tip.

    It is the FK of gripper_frame_link plus R times the reference point of tag_mount.yaml. q is
    what /joint_states says (already corrected by the driver).
    """
    return tm.fk_tag(chain, q, reference_point)


def jaw_jacobian(chain, q, reference_point, step=1e-5):
    """Return the 3 x 2 matrix (m per rad) of the jaw tip for shoulder_lift and elbow_flex."""
    q = np.asarray(q, dtype=float)
    columns = []
    for joint in (1, 2):
        dq = np.zeros(len(q))
        dq[joint] = step
        columns.append((jaw_tip(chain, q + dq, reference_point)
                        - jaw_tip(chain, q - dq, reference_point)) / (2.0 * step))
    return np.array(columns).T


def physical_jaw_tip(mark, rel_x, rel_y, height):
    """
    Return the jaw tip measured with the ruler, in base_link.

    mark = (x, y, face_z) of the ID 1 centre; rel_x and rel_y = the jaw tip minus the ID 1 centre
    seen from above (+x forward, away from the base; +y left); height = from the ID 1 face to the
    front edge of the jaw tip pad.
    """
    return np.array([mark[0] + float(rel_x), mark[1] + float(rel_y), mark[2] + float(height)])


def point_errors(phys, fk_read, fk_ref):
    """
    Return the comparison of one point.

    fraction is 0 if the physical tip is where FK(read) is (A) and 1 if it is where FK(reference)
    is (B), along the line between the two (None if the two are the same point); perp is the part
    of the physical tip off that line.
    """
    phys, fk_read, fk_ref = (np.asarray(v, dtype=float) for v in (phys, fk_read, fk_ref))
    axis = fk_ref - fk_read
    d = float(np.linalg.norm(axis))
    fraction = perp = None
    if d > 0.0:
        fraction = float(np.dot(phys - fk_read, axis) / (d * d))
        perp = float(np.linalg.norm((phys - fk_read) - fraction * axis))
    return {'e_read': phys - fk_read, 'e_ref': phys - fk_ref, 'd': d, 'fraction': fraction,
            'perp': perp}


def decide_ab(errors, gate=AB_GATE_M):
    """
    Decide A (the reading is the physical angle) or B (the reference is) from point_errors().

    Only the points with d >= gate count; the verdict follows the mean fraction (below 0.5 is A,
    above 0.5 is B, exactly 0.5 is not decided). Fewer than two such points: UNDECIDED. Returns
    (verdict, mean fraction or None, standard deviation or None, number of points used). The
    camera (tfb_tag1) is not an input: the physical tip comes from the ruler only.
    """
    used = [e['fraction'] for e in errors if e['fraction'] is not None and e['d'] >= gate]
    if len(used) < 2:
        return 'UNDECIDED', None, None, len(used)
    mean = float(np.mean(used))
    std = float(np.std(used, ddof=1))
    if mean < 0.5:
        return 'A', mean, std, len(used)
    if mean > 0.5:
        return 'B', mean, std, len(used)
    return 'UNDECIDED', mean, std, len(used)


@dataclasses.dataclass(frozen=True)
class DeltaEstimate:
    """A reference value of delta_lift and delta_elbow (degrees; true = read + delta)."""

    ok: bool
    reason: str
    delta_deg: tuple
    rms_m: float
    condition: float
    sd_ruler_deg: tuple
    sd_common_deg: tuple
    sd_total_deg: tuple


def estimate_delta(chain, points, reference_point, ruler_m=0.002,
                   common_m=(0.0015, 0.0015, 0.0015)):
    """
    Estimate delta_lift and delta_elbow from points = [(q_read, physical jaw tip), ...].

    Least squares of phys - FK(q_read) = J delta (J: jaw_jacobian, per rad; delta in degrees on
    return). The uncertainty has two parts: the ruler error ruler_m of each point (independent,
    averages out) and a shift common_m = (x, y, z) that is the same for every point (the height
    difference, the ID 1 thickness, the x reference: it does not average out). Not ok if there are
    fewer than two points or the two deltas are not separated (condition above COND_MAX).
    """
    nan = (math.nan, math.nan)
    if len(points) < 2:
        return DeltaEstimate(False, 'fewer than two points', nan, math.nan, math.inf, nan, nan,
                             nan)
    jacobians = [jaw_jacobian(chain, q, reference_point) for q, _ in points]
    stacked = np.vstack(jacobians)
    singular = np.linalg.svd(stacked, compute_uv=False)
    condition = float(singular[0] / singular[1]) if singular[1] > 0.0 else math.inf
    if not math.isfinite(condition) or condition > COND_MAX:
        return DeltaEstimate(False, f'delta_lift and delta_elbow are not separated '
                             f'(condition {condition:.1f})', nan, math.nan, condition, nan, nan,
                             nan)
    residual = np.concatenate([np.asarray(p, dtype=float) - jaw_tip(chain, q, reference_point)
                               for q, p in points])
    gram_inverse = np.linalg.inv(stacked.T @ stacked)
    delta = gram_inverse @ stacked.T @ residual  # rad
    fit = stacked @ delta - residual
    to_deg = 180.0 / math.pi
    sd_ruler = float(ruler_m) * np.sqrt(np.diag(gram_inverse)) * to_deg
    jsum = np.sum(jacobians, axis=0)  # 3 x 2: a common shift s moves delta by G^-1 jsum^T s
    common = np.array([gram_inverse @ jsum.T @ np.eye(3)[a] * float(common_m[a])
                       for a in range(3)]) * to_deg  # 3 x 2: one row per shift axis
    sd_common = np.sqrt(np.sum(common ** 2, axis=0))
    sd_total = np.sqrt(sd_ruler ** 2 + sd_common ** 2)
    return DeltaEstimate(True, '', tuple(float(v) for v in delta * to_deg),
                         float(np.sqrt(np.mean(fit ** 2))), condition,
                         tuple(float(v) for v in sd_ruler), tuple(float(v) for v in sd_common),
                         tuple(float(v) for v in sd_total))


def goal_difference(q_ref, q_goal):
    """Return (vector, largest absolute value) of reference - goal (rad), or (None, None)."""
    if q_ref is None or q_goal is None:
        return None, None
    diff = np.asarray(q_ref, dtype=float) - np.asarray(q_goal, dtype=float)
    return diff, float(np.max(np.abs(diff)))


def tf_age_s(now_ns, stamp_ns):
    """Return how old a transform is (s): the time now minus the stamp of the transform."""
    return (int(now_ns) - int(stamp_ns)) / 1e9


def tag_tilt_deg(quaternion_in_base):
    """Return the tilt (deg) of a tag face from the horizontal (its quaternion is in base_link)."""
    normal = tag_normal(quaternion_in_base)
    return float(np.degrees(np.arccos(np.clip(abs(normal[2]), 0.0, 1.0))))


def read_move_goal(move_csv):
    """Return the q_goal (5 floats, rad) of the last row of the CSV of tools/ik_move_run.py."""
    header, rows = _read_csv(move_csv)
    if 'q_goal' not in header or not rows:
        raise SessionError(f'{move_csv} has no q_goal or no row')
    text = rows[-1][header.index('q_goal')]
    values = [float(v) for v in re.findall(r'[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?', text)]
    if len(values) != 5 or not all(math.isfinite(v) for v in values):
        raise SessionError(f'the q_goal of {move_csv} is not 5 finite numbers')
    return np.array(values)


def write_verify_row(csv_path, label, row):
    """
    Append one row of the verification CSV (mode 'x' for a new file, 'a+' under a lock).

    The label may be used only once in a file. Returns the idx of the row.
    """
    if not isinstance(label, str) or not LABEL_PATTERN.fullmatch(label):
        raise SessionError('the label may only have letters, digits, "_", "." and "-"')
    csv_path = Path(csv_path)
    row = dict(row, label=label)
    unknown = set(row) - set(VERIFY_COLUMNS)
    if unknown:
        raise SessionError(f'unknown columns: {sorted(unknown)}')
    if not csv_path.exists():
        try:
            with open(csv_path, 'x', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, VERIFY_COLUMNS, lineterminator='\n')
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
            header, rows = _read_csv(csv_path)
            if header != list(VERIFY_COLUMNS):
                raise SessionError(f'the header of {csv_path} is not the present one')
            if label in [r[VERIFY_COLUMNS.index('label')] for r in rows
                         if len(r) == len(VERIFY_COLUMNS)]:
                raise SessionError(f'the label {label} is already in {csv_path}')
            idx = len(rows) + 1
            writer = csv.DictWriter(f, VERIFY_COLUMNS, lineterminator='\n')
            writer.writerow(dict(row, idx=str(idx)))
            f.flush()
            os.fsync(f.fileno())
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)
    return idx


def _text_of(values, digits=6):
    """Return the numbers as text ('' for None)."""
    if values is None:
        return None
    return [f'{float(v):.{digits}f}' for v in np.ravel(values)]


def build_verify_row(time_s, mark_rear, desk_minus_mount, thickness, offset=None, q_read=None,
                     q_ref=None, q_goal=None, ruler=None, fk_read=None, fk_ref=None, phys=None,
                     errors=None, tfb=None, zero_offset=None, still=None):
    """
    Return the row of the verification CSV (column to text; '' where there is no value).

    tfb = (x, y, z, age_s) of ID 1 from the TF tree, if it was read: it is only recorded
    (tfb_tag1_used_for_judgement is always 0). The heights are given twice, above the mounting
    plane (H) and above the ID 1 face (the sign convention is the one of the notes above).
    """
    row = {name: '' for name in VERIFY_COLUMNS}
    row.update(time=f'{float(time_s):.3f}', mark_rear_x=f'{mark_rear[0]:.4f}',
               mark_rear_y=f'{mark_rear[1]:.4f}', desk_minus_mount=f'{desk_minus_mount:.4f}',
               thickness=f'{thickness:.4f}', tfb_tag1_used_for_judgement='0')
    face_z = tag_face_z(desk_minus_mount, thickness)

    def put(names, values, digits=6):
        text = _text_of(values, digits)
        if text is not None:
            row.update(dict(zip(names, text)))
    if offset is not None:
        put(('offset_dx', 'offset_h'), offset, 4)
    put(('q_read_pan', 'q_read_lift', 'q_read_elbow', 'q_read_wflex', 'q_read_roll'), q_read)
    put(('q_ref_pan', 'q_ref_lift', 'q_ref_elbow', 'q_ref_wflex', 'q_ref_roll'), q_ref)
    put(('q_goal_pan', 'q_goal_lift', 'q_goal_elbow', 'q_goal_wflex', 'q_goal_roll'), q_goal)
    _, largest = goal_difference(q_ref, q_goal)
    put(('ref_minus_goal_max',), None if largest is None else [largest])
    put(('ruler_x', 'ruler_y', 'ruler_h'), ruler, 4)
    put(('fk_read_x', 'fk_read_y', 'fk_read_z'), fk_read)
    put(('fk_ref_x', 'fk_ref_y', 'fk_ref_z'), fk_ref)
    put(('phys_x', 'phys_y', 'phys_z'), phys)
    for tag, point in (('read', fk_read), ('ref', fk_ref)):
        if point is not None:
            put((f'fk_{tag}_h_mount', f'fk_{tag}_h_face'),
                [height_above_mount(point[2]), height_above_face(point[2], face_z)])
    if errors is not None:
        put(('d_read_ref_mm',), [errors['d'] * 1000.0], 3)
        if errors['fraction'] is not None:
            put(('fraction', 'perp_mm'), [errors['fraction'], errors['perp'] * 1000.0], 4)
    if tfb is not None:
        put(('tfb_tag1_x', 'tfb_tag1_y', 'tfb_tag1_z', 'tfb_tag1_age_s'), tfb)
    if zero_offset is not None:
        row['zero_offset_deg'] = ';'.join(f'{k}={v:g}' for k, v in zero_offset.items())
    if still is not None:
        row['still'] = '1' if still else '0'
    return row


def read_verify_rows(csv_path):
    """Return the rows (dict per row) of a verification CSV; the header must be the present one."""
    header, rows = _read_csv(csv_path)
    if header != list(VERIFY_COLUMNS):
        raise SessionError(f'the header of {csv_path} is not the present one')
    return [dict(zip(header, r)) for r in rows if len(r) == len(header)]
