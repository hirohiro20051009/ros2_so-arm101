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
