# Copyright 2026 so_arm101 authors. MIT License.
"""
Zero-offset calibration (floor touch): fit zero_offset_deg of lift, elbow and wrist_flex.

This is NOT the center calibration (center/min/max are decided by so_arm101_calib measure/apply).
The tip of the fixed jaw is put on the desk in several poses with the torque off, and the joint
angles of
those touches give the deltas.  True angle = measured angle + delta
(docs/zero_offset_calib_plan.md).

Model of one touch: the lowest vertex of the fixed jaw (the visual meshes of gripper_link) is at
the height h
of the desk.  Unknowns: delta of lift, elbow, wrist_flex and h (h has a prior).  Weighted least
squares with
Gauss-Newton.  Nothing here reads a serial port or starts a ROS node, and nothing writes a file.
"""

import dataclasses
import math
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from so_arm101_ik.urdf_kinematics import ARM_JOINTS, rpy_matrix

CALIBRATION_KIND = 'zero_offset'
BANNER_PREFIX = '[zero-offset calibration]'
FIT_JOINTS = ('shoulder_lift', 'elbow_flex', 'wrist_flex')  # the unknown deltas, in this order
FIT_INDEX = tuple(ARM_JOINTS.index(name) for name in FIT_JOINTS)

EXIT_OK = 0
EXIT_ERROR = 1  # an argument or a file could not be read
EXIT_REJECTED = 2  # the checks failed or the input was refused (nothing is pasted)

# Values of this module.  None of them is measured: every one is a guess (推測) until the
# real touches.
H_PRIOR_MM = -2.4  # the desk height when the arm and the desk are on the same surface
H_SIGMA_MM = 1.0
SIGMA_CONTACT_MM = 1.0  # scatter of one touch
# Added in quadrature to a vertical (EPS_90_MIN_DEG or more) touch.  0: the user checked on the
# real arm that, with the jaw closed, the moving jaw does not touch the desk first (the model says
# 1.26 mm lower at 90 deg; the difference is 未確認, plan 8).  1.0 was the first guess.
SIGMA_90_EXTRA_MM = 0.0
EPS_90_MIN_DEG = 80.0  # an elevation of at least this is called vertical (state: a few deg off)
EPS_MIN_DEG = 45.0
MIN_POSES = 4
MIN_RECORDS = 8
TILT_GAP_DEG = 10.0
TILT_GROUPS = 3
X_SPREAD_MM = 150.0
CONDITION_MAX = 1e3
RMS_MAX_MM = 1.5
RESIDUAL_MAX_MM = 3.0
PAIR_DIFF_WARN_MM = 2.0
H_WARN_MM = 3.0
H_FAIL_MM = 5.0
DELTA_MAX_DEG = 10.0  # the upper limit of the driver (kMaxZeroOffsetDeg)
SD_LIMIT_DEG = {'shoulder_lift': 0.5, 'elbow_flex': 1.0, 'wrist_flex': 1.0}
LOO_LIMIT_DEG = {'shoulder_lift': 0.5, 'elbow_flex': 1.0, 'wrist_flex': 1.0}
WRIST_MIN_DEG = 1.5
WRIST_SD_MAX_DEG = 1.0
DESK_WARN_MM = 60.0  # a record this far above the desk is probably in the air
REF_POINT_Z_M = 0.0063  # the reference point on +z of gripper_frame_link (the same as the study)
FD_STEP_RAD = 1e-4
ITERATIONS = 10
WRIST_ZERO_NOTE = ('wrist is left at 0 (wrist は 0 のまま（理由: 90° の当たり方の 1 mm で約 0.7° の'
                   '偏りが出るため））')

RECORD_BANNER = (
    f'{BANNER_PREFIX} zero_offset_record_run: records touch poses for zero_offset_deg '
    '(NOT the center calibration: center/min/max are decided by so_arm101_calib measure/apply). '
    'Read-only: nothing is written.')
FIT_BANNER = (f'{BANNER_PREFIX} zero_offset_fit_run: result for zero_offset_deg '
              '(display only; the yaml is not written)')


# ----------------------------------------------------------------------------- the fixed jaw mesh

def read_stl_vertices(path):
    """Return the unique vertices (n, 3) in metres of a binary STL file."""
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError(f'{path}: not a binary STL')
    count = int(np.frombuffer(data, dtype='<u4', count=1, offset=80)[0])
    if len(data) != 84 + 50 * count:
        raise ValueError(f'{path}: not a binary STL')
    dtype = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    triangles = np.frombuffer(data, dtype=dtype, count=count, offset=84)
    return np.unique(triangles['v'].reshape(-1, 3).astype(float), axis=0)


def _floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


def link_visuals(urdf_xml, link_name='gripper_link'):
    """Return [(mesh file name, xyz, rotation)] of the visual meshes of a link."""
    root = ElementTree.fromstring(urdf_xml)
    items = []
    for link in root.findall('link'):
        if link.get('name') != link_name:
            continue
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            if mesh.get('scale'):
                raise ValueError(f'{link_name}: a mesh with a scale is not handled')
            origin = visual.find('origin')
            xyz = _floats(None if origin is None else origin.get('xyz'), (0.0, 0.0, 0.0))
            rpy = _floats(None if origin is None else origin.get('rpy'), (0.0, 0.0, 0.0))
            items.append((Path(mesh.get('filename')).name, np.array(xyz), rpy_matrix(*rpy)))
    if not items:
        raise ValueError(f'no visual mesh in the link {link_name}')
    return items


class FixedJaw:
    """The vertices of the fixed jaw (gripper_link) in the frame of the wrist_roll joint."""

    def __init__(self, vertices):
        self.vertices = np.asarray(vertices, dtype=float).reshape(-1, 3)
        if len(self.vertices) == 0:
            raise ValueError('the fixed jaw has no vertex')

    @classmethod
    def from_urdf(cls, urdf_xml, mesh_dir):
        """Build it from the visual meshes of gripper_link (the STL files are in mesh_dir)."""
        parts = []
        for name, xyz, rotation in link_visuals(urdf_xml):
            parts.append(read_stl_vertices(Path(mesh_dir) / name) @ rotation.T + xyz)
        return cls(np.unique(np.concatenate(parts), axis=0))

    def lowest_z(self, chain, q5):
        """Return the lowest z (m, base_link) over all vertices for the five arm joints q5."""
        position, rotation = chain.frames(np.asarray(q5, dtype=float))['wrist_roll']
        return float(np.min(self.vertices @ rotation[2] + position[2]))


def tool_state(chain, q5):
    """
    Return (x in mm of the reference point, elevation in deg of the tool axis).

    The elevation is directed, not folded: 90 is straight down, less than 90 is a forward tilt
    (the tip ahead of the wrist) and more than 90 is a backward tilt (up to 135 and more), so
    that 85 and 95 are told apart.  Forward is the direction of the pan plane (the horizontal
    direction that the pan joint turns from +x of base_link at pan 0).
    """
    q5 = np.asarray(q5, dtype=float)
    position, rotation = chain.fk(q5)
    position, rotation = np.asarray(position, float), np.asarray(rotation, float)
    reference = position + rotation @ np.array([0.0, 0.0, REF_POINT_Z_M])
    axis = rotation[:, 2]
    turned = chain.frames(q5)['shoulder_pan'][1] @ chain.frames(np.zeros_like(q5))[
        'shoulder_pan'][1].T
    forward = turned @ np.array([1.0, 0.0, 0.0])
    eps = math.degrees(math.atan2(-axis[2], float(axis[0] * forward[0] + axis[1] * forward[1])))
    return float(reference[0] * 1000.0), float(eps)


# ----------------------------------------------------------------------------- the records

@dataclasses.dataclass(frozen=True)
class Touch:
    """One record: the state angles (rad) of the five arm joints, as /joint_states had them."""

    label: str
    q: tuple

    @property
    def pose(self):
        """Return the pose name: the label without the trailing '#n' of the repeat."""
        return self.label.split('#')[0]


@dataclasses.dataclass(frozen=True)
class Options:
    """The numbers of the model."""

    h_prior_mm: float = H_PRIOR_MM
    h_sigma_mm: float = H_SIGMA_MM
    sigma_contact_mm: float = SIGMA_CONTACT_MM
    sigma_90_extra_mm: float = SIGMA_90_EXTRA_MM


def raw0_of(touch, delta_old_deg):
    """Return q_raw0: the state minus the delta the driver applied (joint name to deg)."""
    q = np.array(touch.q, dtype=float)
    for index, name in enumerate(ARM_JOINTS):
        q[index] -= math.radians(delta_old_deg.get(name, 0.0))
    return q


def touch_sigma_mm(chain, touch, options):
    """Return the sigma (mm) of a touch: the contact scatter, plus the extra for a vertical one."""
    _, eps = tool_state(chain, touch.q)
    if eps >= EPS_90_MIN_DEG:
        return math.hypot(options.sigma_contact_mm, options.sigma_90_extra_mm)
    return options.sigma_contact_mm


def height_above_desk_mm(chain, jaw, q_state, options=Options()):
    """Return how far the lowest vertex is above the prior desk height (mm) for a state."""
    return jaw.lowest_z(chain, q_state) * 1000.0 - options.h_prior_mm


def predicted_z(chain, jaw, q_raw0, delta_rad):
    """Return the lowest z (m) when the deltas (rad, lift/elbow/wrist) are added to q_raw0."""
    q = np.array(q_raw0, dtype=float)
    for index, value in zip(FIT_INDEX, delta_rad):
        q[index] += value
    return jaw.lowest_z(chain, q)


# ----------------------------------------------------------------------------- the fit

@dataclasses.dataclass
class Solution:
    """The result of one fit.  delta_deg is absolute (it is not a change of the yaml)."""

    delta_deg: np.ndarray
    free: tuple
    h_mm: float
    labels: tuple
    residual_mm: np.ndarray
    rms_mm: float
    max_mm: float
    dof: int
    chi2: float
    sd_model_deg: np.ndarray
    sd_resid_deg: np.ndarray
    corr: np.ndarray
    cond: float
    smin: float
    iterations: int

    @property
    def sd_deg(self):
        """Return the larger of the model sd and the sd scaled by the residuals."""
        return np.maximum(self.sd_model_deg, self.sd_resid_deg)


def fit(chain, jaw, touches, delta_old_deg, options=Options(), exclude=(), fixed_wrist=False):
    """Fit the deltas and h to the touches (labels in exclude are left out)."""
    used = [t for t in touches if t.label not in set(exclude)]
    free = (True, True, not fixed_wrist)
    columns = [i for i in range(3) if free[i]]
    if len(used) < len(columns) + 1:
        raise ValueError(f'{len(used)} records are too few for {len(columns)} deltas and h')
    raw0 = [raw0_of(t, delta_old_deg) for t in used]
    sigma_m = np.array([touch_sigma_mm(chain, t, options) for t in used]) / 1000.0
    h0 = options.h_prior_mm / 1000.0
    h_sigma = options.h_sigma_mm / 1000.0
    count = len(columns) + 1
    u = np.zeros(count)
    u[-1] = h0

    def delta_of(vec):
        delta = np.zeros(3)
        for k, column in enumerate(columns):
            delta[column] = vec[k]
        return delta

    def residual_and_jacobian(vec):
        delta = delta_of(vec[:-1])
        z = np.array([predicted_z(chain, jaw, q, delta) for q in raw0])
        jac = np.zeros((len(used) + 1, count))
        for k, column in enumerate(columns):
            for row, q in enumerate(raw0):
                step = np.zeros(3)
                step[column] = FD_STEP_RAD
                jac[row, k] = (predicted_z(chain, jaw, q, delta + step)
                               - predicted_z(chain, jaw, q, delta - step)) / (2 * FD_STEP_RAD)
        jac[:len(used), -1] = -1.0
        jac[len(used), -1] = 1.0
        r = np.concatenate([z - vec[-1], [vec[-1] - h0]])
        weights = np.concatenate([1.0 / sigma_m, [1.0 / h_sigma]])
        return r * weights, jac * weights[:, None], z

    iterations = 0
    for iterations in range(1, ITERATIONS + 1):
        r, jac, _ = residual_and_jacobian(u)
        step = np.linalg.lstsq(jac, -r, rcond=None)[0]
        u = u + step
        if np.max(np.abs(step[:-1]), initial=0.0) < 1e-10 and abs(step[-1]) < 1e-12:
            break
    r, jac, z = residual_and_jacobian(u)
    cov = np.linalg.inv(jac.T @ jac)
    chi2 = float(r @ r)
    dof = len(used) + 1 - count
    scale = math.sqrt(chi2 / dof) if dof > 0 else 1.0
    sd = np.zeros(3)
    sd_resid = np.zeros(3)
    for k, column in enumerate(columns):
        sd[column] = math.degrees(math.sqrt(max(cov[k, k], 0.0)))
        sd_resid[column] = sd[column] * scale
    corr = np.zeros((4, 4))
    full = [*columns, 3]
    root = np.sqrt(np.diag(cov))
    for a in range(count):
        for b in range(count):
            corr[full[a], full[b]] = cov[a, b] / (root[a] * root[b])
    norms = np.linalg.norm(jac, axis=0)
    singular = np.linalg.svd(jac / norms, compute_uv=False)
    smin = float(singular[-1])
    cond = float(singular[0] / smin) if smin > 0 else math.inf
    touch_residual = (z - u[-1]) * 1000.0
    return Solution(
        delta_deg=np.degrees(delta_of(u[:-1])), free=free, h_mm=float(u[-1] * 1000.0),
        labels=tuple(t.label for t in used), residual_mm=touch_residual,
        rms_mm=float(math.sqrt(np.mean(touch_residual ** 2))),
        max_mm=float(np.max(np.abs(touch_residual))), dof=dof, chi2=chi2, sd_model_deg=sd,
        sd_resid_deg=sd_resid, corr=corr, cond=cond, smin=smin, iterations=iterations)


def leave_one_out(chain, jaw, touches, delta_old_deg, base, options=Options(), fixed_wrist=False):
    """Return {label: change of delta_deg (3,)} when that one record is left out."""
    changes = {}
    for touch in touches:
        if touch.label not in base.labels:
            continue
        other = fit(chain, jaw, touches, delta_old_deg, options, exclude=(touch.label,),
                    fixed_wrist=fixed_wrist)
        changes[touch.label] = other.delta_deg - base.delta_deg
    return changes


# ----------------------------------------------------------------------------- the checks

@dataclasses.dataclass(frozen=True)
class Check:
    """One check: severity is 'fail' (rejects) or 'warn' (accepted with a warning)."""

    name: str
    ok: bool
    severity: str
    detail: str
    joint: str = ''

    @property
    def failed(self):
        """Return True when the check failed and its severity is fail."""
        return not self.ok and self.severity == 'fail'

    @property
    def warned(self):
        """Return True when the check failed and its severity is warn."""
        return not self.ok and self.severity == 'warn'


def tilt_groups(eps_values, gap_deg=TILT_GAP_DEG):
    """Return the number of groups of elevations; a gap of gap_deg or more starts a new group."""
    values = sorted(eps_values)
    if not values:
        return 0
    return 1 + sum(1 for a, b in zip(values, values[1:]) if b - a >= gap_deg)


def pose_summary(chain, touches):
    """Return [(pose, mean elevation deg, mean x mm, count)] in the order of the first record."""
    order = {}
    for touch in touches:
        x, eps = tool_state(chain, touch.q)
        order.setdefault(touch.pose, []).append((eps, x))
    return [(pose, float(np.mean([e for e, _ in v])), float(np.mean([x for _, x in v])), len(v))
            for pose, v in order.items()]


def check_poses(poses, x_values, eps_values, records):
    """Return the checks of the set of poses; poses is the output of pose_summary."""
    spread = (max(x_values) - min(x_values)) if x_values else 0.0
    groups = tilt_groups([eps for _, eps, _, _ in poses])
    low = [eps for eps in eps_values if eps < EPS_MIN_DEG]
    return [
        Check('poses', len(poses) >= MIN_POSES, 'fail',
              f'{len(poses)} poses (at least {MIN_POSES})'),
        Check('records', records >= MIN_RECORDS, 'fail',
              f'{records} records (at least {MIN_RECORDS})'),
        Check('tilt_groups', groups >= TILT_GROUPS, 'fail',
              f'{groups} groups of elevation, {TILT_GAP_DEG:g} deg apart (at least {TILT_GROUPS}; '
              'otherwise wrist and h cannot be separated)'),
        Check('x_spread', spread >= X_SPREAD_MM, 'fail',
              f'x spread {spread:.1f} mm (at least {X_SPREAD_MM:g})'),
        Check('eps_min', not low, 'fail',
              f'lowest elevation {min(eps_values) if eps_values else 0.0:.1f} deg '
              f'(at least {EPS_MIN_DEG:g})'),
    ]


def wrist_rule(delta_deg, sd_deg):
    """Return True when the wrist line may be pasted (1.5 deg or more, sd 1.0 deg or less)."""
    return abs(delta_deg) >= WRIST_MIN_DEG and sd_deg <= WRIST_SD_MAX_DEG


def check_h(h_mm, prior_mm):
    """Return the check of the estimated h (warning beyond 3 mm of the prior, failure beyond 5)."""
    deviation = abs(h_mm - prior_mm)
    severity = 'fail' if deviation > H_FAIL_MM else 'warn'
    return Check('h_estimate', deviation <= H_WARN_MM, severity,
                 f'h {h_mm:.2f} mm, prior {prior_mm:.2f} mm (deviation {deviation:.2f})')


def pair_differences(touches, solution):
    """Return {pose: largest difference (mm) of the residuals of its records}."""
    by_pose = {}
    for touch, residual in zip(touches, solution.residual_mm):
        by_pose.setdefault(touch.pose, []).append(residual)
    return {pose: float(max(v) - min(v)) for pose, v in by_pose.items() if len(v) > 1}


def round_tenth(value):
    """Return the value rounded to 0.1 (never -0.0)."""
    result = round(float(value), 1)
    return 0.0 if result == 0 else result


def paste_line(joint, value_deg, old_deg=0.0):
    """Return the line to paste into the yaml and its difference from the current yaml."""
    value = round_tenth(value_deg)
    return (f'{joint}: zero_offset_deg: {value:.1f}   '
            f'(yaml now {old_deg:.1f}, difference {round_tenth(value - old_deg):+.1f})')


@dataclasses.dataclass
class Report:
    """What evaluate decided."""

    checks: list
    poses: list
    free: object
    used: object
    used_mode: str
    loo: dict
    pairs: dict
    paste: dict
    notes: dict
    accepted: bool
    warnings: int
    delta_old_deg: dict

    @property
    def exit_code(self):
        """Return 0 when accepted and 2 otherwise."""
        return EXIT_OK if self.accepted else EXIT_REJECTED


def _joint_checks(solution, loo, delta_old_deg):
    checks = []
    for index, joint in enumerate(FIT_JOINTS):
        sd = float(solution.sd_deg[index])
        change = max((abs(float(c[index])) for c in loo.values()), default=0.0)
        delta = float(solution.delta_deg[index])
        checks.append(Check(f'sd_{joint}', sd <= SD_LIMIT_DEG[joint], 'fail',
                            f'sd {sd:.2f} deg (at most {SD_LIMIT_DEG[joint]:g})', joint))
        checks.append(Check(f'leave_one_out_{joint}', change <= LOO_LIMIT_DEG[joint], 'fail',
                            f'largest change {change:.2f} deg when one record is left out '
                            f'(at most {LOO_LIMIT_DEG[joint]:g})', joint))
        checks.append(Check(f'delta_abs_{joint}', abs(delta) <= DELTA_MAX_DEG, 'fail',
                            f'delta {delta:.2f} deg (at most {DELTA_MAX_DEG:g})', joint))
    return checks


def evaluate(chain, jaw, touches, delta_old_deg, options=Options()):
    """Run the pose checks, the fits and the result checks; nothing is written."""
    summary = pose_summary(chain, touches)
    states = [tool_state(chain, t.q) for t in touches]
    checks = check_poses(summary, [x for x, _ in states], [e for _, e in states], len(touches))
    free = used = None
    loo, pairs, paste, notes = {}, {}, {}, {}
    mode = 'none'
    if len(touches) < 4:
        checks.append(Check('fit', False, 'fail', f'{len(touches)} records are too few to fit'))
    else:
        try:
            free = fit(chain, jaw, touches, delta_old_deg, options)
            loo_free = leave_one_out(chain, jaw, touches, delta_old_deg, free, options)
            change_wrist = max((abs(float(c[2])) for c in loo_free.values()), default=0.0)
            use_wrist = (wrist_rule(free.delta_deg[2], free.sd_deg[2])
                         and change_wrist <= LOO_LIMIT_DEG['wrist_flex']
                         and abs(free.delta_deg[2]) <= DELTA_MAX_DEG)
            if use_wrist:
                used, loo, mode = free, loo_free, 'free'
            else:
                used = fit(chain, jaw, touches, delta_old_deg, options, fixed_wrist=True)
                loo = leave_one_out(chain, jaw, touches, delta_old_deg, used, options,
                                    fixed_wrist=True)
                mode = 'wrist_fixed'
        except (np.linalg.LinAlgError, ValueError) as exc:
            checks.append(Check('fit', False, 'fail', f'the fit failed: {exc}'))
    if used is not None:
        pairs = pair_differences([t for t in touches if t.label in used.labels], used)
        checks.append(Check('condition', used.cond <= CONDITION_MAX, 'fail',
                            f'condition number {used.cond:.3g} (at most {CONDITION_MAX:g}), '
                            f'smallest singular value {used.smin:.3g}'))
        checks.append(Check('residual_rms', used.rms_mm <= RMS_MAX_MM, 'fail',
                            f'rms {used.rms_mm:.2f} mm (at most {RMS_MAX_MM:g})'))
        checks.append(Check('residual_max', used.max_mm <= RESIDUAL_MAX_MM, 'fail',
                            f'largest residual {used.max_mm:.2f} mm '
                            f'(at most {RESIDUAL_MAX_MM:g})'))
        checks.append(Check('pair_difference', all(d <= PAIR_DIFF_WARN_MM for d in pairs.values()),
                            'warn', 'difference of the residuals of the two touches of a pose: '
                            + (', '.join(f'{p} {d:.2f} mm' for p, d in pairs.items()) or 'none')
                            + f' (at most {PAIR_DIFF_WARN_MM:g})'))
        checks.append(check_h(used.h_mm, options.h_prior_mm))
        for check in _joint_checks(used, loo, delta_old_deg):
            if mode == 'wrist_fixed' and check.joint == 'wrist_flex':
                continue
            checks.append(check)
    failed = [c for c in checks if c.failed]
    accepted = used is not None and not failed
    if accepted:
        for index, joint in enumerate(FIT_JOINTS):
            if joint == 'wrist_flex' and mode == 'wrist_fixed':
                notes[joint] = WRIST_ZERO_NOTE
                continue
            paste[joint] = paste_line(joint, used.delta_deg[index], delta_old_deg.get(joint, 0.0))
    else:
        for joint in FIT_JOINTS:
            notes[joint] = 'not written (rejected: ' + ', '.join(c.name for c in failed) + ')'
    return Report(checks=checks, poses=summary, free=free, used=used, used_mode=mode, loo=loo,
                  pairs=pairs, paste=paste, notes=notes, accepted=accepted,
                  warnings=sum(1 for c in checks if c.warned), delta_old_deg=dict(delta_old_deg))


def report_lines(report):
    """Return the lines of the result (every item is shown, also for a rejection)."""
    lines = ['poses (elevation deg, x mm, records):']
    for pose, eps, x, count in report.poses:
        lines.append(f'  {pose}: {eps:.1f} deg, x {x:.1f} mm, {count} records')
    used = report.used
    if used is not None:
        lines.append(f'model: {report.used_mode} (wrist_flex is '
                     + ('a fitted value' if report.used_mode == 'free' else 'fixed at 0') + ')')
        for label, residual in zip(used.labels, used.residual_mm):
            lines.append(f'  residual {label}: {residual:+.2f} mm')
        lines.append(f'h {used.h_mm:.2f} mm; rms {used.rms_mm:.2f} mm; '
                     f'largest {used.max_mm:.2f} mm; '
                     f'degrees of freedom {used.dof}')
        for index, joint in enumerate(FIT_JOINTS):
            old = report.delta_old_deg.get(joint, 0.0)
            lines.append(f'  {joint}: delta {used.delta_deg[index]:+.2f} deg, sd '
                         f'{used.sd_model_deg[index]:.2f} (model) {used.sd_resid_deg[index]:.2f} '
                         f'(residuals), yaml now {old:.2f}')
        if report.free is not None and report.used_mode != 'free':
            free = report.free
            lines.append('  for reference, wrist_flex free: '
                         + ', '.join(f'{j} {free.delta_deg[i]:+.2f} (sd {free.sd_deg[i]:.2f})'
                                     for i, j in enumerate(FIT_JOINTS)))
        names = ('lift', 'elbow', 'wrist', 'h')
        for a in range(4):
            lines.append('  correlation ' + names[a] + ': '
                         + ' '.join(f'{used.corr[a, b]:+.2f}' for b in range(4)))
    for check in report.checks:
        mark = 'ok' if check.ok else ('WARN' if check.severity == 'warn' else 'FAIL')
        lines.append(f'  [{mark}] {check.name}: {check.detail}')
    if report.accepted:
        warn = f' (with {report.warnings} warning)' if report.warnings else ''
        lines.append(f'{BANNER_PREFIX} result: accepted{warn}')
    else:
        lines.append(f'{BANNER_PREFIX} result: rejected (nothing to paste)')
    for joint in FIT_JOINTS:
        if joint in report.paste:
            lines.append(f'  paste: {report.paste[joint]}')
        else:
            lines.append(f'  {joint}: {report.notes.get(joint, "not written")}')
    return lines
