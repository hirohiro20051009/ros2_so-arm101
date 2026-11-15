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

"""ik_move (the large move): the options, the goal, the checks, the plan, the arrival."""

import numpy as np
import pytest
from so_arm101_ik import analytic_ik as ik
from so_arm101_ik import diff_mode as dm
from so_arm101_ik import ik_move as mv
from so_arm101_ik import step_runner as sr

T_Q = np.radians([0.0, -22.0, 30.0, 83.0, 0.0])  # the target of T (RViz, integer degrees)
T_MEAS = np.array([0.0031, -0.3813, 0.5501, 1.4454, -0.0069])  # T measured (W1)
REST = np.array([-0.0951, -1.4952, 1.6901, 0.4219, -0.0453])  # the start pose (below the floor)
M2 = (0.200, 0.000, 0.060)
M3 = (0.240, 0.060, 0.065)
NAMES = sr.JOINT_NAMES


def _line(q0, q1, count=11, seconds=6.0):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    return [sr.TrajPoint(seconds * k / (count - 1), tuple(q0 + (q1 - q0) * k / (count - 1)))
            for k in range(count)]


def _reasons(**kwargs):
    return mv.solve_goal(kwargs.pop('q_meas', T_MEAS), **kwargs).reasons


# ------------------------------------------------------------------------------- options

def test_exactly_one_of_goal_offset_and_joints_is_given():
    mv.validate_move_options(goal=M2)
    mv.validate_move_options(offset=(0.01, 0, 0))
    mv.validate_move_options(joints_deg=(0, -22, 30, 83, 0))
    for kwargs in ({}, {'goal': M2, 'offset': (0, 0, 0)},
                   {'goal': M2, 'joints_deg': (0, 0, 0, 0, 0)}):
        with pytest.raises(ValueError):
            mv.validate_move_options(**kwargs)


def test_the_numbers_must_be_finite_and_have_the_right_count():
    for kwargs in ({'goal': (0.1, 0.0)}, {'goal': (0.1, 0.0, float('nan'))},
                   {'offset': (0.1, 0.0, float('inf'))}, {'joints_deg': (0, 0, 0, 0)},
                   {'joints_deg': (0, 0, 0, 0, float('nan'))}):
        with pytest.raises(ValueError):
            mv.validate_move_options(**kwargs)


def test_an_angle_in_degrees_must_be_inside_plus_minus_180():
    mv.validate_move_options(joints_deg=(180, -180, 0, 0, 0))
    mv.validate_move_options(goal=M2, pitch_deg=-180.0, roll_deg=180.0)
    for kwargs in ({'joints_deg': (0, 180.01, 0, 0, 0)}, {'joints_deg': (-181, 0, 0, 0, 0)},
                   {'goal': M2, 'pitch_deg': 180.5}, {'goal': M2, 'roll_deg': -200.0},
                   {'goal': M2, 'roll_deg': float('nan')}, {'goal': M2, 'pitch_deg': True}):
        with pytest.raises(ValueError):
            mv.validate_move_options(**kwargs)


def test_pitch_and_roll_cannot_be_given_with_a_joint_goal():
    for extra in ({'pitch_deg': -90.0}, {'roll_deg': 0.0}):
        with pytest.raises(ValueError):
            mv.validate_move_options(joints_deg=(0, -22, 30, 83, 0), **extra)


def test_the_tolerance_of_moveit_is_limited():
    mv.validate_move_options(goal=M2, moveit_tolerance=0.01)
    for tolerance in (0.0, -0.001, 0.011, float('nan'), True):
        with pytest.raises(ValueError):
            mv.validate_move_options(goal=M2, moveit_tolerance=tolerance)


def test_the_limits_are_values_of_the_differential_mode_and_of_the_plan():
    config = dm.DiffModeConfig()
    assert mv.GOAL_FLOOR == config.floor_z == 0.048
    assert mv.PATH_FLOOR == config.measured_floor_z == 0.045
    assert mv.FIRST_DESIRED_TOL == config.max_windup == 0.06
    assert mv.MAX_TIP_MOVE == 0.15 and mv.MAX_JOINT_MOVE == 1.0 and mv.MAX_PLAN_DURATION == 20.0
    assert mv.RECHECK_MEAS_TOL == sr.STOP_TOL and mv.RECHECK_DESIRED_TOL == sr.DESIRED_TOL_DIRECT


# ------------------------------------------------------------------------------- start

def test_the_start_pose_below_the_floor_is_refused_by_the_measured_floor():
    assert ik.fk(REST)[0][2] < 0.045
    found = mv.start_checks(REST)
    assert [code for code, _ in found] == ['START_FLOOR_MEASURED']
    assert mv.start_checks(T_MEAS) == []
    assert 'START_FLOOR_MEASURED' in _reasons(q_meas=REST, goal=M2)


def test_the_measured_floor_is_checked_at_its_boundary():
    # lower the tip of T until z crosses 0.045 (wrist_flex only: a monotonic change of z)
    low = None
    for lift in np.linspace(0.0, 0.5, 501):
        q = T_MEAS + np.array([0.0, lift, 0.0, 0.0, 0.0])
        if ik.fk(q)[0][2] < 0.045:
            low = q
            break
    assert low is not None
    assert mv.start_checks(low)
    assert not mv.start_checks(T_MEAS)


def test_a_measurement_that_is_not_finite_is_refused():
    bad = T_MEAS.copy()
    bad[2] = float('nan')
    plan = mv.solve_goal(bad, goal=M2)
    assert not plan.ok and plan.reasons == ('NOT_FINITE',) and plan.q_goal is None


# -------------------------------------------------------------------------------- goal

@pytest.mark.parametrize('goal', [M2, M3])
def test_the_goals_of_the_real_arm_stages_are_solved_from_t(goal):
    for q_meas in (T_MEAS, T_Q):
        plan = mv.solve_goal(q_meas, goal=goal)
        assert plan.ok, plan.details
        assert plan.candidates >= 1
        assert np.linalg.norm(ik.fk(plan.q_goal)[0] - np.array(goal)) < 1e-6
        assert plan.branch == dm.branch_of_q(q_meas)
        assert ik.within_limits(plan.q_goal)
        assert plan.tip_move < mv.MAX_TIP_MOVE and np.max(plan.joint_move) < mv.MAX_JOINT_MOVE
        assert plan.abs_d <= 0.989


def test_an_offset_is_taken_from_the_measured_tip():
    offset = np.array([0.02, 0.01, 0.003])
    plan = mv.solve_goal(T_MEAS, offset=offset)
    assert plan.ok, plan.details
    assert np.allclose(plan.target, ik.fk(T_MEAS)[0] + offset, atol=1e-12)
    assert np.linalg.norm(ik.fk(plan.q_goal)[0] - plan.target) < 1e-6


def test_pitch_and_roll_default_to_those_of_the_measured_pose_and_can_be_given():
    _, pitch0, roll0 = ik.fk(T_MEAS)
    plan = mv.solve_goal(T_MEAS, goal=M2)
    _, pitch, roll = ik.fk(plan.q_goal)
    assert abs(pitch - pitch0) < 1e-6 and abs(roll - roll0) < 1e-6
    plan = mv.solve_goal(T_MEAS, goal=M2, pitch_deg=-85.0, roll_deg=5.0)
    _, pitch, roll = ik.fk(plan.q_goal)
    assert abs(pitch - np.radians(-85.0)) < 1e-6 and abs(roll - np.radians(5.0)) < 1e-6


def test_a_joint_goal_is_in_degrees_and_its_target_is_its_fk():
    plan = mv.solve_goal(T_MEAS, joints_deg=(0, -22, 30, 83, 0))
    assert plan.ok, plan.details
    assert np.allclose(plan.q_goal, T_Q, atol=1e-12)
    assert np.allclose(plan.target, ik.fk(T_Q)[0])
    assert plan.candidates == 0


def test_a_target_below_the_floor_is_refused_and_the_floor_is_the_goal_floor():
    assert 'GOAL_FLOOR' in _reasons(goal=(0.20, 0.0, 0.047))
    assert 'GOAL_FLOOR' not in _reasons(goal=(0.20, 0.0, 0.0481))
    # a joint goal whose tip is below the floor (the arm is lowered by lift)
    low = T_Q + np.array([0.0, 0.2, 0.0, 0.0, 0.0])
    assert ik.fk(low)[0][2] < 0.048
    assert 'GOAL_FLOOR' in _reasons(joints_deg=tuple(np.degrees(low)))


def test_an_unreachable_target_is_refused():
    plan = mv.solve_goal(T_MEAS, goal=(0.60, 0.0, 0.10))
    assert not plan.ok and 'UNREACHABLE' in plan.reasons and plan.q_goal is None


def test_a_goal_outside_the_joint_range_is_refused_in_two_steps():
    # +z with the pitch kept: wrist_flex runs out first (the strict range at +28.5 mm, the
    # margin of 0.02 rad at +25.6 mm; docs/analytic_diff_plan.md 7.12.4)
    z0 = ik.fk(T_Q)[0]
    far = _reasons(q_meas=T_Q, goal=tuple(z0 + np.array([0.0, 0.0, 0.040])))
    assert 'JOINT_LIMIT' in far
    near = _reasons(q_meas=T_Q, goal=tuple(z0 + np.array([0.0, 0.0, 0.027])))
    assert 'JOINT_RANGE' in near and 'JOINT_LIMIT' not in near
    ok = _reasons(q_meas=T_Q, goal=tuple(z0 + np.array([0.0, 0.0, 0.020])))
    assert 'JOINT_RANGE' not in ok and 'JOINT_LIMIT' not in ok


def test_a_joint_goal_near_a_limit_is_refused():
    q = T_Q.copy()
    q[1] = ik.Q_MIN[1] + 0.01  # shoulder_lift 0.01 rad from its lower limit
    found = [code for code, _ in mv.goal_checks(q, q)]
    assert 'JOINT_RANGE' in found and 'JOINT_LIMIT' not in found
    q[1] = ik.Q_MIN[1] - 0.01
    assert 'JOINT_LIMIT' in [code for code, _ in mv.goal_checks(q, q)]


def test_a_pose_near_the_stretched_arm_is_singular():
    q3 = -(np.pi / 2.0 + ik.CONSTANTS.d2 - ik.CONSTANTS.d3)  # abs(D) = 1
    q = np.array([0.0, -0.4, q3, 1.0, 0.0])
    assert dm.abs_d_of_q(q) > 0.999
    assert 'SINGULAR' in [code for code, _ in mv.goal_checks(q, q)]
    assert 'SINGULAR' not in [code for code, _ in mv.goal_checks(T_Q, T_Q)]
    # both elbow candidates coincide there, so the goal is not on a unique branch of the IK
    assert 'NO_BRANCH' in [code for code, _ in mv.goal_checks(q, q)]
    assert 'NO_BRANCH' not in [code for code, _ in mv.goal_checks(T_Q, T_Q)]


def test_the_other_elbow_branch_is_refused_for_a_joint_goal_and_a_target():
    candidates = ik.ik_geometry(*ik.fk(T_Q)[0], *ik.fk(T_Q)[1:]).candidates
    other = [q for q in candidates if dm.branch_of_q(q) != dm.branch_of_q(T_Q)]
    assert len(other) == 1
    assert 'BRANCH_SWITCH' in [code for code, _ in mv.goal_checks(other[0], T_Q)]
    assert 'BRANCH_SWITCH' in _reasons(q_meas=T_Q, joints_deg=tuple(np.degrees(other[0])))
    assert 'BRANCH_SWITCH' not in [code for code, _ in mv.goal_checks(T_Q, T_Q)]


def test_the_nearest_candidate_is_taken_for_a_target():
    plan = mv.solve_goal(T_MEAS, goal=M2)
    geometry = ik.ik_geometry(*M2, *ik.fk(T_MEAS)[1:])
    valid = [q for q in geometry.candidates if ik.within_limits(q)]
    assert plan.candidates == len(valid)
    best = min(valid, key=lambda q: np.linalg.norm(q - T_MEAS))
    assert np.allclose(plan.q_goal, best)


def test_the_move_is_limited_for_the_tip_and_for_every_joint():
    assert 'MOVE_TIP_TOO_FAR' in _reasons(offset=(0.0, 0.16, 0.0))
    assert 'MOVE_TIP_TOO_FAR' not in _reasons(offset=(0.0, 0.14, 0.0))
    for joint in range(5):
        for size, expected in ((0.99, False), (1.01, True)):
            q_meas = T_Q.copy()
            q_meas[joint] += size  # the goal is T_Q, so the joint moves `size`
            found = [code for code, _ in mv.goal_checks(T_Q, q_meas)]
            assert ('MOVE_JOINT_TOO_FAR' in found) is expected, (joint, size)


def test_the_move_from_the_start_pose_to_t_is_too_big_for_the_tool():
    found = [code for code, _ in mv.goal_checks(T_Q, REST)]
    assert 'MOVE_JOINT_TOO_FAR' in found


def test_every_reason_is_collected_not_only_the_first():
    plan = mv.solve_goal(REST, joints_deg=(0, -22, 30, 83, 0))
    assert not plan.ok
    assert {'START_FLOOR_MEASURED', 'MOVE_JOINT_TOO_FAR'} <= set(plan.reasons)
    assert len(plan.details) >= 2


def test_a_goal_that_passes_is_a_valid_start_of_the_differential_mode():
    rng = np.random.default_rng(20261115)
    passed = 0
    for _ in range(300):
        offset = rng.uniform(-0.06, 0.06, 3)
        plan = mv.solve_goal(T_MEAS, offset=offset)
        if not plan.ok:
            continue
        passed += 1
        dm.DiffMode.start(plan.q_goal, plan.q_goal)  # StartRefused would fail the test
        assert ik.within_limits(plan.q_goal)
        assert ik.fk(plan.q_goal)[0][2] >= 0.048 - 1e-9
        assert np.linalg.norm(ik.fk(plan.q_goal)[0] - plan.target) < 1e-6
    assert passed > 30


# -------------------------------------------------------------------------------- path

def test_a_path_that_stays_in_range_above_the_floor_passes_and_is_sampled_finely():
    goal = mv.solve_goal(T_MEAS, goal=M3).q_goal
    path = mv.check_path([tuple(T_MEAS), tuple(goal)])
    assert path.ok, path
    assert path.samples >= 11 and path.min_z > 0.045
    assert mv.check_path([tuple(T_MEAS)]).samples == 1


def test_a_point_below_the_floor_or_out_of_range_is_found():
    low = T_Q + np.array([0.0, 0.2, 0.0, 0.0, 0.0])
    assert 'PATH_FLOOR' in mv.check_path([tuple(T_Q), tuple(low)]).reasons
    out = T_Q.copy()
    out[3] = ik.Q_MAX[3] + 0.01
    assert 'PATH_RANGE' in mv.check_path([tuple(T_Q), tuple(out)]).reasons
    on_limit = T_Q.copy()
    on_limit[0] = ik.Q_MAX[0]  # exactly on the limit is allowed
    assert 'PATH_RANGE' not in mv.check_path([tuple(T_Q), tuple(on_limit)]).reasons


def test_the_points_between_the_points_of_the_plan_are_checked():
    # both ends are above the floor, a point in the middle of a long segment is not looked at
    # unless the segment is cut: the number of samples follows the size of the step
    a, b = T_Q, T_Q + np.array([0.0, 0.0, 0.0, 0.15, 0.0])
    path = mv.check_path([tuple(a), tuple(b)], step=0.01)
    assert path.ok and path.samples == 16
    assert mv.check_path([tuple(a), tuple(b)], step=0.1).samples == 3


def test_a_path_that_is_not_numbers_is_refused():
    for bad in ([], [(0.0, 1.0)], [tuple(T_Q), (float('nan'),) * 5], None):
        assert mv.check_path(bad).reasons == ('PATH_NOT_FINITE',)


# -------------------------------------------------------------------------------- plan

def test_a_good_plan_passes_even_when_the_arm_stopped_short_of_the_desired():
    plan = mv.solve_goal(T_MEAS, goal=M2)
    desired = T_MEAS + np.array([0.0, 0.0, -0.034, 0.0, 0.0])  # elbow 0.034 rad short (V7)
    check = mv.check_plan(_line(T_MEAS, plan.q_goal), NAMES, plan.q_goal, desired, T_MEAS)
    assert check.ok, check.reasons
    assert abs(check.plan.first_vs_desired - 0.034) < 1e-9 and check.path.min_z > 0.045
    # the old tolerance of the differential mode (0.005) would refuse it every time
    old = sr.check_planned_trajectory(_line(T_MEAS, plan.q_goal), NAMES, plan.q_goal, desired,
                                      T_MEAS)
    assert 'FIRST_DESIRED' in old.reasons


def test_the_first_point_is_checked_against_the_desired_and_the_measurement():
    goal = mv.solve_goal(T_MEAS, goal=M2).q_goal
    points = _line(T_MEAS, goal)
    far = T_MEAS + np.array([0.0, 0.0, 0.07, 0.0, 0.0])
    assert 'FIRST_DESIRED' in mv.check_plan(points, NAMES, goal, far, T_MEAS).reasons
    edge = T_MEAS + np.array([0.0, 0.0, 0.059, 0.0, 0.0])
    assert mv.check_plan(points, NAMES, goal, edge, T_MEAS).ok
    moved = T_MEAS + np.array([0.011, 0.0, 0.0, 0.0, 0.0])  # the plan starts 0.011 from q_meas
    assert 'FIRST_MEASURED' in mv.check_plan(points, NAMES, goal, T_MEAS, moved).reasons


def test_the_last_point_the_names_and_the_speed_are_checked():
    goal = mv.solve_goal(T_MEAS, goal=M2).q_goal
    off = _line(T_MEAS, goal + np.array([0.0, 0.0, 0.003, 0.0, 0.0]))
    assert 'LAST' in mv.check_plan(off, NAMES, goal, T_MEAS, T_MEAS).reasons
    assert 'NAMES' in mv.check_plan(_line(T_MEAS, goal), NAMES[::-1], goal, T_MEAS,
                                    T_MEAS).reasons
    fast = _line(T_MEAS, goal, seconds=0.05)
    assert 'SPEED' in mv.check_plan(fast, NAMES, goal, T_MEAS, T_MEAS, max_duration=20).reasons


def test_a_plan_longer_than_the_limit_is_refused():
    goal = mv.solve_goal(T_MEAS, goal=M2).q_goal
    assert mv.check_plan(_line(T_MEAS, goal, seconds=19.9), NAMES, goal, T_MEAS, T_MEAS).ok
    assert 'DURATION' in mv.check_plan(_line(T_MEAS, goal, seconds=20.5), NAMES, goal, T_MEAS,
                                       T_MEAS).reasons


def test_a_plan_through_the_floor_is_refused_although_both_ends_are_above_it():
    goal = mv.solve_goal(T_MEAS, goal=M2).q_goal
    low = T_Q + np.array([0.0, 0.2, 0.0, 0.0, 0.0])
    points = _line(T_MEAS, low, 6, 3.0)[:-1] + _line(low, goal, 6, 3.0)
    points = [sr.TrajPoint(i * 0.6, p.positions) for i, p in enumerate(points)]
    check = mv.check_plan(points, NAMES, goal, T_MEAS, T_MEAS)
    assert 'PATH_FLOOR' in check.reasons and not check.ok


# ----------------------------------------------------------------------------- recheck

def _recheck(**kwargs):
    goal = mv.solve_goal(T_MEAS, goal=M2).q_goal
    args = dict(q_meas_planned=T_MEAS, q_desired_planned=T_MEAS, q_meas_now=T_MEAS,
                q_desired_now=T_MEAS, active=[], points=_line(T_MEAS, goal), joint_names=NAMES,
                q_goal=goal)
    args.update(kwargs)
    return mv.recheck_before_send(**args)


def test_nothing_changed_nothing_is_reported():
    assert _recheck() is None


def test_the_arm_moved_the_desired_changed_or_another_goal_is_active():
    assert _recheck(q_meas_now=T_MEAS + 0.002) is None  # inside STOP_TOL
    assert _recheck(q_meas_now=T_MEAS + 0.004)[0] == 'ARM_MOVED'
    assert _recheck(q_desired_now=T_MEAS + 0.00005) is None
    assert _recheck(q_desired_now=T_MEAS + 0.0002)[0] == 'DESIRED_CHANGED'
    assert _recheck(active=['/move_action: 1 active goal'])[0] == 'OTHER_GOAL'


def test_the_goal_and_the_plan_are_checked_again_with_the_new_measurement():
    goal = mv.solve_goal(T_MEAS, goal=M2).q_goal
    planned = goal - np.array([0.0, 0.9995, 0.0, 0.0, 0.0])  # the goal is 0.9995 rad away
    now = planned + np.array([0.0, -0.002, 0.0, 0.0, 0.0])  # 1.0015 rad away, inside STOP_TOL
    bad = _recheck(q_meas_planned=planned, q_desired_planned=planned, q_meas_now=now,
                   q_desired_now=planned, q_goal=goal, points=_line(now, goal))
    assert bad is not None and bad[0] == 'MOVE_JOINT_TOO_FAR'
    assert _recheck(points=_line(T_MEAS, goal + 0.01))[0] == 'PLAN_RECHECK'
    assert _recheck(q_meas_now=None)[0] == 'NOT_FINITE'


# ---------------------------------------------------------------------------- arrival

def test_the_arrival_is_judged_from_the_measurement_with_ten_millimetres():
    goal = mv.solve_goal(T_MEAS, goal=M2).q_goal
    target = np.array(M2)
    exact = mv.judge_arrival(goal, target, goal, goal)
    assert exact.label == 'REACHED' and exact.exit_code == 0 and exact.in_band
    assert exact.error_mm < 1e-6 and exact.ref_error_mm < 1e-6
    short = goal + np.array([0.0, 0.0, -0.05, 0.0, 0.0])  # the elbow stopped 0.05 rad short
    arrival = mv.judge_arrival(goal, target, short, goal)
    expected = float(np.linalg.norm(ik.fk(short)[0] - target)) * 1000.0
    assert abs(arrival.error_mm - expected) < 1e-9
    assert np.allclose(arrival.stuck, goal - short)
    assert np.allclose(arrival.joint_error, short - goal)


def test_the_label_follows_the_error_in_millimetres():
    goal = mv.solve_goal(T_MEAS, goal=M2).q_goal
    target = ik.fk(goal)[0]
    for shift, label, code, band in ((0.0005, 'REACHED', 0, True), (0.005, 'REACHED', 0, False),
                                     (0.012, 'SHORT', 4, False)):
        # move the target instead of the arm: the tip is `shift` metres from it
        arrival = mv.judge_arrival(goal, target + np.array([shift, 0.0, 0.0]), goal)
        assert (arrival.label, arrival.exit_code, arrival.in_band) == (label, code, band), shift
    no_desired = mv.judge_arrival(goal, target, goal)
    assert no_desired.ref_error_mm != no_desired.ref_error_mm and no_desired.stuck is None


def test_the_text_and_the_csv_row_show_the_numbers():
    plan = mv.solve_goal(T_MEAS, goal=M2)
    text = mv.goal_lines(plan, T_MEAS, T_MEAS + 0.01)
    assert 'q_goal' in text and 'windup before' in text and 'branch=' in text
    check = mv.check_plan(_line(T_MEAS, plan.q_goal), NAMES, plan.q_goal, T_MEAS, T_MEAS)
    assert 'PLAN OK' in mv.plan_lines(check) and 'lowest z' in mv.plan_lines(check)
    arrival = mv.judge_arrival(plan.q_goal, plan.target, plan.q_goal, plan.q_goal)
    assert 'ARRIVAL REACHED' in mv.arrival_lines(arrival)
    row = mv.move_record(input='goal', q_goal=plan.q_goal, plan_reasons=('A', 'B'),
                         tip_error_mm=1.5)
    assert list(row) == list(mv.MOVE_CSV_FIELDS)
    assert row['plan_reasons'] == 'A B' and row['tip_error_mm'] == '1.5'
    with pytest.raises(ValueError):
        mv.move_record(nonsense=1)
    # the columns of an interrupted run: where it was, whether a goal went out, never cancelled
    for column in ('stage', 'goal_sent', 'goal_in_flight', 'goal_cancelled', 'q_meas_last'):
        assert column in mv.MOVE_CSV_FIELDS
    row = mv.move_record(stage='execute', goal_sent=True, goal_in_flight=True,
                         goal_cancelled=False, q_meas_last=T_MEAS)
    assert row['goal_cancelled'] is False and row['goal_in_flight'] is True
