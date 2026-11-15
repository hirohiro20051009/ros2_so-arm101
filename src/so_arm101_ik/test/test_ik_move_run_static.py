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

"""tools/ik_move_run.py moves the arm: what it can send is fixed by its source (AST, no ROS)."""

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TOOL = ROOT / 'tools' / 'ik_move_run.py'
MODULE = Path(__file__).resolve().parents[1] / 'so_arm101_ik' / 'ik_move.py'

FORBIDDEN_NAMES = {
    'cancel_goal', 'cancel_goal_async', 'cancel', 'create_publisher', 'create_service', 'publish',
    'Popen', 'system', 'pkill', 'killall', 'subprocess', 'JointTrajectory', 'create_timer',
    'FollowJointTrajectory', 'GripperCommand',
}
ROS_PACKAGES = {'rclpy', 'moveit_msgs', 'control_msgs', 'sensor_msgs', 'action_msgs',
                'trajectory_msgs', 'geometry_msgs'}


def _tree(path):
    return ast.parse(path.read_text(encoding='utf-8'))


def _name_of(node):
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return None


def _parents(tree):
    table = {}
    for parent in ast.walk(tree):
        for child in ast.iter_child_nodes(parent):
            table[child] = parent
    return table


def _enclosing(node, parents, kind):
    while node in parents:
        node = parents[node]
        if isinstance(node, kind):
            return node
    return None


def _calls(tree, name):
    return [n for n in ast.walk(tree) if isinstance(n, ast.Call) and _name_of(n.func) == name]


def _used(tree):
    used = {_name_of(n) for n in ast.walk(tree) if isinstance(n, (ast.Name, ast.Attribute))}
    used |= {a.name.split('.')[0] for n in ast.walk(tree) if isinstance(n, ast.Import)
             for a in n.names}
    used |= {n.module.split('.')[0] for n in ast.walk(tree)
             if isinstance(n, ast.ImportFrom) and n.module}
    used |= {a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) for a in n.names}
    return used


def test_nothing_is_cancelled_published_or_run_as_a_process():
    for path in (TOOL, MODULE):
        assert not (_used(_tree(path)) & FORBIDDEN_NAMES), path.name


def test_the_module_has_no_ros_and_the_tool_has_no_direct_route():
    assert not (_used(_tree(MODULE)) & ROS_PACKAGES)
    tool = _tree(TOOL)
    flags = []
    for call in _calls(tool, 'add_argument'):
        flags += [a.value for a in call.args if isinstance(a, ast.Constant)]
    assert '--route' not in flags and '--direct' not in flags


def test_send_goal_async_is_called_in_two_places_one_plans_and_one_moves():
    tree = _tree(TOOL)
    parents = _parents(tree)
    client_type = {}
    for call in _calls(tree, 'ActionClient'):
        assign = parents[call]
        assert isinstance(assign, ast.Assign), 'ActionClient must be assigned to an attribute'
        target = assign.targets[0]
        assert isinstance(target, ast.Attribute)
        owner = _enclosing(call, parents, ast.ClassDef)
        client_type[(owner.name, target.attr)] = _name_of(call.args[1])
    sites = []
    for call in _calls(tree, 'send_goal_async'):
        client = call.func.value
        assert isinstance(client, ast.Attribute) and _name_of(client.value) == 'self'
        owner = _enclosing(call, parents, ast.ClassDef)
        function = _enclosing(call, parents, ast.FunctionDef)
        sites.append((owner.name, function.name, client_type[(owner.name, client.attr)]))
    assert sorted(sites) == sorted([
        ('MoveItSender', 'execute', 'ExecuteTrajectory'),
        ('MoveItSender', 'plan', 'MoveGroup')]), sites


def test_the_goal_for_move_action_is_built_from_the_spec_and_only_plans():
    tree = _tree(TOOL)
    parents = _parents(tree)
    assigned = [n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Attribute) and t.attr == 'plan_only' for t in n.targets)]
    assert len(assigned) == 1
    value = assigned[0].value
    assert isinstance(value, ast.Attribute) and value.attr == 'plan_only'
    assert isinstance(value.value, ast.Name) and value.value.id == 'spec'
    function = _enclosing(assigned[0], parents, ast.FunctionDef)
    assert function.name == 'plan'
    spec = [n for n in ast.walk(function) if isinstance(n, ast.Assign)
            and isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'spec']
    assert len(spec) == 1 and _name_of(spec[0].value.func) == 'build_plan_goal_spec'
    goals = [n for n in ast.walk(tree) if isinstance(n, ast.Attribute) and n.attr == 'Goal'
             and _name_of(n.value) == 'MoveGroup']
    assert len(goals) == 1 and _enclosing(goals[0], parents, ast.FunctionDef) is function
    for node in ast.walk(tree):
        if isinstance(node, (ast.AugAssign, ast.AnnAssign)):
            target = node.target
            assert not (isinstance(target, ast.Attribute) and target.attr == 'plan_only')
        if isinstance(node, ast.keyword):
            assert node.arg != 'plan_only'


def test_only_the_sender_makes_clients_and_only_main_makes_the_sender():
    tree = _tree(TOOL)
    parents = _parents(tree)
    owners = [_enclosing(c, parents, ast.ClassDef).name for c in _calls(tree, 'ActionClient')]
    assert owners == ['MoveItSender', 'MoveItSender']
    for call in _calls(tree, 'ActionClient'):
        assert _enclosing(call, parents, ast.FunctionDef).name == '__init__'
    made = _calls(tree, 'MoveItSender')
    assert len(made) == 1 and _enclosing(made[0], parents, ast.FunctionDef).name == 'main'


def test_there_is_no_option_that_skips_a_check_or_changes_the_limits():
    flags = []
    for call in _calls(_tree(TOOL), 'add_argument'):
        flags += [a.value for a in call.args if isinstance(a, ast.Constant)]
    assert '--dry-run' in flags and '--joints-deg' in flags
    for flag in flags:
        assert flag not in ('--yes', '-y', '--force', '--auto')
        assert 'scaling' not in flag and 'max' not in flag and 'floor' not in flag, flag


def test_the_options_are_checked_before_any_node_or_client_is_made():
    main = [n for n in ast.walk(_tree(TOOL)) if isinstance(n, ast.FunctionDef)
            and n.name == 'main'][0]
    first = {}
    for call in ast.walk(main):
        if isinstance(call, ast.Call):
            name = _name_of(call.func)
            first[name] = min(first.get(name, call.lineno), call.lineno)
    assert 'validate_move_options' in first
    for later in ('init', 'RunNode', 'MoveItSender', 'MoveLoop'):
        assert first['validate_move_options'] < first[later], later
    module = _tree(MODULE)
    parents = _parents(module)
    checks = [_enclosing(c, parents, ast.FunctionDef).name
              for c in _calls(module, 'validate_move_options')]
    assert '__init__' in checks  # MoveLoop checks the options again


def test_the_send_is_made_in_one_place_after_two_confirmations_and_the_recheck():
    module = _tree(MODULE)
    parents = _parents(module)
    executes = _calls(module, 'execute')
    assert len(executes) == 1
    function = _enclosing(executes[0], parents, ast.FunctionDef)
    assert function.name == '_run'
    lines = {}
    for call in ast.walk(function):
        if isinstance(call, ast.Call):
            lines.setdefault(_name_of(call.func), []).append(call.lineno)
    confirms = sorted(lines['_confirm'])
    assert len(confirms) == 2
    assert confirms[1] < lines['recheck_before_send'][0] < executes[0].lineno
    assert lines['plan'][0] > confirms[0] and lines['plan'][0] < confirms[1]
    assert len(lines['recheck_before_send']) == 1


def test_the_interruption_only_says_logs_and_returns():
    module = _tree(MODULE)
    run = [n for n in ast.walk(module) if isinstance(n, ast.FunctionDef) and n.name == 'run'
           and _enclosing(n, _parents(module), ast.ClassDef).name == 'MoveLoop'][0]
    handlers = [h for h in ast.walk(run) if isinstance(h, ast.ExceptHandler)
                and _name_of(h.type) == 'KeyboardInterrupt']
    assert len(handlers) == 1
    called = {_name_of(c.func) for h in handlers for c in ast.walk(h) if isinstance(c, ast.Call)}
    # no send, no plan, no read, no wait, no confirmation, no cancel in the interruption
    assert called <= {'_say', '_log', 'update', 'RunResult'}, called
    assert {'_log', 'RunResult'} <= called
    # the row says that nothing was cancelled, as a literal
    log = [n for n in ast.walk(module) if isinstance(n, ast.FunctionDef) and n.name == '_log'][0]
    cancelled = [k for c in ast.walk(log) if isinstance(c, ast.Call) for k in c.keywords
                 if k.arg == 'goal_cancelled']
    assert len(cancelled) == 1 and isinstance(cancelled[0].value, ast.Constant)
    assert cancelled[0].value.value is False
    assert len(_calls(module, 'execute')) == 1


def test_the_limits_of_the_move_are_constants_that_the_goal_check_uses():
    module = _tree(MODULE)
    names = {t.id: n.value for n in module.body if isinstance(n, ast.Assign)
             for t in n.targets if isinstance(t, ast.Name)}
    for constant in ('MAX_TIP_MOVE', 'MAX_JOINT_MOVE', 'MAX_ANGLE_DEG', 'MAX_PLAN_DURATION'):
        assert constant in names, constant
        assert isinstance(names[constant], ast.Constant), constant
    # the one limit that is the value of the differential mode: exactly _CONFIG.max_windup
    assert 'FIRST_DESIRED_TOL' in names
    value = names['FIRST_DESIRED_TOL']
    assert (isinstance(value, ast.Attribute) and value.attr == 'max_windup'
            and isinstance(value.value, ast.Name) and value.value.id == '_CONFIG'), \
        'FIRST_DESIRED_TOL must be _CONFIG.max_windup'
    goal_checks = [n for n in ast.walk(module) if isinstance(n, ast.FunctionDef)
                   and n.name == 'goal_checks'][0]
    used = {n.id for n in ast.walk(goal_checks) if isinstance(n, ast.Name)}
    assert {'MAX_TIP_MOVE', 'MAX_JOINT_MOVE'} <= used
    validate = [n for n in ast.walk(module) if isinstance(n, ast.FunctionDef)
                and n.name == 'validate_move_options'][0]
    assert 'MAX_ANGLE_DEG' in {n.id for n in ast.walk(validate) if isinstance(n, ast.Name)}
