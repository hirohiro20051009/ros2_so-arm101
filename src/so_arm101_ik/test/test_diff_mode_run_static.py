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

"""tools/diff_mode_run.py moves the arm: what it can send is fixed by its source (AST, no ROS)."""

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TOOL = ROOT / 'tools' / 'diff_mode_run.py'
RUNNER = Path(__file__).resolve().parents[1] / 'so_arm101_ik' / 'step_runner.py'

FORBIDDEN_NAMES = {
    'cancel_goal', 'cancel_goal_async', 'cancel', 'create_publisher', 'create_service', 'publish',
    'Popen', 'system', 'pkill', 'killall', 'subprocess', 'JointTrajectory', 'create_timer',
}
WRITE_TYPES = {'FollowJointTrajectory', 'ExecuteTrajectory'}


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


def test_the_route_is_required_and_has_no_default():
    calls = [c for c in _calls(_tree(TOOL), 'add_argument')
             if c.args and isinstance(c.args[0], ast.Constant) and c.args[0].value == '--route']
    assert len(calls) == 1
    keywords = {k.arg: k.value for k in calls[0].keywords}
    required = keywords.get('required')
    assert isinstance(required, ast.Constant) and required.value is True
    assert 'default' not in keywords


def test_nothing_is_cancelled_published_or_run_as_a_process():
    for path in (TOOL, RUNNER):
        tree = _tree(path)
        used = {_name_of(n) for n in ast.walk(tree) if isinstance(n, (ast.Name, ast.Attribute))}
        used |= {a.name.split('.')[0] for n in ast.walk(tree) if isinstance(n, ast.Import)
                 for a in n.names}
        used |= {n.module.split('.')[0] for n in ast.walk(tree)
                 if isinstance(n, ast.ImportFrom) and n.module}
        assert not (used & FORBIDDEN_NAMES), (path.name, used & FORBIDDEN_NAMES)


def test_send_goal_async_is_called_in_three_places_one_plans_and_two_move():
    tree = _tree(TOOL)
    parents = _parents(tree)
    client_type = {}  # (class name, attribute) -> the action type of the client
    for call in _calls(tree, 'ActionClient'):
        assign = parents[call]
        assert isinstance(assign, ast.Assign), 'ActionClient must be assigned to an attribute'
        target = assign.targets[0]
        assert isinstance(target, ast.Attribute)
        owner = _enclosing(call, parents, ast.ClassDef)
        client_type[(owner.name, target.attr)] = _name_of(call.args[1])
    sites = []
    for call in _calls(tree, 'send_goal_async'):
        client = call.func.value  # self.<client>
        assert isinstance(client, ast.Attribute) and _name_of(client.value) == 'self'
        owner = _enclosing(call, parents, ast.ClassDef)
        function = _enclosing(call, parents, ast.FunctionDef)
        sites.append((owner.name, function.name, client_type[(owner.name, client.attr)]))
    assert sorted(sites) == sorted([
        ('DirectSender', 'execute', 'FollowJointTrajectory'),
        ('MoveItSender', 'execute', 'ExecuteTrajectory'),
        ('MoveItSender', 'prepare', 'MoveGroup')]), sites
    assert sum(1 for s in sites if s[2] in WRITE_TYPES) == 2  # the paths that move the arm
    assert sum(1 for s in sites if s[2] == 'MoveGroup') == 1  # plans only


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
    assert function.name == 'prepare'
    spec = [n for n in ast.walk(function) if isinstance(n, ast.Assign)
            and isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'spec']
    assert len(spec) == 1 and _name_of(spec[0].value.func) == 'build_plan_goal_spec'
    # the only MoveGroup goal of the tool is this one
    goals = [n for n in ast.walk(tree) if isinstance(n, ast.Attribute) and n.attr == 'Goal'
             and _name_of(n.value) == 'MoveGroup']
    assert len(goals) == 1 and _enclosing(goals[0], parents, ast.FunctionDef) is function
    # no other write of plan_only, e.g. a keyword argument or an augmented assignment
    for node in ast.walk(tree):
        if isinstance(node, (ast.AugAssign, ast.AnnAssign)):
            target = node.target
            assert not (isinstance(target, ast.Attribute) and target.attr == 'plan_only')
        if isinstance(node, ast.keyword):
            assert node.arg != 'plan_only'


def test_only_one_sender_is_made_and_only_the_senders_make_clients():
    tree = _tree(TOOL)
    parents = _parents(tree)
    for call in _calls(tree, 'ActionClient'):
        owner = _enclosing(call, parents, ast.ClassDef)
        assert owner.name in ('DirectSender', 'MoveItSender')
        assert _enclosing(call, parents, ast.FunctionDef).name == '__init__'
    clients = {}
    for call in _calls(tree, 'ActionClient'):
        owner = _enclosing(call, parents, ast.ClassDef).name
        clients[owner] = clients.get(owner, 0) + 1
    assert clients == {'DirectSender': 1, 'MoveItSender': 2}
    for name in ('DirectSender', 'MoveItSender'):  # made only by make_sender (one of the two)
        made = _calls(tree, name)
        assert len(made) == 1
        assert _enclosing(made[0], parents, ast.FunctionDef).name == 'make_sender'
    callers = _calls(tree, 'make_sender')
    assert len(callers) == 1 and _enclosing(callers[0], parents, ast.FunctionDef).name == 'main'


def test_there_is_no_option_that_skips_the_confirmation_or_sets_the_scaling():
    flags = []
    for call in _calls(_tree(TOOL), 'add_argument'):
        flags += [a.value for a in call.args if isinstance(a, ast.Constant)]
    assert '--auto' in flags and '--dry-run' in flags
    assert not [f for f in flags if f in ('--yes', '-y', '--force') or 'scaling' in f]


def test_the_options_are_checked_before_any_node_or_client_is_made():
    tree = _tree(TOOL)
    main = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'main'][0]
    first = {}
    for call in ast.walk(main):
        if isinstance(call, ast.Call):
            name = _name_of(call.func)
            first[name] = min(first.get(name, call.lineno), call.lineno)
    assert 'validate_run_options' in first
    for later in ('init', 'RunNode', 'make_sender', 'StepLoop'):
        assert first['validate_run_options'] < first[later], later
    runner = _tree(RUNNER)  # StepLoop checks the options again in its constructor
    runner_parents = _parents(runner)
    checks = [_enclosing(c, runner_parents, ast.FunctionDef).name
              for c in _calls(runner, 'validate_run_options')]
    assert '__init__' in checks


def test_assume_desired_is_measured_is_only_handed_to_the_check_and_the_loop():
    tree = _tree(TOOL)
    uses = [n for n in ast.walk(tree) if isinstance(n, ast.Attribute)
            and n.attr == 'assume_desired_is_measured']
    parents = _parents(tree)
    inside = 0
    for use in uses:
        call = parents[use]
        if isinstance(call, ast.keyword):  # passed as assume_desired_is_measured=args.…
            call = parents[call]
        assert isinstance(call, ast.Call) and _name_of(call.func) in ('validate_run_options',
                                                                      'StepLoop')
        inside += 1
    assert inside >= 2
    for node in ast.walk(tree):  # never the condition of an if or of a conditional expression
        if isinstance(node, (ast.If, ast.IfExp)):
            names = {n.attr for n in ast.walk(node.test) if isinstance(n, ast.Attribute)}
            assert 'assume_desired_is_measured' not in names


def test_step_runner_imports_no_ros_and_rechecks_everything_before_a_send():
    tree = _tree(RUNNER)
    allowed = {'dataclasses', 'math', 'typing', 'numpy'}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            assert {a.name.split('.')[0] for a in node.names} <= allowed
        if isinstance(node, ast.ImportFrom):
            assert node.level == 1 or node.module.split('.')[0] in allowed, node.module
    functions = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]
    recheck = [n for n in functions if n.name == '_recheck']
    assert len(recheck) == 1
    called = {_name_of(c.func) for c in ast.walk(recheck[0]) if isinstance(c, ast.Call)}
    assert {'_read', 'desired_matches', 'other_active_goals', 'verify_command'} <= called
    # _one_step calls _recheck before it prepares and executes anything
    step = [n for n in functions if n.name == '_one_step'][0]
    lines = {}
    for call in ast.walk(step):
        if isinstance(call, ast.Call):
            lines.setdefault(_name_of(call.func), []).append(call.lineno)
    assert max(lines['_recheck']) < min(lines['prepare']) < min(lines['execute'])


def test_the_confirmation_reads_the_input_only_through_the_line_reader():
    tree = _tree(TOOL)
    parents = _parents(tree)
    confirm = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'confirm']
    assert len(confirm) == 1
    called = {_name_of(c.func) for c in ast.walk(confirm[0]) if isinstance(c, ast.Call)}
    assert 'confirm_answer' in called
    assert not ({'readline', 'readlines', 'input', 'read'} & called)
    for call in _calls(tree, 'readline') + _calls(tree, 'input'):
        raise AssertionError('the tool must not read stdin by itself: %d' % call.lineno)
    readers = _calls(tree, 'LineReader')
    assert len(readers) == 1
    assert _enclosing(readers[0], parents, ast.FunctionDef).name == '__init__'
