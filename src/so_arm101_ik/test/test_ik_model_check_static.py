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

"""tools/ik_model_check.py only reads: it must not be able to send anything to the arm."""

import ast
from pathlib import Path

TOOL = Path(__file__).resolve().parents[3] / 'tools' / 'ik_model_check.py'

# nothing that publishes, serves, sends a goal or writes a parameter
FORBIDDEN_NAMES = {
    'create_publisher', 'create_service', 'ActionClient', 'FollowJointTrajectory',
    'GripperCommand', 'JointTrajectory', 'MoveGroup', 'publish', 'send_goal', 'send_goal_async',
    'set_parameters', 'set_parameters_atomically', 'SetParameters', 'AsyncParametersClient',
    'Publisher', 'Popen', 'system', 'create_action_client', 'ServoCommandType',
}
ALLOWED_IMPORTS = {
    'argparse', 'datetime', 'math', 'os', 'pathlib', 'subprocess', 'sys', 'time', 'numpy', 'rclpy',
    'ament_index_python', 'moveit_msgs', 'rcl_interfaces', 'sensor_msgs', 'so_arm101_ik', 'csv',
}
ALLOWED_FROM_IMPORTS = {
    'moveit_msgs.srv': {'GetPositionFK'},
    'moveit_msgs.msg': {'RobotState'},
    'rcl_interfaces.srv': {'GetParameters'},
    'sensor_msgs.msg': {'JointState'},
    'rclpy.node': {'Node'},
}
ALLOWED_SUBPROCESS_COMMANDS = {('git', 'rev-parse'), ('git', 'status'), ('dpkg-query', '-W')}


def _tree():
    assert TOOL.exists(), f'{TOOL} is missing'
    return ast.parse(TOOL.read_text())


def _names(tree):
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            yield node.id, node
        elif isinstance(node, ast.Attribute):
            yield node.attr, node
        elif isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            yield node.name, node
        elif isinstance(node, ast.alias):
            yield node.name.split('.')[-1], node


def _constants(tree):
    return {t.id: n.value.value for n in tree.body if isinstance(n, ast.Assign)
            and isinstance(n.value, ast.Constant) for t in n.targets if isinstance(t, ast.Name)}


def test_the_tool_has_no_publisher_action_client_or_writing_service():
    found = sorted({name for name, _ in _names(_tree()) if name in FORBIDDEN_NAMES})
    assert not found, f'forbidden names in {TOOL.name}: {found}'


def test_the_only_subscription_and_clients_are_the_read_ones():
    tree = _tree()
    constants = _constants(tree)
    assert constants.get('FK_SERVICE') == '/compute_fk'
    assert constants.get('PARAMETER_SERVICE') == '/move_group/get_parameters'
    clients, subscriptions = [], []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            args = [a.id if isinstance(a, ast.Name) else None for a in node.args[:2]]
            if node.func.attr == 'create_client':
                clients.append(tuple(args))
            elif node.func.attr == 'create_subscription':
                subscriptions.append(args[0])
    assert sorted(clients) == [('GetParameters', 'PARAMETER_SERVICE'),
                               ('GetPositionFK', 'FK_SERVICE')]
    assert subscriptions == ['JointState']


def test_imports_and_subprocess_calls_are_limited_to_the_read_only_ones():
    tree = _tree()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                assert alias.name.split('.')[0] in ALLOWED_IMPORTS, alias.name
        elif isinstance(node, ast.ImportFrom):
            module = node.module
            if module.split('.')[0] in ('so_arm101_ik', 'ament_index_python', 'pathlib'):
                continue
            assert module in ALLOWED_FROM_IMPORTS, module
            assert {a.name for a in node.names} <= ALLOWED_FROM_IMPORTS[module], module
    # subprocess is used in run_text() only, and run_text() is only called with a literal list
    # that starts with a read-only command
    in_run_text = [n for f in ast.walk(tree) if isinstance(f, ast.FunctionDef)
                   and f.name == 'run_text' for n in ast.walk(f)]
    subprocess_calls = [
        n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        and isinstance(n.func.value, ast.Name) and n.func.value.id == 'subprocess']
    assert len(subprocess_calls) == 1 and subprocess_calls[0].func.attr == 'run'
    assert subprocess_calls[0] in in_run_text
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == 'run_text']
    assert calls
    for call in calls:
        command = call.args[0]
        assert isinstance(command, ast.List) and len(command.elts) >= 2
        assert all(isinstance(e, ast.Constant) for e in command.elts[:2])
        assert tuple(e.value for e in command.elts[:2]) in ALLOWED_SUBPROCESS_COMMANDS
