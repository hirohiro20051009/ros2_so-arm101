# Copyright 2026 so_arm101 authors. MIT License.
"""The pure modules of the zero-offset calibration: their imports and form, fixed by the source."""

import ast
from pathlib import Path
import sys
import xml.etree.ElementTree as ElementTree

PACKAGE = Path(__file__).resolve().parents[1]
MODULES = sorted((PACKAGE / 'so_arm101_calibration').glob('*.py'))
SOURCES = MODULES + sorted((PACKAGE / 'test').glob('*.py'))

# Python module of each third-party import and the package.xml name that provides it.
PACKAGE_XML_NAMES = {'numpy': 'python3-numpy', 'yaml': 'python3-yaml',
                     'so_arm101_ik': 'so_arm101_ik'}
FORBIDDEN_IMPORTS = {'rclpy', 'serial', 'socket', 'subprocess', 'sensor_msgs', 'ctypes'}


def imports_of(path):
    found = set()
    for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
        if isinstance(node, ast.Import):
            found |= {a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            found.add(node.module.split('.')[0])
    return found


def declared():
    root = ElementTree.parse(PACKAGE / 'package.xml').getroot()
    return {e.text for tag in ('depend', 'exec_depend') for e in root.findall(tag)}


def test_every_third_party_import_of_the_modules_is_declared_in_package_xml():
    imported = set()
    for path in MODULES:
        imported |= imports_of(path)
    third_party = {m for m in imported if m not in sys.stdlib_module_names}
    assert third_party - {'so_arm101_calibration'} <= set(PACKAGE_XML_NAMES), third_party
    for module in third_party & set(PACKAGE_XML_NAMES):
        assert PACKAGE_XML_NAMES[module] in declared(), module


def test_no_line_is_longer_than_99_characters():
    long_lines = [(p.name, i + 1) for p in SOURCES
                  for i, line in enumerate(p.read_text(encoding='utf-8').splitlines())
                  if len(line) > 99]
    assert long_lines == []


def test_the_pure_modules_import_no_ros_serial_or_process_module():
    for path in MODULES:
        if path.name in ('__init__.py',):
            continue
        assert imports_of(path) & FORBIDDEN_IMPORTS == set(), path.name


def test_the_fit_module_knows_nothing_of_an_expected_offset():
    source = (PACKAGE / 'so_arm101_calibration' / 'zero_offset.py').read_text(encoding='utf-8')
    assert 'expected' not in source.lower()
