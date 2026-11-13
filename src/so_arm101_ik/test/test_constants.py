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

"""The hand-written constants of analytic_ik equal the values derived from the URDF."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
from so_arm101_ik import analytic_ik
from so_arm101_ik.urdf_kinematics import ARM_JOINTS, derive_constants, ModelConstants, UrdfChain

LENGTH_TOLERANCE = 1e-4  # m (0.1 mm)
ANGLE_TOLERANCE = 1e-5  # rad
ANGLES = ('d2', 'd3', 'd4', 'psi0')


def _urdf_xml():
    share = Path(get_package_share_directory('so_arm101_description'))
    return (share / 'urdf' / 'so101_new_calib.urdf').read_text()


def _differences(constants, derived):
    """Return {name: difference} with angles wrapped to (-pi, pi]."""
    result = {}
    for name in ModelConstants._fields:
        diff = getattr(constants, name) - getattr(derived, name)
        if name in ANGLES:
            diff = analytic_ik.normalize_angle(diff)
        result[name] = abs(diff)
    return result


def _violations(constants, derived):
    return {name: diff for name, diff in _differences(constants, derived).items()
            if diff > (ANGLE_TOLERANCE if name in ANGLES else LENGTH_TOLERANCE)}


def test_hand_written_constants_equal_derived_values():
    derived = derive_constants(_urdf_xml())
    differences = _differences(analytic_ik.CONSTANTS, derived)
    assert not _violations(analytic_ik.CONSTANTS, derived), (
        f'hand-written constants differ from the URDF: {differences}')


def test_derived_values_are_the_known_geometry():
    # coarse sanity of the derivation itself (independent of the hand-written numbers)
    c = derive_constants(_urdf_xml())
    assert (c.pan_x, c.pan_z) == (0.0388353, 0.0624)
    assert abs(c.ap - 0.0304) < 1e-4 and abs(c.hp - 0.0542) < 1e-4
    assert abs(c.l1 - 0.116) < 1e-4 and abs(c.l2 - 0.135) < 1e-4
    assert abs(math.degrees(c.d2) + 13.97) < 0.01 and abs(math.degrees(c.d3) - 2.21) < 0.01
    assert abs(c.d4) < ANGLE_TOLERANCE
    # the TCP is about 7.9 mm off the wrist_roll axis, on the -radial side at roll 0
    assert abs(c.k - 0.0079) < 1e-4 and abs(math.degrees(c.psi0) - 178.79) < 0.01


def test_a_changed_urdf_is_detected():
    xml = _urdf_xml()
    for old, new in (('-0.11257 -0.028 1.73763e-16', '-0.11357 -0.028 1.73763e-16'),
                     ('-0.1349 0.0052 3.62355e-17', '-0.1359 0.0052 3.62355e-17'),
                     ('xyz="-0.0079 -0.000218121 -0.0981274"',
                      'xyz="-0.0089 -0.000218121 -0.0981274"'),
                     ('xyz="-0.0303992 -0.0182778 -0.0542"',
                      'xyz="-0.0303992 -0.0182778 -0.0552"')):
        assert xml.count(old) == 1, old
        changed = derive_constants(xml.replace(old, new))
        assert _violations(analytic_ik.CONSTANTS, changed), f'a 1 mm change {old} was not detected'


def test_a_changed_constant_is_detected():
    derived = derive_constants(_urdf_xml())
    for name in ModelConstants._fields:
        step = ANGLE_TOLERANCE if name in ANGLES else LENGTH_TOLERANCE
        value = getattr(analytic_ik.CONSTANTS, name)
        changed = analytic_ik.CONSTANTS._replace(**{name: value + 2 * step})
        assert name in _violations(changed, derived), name


def test_joint_limits_equal_the_urdf():
    lower, upper = UrdfChain(_urdf_xml()).joint_limits()
    assert np.max(np.abs(lower - analytic_ik.Q_MIN)) < 1e-9, (lower, analytic_ik.Q_MIN)
    assert np.max(np.abs(upper - analytic_ik.Q_MAX)) < 1e-9, (upper, analytic_ik.Q_MAX)
    assert len(ARM_JOINTS) == len(lower) == 5
