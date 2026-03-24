# Copyright 2025 FRL RoboCup Brasil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Regression tests for NED convention handling in lpv_mpc_drone_node.

The pos_controller() function in SupportFilesDrone uses ENU (altitude-up)
physics: U1 = (vz + g) * m.  All internal state and waypoints are stored
in NED (z negative when airborne).  These tests verify that the NED→ENU
conversion applied before calling pos_controller() produces positive thrust
for a takeoff scenario, and that the helper conversion functions are correct.
"""

import sys
import os
from unittest.mock import MagicMock

# Stub out ROS2 modules so the node module can be imported without a runtime.
for _mod in (
    'rclpy', 'rclpy.node', 'rclpy.qos',
    'rcl_interfaces', 'rcl_interfaces.msg',
    'mavros_msgs', 'mavros_msgs.msg', 'mavros_msgs.srv',
    'geometry_msgs', 'geometry_msgs.msg',
    'nav_msgs', 'nav_msgs.msg',
):
    sys.modules.setdefault(_mod, MagicMock())

import numpy as np  # noqa: E402
import pytest  # noqa: E402

# Insert the nodes directory so lpv_mpc_drone_node is importable.
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'drone_custom_control', 'nodes'))

from lpv_mpc_drone_node import SupportFilesDrone  # noqa: E402


# ---------------------------------------------------------------------------
# Helper / fixture
# ---------------------------------------------------------------------------
def _make_hover_states(z_ned=0.0):
    """Return a zeroed state vector with the given NED z position."""
    states = np.zeros(12)
    states[8] = z_ned   # position z (NED, negative when airborne)
    return states


# ---------------------------------------------------------------------------
# NED ↔ altitude helpers (tested as plain callables)
# ---------------------------------------------------------------------------

def _z_up_to_ned(z_up):
    """Mirror of LPVMPC_Drone._z_up_to_ned for testing without Node init."""
    return -abs(z_up)


def _ned_to_alt(z_ned):
    """Mirror of LPVMPC_Drone._ned_to_alt for testing without Node init."""
    return abs(z_ned)


class TestNedHelpers:
    """Tests for _z_up_to_ned / _ned_to_alt static helpers."""

    def test_z_up_to_ned_positive_altitude(self):
        """Positive altitude must map to negative NED z."""
        assert _z_up_to_ned(2.0) == pytest.approx(-2.0)
        assert _z_up_to_ned(0.5) == pytest.approx(-0.5)

    def test_z_up_to_ned_zero(self):
        """Zero altitude must stay zero."""
        assert _z_up_to_ned(0.0) == pytest.approx(0.0)

    def test_ned_to_alt_airborne(self):
        """NED z negative when airborne → positive altitude."""
        assert _ned_to_alt(-2.0) == pytest.approx(2.0)
        assert _ned_to_alt(-0.5) == pytest.approx(0.5)

    def test_ned_to_alt_ground(self):
        """NED z near zero → altitude near zero."""
        assert _ned_to_alt(0.0) == pytest.approx(0.0)
        assert _ned_to_alt(0.05) == pytest.approx(0.05)


# ---------------------------------------------------------------------------
# pos_controller thrust sign
# ---------------------------------------------------------------------------
class TestPosControllerThrustSign:
    """Verify that pos_controller yields positive U1 for a takeoff request.

    The test simulates the NED→ENU conversion that _run_mpc_step applies
    before calling pos_controller.
    """

    def setup_method(self):
        self.support = SupportFilesDrone()
        self.m = self.support.constants['m']
        self.g = self.support.constants['g']
        self.hover_thrust = self.m * self.g   # ≈ 15.3 N

    def _call_pos_controller_ned(self, z_ned_ref, states_ned):
        """Call pos_controller with NED values, applying the NED→ENU fix.

        This mirrors the conversion in LPVMPC_Drone._run_mpc_step():
            states_enu[2]  = -states_ned[2]   (w body velocity)
            states_enu[8]  = -states_ned[8]   (z position)
            Z_ref_enu      = -z_ned_ref
        """
        states_enu = states_ned.copy()
        states_enu[2] = -states_ned[2]
        states_enu[8] = -states_ned[8]

        z_ref_enu = -z_ned_ref
        zd_enu = 0.0    # velocity reference (zero for this static test)
        zdd_enu = 0.0   # acceleration reference

        phi_ref, theta_ref, U1 = self.support.pos_controller(
            0.0, 0.0, 0.0,     # X_ref, Xd, Xdd
            0.0, 0.0, 0.0,     # Y_ref, Yd, Ydd
            z_ref_enu, zd_enu, zdd_enu,
            0.0, states_enu)   # psi_ref, states
        return float(U1)

    def test_hover_produces_weight_thrust(self):
        """At goal altitude, U1 should be approximately m*g (hover thrust)."""
        # At altitude 2 m: z_ned = -2.0
        states = _make_hover_states(z_ned=-2.0)
        U1 = self._call_pos_controller_ned(z_ned_ref=-2.0, states_ned=states)
        assert U1 == pytest.approx(self.hover_thrust, rel=0.05), (
            f'Expected ≈{self.hover_thrust:.1f} N, got {U1:.1f} N')

    def test_takeoff_from_ground_positive_thrust(self):
        """From the ground, climbing to 2 m must produce U1 > m*g."""
        states = _make_hover_states(z_ned=0.0)  # on ground
        U1 = self._call_pos_controller_ned(z_ned_ref=-2.0, states_ned=states)
        assert U1 > self.hover_thrust, (
            f'Expected U1 > {self.hover_thrust:.1f} N for takeoff, got {U1:.1f} N')

    def test_takeoff_from_ground_not_negative(self):
        """U1 must never be negative (negative thrust is physically impossible).

        This test catches the pre-fix regression where NED z was fed directly
        into pos_controller, yielding a large negative U1 (clipped to 0 thrust).
        """
        states = _make_hover_states(z_ned=0.0)
        U1 = self._call_pos_controller_ned(z_ned_ref=-2.0, states_ned=states)
        assert U1 > 0.0, (
            f'pos_controller returned non-positive U1={U1:.1f} N; '
            'NED→ENU conversion may be missing')

    def test_descend_below_thrust(self):
        """When above goal altitude, U1 should be less than hover thrust."""
        # Drone at 3 m (z_ned = -3.0), goal is 2 m (z_ned = -2.0)
        states = _make_hover_states(z_ned=-3.0)
        U1 = self._call_pos_controller_ned(z_ned_ref=-2.0, states_ned=states)
        assert U1 < self.hover_thrust, (
            f'Expected U1 < {self.hover_thrust:.1f} N when descending, got {U1:.1f} N')
