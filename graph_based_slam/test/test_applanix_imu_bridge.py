# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Regression tests for Applanix-to-Imu conversion helpers."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'convert_applanix_gsof_to_imu_bag.py'


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'convert_applanix_gsof_to_imu_bag',
        SCRIPT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_applanix_attitude_to_ros_quaternion_maps_zero_heading_to_east_facing_ros():
    """Zero Applanix attitude should become +90 deg yaw in ROS ENU."""
    module = _load_module()
    qx, qy, qz, qw = module.applanix_attitude_to_ros_quaternion_xyzw(
        roll_deg=0.0,
        pitch_deg=0.0,
        heading_deg=0.0,
    )

    assert qx == pytest.approx(0.0)
    assert qy == pytest.approx(0.0)
    assert qz == pytest.approx(math.sqrt(0.5))
    assert qw == pytest.approx(math.sqrt(0.5))


def test_applanix_motion_vectors_map_into_ros_axes():
    """Angular velocity and acceleration should follow the upstream sign convention."""
    module = _load_module()

    wx, wy, wz = module.applanix_angular_velocity_to_ros_rad_s(
        ang_rate_long_deg_s=10.0,
        ang_rate_trans_deg_s=20.0,
        ang_rate_down_deg_s=-30.0,
    )
    ax, ay, az = module.applanix_linear_acceleration_to_ros_m_s2(
        acc_long_m_s2=1.5,
        acc_trans_m_s2=-2.0,
        acc_down_m_s2=3.0,
    )

    assert wx == pytest.approx(math.radians(10.0))
    assert wy == pytest.approx(-math.radians(20.0))
    assert wz == pytest.approx(math.radians(30.0))
    assert ax == pytest.approx(1.5)
    assert ay == pytest.approx(2.0)
    assert az == pytest.approx(-3.0)


def test_orientation_covariance_from_applanix_rms_uses_radians_squared():
    """Attitude RMS should populate the diagonal covariance in rad^2."""
    module = _load_module()
    covariance = module.orientation_covariance_from_applanix_rms(
        roll_rms_deg=0.5,
        pitch_rms_deg=1.0,
        heading_rms_deg=2.0,
    )

    assert covariance.tolist() == pytest.approx(
        [
            math.radians(0.5) ** 2,
            0.0,
            0.0,
            0.0,
            math.radians(1.0) ** 2,
            0.0,
            0.0,
            0.0,
            math.radians(2.0) ** 2,
        ],
    )


def test_sec_nsec_from_ns_preserves_bag_epoch_time():
    module = _load_module()
    sec, nanosec = module.sec_nsec_from_ns(1_654_865_262_868_823_520)

    assert sec == 1_654_865_262
    assert nanosec == 868_823_520
