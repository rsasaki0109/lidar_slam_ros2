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

"""Regression tests for the map gravity alignment check."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'check_map_gravity_alignment.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('check_map_gravity_alignment', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _quat_about_x(angle_rad):
    return np.array([math.sin(angle_rad / 2.0), 0.0, 0.0, math.cos(angle_rad / 2.0)])


def _trajectory(quat_xyzw):
    times = np.arange(0.0, 10.0, 0.5)
    count = len(times)
    return np.column_stack([times, times, np.zeros((count, 2)), np.tile(quat_xyzw, (count, 1))])


def _still_imu(up_in_sensor):
    times = np.arange(0.0, 10.0, 0.01)
    return times, np.tile(9.81 * np.asarray(up_in_sensor), (len(times), 1))


def test_tilted_sensor_in_an_unlevelled_frame_reports_the_tilt():
    module = _load_module()
    tilt = math.radians(7.0)
    times, accel = _still_imu([0.0, math.sin(tilt), math.cos(tilt)])

    result = module.estimate_map_tilt(
        _trajectory(_quat_about_x(0.0)), times, accel, np.array([0.0, 0.0, 0.0, 1.0])
    )

    assert abs(result['tilt_deg'] - 7.0) < 1e-6
    assert result['imu_samples'] == 951


def test_levelled_frame_reports_no_tilt():
    module = _load_module()
    tilt = math.radians(7.0)
    times, accel = _still_imu([0.0, math.sin(tilt), math.cos(tilt)])

    # The pose rotates the sensor's measured up back onto the map's +z.
    result = module.estimate_map_tilt(
        _trajectory(_quat_about_x(tilt)), times, accel, np.array([0.0, 0.0, 0.0, 1.0])
    )

    assert result['tilt_deg'] < 1e-6


def test_imu_extrinsic_is_applied():
    module = _load_module()
    tilt = math.radians(7.0)
    times, accel = _still_imu([0.0, math.sin(tilt), math.cos(tilt)])

    # The same reading from an IMU mounted 7 deg about x relative to a level base.
    result = module.estimate_map_tilt(
        _trajectory(_quat_about_x(0.0)), times, accel, _quat_about_x(tilt)
    )

    assert result['tilt_deg'] < 1e-6


def test_hint_only_above_the_threshold():
    module = _load_module()
    base = {'imu_topic': '/livox/imu', 'span_sec': 266.0, 'max_tilt_deg': 3.0}

    assert module.tilt_hint({**base, 'tilt_deg': 1.6, 'level': True}) is None
    hint = module.tilt_hint({**base, 'tilt_deg': 8.0, 'level': False})
    assert 'tilted 8.0 deg' in hint
    assert 'initialization_phase' in hint
