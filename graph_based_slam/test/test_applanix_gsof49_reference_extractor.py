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

"""Tests for the Applanix GSOF49 reference extractor."""

from __future__ import annotations

import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'extract_applanix_gsof49_reference.py'


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'extract_applanix_gsof49_reference',
        SCRIPT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_heading_to_enu_yaw_matches_cardinal_directions():
    """North-clockwise headings should map into ENU yaw."""
    module = _load_module()

    assert module.heading_deg_to_enu_yaw_deg(0.0) == 90.0
    assert module.heading_deg_to_enu_yaw_deg(90.0) == 0.0
    assert module.heading_deg_to_enu_yaw_deg(180.0) == -90.0


def test_lla_origin_maps_close_to_zero():
    """The chosen origin should map to approximately zero in ENU."""
    module = _load_module()
    x, y, z = module.lla_to_enu(
        40.81352945577717,
        29.3602286349187,
        51.20780572262455,
        40.81352945577717,
        29.3602286349187,
        51.20780572262455,
    )

    assert abs(x) < 1e-6
    assert abs(y) < 1e-6
    assert abs(z) < 1e-6


def test_is_usable_fix_filters_bad_status_and_zero_origin():
    """Reference extraction should drop unusable fixes."""
    module = _load_module()

    usable, reason = module.is_usable_fix(
        latitude_deg=0.0,
        longitude_deg=0.0,
        altitude_m=10.0,
        imu_alignment=3,
        gnss_status=4,
        aligned_value=3,
        fix_not_available_value=0,
        gnss_unknown_value=7,
    )
    assert usable is False
    assert reason == 'zero_origin'

    usable, reason = module.is_usable_fix(
        latitude_deg=40.0,
        longitude_deg=29.0,
        altitude_m=10.0,
        imu_alignment=2,
        gnss_status=4,
        aligned_value=3,
        fix_not_available_value=0,
        gnss_unknown_value=7,
    )
    assert usable is False
    assert reason == 'imu_not_aligned'

    usable, reason = module.is_usable_fix(
        latitude_deg=40.0,
        longitude_deg=29.0,
        altitude_m=10.0,
        imu_alignment=3,
        gnss_status=4,
        aligned_value=3,
        fix_not_available_value=0,
        gnss_unknown_value=7,
    )
    assert usable is True
    assert reason == 'ok'
