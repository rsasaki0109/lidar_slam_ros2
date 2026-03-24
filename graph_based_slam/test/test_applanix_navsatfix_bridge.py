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

"""Regression tests for Applanix-to-NavSatFix conversion helpers."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'convert_applanix_gsof_to_navsatfix_bag.py'


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'convert_applanix_gsof_to_navsatfix_bag',
        SCRIPT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_applanix_gnss_status_to_navsat_status_maps_fix_classes():
    module = _load_module()
    assert module.applanix_gnss_status_to_navsat_status(module.APPLANIX_FIX_NOT_AVAILABLE) == -1
    assert module.applanix_gnss_status_to_navsat_status(module.APPLANIX_GNSS_UNKNOWN) == -2
    assert module.applanix_gnss_status_to_navsat_status(module.APPLANIX_GNSS_SPS_MODE) == 0
    assert module.applanix_gnss_status_to_navsat_status(module.APPLANIX_DIFFERENTIAL_GPS_SPS) == 1
    assert module.applanix_gnss_status_to_navsat_status(module.APPLANIX_FIXED_RTK_MODE) == 2
    assert module.applanix_gnss_status_to_navsat_status(module.APPLANIX_FLOAT_RTK) == 2
    assert (
        module.applanix_gnss_status_to_navsat_status(
            module.APPLANIX_DIRECT_GEOREFERENCING_MODE,
        ) == 2
    )


def test_covariance_from_applanix_rms_builds_enu_diagonal():
    module = _load_module()
    covariance, covariance_type = module.covariance_from_applanix_rms(
        east_rms_m=0.4,
        north_rms_m=0.3,
        down_rms_m=1.2,
    )
    assert covariance.tolist() == pytest.approx(
        [0.16, 0.0, 0.0, 0.0, 0.09, 0.0, 0.0, 0.0, 1.44],
    )
    assert covariance_type == 2


def test_sec_nsec_from_ns_preserves_bag_epoch_time():
    module = _load_module()
    sec, nanosec = module.sec_nsec_from_ns(1_654_865_262_868_823_520)

    assert sec == 1_654_865_262
    assert nanosec == 868_823_520
