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

"""Regression tests for Applanix GSOF50 inspection helpers."""

from __future__ import annotations

import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'inspect_applanix_gsof50_quality.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('inspect_applanix_gsof50_quality', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_summary_counts_modes_and_rms_distributions():
    module = _load_module()
    records = [
        module.ApplanixGsof50Record(
            stamp_sec=1.0,
            gnss_status=4,
            imu_alignment=3,
            pos_rms_north_m=0.02,
            pos_rms_east_m=0.03,
            heading_rms_deg=0.1,
        ),
        module.ApplanixGsof50Record(
            stamp_sec=2.0,
            gnss_status=2,
            imu_alignment=3,
            pos_rms_north_m=0.5,
            pos_rms_east_m=0.4,
            heading_rms_deg=0.2,
        ),
        module.ApplanixGsof50Record(
            stamp_sec=3.0,
            gnss_status=6,
            imu_alignment=3,
            pos_rms_north_m=1.0,
            pos_rms_east_m=2.0,
            heading_rms_deg=0.3,
        ),
    ]

    summary = module.summarize_applanix_gsof50_records(records)

    assert summary['total_messages'] == 3
    assert summary['gnss_mode_counts'] == {
        'DIFFERENTIAL_GPS_SPS': 1,
        'DIRECT_GEOREFERENCING_MODE': 1,
        'FIXED_RTK_MODE': 1,
    }
    assert summary['imu_alignment_counts'] == {'ALIGNED': 3}
    assert summary['horizontal_rms_m'] == {
        'min': 0.03605551275463989,
        'p50': 0.6403124237432849,
        'p90': 2.23606797749979,
        'p95': 2.23606797749979,
        'max': 2.23606797749979,
    }
    assert summary['heading_rms_deg'] == {
        'min': 0.1,
        'p50': 0.2,
        'p90': 0.3,
        'p95': 0.3,
        'max': 0.3,
    }


def test_summary_handles_non_finite_rms_values():
    module = _load_module()
    records = [
        module.ApplanixGsof50Record(
            stamp_sec=1.0,
            gnss_status=4,
            imu_alignment=4,
            pos_rms_north_m=float('nan'),
            pos_rms_east_m=0.1,
            heading_rms_deg=float('nan'),
        ),
    ]

    summary = module.summarize_applanix_gsof50_records(records)

    assert summary['total_messages'] == 1
    assert summary['gnss_mode_counts'] == {'FIXED_RTK_MODE': 1}
    assert summary['imu_alignment_counts'] == {'FULL_NAV': 1}
    assert summary['horizontal_rms_m'] is None
    assert summary['heading_rms_deg'] is None
