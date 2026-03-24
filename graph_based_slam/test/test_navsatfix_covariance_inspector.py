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

"""Regression tests for NavSatFix covariance inspection helpers."""

from __future__ import annotations

import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'inspect_navsatfix_covariance.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('inspect_navsatfix_covariance', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_summary_counts_rtk_like_non_rtk_and_invalid_fixes():
    module = _load_module()
    records = [
        module.NavSatFixRecord(
            stamp_sec=1.0,
            latitude=0.0,
            longitude=0.0,
            altitude=10.0,
            status=0,
            covariance_type=0,
            position_covariance=(0.0,) * 9,
        ),
        module.NavSatFixRecord(
            stamp_sec=2.0,
            latitude=35.0,
            longitude=139.0,
            altitude=10.0,
            status=2,
            covariance_type=2,
            position_covariance=(0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.25),
        ),
        module.NavSatFixRecord(
            stamp_sec=3.0,
            latitude=35.0001,
            longitude=139.0001,
            altitude=11.0,
            status=0,
            covariance_type=2,
            position_covariance=(1.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 9.0),
        ),
        module.NavSatFixRecord(
            stamp_sec=4.0,
            latitude=35.0002,
            longitude=139.0002,
            altitude=12.0,
            status=0,
            covariance_type=0,
            position_covariance=(0.0,) * 9,
        ),
    ]

    summary = module.summarize_navsatfix_records(records)

    assert summary['total_messages'] == 4
    assert summary['usable_fixes'] == 3
    assert summary['invalid_fixes'] == 1
    assert summary['invalid_reason_counts'] == {'zero_origin': 1}
    assert summary['covariance_known'] == 2
    assert summary['covariance_unknown'] == 2
    assert summary['usable_covariance_known'] == 2
    assert summary['usable_covariance_unknown'] == 1
    assert summary['rtk_like'] == 1
    assert summary['non_rtk'] == 1
    assert summary['status_counts'] == {'0': 3, '2': 1}
    assert summary['covariance_type_counts'] == {'0': 2, '2': 2}
    assert summary['horizontal_stddev_m'] == {
        'min': 0.2,
        'p50': 1.1,
        'p90': 2.0,
        'p95': 2.0,
        'max': 2.0,
    }


def test_summary_clamps_tiny_variance_before_rtk_classification():
    module = _load_module()
    records = [
        module.NavSatFixRecord(
            stamp_sec=1.0,
            latitude=35.0,
            longitude=139.0,
            altitude=10.0,
            status=2,
            covariance_type=2,
            position_covariance=(1.0e-6, 0.0, 0.0, 0.0, 1.0e-6, 0.0, 0.0, 0.0, 1.0e-6),
        ),
    ]

    summary = module.summarize_navsatfix_records(records, min_variance_m2=0.01)

    assert summary['rtk_like'] == 1
    assert summary['non_rtk'] == 0
    assert summary['horizontal_stddev_m'] == {
        'min': 0.1,
        'p50': 0.1,
        'p90': 0.1,
        'p95': 0.1,
        'max': 0.1,
    }
