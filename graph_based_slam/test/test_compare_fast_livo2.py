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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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

"""Tests for the FAST-LIVO2 head-to-head scorecard."""

import importlib.util
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'compare_fast_livo2', REPO_ROOT / 'scripts' / 'compare_fast_livo2.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _manifest(system, ape, rtf, coverage):
    return {
        'system': system, 'dataset': 'hilti2022_exp04',
        'trajectory': {'ape_rmse_m': ape},
        'runtime': {'realtime_factor': rtf},
        'geometry': {'planar_coverage': coverage},
    }


def test_compare_scores_lower_and_higher_metrics():
    result = MODULE.compare(
        _manifest('lidarslam_ros2', 0.07, 0.8, 0.54),
        _manifest('FAST-LIVO2', 0.08, 0.6, 0.50))
    assert result['score'] == {
        'ours': 2, 'fast_livo2': 1, 'tie': 0, 'missing': 5}
    assert result['overall'] == 'ours'
    assert result['metrics'][0]['delta_percent'] == pytest.approx(12.5)


def test_compare_uses_relative_tie_tolerance():
    result = MODULE.compare(
        _manifest('lidarslam_ros2', 1.005, 1.0, 0.5),
        _manifest('FAST-LIVO2', 1.0, 1.0, 0.5), tie_rel=0.01)
    assert result['score']['tie'] == 3


def test_compare_rejects_different_datasets():
    ours = _manifest('lidarslam_ros2', 1.0, 1.0, 0.5)
    rival = _manifest('FAST-LIVO2', 1.0, 1.0, 0.5)
    rival['dataset'] = 'different'
    with pytest.raises(ValueError, match='dataset must match'):
        MODULE.compare(ours, rival)


def test_markdown_exposes_missing_metrics_and_winner():
    result = MODULE.compare(
        _manifest('lidarslam_ros2', 0.07, 0.8, 0.54),
        _manifest('FAST-LIVO2', 0.08, 0.6, 0.50))
    text = MODULE.markdown(result)
    assert 'Overall: **ours**' in text
    assert '| APE RMSE | lower | 0.07 m | 0.08 m | +12.50% | **ours** |' in text
    assert '| Peak RSS | lower | — | — | — | **missing** |' in text
