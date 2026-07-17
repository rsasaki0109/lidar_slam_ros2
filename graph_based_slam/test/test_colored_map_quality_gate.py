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
"""Tests for the integrated coloured-map quality gate."""

import importlib.util
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'check_colored_map_quality',
    REPO_ROOT / 'scripts' / 'check_colored_map_quality.py')
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


def _reports():
    return {
        'trajectory': {'evo': {'ape': {'rmse': 0.07}}},
        'geometry': {'map_quality_report': {'plane_metrics': {
            'thickness_rms_mean_m': 0.06,
            'thickness_rms_p95_m': 0.11,
            'planar_coverage': 0.54}}},
        'alignment': {'weighted_median_px': 6.7,
                      'weighted_inlier_2px': 0.26},
        'colour': {'rgb_l2_median': 36.4, 'rgb_l2_inlier_20': 0.35,
                   'heldout_scored_fraction': 0.999},
        'appearance': {'coverage': 0.998, 'chroma_retention': 1.01,
                       'roughness': {'roughness_median': 2.05,
                                     'roughness_p90': 12.3}},
    }


def _profile(enforcement='blocking'):
    return {'colored_map_quality_profile': {
        'name': 'test', 'enforcement': enforcement, 'thresholds': {
            'ape_rmse_max_m': 0.1,
            'thickness_rms_mean_max_m': 0.07,
            'planar_coverage_min': 0.5,
            'alignment_median_max_px': 7.0,
            'heldout_rgb_median_max': 40.0,
            'heldout_scored_fraction_min': 0.95}}}


def test_all_quality_domains_pass():
    result = gate.evaluate(_reports(), _profile())
    assert result['overall'] == 'OK'
    assert result['violations'] == 0
    assert {row['source'] for row in result['checks']} == {
        'trajectory', 'geometry', 'alignment', 'colour'}


def test_blocking_profile_fails_a_regression():
    reports = _reports()
    reports['colour']['rgb_l2_median'] = 50.0
    result = gate.evaluate(reports, _profile())
    assert result['overall'] == 'FAILED'
    assert result['violations'] == 1


def test_report_only_records_violation_without_failure():
    reports = _reports()
    reports['trajectory']['evo']['ape']['rmse'] = 1.0
    result = gate.evaluate(reports, _profile('report_only'))
    assert result['overall'] == 'REPORT_ONLY'
    assert result['violations'] == 1


def test_missing_metric_is_actionable():
    reports = _reports()
    del reports['alignment']['weighted_median_px']
    with pytest.raises(gate.QualityGateError, match='weighted_median_px'):
        gate.evaluate(reports, _profile())


def test_appearance_domain_gates_pepper_and_coverage():
    profile = {'colored_map_quality_profile': {
        'name': 'test', 'enforcement': 'blocking', 'thresholds': {
            'appearance_coverage_min': 0.95,
            'appearance_roughness_median_max': 2.5,
            'appearance_roughness_p90_max': 15.0,
            'appearance_chroma_retention_min': 0.9}}}
    result = gate.evaluate(_reports(), profile)
    assert result['overall'] == 'OK'
    peppered = _reports()
    peppered['appearance']['roughness']['roughness_p90'] = 18.5
    peppered['appearance']['coverage'] = 0.768
    result = gate.evaluate(peppered, profile)
    assert result['violations'] == 2


def test_appearance_threshold_without_report_is_actionable():
    reports = _reports()
    del reports['appearance']
    profile = {'colored_map_quality_profile': {
        'name': 'test', 'enforcement': 'report_only', 'thresholds': {
            'appearance_coverage_min': 0.95}}}
    with pytest.raises(gate.QualityGateError, match='appearance-report'):
        gate.evaluate(reports, profile)
