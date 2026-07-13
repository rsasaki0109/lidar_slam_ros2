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

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'cross_repo', ROOT / 'scripts/run_cross_repo_slam_benchmark.py')
cross = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cross)


def test_profile_freezes_hilti_position_only_contract():
    profile, dataset = cross.load_profile(
        ROOT / 'configs/slam_benchmark_profiles/public_suite_v1.yaml',
        'hilti_exp04')
    assert profile['name'] == 'public_suite_v1'
    assert dataset['alignment'] == 'se3'
    assert dataset['position_only_reference'] is True
    assert 'rotational_rpe' in dataset['forbidden_claims']


def test_zoo_command_compares_raw_and_corrected_and_preserves_gt_policy(tmp_path):
    config = {'alignment': 'se3', 'position_only_reference': True,
              'segment_length_m': 10.0, 'max_time_difference_s': 0.05}
    command = cross.zoo_command(
        tmp_path / 'zoo', config, tmp_path / 'gt.tum',
        tmp_path / 'raw.tum', tmp_path / 'corrected.tum',
        tmp_path / 'summary.json')
    assert 'frontend_raw:' + str(tmp_path / 'raw.tum') in command
    assert 'graph_corrected:' + str(tmp_path / 'corrected.tum') in command
    assert '--position-only-reference' in command


def test_manifest_marks_missing_evidence_and_computes_graph_delta(tmp_path):
    for name in ('profile', 'gt', 'raw', 'corrected'):
        (tmp_path / name).write_text(name)
    summary = {'methods': [
        {'name': 'frontend_raw', 'ate_m': 0.10},
        {'name': 'graph_corrected', 'ate_m': 0.08},
    ]}
    profile = {'name': 'test', 'enforcement': 'report_only',
               'adoption_policy': {'minimum_improved_datasets': 2}}
    dataset = {'primary_metric': 'ate_m',
               'required_reports': ['trajectory', 'geometry', 'runtime'],
               'required_metrics': ['trajectory.frontend_raw.ate_m',
                                    'runtime.peak_rss_mb']}
    manifest = cross.build_manifest(
        profile, 'case', dataset, tmp_path,
        {name: tmp_path / name for name in ('profile', 'gt', 'raw', 'corrected')},
        summary, {'geometry': {'ok': True}})
    assert manifest['primary_delta']['change_percent'] == pytest.approx(-20.0)
    assert manifest['primary_delta']['improved'] is True
    assert manifest['evidence']['missing_reports'] == ['runtime']
    assert manifest['evidence']['missing_metrics'] == ['runtime.peak_rss_mb']
    assert manifest['verdict'] == 'INCOMPLETE'
