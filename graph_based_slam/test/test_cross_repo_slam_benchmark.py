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


def test_dense_graph_trajectory_is_a_required_preprocessing_step(tmp_path):
    command = cross.densify_command(
        tmp_path / 'raw.tum', tmp_path / 'sparse.tum', tmp_path / 'dense.tum')
    assert command[1].endswith('densify_corrected_trajectory.py')
    assert command[command.index('--raw') + 1].endswith('raw.tum')
    assert command[command.index('--corrected') + 1].endswith('sparse.tum')


def test_manifest_marks_missing_evidence_and_computes_graph_delta(tmp_path):
    for name in ('profile', 'gt', 'raw', 'corrected'):
        (tmp_path / name).write_text(name)
    summary = {'methods': [
        {'name': 'frontend_raw', 'ate_m': 0.10},
        {'name': 'graph_corrected', 'ate_m': 0.08},
    ]}
    profile = {'name': 'test', 'enforcement': 'report_only',
               'adoption_policy': {'minimum_improved_datasets': 2},
               'required_success_metrics': {'runtime.process_exit_status': 0}}
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
    assert manifest['evidence']['failed_success_metrics'][0]['value'] is None
    assert manifest['verdict'] == 'INCOMPLETE'


def test_metric_delta_treats_numerical_noise_as_tie():
    summary = {'methods': [
        {'name': 'frontend_raw', 'ate_m': 0.07155989354899428},
        {'name': 'graph_corrected', 'ate_m': 0.07155989354899392},
    ]}
    assert cross.metric_delta(summary, 'ate_m')['improved'] is None


def test_report_only_manifest_records_primary_observation(tmp_path):
    (tmp_path / 'profile').write_text('profile')
    profile = {'name': 'test', 'required_success_metrics': {
        'runtime.process_exit_status': 0}}
    dataset = {
        'primary_metric_path': 'colour.stats.chroma',
        'required_reports': ['colour', 'runtime'],
        'required_metrics': ['colour.stats.chroma', 'runtime.peak_rss_mb'],
    }
    reports = {
        'colour': {'stats': {'chroma': 0.61}},
        'runtime': {'peak_rss_mb': 1000.0, 'process_exit_status': 0},
    }

    manifest = cross.build_manifest(
        profile, 'rgb', dataset, tmp_path, {'profile': tmp_path / 'profile'},
        {'methods': []}, reports)

    assert manifest['evidence']['complete'] is True
    assert manifest['evidence']['missing_reports'] == []
    assert manifest['primary_delta'] == {
        'mode': 'observation', 'metric': 'colour.stats.chroma',
        'value': 0.61, 'improved': None}
    assert manifest['verdict'] == 'RECORDED'


def test_aist_profile_is_report_only_and_freezes_all_quality_axes():
    _, dataset = cross.load_profile(
        ROOT / 'configs/slam_benchmark_profiles/public_suite_v1.yaml',
        'aist_ouster_rgb')

    assert 'trajectory' not in dataset['required_reports']
    assert set(dataset['required_reports']) == {
        'geometry', 'alignment', 'colour', 'runtime'}
    assert dataset['primary_metric_path'].startswith('colour.')


def test_git_provenance_records_revision_and_tracked_diff_hash():
    provenance = cross.git_provenance(ROOT)
    assert len(provenance['revision']) == 40
    assert len(provenance['tracked_diff_sha256']) == 64
    assert isinstance(provenance['tracked_dirty'], bool)
