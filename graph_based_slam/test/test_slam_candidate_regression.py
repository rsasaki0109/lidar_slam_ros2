# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for the Phase 7 baseline/candidate SLAM regression gate."""

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'candidate_gate', ROOT / 'scripts/evaluate_slam_candidate_regression.py')
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


def _manifest(dataset, ate, rpe, thickness, coverage, runtime, complete=True):
    return {
        'profile': 'public_suite_v1',
        'dataset': dataset,
        'dataset_contract': {'dataset': dataset, 'alignment': 'se3'},
        'inputs': {
            'gt_tum': {'sha256': f'{dataset}-gt'},
            'raw_artifact.bag': {'sha256': f'{dataset}-bag'},
        },
        'evidence': {'complete': complete},
        'trajectory': {'methods': [{
            'name': 'graph_corrected', 'ate_m': ate, 'rpe_trans_pct': rpe}]},
        'reports': {
            'geometry': {'map_quality_report': {'plane_metrics': {
                'thickness_rms_mean_m': thickness,
                'planar_coverage': coverage}}},
            'runtime': {'realtime_factor': runtime},
        },
    }


def _inputs(tmp_path, mid_candidate=None, hilti_candidate=None):
    baseline_documents = {
        'mid360_public': _manifest('mid360_public', 1.0, 2.0, 0.05, 0.40, 0.8),
        'hilti_exp04': _manifest('hilti_exp04', 0.5, None, 0.04, 0.50, 0.7),
        'rtkslam_construction_seq2': _manifest(
            'rtkslam_construction_seq2', 0.20, None, 0.08, 0.42, 0.9),
    }
    candidate_documents = {
        'mid360_public': mid_candidate or _manifest(
            'mid360_public', 0.8, 1.9, 0.049, 0.401, 0.85),
        'hilti_exp04': hilti_candidate or _manifest(
            'hilti_exp04', 0.5, None, 0.04, 0.50, 0.72),
        'rtkslam_construction_seq2': _manifest(
            'rtkslam_construction_seq2', 0.18, None, 0.079, 0.421, 0.95),
    }
    baselines = {}
    candidates = {}
    for name, document in baseline_documents.items():
        baselines[name] = (tmp_path / f'{name}_baseline.json', document)
    for name, document in candidate_documents.items():
        candidates[name] = (tmp_path / f'{name}_candidate.json', document)
    return baselines, candidates


def test_phase7_profile_passes_positive_mid360_and_unchanged_hilti(tmp_path):
    """The promotion shape requires improvement plus a non-regressed holdout."""
    profile = gate._load_profile(
        ROOT / 'configs/slam_benchmark_profiles/phase7_regression_v1.yaml')
    baselines, candidates = _inputs(tmp_path)

    report = gate.evaluate(profile, baselines, candidates)

    assert report['verdict'] == 'ADOPT_CANDIDATE'
    assert all(report['checks'].values())
    assert report['datasets'][0]['metrics'][0]['change_percent'] == pytest.approx(-20.0)


def test_rejects_runtime_regression_beyond_budget(tmp_path):
    """A candidate slower than the frozen ten-percent budget is rejected."""
    candidate = _manifest('mid360_public', 0.8, 1.9, 0.049, 0.401, 0.89)
    profile = gate._load_profile(
        ROOT / 'configs/slam_benchmark_profiles/phase7_regression_v1.yaml')
    baselines, candidates = _inputs(tmp_path, mid_candidate=candidate)

    report = gate.evaluate(profile, baselines, candidates)

    mid360 = report['datasets'][0]
    runtime = next(row for row in mid360['metrics']
                   if row['path'] == 'runtime.realtime_factor')
    assert runtime['passed'] is False
    assert report['verdict'] == 'REJECT_CANDIDATE'


def test_rejects_missing_rpe_and_incomplete_manifest(tmp_path):
    """Missing candidate evidence cannot silently turn into a passing metric."""
    candidate = _manifest(
        'mid360_public', 0.8, None, 0.049, 0.401, 0.8, complete=False)
    profile = gate._load_profile(
        ROOT / 'configs/slam_benchmark_profiles/phase7_regression_v1.yaml')
    baselines, candidates = _inputs(tmp_path, mid_candidate=candidate)

    report = gate.evaluate(profile, baselines, candidates)

    mid360 = report['datasets'][0]
    assert mid360['checks']['manifests_complete'] is False
    assert mid360['checks']['all_metrics_within_budget'] is False


def test_rejects_when_required_dataset_pair_is_missing(tmp_path):
    """Both the positive sequence and negative holdout are mandatory."""
    profile = gate._load_profile(
        ROOT / 'configs/slam_benchmark_profiles/phase7_regression_v1.yaml')
    baselines, candidates = _inputs(tmp_path)
    candidates.pop('hilti_exp04')

    report = gate.evaluate(profile, baselines, candidates)

    assert report['missing_datasets'] == ['hilti_exp04']
    assert report['checks']['all_required_datasets_present'] is False


def test_rejects_different_raw_capture_hashes(tmp_path):
    """Metric deltas from two different bag captures are not comparable."""
    profile = gate._load_profile(
        ROOT / 'configs/slam_benchmark_profiles/phase7_regression_v1.yaml')
    baselines, candidates = _inputs(tmp_path)
    candidates['mid360_public'][1]['inputs']['raw_artifact.bag']['sha256'] = 'other'

    report = gate.evaluate(profile, baselines, candidates)

    mid360 = report['datasets'][0]
    assert mid360['checks']['inputs_comparable'] is False
    assert mid360['comparison_issues'] == ['raw_artifact.bag: hash differs']
    assert report['verdict'] == 'REJECT_CANDIDATE'


def test_hilti_short_runtime_uses_frozen_absolute_budget(tmp_path):
    """Avoid unstable percentages for the four-second negative holdout."""
    profile = gate._load_profile(
        ROOT / 'configs/slam_benchmark_profiles/phase7_regression_v1.yaml')
    baselines, candidates = _inputs(tmp_path)
    baselines['hilti_exp04'][1]['reports']['runtime']['realtime_factor'] = 0.0115
    candidates['hilti_exp04'][1]['reports']['runtime']['realtime_factor'] = 0.0130

    report = gate.evaluate(profile, baselines, candidates)

    hilti = report['datasets'][1]
    runtime_metric = next(
        row for row in hilti['metrics'] if row['path'] == 'runtime.realtime_factor')
    assert runtime_metric['change_percent'] > 10.0
    assert runtime_metric['allowed_regression_absolute'] == 0.005
    assert runtime_metric['passed'] is True


def test_markdown_contains_review_table(tmp_path):
    """The gate writes a compact human-reviewable metric table."""
    profile = gate._load_profile(
        ROOT / 'configs/slam_benchmark_profiles/phase7_regression_v1.yaml')
    report = gate.evaluate(profile, *_inputs(tmp_path))

    summary = gate.markdown(report)

    assert '# SLAM candidate regression: ADOPT_CANDIDATE' in summary
    assert '| mid360_public | ATE RMSE (m) |' in summary


def test_rtkslam_second_positive_must_improve_surveyed_ate(tmp_path):
    """A second positive dataset cannot pass by merely avoiding regression."""
    profile = gate._load_profile(
        ROOT / 'configs/slam_benchmark_profiles/phase7_regression_v1.yaml')
    baselines, candidates = _inputs(tmp_path)
    candidates['rtkslam_construction_seq2'][1]['trajectory']['methods'][0][
        'ate_m'] = 0.20

    report = gate.evaluate(profile, baselines, candidates)

    rtkslam = next(
        row for row in report['datasets']
        if row['dataset'] == 'rtkslam_construction_seq2')
    assert rtkslam['checks']['all_metrics_within_budget'] is True
    assert rtkslam['checks']['primary_improved'] is False
    assert report['verdict'] == 'REJECT_CANDIDATE'
