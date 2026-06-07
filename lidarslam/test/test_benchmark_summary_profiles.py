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

"""Regression tests for release-profile gate evaluation in benchmark_summary."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import textwrap

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'benchmark_summary.py'
DEFAULT_PROFILE_YAML = REPO_ROOT / 'scripts' / 'release_profiles.yaml'


def _load_module():
    spec = importlib.util.spec_from_file_location('benchmark_summary', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _rec(**overrides):
    base = {
        'run': overrides.get('run', 'r0'),
        'bag': overrides.get('bag', 'demo_bag'),
        'points_topic': overrides.get('points_topic', '/livox/lidar'),
        'ape_ref_kind': overrides.get('ape_ref_kind', 'ground_truth'),
        'ape_ref_src': overrides.get('ape_ref_src', 'leica_prism_gt'),
        'ape_rmse_m': overrides.get('ape_rmse_m', '0.500'),
        'ape_pairs': overrides.get('ape_pairs', 500),
    }
    return base


def test_default_release_profiles_yaml_loads():
    """The shipped scripts/release_profiles.yaml must parse without errors."""
    module = _load_module()
    profiles = module.load_release_profiles(DEFAULT_PROFILE_YAML)
    names = [p['name'] for p in profiles]
    assert 'newer_college_math_hard' in names
    assert 'mid360_vs_glim' in names


def test_research_track_profiles_graduated_to_blocking():
    """v0.4 decision (2026-06-07): the three former research-track profiles
    graduated from report_only_until (WARN) to blocking (FAIL). Pin that no
    shipped profile downgrades a regression anymore.
    """
    module = _load_module()
    profiles = module.load_release_profiles(DEFAULT_PROFILE_YAML)
    by_name = {p['name']: p for p in profiles}
    graduated = (
        'ntu_viral_tnp_01',
        'mid360_vs_glim',
        'leo_drive_applanix_velodyne_cross',
    )
    for name in graduated:
        assert name in by_name, name
        assert by_name[name].get('report_only_until') is None, (
            f'{name} must block (no report_only_until) as of v0.4')
    # No shipped profile may carry report_only_until at the v0.4 cut.
    assert all(p.get('report_only_until') is None for p in profiles)


def test_graduated_profile_fails_gate_on_regression():
    """A mid360_vs_glim regression past its 4.0 m cross-val pass must now FAIL
    (blocking), not WARN -- the observable effect of the graduation.
    """
    module = _load_module()
    profiles = module.load_release_profiles(DEFAULT_PROFILE_YAML)
    mid360 = next(p for p in profiles if p['name'] == 'mid360_vs_glim')
    records = [
        _rec(run='regressed', points_topic='/livox/lidar',
             ape_ref_kind='cross_validation', ape_ref_src='glim_mid360_reference',
             ape_rmse_m='5.00', ape_pairs=600),
    ]
    [result] = module.evaluate_release_profiles([mid360], records)
    assert result['status'] == 'FAIL'


def test_load_release_profiles_rejects_unknown_metric(tmp_path: Path):
    module = _load_module()
    bad = tmp_path / 'bad.yaml'
    bad.write_text(
        textwrap.dedent(
            """
            release_profiles:
              - name: x
                metric: not_a_metric
                pass: 1.0
            """
        )
    )
    with pytest.raises(ValueError, match='metric must be one of'):
        module.load_release_profiles(bad)


def test_load_release_profiles_rejects_duplicate_name(tmp_path: Path):
    module = _load_module()
    bad = tmp_path / 'bad.yaml'
    bad.write_text(
        textwrap.dedent(
            """
            release_profiles:
              - name: dup
                metric: ape_rmse_gt_m
                pass: 1.0
              - name: dup
                metric: ape_rmse_gt_m
                pass: 1.0
            """
        )
    )
    with pytest.raises(ValueError, match='duplicate profile name'):
        module.load_release_profiles(bad)


def test_evaluate_pass_picks_best_matching_run():
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 1.0,
        'match': {'bag_name_contains': 'tnp_01', 'reference_kind': 'ground_truth'},
    }
    records = [
        _rec(run='a', bag='tnp_01_run', ape_rmse_m='0.95'),
        _rec(run='b', bag='tnp_01_run', ape_rmse_m='0.80'),
        _rec(run='c', bag='unrelated_run', ape_rmse_m='0.10'),
    ]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['status'] == 'PASS'
    assert result['best_run'] == 'b'
    assert result['best_value'] == pytest.approx(0.80)


def test_evaluate_target_met_when_under_target():
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 1.0,
        'target': 0.5,
        'match': {'bag_name_contains': 'foo'},
    }
    records = [_rec(run='a', bag='foo_run', ape_rmse_m='0.30')]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['status'] == 'TARGET_MET'


def test_evaluate_fail_when_over_pass_without_report_only():
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 0.10,
        'match': {'bag_name_contains': 'foo'},
    }
    records = [_rec(run='a', bag='foo_run', ape_rmse_m='0.95')]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['status'] == 'FAIL'


def test_evaluate_warn_when_over_pass_with_report_only_until():
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 0.10,
        'report_only_until': 'v0.4',
        'match': {'bag_name_contains': 'foo'},
    }
    records = [_rec(run='a', bag='foo_run', ape_rmse_m='0.95')]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['status'] == 'WARN'


def test_evaluate_no_data_when_no_matches():
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 0.10,
        'match': {'bag_name_contains': 'nothing'},
    }
    records = [_rec(run='a', bag='foo', ape_rmse_m='0.5')]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['status'] == 'NO_DATA'
    assert result['best_run'] is None


def test_metric_ape_rmse_gt_m_skips_cross_validation():
    """ape_rmse_gt_m must only score ground_truth references."""
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_gt_m',
        'pass': 1.0,
        'match': {'bag_name_contains': 'foo'},
    }
    records = [
        _rec(run='cv', bag='foo', ape_ref_kind='cross_validation', ape_rmse_m='0.10'),
        _rec(run='gt', bag='foo', ape_ref_kind='ground_truth', ape_rmse_m='0.50'),
    ]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['best_run'] == 'gt'
    assert result['best_value'] == pytest.approx(0.50)


def test_min_ape_pairs_filters_incomplete_runs():
    """Match should drop runs whose APE was computed on too few pose pairs."""
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_vs_reference_m',
        'pass': 4.0,
        'match': {
            'points_topic': '/livox/lidar',
            'reference_kind': 'cross_validation',
            'min_ape_pairs': 400,
        },
    }
    records = [
        _rec(run='partial', ape_ref_kind='cross_validation', ape_rmse_m='0.11', ape_pairs=119),
        _rec(run='full', ape_ref_kind='cross_validation', ape_rmse_m='3.45', ape_pairs=580),
    ]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['best_run'] == 'full'
    assert result['best_value'] == pytest.approx(3.45)


def test_profile_match_combines_predicates():
    """All predicates in match must hold simultaneously (AND semantics)."""
    module = _load_module()
    profile = {
        'name': 'p',
        'metric': 'ape_rmse_vs_reference_m',
        'pass': 1.0,
        'match': {
            'points_topic': '/livox/lidar',
            'reference_source_contains': 'glim_mid360',
        },
    }
    records = [
        _rec(run='other_topic',
             points_topic='/os1/points',
             ape_ref_kind='cross_validation',
             ape_ref_src='glim_mid360_reference',
             ape_rmse_m='0.1'),
        _rec(run='other_ref',
             points_topic='/livox/lidar',
             ape_ref_kind='cross_validation',
             ape_ref_src='applanix_gsof49_reference',
             ape_rmse_m='0.2'),
        _rec(run='match_me',
             points_topic='/livox/lidar',
             ape_ref_kind='cross_validation',
             ape_ref_src='glim_mid360_reference',
             ape_rmse_m='0.3'),
    ]
    [result] = module.evaluate_release_profiles([profile], records)
    assert result['best_run'] == 'match_me'


def test_render_release_profile_section_has_required_columns():
    module = _load_module()
    results = [
        {
            'name': 'newer',
            'status': 'PASS',
            'metric': 'ape_rmse_gt_m',
            'best_run': 'r1',
            'best_value': 0.08,
            'pass': 0.10,
            'target': 0.08,
            'report_only_until': None,
        }
    ]
    lines = module.render_release_profile_section(results)
    body = '\n'.join(lines)
    assert '## Release profile gate' in body
    assert 'newer' in body
    assert 'PASS' in body
    assert 'ape_rmse_gt_m' in body
