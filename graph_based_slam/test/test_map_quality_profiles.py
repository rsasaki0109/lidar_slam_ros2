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
"""Tests for map-quality threshold profiles and comparator CLI."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_map_quality_thresholds.py'
PROFILES_DIR = ROOT / 'configs' / 'map_quality_profiles'


def load_comparator_module():
    """Load the comparator script as an importable module."""
    spec = importlib.util.spec_from_file_location('check_map_quality_thresholds', SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def base_report(meaningful=True, thickness=0.05, coverage=0.6, valid_fraction=0.99):
    """Build a synthetic map_quality_report dict in the frozen format."""
    return {
        'map_quality_report': {
            'input_points': 3060570,
            'evaluated_points': 648113,
            'downsample_voxel_size_m': 0.1,
            'mean_map_entropy': {
                'value_nats': -1.110389903,
                'radius_m': 0.5,
                'valid_points': 642003,
                'valid_fraction': valid_fraction,
            },
            'plane_metrics': {
                'meaningful': meaningful,
                'patch_count': 7198,
                'thickness_rms_mean_m': thickness,
                'thickness_rms_p95_m': 0.11,
                'planar_coverage': coverage,
                'min_meaningful_planar_coverage': 0.05,
            },
            'density': {
                'occupied_root_voxels': 9786,
                'mean_points_per_voxel': 66.228591866,
                'stddev_points_per_voxel': 97.275177782,
            },
        }
    }


def profile(enforcement='blocking', require_meaningful_planes=True, thresholds=None):
    """Build a synthetic map_quality_profile dict."""
    return {
        'map_quality_profile': {
            'name': 'test_profile',
            'enforcement': enforcement,
            'require_meaningful_planes': require_meaningful_planes,
            'thresholds': thresholds
            if thresholds is not None
            else {
                'thickness_rms_mean_max_m': 0.2,
                'planar_coverage_min': 0.1,
                'mme_valid_fraction_min': 0.5,
            },
        }
    }


def write_yaml(path, data):
    """Write a dict as YAML to path."""
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')


def run_cli(tmp_path, report_data, profile_data, extra_args=None):
    """Run the comparator CLI on synthetic report/profile files."""
    report_path = tmp_path / 'report.yaml'
    profile_path = tmp_path / 'profile.yaml'
    write_yaml(report_path, report_data)
    write_yaml(profile_path, profile_data)

    cmd = [
        sys.executable,
        str(SCRIPT),
        '--report',
        str(report_path),
        '--profile',
        str(profile_path),
    ]
    if extra_args:
        cmd.extend(extra_args)

    return subprocess.run(cmd, check=False, capture_output=True, text=True)


def test_blocking_profile_all_thresholds_pass(tmp_path):
    """A blocking profile with satisfied thresholds exits 0 with OK."""
    completed = run_cli(tmp_path, base_report(), profile())

    assert completed.returncode == 0
    assert 'MAP_QUALITY_THRESHOLDS_OK' in completed.stdout
    assert 'violations=0' in completed.stdout


def test_blocking_profile_violation_fails(tmp_path):
    """A blocking violation exits 1 and names the violated key."""
    completed = run_cli(
        tmp_path,
        base_report(thickness=0.5),
        profile(thresholds={'thickness_rms_mean_max_m': 0.2}),
    )

    assert completed.returncode == 1
    assert 'MAP_QUALITY_THRESHOLDS_FAILED' in completed.stdout
    assert (
        'THRESHOLD thickness_rms_mean_max_m value=0.500000000 '
        'limit=0.200000000 verdict=VIOLATION'
    ) in completed.stdout


def test_report_only_profile_violation_exits_zero(tmp_path):
    """report_only enforcement reports violations but exits 0."""
    completed = run_cli(
        tmp_path,
        base_report(thickness=0.5),
        profile(
            enforcement='report_only',
            require_meaningful_planes=False,
            thresholds={'thickness_rms_mean_max_m': 0.2},
        ),
    )

    assert completed.returncode == 0
    assert 'MAP_QUALITY_THRESHOLDS_REPORT_ONLY' in completed.stdout
    assert 'violations=1' in completed.stdout


def test_not_meaningful_required_planes_violates_and_skips_plane_thresholds(tmp_path):
    """meaningful=false violates a require_meaningful_planes gate; plane keys skip."""
    completed = run_cli(
        tmp_path,
        base_report(meaningful=False, valid_fraction=0.75),
        profile(
            require_meaningful_planes=True,
            thresholds={
                'thickness_rms_mean_max_m': 0.2,
                'planar_coverage_min': 0.1,
                'mme_valid_fraction_min': 0.5,
            },
        ),
    )

    assert completed.returncode == 1
    assert 'MAP_QUALITY_THRESHOLDS_FAILED' in completed.stdout
    assert 'THRESHOLD thickness_rms_mean_max_m value=NA limit=0.200000000' in completed.stdout
    assert 'verdict=SKIPPED(not_meaningful)' in completed.stdout
    assert (
        'THRESHOLD mme_valid_fraction_min value=0.750000000 '
        'limit=0.500000000 verdict=PASS'
    ) in completed.stdout


def test_not_meaningful_not_required_with_only_plane_thresholds_passes(tmp_path):
    """meaningful=false with only plane thresholds and no requirement passes."""
    completed = run_cli(
        tmp_path,
        base_report(meaningful=False),
        profile(
            require_meaningful_planes=False,
            thresholds={
                'thickness_rms_mean_max_m': 0.2,
                'planar_coverage_min': 0.1,
            },
        ),
    )

    assert completed.returncode == 0
    assert 'MAP_QUALITY_THRESHOLDS_OK' in completed.stdout
    assert completed.stdout.count('SKIPPED(not_meaningful)') == 2
    assert 'violations=0' in completed.stdout


def test_unknown_threshold_key_exits_usage_error(tmp_path):
    """A typo in a threshold key must fail loudly with exit 2."""
    completed = run_cli(
        tmp_path,
        base_report(),
        profile(thresholds={'thickness_rms_mean_typo_m': 0.2}),
    )

    assert completed.returncode == 2
    assert 'unknown threshold key' in completed.stderr


def test_committed_profiles_parse_and_keep_schema_sane():
    """The committed profile YAMLs parse and keep the required schema."""
    for path in (
        PROFILES_DIR / 'indoor_construction.yaml',
        PROFILES_DIR / 'outdoor_vegetation.yaml',
    ):
        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)

        assert isinstance(data, dict)
        body = data['map_quality_profile']
        assert isinstance(body['name'], str)
        assert body['enforcement'] in {'blocking', 'report_only'}
        assert isinstance(body['require_meaningful_planes'], bool)
        assert isinstance(body['thresholds'], dict)


def test_out_verdict_yaml_round_trips(tmp_path):
    """The --out verdict YAML matches the exit status and checks."""
    out_path = tmp_path / 'verdict.yaml'
    completed = run_cli(
        tmp_path,
        base_report(thickness=0.5),
        profile(thresholds={'thickness_rms_mean_max_m': 0.2}),
        extra_args=['--out', str(out_path)],
    )

    assert completed.returncode == 1
    with out_path.open('r', encoding='utf-8') as stream:
        verdict = yaml.safe_load(stream)

    assert verdict['overall'] == 'FAILED'
    assert verdict['violations'] == 1
    assert verdict['enforcement'] == 'blocking'
    assert verdict['checks'][0]['key'] == 'thickness_rms_mean_max_m'
    assert verdict['checks'][0]['verdict'] == 'VIOLATION'


def test_compare_core_returns_report_only_result():
    """compare() is callable directly and returns a structured result."""
    module = load_comparator_module()
    result = module.compare(
        base_report(thickness=0.5),
        profile(
            enforcement='report_only',
            require_meaningful_planes=False,
            thresholds={'thickness_rms_mean_max_m': 0.2},
        ),
    )

    assert result.overall == 'REPORT_ONLY'
    assert result.violations == 1
