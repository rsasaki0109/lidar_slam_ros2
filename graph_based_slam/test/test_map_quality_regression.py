# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for the fail-closed paired map-quality non-regression gate."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_map_quality_regression.py'


def load_checker_module():
    """Load the script as a module so core validation is tested directly."""
    spec = importlib.util.spec_from_file_location('check_map_quality_regression', SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def report(
    *,
    thickness_mean=0.05,
    thickness_p95=0.11,
    coverage=0.60,
    valid_fraction=0.99,
    entropy=-1.10,
    meaningful=True,
    downsample=0.1,
    radius=0.5,
    min_coverage=0.05,
):
    """Build one complete report in the map_quality_report schema."""
    return {
        'map_quality_report': {
            'input_points': 1000,
            'evaluated_points': 900,
            'downsample_voxel_size_m': downsample,
            'mean_map_entropy': {
                'value_nats': entropy,
                'radius_m': radius,
                'valid_points': 800,
                'valid_fraction': valid_fraction,
            },
            'plane_metrics': {
                'meaningful': meaningful,
                'patch_count': 12,
                'thickness_rms_mean_m': thickness_mean,
                'thickness_rms_p95_m': thickness_p95,
                'planar_coverage': coverage,
                'min_meaningful_planar_coverage': min_coverage,
            },
            'density': {
                'occupied_root_voxels': 100,
                'mean_points_per_voxel': 9.0,
                'stddev_points_per_voxel': 2.0,
            },
        }
    }


def write_yaml(path: Path, value) -> None:
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')


def run_cli(tmp_path, baseline, candidate, *extra):
    baseline_path = tmp_path / 'baseline.yaml'
    candidate_path = tmp_path / 'candidate.yaml'
    write_yaml(baseline_path, baseline)
    write_yaml(candidate_path, candidate)
    return subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--baseline-report',
            str(baseline_path),
            '--candidate-report',
            str(candidate_path),
            *extra,
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def test_all_directions_pass_within_two_percent():
    """Lower, higher, and entropy directions use the same relative budget."""
    module = load_checker_module()
    baseline = report()
    candidate = report(
        thickness_mean=0.0505,
        thickness_p95=0.1105,
        coverage=0.597,
        valid_fraction=0.985,
        entropy=-1.115,
    )
    result = module.compare(baseline, candidate, max_regression_percent=2.0)
    assert result['status'] == 'PASS'
    assert result['violations'] == 0
    assert len(result['checks']) == 5
    assert all(row['verdict'] == 'PASS' for row in result['checks'])


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('thickness_mean', 0.052),
        ('thickness_p95', 0.113),
        ('coverage', 0.58),
        ('valid_fraction', 0.96),
        ('entropy', -1.05),
    ],
)
def test_directional_regression_fails_closed(field, value):
    """A greater-than-two-percent change in either direction is a failure."""
    module = load_checker_module()
    baseline = report()
    candidate_kwargs = {field: value}
    result = module.compare(
        baseline, report(**candidate_kwargs), max_regression_percent=2.0
    )
    assert result['status'] == 'FAIL'
    assert result['violations'] == 1
    failed = [row for row in result['checks'] if row['verdict'] == 'VIOLATION']
    assert len(failed) == 1


def test_candidate_zero_is_directionally_evaluated():
    """Zero candidate values are not silently dropped or treated as missing."""
    module = load_checker_module()
    baseline = report()
    lower_improvement = module.compare(
        baseline, report(thickness_mean=0.0), max_regression_percent=2.0
    )
    assert lower_improvement['status'] == 'PASS'

    higher_regression = module.compare(
        baseline, report(coverage=0.0), max_regression_percent=2.0
    )
    assert higher_regression['status'] == 'FAIL'


@pytest.mark.parametrize(
    'mutate',
    [
        lambda data: data['map_quality_report'].pop('evaluated_points'),
        lambda data: data['map_quality_report']['plane_metrics'].__setitem__(
            'thickness_rms_mean_m', float('nan')
        ),
        lambda data: data['map_quality_report']['plane_metrics'].__setitem__(
            'patch_count', 0
        ),
        lambda data: data['map_quality_report']['plane_metrics'].__setitem__(
            'meaningful', False
        ),
    ],
)
def test_schema_nonfinite_and_meaningful_plane_errors(mutate):
    """Missing, non-finite, zero-patch, and meaningless reports are invalid."""
    module = load_checker_module()
    candidate = report()
    mutate(candidate)
    with pytest.raises(module.RegressionError):
        module.compare(report(), candidate)


def test_extraction_configuration_mismatch_is_invalid():
    """The paired gate cannot compare reports extracted with different settings."""
    module = load_checker_module()
    with pytest.raises(module.RegressionError, match='extraction configuration mismatch'):
        module.compare(report(), report(radius=0.75))


def test_zero_baseline_and_invalid_percent_are_rejected():
    """A relative comparison has no safe answer for a zero baseline."""
    module = load_checker_module()
    with pytest.raises(module.RegressionError, match='must be non-zero'):
        module.compare(report(thickness_mean=0.0), report())
    with pytest.raises(module.RegressionError, match='finite decimal'):
        module._strict_percent('-1')
    with pytest.raises(module.RegressionError, match='finite decimal'):
        module._strict_percent('nan')


def test_cli_writes_yaml_and_json_with_matching_status(tmp_path):
    """Both machine formats are emitted and carry the same PASS status."""
    yaml_out = tmp_path / 'paired.yaml'
    json_out = tmp_path / 'paired.json'
    completed = run_cli(
        tmp_path,
        report(),
        report(thickness_mean=0.0505),
        '--out',
        str(yaml_out),
        '--json-out',
        str(json_out),
    )
    assert completed.returncode == 0
    assert 'MAP_QUALITY_PAIRED_NON_REGRESSION_OK' in completed.stdout
    yaml_receipt = yaml.safe_load(yaml_out.read_text(encoding='utf-8'))
    json_receipt = json.loads(json_out.read_text(encoding='utf-8'))
    assert yaml_receipt['status'] == json_receipt['status'] == 'PASS'
    assert yaml_receipt['receipt_kind'] == 'map_quality_paired_non_regression'


def test_cli_invalid_input_still_writes_invalid_receipts(tmp_path):
    """An invalid report is never represented as a machine-readable PASS."""
    yaml_out = tmp_path / 'paired.yaml'
    json_out = tmp_path / 'paired.json'
    baseline = report()
    candidate = copy.deepcopy(report())
    candidate['map_quality_report']['mean_map_entropy']['radius_m'] = 0.25
    completed = run_cli(
        tmp_path,
        baseline,
        candidate,
        '--out',
        str(yaml_out),
        '--json-out',
        str(json_out),
    )
    assert completed.returncode == 2
    assert 'PAIRED_MAP_QUALITY_INVALID' in completed.stderr
    assert yaml.safe_load(yaml_out.read_text(encoding='utf-8'))['status'] == 'INVALID'
