#!/usr/bin/env python3
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

"""Fail-closed paired non-regression check for map-quality reports.

This comparator is deliberately independent from the absolute threshold
profiles.  It answers a narrower question: did a candidate map get worse than
the named baseline under the same metric-extraction configuration?  A missing
field, non-finite value, zero baseline denominator, or inconsistent extraction
configuration is an invalid gate rather than a pass.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class RegressionError(Exception):
    """Raised for a malformed or unsafe paired-comparison input."""


@dataclass(frozen=True)
class MetricSpec:
    """One metric and the direction in which it is expected to improve."""

    path: tuple[str, ...]
    direction: str


@dataclass(frozen=True)
class Snapshot:
    """Validated metric snapshot extracted from one report."""

    source: str
    values: dict[str, float]
    extraction_config: dict[str, float]
    meaningful: bool


METRIC_SPECS: tuple[tuple[str, MetricSpec], ...] = (
    (
        'plane_metrics.thickness_rms_mean_m',
        MetricSpec(('plane_metrics', 'thickness_rms_mean_m'), 'lower_is_better'),
    ),
    (
        'plane_metrics.thickness_rms_p95_m',
        MetricSpec(('plane_metrics', 'thickness_rms_p95_m'), 'lower_is_better'),
    ),
    (
        'plane_metrics.planar_coverage',
        MetricSpec(('plane_metrics', 'planar_coverage'), 'higher_is_better'),
    ),
    (
        'mean_map_entropy.valid_fraction',
        MetricSpec(('mean_map_entropy', 'valid_fraction'), 'higher_is_better'),
    ),
    (
        'mean_map_entropy.value_nats',
        MetricSpec(('mean_map_entropy', 'value_nats'), 'higher_is_worse'),
    ),
)

_DECIMAL_PATTERN = re.compile(
    r'^[+]?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?$'
)


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except FileNotFoundError as exc:
        raise RegressionError(f'missing report: {path}') from exc
    except (OSError, UnicodeError) as exc:
        raise RegressionError(f'cannot read report {path}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise RegressionError(f'cannot parse YAML {path}: {exc}') from exc

    if not isinstance(data, dict):
        raise RegressionError(f'expected YAML mapping in {path}')
    return data


def _report_body(data: dict[str, Any], source: str) -> dict[str, Any]:
    body = data.get('map_quality_report')
    if not isinstance(body, dict):
        raise RegressionError(f'{source}: missing map_quality_report mapping')
    return body


def _nested_get(data: dict[str, Any], path: tuple[str, ...], source: str) -> Any:
    current: Any = data
    for part in path:
        if not isinstance(current, dict) or part not in current:
            raise RegressionError(f'{source}: missing required field {".".join(path)}')
        current = current[part]
    return current


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RegressionError(f'{label} must be numeric')
    converted = float(value)
    if not math.isfinite(converted):
        raise RegressionError(f'{label} must be finite')
    return converted


def _nonnegative_integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise RegressionError(f'{label} must be a non-negative integer')
    if value < 0:
        raise RegressionError(f'{label} must be a non-negative integer')
    return value


def _fraction(value: Any, label: str) -> float:
    number = _finite_number(value, label)
    if number < 0.0 or number > 1.0:
        raise RegressionError(f'{label} must be within [0, 1]')
    return number


def _validate_snapshot(data: dict[str, Any], source: str) -> Snapshot:
    report = _report_body(data, source)

    input_points = _nonnegative_integer(
        report.get('input_points'), f'{source}: input_points'
    )
    evaluated_points = _nonnegative_integer(
        report.get('evaluated_points'), f'{source}: evaluated_points'
    )
    if input_points == 0 or evaluated_points == 0:
        raise RegressionError(f'{source}: input_points and evaluated_points must be > 0')
    if evaluated_points > input_points:
        raise RegressionError(f'{source}: evaluated_points exceeds input_points')

    downsample = _finite_number(
        report.get('downsample_voxel_size_m'),
        f'{source}: downsample_voxel_size_m',
    )
    if downsample <= 0.0:
        raise RegressionError(f'{source}: downsample_voxel_size_m must be > 0')

    entropy = report.get('mean_map_entropy')
    if not isinstance(entropy, dict):
        raise RegressionError(f'{source}: mean_map_entropy must be a mapping')
    entropy_radius = _finite_number(
        entropy.get('radius_m'), f'{source}: mean_map_entropy.radius_m'
    )
    if entropy_radius <= 0.0:
        raise RegressionError(f'{source}: mean_map_entropy.radius_m must be > 0')
    valid_points = _nonnegative_integer(
        entropy.get('valid_points'), f'{source}: mean_map_entropy.valid_points'
    )
    if valid_points > evaluated_points:
        raise RegressionError(f'{source}: valid_points exceeds evaluated_points')

    planes = report.get('plane_metrics')
    if not isinstance(planes, dict):
        raise RegressionError(f'{source}: plane_metrics must be a mapping')
    meaningful = planes.get('meaningful')
    if not isinstance(meaningful, bool):
        raise RegressionError(f'{source}: plane_metrics.meaningful must be boolean')
    if not meaningful:
        raise RegressionError(f'{source}: plane metrics are not meaningful')
    patch_count = _nonnegative_integer(
        planes.get('patch_count'), f'{source}: plane_metrics.patch_count'
    )
    if patch_count == 0:
        raise RegressionError(f'{source}: meaningful plane metrics have zero patches')
    min_coverage = _fraction(
        planes.get('min_meaningful_planar_coverage'),
        f'{source}: plane_metrics.min_meaningful_planar_coverage',
    )

    density = report.get('density')
    if not isinstance(density, dict):
        raise RegressionError(f'{source}: density must be a mapping')
    occupied_root_voxels = _nonnegative_integer(
        density.get('occupied_root_voxels'),
        f'{source}: density.occupied_root_voxels',
    )
    if occupied_root_voxels == 0:
        raise RegressionError(f'{source}: density.occupied_root_voxels must be > 0')
    for density_key in ('mean_points_per_voxel', 'stddev_points_per_voxel'):
        density_value = _finite_number(
            density.get(density_key), f'{source}: density.{density_key}'
        )
        if density_value < 0.0:
            raise RegressionError(f'{source}: density.{density_key} must be non-negative')

    values: dict[str, float] = {}
    for key, spec in METRIC_SPECS:
        value = _finite_number(_nested_get(report, spec.path, source), f'{source}: {key}')
        if key.endswith('thickness_rms_mean_m') or key.endswith('thickness_rms_p95_m'):
            if value < 0.0:
                raise RegressionError(f'{source}: {key} must be non-negative')
        elif key.endswith('planar_coverage') or key.endswith('valid_fraction'):
            if value < 0.0 or value > 1.0:
                raise RegressionError(f'{source}: {key} must be within [0, 1]')
        values[key] = value

    return Snapshot(
        source=source,
        values=values,
        extraction_config={
            'downsample_voxel_size_m': downsample,
            'mean_entropy_radius_m': entropy_radius,
            'min_meaningful_planar_coverage': min_coverage,
        },
        meaningful=meaningful,
    )


def _strict_percent(value: str) -> float:
    if not isinstance(value, str) or not _DECIMAL_PATTERN.fullmatch(value):
        raise RegressionError(
            f'max regression percent must be a finite decimal, got {value!r}'
        )
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise RegressionError(f'max regression percent must be non-negative: {value!r}')
    return parsed


def _check_extraction_config(
    baseline: Snapshot, candidate: Snapshot
) -> None:
    for key in baseline.extraction_config:
        baseline_value = baseline.extraction_config[key]
        candidate_value = candidate.extraction_config[key]
        if baseline_value != candidate_value:
            raise RegressionError(
                'extraction configuration mismatch for '
                f'{key}: baseline={baseline_value!r} candidate={candidate_value!r}'
            )


def _check_baseline_denominators(baseline: Snapshot) -> None:
    for key, value in baseline.values.items():
        if value == 0.0:
            raise RegressionError(f'baseline metric {key} must be non-zero')


def _regression_percent(baseline: float, candidate: float, direction: str) -> float:
    denominator = abs(baseline)
    if direction == 'lower_is_better':
        delta = candidate - baseline
    elif direction == 'higher_is_better':
        delta = baseline - candidate
    elif direction == 'higher_is_worse':
        delta = candidate - baseline
    else:
        raise RegressionError(f'internal error: unknown direction {direction}')
    return max(0.0, delta / denominator * 100.0)


def compare(
    baseline_data: dict[str, Any],
    candidate_data: dict[str, Any],
    *,
    baseline_source: str = 'baseline',
    candidate_source: str = 'candidate',
    max_regression_percent: float = 2.0,
) -> dict[str, Any]:
    """Validate and compare two reports, returning a serializable receipt."""
    if not math.isfinite(max_regression_percent) or max_regression_percent < 0.0:
        raise RegressionError('max_regression_percent must be finite and non-negative')
    baseline = _validate_snapshot(baseline_data, baseline_source)
    candidate = _validate_snapshot(candidate_data, candidate_source)
    _check_extraction_config(baseline, candidate)
    _check_baseline_denominators(baseline)

    checks: list[dict[str, Any]] = []
    violations = 0
    for key, spec in METRIC_SPECS:
        baseline_value = baseline.values[key]
        candidate_value = candidate.values[key]
        allowed_abs = abs(baseline_value) * max_regression_percent / 100.0
        regression = _regression_percent(baseline_value, candidate_value, spec.direction)
        verdict = 'PASS' if regression <= max_regression_percent else 'VIOLATION'
        if verdict == 'VIOLATION':
            violations += 1
        checks.append(
            {
                'metric': key,
                'direction': spec.direction,
                'baseline': baseline_value,
                'candidate': candidate_value,
                'allowed_abs': allowed_abs,
                'regression_percent': regression,
                'verdict': verdict,
            }
        )

    status = 'PASS' if violations == 0 else 'FAIL'
    return {
        'schema': 1,
        'receipt_kind': 'map_quality_paired_non_regression',
        'status': status,
        'max_regression_percent': max_regression_percent,
        'baseline_report': baseline.source,
        'candidate_report': candidate.source,
        'extraction_config': {
            'consistent': True,
            'baseline': baseline.extraction_config,
            'candidate': candidate.extraction_config,
        },
        'meaningful_planes': {
            'baseline': baseline.meaningful,
            'candidate': candidate.meaningful,
        },
        'checks': checks,
        'violations': violations,
        'errors': [],
    }


def _invalid_receipt(
    baseline_source: str,
    candidate_source: str,
    max_regression_percent: str,
    error: str,
) -> dict[str, Any]:
    return {
        'schema': 1,
        'receipt_kind': 'map_quality_paired_non_regression',
        'status': 'INVALID',
        'max_regression_percent': max_regression_percent,
        'baseline_report': baseline_source,
        'candidate_report': candidate_source,
        'extraction_config': {'consistent': False},
        'meaningful_planes': None,
        'checks': [],
        'violations': 0,
        'errors': [error],
    }


def _write_receipt(path: Path, receipt: dict[str, Any], *, json_format: bool) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', encoding='utf-8') as stream:
            if json_format:
                json.dump(receipt, stream, indent=2, sort_keys=False)
                stream.write('\n')
            else:
                yaml.safe_dump(receipt, stream, sort_keys=False)
    except OSError as exc:
        raise RegressionError(f'cannot write {path}: {exc}') from exc


def _print_receipt(receipt: dict[str, Any]) -> None:
    if receipt['status'] == 'INVALID':
        for error in receipt['errors']:
            print(f'PAIRED_MAP_QUALITY_INVALID error={error}', file=sys.stderr)
        print('MAP_QUALITY_PAIRED_NON_REGRESSION_INVALID', file=sys.stderr)
        return

    for check in receipt['checks']:
        print(
            'PAIRED_MAP_QUALITY '
            f"metric={check['metric']} direction={check['direction']} "
            f"baseline={check['baseline']:.9f} candidate={check['candidate']:.9f} "
            f"regression_percent={check['regression_percent']:.9f} "
            f"allowed_abs={check['allowed_abs']:.9f} verdict={check['verdict']}"
        )
    if receipt['status'] == 'PASS':
        print(
            'MAP_QUALITY_PAIRED_NON_REGRESSION_OK: '
            f"checked={len(receipt['checks'])} violations=0 "
            f"max_regression_percent={receipt['max_regression_percent']:.9f}"
        )
    else:
        print(
            'MAP_QUALITY_PAIRED_NON_REGRESSION_FAILED: '
            f"checked={len(receipt['checks'])} violations={receipt['violations']} "
            f"max_regression_percent={receipt['max_regression_percent']:.9f}"
        )


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-report', required=True, type=Path)
    parser.add_argument('--candidate-report', required=True, type=Path)
    parser.add_argument('--max-regression-percent', default='2.0')
    parser.add_argument('--out', type=Path, help='YAML machine-readable receipt')
    parser.add_argument('--json-out', type=Path, help='JSON machine-readable receipt')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    receipt: dict[str, Any]
    try:
        max_percent = _strict_percent(args.max_regression_percent)
        baseline = _load_yaml(args.baseline_report)
        candidate = _load_yaml(args.candidate_report)
        receipt = compare(
            baseline,
            candidate,
            baseline_source=str(args.baseline_report),
            candidate_source=str(args.candidate_report),
            max_regression_percent=max_percent,
        )
    except RegressionError as exc:
        receipt = _invalid_receipt(
            str(args.baseline_report),
            str(args.candidate_report),
            args.max_regression_percent,
            str(exc),
        )

    write_error: RegressionError | None = None
    try:
        if args.out is not None:
            _write_receipt(args.out, receipt, json_format=False)
        if args.json_out is not None:
            _write_receipt(args.json_out, receipt, json_format=True)
    except RegressionError as exc:
        write_error = exc

    if write_error is not None:
        print(f'ERROR: {write_error}', file=sys.stderr)
        return 2

    _print_receipt(receipt)
    if receipt['status'] == 'INVALID':
        return 2
    return 0 if receipt['status'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
