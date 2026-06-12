#!/usr/bin/env python3
"""Check map-quality report metrics against an explicit threshold profile."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class ThresholdError(Exception):
    """Raised for user-facing threshold/profile/report errors."""


@dataclass(frozen=True)
class ThresholdSpec:
    """Where a threshold key reads from the report and how it compares."""

    report_path: tuple[str, ...]
    comparison: str
    plane_domain: bool = False


@dataclass(frozen=True)
class ThresholdCheck:
    """One evaluated threshold row."""

    key: str
    value: float | int | None
    limit: float | int
    verdict: str


@dataclass(frozen=True)
class ThresholdResult:
    """Deterministic comparison outcome for one report/profile pair."""

    profile: str
    enforcement: str
    checks: list[ThresholdCheck]
    violations: int
    overall: str


THRESHOLD_SPECS: dict[str, ThresholdSpec] = {
    'mean_map_entropy_max_nats': ThresholdSpec(
        ('mean_map_entropy', 'value_nats'),
        'max',
    ),
    'mme_valid_fraction_min': ThresholdSpec(
        ('mean_map_entropy', 'valid_fraction'),
        'min',
    ),
    'thickness_rms_mean_max_m': ThresholdSpec(
        ('plane_metrics', 'thickness_rms_mean_m'),
        'max',
        plane_domain=True,
    ),
    'thickness_rms_p95_max_m': ThresholdSpec(
        ('plane_metrics', 'thickness_rms_p95_m'),
        'max',
        plane_domain=True,
    ),
    'planar_coverage_min': ThresholdSpec(
        ('plane_metrics', 'planar_coverage'),
        'min',
        plane_domain=True,
    ),
    'patch_count_min': ThresholdSpec(
        ('plane_metrics', 'patch_count'),
        'min',
        plane_domain=True,
    ),
}

VALID_ENFORCEMENTS = {'blocking', 'report_only'}


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except FileNotFoundError as exc:
        raise ThresholdError(f'missing file: {path}') from exc
    except OSError as exc:
        raise ThresholdError(f'cannot read {path}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise ThresholdError(f'cannot parse YAML {path}: {exc}') from exc

    if not isinstance(data, dict):
        raise ThresholdError(f'expected YAML mapping in {path}')
    return data


def _nested_get(data: dict[str, Any], path: tuple[str, ...]) -> Any:
    current: Any = data
    for part in path:
        if not isinstance(current, dict) or part not in current:
            dotted = '.'.join(path)
            raise ThresholdError(f'report is missing required metric: {dotted}')
        current = current[part]
    return current


def _as_number(value: Any, label: str) -> float | int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ThresholdError(f'{label} must be numeric')
    return value


def _profile_body(profile_dict: dict[str, Any]) -> dict[str, Any]:
    body = profile_dict.get('map_quality_profile')
    if not isinstance(body, dict):
        raise ThresholdError('profile must contain map_quality_profile mapping')
    return body


def _report_body(report_dict: dict[str, Any]) -> dict[str, Any]:
    body = report_dict.get('map_quality_report')
    if not isinstance(body, dict):
        raise ThresholdError('report must contain map_quality_report mapping')
    return body


def _validate_profile(profile: dict[str, Any]) -> tuple[str, str, bool, dict[str, float | int]]:
    name = profile.get('name')
    if not isinstance(name, str) or not name:
        raise ThresholdError('profile field name must be a non-empty string')

    enforcement = profile.get('enforcement')
    if enforcement not in VALID_ENFORCEMENTS:
        valid = ', '.join(sorted(VALID_ENFORCEMENTS))
        raise ThresholdError(f'profile enforcement must be one of: {valid}')

    require_meaningful_planes = profile.get('require_meaningful_planes')
    if not isinstance(require_meaningful_planes, bool):
        raise ThresholdError('profile field require_meaningful_planes must be boolean')

    thresholds = profile.get('thresholds')
    if thresholds is None:
        thresholds = {}
    if not isinstance(thresholds, dict):
        raise ThresholdError('profile field thresholds must be a mapping')

    unknown = sorted(set(thresholds) - set(THRESHOLD_SPECS))
    if unknown:
        raise ThresholdError(f"unknown threshold key(s): {', '.join(unknown)}")

    numeric_thresholds: dict[str, float | int] = {}
    for key, limit in thresholds.items():
        numeric_thresholds[key] = _as_number(limit, f'threshold {key}')

    return name, enforcement, require_meaningful_planes, numeric_thresholds


def _planes_are_meaningful(report: dict[str, Any]) -> bool:
    plane_metrics = report.get('plane_metrics')
    if not isinstance(plane_metrics, dict):
        raise ThresholdError('report is missing required mapping: plane_metrics')
    meaningful = plane_metrics.get('meaningful')
    if not isinstance(meaningful, bool):
        raise ThresholdError('report metric plane_metrics.meaningful must be boolean')
    return meaningful


def _passes(value: float | int, limit: float | int, comparison: str) -> bool:
    if comparison == 'max':
        return value <= limit
    if comparison == 'min':
        return value >= limit
    raise ThresholdError(f'internal error: unknown comparison {comparison}')


def compare(report_dict: dict[str, Any], profile_dict: dict[str, Any]) -> ThresholdResult:
    """Compare a report against a profile and return a deterministic verdict."""
    report = _report_body(report_dict)
    profile = _profile_body(profile_dict)
    name, enforcement, require_meaningful_planes, thresholds = _validate_profile(profile)
    meaningful_planes = _planes_are_meaningful(report)

    checks: list[ThresholdCheck] = []
    violations = 0

    if require_meaningful_planes and not meaningful_planes:
        violations += 1
        checks.append(
            ThresholdCheck(
                key='require_meaningful_planes',
                value=None,
                limit=1,
                verdict='VIOLATION',
            )
        )

    for key, limit in thresholds.items():
        spec = THRESHOLD_SPECS[key]
        if spec.plane_domain and not meaningful_planes:
            checks.append(
                ThresholdCheck(
                    key=key,
                    value=None,
                    limit=limit,
                    verdict='SKIPPED(not_meaningful)',
                )
            )
            continue

        value = _as_number(_nested_get(report, spec.report_path), '.'.join(spec.report_path))
        verdict = 'PASS' if _passes(value, limit, spec.comparison) else 'VIOLATION'
        if verdict == 'VIOLATION':
            violations += 1
        checks.append(ThresholdCheck(key=key, value=value, limit=limit, verdict=verdict))

    if enforcement == 'report_only':
        overall = 'REPORT_ONLY'
    elif violations:
        overall = 'FAILED'
    else:
        overall = 'OK'

    return ThresholdResult(
        profile=name,
        enforcement=enforcement,
        checks=checks,
        violations=violations,
        overall=overall,
    )


def _format_value(value: float | int | None) -> str:
    if value is None:
        return 'NA'
    if isinstance(value, int):
        return str(value)
    return f'{value:.9f}'


def _print_result(result: ThresholdResult) -> None:
    for check in result.checks:
        print(
            'THRESHOLD '
            f'{check.key} '
            f'value={_format_value(check.value)} '
            f'limit={_format_value(check.limit)} '
            f'verdict={check.verdict}'
        )

    checked = len(result.checks)
    if result.overall == 'REPORT_ONLY':
        print(
            'MAP_QUALITY_THRESHOLDS_REPORT_ONLY: '
            f'profile={result.profile} checked={checked} violations={result.violations}'
        )
    elif result.overall == 'FAILED':
        print(
            'MAP_QUALITY_THRESHOLDS_FAILED: '
            f'profile={result.profile} enforcement={result.enforcement} '
            f'checked={checked} violations={result.violations}'
        )
    else:
        print(
            'MAP_QUALITY_THRESHOLDS_OK: '
            f'profile={result.profile} enforcement={result.enforcement} '
            f'checked={checked} violations=0'
        )


def _result_for_yaml(result: ThresholdResult) -> dict[str, Any]:
    return {
        'profile': result.profile,
        'enforcement': result.enforcement,
        'checks': [
            {
                'key': check.key,
                'value': check.value,
                'limit': check.limit,
                'verdict': check.verdict,
            }
            for check in result.checks
        ],
        'violations': result.violations,
        'overall': result.overall,
    }


def _write_out(path: Path, result: ThresholdResult) -> None:
    try:
        with path.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(_result_for_yaml(result), stream, sort_keys=False)
    except OSError as exc:
        raise ThresholdError(f'cannot write {path}: {exc}') from exc


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--profile', required=True, type=Path)
    parser.add_argument('--out', type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """CLI entry point; returns the process exit code."""
    args = _parse_args(sys.argv[1:] if argv is None else argv)

    try:
        report = _load_yaml(args.report)
        profile = _load_yaml(args.profile)
        result = compare(report, profile)
        _print_result(result)
        if args.out is not None:
            _write_out(args.out, result)
    except ThresholdError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2

    if result.overall == 'FAILED':
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
