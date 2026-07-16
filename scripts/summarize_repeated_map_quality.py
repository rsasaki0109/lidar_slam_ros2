#!/usr/bin/env python3
"""Aggregate repeated map-quality reports with conservative worst cases."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
from typing import Any

import yaml


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def summarize(reports: list[Path], maps: list[Path] | None = None) -> dict[str, Any]:
    if not reports:
        raise ValueError('at least one map-quality report is required')
    if maps is not None and len(maps) != len(reports):
        raise ValueError('map and report counts differ')
    runs = []
    for index, report_path in enumerate(reports):
        body = yaml.safe_load(report_path.read_text())['map_quality_report']
        plane = body['plane_metrics']
        row = {
            'report_path': str(report_path.resolve()),
            'report_sha256': sha256(report_path),
            'input_points': int(body['input_points']),
            'evaluated_points': int(body['evaluated_points']),
            'plane_metrics_meaningful': bool(plane.get('meaningful')),
            'plane_thickness_mean_m': float(plane['thickness_rms_mean_m']),
            'plane_thickness_p95_m': float(plane['thickness_rms_p95_m']),
            'planar_coverage': float(plane['planar_coverage']),
        }
        if maps is not None:
            row['map_path'] = str(maps[index].resolve())
            row['map_sha256'] = sha256(maps[index])
        runs.append(row)
    non_meaningful = [
        row['report_path'] for row in runs
        if not row['plane_metrics_meaningful']]
    if non_meaningful:
        return {
            'schema_version': 1,
            'valid_repetitions': len(runs),
            'meaningful_repetitions': len(runs) - len(non_meaningful),
            'aggregation_valid': False,
            'failure_reasons': [
                f'plane metrics are not meaningful: {path}'
                for path in non_meaningful],
            'aggregation_policy': {
                'lower_is_better_metrics': 'maximum_worst_case',
                'higher_is_better_metrics': 'minimum_worst_case'},
            'aggregate': None,
            'runs': runs,
        }
    means = [row['plane_thickness_mean_m'] for row in runs]
    p95s = [row['plane_thickness_p95_m'] for row in runs]
    coverages = [row['planar_coverage'] for row in runs]
    return {
        'schema_version': 1,
        'valid_repetitions': len(runs),
        'meaningful_repetitions': len(runs),
        'aggregation_valid': True,
        'failure_reasons': [],
        'aggregation_policy': {
            'lower_is_better_metrics': 'maximum_worst_case',
            'higher_is_better_metrics': 'minimum_worst_case'},
        'aggregate': {
            'plane_thickness_mean_worst_m': max(means),
            'plane_thickness_p95_worst_m': max(p95s),
            'planar_coverage_worst': min(coverages),
            'plane_thickness_mean_median_m': statistics.median(means),
            'plane_thickness_p95_median_m': statistics.median(p95s),
            'planar_coverage_median': statistics.median(coverages),
        },
        'runs': runs,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, action='append', required=True)
    parser.add_argument('--map', type=Path, action='append')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    document = summarize(args.report, args.map)
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    if not document['aggregation_valid']:
        print(json.dumps({
            'aggregation_valid': False,
            'failure_reasons': document['failure_reasons'],
        }, indent=2, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(document['aggregate'], indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
