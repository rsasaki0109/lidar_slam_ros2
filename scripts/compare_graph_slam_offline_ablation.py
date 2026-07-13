#!/usr/bin/env python3
"""Compare deterministic graph-SLAM runs without overstating loop residuals."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
from typing import Any

import yaml


QUALITY_RELATIVE_PATHS = {
    'optimized': Path('map_quality_optimized/run1/map_quality_report.yaml'),
    'refined': Path('map_quality_refined/run1/map_quality_report.yaml'),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def change_percent(baseline: float, candidate: float) -> float | None:
    if baseline == 0.0:
        return None
    return (candidate - baseline) / abs(baseline) * 100.0


def load_quality(run_dir: Path, stage: str) -> dict[str, float]:
    document = yaml.safe_load((run_dir / QUALITY_RELATIVE_PATHS[stage]).read_text())
    report = document['map_quality_report']
    plane = report['plane_metrics']
    return {
        'thickness_rms_mean_m': float(plane['thickness_rms_mean_m']),
        'thickness_rms_p95_m': float(plane['thickness_rms_p95_m']),
        'planar_coverage': float(plane['planar_coverage']),
        'mean_map_entropy_nats': float(report['mean_map_entropy']['value_nats']),
    }


def load_residual(run_dir: Path) -> dict[str, float]:
    report = json.loads((run_dir / 'pose_graph_loop_residuals.json').read_text())
    if report.get('status') != 'PASS' or report.get('loop_edge_count', 0) < 1:
        raise ValueError(f'{run_dir}: loop residual report did not pass')
    summary = report['summary']
    return {
        'translation_residual_mean_m': float(summary['translation_residual_mean_m']),
        'rotation_residual_mean_deg': float(summary['rotation_residual_mean_deg']),
    }


def parse_elapsed_seconds(value: str) -> float:
    fields = value.strip().split(':')
    if len(fields) == 2:
        minutes, seconds = fields
        return float(minutes) * 60.0 + float(seconds)
    if len(fields) == 3:
        hours, minutes, seconds = fields
        return float(hours) * 3600.0 + float(minutes) * 60.0 + float(seconds)
    raise ValueError(f'invalid GNU time elapsed value: {value!r}')


def load_resources(root: Path, runs: int) -> dict[str, float]:
    text = (root / 'process_time.txt').read_text()
    elapsed_value = next((line.split()[-1] for line in text.splitlines()
                          if 'Elapsed (wall clock) time' in line), None)
    rss_match = re.search(r'Maximum resident set size \(kbytes\):\s*(\d+)', text)
    exit_match = re.search(r'Exit status:\s*(-?\d+)', text)
    if not (elapsed_value and rss_match and exit_match):
        raise ValueError(f'{root / "process_time.txt"}: incomplete GNU time report')
    return {
        'wall_seconds_per_run': parse_elapsed_seconds(elapsed_value) / runs,
        'peak_rss_mb': int(rss_match.group(1)) / 1024.0,
        'process_exit_status': int(exit_match.group(1)),
    }


def metric_pair(baseline: float, candidate: float, direction: str) -> dict[str, Any]:
    delta = change_percent(baseline, candidate)
    improved = candidate < baseline if direction == 'lower' else candidate > baseline
    return {
        'baseline': baseline,
        'candidate': candidate,
        'change_percent': delta,
        'direction': direction,
        'improved': improved,
    }


def build_report(
    baseline_root: Path,
    candidate_root: Path,
    *,
    baseline_runs: int,
    candidate_runs: int,
    dataset: str,
    parameter: str,
    baseline_value: float,
    candidate_value: float,
    improved_datasets: int,
    minimum_improved_datasets: int,
    max_geometry_regression_percent: float,
) -> dict[str, Any]:
    baseline = baseline_root / 'run1'
    candidate = candidate_root / 'run1'
    loop_baseline_hash = sha256(baseline / 'loop_edges.csv')
    loop_candidate_hash = sha256(candidate / 'loop_edges.csv')
    fixed_input = loop_baseline_hash == loop_candidate_hash

    baseline_residual = load_residual(baseline)
    candidate_residual = load_residual(candidate)
    residual = {
        key: metric_pair(baseline_residual[key], candidate_residual[key], 'lower')
        for key in baseline_residual
    }
    quality: dict[str, Any] = {}
    geometry_regressions = []
    for stage in QUALITY_RELATIVE_PATHS:
        before = load_quality(baseline, stage)
        after = load_quality(candidate, stage)
        quality[stage] = {}
        for key in before:
            direction = 'higher' if key == 'planar_coverage' else 'lower'
            pair = metric_pair(before[key], after[key], direction)
            quality[stage][key] = pair
            delta = pair['change_percent']
            regression = (-delta if direction == 'higher' else delta) if delta is not None else 0.0
            if regression > max_geometry_regression_percent:
                geometry_regressions.append({
                    'stage': stage, 'metric': key, 'regression_percent': regression,
                })

    baseline_resources = load_resources(baseline_root, baseline_runs)
    candidate_resources = load_resources(candidate_root, candidate_runs)
    resources = {
        key: metric_pair(
            baseline_resources[key], candidate_resources[key],
            'lower' if key != 'process_exit_status' else 'higher',
        )
        for key in baseline_resources
    }
    process_ok = (baseline_resources['process_exit_status'] == 0 and
                  candidate_resources['process_exit_status'] == 0)
    multi_dataset_ok = improved_datasets >= minimum_improved_datasets
    adoption_ready = fixed_input and not geometry_regressions and process_ok and multi_dataset_ok
    return {
        'schema_version': 1,
        'dataset': dataset,
        'ablation': {
            'parameter': parameter,
            'baseline_value': baseline_value,
            'candidate_value': candidate_value,
        },
        'evidence_scope': 'constraint_fit_map_geometry_runtime_memory',
        'trajectory_accuracy_claimed': False,
        'fixed_input': {
            'passed': fixed_input,
            'loop_edges_sha256': {
                'baseline': loop_baseline_hash,
                'candidate': loop_candidate_hash,
            },
            'note': 'Byte-identical accepted edges isolate pose-graph weighting from loop selection.',
        },
        'loop_constraint_residuals': residual,
        'map_quality': quality,
        'resources': resources,
        'gates': {
            'process_exit_status_zero': process_ok,
            'maximum_geometry_regression_percent': max_geometry_regression_percent,
            'geometry_regressions': geometry_regressions,
            'minimum_improved_datasets': minimum_improved_datasets,
            'improved_datasets': improved_datasets,
            'multi_dataset_improvement': multi_dataset_ok,
        },
        'verdict': 'ADOPT' if adoption_ready else 'DO_NOT_ADOPT',
        'verdict_reasons': [
            reason for condition, reason in (
                (not fixed_input, 'accepted loop edges differ'),
                (bool(geometry_regressions), 'map geometry exceeds the regression budget'),
                (not process_ok, 'a benchmark process failed'),
                (not multi_dataset_ok, 'minimum improved-dataset count is not met'),
            ) if condition
        ],
        'interpretation': (
            'A smaller accepted-edge residual proves stronger constraint fit, not lower trajectory '
            'error. Ground truth on enough datasets is still required for default adoption.'
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-dir', type=Path, required=True)
    parser.add_argument('--candidate-dir', type=Path, required=True)
    parser.add_argument('--baseline-runs', type=int, required=True)
    parser.add_argument('--candidate-runs', type=int, required=True)
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--parameter', required=True)
    parser.add_argument('--baseline-value', type=float, required=True)
    parser.add_argument('--candidate-value', type=float, required=True)
    parser.add_argument('--improved-datasets', type=int, default=0)
    parser.add_argument('--minimum-improved-datasets', type=int, default=2)
    parser.add_argument('--max-geometry-regression-percent', type=float, default=2.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.baseline_runs < 1 or args.candidate_runs < 1:
        parser.error('run counts must be positive')
    try:
        report = build_report(
            args.baseline_dir.resolve(), args.candidate_dir.resolve(),
            baseline_runs=args.baseline_runs, candidate_runs=args.candidate_runs,
            dataset=args.dataset, parameter=args.parameter,
            baseline_value=args.baseline_value, candidate_value=args.candidate_value,
            improved_datasets=args.improved_datasets,
            minimum_improved_datasets=args.minimum_improved_datasets,
            max_geometry_regression_percent=args.max_geometry_regression_percent,
        )
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        parser.exit(2, f'failed to compare graph-SLAM ablation: {exc}\n')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(report, indent=2, sort_keys=True) + '\n'
    args.output.write_text(payload)
    print(payload, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
