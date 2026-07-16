#!/usr/bin/env python3
"""Validate accelerated FAST-LIVO2 replay as a processing-RTF bound."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import sys
from typing import Any


def validate(baseline: dict[str, Any], accelerated: dict[str, Any],
             run_reports: list[dict[str, Any]], drain_seconds: float,
             required_repetitions: int, max_ape_drift_percent: float,
             maximum_rtf: float) -> dict[str, Any]:
    baseline_runs = baseline['runs']
    accelerated_runs = accelerated['runs']
    rates = {float(report['provenance']['rate']) for report in run_reports}
    rate = next(iter(rates)) if len(rates) == 1 else None
    baseline_counts = [row['trajectory_samples'] for row in baseline_runs]
    accelerated_counts = [row['trajectory_samples'] for row in accelerated_runs]
    bounds = []
    for report in run_reports:
        runtime = report['runtime']
        duration = float(runtime['bag_duration_seconds'])
        replay_rtf = float(runtime['replay_wall_realtime_factor'])
        bounds.append(replay_rtf + max(0.0, drain_seconds) / duration)
    baseline_ape = float(baseline['aggregate']['ape_rmse_median_m'])
    accelerated_ape = float(accelerated['aggregate']['ape_rmse_median_m'])
    ape_drift = 100.0 * abs(accelerated_ape - baseline_ape) / baseline_ape
    checks = {
        'required_repetitions': (
            len(run_reports) == required_repetitions and
            baseline['valid_repetitions'] >= required_repetitions and
            accelerated['valid_repetitions'] == required_repetitions),
        'single_accelerated_rate': rate is not None and rate > 1.0,
        'trajectory_sample_counts_equal': baseline_counts == accelerated_counts,
        'ape_drift_within_tolerance': ape_drift <= max_ape_drift_percent,
        'all_processing_bounds_within_gate': all(value <= maximum_rtf for value in bounds),
        'all_runs_complete_and_clean': all(
            row['trajectory_complete'] and row['process_exit_status'] == 0
            for row in accelerated_runs),
    }
    return {
        'schema_version': 1,
        'valid_processing_rtf_evidence': all(checks.values()),
        'checks': checks,
        'accelerated_rate': rate,
        'fixed_drain_seconds': drain_seconds,
        'baseline_trajectory_samples': baseline_counts,
        'accelerated_trajectory_samples': accelerated_counts,
        'baseline_ape_rmse_median_m': baseline_ape,
        'accelerated_ape_rmse_median_m': accelerated_ape,
        'ape_absolute_drift_percent': ape_drift,
        'processing_rtf_upper_bounds': bounds,
        'processing_rtf_upper_bound_median': statistics.median(bounds),
        'processing_rtf_upper_bound_max': max(bounds),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-scored', type=Path, required=True)
    parser.add_argument('--accelerated-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--drain-seconds', type=float, default=5.0)
    parser.add_argument('--required-repetitions', type=int, default=3)
    parser.add_argument('--max-ape-drift-percent', type=float, default=1.0)
    parser.add_argument('--maximum-rtf', type=float, default=1.0)
    args = parser.parse_args()
    baseline = json.loads(args.baseline_scored.read_text())
    accelerated = json.loads(
        (args.accelerated_dir / 'scored_summary.json').read_text())
    reports = [json.loads(path.read_text()) for path in sorted(
        args.accelerated_dir.glob('run_*/run.json'))]
    result = validate(baseline, accelerated, reports, args.drain_seconds,
                      args.required_repetitions, args.max_ape_drift_percent,
                      args.maximum_rtf)
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['valid_processing_rtf_evidence'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
