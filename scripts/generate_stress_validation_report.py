#!/usr/bin/env python3

"""Generate a stress-validation snapshot from local benchmark artifacts."""

from __future__ import annotations

import argparse
from datetime import date
import json
from pathlib import Path
import re
from typing import Any


def _load_json(path: Path) -> dict[str, Any]:
    with path.open('r', encoding='utf-8') as handle:
        return json.load(handle)


def _fmt_float(value: Any, digits: int = 3) -> str:
    try:
        return f'{float(value):.{digits}f}'
    except Exception:
        return 'n/a'


def _extract_total_points(metrics: dict[str, Any]) -> str:
    map_verify = ((metrics.get('graph_based_slam') or {}).get('map_verify') or {})
    for line in map_verify.get('passes', []):
        if 'Total points across all tiles:' in line:
            return line.split(':', 1)[1].strip()
    return 'unknown'


def _extract_mid360_loop(log_path: Path) -> tuple[str, str]:
    text = log_path.read_text(encoding='utf-8', errors='replace')

    distance_matches = re.findall(
        r'Odom input:\s+\d+\s+submaps,\s+distance:\s+([0-9]+(?:\.[0-9]+)?)m',
        text,
    )
    loop_match = re.search(
        r'id_loop_point 1:(\d+)\s+id_loop_point 2:(\d+)',
        text,
    )

    if distance_matches:
        distance = f'{max(float(item) for item in distance_matches):.1f}'
    else:
        distance = 'n/a'
    if loop_match:
        loop_pair = f'{loop_match.group(1)} -> {loop_match.group(2)}'
    else:
        loop_pair = 'n/a'
    return distance, loop_pair


def _extract_newer_college_loop_phrase(readme_path: Path) -> str:
    text = readme_path.read_text(encoding='utf-8', errors='replace')
    match = re.search(r'\|\s*Path length\s*\|\s*([^|]+?)\s*\|', text)
    if not match:
        return 'unknown'
    return match.group(1).strip()


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Generate a stress-validation markdown report.',
    )
    parser.add_argument(
        '--benchmark-summary',
        default='output/benchmark_summary.md',
        help='Markdown benchmark summary for the current default path',
    )
    parser.add_argument(
        '--fresh-metrics',
        default='output/bench_rko_lio_ntu_viral_fresh_20260324/metrics.json',
        help='metrics.json for the current default-path fresh benchmark',
    )
    parser.add_argument(
        '--best-metrics',
        default='output/bench_rko_lio_ntu_viral_loopgate_20260324/metrics.json',
        help='metrics.json for the current best benchmark run',
    )
    parser.add_argument(
        '--mid360-metrics',
        default='output/bench_rko_lio_mid360_current_default_20260325/metrics.json',
        help='metrics.json for the current-path MID360 long-loop cross-validation run',
    )
    parser.add_argument(
        '--mid360-log',
        default='output/bench_rko_lio_mid360_current_default_20260325/slam.launch.log',
        help='graph_based_slam log used to extract a long-loop closure example',
    )
    parser.add_argument(
        '--mid360-legacy-summary',
        default='output/mid360_compare_summary.json',
        help='Older MID360 comparison summary kept as legacy context',
    )
    parser.add_argument(
        '--benchmark-readme',
        default='output/BENCHMARK_README.md',
        help='Benchmark README containing the Newer College dataset description',
    )
    parser.add_argument(
        '--newer-college-summary',
        default='output/newer_college_mathhard_report_summary.json',
        help='Legacy Newer College summary artifact',
    )
    parser.add_argument(
        '--ntu-legacy-summary',
        default='output/ntu_viral_tnp01_report_summary.json',
        help='Legacy NTU VIRAL summary artifact',
    )
    parser.add_argument(
        '--ntu-prism-summary',
        default='output/ntu_viral_tnp01_report_threads1_prism_summary.json',
        help='Legacy NTU VIRAL prism-aligned summary artifact',
    )
    parser.add_argument(
        '--out',
        default='output/stress_validation_report_20260324.md',
        help='Markdown report output path',
    )
    args = parser.parse_args()

    summary_path = Path(args.benchmark_summary).expanduser().resolve()
    fresh_metrics_path = Path(args.fresh_metrics).expanduser().resolve()
    best_metrics_path = Path(args.best_metrics).expanduser().resolve()
    mid360_metrics_path = Path(args.mid360_metrics).expanduser().resolve()
    mid360_log_path = Path(args.mid360_log).expanduser().resolve()
    mid360_legacy_summary_path = Path(args.mid360_legacy_summary).expanduser().resolve()
    benchmark_readme_path = Path(args.benchmark_readme).expanduser().resolve()
    newer_college_summary_path = Path(args.newer_college_summary).expanduser().resolve()
    ntu_legacy_summary_path = Path(args.ntu_legacy_summary).expanduser().resolve()
    ntu_prism_summary_path = Path(args.ntu_prism_summary).expanduser().resolve()
    out_path = Path(args.out).expanduser().resolve()

    fresh = _load_json(fresh_metrics_path)
    best = _load_json(best_metrics_path)
    mid360_metrics = _load_json(mid360_metrics_path)
    mid360_legacy = _load_json(mid360_legacy_summary_path)
    newer_college = _load_json(newer_college_summary_path)
    ntu_legacy = _load_json(ntu_legacy_summary_path)
    ntu_prism = _load_json(ntu_prism_summary_path)

    fresh_ape = ((fresh.get('evo') or {}).get('ape') or {})
    best_ape = ((best.get('evo') or {}).get('ape') or {})
    map_verify = ((fresh.get('graph_based_slam') or {}).get('map_verify') or {})
    total_points = _extract_total_points(fresh)
    mid360_distance, mid360_loop_pair = _extract_mid360_loop(mid360_log_path)
    newer_college_path_note = _extract_newer_college_loop_phrase(benchmark_readme_path)
    mid360_ape = ((mid360_metrics.get('evo') or {}).get('ape') or {})
    mid360_raw_ape = ((mid360_metrics.get('evo') or {}).get('raw_ape') or {})
    mid360_cross = mid360_metrics.get('cross_validation') or {}

    report = f"""# Stress Validation Report

Date: {date.today().isoformat()}

## Verdict

`lidarslam_ros2` has usable long-loop evidence and a solid current default-path
benchmark on NTU VIRAL, but aggressive-motion validation for the exact `v2`
default path is still incomplete.

This is strong enough for a public `beta`, but not strong enough to claim that
the current default path is already the fully stress-validated standard across
aggressive motion and long-range loop closures.

## Current Default-Path Evidence

- dataset: `NTU VIRAL tnp_01`
- fresh default-path APE RMSE: `{_fmt_float(fresh_ape.get('rmse'))} m`
- current best APE RMSE: `{_fmt_float(best_ape.get('rmse'))} m`
- fresh map verify: `{"PASS" if map_verify.get('ok') else "FAIL"}`
- fresh pointcloud-map total points: `{total_points}`
- benchmark summary: `{summary_path}`

## Long-Loop Evidence

### MID360 Cross-Validation

- matched poses: `{mid360_ape.get('pairs', 'n/a')}`
- aligned path length: `{_fmt_float(mid360_cross.get('estimated_aligned_path_length_m'))} m`
- current-path cross-validation APE RMSE: `{_fmt_float(mid360_ape.get('rmse'))} m`
- current-path raw APE RMSE: `{_fmt_float(mid360_raw_ape.get('rmse'))} m`
- loop-search distance seen in log: `{mid360_distance} m`
- loop closure example from log: `{mid360_loop_pair}`

This is useful evidence that the graph backend has been exercised on a path of
roughly one kilometer with at least one long-range loop closure event recorded
in the log. It is also a clear sign that the current path still needs more work
on this dataset, because the current-path cross-validation error is much worse
than the NTU VIRAL result.

### MID360 Legacy Sample Context

- older sample-summary APE RMSE: `{_fmt_float(mid360_legacy.get('ape_rmse_m'))} m`

This older artifact is still useful context, but it does not override the
current-path MID360 metrics above.

### Newer College math-hard

- dataset note from benchmark README: `{newer_college_path_note}`
- reference path length: `{_fmt_float(((newer_college.get('reference') or {}).get('path_length_m')))} m`
- legacy `lidarslam` RMSE: `{_fmt_float(((newer_college.get('lidarslam') or {}).get('rmse')))} m`

This artifact is useful as a hard-dataset reminder, but it is not evidence for
the current `RKO-LIO + graph_based_slam` default path. It should not be cited
as the current release-quality benchmark.

## Aggressive-Motion / Hard-Data Evidence

- legacy NTU VIRAL summary RMSE: `{_fmt_float(((ntu_legacy.get('lidarslam') or {}).get('rmse')))} m`
- legacy NTU VIRAL prism summary RMSE: `{_fmt_float(((ntu_prism.get('lidarslam') or {}).get('rmse')))} m`

These NTU VIRAL summaries show that this repository has already been exercised
on a harder real-world sequence, but they are older report artifacts and do not
fully replace a fresh stress benchmark on the current `v2` default path.

## Release Interpretation

- safe claim: the current default path is benchmarked, dogfooded into Autoware,
  and backed by additional long-loop evidence
- unsafe claim: the current default path is already fully validated against
  aggressive motion and long-loop stress on multiple fresh benchmark datasets
- next benchmark to close the gap: rerun the current `RKO-LIO + graph_based_slam`
  path on `Newer College math-hard` and/or promote `MID360` into the same
  `metrics.json` reporting pipeline used by `NTU VIRAL`

## Source Artifacts

- fresh metrics: `{fresh_metrics_path}`
- best metrics: `{best_metrics_path}`
- benchmark summary: `{summary_path}`
- MID360 current metrics: `{mid360_metrics_path}`
- MID360 graph log: `{mid360_log_path}`
- MID360 legacy summary: `{mid360_legacy_summary_path}`
- benchmark README: `{benchmark_readme_path}`
- Newer College summary: `{newer_college_summary_path}`
- NTU VIRAL legacy summary: `{ntu_legacy_summary_path}`
- NTU VIRAL prism summary: `{ntu_prism_summary_path}`
"""

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding='utf-8')
    print(out_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
