#!/usr/bin/env python3
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

"""Generate a short Scan Context on/off comparison report."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re


DEFAULT_BASELINE_METRICS = Path(
    'output/bench_rko_lio_mid360_current_default_rerun_20260326/metrics.json',
)
DEFAULT_CANDIDATE_METRICS = Path(
    'output/bench_rko_lio_mid360_sc055_yawguess_scagg_screg_20260326/metrics.json',
)


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _ape_rmse(metrics: dict) -> float | None:
    evo = metrics.get('evo') or {}
    ape = evo.get('ape') if isinstance(evo, dict) else None
    if not isinstance(ape, dict):
        return None
    value = ape.get('rmse')
    try:
        return float(value)
    except Exception:
        return None


def _loop_count(metrics: dict, key: str) -> int:
    graph = metrics.get('graph_based_slam') or {}
    value = graph.get(key)
    try:
        return int(value)
    except Exception:
        return 0


def parse_log_summary(log_path: Path) -> dict[str, object]:
    """Extract place-recognition related counters from a launch log."""
    summary = {
        'use_scan_context': None,
        'accepted_source_counts': {'distance': 0, 'scan_context': 0},
        'scan_context_candidate_count': 0,
    }
    if not log_path.is_file():
        return summary

    text = log_path.read_text(encoding='utf-8', errors='replace')
    use_scan_context = re.search(r'use_scan_context:(true|false)', text)
    if use_scan_context:
        summary['use_scan_context'] = use_scan_context.group(1) == 'true'

    summary['scan_context_candidate_count'] = len(
        re.findall(r'ScanContext loop candidate:', text),
    )
    for source in re.findall(r'loop_candidate_source:([a-z_]+)', text):
        counts = summary['accepted_source_counts']
        counts[source] = counts.get(source, 0) + 1
    return summary


def _fmt(value: float | None) -> str:
    if value is None:
        return '-'
    return f'{value:.3f}'


def _conclusion(
    baseline_rmse: float | None,
    candidate_rmse: float | None,
    candidate_log: dict[str, object],
) -> str:
    scan_context_loops = (
        candidate_log.get('accepted_source_counts', {}).get('scan_context', 0)
    )
    scan_context_candidates = int(candidate_log.get('scan_context_candidate_count', 0))
    if scan_context_loops > 0:
        source_text = 'accepted loop closures from Scan Context'
    elif scan_context_candidates > 0:
        source_text = 'Scan Context produced candidates, but none survived geometric validation'
    else:
        source_text = 'no Scan Context candidate made it into the accepted loop set'

    if baseline_rmse is None or candidate_rmse is None:
        return source_text
    delta = candidate_rmse - baseline_rmse
    if delta < -0.01:
        return (
            f'{source_text}; enabling Scan Context improved APE RMSE by '
            f'{abs(delta):.3f} m'
        )
    if delta > 0.01:
        return (
            f'{source_text}; enabling Scan Context regressed APE RMSE by '
            f'{delta:.3f} m'
        )
    return f'{source_text}; APE RMSE stayed effectively unchanged'


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Generate a Scan Context on/off benchmark comparison report.',
    )
    parser.add_argument(
        '--baseline-metrics',
        default=str(DEFAULT_BASELINE_METRICS),
        help='metrics.json for the distance-only baseline run.',
    )
    parser.add_argument(
        '--baseline-log',
        default='',
        help='Optional launch log for the baseline run.',
    )
    parser.add_argument(
        '--candidate-metrics',
        default=str(DEFAULT_CANDIDATE_METRICS),
        help='metrics.json for the Scan Context enabled run.',
    )
    parser.add_argument(
        '--candidate-log',
        default='',
        help='Optional launch log for the Scan Context enabled run.',
    )
    parser.add_argument(
        '--out',
        default='',
        help='Output markdown path (default: output/place_recognition_report_<YYYYMMDD>.md).',
    )
    parser.add_argument(
        '--write-json',
        default='',
        help='Optional JSON summary path.',
    )
    return parser.parse_args()


def main() -> int:
    """Generate the markdown report."""
    args = parse_args()
    baseline_metrics_path = Path(args.baseline_metrics).expanduser().resolve()
    candidate_metrics_path = Path(args.candidate_metrics).expanduser().resolve()
    if not baseline_metrics_path.is_file():
        raise SystemExit(f'baseline metrics not found: {baseline_metrics_path}')
    if not candidate_metrics_path.is_file():
        raise SystemExit(f'candidate metrics not found: {candidate_metrics_path}')

    baseline = _load_json(baseline_metrics_path)
    candidate = _load_json(candidate_metrics_path)

    baseline_log = (
        Path(args.baseline_log).expanduser().resolve()
        if args.baseline_log else baseline_metrics_path.parent / 'slam.launch.log'
    )
    candidate_log = (
        Path(args.candidate_log).expanduser().resolve()
        if args.candidate_log else candidate_metrics_path.parent / 'slam.launch.log'
    )
    baseline_log_summary = parse_log_summary(baseline_log)
    candidate_log_summary = parse_log_summary(candidate_log)

    baseline_rmse = _ape_rmse(baseline)
    candidate_rmse = _ape_rmse(candidate)
    out_path = (
        Path(args.out).expanduser().resolve()
        if args.out
        else (
            Path.cwd()
            / 'output'
            / f'place_recognition_report_{datetime.now().strftime("%Y%m%d")}.md'
        ).resolve()
    )
    json_path = Path(args.write_json).expanduser().resolve() if args.write_json else None
    payload = {
        'baseline_metrics': str(baseline_metrics_path),
        'baseline_log': str(baseline_log),
        'candidate_metrics': str(candidate_metrics_path),
        'candidate_log': str(candidate_log),
        'baseline': {
            'ape_rmse_m': baseline_rmse,
            'loop_count': _loop_count(baseline, 'loop_count'),
            'loop_count_attempted': _loop_count(baseline, 'loop_count_attempted'),
            'log_summary': baseline_log_summary,
        },
        'candidate': {
            'ape_rmse_m': candidate_rmse,
            'loop_count': _loop_count(candidate, 'loop_count'),
            'loop_count_attempted': _loop_count(candidate, 'loop_count_attempted'),
            'log_summary': candidate_log_summary,
        },
    }

    report = f"""# Place Recognition Report

This report compares a fair current-code MID360 baseline rerun against a Scan
Context candidate run.

## Inputs

- baseline metrics: `{baseline_metrics_path}`
- baseline log: `{baseline_log}`
- candidate metrics: `{candidate_metrics_path}`
- candidate log: `{candidate_log}`

## Summary

| Run | Runtime `use_scan_context` | APE RMSE (m) | Accepted loops | Attempted loops | Accepted distance loops | Accepted Scan Context loops | Observed Scan Context candidates |
| --- | --- | --- | --- | --- | --- | --- | --- |
| baseline | `{baseline_log_summary.get("use_scan_context")}` | `{_fmt(baseline_rmse)}` | `{_loop_count(baseline, "loop_count")}` | `{_loop_count(baseline, "loop_count_attempted")}` | `{baseline_log_summary["accepted_source_counts"].get("distance", 0)}` | `{baseline_log_summary["accepted_source_counts"].get("scan_context", 0)}` | `{baseline_log_summary.get("scan_context_candidate_count", 0)}` |
| candidate | `{candidate_log_summary.get("use_scan_context")}` | `{_fmt(candidate_rmse)}` | `{_loop_count(candidate, "loop_count")}` | `{_loop_count(candidate, "loop_count_attempted")}` | `{candidate_log_summary["accepted_source_counts"].get("distance", 0)}` | `{candidate_log_summary["accepted_source_counts"].get("scan_context", 0)}` | `{candidate_log_summary.get("scan_context_candidate_count", 0)}` |

## Conclusion

- {_conclusion(baseline_rmse, candidate_rmse, candidate_log_summary)}
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding='utf-8')
    if json_path is not None:
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    print(out_path)
    if json_path is not None:
        print(json_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
