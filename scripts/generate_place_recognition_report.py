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

"""Generate a short place-recognition comparison report."""

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
        'accepted_source_counts': {
            'distance': 0,
            'scan_context': 0,
            'bev_descriptor': 0,
            'solid_descriptor': 0,
        },
        'scan_context_candidate_count': 0,
        'bev_rerank_hint_count': 0,
        'solid_rerank_candidate_count': 0,
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
    summary['bev_rerank_hint_count'] = len(
        re.findall(r'BEV rerank hint:', text),
    )
    summary['solid_rerank_candidate_count'] = len(
        re.findall(r'SOLiD rerank candidate:', text),
    )
    for source in re.findall(r'loop_candidate_source:([a-z_]+)', text):
        counts = summary['accepted_source_counts']
        counts[source] = counts.get(source, 0) + 1
    return summary


def _fmt(value: float | None) -> str:
    if value is None:
        return '-'
    return f'{value:.3f}'


def _write_rmse_svg(
    out_path: Path,
    baseline_rmse: float | None,
    candidate_rmse: float | None,
    baseline_label: str,
    candidate_label: str,
) -> None:
    baseline_value = baseline_rmse or 0.0
    candidate_value = candidate_rmse or 0.0
    max_value = max(baseline_value, candidate_value, 1.0)
    bar_max_width = 420
    baseline_width = int(round((baseline_value / max_value) * bar_max_width))
    candidate_width = int(round((candidate_value / max_value) * bar_max_width))
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="760" height="180" viewBox="0 0 760 180">
  <style>
    .title {{ font: 600 18px sans-serif; fill: #111827; }}
    .label {{ font: 14px sans-serif; fill: #374151; }}
    .value {{ font: 600 14px sans-serif; fill: #111827; }}
  </style>
  <rect x="0" y="0" width="760" height="180" fill="#ffffff"/>
  <text x="24" y="32" class="title">Place recognition APE RMSE comparison</text>
  <text x="24" y="74" class="label">{baseline_label}</text>
  <rect x="180" y="54" width="{baseline_width}" height="24" rx="4" fill="#9ca3af"/>
  <text x="{190 + baseline_width}" y="72" class="value">{baseline_value:.3f} m</text>
  <text x="24" y="124" class="label">{candidate_label}</text>
  <rect x="180" y="104" width="{candidate_width}" height="24" rx="4" fill="#2563eb"/>
  <text x="{190 + candidate_width}" y="122" class="value">{candidate_value:.3f} m</text>
</svg>
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(svg, encoding='utf-8')


def _conclusion(
    baseline_rmse: float | None,
    candidate_rmse: float | None,
    candidate_log: dict[str, object],
    candidate_kind: str,
    candidate_label: str,
) -> str:
    accepted_counts = candidate_log.get('accepted_source_counts', {})
    if candidate_kind == 'scan_context':
        accepted = int(accepted_counts.get('scan_context', 0))
        observed = int(candidate_log.get('scan_context_candidate_count', 0))
        if accepted > 0:
            source_text = 'accepted loop closures from Scan Context'
        elif observed > 0:
            source_text = (
                'Scan Context produced candidates, but none survived geometric validation'
            )
        else:
            source_text = 'no Scan Context candidate made it into the accepted loop set'
    elif candidate_kind == 'bev_rerank':
        observed = int(candidate_log.get('bev_rerank_hint_count', 0))
        if observed > 0:
            source_text = (
                f'{candidate_label} reprioritized distance candidates with BEV hints'
            )
        else:
            source_text = f'{candidate_label} produced no usable BEV rerank hint'
    elif candidate_kind == 'solid_descriptor':
        accepted = int(accepted_counts.get('solid_descriptor', 0))
        observed = int(candidate_log.get('solid_rerank_candidate_count', 0))
        if accepted > 0:
            source_text = 'accepted loop closures from SOLiD-based reranking'
        elif observed > 0:
            source_text = 'SOLiD produced rerank candidates, but none survived geometric validation'
        else:
            source_text = 'no SOLiD rerank candidate made it into the accepted loop set'
    else:
        source_text = f'{candidate_label} completed'

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
        help='metrics.json for the candidate run.',
    )
    parser.add_argument(
        '--candidate-log',
        default='',
        help='Optional launch log for the candidate run.',
    )
    parser.add_argument(
        '--baseline-label',
        default='baseline',
        help='Human-readable baseline label for the report.',
    )
    parser.add_argument(
        '--candidate-label',
        default='candidate',
        help='Human-readable candidate label for the report.',
    )
    parser.add_argument(
        '--candidate-kind',
        default='scan_context',
        choices=['scan_context', 'bev_rerank', 'solid_descriptor', 'generic'],
        help='Descriptor family used by the candidate run.',
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
    parser.add_argument(
        '--write-svg',
        default='',
        help='Optional SVG summary path.',
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
    svg_path = Path(args.write_svg).expanduser().resolve() if args.write_svg else None
    payload = {
        'baseline_metrics': str(baseline_metrics_path),
        'baseline_log': str(baseline_log),
        'candidate_metrics': str(candidate_metrics_path),
        'candidate_log': str(candidate_log),
        'baseline': {
            'label': args.baseline_label,
            'ape_rmse_m': baseline_rmse,
            'loop_count': _loop_count(baseline, 'loop_count'),
            'loop_count_attempted': _loop_count(baseline, 'loop_count_attempted'),
            'log_summary': baseline_log_summary,
        },
        'candidate': {
            'label': args.candidate_label,
            'kind': args.candidate_kind,
            'ape_rmse_m': candidate_rmse,
            'loop_count': _loop_count(candidate, 'loop_count'),
            'loop_count_attempted': _loop_count(candidate, 'loop_count_attempted'),
            'log_summary': candidate_log_summary,
        },
    }

    report = f"""# Place Recognition Report

This report compares a fair current-code MID360 baseline rerun against an
experimental place-recognition candidate run.

## Inputs

- baseline metrics: `{baseline_metrics_path}`
- baseline log: `{baseline_log}`
- candidate metrics: `{candidate_metrics_path}`
- candidate log: `{candidate_log}`

## Summary

| Run | Runtime `use_scan_context` | APE RMSE (m) | Accepted loops | Attempted loops | Accepted distance loops | Accepted Scan Context loops | Accepted BEV loops | Accepted SOLiD loops | Observed Scan Context candidates | Observed BEV rerank hints | Observed SOLiD rerank candidates |
| --- | --- | --- | --- | --- | --- | --- | --- |
| {args.baseline_label} | `{baseline_log_summary.get("use_scan_context")}` | `{_fmt(baseline_rmse)}` | `{_loop_count(baseline, "loop_count")}` | `{_loop_count(baseline, "loop_count_attempted")}` | `{baseline_log_summary["accepted_source_counts"].get("distance", 0)}` | `{baseline_log_summary["accepted_source_counts"].get("scan_context", 0)}` | `{baseline_log_summary["accepted_source_counts"].get("bev_descriptor", 0)}` | `{baseline_log_summary["accepted_source_counts"].get("solid_descriptor", 0)}` | `{baseline_log_summary.get("scan_context_candidate_count", 0)}` | `{baseline_log_summary.get("bev_rerank_hint_count", 0)}` | `{baseline_log_summary.get("solid_rerank_candidate_count", 0)}` |
| {args.candidate_label} | `{candidate_log_summary.get("use_scan_context")}` | `{_fmt(candidate_rmse)}` | `{_loop_count(candidate, "loop_count")}` | `{_loop_count(candidate, "loop_count_attempted")}` | `{candidate_log_summary["accepted_source_counts"].get("distance", 0)}` | `{candidate_log_summary["accepted_source_counts"].get("scan_context", 0)}` | `{candidate_log_summary["accepted_source_counts"].get("bev_descriptor", 0)}` | `{candidate_log_summary["accepted_source_counts"].get("solid_descriptor", 0)}` | `{candidate_log_summary.get("scan_context_candidate_count", 0)}` | `{candidate_log_summary.get("bev_rerank_hint_count", 0)}` | `{candidate_log_summary.get("solid_rerank_candidate_count", 0)}` |

## Conclusion

- {_conclusion(baseline_rmse, candidate_rmse, candidate_log_summary, args.candidate_kind, args.candidate_label)}
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding='utf-8')
    if json_path is not None:
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    if svg_path is not None:
        _write_rmse_svg(
            svg_path,
            baseline_rmse,
            candidate_rmse,
            args.baseline_label,
            args.candidate_label,
        )
    print(out_path)
    if json_path is not None:
        print(json_path)
    if svg_path is not None:
        print(svg_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
