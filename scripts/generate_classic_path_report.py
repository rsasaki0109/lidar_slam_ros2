#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Generate a short report for the Leo Drive classic scanmatcher path."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path


DEFAULT_NO_GNSS_DIR = Path(
    'output/open_data_applanix_velodyne_benchmark_driving30_no_gnss_20260325',
)
DEFAULT_GNSS_ONLY_DIR = Path(
    'output/open_data_applanix_velodyne_gnss_benchmark_driving30_20260325b',
)
DEFAULT_GNSS_IMU_DIR = Path(
    'output/open_data_applanix_velodyne_gnss_benchmark_driving30_with_imu_tf_20260325',
)


def _parse_projector_type(run_dir: Path) -> str:
    proj_path = run_dir / 'map_projector_info.yaml'
    if not proj_path.is_file():
        return 'missing'
    for line in proj_path.read_text(encoding='utf-8').splitlines():
        if line.startswith('projector_type:'):
            return line.split(':', 1)[1].strip()
    return 'unknown'


def _verify_result(run_dir: Path) -> str:
    verify_path = run_dir / 'verify_autoware_map.log'
    if not verify_path.is_file():
        return 'not_run'
    text = verify_path.read_text(encoding='utf-8', errors='replace')
    if 'RESULT: PASS' in text:
        return 'PASS'
    if 'RESULT: FAIL' in text:
        return 'FAIL'
    return 'unknown'


def _load_run(run_dir: Path) -> dict[str, object]:
    metrics_path = run_dir / 'metrics.json'
    if not metrics_path.is_file():
        raise SystemExit(f'metrics.json not found under {run_dir}')
    payload = json.loads(metrics_path.read_text(encoding='utf-8'))
    ape = payload.get('evo', {}).get('ape', {})
    graph = payload.get('graph_based_slam', {})
    return {
        'run_dir': str(run_dir),
        'ape_rmse_m': float(ape['rmse']),
        'ape_mean_m': float(ape['mean']),
        'ape_max_m': float(ape['max']),
        'ape_pairs': int(ape['pairs']),
        'loop_count': int(graph.get('loop_count', 0)),
        'loop_count_attempted': int(graph.get('loop_count_attempted', 0)),
        'projector_type': _parse_projector_type(run_dir),
        'verify_result': _verify_result(run_dir),
    }


def _fmt(value: float | None) -> str:
    if value is None:
        return '-'
    return f'{value:.3f}'


def _write_rmse_svg(out_path: Path, labels: list[str], values: list[float]) -> None:
    max_value = max(max(values), 1.0)
    bar_max_width = 420
    fills = ['#9ca3af', '#2563eb', '#1d4ed8']
    rows = []
    for idx, (label, value) in enumerate(zip(labels, values)):
        width = int(round((value / max_value) * bar_max_width))
        y = 54 + idx * 46
        rows.append(
            f'  <text x="24" y="{y + 18}" class="label">{label}</text>\n'
            f'  <rect x="180" y="{y}" width="{width}" height="24" rx="4" fill="{fills[idx % len(fills)]}"/>\n'
            f'  <text x="{190 + width}" y="{y + 18}" class="value">{value:.3f} m</text>',
        )
    height = 70 + len(rows) * 46
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="760" height="{height}" '
        f'viewBox="0 0 760 {height}">\n'
        '  <style>\n'
        '    .title { font: 600 18px sans-serif; fill: #111827; }\n'
        '    .label { font: 14px sans-serif; fill: #374151; }\n'
        '    .value { font: 600 14px sans-serif; fill: #111827; }\n'
        '  </style>\n'
        f'  <rect x="0" y="0" width="760" height="{height}" fill="#ffffff"/>\n'
        '  <text x="24" y="32" class="title">Classic path APE RMSE comparison</text>\n'
        + '\n'.join(rows) + '\n</svg>\n'
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(svg, encoding='utf-8')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate a short report comparing Leo Drive classic-path variants.',
    )
    parser.add_argument('--no-gnss-dir', default=str(DEFAULT_NO_GNSS_DIR))
    parser.add_argument('--gnss-only-dir', default=str(DEFAULT_GNSS_ONLY_DIR))
    parser.add_argument('--gnss-imu-dir', default=str(DEFAULT_GNSS_IMU_DIR))
    parser.add_argument('--out', default='')
    parser.add_argument('--write-json', default='')
    parser.add_argument('--write-svg', default='')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    no_gnss_dir = Path(args.no_gnss_dir).expanduser().resolve()
    gnss_only_dir = Path(args.gnss_only_dir).expanduser().resolve()
    gnss_imu_dir = Path(args.gnss_imu_dir).expanduser().resolve()
    out_path = (
        Path(args.out).expanduser().resolve()
        if args.out
        else (
            Path.cwd() / 'output' / f'classic_path_report_{datetime.now().strftime("%Y%m%d")}.md'
        ).resolve()
    )
    json_path = Path(args.write_json).expanduser().resolve() if args.write_json else None
    svg_path = Path(args.write_svg).expanduser().resolve() if args.write_svg else None

    no_gnss = _load_run(no_gnss_dir)
    gnss_only = _load_run(gnss_only_dir)
    gnss_imu = _load_run(gnss_imu_dir)

    gnss_gain = no_gnss['ape_rmse_m'] - gnss_only['ape_rmse_m']
    imu_delta = gnss_imu['ape_rmse_m'] - gnss_only['ape_rmse_m']

    payload = {
        'no_gnss': no_gnss,
        'gnss_only': gnss_only,
        'gnss_imu': gnss_imu,
        'gnss_gain_m': gnss_gain,
        'imu_delta_vs_gnss_only_m': imu_delta,
    }

    report = f"""# Classic Path Report

This report compares the current Leo Drive `driving_30_kmh` classic scanmatcher
path with and without backend GNSS and with the current packet IMU path.

## Inputs

- no GNSS: `{no_gnss_dir}`
- GNSS only: `{gnss_only_dir}`
- GNSS + IMU: `{gnss_imu_dir}`

## Summary

| Run | APE RMSE (m) | Mean (m) | Max (m) | APE pairs | Accepted loops | Attempted loops | Verify | Projector type |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| no GNSS | `{_fmt(no_gnss["ape_rmse_m"])}` | `{_fmt(no_gnss["ape_mean_m"])}` | `{_fmt(no_gnss["ape_max_m"])}` | `{no_gnss["ape_pairs"]}` | `{no_gnss["loop_count"]}` | `{no_gnss["loop_count_attempted"]}` | `{no_gnss["verify_result"]}` | `{no_gnss["projector_type"]}` |
| GNSS only | `{_fmt(gnss_only["ape_rmse_m"])}` | `{_fmt(gnss_only["ape_mean_m"])}` | `{_fmt(gnss_only["ape_max_m"])}` | `{gnss_only["ape_pairs"]}` | `{gnss_only["loop_count"]}` | `{gnss_only["loop_count_attempted"]}` | `{gnss_only["verify_result"]}` | `{gnss_only["projector_type"]}` |
| GNSS + IMU | `{_fmt(gnss_imu["ape_rmse_m"])}` | `{_fmt(gnss_imu["ape_mean_m"])}` | `{_fmt(gnss_imu["ape_max_m"])}` | `{gnss_imu["ape_pairs"]}` | `{gnss_imu["loop_count"]}` | `{gnss_imu["loop_count_attempted"]}` | `{gnss_imu["verify_result"]}` | `{gnss_imu["projector_type"]}` |

## Conclusion

- Backend GNSS improves APE RMSE by `{_fmt(gnss_gain)}` m relative to the no-GNSS classic path.
- The current GNSS + IMU packet path changes APE RMSE by `{_fmt(imu_delta)}` m relative to GNSS-only.
- All three runs still produce map bundles that can be checked independently of the APE numbers.
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding='utf-8')
    if json_path is not None:
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    if svg_path is not None:
        _write_rmse_svg(
            svg_path,
            ['no GNSS', 'GNSS only', 'GNSS + IMU'],
            [
                no_gnss['ape_rmse_m'],
                gnss_only['ape_rmse_m'],
                gnss_imu['ape_rmse_m'],
            ],
        )

    print(out_path)
    if json_path is not None:
        print(json_path)
    if svg_path is not None:
        print(svg_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
