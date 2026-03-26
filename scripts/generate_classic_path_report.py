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
DEFAULT_GNSS_ODOM_DIR = Path(
    'output/open_data_applanix_velodyne_gnss_benchmark_driving30_with_odom_velocity_planar_translation_20260327',
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
    fills = ['#9ca3af', '#2563eb', '#0f766e', '#1d4ed8']
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
    parser.add_argument('--gnss-odom-dir', default='')
    parser.add_argument('--gnss-imu-dir', default=str(DEFAULT_GNSS_IMU_DIR))
    parser.add_argument('--out', default='')
    parser.add_argument('--write-json', default='')
    parser.add_argument('--write-svg', default='')
    return parser.parse_args()


def _row(label: str, run: dict[str, object]) -> str:
    return (
        f'| {label} | `{_fmt(run["ape_rmse_m"])}` | `{_fmt(run["ape_mean_m"])}` | '
        f'`{_fmt(run["ape_max_m"])}` | `{run["ape_pairs"]}` | `{run["loop_count"]}` | '
        f'`{run["loop_count_attempted"]}` | `{run["verify_result"]}` | '
        f'`{run["projector_type"]}` |'
    )


def main() -> int:
    args = parse_args()
    no_gnss_dir = Path(args.no_gnss_dir).expanduser().resolve()
    gnss_only_dir = Path(args.gnss_only_dir).expanduser().resolve()
    gnss_odom_dir = Path(args.gnss_odom_dir).expanduser().resolve() if args.gnss_odom_dir else None
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
    gnss_odom = _load_run(gnss_odom_dir) if gnss_odom_dir is not None else None
    gnss_imu = _load_run(gnss_imu_dir)

    gnss_gain = no_gnss['ape_rmse_m'] - gnss_only['ape_rmse_m']
    odom_delta = (
        gnss_odom['ape_rmse_m'] - gnss_only['ape_rmse_m']
        if gnss_odom is not None else None
    )
    imu_delta = gnss_imu['ape_rmse_m'] - gnss_only['ape_rmse_m']

    payload = {
        'no_gnss': no_gnss,
        'gnss_only': gnss_only,
        'gnss_odom_prior': gnss_odom,
        'gnss_imu': gnss_imu,
        'gnss_gain_m': gnss_gain,
        'odom_delta_vs_gnss_only_m': odom_delta,
        'imu_delta_vs_gnss_only_m': imu_delta,
    }

    input_lines = [
        f'- no GNSS: `{no_gnss_dir}`',
        f'- GNSS only: `{gnss_only_dir}`',
    ]
    summary_rows = [
        _row('no GNSS', no_gnss),
        _row('GNSS only', gnss_only),
    ]
    conclusion_lines = [
        f'- Backend GNSS improves APE RMSE by `{_fmt(gnss_gain)}` m relative to the no-GNSS classic path.',
    ]
    svg_labels = ['no GNSS', 'GNSS only']
    svg_values = [no_gnss['ape_rmse_m'], gnss_only['ape_rmse_m']]

    if gnss_odom is not None:
        input_lines.append(f'- GNSS + odom prior: `{gnss_odom_dir}`')
        summary_rows.append(_row('GNSS + odom prior', gnss_odom))
        conclusion_lines.append(
            f'- The current GNSS + odom prior path changes APE RMSE by '
            f'`{_fmt(odom_delta)}` m relative to GNSS-only.',
        )
        svg_labels.append('GNSS + odom prior')
        svg_values.append(gnss_odom['ape_rmse_m'])

    input_lines.append(f'- GNSS + IMU: `{gnss_imu_dir}`')
    summary_rows.append(_row('GNSS + IMU', gnss_imu))
    conclusion_lines.append(
        f'- The current GNSS + IMU packet path changes APE RMSE by '
        f'`{_fmt(imu_delta)}` m relative to GNSS-only.',
    )
    conclusion_lines.append(
        '- All runs still produce map bundles that can be checked independently of the APE numbers.',
    )
    svg_labels.append('GNSS + IMU')
    svg_values.append(gnss_imu['ape_rmse_m'])

    report = (
        '# Classic Path Report\n\n'
        'This report compares the current Leo Drive `driving_30_kmh` classic '
        'scanmatcher path with and without backend GNSS, with a frontend odom '
        'prior, and with the current packet IMU path.\n\n'
        '## Inputs\n\n'
        + '\n'.join(input_lines) + '\n\n'
        '## Summary\n\n'
        '| Run | APE RMSE (m) | Mean (m) | Max (m) | APE pairs | Accepted loops | Attempted loops | Verify | Projector type |\n'
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |\n'
        + '\n'.join(summary_rows) + '\n\n'
        '## Conclusion\n\n'
        + '\n'.join(conclusion_lines) + '\n'
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding='utf-8')
    if json_path is not None:
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    if svg_path is not None:
        _write_rmse_svg(svg_path, svg_labels, svg_values)

    print(out_path)
    if json_path is not None:
        print(json_path)
    if svg_path is not None:
        print(svg_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
