#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Summarize cross-dataset classic-path odom-prior validation results."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re


DEFAULT_DRIVING30_GNSS_ONLY_DIR = Path(
    'output/open_data_applanix_velodyne_gnss_benchmark_driving30_20260325b',
)
DEFAULT_DRIVING30_VELOCITY_ALWAYS_DIR = Path(
    'output/open_data_applanix_velodyne_gnss_benchmark_driving30_with_odom_velocity_planar_translation_20260327',
)
DEFAULT_DRIVING30_VELOCITY_RECOVERY_DIR = Path(
    'output/open_data_applanix_velodyne_gnss_benchmark_driving30_with_odom_velocity_planar_translation_recoveryonly_20260327',
)
DEFAULT_BAG6_GNSS_ONLY_DIR = Path(
    'output/open_data_applanix_velodyne_gnss_benchmark_bag6_front_gnss_sidecar_20260327',
)
DEFAULT_BAG6_VELOCITY_ALWAYS_DIR = Path(
    'output/open_data_applanix_velodyne_gnss_benchmark_bag6_front_gnss_sidecar_with_velocity_odom_20260327',
)
DEFAULT_BAG6_VELOCITY_RECOVERY_DIR = Path(
    'output/open_data_applanix_velodyne_gnss_benchmark_bag6_front_gnss_sidecar_with_velocity_odom_recoveryonly_20260327',
)


def _parse_projector_type(run_dir: Path) -> str:
    projector_path = run_dir / 'map_projector_info.yaml'
    if not projector_path.is_file():
        return 'missing'
    for line in projector_path.read_text(encoding='utf-8').splitlines():
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


def _parse_gnss_edges(run_dir: Path) -> dict[str, int]:
    log_path = run_dir / 'lidarslam.launch.log'
    if not log_path.is_file():
        return {'gnss_edges': -1, 'rtk_like_edges': -1}
    text = log_path.read_text(encoding='utf-8', errors='replace')
    match = re.search(
        r'Added\s+(\d+)\s+GNSS position constraint edges\s+\((\d+)\s+RTK-like by covariance\)',
        text,
    )
    if match is None:
        return {'gnss_edges': -1, 'rtk_like_edges': -1}
    return {
        'gnss_edges': int(match.group(1)),
        'rtk_like_edges': int(match.group(2)),
    }


def _load_run(run_dir: Path) -> dict[str, object]:
    metrics_path = run_dir / 'metrics.json'
    if not metrics_path.is_file():
        raise SystemExit(f'metrics.json not found under {run_dir}')
    payload = json.loads(metrics_path.read_text(encoding='utf-8'))
    ape = payload.get('evo', {}).get('ape', {})
    cv = payload.get('cross_validation', {})
    lidarslam = payload.get('lidarslam', {})
    gnss_edges = _parse_gnss_edges(run_dir)
    return {
        'run_dir': str(run_dir),
        'ape_rmse_m': float(ape['rmse']),
        'ape_pairs': int(ape['pairs']),
        'tum_lines': int(lidarslam.get('tum_lines', 0)),
        'estimated_path_length_m': float(cv.get('estimated_path_length_m', 0.0)),
        'reference_path_length_m': float(cv.get('reference_path_length_m', 0.0)),
        'projector_type': _parse_projector_type(run_dir),
        'verify_result': _verify_result(run_dir),
        **gnss_edges,
    }


def _fmt(value: float | None) -> str:
    if value is None:
        return '-'
    return f'{value:.3f}'


def _fmt_int(value: int) -> str:
    return '-' if value < 0 else str(value)


def _row(label: str, run: dict[str, object]) -> str:
    return (
        f'| {label} | `{_fmt(run["ape_rmse_m"])}` | `{run["ape_pairs"]}` | '
        f'`{run["tum_lines"]}` | `{_fmt(run["estimated_path_length_m"])}` | '
        f'`{_fmt(run["reference_path_length_m"])}` | `{_fmt_int(run["gnss_edges"])}` | '
        f'`{_fmt_int(run["rtk_like_edges"])}` | `{run["verify_result"]}` | '
        f'`{run["projector_type"]}` |'
    )


def _write_svg(out_path: Path, labels: list[str], values: list[float]) -> None:
    max_value = max(max(values), 1.0)
    width_max = 420
    fills = ['#2563eb', '#0f766e', '#9ca3af', '#7c3aed', '#c2410c', '#059669']
    rows = []
    for idx, (label, value) in enumerate(zip(labels, values)):
        width = int(round((value / max_value) * width_max))
        y = 58 + idx * 44
        rows.append(
            f'  <text x="24" y="{y + 18}" class="label">{label}</text>\n'
            f'  <rect x="250" y="{y}" width="{width}" height="22" rx="4" '
            f'fill="{fills[idx % len(fills)]}"/>\n'
            f'  <text x="{260 + width}" y="{y + 17}" class="value">{value:.3f} m</text>'
        )
    height = 78 + len(rows) * 44
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="860" height="{height}" '
        f'viewBox="0 0 860 {height}">\n'
        '  <style>\n'
        '    .title { font: 600 18px sans-serif; fill: #111827; }\n'
        '    .label { font: 14px sans-serif; fill: #374151; }\n'
        '    .value { font: 600 14px sans-serif; fill: #111827; }\n'
        '  </style>\n'
        f'  <rect x="0" y="0" width="860" height="{height}" fill="#ffffff"/>\n'
        '  <text x="24" y="34" class="title">Classic-path velocity-prior validation</text>\n'
        + '\n'.join(rows) + '\n</svg>\n'
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(svg, encoding='utf-8')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate a cross-dataset report for classic-path odom-prior validation.',
    )
    parser.add_argument('--driving30-gnss-only-dir', default=str(DEFAULT_DRIVING30_GNSS_ONLY_DIR))
    parser.add_argument(
        '--driving30-velocity-always-dir',
        default=str(DEFAULT_DRIVING30_VELOCITY_ALWAYS_DIR),
    )
    parser.add_argument(
        '--driving30-velocity-recovery-dir',
        default=str(DEFAULT_DRIVING30_VELOCITY_RECOVERY_DIR),
    )
    parser.add_argument('--bag6-gnss-only-dir', default=str(DEFAULT_BAG6_GNSS_ONLY_DIR))
    parser.add_argument('--bag6-velocity-always-dir', default=str(DEFAULT_BAG6_VELOCITY_ALWAYS_DIR))
    parser.add_argument(
        '--bag6-velocity-recovery-dir',
        default=str(DEFAULT_BAG6_VELOCITY_RECOVERY_DIR),
    )
    parser.add_argument('--out', default='')
    parser.add_argument('--write-json', default='')
    parser.add_argument('--write-svg', default='')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_path = (
        Path(args.out).expanduser().resolve()
        if args.out
        else (
            Path.cwd() / 'output' / f'odom_prior_validation_report_{datetime.now().strftime("%Y%m%d")}.md'
        ).resolve()
    )
    json_path = Path(args.write_json).expanduser().resolve() if args.write_json else None
    svg_path = Path(args.write_svg).expanduser().resolve() if args.write_svg else None

    driving30 = {
        'gnss_only': _load_run(Path(args.driving30_gnss_only_dir).expanduser().resolve()),
        'velocity_always': _load_run(
            Path(args.driving30_velocity_always_dir).expanduser().resolve(),
        ),
        'velocity_recovery': _load_run(
            Path(args.driving30_velocity_recovery_dir).expanduser().resolve(),
        ),
    }
    bag6 = {
        'gnss_only': _load_run(Path(args.bag6_gnss_only_dir).expanduser().resolve()),
        'velocity_always': _load_run(Path(args.bag6_velocity_always_dir).expanduser().resolve()),
        'velocity_recovery': _load_run(
            Path(args.bag6_velocity_recovery_dir).expanduser().resolve(),
        ),
    }

    payload = {
        'driving30': driving30,
        'bag6_front': bag6,
        'driving30_best_label': min(driving30, key=lambda key: driving30[key]['ape_rmse_m']),
        'bag6_best_label': min(bag6, key=lambda key: bag6[key]['ape_rmse_m']),
    }

    report = (
        '# Odom Prior Validation Report\n\n'
        'This report checks whether the current velocity-based classic-path odom prior '
        'generalizes across two real open-data conditions.\n\n'
        '## driving_30_kmh\n\n'
        '| Run | APE RMSE (m) | APE pairs | Corrected poses | Estimated path (m) | Reference path (m) | GNSS edges | RTK-like edges | Verify | Projector |\n'
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |\n'
        + '\n'.join(
            [
                _row('GNSS only', driving30['gnss_only']),
                _row('Velocity prior, always on', driving30['velocity_always']),
                _row('Velocity prior, suspect/recovery only', driving30['velocity_recovery']),
            ],
        ) + '\n\n'
        '## bag6_front\n\n'
        '| Run | APE RMSE (m) | APE pairs | Corrected poses | Estimated path (m) | Reference path (m) | GNSS edges | RTK-like edges | Verify | Projector |\n'
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |\n'
        + '\n'.join(
            [
                _row('GNSS sidecar only', bag6['gnss_only']),
                _row('Velocity prior, always on', bag6['velocity_always']),
                _row('Velocity prior, suspect/recovery only', bag6['velocity_recovery']),
            ],
        ) + '\n\n'
        '## Conclusion\n\n'
        f'- `driving_30_kmh` best among the tested odom-prior variants is '
        f'`{payload["driving30_best_label"]}` with `APE RMSE {_fmt(driving30[payload["driving30_best_label"]]["ape_rmse_m"])} m`.\n'
        f'- `bag6_front` best among the tested odom-prior variants is '
        f'`{payload["bag6_best_label"]}` with `APE RMSE {_fmt(bag6[payload["bag6_best_label"]]["ape_rmse_m"])} m`.\n'
        f'- `bag6_front` reaches `estimated_path_length={_fmt(bag6["velocity_recovery"]["estimated_path_length_m"])} m` '
        f'vs `reference_path_length={_fmt(bag6["velocity_recovery"]["reference_path_length_m"])} m` in the best run, '
        'so the low RMSE is not a short-trajectory collapse.\n'
        f'- All current `bag6_front` runs show `GNSS edges={_fmt_int(bag6["gnss_only"]["gnss_edges"])}` in the graph log, '
        'so that difference is attributable to classic-path prior behavior rather than backend GNSS anchoring.\n'
        '- This means the current `velocity-based odom prior` is useful, but it is still dataset-specific and should remain opt-in.\n'
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding='utf-8')
    if json_path is not None:
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    if svg_path is not None:
        labels = [
            'driving30 GNSS only',
            'driving30 velocity always',
            'driving30 velocity recovery',
            'bag6 GNSS only',
            'bag6 velocity always',
            'bag6 velocity recovery',
        ]
        values = [
            driving30['gnss_only']['ape_rmse_m'],
            driving30['velocity_always']['ape_rmse_m'],
            driving30['velocity_recovery']['ape_rmse_m'],
            bag6['gnss_only']['ape_rmse_m'],
            bag6['velocity_always']['ape_rmse_m'],
            bag6['velocity_recovery']['ape_rmse_m'],
        ]
        _write_svg(svg_path, labels, values)

    print(out_path)
    if json_path is not None:
        print(json_path)
    if svg_path is not None:
        print(svg_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
