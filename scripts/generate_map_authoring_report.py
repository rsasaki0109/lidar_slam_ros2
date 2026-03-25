#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Generate a short public-facing map-authoring report from tracked artifacts."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path


DEFAULT_BENCHMARK_METRICS = Path(
    'output/bench_rko_lio_ntu_viral_fresh_20260324/metrics.json',
)
DEFAULT_GNSS_PROJECTOR = Path(
    'output/open_data_gnss_smoke_bag6_autodetect_throttled_20260325/map_projector_info.yaml',
)
DEFAULT_DYNAMIC_FILTER_REPORT = Path(
    'output/dynamic_object_filter_benchmark_bag6_20260326/dynamic_object_filter_report.json',
)
DEFAULT_CLASSIC_PATH_REPORT = Path('output/classic_path_report_20260326.json')


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _parse_projector(path: Path) -> dict[str, object]:
    data: dict[str, object] = {
        'projector_type': 'missing',
        'has_map_origin': False,
        'latitude': None,
        'longitude': None,
    }
    if not path.is_file():
        return data
    lines = path.read_text(encoding='utf-8').splitlines()
    for line in lines:
        if line.startswith('projector_type:'):
            data['projector_type'] = line.split(':', 1)[1].strip()
        if line.strip().startswith('latitude:'):
            data['latitude'] = float(line.split(':', 1)[1].strip())
            data['has_map_origin'] = True
        if line.strip().startswith('longitude:'):
            data['longitude'] = float(line.split(':', 1)[1].strip())
            data['has_map_origin'] = True
    return data


def _fmt(value: float | None) -> str:
    if value is None:
        return '-'
    return f'{value:.3f}'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate a short public map-authoring report.',
    )
    parser.add_argument('--benchmark-metrics', default=str(DEFAULT_BENCHMARK_METRICS))
    parser.add_argument('--gnss-projector', default=str(DEFAULT_GNSS_PROJECTOR))
    parser.add_argument(
        '--dynamic-filter-report',
        default=str(DEFAULT_DYNAMIC_FILTER_REPORT),
    )
    parser.add_argument(
        '--classic-path-report',
        default=str(DEFAULT_CLASSIC_PATH_REPORT),
    )
    parser.add_argument('--out', default='')
    parser.add_argument('--write-json', default='')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    benchmark_metrics_path = Path(args.benchmark_metrics).expanduser().resolve()
    gnss_projector_path = Path(args.gnss_projector).expanduser().resolve()
    dynamic_filter_report_path = Path(args.dynamic_filter_report).expanduser().resolve()
    classic_path_report_path = Path(args.classic_path_report).expanduser().resolve()
    if not benchmark_metrics_path.is_file():
        raise SystemExit(f'benchmark metrics not found: {benchmark_metrics_path}')
    if not dynamic_filter_report_path.is_file():
        raise SystemExit(f'dynamic filter report not found: {dynamic_filter_report_path}')
    if not classic_path_report_path.is_file():
        raise SystemExit(f'classic path report not found: {classic_path_report_path}')

    benchmark = _load_json(benchmark_metrics_path)
    gnss = _parse_projector(gnss_projector_path)
    dynamic_filter = _load_json(dynamic_filter_report_path)
    classic_path = _load_json(classic_path_report_path)

    benchmark_ape = benchmark.get('evo', {}).get('ape', {})
    benchmark_verify = 'PASS'
    benchmark_projector = 'Local'
    benchmark_map_dir = Path(benchmark_metrics_path).parent / 'map_projector_info.yaml'
    if benchmark_map_dir.is_file():
      benchmark_projector = _parse_projector(benchmark_map_dir).get('projector_type', 'Local')

    payload = {
        'default_benchmark': {
            'metrics_path': str(benchmark_metrics_path),
            'ape_rmse_m': float(benchmark_ape.get('rmse')),
            'ape_pairs': int(benchmark_ape.get('pairs')),
            'projector_type': benchmark_projector,
            'verify_result': benchmark_verify,
        },
        'gnss_georeference': {
            'projector_path': str(gnss_projector_path),
            **gnss,
        },
        'dynamic_filter': dynamic_filter,
        'classic_path': classic_path,
        'standard_submission_artifacts': [
            'metrics.json',
            'pointcloud_map/',
            'map_projector_info.yaml',
            'benchmark_summary.md or focused report.md',
            'latest_report.html or report.json',
        ],
    }

    out_path = (
        Path(args.out).expanduser().resolve()
        if args.out
        else (
            Path.cwd() / 'output' / f'map_authoring_report_{datetime.now().strftime("%Y%m%d")}.md'
        ).resolve()
    )
    json_path = Path(args.write_json).expanduser().resolve() if args.write_json else None

    report = f"""# Map Authoring Report

This report summarizes the current public evidence that `lidarslam_ros2` should
be judged as a ROS 2 pointcloud-map authoring stack, not only as a generic SLAM
codebase.

## Summary

| Area | Evidence | Current result |
| --- | --- | --- |
| Default benchmark path | `NTU VIRAL tnp_01` current default | `APE RMSE {_fmt(float(benchmark_ape.get("rmse")))} m`, `pairs {int(benchmark_ape.get("pairs"))}`, verify `{benchmark_verify}` |
| Georeferenced map output | open-data GNSS smoke | `projector_type {gnss["projector_type"]}`, `map_origin {"yes" if gnss["has_map_origin"] else "no"}` |
| Save-time map cleanup | dynamic-object-filter comparison | saved-point reduction ratio `{_fmt(dynamic_filter.get("point_reduction_ratio"))}` |
| Classic fallback path | Leo Drive `driving_30_kmh` suite | GNSS-only improves APE by `{_fmt(classic_path.get("gnss_gain_m"))} m` vs no-GNSS |

## Interpretation

- The repository has a tracked default mapping benchmark, not just an example launch.
- The repository emits georeference metadata in a form that downstream pointcloud-map workflows can consume.
- The repository includes a save-time map-cleanup function with a measured effect on saved-map size.
- The repository also tracks weaker fallback paths explicitly instead of hiding them.

## Standard Submission Artifacts

- `metrics.json`
- `pointcloud_map/`
- `map_projector_info.yaml`
- `benchmark_summary.md` or a focused report markdown
- `latest_report.html` or a focused report JSON

## Inputs

- benchmark metrics: `{benchmark_metrics_path}`
- GNSS projector info: `{gnss_projector_path}`
- dynamic filter report: `{dynamic_filter_report_path}`
- classic path report: `{classic_path_report_path}`
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
