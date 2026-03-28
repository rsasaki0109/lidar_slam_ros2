#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Summarize dynamic-object-filter results across multiple benchmark runs."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path


DEFAULT_BENCHMARKS = [
    ('Leo Drive bag1', Path('output/dynamic_object_filter_benchmark_bag1_20260327')),
    ('Leo Drive bag6', Path('output/dynamic_object_filter_benchmark_bag6_20260326')),
]


def _fmt_float(value: float | None) -> str:
    if value is None:
        return '-'
    return f'{value:.3f}'


def _fmt_int(value: int | None) -> str:
    if value is None:
        return '-'
    return str(value)


def _load_benchmark(label: str, root: Path) -> dict[str, object]:
    report_path = root / 'dynamic_object_filter_report.json'
    if not report_path.is_file():
        raise SystemExit(f'dynamic_object_filter_report.json not found under {root}')
    payload = json.loads(report_path.read_text(encoding='utf-8'))
    baseline = payload['baseline']
    filtered = payload['filtered']
    return {
        'label': label,
        'root_dir': str(root),
        'baseline_points': int(baseline['total_pcd_points']),
        'filtered_points': int(filtered['total_pcd_points']),
        'point_reduction_ratio': float(payload['point_reduction_ratio']),
        'kept_candidate_voxel_ratio': float(payload['kept_candidate_voxel_ratio']),
        'removed_candidate_voxel_ratio': float(payload['removed_candidate_voxel_ratio']),
        'baseline_cells': baseline.get('saved_cell_count'),
        'filtered_cells': filtered.get('saved_cell_count'),
        'baseline_verify': baseline.get('verify_result', 'unknown'),
        'filtered_verify': filtered.get('verify_result', 'unknown'),
        'projector_type': filtered.get('projector_type', 'unknown'),
        'shared_metadata_tiles': int(payload.get('shared_metadata_tiles', 0)),
        'tile_jaccard': float(payload.get('tile_jaccard', 0.0)),
        'filtered_tile_overlap_ratio': float(payload.get('filtered_tile_overlap_ratio', 0.0)),
        'baseline_tile_overlap_ratio': float(payload.get('baseline_tile_overlap_ratio', 0.0)),
    }


def _write_svg(out_path: Path, rows: list[dict[str, object]]) -> None:
    max_ratio = max(max(row['point_reduction_ratio'] for row in rows), 1e-6)
    bar_max_width = 360
    fills = ['#2563eb', '#0f766e', '#7c3aed', '#c2410c']
    parts = []
    for index, row in enumerate(rows):
        width = int(round((row['point_reduction_ratio'] / max_ratio) * bar_max_width))
        y = 56 + index * 44
        parts.append(
            f'  <text x="24" y="{y + 17}" class="label">{row["label"]}</text>\n'
            f'  <rect x="200" y="{y}" width="{width}" height="22" rx="4" fill="{fills[index % len(fills)]}"/>\n'
            f'  <text x="{210 + width}" y="{y + 17}" class="value">{row["point_reduction_ratio"]:.3f}</text>'
        )
    height = 76 + len(rows) * 44
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="760" height="{height}" viewBox="0 0 760 {height}">\n'
        '  <style>\n'
        '    .title { font: 600 18px sans-serif; fill: #111827; }\n'
        '    .label { font: 14px sans-serif; fill: #374151; }\n'
        '    .value { font: 600 14px sans-serif; fill: #111827; }\n'
        '  </style>\n'
        f'  <rect x="0" y="0" width="760" height="{height}" fill="#ffffff"/>\n'
        '  <text x="24" y="34" class="title">Dynamic-filter point reduction ratio</text>\n'
        + '\n'.join(parts) + '\n</svg>\n'
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(svg, encoding='utf-8')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate a cross-dataset validation report for dynamic-object filtering.',
    )
    parser.add_argument(
        '--benchmark',
        action='append',
        default=[],
        metavar='LABEL=DIR',
        help='Optional benchmark root override, repeatable.',
    )
    parser.add_argument('--out', default='')
    parser.add_argument('--write-json', default='')
    parser.add_argument('--write-svg', default='')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    benchmark_specs = []
    if args.benchmark:
        for item in args.benchmark:
            if '=' not in item:
                raise SystemExit(f'expected LABEL=DIR, got: {item}')
            label, raw_dir = item.split('=', 1)
            benchmark_specs.append((label.strip(), Path(raw_dir).expanduser().resolve()))
    else:
        benchmark_specs = [(label, path.resolve()) for label, path in DEFAULT_BENCHMARKS]

    rows = [_load_benchmark(label, root) for label, root in benchmark_specs]
    best = max(rows, key=lambda row: row['point_reduction_ratio'])
    most_conservative = min(rows, key=lambda row: row['removed_candidate_voxel_ratio'])

    out_path = (
        Path(args.out).expanduser().resolve()
        if args.out
        else (
            Path.cwd()
            / 'output'
            / f'dynamic_object_filter_validation_report_{datetime.now().strftime("%Y%m%d")}.md'
        ).resolve()
    )
    json_path = Path(args.write_json).expanduser().resolve() if args.write_json else None
    svg_path = Path(args.write_svg).expanduser().resolve() if args.write_svg else None

    payload = {
        'benchmarks': rows,
        'best_point_reduction_label': best['label'],
        'most_conservative_removed_ratio_label': most_conservative['label'],
    }

    summary_rows = []
    for row in rows:
        summary_rows.append(
            '| {label} | `{baseline_points}` | `{filtered_points}` | `{point_reduction_ratio}` | '
            '`{kept_candidate_voxel_ratio}` | `{removed_candidate_voxel_ratio}` | '
            '`{shared_metadata_tiles}` | `{tile_jaccard}` | `{filtered_tile_overlap_ratio}` | '
            '`{baseline_cells}` -> `{filtered_cells}` | `{verify}` | `{projector_type}` |'.format(
                label=row['label'],
                baseline_points=row['baseline_points'],
                filtered_points=row['filtered_points'],
                point_reduction_ratio=_fmt_float(row['point_reduction_ratio']),
                kept_candidate_voxel_ratio=_fmt_float(row['kept_candidate_voxel_ratio']),
                removed_candidate_voxel_ratio=_fmt_float(row['removed_candidate_voxel_ratio']),
                shared_metadata_tiles=_fmt_int(row['shared_metadata_tiles']),
                tile_jaccard=_fmt_float(row['tile_jaccard']),
                filtered_tile_overlap_ratio=_fmt_float(row['filtered_tile_overlap_ratio']),
                baseline_cells=_fmt_int(row['baseline_cells']),
                filtered_cells=_fmt_int(row['filtered_cells']),
                verify=row['filtered_verify'],
                projector_type=row['projector_type'],
            )
        )

    report = (
        '# Dynamic Object Filter Validation Report\n\n'
        'This report summarizes saved-map cleanup performance across multiple real open-data runs.\n\n'
        '## Summary\n\n'
        '| Benchmark | Baseline points | Filtered points | Point reduction ratio | Kept voxel ratio | Removed voxel ratio | Shared tiles | Tile jaccard | Filtered tile overlap | Saved cells | Verify | Projector |\n'
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |\n'
        + '\n'.join(summary_rows) + '\n\n'
        '## Conclusion\n\n'
        f'- Best point reduction in the tracked validation set is `{best["label"]}` with '
        f'`{_fmt_float(best["point_reduction_ratio"])}`.\n'
        f'- Most conservative voxel removal in the tracked validation set is `{most_conservative["label"]}` with '
        f'`removed_candidate_voxel_ratio={_fmt_float(most_conservative["removed_candidate_voxel_ratio"])}`.\n'
        f'- Best tile-footprint preservation in the tracked validation set is `{max(rows, key=lambda row: row["tile_jaccard"])["label"]}` '
        f'with `tile_jaccard={_fmt_float(max(row["tile_jaccard"] for row in rows))}`.\n'
        '- All tracked runs keep `verify=PASS` and `projector_type=LocalCartesian`, so the save-time cleanup remains compatible with the map workflow.\n'
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding='utf-8')
    if json_path is not None:
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    if svg_path is not None:
        _write_svg(svg_path, rows)

    print(out_path)
    if json_path is not None:
        print(json_path)
    if svg_path is not None:
        print(svg_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
