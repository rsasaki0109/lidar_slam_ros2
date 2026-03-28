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

"""Generate a short report comparing saved maps with dynamic filtering on/off."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re


def _write_points_svg(
    out_path: Path,
    baseline_points: int,
    filtered_points: int,
) -> None:
    max_points = max(baseline_points, filtered_points, 1)
    bar_max_width = 420
    baseline_width = int(round((baseline_points / max_points) * bar_max_width))
    filtered_width = int(round((filtered_points / max_points) * bar_max_width))
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="720" height="180" viewBox="0 0 720 180">
  <style>
    .title {{ font: 600 18px sans-serif; fill: #111827; }}
    .label {{ font: 14px sans-serif; fill: #374151; }}
    .value {{ font: 600 14px sans-serif; fill: #111827; }}
  </style>
  <rect x="0" y="0" width="720" height="180" fill="#ffffff"/>
  <text x="24" y="32" class="title">Saved point count comparison</text>
  <text x="24" y="74" class="label">No filter</text>
  <rect x="150" y="54" width="{baseline_width}" height="24" rx="4" fill="#9ca3af"/>
  <text x="{160 + max(baseline_width, 0)}" y="72" class="value">{baseline_points}</text>
  <text x="24" y="124" class="label">Dynamic filter</text>
  <rect x="150" y="104" width="{filtered_width}" height="24" rx="4" fill="#2563eb"/>
  <text x="{160 + max(filtered_width, 0)}" y="122" class="value">{filtered_points}</text>
</svg>
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(svg, encoding='utf-8')


def _parse_filter_stats(log_path: Path) -> dict[str, int] | None:
    if not log_path.is_file():
        return None
    text = log_path.read_text(encoding='utf-8', errors='replace')
    match = re.search(
        (
            r'Dynamic object filter: input_points (?P<input_points>\d+), '
            r'kept (?P<kept>\d+)/(?P<candidate>\d+) candidate voxels, '
            r'removed (?P<removed>\d+), '
            r'always_keep (?P<always_keep>\d+), '
            r'output_points (?P<output_points>\d+)'
        ),
        text,
    )
    if not match:
        return None
    return {
        'input_points': int(match.group('input_points')),
        'kept_candidate_voxels': int(match.group('kept')),
        'candidate_voxels': int(match.group('candidate')),
        'removed_candidate_voxels': int(match.group('removed')),
        'always_keep_voxels': int(match.group('always_keep')),
        'output_points': int(match.group('output_points')),
    }


def _parse_saved_cell_count(log_path: Path) -> int | None:
    if not log_path.is_file():
        return None
    text = log_path.read_text(encoding='utf-8', errors='replace')
    match = re.search(r'Saved grid-divided map: (?P<cells>\d+) cells', text)
    if not match:
        return None
    return int(match.group('cells'))


def _count_total_pcd_points(run_dir: Path) -> int:
    total = 0
    for pcd_path in sorted((run_dir / 'pointcloud_map').glob('*.pcd')):
        for line in pcd_path.read_text(encoding='utf-8', errors='ignore').splitlines():
            if line.startswith('POINTS '):
                total += int(line.split()[1])
                break
    return total


def _count_metadata_tiles(run_dir: Path) -> int:
    metadata_path = run_dir / 'pointcloud_map' / 'pointcloud_map_metadata.yaml'
    if not metadata_path.is_file():
        return 0
    tile_count = 0
    for line in metadata_path.read_text(encoding='utf-8').splitlines():
        if '.pcd:' in line:
            tile_count += 1
    return tile_count


def _load_metadata_tile_keys(run_dir: Path) -> set[str]:
    metadata_path = run_dir / 'pointcloud_map' / 'pointcloud_map_metadata.yaml'
    if not metadata_path.is_file():
        return set()
    keys: set[str] = set()
    for line in metadata_path.read_text(encoding='utf-8').splitlines():
        stripped = line.strip()
        if '.pcd:' not in stripped:
            continue
        keys.add(stripped.split(':', 1)[0])
    return keys


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
    launch_log = run_dir / 'lidarslam.launch.log'
    return {
        'run_dir': str(run_dir),
        'projector_type': _parse_projector_type(run_dir),
        'verify_result': _verify_result(run_dir),
        'total_pcd_points': _count_total_pcd_points(run_dir),
        'metadata_tiles': _count_metadata_tiles(run_dir),
        'metadata_tile_keys': sorted(_load_metadata_tile_keys(run_dir)),
        'saved_cell_count': _parse_saved_cell_count(launch_log),
        'dynamic_filter_stats': _parse_filter_stats(launch_log),
    }


def _fmt_int(value: int | None) -> str:
    if value is None:
        return '-'
    return str(value)


def _fmt_ratio(value: float | None) -> str:
    if value is None:
        return '-'
    return f'{value:.3f}'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate a saved-map comparison report for dynamic-object filtering.',
    )
    parser.add_argument('--baseline-dir', required=True, help='Run directory without filtering.')
    parser.add_argument('--filtered-dir', required=True, help='Run directory with filtering enabled.')
    parser.add_argument('--out', default='', help='Output markdown path.')
    parser.add_argument('--write-json', default='', help='Optional JSON summary path.')
    parser.add_argument('--write-svg', default='', help='Optional SVG summary path.')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    baseline_dir = Path(args.baseline_dir).expanduser().resolve()
    filtered_dir = Path(args.filtered_dir).expanduser().resolve()
    if not baseline_dir.is_dir():
        raise SystemExit(f'baseline dir not found: {baseline_dir}')
    if not filtered_dir.is_dir():
        raise SystemExit(f'filtered dir not found: {filtered_dir}')

    baseline = _load_run(baseline_dir)
    filtered = _load_run(filtered_dir)
    baseline_points = int(baseline['total_pcd_points'])
    filtered_points = int(filtered['total_pcd_points'])
    point_reduction_ratio = None
    if baseline_points > 0:
        point_reduction_ratio = (baseline_points - filtered_points) / baseline_points

    filter_stats = filtered.get('dynamic_filter_stats') or {}
    kept_ratio = None
    removed_ratio = None
    candidate_voxels = filter_stats.get('candidate_voxels')
    kept_candidate_voxels = filter_stats.get('kept_candidate_voxels')
    if isinstance(candidate_voxels, int) and candidate_voxels > 0:
        kept_ratio = float(kept_candidate_voxels) / float(candidate_voxels)
        removed_ratio = float(filter_stats.get('removed_candidate_voxels', 0)) / float(
            candidate_voxels,
        )

    baseline_tiles = set(baseline.get('metadata_tile_keys', []))
    filtered_tiles = set(filtered.get('metadata_tile_keys', []))
    shared_tiles = baseline_tiles & filtered_tiles
    tile_jaccard = None
    filtered_tile_overlap_ratio = None
    baseline_tile_overlap_ratio = None
    if baseline_tiles or filtered_tiles:
        tile_jaccard = len(shared_tiles) / len(baseline_tiles | filtered_tiles)
    if filtered_tiles:
        filtered_tile_overlap_ratio = len(shared_tiles) / len(filtered_tiles)
    if baseline_tiles:
        baseline_tile_overlap_ratio = len(shared_tiles) / len(baseline_tiles)

    payload = {
        'baseline': baseline,
        'filtered': filtered,
        'point_reduction_ratio': point_reduction_ratio,
        'kept_candidate_voxel_ratio': kept_ratio,
        'removed_candidate_voxel_ratio': removed_ratio,
        'shared_metadata_tiles': len(shared_tiles),
        'tile_jaccard': tile_jaccard,
        'filtered_tile_overlap_ratio': filtered_tile_overlap_ratio,
        'baseline_tile_overlap_ratio': baseline_tile_overlap_ratio,
    }

    out_path = (
        Path(args.out).expanduser().resolve()
        if args.out
        else (
            Path.cwd() / 'output' / f'dynamic_object_filter_report_{datetime.now().strftime("%Y%m%d")}.md'
        ).resolve()
    )
    json_path = Path(args.write_json).expanduser().resolve() if args.write_json else None
    svg_path = Path(args.write_svg).expanduser().resolve() if args.write_svg else None

    report = f"""# Dynamic Object Filter Report

This report compares the same save-time map workflow with dynamic-object filtering
disabled and enabled.

## Inputs

- baseline dir: `{baseline_dir}`
- filtered dir: `{filtered_dir}`

## Summary

| Run | Verify | Projector type | Saved cells | Metadata tiles | Total saved points |
| --- | --- | --- | ---: | ---: | ---: |
| baseline | `{baseline["verify_result"]}` | `{baseline["projector_type"]}` | `{_fmt_int(baseline["saved_cell_count"])}` | `{baseline["metadata_tiles"]}` | `{baseline_points}` |
| filtered | `{filtered["verify_result"]}` | `{filtered["projector_type"]}` | `{_fmt_int(filtered["saved_cell_count"])}` | `{filtered["metadata_tiles"]}` | `{filtered_points}` |

## Filter Stats

- point reduction ratio: `{_fmt_ratio(point_reduction_ratio)}`
- kept candidate voxel ratio: `{_fmt_ratio(kept_ratio)}`
- removed candidate voxel ratio: `{_fmt_ratio(removed_ratio)}`
- shared metadata tiles: `{len(shared_tiles)}`
- tile jaccard: `{_fmt_ratio(tile_jaccard)}`
- filtered tile overlap ratio: `{_fmt_ratio(filtered_tile_overlap_ratio)}`
- baseline tile overlap ratio: `{_fmt_ratio(baseline_tile_overlap_ratio)}`
- filter input points: `{_fmt_int(filter_stats.get("input_points"))}`
- candidate voxels: `{_fmt_int(filter_stats.get("candidate_voxels"))}`
- kept candidate voxels: `{_fmt_int(filter_stats.get("kept_candidate_voxels"))}`
- removed candidate voxels: `{_fmt_int(filter_stats.get("removed_candidate_voxels"))}`
- always-keep voxels: `{_fmt_int(filter_stats.get("always_keep_voxels"))}`
- filter output points: `{_fmt_int(filter_stats.get("output_points"))}`

## Conclusion

- The dynamic filter is save-time only. Live odometry and loop closure are unchanged.
- In this comparison, saved point count changed from `{baseline_points}` to `{filtered_points}`.
- That corresponds to a saved-point reduction ratio of `{_fmt_ratio(point_reduction_ratio)}`.
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding='utf-8')
    if json_path is not None:
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    if svg_path is not None:
        _write_points_svg(svg_path, baseline_points, filtered_points)

    print(out_path)
    if json_path is not None:
        print(json_path)
    if svg_path is not None:
        print(svg_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
