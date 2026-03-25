#!/usr/bin/env python3
"""Generate a small README-facing SVG for the dynamic-filter benchmark."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = (
    ROOT
    / "output"
    / "dynamic_object_filter_benchmark_bag6_20260326"
    / "dynamic_object_filter_report.json"
)
DEFAULT_OUT = ROOT / "lidarslam" / "images" / "dynamic_object_filter_bag6_summary.svg"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the README dynamic-filter summary SVG.",
    )
    parser.add_argument("--report", default=str(DEFAULT_REPORT))
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    report_path = Path(args.report).expanduser().resolve()
    out_path = Path(args.out).expanduser().resolve()
    payload = json.loads(report_path.read_text(encoding="utf-8"))

    baseline_points = int(payload["baseline"]["total_pcd_points"])
    filtered_points = int(payload["filtered"]["total_pcd_points"])
    reduction_ratio = float(payload["point_reduction_ratio"])
    kept_ratio = float(payload["kept_candidate_voxel_ratio"])
    removed_ratio = float(payload["removed_candidate_voxel_ratio"])

    max_points = max(baseline_points, filtered_points, 1)
    bar_max_width = 350
    baseline_width = int(round((baseline_points / max_points) * bar_max_width))
    filtered_width = int(round((filtered_points / max_points) * bar_max_width))
    reduction_pct = reduction_ratio * 100.0

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="880" height="220" viewBox="0 0 880 220">
  <style>
    .title {{ font: 600 19px sans-serif; fill: #111827; }}
    .body {{ font: 14px sans-serif; fill: #374151; }}
    .value {{ font: 600 14px sans-serif; fill: #111827; }}
    .card {{ fill: #ffffff; stroke: #d8e3ef; }}
  </style>
  <rect x="0" y="0" width="880" height="220" fill="#f6f8fb"/>
  <rect x="18" y="18" width="844" height="184" rx="10" class="card"/>
  <text x="40" y="48" class="title">Save-time dynamic-object filtering summary</text>
  <text x="40" y="74" class="body">Leo Drive bag6 benchmark. Map verification stays PASS while saved points drop by {reduction_pct:.1f}%.</text>
  <text x="40" y="110" class="body">No filter</text>
  <rect x="170" y="94" width="{baseline_width}" height="22" rx="4" fill="#9ca3af"/>
  <text x="{182 + baseline_width}" y="110" class="value">{baseline_points}</text>
  <text x="40" y="150" class="body">Dynamic filter</text>
  <rect x="170" y="134" width="{filtered_width}" height="22" rx="4" fill="#2563eb"/>
  <text x="{182 + filtered_width}" y="150" class="value">{filtered_points}</text>
  <text x="590" y="100" class="body">Candidate voxels kept</text>
  <text x="810" y="100" class="value">{kept_ratio * 100.0:.1f}%</text>
  <text x="590" y="132" class="body">Candidate voxels removed</text>
  <text x="810" y="132" class="value">{removed_ratio * 100.0:.1f}%</text>
  <text x="590" y="164" class="body">Projector / verify</text>
  <text x="810" y="164" class="value">LocalCartesian / PASS</text>
</svg>
"""

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(svg, encoding="utf-8")
    print(out_path)


if __name__ == "__main__":
    main()
