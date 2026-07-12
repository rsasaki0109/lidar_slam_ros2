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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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

"""Generate a fair, machine-readable lidarslam_ros2 vs FAST-LIVO2 scorecard."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


METRICS = (
    ('trajectory.ape_rmse_m', 'APE RMSE', 'm', 'lower'),
    ('trajectory.rpe_translation_rmse_m', 'RPE translation RMSE', 'm', 'lower'),
    ('runtime.realtime_factor', 'Realtime factor', 'x', 'lower'),
    ('runtime.peak_rss_mb', 'Peak RSS', 'MB', 'lower'),
    ('geometry.plane_thickness_mean_m', 'Plane thickness mean', 'm', 'lower'),
    ('geometry.planar_coverage', 'Planar coverage', '', 'higher'),
    ('colour.heldout_rgb_l2_median', 'Held-out RGB median', 'RGB L2', 'lower'),
    ('colour.heldout_rgb_inlier_20', 'Held-out RGB <=20 inlier', '', 'higher'),
)


def nested(data: dict[str, Any], path: str) -> Any:
    """Read a dot-separated path, returning None for absent values."""
    value: Any = data
    for key in path.split('.'):
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def finite_number(value: Any) -> float | None:
    """Convert finite JSON numbers to float without accepting booleans."""
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def compare(ours: dict[str, Any], rival: dict[str, Any], tie_rel: float = 0.01
            ) -> dict[str, Any]:
    """Compare two manifests; values within ``tie_rel`` are a tie."""
    if ours.get('dataset') != rival.get('dataset'):
        raise ValueError('dataset must match for a head-to-head comparison')
    rows = []
    wins = {'ours': 0, 'fast_livo2': 0, 'tie': 0, 'missing': 0}
    for path, label, unit, direction in METRICS:
        left = finite_number(nested(ours, path))
        right = finite_number(nested(rival, path))
        verdict = 'missing'
        delta_percent = None
        if left is not None and right is not None:
            scale = max(abs(left), abs(right), 1e-12)
            if abs(left - right) <= tie_rel * scale:
                verdict = 'tie'
            elif ((left < right) if direction == 'lower' else (left > right)):
                verdict = 'ours'
            else:
                verdict = 'fast_livo2'
            if abs(right) > 1e-12:
                improvement = ((right - left) if direction == 'lower'
                               else (left - right))
                delta_percent = 100.0 * improvement / abs(right)
        wins[verdict] += 1
        rows.append({
            'path': path, 'label': label, 'unit': unit,
            'direction': direction, 'ours': left, 'fast_livo2': right,
            'delta_percent': delta_percent, 'winner': verdict,
        })
    compared = wins['ours'] + wins['fast_livo2'] + wins['tie']
    overall = 'insufficient_data'
    if compared:
        if wins['ours'] > wins['fast_livo2']:
            overall = 'ours'
        elif wins['fast_livo2'] > wins['ours']:
            overall = 'fast_livo2'
        else:
            overall = 'tie'
    return {
        'dataset': ours.get('dataset'),
        'ours': ours.get('system', 'lidarslam_ros2'),
        'rival': rival.get('system', 'FAST-LIVO2'),
        'tie_relative_tolerance': tie_rel,
        'score': wins,
        'overall': overall,
        'metrics': rows,
    }


def markdown(result: dict[str, Any]) -> str:
    """Render a compact review-friendly scorecard."""
    score = result['score']
    lines = [
        f"# {result['ours']} vs {result['rival']}", '',
        f"Dataset: `{result['dataset']}`", '',
        f"Overall: **{result['overall']}** — {score['ours']} wins / "
        f"{score['fast_livo2']} losses / {score['tie']} ties / "
        f"{score['missing']} missing", '',
        '| Metric | Better | lidarslam_ros2 | FAST-LIVO2 | Delta | Winner |',
        '| --- | --- | ---: | ---: | ---: | --- |',
    ]
    for row in result['metrics']:
        unit = f" {row['unit']}" if row['unit'] else ''
        left = '—' if row['ours'] is None else f"{row['ours']:.6g}{unit}"
        right = ('—' if row['fast_livo2'] is None
                 else f"{row['fast_livo2']:.6g}{unit}")
        delta = ('—' if row['delta_percent'] is None
                 else f"{row['delta_percent']:+.2f}%")
        lines.append(
            f"| {row['label']} | {row['direction']} | {left} | {right} | "
            f"{delta} | **{row['winner']}** |")
    lines.extend(['', '> A winner is declared only from metrics present for both '
                  'systems on the exact same dataset and sensor messages.', ''])
    return '\n'.join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ours', type=Path, required=True)
    parser.add_argument('--fast-livo2', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--tie-relative-tolerance', type=float, default=0.01)
    args = parser.parse_args()
    if args.tie_relative_tolerance < 0.0:
        raise SystemExit('--tie-relative-tolerance must be >= 0')
    ours = json.loads(args.ours.read_text())
    rival = json.loads(args.fast_livo2.read_text())
    result = compare(ours, rival, args.tie_relative_tolerance)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + '\n')
    summary = args.out.with_suffix('.md')
    summary.write_text(markdown(result))
    print(summary)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
