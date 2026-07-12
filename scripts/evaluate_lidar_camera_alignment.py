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

"""Measure LiDAR depth-edge alignment against camera image edges."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import pointcloud_io as pcio  # noqa: E402
import train_gsplat as tg  # noqa: E402


def image_edges(image: np.ndarray, percentile: float = 95.0) -> np.ndarray:
    """Return a strong grayscale-gradient edge mask without OpenCV/SciPy."""
    array = np.asarray(image, dtype=np.float32)
    if array.ndim == 3:
        array = array[:, :, :3] @ np.array([0.299, 0.587, 0.114])
    if array.ndim != 2:
        raise ValueError(f'image must be HxW or HxWxC, got {array.shape}')
    if not 0.0 <= percentile <= 100.0:
        raise ValueError('percentile must be between 0 and 100')
    gx = np.zeros_like(array)
    gy = np.zeros_like(array)
    gx[:, 1:-1] = np.abs(array[:, 2:] - array[:, :-2])
    gy[1:-1, :] = np.abs(array[2:, :] - array[:-2, :])
    magnitude = np.hypot(gx, gy)
    nonzero = magnitude[magnitude > 0.0]
    if nonzero.size == 0:
        return np.zeros(array.shape, dtype=bool)
    return magnitude >= np.percentile(nonzero, percentile)


def depth_edges(depth: np.ndarray, absolute: float = 0.25,
                relative: float = 0.02) -> np.ndarray:
    """Return pixels adjacent to a supported LiDAR depth discontinuity."""
    values = np.asarray(depth, dtype=np.float32)
    if values.ndim != 2:
        raise ValueError(f'depth must be HxW, got {values.shape}')
    if absolute < 0.0 or relative < 0.0:
        raise ValueError('depth thresholds must be non-negative')
    valid = np.isfinite(values) & (values > 0.0)
    edges = np.zeros(values.shape, dtype=bool)
    with np.errstate(invalid='ignore'):
        horizontal = (valid[:, :-1] & valid[:, 1:] &
                      (np.abs(values[:, :-1] - values[:, 1:]) >
                       absolute + relative * np.minimum(
                           values[:, :-1], values[:, 1:])))
        vertical = (valid[:-1, :] & valid[1:, :] &
                    (np.abs(values[:-1, :] - values[1:, :]) >
                     absolute + relative * np.minimum(
                         values[:-1, :], values[1:, :])))
    edges[:, :-1] |= horizontal
    edges[:, 1:] |= horizontal
    edges[:-1, :] |= vertical
    edges[1:, :] |= vertical
    return edges


def nearest_edge_distances(query: np.ndarray, target: np.ndarray,
                           max_distance: int = 12) -> np.ndarray:
    """Find target-edge distance for query pixels with a bounded local search."""
    query_y, query_x = np.nonzero(query)
    if query_y.size == 0:
        return np.zeros(0, dtype=np.float32)
    distances = np.full(query_y.size, float(max_distance + 1), dtype=np.float32)
    height, width = target.shape
    offsets = [(dy, dx) for dy in range(-max_distance, max_distance + 1)
               for dx in range(-max_distance, max_distance + 1)
               if dy * dy + dx * dx <= max_distance * max_distance]
    offsets.sort(key=lambda item: item[0] * item[0] + item[1] * item[1])
    for dy, dx in offsets:
        distance = float(np.hypot(dy, dx))
        pending = distances > distance
        y = query_y + dy
        x = query_x + dx
        inside = pending & (y >= 0) & (y < height) & (x >= 0) & (x < width)
        hit = np.zeros(query_y.size, dtype=bool)
        hit[inside] = target[y[inside], x[inside]]
        distances[hit] = distance
    return distances


def projected_depth(points: np.ndarray, viewmat: np.ndarray, K: np.ndarray,
                    width: int, height: int) -> np.ndarray:
    """Project one view into an image containing sparse nearest depths."""
    (pixels, depths), = pcio.project_depth_maps(
        points, np.asarray(viewmat)[None], K, width, height)
    image = np.full((height, width), np.inf, dtype=np.float32)
    image.reshape(-1)[pixels] = depths
    return image


def score_view(points: np.ndarray, viewmat: np.ndarray, K: np.ndarray,
               image: np.ndarray, *, edge_percentile: float = 95.0,
               max_distance: int = 12) -> dict:
    """Score one camera view and return pixel-distance alignment metrics."""
    height, width = image.shape[:2]
    lidar_edges = depth_edges(projected_depth(points, viewmat, K, width, height))
    distances = nearest_edge_distances(
        lidar_edges, image_edges(image, edge_percentile), max_distance)
    if distances.size == 0:
        return {'edge_points': 0, 'median_px': None, 'p90_px': None,
                'inlier_2px': None}
    return {'edge_points': int(distances.size),
            'median_px': float(np.median(distances)),
            'p90_px': float(np.percentile(distances, 90)),
            'inlier_2px': float(np.mean(distances <= 2.0))}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pointcloud', type=Path, required=True)
    parser.add_argument('--transforms', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--view-stride', type=int, default=10)
    parser.add_argument('--edge-percentile', type=float, default=95.0,
                        help='retain this percentile of nonzero image gradients; '
                             '95 focuses the metric on structural edges')
    parser.add_argument('--max-distance', type=int, default=12)
    args = parser.parse_args()
    if args.view_stride < 1 or args.max_distance < 1:
        raise SystemExit('--view-stride and --max-distance must be >= 1')
    import imageio.v3 as iio
    points, _ = pcio.read_ply_xyz(args.pointcloud)
    dataset = tg.load_transforms(args.transforms)
    per_view = []
    for index in range(0, len(dataset['viewmats']), args.view_stride):
        score = score_view(
            points, dataset['viewmats'][index], dataset['K'],
            iio.imread(dataset['image_paths'][index]),
            edge_percentile=args.edge_percentile,
            max_distance=args.max_distance)
        score['view_index'] = index
        per_view.append(score)
    valid = [item for item in per_view if item['edge_points'] > 0]
    if not valid:
        raise SystemExit('no LiDAR depth edges were measurable')
    weights = np.asarray([item['edge_points'] for item in valid], dtype=float)
    report = {'pointcloud': str(args.pointcloud.resolve()),
              'transforms': str(args.transforms.resolve()),
              'views_scored': len(valid), 'edge_points': int(weights.sum())}
    for name in ('median_px', 'p90_px', 'inlier_2px'):
        report[f'weighted_{name}'] = float(np.average(
            [item[name] for item in valid], weights=weights))
    report['per_view'] = per_view
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items()
                      if key != 'per_view'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
