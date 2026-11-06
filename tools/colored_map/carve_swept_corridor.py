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

"""Remove map points inside the corridor the camera bearer walked through.

Dynamic-object removal votes with LiDAR rays that pass through a voxel. A
person who stood on the route and left before the camera arrived is never
looked back at by a forward-facing sensor, so they stay in the map as a
ghost. The camera bearer later walked through exactly that space, which
nothing static can occupy, so points within ``--radius`` of the camera path
and between ``--below`` and ``--above`` around camera height are removed.
"Up" is the mean camera up vector of the posed images.

With ``--fill-ground-holes`` the ground the LiDAR never measured near the path
(behind a person who walked ahead, under the rig where it stood, on wet dark
asphalt) is filled with points on the plane of the ground around it, so the
colouring that follows can paint it from the images. Only small holes that
are enclosed by ground on all sides and set in a flat surround are filled;
steps, kerbs and planter walls fail the flatness test.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Optional, Sequence

import numpy as np

_HERE = Path(__file__).resolve().parent
_GS_DIR = _HERE.parent / 'gaussian_splatting'
for _path in (_HERE, _GS_DIR):
    if str(_path) not in sys.path:
        sys.path.append(str(_path))

import pointcloud_io as pcio  # noqa: E402, I100


def camera_path(transforms: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Return camera centres in frame order and the mean camera up vector."""
    import train_gsplat as tg

    dataset = tg.load_transforms(transforms)
    viewmats = np.asarray(dataset['viewmats'], dtype=np.float64)
    centres = np.einsum('nji,nj->ni', viewmats[:, :3, :3], -viewmats[:, :3, 3])
    return centres, pcio.estimate_world_up(viewmats)


def height_histogram(points, path, up, *, radius: float, bins=None) -> dict:
    """Count points by height around the path within ``radius`` (for tuning)."""
    bins = np.arange(-3.0, 1.01, 0.1) if bins is None else bins
    near = pcio.swept_corridor_mask(points, path, up, radius=radius,
                                    below=-bins[0], above=bins[-1])
    from scipy.spatial import cKDTree

    xyz = np.asarray(points, dtype=np.float64)[near]
    if len(xyz) == 0:
        return {'edges': bins.tolist(), 'counts': [0] * (len(bins) - 1)}
    _, nearest = cKDTree(path).query(xyz)
    height = (xyz - path[nearest]) @ (up / np.linalg.norm(up))
    counts, _ = np.histogram(height, bins=bins)
    return {'edges': [round(float(x), 2) for x in bins],
            'counts': counts.tolist()}


def _disk(radius_cells: int) -> np.ndarray:
    grid = np.mgrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
    return np.hypot(grid[0], grid[1]) <= radius_cells


def fill_ground_holes(points, path, up, *, cell: float = 0.1, reach: float = 2.0,
                      close_radius: float = 0.5, max_area: float = 4.0,
                      ring: float = 0.5, max_rms: float = 0.04,
                      inlier_tol: float = 0.05, min_inliers: float = 0.7,
                      min_sectors: int = 10, band: tuple = (-2.5, 0.5),
                      seed: int = 0) -> tuple[np.ndarray, dict]:
    """Return points that fill small enclosed ground holes near the path, and stats.

    A cell (``cell`` m, horizontal) holds ground when any point lies between
    ``band`` metres of the nearest camera centre (along ``up``); its height is
    the median of the points within 10 cm of its lowest point. Empty cells
    within ``reach`` of the path that a closing of radius ``close_radius``
    fills are holes. A hole is filled when it is at most ``max_area`` m2, the
    ground cells within ``ring`` of it surround it (``min_sectors`` of 12
    directions) and fit a plane: at least ``min_inliers`` of them within
    ``inlier_tol``, those with RMS at most ``max_rms``. Each hole cell gets as
    many points as a ring cell holds near its ground (4 to 64), on that plane.
    """
    from scipy import ndimage
    from scipy.spatial import cKDTree

    xyz = np.asarray(points, dtype=np.float64)
    path = np.asarray(path, dtype=np.float64)
    up = np.asarray(up, dtype=np.float64)
    up = up / np.linalg.norm(up)
    e1 = np.cross(up, [1.0, 0.0, 0.0])
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(up, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    basis = np.stack([e1, e2], axis=1)
    stats = {'holes': 0, 'filled': 0, 'too_large': 0, 'not_enclosed': 0,
             'not_flat': 0, 'filled_area_m2': 0.0, 'points_added': 0}
    empty = np.zeros((0, 3))
    if len(xyz) == 0 or len(path) == 0:
        return empty, stats
    tree = cKDTree(path @ basis)
    uv = xyz @ basis
    dist, nearest = tree.query(uv, distance_upper_bound=reach + ring + cell)
    near = np.isfinite(dist)
    rel = np.full(len(xyz), np.nan)
    rel[near] = (xyz[near] - path[nearest[near]]) @ up
    sel = near & (rel > band[0]) & (rel < band[1])
    if not sel.any():
        return empty, stats
    g_uv, g_z = uv[sel], xyz[sel] @ up
    margin = ring + close_radius + cell
    lo = g_uv.min(axis=0) - margin
    ij = np.floor((g_uv - lo) / cell).astype(np.int64)
    shape = tuple(int(v) for v in ij.max(axis=0) + int(np.ceil(margin / cell)) + 2)
    flat = ij[:, 0] * shape[1] + ij[:, 1]
    lowest = np.full(shape[0] * shape[1], np.inf)
    np.minimum.at(lowest, flat, g_z)
    occupied = np.isfinite(lowest).reshape(shape)
    low = g_z <= lowest[flat] + 0.10
    ground_count = np.bincount(flat[low], minlength=occupied.size).reshape(shape)
    order = np.lexsort((g_z[low], flat[low]))
    f_low, z_low = flat[low][order], g_z[low][order]
    starts = np.r_[0, np.nonzero(np.diff(f_low))[0] + 1]
    counts = np.diff(np.r_[starts, len(f_low)])
    ground_z = np.full(occupied.size, np.nan)
    ground_z[f_low[starts]] = z_low[starts + counts // 2]
    ground_z = ground_z.reshape(shape)
    centre_u = (np.arange(shape[0]) + 0.5) * cell + lo[0]
    centre_v = (np.arange(shape[1]) + 0.5) * cell + lo[1]
    grid_u, grid_v = np.meshgrid(centre_u, centre_v, indexing='ij')
    path_dist, _ = tree.query(np.stack([grid_u.ravel(), grid_v.ravel()], axis=1))
    corridor = path_dist.reshape(shape) <= reach
    close_cells = max(1, int(round(close_radius / cell)))
    holes = (ndimage.binary_closing(occupied, structure=_disk(close_cells))
             & ~occupied & corridor)
    labels, count = ndimage.label(holes)
    stats['holes'] = int(count)
    ring_cells = max(1, int(round(ring / cell)))
    rng = np.random.default_rng(seed)
    added = []
    for label, box in enumerate(ndimage.find_objects(labels), start=1):
        pad = ring_cells + 1
        rows = slice(max(box[0].start - pad, 0), min(box[0].stop + pad, shape[0]))
        cols = slice(max(box[1].start - pad, 0), min(box[1].stop + pad, shape[1]))
        hole = labels[rows, cols] == label
        area = float(hole.sum()) * cell * cell
        if area > max_area:
            stats['too_large'] += 1
            continue
        around = ndimage.binary_dilation(hole, structure=_disk(ring_cells)) & occupied[rows, cols]
        ri, rj = np.nonzero(around)
        hi, hj = np.nonzero(hole)
        cu, cv = grid_u[rows, cols], grid_v[rows, cols]
        u0, v0 = cu[hi, hj].mean(), cv[hi, hj].mean()
        ring_u, ring_v = cu[ri, rj], cv[ri, rj]
        angle = np.arctan2(ring_v - v0, ring_u - u0)
        sectors = np.unique(np.floor((angle + np.pi) / (np.pi / 6.0)).astype(int) % 12).size
        if len(ri) < 6 or sectors < min_sectors:
            stats['not_enclosed'] += 1
            continue
        ring_z = ground_z[rows, cols][ri, rj]
        design = np.stack([ring_u - u0, ring_v - v0, np.ones_like(ring_u)], axis=1)
        inliers = np.ones(len(ring_z), dtype=bool)
        for _ in range(3):  # drop ring cells off the plane (a post, a kerb), refit
            coef, *_ = np.linalg.lstsq(design[inliers], ring_z[inliers], rcond=None)
            residual = np.abs(design @ coef - ring_z)
            inliers = residual <= max(3.0 * float(np.median(residual[inliers])), inlier_tol)
        rms = float(np.sqrt(np.mean(residual[inliers] ** 2)))
        if inliers.mean() < min_inliers or rms > max_rms:
            stats['not_flat'] += 1
            continue
        per_cell = int(np.clip(np.median(ground_count[rows, cols][ri, rj]), 4, 64))
        fill_u = cu[hi, hj][:, None] + (rng.random((len(hi), per_cell)) - 0.5) * cell
        fill_v = cv[hi, hj][:, None] + (rng.random((len(hi), per_cell)) - 0.5) * cell
        fill_z = coef[0] * (fill_u - u0) + coef[1] * (fill_v - v0) + coef[2]
        added.append(fill_u.reshape(-1, 1) * e1 + fill_v.reshape(-1, 1) * e2
                     + fill_z.reshape(-1, 1) * up)
        stats['filled'] += 1
        stats['filled_area_m2'] += area
    filled = np.concatenate(added) if added else empty
    stats['points_added'] = int(len(filled))
    stats['filled_area_m2'] = round(stats['filled_area_m2'], 3)
    return filled, stats


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--input', required=True, help='map PLY (xyz[+rgb])')
    parser.add_argument('--transforms', required=True,
                        help='posed-image transforms.json along the same path')
    parser.add_argument('--out', required=True)
    parser.add_argument('--radius', type=float, default=0.5,
                        help='horizontal corridor radius around the camera (m)')
    parser.add_argument('--below', type=float, default=0.8,
                        help='corridor depth below camera height (m); keep it '
                             'under the camera height above the ground')
    parser.add_argument('--above', type=float, default=0.4,
                        help='corridor height above the camera (m)')
    parser.add_argument('--report', default=None,
                        help='JSON with the removed count and a height '
                             'histogram near the path')
    parser.add_argument('--fill-ground-holes', action='store_true',
                        help='after carving, fill small enclosed ground holes '
                             'near the path with points on the surrounding '
                             'ground plane (colour them with the next step)')
    parser.add_argument('--fill-reach', type=float, default=2.0,
                        help='fill holes within this distance of the path (m)')
    parser.add_argument('--fill-max-area', type=float, default=4.0,
                        help='largest hole to fill (m2)')
    parser.add_argument('--fill-max-rms', type=float, default=0.04,
                        help='flatness of the ground around a hole (m RMS)')
    args = parser.parse_args(argv)
    xyz, rgb = pcio.read_ply_xyz(args.input)
    path, up = camera_path(args.transforms)
    carve = pcio.swept_corridor_mask(xyz, path, up, radius=args.radius,
                                     below=args.below, above=args.above)
    kept, kept_rgb = xyz[~carve], None if rgb is None else rgb[~carve]
    fill_stats = None
    if args.fill_ground_holes:
        added, fill_stats = fill_ground_holes(
            kept, path, up, reach=args.fill_reach, max_area=args.fill_max_area,
            max_rms=args.fill_max_rms)
        kept = np.concatenate([kept, added.astype(kept.dtype)])
        if kept_rgb is not None:
            grey = np.full((len(added), kept_rgb.shape[1]), 128, dtype=kept_rgb.dtype)
            kept_rgb = np.concatenate([kept_rgb, grey])
    out = pcio.write_ply(args.out, kept, kept_rgb)
    report = {'input_points': int(len(xyz)), 'removed': int(carve.sum()),
              'radius': args.radius, 'below': args.below, 'above': args.above,
              'height_histogram': height_histogram(xyz, path, up,
                                                   radius=args.radius),
              'out': str(out)}
    if fill_stats is not None:
        report['ground_hole_fill'] = fill_stats
    if args.report:
        Path(args.report).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items()
                      if k != 'height_histogram'}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
