#!/usr/bin/env python3
"""KITTI Odometry translational/rotational drift metrics.

Reads two TUM trajectories (ground truth + estimate) over the same set of
frames and reports the standard KITTI Odometry metrics:

  t_rel : average translational drift (%)
  r_rel : average rotational drift   (deg/m)
  per-length breakdowns over the standard {100, 200, ..., 800} m windows.

The algorithm follows the KITTI devkit:
  1. compute the cumulative arc-length along the GT trajectory
  2. for every pose i and every requested window length L, find the smallest
     j > i with `dist[j] - dist[i] >= L`
  3. compute the relative-pose error
       err = inv(gt_rel) @ est_rel
       gt_rel  = inv(T_gt[i])  @ T_gt[j]
       est_rel = inv(T_est[i]) @ T_est[j]
     and accumulate `||err.translation|| / L` and `theta(err.rotation) / L`
  4. average over all valid (i, L) pairs.

The two TUM files must list poses for the same frames in the same order. They
do not have to be in the same reference frame; the metric is invariant to a
global SE(3) transform.

Usage:
  python3 scripts/kitti_metrics.py --gt path/to/gt.tum --est path/to/est.tum \\
      [--lengths 100,200,300,400,500,600,700,800] [--out-json path]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


# Standard KITTI Odometry sub-trajectory window lengths (metres).
DEFAULT_LENGTHS_M = (100, 200, 300, 400, 500, 600, 700, 800)


def read_tum(path: Path) -> np.ndarray:
    """Read a TUM trajectory. Returns Nx8 array (t x y z qx qy qz qw)."""
    rows: list[list[float]] = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        rows.append([float(v) for v in parts[:8]])
    if not rows:
        raise ValueError(f'No valid TUM rows in {path}')
    return np.array(rows)


def _quat_to_R(q: np.ndarray) -> np.ndarray:
    """Quaternion (qx, qy, qz, qw) -> 3x3 rotation matrix."""
    qx, qy, qz, qw = q
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )


def tum_to_poses(tum: np.ndarray) -> np.ndarray:
    """Convert Nx8 TUM array (t x y z qx qy qz qw) into N 4x4 SE(3) matrices."""
    n = tum.shape[0]
    poses = np.tile(np.eye(4), (n, 1, 1))
    poses[:, :3, 3] = tum[:, 1:4]
    for i in range(n):
        poses[i, :3, :3] = _quat_to_R(tum[i, 4:8])
    return poses


def _cumulative_lengths(positions: np.ndarray) -> np.ndarray:
    """Cumulative arc-length at each pose (positions: Nx3)."""
    if len(positions) < 2:
        return np.zeros(len(positions))
    deltas = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(deltas)))


def _rotation_angle(R: np.ndarray) -> float:
    """Geodesic rotation angle (radians) of a 3x3 matrix."""
    trace = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    return float(math.acos(trace))


def compute_kitti_errors(
    gt_poses: np.ndarray,
    est_poses: np.ndarray,
    lengths_m: tuple[int, ...] = DEFAULT_LENGTHS_M,
    step: int = 10,
) -> dict[str, object]:
    """Compute KITTI per-length and aggregate drift errors.

    Args:
      gt_poses:  Nx4x4 SE(3) ground-truth poses.
      est_poses: Nx4x4 SE(3) estimated poses (same ordering).
      lengths_m: window lengths to evaluate (metres).
      step:      stride over starting indices i (KITTI devkit uses 10).

    Returns a dict with per-length statistics and aggregate t_rel / r_rel.
    """
    if gt_poses.shape != est_poses.shape:
        raise ValueError(
            f'gt/est pose shape mismatch: {gt_poses.shape} vs {est_poses.shape}'
        )
    if gt_poses.ndim != 3 or gt_poses.shape[1:] != (4, 4):
        raise ValueError(f'expected Nx4x4 SE(3) array, got shape {gt_poses.shape}')

    n = gt_poses.shape[0]
    if n < 2:
        return {
            'lengths_m': list(lengths_m),
            'per_length': {},
            't_rel_percent_avg': None,
            'r_rel_deg_per_m_avg': None,
            'pairs_total': 0,
        }

    dist = _cumulative_lengths(gt_poses[:, :3, 3])
    per_length: dict[int, dict[str, float | int]] = {}
    all_t_errs: list[float] = []
    all_r_errs: list[float] = []

    for L in lengths_m:
        t_errs: list[float] = []
        r_errs: list[float] = []
        for i in range(0, n, step):
            target = dist[i] + L
            j = int(np.searchsorted(dist, target, side='left'))
            if j >= n:
                continue
            gt_rel = np.linalg.inv(gt_poses[i]) @ gt_poses[j]
            est_rel = np.linalg.inv(est_poses[i]) @ est_poses[j]
            err = np.linalg.inv(gt_rel) @ est_rel
            t_err = float(np.linalg.norm(err[:3, 3]))
            r_err = _rotation_angle(err[:3, :3])
            t_errs.append(t_err / L)
            r_errs.append(r_err / L)
        if t_errs:
            per_length[int(L)] = {
                'pairs': len(t_errs),
                't_rel_percent': float(np.mean(t_errs)) * 100.0,
                'r_rel_deg_per_m': math.degrees(float(np.mean(r_errs))),
            }
            all_t_errs.extend(t_errs)
            all_r_errs.extend(r_errs)
        else:
            per_length[int(L)] = {'pairs': 0, 't_rel_percent': None, 'r_rel_deg_per_m': None}

    if all_t_errs:
        t_rel_avg = float(np.mean(all_t_errs)) * 100.0
        r_rel_avg = math.degrees(float(np.mean(all_r_errs)))
    else:
        t_rel_avg = None
        r_rel_avg = None

    return {
        'lengths_m': list(lengths_m),
        'per_length': per_length,
        't_rel_percent_avg': t_rel_avg,
        'r_rel_deg_per_m_avg': r_rel_avg,
        'pairs_total': len(all_t_errs),
    }


def _parse_lengths(spec: str) -> tuple[int, ...]:
    parts = [p.strip() for p in spec.split(',') if p.strip()]
    if not parts:
        raise argparse.ArgumentTypeError('--lengths must list at least one value')
    return tuple(int(p) for p in parts)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description='Compute KITTI Odometry t_rel / r_rel drift metrics from TUM files.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument('--gt', required=True, type=Path, help='Ground-truth TUM trajectory')
    ap.add_argument('--est', required=True, type=Path, help='Estimated TUM trajectory')
    ap.add_argument(
        '--lengths',
        default=','.join(str(L) for L in DEFAULT_LENGTHS_M),
        type=_parse_lengths,
        help='Comma-separated window lengths in metres',
    )
    ap.add_argument('--step', type=int, default=10, help='Stride over starting indices')
    ap.add_argument('--out-json', type=Path, default=None, help='Write metrics JSON here')
    ap.add_argument('--label', default='', help='Optional label embedded in the JSON')
    args = ap.parse_args(argv)

    gt = read_tum(args.gt)
    est = read_tum(args.est)

    if gt.shape[0] != est.shape[0]:
        n = min(gt.shape[0], est.shape[0])
        print(
            f'warning: trimming to common length {n} '
            f'(gt={gt.shape[0]}, est={est.shape[0]})',
            file=sys.stderr,
        )
        gt = gt[:n]
        est = est[:n]

    gt_poses = tum_to_poses(gt)
    est_poses = tum_to_poses(est)
    metrics = compute_kitti_errors(gt_poses, est_poses, args.lengths, args.step)

    output = {
        'label': args.label,
        'gt_path': str(args.gt),
        'est_path': str(args.est),
        'frames': int(gt.shape[0]),
        **metrics,
    }

    print(f'frames           : {output["frames"]}')
    print(f't_rel (% / 100m) : {output["t_rel_percent_avg"]}')
    print(f'r_rel (deg / m)  : {output["r_rel_deg_per_m_avg"]}')
    print(f'pairs total      : {output["pairs_total"]}')
    for L, stats in output['per_length'].items():
        print(
            f'  L={L:>4}m  pairs={stats["pairs"]:>5}  '
            f't_rel%={stats["t_rel_percent"]}  r_rel_deg/m={stats["r_rel_deg_per_m"]}'
        )

    if args.out_json:
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        args.out_json.write_text(json.dumps(output, indent=2), encoding='utf-8')
        print(f'wrote {args.out_json}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
