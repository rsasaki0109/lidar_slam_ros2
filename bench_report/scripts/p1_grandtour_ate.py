#!/usr/bin/env python3
"""
ATE of a GrandTour trajectory against the Leica total-station prism reference (chapter 31), for validation missions.

The estimate (TUM, T_world_prism) is interpolated linearly at the reference timestamps inside its time span, aligned
to the reference positions with a rigid SE(3) Umeyama fit (no scale), and the RMSE of the position residual is
reported, the benchmark's ATE. Only validation-split references may be read with this; the test references are hidden.
Usage: p1_grandtour_ate.py <estimate.tum> <prism_position zarr dir> [--offset dt_seconds]
"""
import argparse

import numpy as np

import zarr


def align_se3(est, ref):
    mu_e, mu_r = est.mean(0), ref.mean(0)
    U, _, Vt = np.linalg.svd((est - mu_e).T @ (ref - mu_r))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, mu_r - R @ mu_e


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('estimate')
    parser.add_argument('reference')
    parser.add_argument('--offset', type=float, default=0.0, help='added to the reference timestamps')
    args = parser.parse_args()
    est = np.loadtxt(args.estimate)
    ref = zarr.open(args.reference, mode='r')
    rt = ref['timestamp'][:].astype(float) + args.offset
    rp = ref['point'][:]
    inside = (rt >= est[0, 0]) & (rt <= est[-1, 0])
    rt, rp = rt[inside], rp[inside]
    ep = np.stack([np.interp(rt, est[:, 0], est[:, k]) for k in (1, 2, 3)], 1)
    R, t = align_se3(ep, rp)
    err = np.linalg.norm(ep @ R.T + t - rp, axis=1)
    length = np.linalg.norm(np.diff(rp, axis=0), axis=1).sum()
    print(f'reference points {inside.sum()} of {len(inside)}, path {length:.1f} m, '
          f'ATE rmse {np.sqrt((err ** 2).mean()):.4f} m, median {np.median(err):.4f}, max {err.max():.4f}')


if __name__ == '__main__':
    main()
