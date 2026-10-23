#!/usr/bin/env python3
"""Is a trajectory's apparent scale error a clock-rate difference instead? (chapter 29)

Re-samples the estimate at warped GT stamps t' = t0 + (1 + k) (t - t0) + dt for a grid of rate errors k (ppm) and
offsets dt, aligns with SE(3) Umeyama, and prints the best (k, dt) and its RMSE next to the plain SE(3) and Sim(3)
RMSE. A clock-rate difference turns into a scale error only for motion along one direction; if it explains the
error, the best k reaches the Sim(3) RMSE.
Usage: time_warp_check.py <gt_tum> <est_tum>
"""
import sys

import numpy as np

gt = np.loadtxt(sys.argv[1])
est = np.loadtxt(sys.argv[2])
t0 = gt[0, 0]


def rmse(e, p, scale):
    me, mp = e.mean(0), p.mean(0)
    E, P = e - me, p - mp
    U, D, Vt = np.linalg.svd(P.T @ E / len(e))
    S = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / (E ** 2).sum(1).mean() if scale else 1.0
    return np.sqrt(((s * (R @ E.T).T + mp - p) ** 2).sum(1).mean())


g = gt[(gt[:, 0] >= est[0, 0] + 1) & (gt[:, 0] <= est[-1, 0] - 1)][::10]
p = g[:, 1:4]


def sample(ts):
    return np.column_stack([np.interp(ts, est[:, 0], est[:, k]) for k in (1, 2, 3)])


e = sample(g[:, 0])
print(f'se3 {rmse(e, p, False):.4f}  sim3 {rmse(e, p, True):.4f}')
best = None
for k in np.arange(-3000, 3001, 100) * 1e-6:
    for dt in np.arange(-0.10, 0.101, 0.01):
        r = rmse(sample(t0 + (1 + k) * (g[:, 0] - t0) + dt), p, False)
        if best is None or r < best[0]:
            best = (r, k, dt)
print(f'best time warp: rate {best[1] * 1e6:+.0f} ppm, offset {best[2] * 1e3:+.0f} ms -> se3 {best[0]:.4f}')
