#!/usr/bin/env python3
"""APE (SE(3) Umeyama, positions interpolated at the ground-truth stamps) of an estimate shifted in time by dt,
for dt in [-0.2, 0.2] s: a check that the estimate and the ground truth share one time base (minimum near 0).
Usage: time_offset_scan.py <est_tum> <gt_tum>
"""
import sys

import numpy as np

est = np.loadtxt(sys.argv[1], comments='#')
gt = np.loadtxt(sys.argv[2], comments='#')


def ape(dt):
    t = est[:, 0] + dt
    g = gt[(gt[:, 0] >= t[0]) & (gt[:, 0] <= t[-1])]
    e = np.column_stack([np.interp(g[:, 0], t, est[:, k]) for k in (1, 2, 3)])
    p = g[:, 1:4]
    mu_e, mu_p = e.mean(0), p.mean(0)
    U, _, Vt = np.linalg.svd((p - mu_p).T @ (e - mu_e))
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    return float(np.sqrt((((R @ (e - mu_e).T).T + mu_p - p) ** 2).sum(1).mean()))


offsets = np.round(np.arange(-0.2, 0.2001, 0.01), 3)
values = [ape(dt) for dt in offsets]
best = int(np.argmin(values))
print(' '.join(f'{dt:+.2f}:{v:.4f}' for dt, v in zip(offsets[::5], values[::5])))
print(f'minimum {values[best]:.4f} m at dt = {offsets[best]:+.2f} s; at 0: {values[len(offsets) // 2]:.4f} m')
