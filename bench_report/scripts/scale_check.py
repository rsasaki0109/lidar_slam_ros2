#!/usr/bin/env python3
"""How much of a MapsHD trajectory's error is a scale error (chapters 21 and 29).

Aligns the estimate to the ground truth (positions interpolated at the GT stamps) with SE(3) Umeyama, as the
benchmark does, and with Sim(3) Umeyama; prints both RMSEs and the Sim(3) scale (estimate length / GT length - 1
in percent), for the whole trial and for windows of <win> seconds.
Usage: scale_check.py <gt_tum> <est_tum> [win_s=0]
"""
import sys

import numpy as np

gt = np.loadtxt(sys.argv[1])
est = np.loadtxt(sys.argv[2])
win = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
g = gt[(gt[:, 0] >= est[0, 0]) & (gt[:, 0] <= est[-1, 0])]
e_all = np.column_stack([np.interp(g[:, 0], est[:, 0], est[:, k]) for k in (1, 2, 3)])
p_all = g[:, 1:4]


def align(e, p, with_scale):
    me, mp = e.mean(0), p.mean(0)
    E, P = e - me, p - mp
    U, D, Vt = np.linalg.svd(P.T @ E / len(e))
    S = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / (E ** 2).sum(1).mean() if with_scale else 1.0
    err = s * (R @ E.T).T + mp - p
    return np.sqrt((err ** 2).sum(1).mean()), s


r1, _ = align(e_all, p_all, False)
r2, s = align(e_all, p_all, True)
print(f'whole: se3 {r1:.4f}  sim3 {r2:.4f}  scale {(1 / s - 1) * 100:+.3f}% (estimate longer if +)')
if win > 0:
    t = g[:, 0] - g[0, 0]
    print('t0_s,path_m,se3,sim3,scale_pct')
    for a in np.arange(0, t[-1] - win / 2, win):
        m = (t >= a) & (t < a + win)
        if m.sum() < 100:
            continue
        path = np.linalg.norm(np.diff(p_all[m], axis=0), axis=1).sum()
        if path < 10:
            continue
        r1, _ = align(e_all[m], p_all[m], False)
        r2, s = align(e_all[m], p_all[m], True)
        print(f'{a:.0f},{path:.0f},{r1:.4f},{r2:.4f},{(1 / s - 1) * 100:+.3f}')
