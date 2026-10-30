#!/usr/bin/env python3
"""
Multi-session step 7, a check without ground truth for the 3-part splits (plan.txt).

The identity is the truth only up to the frontend's drift between the parts. The runner's own loop-closed trajectory of
the whole session (trajectory_optimized.tum) gives another reference: each part's raw submap poses fitted to it (SE(3))
place the part relative to part 0. Prints, per joined part: the join's transform against the identity and against that
reference, and the optimized joint graph's submap positions against the reference (RMS over the part's submaps, m).
Usage: p3_split3_loopcheck.py <join output dir> <runner dir>
"""
import sys

import numpy as np
import yaml


def fit(a, b):
    """R, t with R a + t ~ b (least squares)."""
    ca, cb = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    d = np.diag([1.0, 1.0, np.sign(np.linalg.det(vt.T @ u.T))])
    r = vt.T @ d @ u.T
    return r, cb - r @ ca


def main():
    """Print the check for one join."""
    join, runner = sys.argv[1:3]
    raw = np.loadtxt(f'{runner}/trajectory_raw.tum')
    opt = np.loadtxt(f'{runner}/trajectory_optimized.tum')
    report = yaml.safe_load(open(f'{join}/join_report.yaml'))
    t0, t1 = raw[0, 0], raw[-1, 0]
    part_s = (t1 - t0) / 3.0
    gap = 30.0
    t = raw[:, 0] - t0
    p = np.minimum(2, (t // part_s).astype(int))
    keep = ~(((p > 0) & (t - p * part_s <= gap / 2)) | ((p < 2) & ((p + 1) * part_s - t <= gap / 2)))
    opt_pos = np.stack([np.interp(raw[:, 0], opt[:, 0], opt[:, k]) for k in (1, 2, 3)], 1)
    ref = {}
    for k in range(3):
        m = keep & (p == k)
        ref[k] = fit(raw[m, 1:4], opt_pos[m])
    r0, c0 = ref[0]
    for s in report['sessions'][1:]:
        k = s['session']
        if not s['joined']:
            continue
        m = keep & (p == k)
        x = raw[m, 1:4]
        tj = np.eye(4)
        tj[:3] = np.array(s['transform_to_session_0']).reshape(3, 4)
        joined = x @ tj[:3, :3].T + tj[:3, 3]
        rk, ck = ref[k]
        loop = ((x @ rk.T + ck) - c0) @ r0  # part 0's raw frame
        rms = lambda d: float(np.sqrt((np.linalg.norm(d, axis=1) ** 2).mean()))  # noqa: E731
        graph = np.loadtxt(f'{join}/trajectory_session{k}.tum')[:, 1:4]  # the joint graph, optimized
        print(f'part {k}: transform to the identity {rms(joined - x):.4f} m, to the loop-closed reference '
              f'{rms(joined - loop):.4f} m (reference vs identity {rms(loop - x):.4f} m); the optimized joint graph to '
              f'the reference {rms(graph - loop):.4f} m')


if __name__ == '__main__':
    main()
