#!/usr/bin/env python3
"""Tilt (roll/pitch) error of an Oxford estimate after the same SE(3) Umeyama alignment as ape_oxford.py.

Reads <ape_dir>/estimate_base_frame.tum written by ape_oxford.py (estimate already in the dataset's base frame) and
the ground truth. Positions are interpolated at the ground-truth stamps and aligned as in ape_oxford.py; orientations
are interpolated by slerp. The tilt error is the angle between the body's up direction in the aligned estimate and in
the ground truth, so it ignores heading. Usage: tilt_oxford.py <ape_dir> <gt_tum>
"""
import sys

import numpy as np

ape_dir, gt_path = sys.argv[1:3]


def read_tum(path):
    return np.array([list(map(float, l.split())) for l in open(path) if l.strip() and not l.startswith('#')])


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def slerp(q0, q1, u):
    if np.dot(q0, q1) < 0:
        q1 = -q1
    d = min(1.0, float(np.dot(q0, q1)))
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
    else:
        th = np.arccos(d)
        q = (np.sin((1 - u) * th) * q0 + np.sin(u * th) * q1) / np.sin(th)
    return q / np.linalg.norm(q)


est = read_tum(f'{ape_dir}/estimate_base_frame.tum')
gt = read_tum(gt_path)
g = gt[(gt[:, 0] >= est[0, 0]) & (gt[:, 0] <= est[-1, 0])]
e = np.column_stack([np.interp(g[:, 0], est[:, 0], est[:, k]) for k in (1, 2, 3)])
p = g[:, 1:4]
mu_e, mu_p = e.mean(0), p.mean(0)
U, _, Vt = np.linalg.svd((p - mu_p).T @ (e - mu_e))
R_align = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
idx = np.clip(np.searchsorted(est[:, 0], g[:, 0]), 1, len(est) - 1)
tilt = []
for i, t in enumerate(g[:, 0]):
    a, b = est[idx[i] - 1], est[idx[i]]
    u = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
    R_est = R_align @ quat_to_R(slerp(a[4:8], b[4:8], u))
    R_gt = quat_to_R(g[i, 4:8])
    up_est, up_gt = R_est.T @ np.array([0, 0, 1.0]), R_gt.T @ np.array([0, 0, 1.0])
    tilt.append(np.degrees(np.arccos(np.clip(up_est @ up_gt, -1, 1))))
tilt = np.array(tilt)
print(f'tilt error (deg): median {np.median(tilt):.3f}  p95 {np.percentile(tilt, 95):.3f}  max {tilt.max():.3f}  '
      f'rms {np.sqrt((tilt ** 2).mean()):.3f}  (n={len(tilt)})')
