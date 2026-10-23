#!/usr/bin/env python3
"""Where and along which direction a MapsHD trajectory's error builds up (chapter 28).

Aligns the estimate to the ground truth like the benchmark (SE(3) Umeyama on positions interpolated at the GT
stamps), then reports the error in the GT path frame: along-track (the GT's horizontal direction of travel),
cross-track (horizontal, perpendicular to it) and vertical. Also prints the error every <step> seconds with the
GT's heading, speed and travelled distance, and the unaligned drift (estimate minus GT after aligning only the
first <head> seconds).
Usage: corridor_error_profile.py <gt_tum> <est_tum> [step_s=10] [head_s=5]
"""
import sys

import numpy as np

gt_path, est_path = sys.argv[1:3]
step = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
head = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0
gt = np.loadtxt(gt_path)
est = np.loadtxt(est_path)
g = gt[(gt[:, 0] >= est[0, 0]) & (gt[:, 0] <= est[-1, 0])]
t = g[:, 0]
p = g[:, 1:4]
e = np.column_stack([np.interp(t, est[:, 0], est[:, k]) for k in (1, 2, 3)])


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((dst - mu_d).T @ (src - mu_s))
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    return R, mu_d - R @ mu_s


R, tr = umeyama(e, p)
err = (R @ e.T).T + tr - p
# GT horizontal direction of travel, smoothed over 1 s
vel = np.gradient(p, t, axis=0)
k = max(1, int(round(1.0 / np.median(np.diff(t)))))
ker = np.ones(k) / k
vx = np.convolve(vel[:, 0], ker, mode='same')
vy = np.convolve(vel[:, 1], ker, mode='same')
speed = np.hypot(vx, vy)
along_dir = np.column_stack([vx, vy]) / np.maximum(speed, 1e-6)[:, None]
cross_dir = np.column_stack([-along_dir[:, 1], along_dir[:, 0]])
moving = speed > 0.1
e_along = (err[:, :2] * along_dir).sum(1)
e_cross = (err[:, :2] * cross_dir).sum(1)
rms = lambda a: float(np.sqrt(np.mean(a ** 2)))  # noqa: E731
print(f'aligned rmse 3d {np.sqrt((err ** 2).sum(1).mean()):.4f}  x {rms(err[:, 0]):.4f} y {rms(err[:, 1]):.4f} '
      f'z {rms(err[:, 2]):.4f}')
print(f'while moving (> 0.1 m/s, {moving.mean():.0%} of the time): along {rms(e_along[moving]):.4f} '
      f'cross {rms(e_cross[moving]):.4f} vertical {rms(err[moving, 2]):.4f}')
dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
# drift: align only on the first `head` seconds
h = t <= t[0] + head
Rh, trh = umeyama(e[h], p[h]) if h.sum() > 10 else (R, tr)
drift = (Rh @ e.T).T + trh - p
print('t_s,dist_m,heading_deg,speed_mps,err_x,err_y,err_z,err_along,err_cross,drift_x,drift_y,drift_z')
for t0 in np.arange(t[0], t[-1], step):
    i = int(np.searchsorted(t, t0))
    hd = np.degrees(np.arctan2(along_dir[i, 1], along_dir[i, 0]))
    print(f'{t[i] - t[0]:.0f},{dist[i]:.1f},{hd:.0f},{speed[i]:.2f},{err[i, 0]:.3f},{err[i, 1]:.3f},{err[i, 2]:.3f},'
          f'{e_along[i]:.3f},{e_cross[i]:.3f},{drift[i, 0]:.3f},{drift[i, 1]:.3f},{drift[i, 2]:.3f}')
