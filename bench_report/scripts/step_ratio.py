#!/usr/bin/env python3
"""What the frontend's over-estimate of travel goes with (chapter 29).

Over windows of <win> seconds: horizontal displacement of the frontend (LiDAR frame) and of the ground truth,
both in their own world frames, plus the GT's horizontal speed, the yaw rate and the share of the displacement
along the sensor's own x axis (forward walking vs sideways). Prints the length-weighted excess
(sum |d_est| / sum |d_gt| - 1) overall and binned by speed, yaw rate and |cos(heading - sensor x)|.
Usage: step_ratio.py <gt_tum> <est_lidar_tum> [win_s=2]
"""
import sys

import numpy as np
from scipy.spatial.transform import Rotation

gt = np.loadtxt(sys.argv[1])
est = np.loadtxt(sys.argv[2])
win = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
t = np.arange(max(gt[0, 0], est[0, 0]) + 1, min(gt[-1, 0], est[-1, 0]) - 1, win)


def pos(traj, s):
    return np.column_stack([np.interp(s, traj[:, 0], traj[:, k]) for k in (1, 2, 3)])


pe, pg = pos(est, t), pos(gt, t)
de = np.linalg.norm(np.diff(pe[:, :2], axis=0), axis=1)
dg = np.linalg.norm(np.diff(pg[:, :2], axis=0), axis=1)
yaw = Rotation.from_quat(est[:, 4:8]).as_euler('zyx')[:, 0]
yaw_t = np.unwrap(np.interp(t, est[:, 0], np.unwrap(yaw)))
yaw_rate = np.abs(np.diff(yaw_t)) / win
# sensor x axis in the world (estimate frame) vs the estimate's displacement direction
R = Rotation.from_quat(np.column_stack([np.interp(t[:-1], est[:, 0], est[:, k]) for k in (4, 5, 6, 7)]))
xs = R.apply(np.array([1.0, 0.0, 0.0]))[:, :2]
dirs = np.diff(pe[:, :2], axis=0)
cosx = np.abs((xs * dirs).sum(1) / np.maximum(np.linalg.norm(xs, axis=1) * de, 1e-9))
speed = dg / win
m = speed > 0.3
print(f'overall (moving > 0.3 m/s): excess {(de[m].sum() / dg[m].sum() - 1) * 100:+.3f}%  ({m.sum()} windows)')
for name, v, edges in [('speed m/s', speed, [0.3, 0.7, 0.85, 1.0, 2.0]),
                       ('yaw rate rad/s', yaw_rate, [0.0, 0.05, 0.15, 0.4, 5.0]),
                       ('|cos(motion, sensor x)|', cosx, [0.0, 0.5, 0.85, 0.97, 1.01])]:
    print(name)
    for a, b in zip(edges[:-1], edges[1:]):
        k = m & (v >= a) & (v < b)
        if k.sum() >= 3:
            print(f'  [{a}, {b}): excess {(de[k].sum() / dg[k].sum() - 1) * 100:+.3f}%  n={k.sum()}')
