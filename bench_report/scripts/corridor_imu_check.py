#!/usr/bin/env python3
"""Is a short along-track excursion of a MapsHD trajectory physical? (chapter 28)

Rotates the raw IMU specific force (HDMapping imu*.csv, g units) into the world with the frontend's orientation,
removes gravity, and compares the resulting horizontal velocity change over a short window with the velocity
change that the ground truth and the frontend trajectory imply over the same window. A pose excursion that the IMU
does not see needs accelerations far above walking's.
Usage: corridor_imu_check.py <hdmapping_raw_dir> <gt_tum> <frontend_lidar_tum> <t_rel_start> <t_rel_end> [win_s=0.2]
(times relative to the first GT stamp)
"""
import glob
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

raw, gt_path, est_path = sys.argv[1:4]
ta, tb = float(sys.argv[4]), float(sys.argv[5])
win = float(sys.argv[6]) if len(sys.argv) > 6 else 0.2
imu = np.vstack([np.loadtxt(f, skiprows=1, usecols=range(7)) for f in sorted(glob.glob(os.path.join(raw, 'imu*.csv')))])
imu = imu[np.argsort(imu[:, 0])]
ti = imu[:, 0] * 1e-9
gt = np.loadtxt(gt_path)
est = np.loadtxt(est_path)
t0 = gt[0, 0]
slerp = Slerp(est[:, 0], Rotation.from_quat(est[:, 4:8]))
m = (ti > est[0, 0]) & (ti < est[-1, 0])
ti, imu = ti[m], imu[m]
a_world = slerp(ti).apply(imu[:, 4:7] * 9.80665)
# gravity: the mean specific force over the trial (the robot ends where it starts moving slowly)
g = np.median(a_world[(ti - t0) < 20], axis=0)
lin = a_world - g
print(f'gravity estimate {np.round(g, 3)} m/s^2 (|g| {np.linalg.norm(g):.3f})')
rms_h = np.sqrt(np.mean((lin[:, :2] ** 2).sum(1)[(ti - t0 > 25) & (ti - t0 < 95)]))
print(f'IMU horizontal linear acceleration while walking: rms {rms_h:.2f} m/s^2, '
      f'p99 {np.percentile(np.linalg.norm(lin[(ti - t0 > 25) & (ti - t0 < 95), :2], axis=1), 99):.2f} m/s^2')


def pos(traj, t):
    return np.column_stack([np.interp(t, traj[:, 0], traj[:, k]) for k in (1, 2, 3)])


def vel(traj, t, h=0.05):
    return (pos(traj, t + h) - pos(traj, t - h)) / (2 * h)


print('t_rel,dv_imu_xy,dv_gt_xy,dv_est_xy  (velocity change over the next win_s, m/s; horizontal vectors as |.|)')
for s in np.arange(ta, tb, win / 2):
    a, b = t0 + s, t0 + s + win
    k = (ti >= a) & (ti < b)
    dv_imu = np.trapz(lin[k, :2], ti[k], axis=0) if k.sum() > 1 else np.zeros(2)
    dv_gt = (vel(gt, np.array([b])) - vel(gt, np.array([a])))[0, :2]
    dv_est = (vel(est, np.array([b])) - vel(est, np.array([a])))[0, :2]
    print(f'{s:.2f},{np.linalg.norm(dv_imu):.2f},{np.linalg.norm(dv_gt):.2f},{np.linalg.norm(dv_est):.2f}')
