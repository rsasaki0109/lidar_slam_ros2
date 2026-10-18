#!/usr/bin/env python3
"""Scores a frontend trajectory (IMU frame, TUM) on a MapsHD trial like scripts/mapshd_score.py does for the
frontend: moved to the LiDAR frame (T_imu_lidar), evo_ape -a against the ground truth. Also the per-axis RMSE of
the same SE(3)-aligned trajectory (nearest GT stamp within 10 ms) and the high-frequency vertical error (z error
minus its 5 s moving average). Prints: label,ape_rmse,rmse_x,rmse_y,rmse_z,z_hf_std
Usage: front_score.py <label> <front_tum> <gt_tum>"""
import os
import re
import subprocess
import sys
import tempfile

import numpy as np

EVO_APE = '/home/user/mapshd/venv/bin/evo_ape'
T_IMU_LIDAR = np.array([-0.011, -0.02329, 0.04412])


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


label, front_path, gt_path = sys.argv[1:4]
fr = np.loadtxt(front_path)
lid = np.array([np.r_[r[0], r[1:4] + quat_to_R(r[4:8]) @ T_IMU_LIDAR, r[4:8]] for r in fr])
with tempfile.TemporaryDirectory() as tmp:
    p = os.path.join(tmp, 'lidar.tum')
    np.savetxt(p, lid, fmt='%.9f')
    res = subprocess.run([EVO_APE, 'tum', gt_path, p, '-a'], capture_output=True, text=True,
                         env=dict(os.environ, MPLBACKEND='Agg'))
    ape = float(re.search(r'^\s*rmse\s+([0-9.eE+-]+)', res.stdout, re.M).group(1))
gt = np.loadtxt(gt_path)
k = np.clip(np.searchsorted(gt[:, 0], lid[:, 0]), 1, len(gt) - 1)
k = np.where(np.abs(gt[k - 1, 0] - lid[:, 0]) < np.abs(gt[k, 0] - lid[:, 0]), k - 1, k)
ok = np.abs(gt[k, 0] - lid[:, 0]) < 0.01
A, B, t = lid[ok, 1:4], gt[k[ok], 1:4], lid[ok, 0]
ma, mb = A.mean(0), B.mean(0)
U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
D = np.eye(3)
D[2, 2] = np.sign(np.linalg.det(U @ Vt))
R = U @ D @ Vt
e = (R @ (A - ma).T).T + mb - B
w = max(1, int(round(5.0 / np.median(np.diff(t)))))
ez = e[:, 2] - np.convolve(e[:, 2], np.ones(w) / w, mode='same')
ez = ez[w:-w]
rx, ry, rz = np.sqrt((e ** 2).mean(0))
print(f'{label},{ape:.6f},{rx:.6f},{ry:.6f},{rz:.6f},{ez.std():.6f}')
