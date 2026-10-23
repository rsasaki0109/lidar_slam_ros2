#!/usr/bin/env python3
"""Sim(3) scale and SE(3)/Sim(3) RMSE of frontend runs (front_0/front_tum_0.txt, IMU frame) on a MapsHD trial,
moved to the LiDAR frame like front_score.py (chapter 29).
Usage: front_scale.py <gt_tum> <run_dir>...
"""
import os
import subprocess
import sys
import tempfile

import numpy as np

T_IMU_LIDAR = np.array([-0.011, -0.02329, 0.04412])
HERE = os.path.dirname(os.path.abspath(__file__))


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


gt = sys.argv[1]
for run in sys.argv[2:]:
    f = os.path.join(run, 'front_0', 'front_tum_0.txt')
    if not os.path.isfile(f):
        print(f'{os.path.basename(run)},missing')
        continue
    fr = np.loadtxt(f)
    lid = np.array([np.r_[r[0], r[1:4] + quat_to_R(r[4:8]) @ T_IMU_LIDAR, r[4:8]] for r in fr])
    with tempfile.NamedTemporaryFile('w', suffix='.tum', delete=False) as tmp:
        np.savetxt(tmp, lid, fmt='%.9f')
    out = subprocess.run(['python3', os.path.join(HERE, 'scale_check.py'), gt, tmp.name], capture_output=True,
                         text=True).stdout.strip()
    os.unlink(tmp.name)
    print(f'{os.path.basename(run)},{out}')
