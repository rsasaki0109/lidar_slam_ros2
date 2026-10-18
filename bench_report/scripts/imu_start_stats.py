#!/usr/bin/env python3
"""Angular rate and acceleration statistics of the first seconds of an Oxford Spires raw/imu.csv (no ground truth).

A static IMU shows |mean gyro| and RMS |gyro| at the bias and noise level (a few 1e-3 rad/s) and std |acc| near
the accelerometer noise; a hand-held start in motion does not. Usage: imu_start_stats.py <imu.csv>
"""
import csv
import sys

import numpy as np

rows = []
with open(sys.argv[1]) as f:
    for i, row in enumerate(csv.DictReader(f)):
        if i > 400 * 30:
            break
        rows.append([int(row['secs']) + int(row['nsecs']) * 1e-9] +
                    [float(row[k]) for k in ('ang_vel_x', 'ang_vel_y', 'ang_vel_z', 'acc_x', 'acc_y', 'acc_z')])
a = np.array(rows)
t = a[:, 0] - a[0, 0]
print('window_s     n  |mean_gyro|_rad_s  rms|gyro|_rad_s  std|acc|_m_s2  mean|acc|_m_s2')
for lo, hi in [(0, 0.1), (0, 0.5), (0, 1), (1, 2), (2, 5), (5, 10), (10, 20)]:
    m = (t >= lo) & (t < hi)
    g, acc = a[m, 1:4], a[m, 4:7]
    print(f'{lo:4.1f}-{hi:4.1f} {m.sum():5d} {np.linalg.norm(g.mean(0)):18.4f} '
          f'{np.sqrt((np.linalg.norm(g, axis=1) ** 2).mean()):16.4f} {np.linalg.norm(acc, axis=1).std():14.3f} '
          f'{np.linalg.norm(acc, axis=1).mean():15.3f}')
