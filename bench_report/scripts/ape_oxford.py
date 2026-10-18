#!/usr/bin/env python3
"""APE of an RKO-LIO trajectory on Oxford Spires against the dataset's ground truth.

The estimate is RKO-LIO's base = IMU frame; the ground truth (processed/trajectory/gt-tum.txt) is the dataset's
'base' frame (0.124 m above the LiDAR, turned 180 deg about z). The estimate is moved to that frame with
T_world_base = T_world_imu * T_imu_base (T_imu_base from the dataset's cam-lidar-imu.yaml, see calib/), then scored
with scripts/ape_from_tum.py --interpolate (SE(3) Umeyama) like the other benchmarks. Also prints the per-axis error
of the aligned trajectory and the error at every 10 % of the time span.
Usage: ape_oxford.py <front_tum> <gt_tum> <T_imu_base.npy> <out_dir>
"""
import os
import subprocess
import sys

import numpy as np

REPO = '/home/user/lidar_slam_ros2'
est_path, gt_path, extrinsic_path, out = sys.argv[1:5]
os.makedirs(out, exist_ok=True)


def read_tum(path):
    rows = [list(map(float, l.split())) for l in open(path) if l.strip() and not l.startswith('#')]
    return np.array(rows)


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    """Shepperd's method: stable for every rotation, including half turns."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 2 * np.sqrt(trace + 1)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [s / 4, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2 * np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 1] + R[1, 0]) / s, s / 4, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    else:
        s = 2 * np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4, (R[1, 0] - R[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


T_imu_base = np.load(extrinsic_path)
est = read_tum(est_path)
moved = []
for row in est:
    T = np.eye(4)
    T[:3, :3] = quat_to_R(row[4:8])
    T[:3, 3] = row[1:4]
    B = T @ T_imu_base
    moved.append([row[0], *B[:3, 3], *R_to_quat(B[:3, :3])])
moved_path = os.path.join(out, 'estimate_base_frame.tum')
np.savetxt(moved_path, np.array(moved), fmt='%.9f')
report = os.path.join(out, 'ape.txt')
subprocess.run(['python3', f'{REPO}/scripts/ape_from_tum.py', '--ref', gt_path, '--est', moved_path, '--out', report,
                '--interpolate'], check=True)
print(open(report).read())

# Per-axis and over-time error after the same SE(3) Umeyama alignment, at the ground-truth stamps.
gt = read_tum(gt_path)
est_b = np.array(moved)
inside = (gt[:, 0] >= est_b[0, 0]) & (gt[:, 0] <= est_b[-1, 0])
g = gt[inside]
e = np.column_stack([np.interp(g[:, 0], est_b[:, 0], est_b[:, k]) for k in (1, 2, 3)])
p = g[:, 1:4]
mu_e, mu_p = e.mean(0), p.mean(0)
U, S, Vt = np.linalg.svd((p - mu_p).T @ (e - mu_e))
D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
R = U @ D @ Vt
aligned = (R @ (e - mu_e).T).T + mu_p
err = aligned - p
print('aligned per-axis RMSE x/y/z (m):', np.sqrt((err ** 2).mean(0)).round(4))
print('aligned 3D RMSE (m):', round(float(np.sqrt((err ** 2).sum(1).mean())), 4))
for frac in np.linspace(0.1, 1.0, 10):
    i = min(len(g) - 1, int(frac * (len(g) - 1)))
    print(f'  t={g[i, 0] - g[0, 0]:7.1f} s  |err|={np.linalg.norm(err[i]):.3f} m  dz={err[i, 2]:+.3f} m')
length = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
print(f'reference path length {length:.1f} m over {g[-1, 0] - g[0, 0]:.1f} s; estimate poses {len(est_b)}')
