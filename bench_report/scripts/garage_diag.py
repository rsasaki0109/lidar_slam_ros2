#!/usr/bin/env python3
"""Error shape of a frontend trajectory on a MapsHD trial (LiDAR frame, nearest GT stamp within 10 ms):
SE(3) vs Sim(3) Umeyama RMSE and scale, per-axis RMSE, along-track vs cross-track error, travelled distance of
estimate vs GT, and the error at every 10 % of the run. Usage: garage_diag.py <front_tum> <gt_tum>"""
import sys

import numpy as np

T_IMU_LIDAR = np.array([-0.011, -0.02329, 0.04412])


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def umeyama(A, B, scale):
    ma, mb = A.mean(0), B.mean(0)
    Ac, Bc = A - ma, B - mb
    U, S, Vt = np.linalg.svd(Bc.T @ Ac / len(A))
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    s = (S * np.diag(D)).sum() / (Ac ** 2).sum(1).mean() if scale else 1.0
    return s, R, mb - s * R @ ma


fr = np.loadtxt(sys.argv[1]); gt = np.loadtxt(sys.argv[2])
lid = np.array([r[1:4] + quat_to_R(r[4:8]) @ T_IMU_LIDAR for r in fr])
k = np.clip(np.searchsorted(gt[:, 0], fr[:, 0]), 1, len(gt) - 1)
k = np.where(np.abs(gt[k - 1, 0] - fr[:, 0]) < np.abs(gt[k, 0] - fr[:, 0]), k - 1, k)
ok = np.abs(gt[k, 0] - fr[:, 0]) < 0.01
A, B, t = lid[ok], gt[k[ok], 1:4], fr[ok, 0]
for scale in (False, True):
    s, R, tr = umeyama(A, B, scale)
    e = (s * (R @ A.T)).T + tr - B
    print(f'{"Sim3" if scale else "SE3 "}: scale {s:.5f}  RMSE {np.sqrt((e ** 2).sum(1).mean()):.4f} m  '
          f'x/y/z {np.sqrt((e ** 2).mean(0)).round(4)}')
s, R, tr = umeyama(A, B, False)
Aa = (R @ A.T).T + tr
e = Aa - B
d = np.gradient(B, axis=0)
d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
along = (e * d).sum(1)
cross = np.linalg.norm(e - along[:, None] * d, axis=1)
print(f'along-track RMSE {np.sqrt((along ** 2).mean()):.4f} m, cross-track RMSE {np.sqrt((cross ** 2).mean()):.4f} m')
lg = np.linalg.norm(np.diff(B, axis=0), axis=1).sum(); le = np.linalg.norm(np.diff(A, axis=0), axis=1).sum()
print(f'path length GT {lg:.1f} m, estimate {le:.1f} m (ratio {le / lg:.4f}); GT extent x {np.ptp(B[:, 0]):.1f} y {np.ptp(B[:, 1]):.1f} z {np.ptp(B[:, 2]):.1f} m')
n = len(t)
for i in np.linspace(0, n - 1, 11).astype(int):
    print(f'  t={t[i] - t[0]:6.1f} s  gt=({B[i, 0]:+7.2f},{B[i, 1]:+7.2f})  err=({e[i, 0]:+.3f},{e[i, 1]:+.3f},{e[i, 2]:+.3f})  along {along[i]:+.3f}')
