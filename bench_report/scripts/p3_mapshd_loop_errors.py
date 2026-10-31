#!/usr/bin/env python3
"""
Loop edge errors of p3_dev_backend.sh runs on a MapsHD trial against its 6-DoF ground truth (Phase 3, robust loops on
the other development data): for each loop edge, the translation (mm) and rotation (deg) of its relative pose against
the ground truth's relative pose of the same two submap stamps (the ground truth is in the LiDAR frame, moved to the
IMU frame with the MID-360 extrinsic, identity rotation), and the same for the frontend's own relative pose.
For development data only (reads the ground truth).
Usage: p3_mapshd_loop_errors.py <run dir (p3_dev_backend.sh output of one trial)> <ground_truth.tum> <variant> ...
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
run, gt_path = sys.argv[1:3]
T_IL = np.array([-0.011, -0.02329, 0.04412])  # IMU -> LiDAR translation, identity rotation
def tum(p):
    return np.array([list(map(float, l.split())) for l in open(p) if l.strip() and not l.startswith('#')])
def T(row):
    m = np.eye(4); m[:3, :3] = Rot.from_quat(row[4:8]).as_matrix(); m[:3, 3] = row[1:4]; return m
gt = tum(gt_path); front = tum(f'{run}/front/front_0/front_tum_0.txt')
def interp(traj, t):
    i = int(np.clip(np.searchsorted(traj[:, 0], t), 1, len(traj) - 1))
    a, b = traj[i - 1], traj[i]
    if abs(a[0] - t) > 0.05 and abs(b[0] - t) > 0.05:
        return None
    u = (t - a[0]) / (b[0] - a[0]); r = a.copy(); r[1:4] = a[1:4] + u * (b[1:4] - a[1:4])
    r[4:8] = a[4:8] if u < 0.5 else b[4:8]
    return T(r)
off = np.eye(4); off[:3, 3] = -T_IL  # T_lidar_imu
for v in sys.argv[3:]:
    stamps = tum(f'{run}/{v}/trajectory_raw.tum')[:, 0]
    edges = np.loadtxt(f'{run}/{v}/loop_edges.csv', delimiter=',', skiprows=1, ndmin=2)
    out = []
    for e in edges:
        i, j = int(e[0]), int(e[1])
        gi, gj = interp(gt, stamps[i]), interp(gt, stamps[j])
        fi, fj = interp(front, stamps[i]), interp(front, stamps[j])
        if gi is None or gj is None:
            continue
        G = np.linalg.inv(gi @ off) @ (gj @ off)
        F = np.linalg.inv(fi) @ fj
        E = np.eye(4); E[:3, :3] = Rot.from_quat(e[6:10]).as_matrix(); E[:3, 3] = e[3:6]
        def err(A):
            D = np.linalg.inv(G) @ A
            return np.linalg.norm(D[:3, 3]) * 1e3, np.degrees(np.linalg.norm(Rot.from_matrix(D[:3, :3]).as_rotvec()))
        le, lr = err(E); fe, fr = err(F)
        out.append((i, j, np.linalg.norm(e[3:6]), le, lr, fe, fr))
    out = np.array(out)
    print(f'{v}: {len(edges)} edges, {len(out)} with GT; loop err median {np.median(out[:,3]):.1f} mm / {np.median(out[:,4]):.3f} deg, '
          f'max {out[:,3].max():.1f} mm / {out[:,4].max():.3f} deg; frontend median {np.median(out[:,5]):.1f} mm / {np.median(out[:,6]):.3f} deg; '
          f'loop better (translation) {int((out[:,3] < out[:,5]).sum())}/{len(out)}')
