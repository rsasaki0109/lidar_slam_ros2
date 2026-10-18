#!/usr/bin/env python3
"""Translation and rotation error, against the ground truth, of every loop edge of a run and of the frontend's
relative pose between the same two submaps, with the path length between them.

The submap poses (trajectory_optimized.tum stamps) and the loop edges are in the IMU frame; the ground truth is in
the frame of T_imu_gt (the GT pose times T_gt_imu is the IMU pose). Writes one CSV row per loop edge.
Usage: loop_edge_errors.py <label> <run_dir> <gt_tum> <T_imu_gt.npy>
"""
import sys

import numpy as np


def read_tum(path):
    return np.array([list(map(float, l.split())) for l in open(path) if l.strip() and not l.startswith('#')])


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose(row):
    T = np.eye(4)
    T[:3, :3] = quat_to_R(np.asarray(row[3:7]))
    T[:3, 3] = row[0:3]
    return T


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


def at(traj, t):
    i = int(np.clip(np.searchsorted(traj[:, 0], t), 1, len(traj) - 1))
    a, b = traj[i - 1], traj[i]
    u = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
    return pose(np.concatenate([a[1:4] + u * (b[1:4] - a[1:4]), slerp(a[4:8], b[4:8], u)]))


def err(A, B):
    D = np.linalg.inv(A) @ B
    angle = np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))
    return np.linalg.norm(D[:3, 3]), angle


label, run, gt_path, T_path = sys.argv[1:5]
stamps = read_tum(f'{run}/trajectory_optimized.tum')[:, 0]
front = read_tum(f'{run}/front_0/front_tum_0.txt')
gt = read_tum(gt_path)
T_gt_imu = np.linalg.inv(np.load(T_path))
edges = np.loadtxt(f'{run}/loop_edges.csv', delimiter=',', skiprows=1, ndmin=2)
fx = front[:, 1:4]
cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(fx, axis=0), axis=1))])
for e in edges:
    i, j = int(e[0]), int(e[1])
    L = pose(e[3:10])
    F = np.linalg.inv(at(front, stamps[i])) @ at(front, stamps[j])
    G = np.linalg.inv(at(gt, stamps[i]) @ T_gt_imu) @ (at(gt, stamps[j]) @ T_gt_imu)
    lt, lr = err(G, L)
    ft, fr = err(G, F)
    path = np.interp(stamps[j], front[:, 0], cum) - np.interp(stamps[i], front[:, 0], cum)
    print(f'{label},{i},{j},{e[2]:.4f},{path:.1f},{lt:.4f},{lr:.3f},{ft:.4f},{fr:.3f}')
