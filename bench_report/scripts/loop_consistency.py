#!/usr/bin/env python3
"""Loop edges of a graph_based_slam run against the frontend's relative poses and, optionally, the ground truth.

For every accepted loop edge (loop_edges.csv: from, to, fitness, relative pose T_from^-1 T_to) prints the frontend's
relative pose between the same two submaps (front_0/front_tum_0.txt at the submap stamps of
trajectory_optimized.tum) and the difference. With a ground truth (dataset base frame) and T_imu_base, also prints
the error of the loop edge and of the frontend against the ground-truth relative pose, all in the IMU frame.
Usage: loop_consistency.py <run_dir> [<gt_tum> <T_imu_base.npy>]
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


def diff(A, B):
    D = np.linalg.inv(A) @ B
    angle = np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))
    return np.linalg.norm(D[:3, 3]), angle


run = sys.argv[1]
stamps = read_tum(f'{run}/trajectory_optimized.tum')[:, 0]
front = read_tum(f'{run}/front_0/front_tum_0.txt')
gt = read_tum(sys.argv[2]) if len(sys.argv) > 3 else None
T_base_imu = np.linalg.inv(np.load(sys.argv[3])) if gt is not None else None
edges = np.loadtxt(f'{run}/loop_edges.csv', delimiter=',', skiprows=1, ndmin=2)
print('from  to span fitness   info  |t|_m  front_dt_m front_drot_deg' + ('  gt_loop_dt_m gt_front_dt_m' if gt is not None else ''))
rows = []
for e in edges:
    i, j = int(e[0]), int(e[1])
    L = pose(e[3:10])
    F = np.linalg.inv(at(front, stamps[i])) @ at(front, stamps[j])
    dt, dr = diff(F, L)
    line = f'{i:4d} {j:4d} {j - i:4d} {e[2]:7.4f} {100 / max(e[2], 1e-3):6.0f} {np.linalg.norm(L[:3, 3]):6.2f} {dt:10.3f} {dr:13.2f}'
    row = [i, j, e[2], dt, dr]
    if gt is not None:
        G = np.linalg.inv(at(gt, stamps[i]) @ T_base_imu) @ (at(gt, stamps[j]) @ T_base_imu)
        g_l, _ = diff(G, L)
        g_f, _ = diff(G, F)
        line += f' {g_l:13.3f} {g_f:13.3f}'
        row += [g_l, g_f]
    rows.append(row)
    print(line)
r = np.array(rows)
print(f'loop edges {len(r)}; loop vs frontend: median {np.median(r[:, 3]):.3f} m, max {r[:, 3].max():.3f} m')
if gt is not None:
    print(f'against GT: loop edges median {np.median(r[:, 5]):.3f} m (max {r[:, 5].max():.3f}), '
          f'frontend median {np.median(r[:, 6]):.3f} m (max {r[:, 6].max():.3f})')
