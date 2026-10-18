#!/usr/bin/env python3
"""How well a trajectory satisfies each loop edge of a run: |T_i^-1 T_j - measured| in translation and rotation.
Also the same for the frontend (before any correction). Usage: loop_residuals.py <run_dir> <trajectory_tum>"""
import sys

import numpy as np


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T(r):
    m = np.eye(4)
    m[:3, :3] = quat_to_R(np.asarray(r[3:7]))
    m[:3, 3] = r[0:3]
    return m


def res(A, B):
    D = np.linalg.inv(A) @ B
    return np.linalg.norm(D[:3, 3]), np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))


run, traj = sys.argv[1:3]
est = np.loadtxt(traj)
edges = np.loadtxt(f'{run}/loop_edges.csv', delimiter=',', skiprows=1, ndmin=2)
for e in edges:
    i, j = int(e[0]), int(e[1])
    L = T(e[3:10])
    t, r = res(np.linalg.inv(T(est[i, 1:])) @ T(est[j, 1:]), L)
    print(f'{i:4d} {j:4d}  residual {t:7.3f} m {r:6.2f} deg')
