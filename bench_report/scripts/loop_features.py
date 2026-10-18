#!/usr/bin/env python3
"""GT-free features of every accepted loop edge of a run: fitness, frontend path length between the two submaps,
and the correction the edge asks for relative to the frontend's relative pose between them (translation of the
later submap's origin and rotation angle). Frontend poses are taken at the submap stamps (trajectory_optimized.tum).
Usage: loop_features.py <label> <run_dir>   (CSV rows: label,from,to,fitness,path_m,dist_m,corr_t_m,corr_r_deg)"""
import sys

import numpy as np


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T(t, q):
    m = np.eye(4)
    m[:3, :3] = quat_to_R(np.asarray(q))
    m[:3, 3] = t
    return m


label, run = sys.argv[1:3]
st = np.array([list(map(float, l.split())) for l in open(f'{run}/trajectory_optimized.tum')
               if l.strip() and not l.startswith('#')])[:, 0]
fr = np.loadtxt(f'{run}/front_0/front_tum_0.txt')
idx = [int(np.argmin(np.abs(fr[:, 0] - t))) for t in st]
F = [T(fr[k, 1:4], fr[k, 4:8]) for k in idx]
pos = np.array([f[:3, 3] for f in F])
cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))])
rows = [l.strip().split(',') for l in open(f'{run}/loop_edges.csv')][1:]
for r in rows:
    i, j = int(r[0]), int(r[1])
    v = list(map(float, r[2:]))
    L = T(v[1:4], v[4:8])
    D = np.linalg.inv(np.linalg.inv(F[i]) @ F[j]) @ L
    ang = np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))
    print(f'{label},{i},{j},{v[0]:.4f},{cum[j] - cum[i]:.1f},{np.linalg.norm(pos[j] - pos[i]):.1f},'
          f'{np.linalg.norm(D[:3, 3]):.3f},{ang:.2f}')
