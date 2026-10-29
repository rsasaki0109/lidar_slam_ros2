#!/usr/bin/env python3
"""
Translation error of each loop edge of a graph backend run on a GrandTour development mission, against the
reference (Phase 3, robust loops). The reference is the prism's position only, so the edge is compared in the frame
of its first submap with the frontend's attitude there:
  - the frontend trajectory (IMU frame) is moved to the prism and fitted to the reference (SE(3); points more than
    1 m off, the reference spikes, left out);
  - the reference IMU position at time t is the reference prism position in the frontend's frame minus the
    frontend's attitude times the IMU-to-prism lever;
  - for an edge (i, j) the reference translation is R_i^T (g_j - g_i), and the edge's error is its translation's
    distance from it; the frontend's own relative translation R_i^T (p_j - p_i) is scored the same way.
An error of the frontend's attitude at i of a degrees adds up to a * 17 mm per metre of edge length, so the edge
length is printed too. The reference is interpolated linearly at the stamps (until 2026-10-29 the nearest sample was
taken, which at about 23 Hz and walking speed put up to 2-4 cm into each end); edges whose stamps fall between
reference samples more than 0.2 s apart are skipped (the total station loses the prism on stairs and indoors).
For development missions only (reads the reference).
Usage: p3_grandtour_loop_errors.py <label> <mission> <backend dir> <frontend front_tum_0.txt> <extrinsics.txt>
Prints one CSV row per edge: label,mission,i,j,fitness,length_m,path_m,loop_err_m,frontend_err_m
"""
import sys

import numpy as np
import yaml
import zarr

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import matrix  # noqa: E402


def read_tum(path):
    return np.array([list(map(float, l.split())) for l in open(path) if l.strip() and not l.startswith('#')])


def rot(q):
    return matrix([0.0, 0.0, 0.0], q)[:3, :3]


def fit(A, B):
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def edge_errors(mission, backend, front_path, ext_path):
    """One row per loop edge: (i, j, fitness, edge translation, length, path, loop error vector, frontend error
    vector), the vectors in the first submap's frame (None when the edge is not scored)."""
    front = read_tum(front_path)
    q_t = yaml.safe_load(open(ext_path))['T_imu_prism_quat_xyzw_xyz']
    lever = matrix(q_t[4:], q_t[:4])[:3, 3]
    ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{mission}/prism_position', mode='r')
    rt = ref['timestamp'][:].astype(float)
    rp = ref['point'][:]
    Rf = np.stack([rot(r[4:8]) for r in front])
    prism = front[:, 1:4] + np.einsum('nij,j->ni', Rf, lever)
    k = (rt > front[0, 0]) & (rt < front[-1, 0])
    est = np.column_stack([np.interp(rt[k], front[:, 0], prism[:, c]) for c in range(3)])
    R, t = fit(est, rp[k])
    good = np.linalg.norm(est @ R.T + t - rp[k], axis=1) < 1.0
    R, t = fit(est[good], rp[k][good])
    ref_in_front = (rp - t) @ R  # R^T (p - t) for each row

    def at(ts):
        i = int(np.clip(np.searchsorted(front[:, 0], ts), 1, len(front) - 1))
        a, b = front[i - 1], front[i]
        u = 0.0 if b[0] == a[0] else (ts - a[0]) / (b[0] - a[0])
        p = a[1:4] + u * (b[1:4] - a[1:4])
        return p, rot(a[4:8] if u < 0.5 else b[4:8])

    def gt_imu(ts, R_imu):
        # Linear between the two reference samples around ts (the prism moves 2-4 cm between samples at walking
        # speed); none if they are more than 0.2 s apart.
        n = int(np.searchsorted(rt, ts))
        if n == 0 or n == len(rt) or rt[n] - rt[n - 1] > 0.2:
            return None
        u = (ts - rt[n - 1]) / (rt[n] - rt[n - 1])
        return ref_in_front[n - 1] + u * (ref_in_front[n] - ref_in_front[n - 1]) - R_imu @ lever

    stamps = read_tum(f'{backend}/trajectory_raw.tum')[:, 0]
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(front[:, 1:4], axis=0), axis=1))])
    edges = np.loadtxt(f'{backend}/loop_edges.csv', delimiter=',', skiprows=1, ndmin=2)
    for e in edges:
        i, j = int(e[0]), int(e[1])
        pi, Ri = at(stamps[i])
        pj, Rj = at(stamps[j])
        gi, gj = gt_imu(stamps[i], Ri), gt_imu(stamps[j], Rj)
        path = abs(np.interp(stamps[j], front[:, 0], cum) - np.interp(stamps[i], front[:, 0], cum))
        if gi is None or gj is None:
            yield i, j, e[2], e[3:6], np.linalg.norm(e[3:6]), path, None, None
            continue
        G = Ri.T @ (gj - gi)
        F = Ri.T @ (pj - pi)
        yield i, j, e[2], e[3:6], np.linalg.norm(e[3:6]), path, e[3:6] - G, F - G


def main():
    label, mission, backend, front_path, ext_path = sys.argv[1:6]
    for i, j, fitness, _, length, path, loop_err, front_err in edge_errors(mission, backend, front_path, ext_path):
        if loop_err is None:
            print(f'{label},{mission},{i},{j},{fitness:.4f},{length:.2f},{path:.1f},nan,nan')
            continue
        print(f'{label},{mission},{i},{j},{fitness:.4f},{length:.2f},{path:.1f},'
              f'{np.linalg.norm(loop_err):.4f},{np.linalg.norm(front_err):.4f}')


if __name__ == '__main__':
    main()
