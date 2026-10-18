#!/usr/bin/env python3
"""Ground-truth relative pose (IMU frame) of every loop edge of a run, for graph_replay.py meas=<csv>.
T_imu_gt maps the GT frame into the IMU frame (the GT pose times T_gt_imu is the IMU pose).
Usage: loop_gt_measurements.py <run_dir> <gt_tum> <T_imu_gt.npy> <out_csv>   (rows: from,to,tx,ty,tz,qx,qy,qz,qw)"""
import importlib.util
import sys

import numpy as np

spec = importlib.util.spec_from_file_location(
    'lee', '/home/user/lidar_slam_ros2/bench_report/scripts/loop_edge_errors.py')
run, gt_path, T_path, out = sys.argv[1:5]
src = open(spec.origin).read()
src = src[:src.index("label, run, gt_path, T_path = sys.argv[1:5]")]
ns = {}
exec(compile(src, spec.origin, 'exec'), ns)


def R_to_quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


stamps = ns['read_tum'](f'{run}/trajectory_optimized.tum')[:, 0]
gt = ns['read_tum'](gt_path)
T_gt_imu = np.linalg.inv(np.load(T_path))
edges = np.loadtxt(f'{run}/loop_edges.csv', delimiter=',', skiprows=1, ndmin=2)
with open(out, 'w') as f:
    for e in edges:
        i, j = int(e[0]), int(e[1])
        G = np.linalg.inv(ns['at'](gt, stamps[i]) @ T_gt_imu) @ (ns['at'](gt, stamps[j]) @ T_gt_imu)
        f.write('%d,%d,%s\n' % (i, j, ','.join('%.12g' % v for v in [*G[:3, 3], *R_to_quat(G[:3, :3])])))
