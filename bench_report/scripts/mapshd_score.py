#!/usr/bin/env python3
"""Score one MapsHD pipeline run like the benchmark (evo_ape tum <gt> <est> -a) and with ape_from_tum.py.

The corrected trajectory (trajectory_optimized.tum) is first densified onto the frontend's full-rate trajectory
(front_0/front_tum_0.txt) with scripts/densify_corrected_trajectory.py, as in chapters 13 and 14. Both are in
RKO-LIO's base = IMU frame; they are scored in the LiDAR (livox) frame, T_world_lidar = T_world_imu * T_imu_lidar
with the MID-360 extrinsic of the frontend profile (identity rotation, translation [-0.011, -0.02329, 0.04412] m),
and, for reference, as they are (IMU frame).
Prints one CSV row per trajectory and frame:
  run,trajectory,frame,poses,evo_pairs,evo_rmse,evo_mean,evo_median,evo_max,interp_rmse
Usage: mapshd_score.py <run_dir> <gt_tum> <out_dir>
"""
import os
import re
import subprocess
import sys

import numpy as np

REPO = '/home/user/lidar_slam_ros2'
EVO_APE = '/home/user/mapshd/venv/bin/evo_ape'
T_IMU_LIDAR = np.array([-0.011, -0.02329, 0.04412])
run, gt, out = sys.argv[1:4]
os.makedirs(out, exist_ok=True)


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_lidar(src, dst):
    rows = [list(map(float, l.split())) for l in open(src) if l.strip() and not l.startswith('#')]
    with open(dst, 'w') as f:
        for r in rows:
            p = np.array(r[1:4]) + quat_to_R(np.array(r[4:8])) @ T_IMU_LIDAR  # rotation is identity
            f.write(f'{r[0]:.9f} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {r[4]:.9f} {r[5]:.9f} {r[6]:.9f} {r[7]:.9f}\n')
    return len(rows)


def evo(est):
    env = dict(os.environ, MPLBACKEND='Agg')
    r = subprocess.run([EVO_APE, 'tum', gt, est, '-a'], capture_output=True, text=True, env=env)
    stats = {k: float(v) for k, v in re.findall(r'^\s*(rmse|mean|median|max|sse)\s+([0-9.eE+-]+)\s*$', r.stdout, re.M)}
    open(est + '.evo.txt', 'w').write(r.stdout + r.stderr)
    # evo prints no pair count without -v; sse / rmse^2 is that count.
    pairs = round(stats['sse'] / stats['rmse'] ** 2) if stats.get('rmse') else -1
    return (pairs, *(stats.get(k, float('nan')) for k in ('rmse', 'mean', 'median', 'max')))


def interp(est):
    report = est + '.interp.txt'
    subprocess.run(['python3', f'{REPO}/scripts/ape_from_tum.py', '--ref', gt, '--est', est, '--out', report,
                    '--interpolate'], check=True, capture_output=True)
    return float(next(l.split()[1] for l in open(report) if l.startswith('rmse:')))


front = os.path.join(run, 'front_0', 'front_tum_0.txt')
dense = os.path.join(out, 'pipeline_imu.tum')
subprocess.run(['python3', f'{REPO}/scripts/densify_corrected_trajectory.py', '--raw', front, '--corrected',
                os.path.join(run, 'trajectory_optimized.tum'), '--output', dense], check=True, capture_output=True,
               env=dict(os.environ, PYTHONPATH=REPO))
name = os.path.basename(run.rstrip('/'))
for kind, imu_path in (('frontend', front), ('pipeline', dense)):
    lidar_path = os.path.join(out, f'{kind}_lidar.tum')
    poses = to_lidar(imu_path, lidar_path)
    for frame, path in (('lidar', lidar_path), ('imu', imu_path)):
        pairs, rmse, mean, median, mx = evo(path)
        print(f'{name},{kind},{frame},{poses},{pairs},{rmse:.6f},{mean:.6f},{median:.6f},{mx:.6f},{interp(path):.6f}',
              flush=True)
