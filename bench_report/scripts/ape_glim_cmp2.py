#!/usr/bin/env python3
"""APE of the glim_vs_ours2.sh runs, one method for every system.

IMU (= RKO-LIO base) frame trajectories, no lever-arm offset, scripts/ape_from_tum.py --interpolate
(SE(3) Umeyama), against the exp04 surveyed checkpoints: the 6 common ones (the first excluded, as in
docs/comparison.md, because GLIM's trajectory starts after it) and all 7.
  glim        : dump/traj_imu.txt (global mapping result) and dump/odom_imu.txt (odometry only)
  ours_readme : front_0/front_tum_0.txt (frontend; loop closure off, so it is also the final result)
  ours_loop   : trajectory_optimized.tum (graph anchors after /map_save) densified onto
                front_0/front_tum_0.txt with scripts/densify_corrected_trajectory.py, and the raw frontend
Usage: ape_glim_cmp2.py <glim_vs_ours2.csv> <out.csv> [result_column (13; 10 for glim_vs_ours3.csv)]
Any other ours system (ours_off) is scored like ours_loop.
Output columns: system,round,trajectory,poses,md5,ape6_rmse,ape6_pairs,ape7_rmse,ape7_pairs
"""
import csv
import hashlib
import os
import subprocess
import sys

REPO = '/home/user/lidar_slam_ros2'
GT = '/home/user/data/hilti2022/exp04_construction_upper_level_gt.txt'


def ape(ref, est, scratch):
    out = os.path.join(scratch, 'ape.txt')
    if os.path.exists(out):
        os.remove(out)
    subprocess.run(['python3', f'{REPO}/scripts/ape_from_tum.py', '--ref', ref, '--est', est, '--out', out,
                    '--interpolate'], check=False, capture_output=True)
    values = {}
    if os.path.exists(out):
        for line in open(out):
            if ':' in line:
                key, value = line.split(':', 1)
                values[key.strip()] = value.strip()
    return values.get('rmse', 'NA'), values.get('pairs', 'NA')


def poses(path):
    return sum(1 for line in open(path) if line.strip() and not line.startswith('#'))


def md5(path):
    return hashlib.md5(open(path, 'rb').read()).hexdigest()


def main():
    src, dst = sys.argv[1], sys.argv[2]
    result_col = int(sys.argv[3]) if len(sys.argv) > 3 else 13
    scratch = os.path.join(os.path.dirname(os.path.abspath(dst)), '.ape_scratch')
    os.makedirs(scratch, exist_ok=True)
    lines = [line for line in open(GT) if line.strip() and not line.startswith('#')]
    gt7, gt6 = os.path.join(scratch, 'gt7.tum'), os.path.join(scratch, 'gt6.tum')
    open(gt7, 'w').writelines(lines)
    open(gt6, 'w').writelines(lines[1:])
    env = dict(os.environ, PYTHONPATH=REPO)
    rows = []
    for r in csv.reader(open(src)):
        system, rnd, result = r[0], r[1], r[result_col]
        run_dir = os.path.dirname(result)
        if system == 'glim':
            estimates = [('traj_imu', result), ('odom_imu', os.path.join(run_dir, 'odom_imu.txt'))]
        elif system == 'ours_readme':
            estimates = [('frontend', result)]
        else:
            raw = os.path.join(run_dir, 'front_0', 'front_tum_0.txt')
            dense = os.path.join(run_dir, 'traj_corrected_dense.tum')
            subprocess.run(['python3', f'{REPO}/scripts/densify_corrected_trajectory.py', '--raw', raw,
                            '--corrected', result, '--output', dense], check=True, capture_output=True, env=env)
            estimates = [('corrected_dense', dense), ('frontend', raw)]
        for kind, est in estimates:
            a6, p6 = ape(gt6, est, scratch)
            a7, p7 = ape(gt7, est, scratch)
            rows.append([system, rnd, kind, poses(est), md5(est), a6, p6, a7, p7])
    with open(dst, 'w') as f:
        f.write('system,round,trajectory,poses,md5,ape6_rmse,ape6_pairs,ape7_rmse,ape7_pairs\n')
        for row in rows:
            f.write(','.join(str(v) for v in row) + '\n')
    for row in rows:
        print(','.join(str(v) for v in row))


if __name__ == '__main__':
    main()
