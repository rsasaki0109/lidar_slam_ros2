#!/usr/bin/env python3
"""APE of pipe_e2e.sh run directories against the exp04 surveyed checkpoints.

For each run dir: densify trajectory_optimized.tum (graph anchors) onto front_0/front_tum_0.txt with
scripts/densify_corrected_trajectory.py, then scripts/ape_from_tum.py --interpolate against all 7
checkpoints and against the 6 used by the README comparison (the first one excluded).
Usage: ape_runs.py <out.csv> <run_dir>...
"""
import os
import subprocess
import sys

REPO = '/home/user/lidar_slam_ros2'
GT = '/home/user/data/hilti2022/exp04_construction_upper_level_gt.txt'


def rmse(ref, est, scratch):
    out = os.path.join(scratch, 'ape.txt')
    subprocess.run(['python3', f'{REPO}/scripts/ape_from_tum.py', '--ref', ref, '--est', est, '--out', out,
                    '--interpolate'], check=True, capture_output=True)
    for line in open(out):
        if line.startswith('rmse:'):
            return float(line.split()[1])
    return float('nan')


def main():
    out_csv, runs = sys.argv[1], sys.argv[2:]
    scratch = os.path.join(os.path.dirname(os.path.abspath(out_csv)), '.ape_scratch')
    os.makedirs(scratch, exist_ok=True)
    lines = [l for l in open(GT) if l.strip() and not l.startswith('#')]
    gt7, gt6 = os.path.join(scratch, 'gt7.tum'), os.path.join(scratch, 'gt6.tum')
    open(gt7, 'w').writelines(lines)
    open(gt6, 'w').writelines(lines[1:])
    rows = []
    for run in runs:
        raw = os.path.join(run, 'front_0', 'front_tum_0.txt')
        anchors = os.path.join(run, 'trajectory_optimized.tum')
        dense = os.path.join(run, 'traj_corrected_dense.tum')
        env = dict(os.environ, PYTHONPATH=REPO)
        subprocess.run(['python3', f'{REPO}/scripts/densify_corrected_trajectory.py', '--raw', raw,
                        '--corrected', anchors, '--output', dense], check=True, capture_output=True, env=env)
        row = (os.path.basename(run.rstrip('/')), rmse(gt7, raw, scratch), rmse(gt6, raw, scratch),
               rmse(gt7, dense, scratch), rmse(gt6, dense, scratch))
        rows.append(row)
        print('%s,frontend_all7=%.4f,frontend_common6=%.4f,corrected_all7=%.4f,corrected_common6=%.4f' % row)
    with open(out_csv, 'a') as f:
        for row in rows:
            f.write('%s,%.6f,%.6f,%.6f,%.6f\n' % row)


if __name__ == '__main__':
    main()
