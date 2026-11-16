#!/usr/bin/env python3
"""ATE of the Odyssey diagnosis runs (bench_report/data/odyssey_dev_2026-11-13/plan.txt), scored as in H9.

usage: p9_odyssey_dev_score.py DEV_DIR GT_DIR  ->  one line per <seq>/<variant>/traj_imu.tum: ATE (m), wall time, RSS
"""
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p8_blind_score_h9 import ate  # noqa: E402


def main():
    dev, gt = sys.argv[1:]
    for seq in sorted(os.listdir(dev)):
        if not os.path.exists(os.path.join(gt, seq, 'lidar_stamps.txt')):
            continue
        stamps = np.loadtxt(os.path.join(gt, seq, 'lidar_stamps.txt'), dtype=np.int64)
        ref_p = np.loadtxt(os.path.join(gt, seq, 'lidar_poses_ref.txt'), ndmin=2)[:, [3, 7, 11]]
        ref_t = (stamps - stamps[0]) * 1e-9
        for v in sorted(os.listdir(os.path.join(dev, seq))):
            traj = os.path.join(dev, seq, v, 'traj_imu.tum')
            if not os.path.exists(traj):
                continue
            est = np.loadtxt(traj, ndmin=2)
            sec = np.array([float(x) - stamps[0] * 1e-9 for x in est[:, 0]])
            info = open(os.path.join(dev, seq, v, 'run_info.txt')).read()
            rss = int(re.search(r"Maximum resident set size \(kbytes\): (\d+)", open(os.path.join(dev, seq, v, 'time.txt')).read()).group(1))
            print(f"{seq} {v:12s} ATE {ate(sec, est[:, 1:4], ref_t, ref_p):.4f} m  wall "
                  f"{re.search(r'wall_seconds ([0-9.]+)', info).group(1)} s  RSS {rss / 1024:.0f} MB  poses {len(est)}")


if __name__ == '__main__':
    main()
