#!/usr/bin/env python3
"""Position difference between the dense corrected trajectories of two pipe_e2e.sh runs that share the
frontend trajectory (same timestamps): densify each trajectory_optimized.tum onto front_0/front_tum_0.txt and
report the RMS and maximum position difference. Usage: traj_diff.py <reference_run> <run>..."""
import os
import subprocess
import sys

import numpy as np

REPO = '/home/user/lidar_slam_ros2'


def dense(run):
    out = os.path.join(run, 'traj_corrected_dense.tum')
    if not os.path.exists(out):
        subprocess.run(['python3', f'{REPO}/scripts/densify_corrected_trajectory.py', '--raw',
                        os.path.join(run, 'front_0', 'front_tum_0.txt'), '--corrected',
                        os.path.join(run, 'trajectory_optimized.tum'), '--output', out],
                       check=True, capture_output=True, env=dict(os.environ, PYTHONPATH=REPO))
    data = np.loadtxt(out, comments='#')
    return data[:, 0], data[:, 1:4]


ref_t, ref_p = dense(sys.argv[1])
for run in sys.argv[2:]:
    t, p = dense(run)
    assert np.array_equal(t, ref_t), 'runs do not share the frontend timestamps'
    d = np.linalg.norm(p - ref_p, axis=1)
    print(f'{os.path.basename(run.rstrip("/"))}: rms {np.sqrt(np.mean(d * d)):.3f} m, max {d.max():.3f} m, '
          f'poses {len(t)}')
