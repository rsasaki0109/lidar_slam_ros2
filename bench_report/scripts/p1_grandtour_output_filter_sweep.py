#!/usr/bin/env python3
"""
Replay rko_lio's output position filter with other settings on GrandTour runs (chapter 35).

A run with output_filter_log writes the filter's inputs (output_filter_events.txt: IMU world accelerations,
registered positions, restarts). tools/output_filter_replay.cpp feeds them to rko_lio's own OutputPositionFilter with
each setting (position sigma, acceleration noise, bias noise); the replayed base positions take the run's attitudes,
are moved to the prism as in p1_grandtour_run.sh and scored like p1_grandtour_ate.py (ATE without reference spikes)
with the band split of p1_grandtour_bands.py. The first setting must be the run's own: its replay is checked against
the run's output (to 1 um, the TUM precision) before anything else is reported.
For development missions only (reads the reference).
Usage: p1_grandtour_output_filter_sweep.py <label> <sigma,q_a,q_b> [<sigma,q_a,q_b> ...] -- <mission> ...
  (runs at /home/user/bench/grandtour/<label>/<mission>/)
"""
import os
import subprocess
import sys
import tempfile

import numpy as np
import yaml
import zarr

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_bands import bands  # noqa: E402
from p1_grandtour_time_offset import ate  # noqa: E402
from p1_grandtour_to_ros2 import matrix  # noqa: E402

B = '/home/user/bench/grandtour'
SRC = '/home/user/lidar_slam_ros2/bench_report/tools/output_filter_replay.cpp'
BIN = os.path.join(tempfile.gettempdir(), 'output_filter_replay')


def build():
    if not os.path.exists(BIN) or os.path.getmtime(BIN) < os.path.getmtime(SRC):
        subprocess.run(['g++', '-O2', '-std=c++17', '-I/usr/include/eigen3', '-I/home/user/rko_lio/rko_lio/core', SRC,
                        '-o', BIN], check=True)


def replay(events, setting):
    out = subprocess.run([BIN, events] + [str(v) for v in setting], check=True, capture_output=True, text=True).stdout
    return np.array([[float(v) for v in line.split()] for line in out.splitlines()])


def main():
    args = sys.argv[1:]
    label = args[0]
    split = args.index('--')
    settings = [tuple(float(v) for v in s.split(',')) for s in args[1:split]]
    missions = args[split + 1:]
    build()
    print('mission  ' + ' | '.join(f'{s[0]:g},{s[1]:g},{s[2]:g}: ATE <1s 1-10s slow' for s in settings))
    for short in missions:
        run = f'{B}/{label}/{short}'
        front = np.loadtxt(f'{run}/run/front_0/front_tum_0.txt')
        events = f'{run}/run/front_0/output_filter_events.txt'
        q_t = yaml.safe_load(open(f'{run}/extrinsics.txt'))['T_imu_prism_quat_xyzw_xyz']
        T_imu_prism = matrix(q_t[4:], q_t[:4])
        ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{short}/prism_position', mode='r')
        rt = ref['timestamp'][:].astype(float)
        rp = ref['point'][:]
        row = []
        for k, setting in enumerate(settings):
            positions = replay(events, setting)
            if len(positions) != len(front):
                sys.exit(f'{short}: {len(positions)} replayed positions for {len(front)} poses')
            if k == 0:
                gap = np.abs(positions[:, 1:4] - front[:, 1:4]).max()
                if gap > 1.5e-6:
                    sys.exit(f'{short}: the replay of the run\'s own setting differs from its output by {gap:.2e} m')
            est = np.empty((len(front), 4))
            est[:, 0] = front[:, 0]
            for i, r in enumerate(front):
                est[i, 1:4] = (matrix(positions[i, 1:4], r[4:8]) @ T_imu_prism)[:3, 3]
            with tempfile.TemporaryDirectory() as d:
                path = os.path.join(d, f'{short}.tum')
                np.savetxt(path, np.column_stack([est, np.tile([0.0, 0.0, 0.0, 1.0], (len(est), 1))]), fmt='%.9f')
                b = bands(path, short)
            row.append(f'{ate(est, rt, rp, 0.0) * 1000:5.1f} {b[0]:4.1f} {b[2]:4.1f} {b[4]:5.1f}')
        print(f'{short:8s} ' + ' | '.join(row), flush=True)


if __name__ == '__main__':
    main()
