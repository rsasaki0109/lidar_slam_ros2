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
Usage: p1_grandtour_output_filter_sweep.py [--csv] <label> <setting> [<setting> ...] -- <mission> ...
  setting: sigma,q_a,q_b (rko_lio's filter) or mode,sigma,q_a,q_b,alpha,lo,hi (output_filter_replay_adaptive.cpp)
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
SRC_ADAPTIVE = '/home/user/lidar_slam_ros2/bench_report/tools/output_filter_replay_adaptive.cpp'
BIN_ADAPTIVE = os.path.join(tempfile.gettempdir(), 'output_filter_replay_adaptive')


def build():
    for src, binary in ((SRC, BIN), (SRC_ADAPTIVE, BIN_ADAPTIVE)):
        if not os.path.exists(binary) or os.path.getmtime(binary) < os.path.getmtime(src):
            subprocess.run(['g++', '-O2', '-std=c++17', '-I/usr/include/eigen3', '-I/home/user/rko_lio/rko_lio/core',
                            src, '-o', binary], check=True)


def replay(events, setting):
    """setting: (sigma, q_a, q_b) for rko_lio's filter, or 'mode,sigma,q_a,q_b,alpha,lo,hi' for
    tools/output_filter_replay_adaptive.cpp."""
    args = [BIN_ADAPTIVE, events] + setting.split(',') if isinstance(setting, str) else \
        [BIN, events] + [str(v) for v in setting]
    out = subprocess.run(args, check=True, capture_output=True, text=True).stdout
    return np.array([[float(v) for v in line.split()] for line in out.splitlines()])


def score(label, short, settings):
    """[(ATE, <1 s, 1-10 s, slow) in mm] for each setting on one mission; the first setting is checked against the
    run's output."""
    run = f'{B}/{label}/{short}'
    front = np.loadtxt(f'{run}/run/front_0/front_tum_0.txt')
    events = f'{run}/run/front_0/output_filter_events.txt'
    q_t = yaml.safe_load(open(f'{run}/extrinsics.txt'))['T_imu_prism_quat_xyzw_xyz']
    T_imu_prism = matrix(q_t[4:], q_t[:4])
    ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{short}/prism_position', mode='r')
    rt = ref['timestamp'][:].astype(float)
    rp = ref['point'][:]
    R = np.stack([matrix([0.0, 0.0, 0.0], r[4:8])[:3, :3] for r in front])
    lever = np.einsum('nij,j->ni', R, T_imu_prism[:3, 3])
    results = []
    for k, setting in enumerate(settings):
        positions = replay(events, setting)
        if len(positions) != len(front):
            sys.exit(f'{short}: {len(positions)} replayed positions for {len(front)} poses')
        if k == 0:
            gap = np.abs(positions[:, 1:4] - front[:, 1:4]).max()
            if gap > 1.5e-6:
                sys.exit(f'{short}: the replay of the run\'s own setting differs from its output by {gap:.2e} m')
        est = np.column_stack([front[:, 0], positions[:, 1:4] + lever])
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, f'{short}.tum')
            np.savetxt(path, np.column_stack([est, np.tile([0.0, 0.0, 0.0, 1.0], (len(est), 1))]), fmt='%.9f')
            b = bands(path, short)
        results.append((ate(est, rt, rp, 0.0) * 1000, b[0], b[2], b[4]))
    return results


def main():
    args = sys.argv[1:]
    csv = '--csv' in args
    args = [a for a in args if a != '--csv']
    label = args[0]
    split = args.index('--')
    settings = [s if s[0].isalpha() else tuple(float(v) for v in s.split(',')) for s in args[1:split]]
    missions = args[split + 1:]
    build()
    def name(s):
        return s.replace(',', ' ') if isinstance(s, str) else f'{s[0]:g} {s[1]:g} {s[2]:g}'

    if csv:
        print('mission,setting,ate_mm,below_1s_mm,1_10s_mm,slow_mm')
    else:
        print('mission  ' + ' | '.join(f'{name(s)}: ATE <1s 1-10s slow' for s in settings))
    for short in missions:
        results = score(label, short, settings)
        if csv:
            for s, r in zip(settings, results):
                print(f'{short},{name(s)},' + ','.join(f'{v:.3f}' for v in r), flush=True)
        else:
            print(f'{short:8s} ' + ' | '.join(f'{r[0]:5.1f} {r[1]:4.1f} {r[2]:4.1f} {r[3]:5.1f}' for r in results),
                  flush=True)


if __name__ == '__main__':
    main()
