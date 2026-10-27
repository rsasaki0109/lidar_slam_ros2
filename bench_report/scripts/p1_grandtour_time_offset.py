#!/usr/bin/env python3
"""
ATE of GrandTour trajectories with the reference timestamps shifted (chapter 34, reference_time_offset_2026-10-27.txt).

For development missions only. For dt from -40 to +40 ms in 5 ms steps, then 1 ms steps around the best, the estimate
is interpolated at the reference stamps plus dt and scored as p1_grandtour_ate.py's ATE without reference spikes.
A diagnostic of the reference's time chain: the offset found is NOT applied to any submission.
Usage: p1_grandtour_time_offset.py <label> <mission> ...
"""
import sys

import numpy as np

import zarr

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_ate import align_se3  # noqa: E402


def ate(est, rt, rp, dt):
    rt = rt + dt
    inside = (rt >= est[0, 0]) & (rt <= est[-1, 0])
    rt, rp = rt[inside], rp[inside]
    ep = np.stack([np.interp(rt, est[:, 0], est[:, k]) for k in (1, 2, 3)], 1)
    spike = np.zeros(len(rp), dtype=bool)
    gap_prev = np.linalg.norm(rp[1:-1] - rp[:-2], axis=1)
    gap_next = np.linalg.norm(rp[1:-1] - rp[2:], axis=1)
    across = np.linalg.norm(rp[2:] - rp[:-2], axis=1)
    spike[1:-1] = (gap_prev > 1.0) & (gap_next > 1.0) & (across < 0.5 + 2.0 * (rt[2:] - rt[:-2]))
    R, t = align_se3(ep[~spike], rp[~spike])
    return np.sqrt((np.linalg.norm(ep[~spike] @ R.T + t - rp[~spike], axis=1) ** 2).mean())


def main():
    label = sys.argv[1]
    offsets = np.arange(-0.04, 0.0401, 0.005)
    print('mission  best dt (ms)  ATE at best / at 0 (mm)')
    for short in sys.argv[2:]:
        est = np.loadtxt(f'/home/user/bench/grandtour/{label}/{short}/{short}.tum')
        ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{short}/prism_position', mode='r')
        rt = ref['timestamp'][:].astype(float)
        rp = ref['point'][:]
        coarse = [ate(est, rt, rp, o) for o in offsets]
        best = offsets[int(np.argmin(coarse))]
        fine = np.arange(best - 0.005, best + 0.0051, 0.001)
        values = [ate(est, rt, rp, o) for o in fine]
        i = int(np.argmin(values))
        print(f'{short:7s}  {fine[i] * 1000:+4.0f}  {values[i] * 1000:5.1f} / {ate(est, rt, rp, 0.0) * 1000:5.1f}',
              flush=True)


if __name__ == '__main__':
    main()
