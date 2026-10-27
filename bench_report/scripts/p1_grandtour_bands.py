#!/usr/bin/env python3
"""
Split the ATE residual of GrandTour trajectories into three bands (chapter 34).

For development missions only (it reads the prism reference). The estimate is aligned to the reference as in
p1_grandtour_ate.py (SE(3), no scale), the residual is resampled at 10 Hz, and split into the part below 1 s (residual
minus its centred 1-s moving average), 1-10 s (the 1-s average minus the 10-s average) and the slow part (the 10-s
average). Prints the RMS of each in mm, with the vertical component in brackets for the two faster bands; only grid
points with a reference sample within 0.1 s count.
Usage: p1_grandtour_bands.py <label,label,...> <mission> ...
  (trajectories at /home/user/bench/grandtour/<label>/<mission>/<mission>.tum)
"""
import sys

import numpy as np
from scipy.ndimage import uniform_filter1d

import zarr

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_ate import align_se3  # noqa: E402


def bands(path, short):
    """Return RMS (mm) of the <1 s band, its z, the 1-10 s band, its z, and the slow band."""
    est = np.loadtxt(path)
    ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{short}/prism_position', mode='r')
    rt = ref['timestamp'][:].astype(float)
    rp = ref['point'][:]
    k = (rt >= est[0, 0]) & (rt <= est[-1, 0])
    rt, rp = rt[k], rp[k]
    ep = np.stack([np.interp(rt, est[:, 0], est[:, i]) for i in (1, 2, 3)], 1)
    R, t = align_se3(ep, rp)
    r = ep @ R.T + t - rp
    tg = np.arange(rt[0], rt[-1], 0.1)
    near = np.abs(rt[np.clip(np.searchsorted(rt, tg), 0, len(rt) - 1)] - tg) < 0.1
    rg = np.stack([np.interp(tg, rt, r[:, i]) for i in range(3)], 1)
    m1 = uniform_filter1d(rg, 10, axis=0, mode='nearest')
    m10 = uniform_filter1d(rg, 100, axis=0, mode='nearest')

    def rms(x):
        return np.sqrt((np.linalg.norm(x[near], axis=1) ** 2).mean()) * 1000

    def rmsz(x):
        return np.sqrt((x[near, 2] ** 2).mean()) * 1000

    return rms(rg - m1), rmsz(rg - m1), rms(m1 - m10), rmsz(m1 - m10), rms(m10)


def main():
    labels = sys.argv[1].split(',')
    print('mission | ' + ' | '.join(f'{lab[:12]:>12s} <1s(z) 1-10s(z) slow' for lab in labels))
    for short in sys.argv[2:]:
        cells = []
        for lab in labels:
            v = bands(f'/home/user/bench/grandtour/{lab}/{short}/{short}.tum', short)
            cells.append(f'{v[0]:5.1f}({v[1]:4.1f}) {v[2]:5.1f}({v[3]:4.1f}) {v[4]:5.1f}')
        print(f'{short:7s} | ' + ' | '.join(cells), flush=True)


if __name__ == '__main__':
    main()
