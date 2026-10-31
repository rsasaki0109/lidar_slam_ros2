#!/usr/bin/env python3
"""
Split the ATE of GrandTour trajectories into a slow and a fast part (chapter 33).

For development missions only (it reads the prism reference). The estimate is aligned to the reference as in
p1_grandtour_ate.py (SE(3), no scale), the residual is resampled at 10 Hz, and its centred 10-s moving average is the
slow part (drift, bends), the rest the fast part (scan-to-scan noise, short slips). Prints the RMS of each and of the
slow part's height.
Usage: p1_grandtour_error_split.py <bench dir> <label,label,...> <mission> ...
  (trajectories at <bench dir>/<label>/<mission>/<mission>.tum, references at
  /home/user/data/grandtour/reference_dev/<mission>/prism_position)
"""
import sys

import numpy as np
from scipy.ndimage import uniform_filter1d
import zarr

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_ate import align_se3  # noqa: E402


def split(est, rt, rp):
    """Return ATE, slow RMS, slow height RMS and fast RMS of one trajectory."""
    k = (rt >= est[0, 0]) & (rt <= est[-1, 0])
    rt, rp = rt[k], rp[k]
    ep = np.stack([np.interp(rt, est[:, 0], est[:, i]) for i in (1, 2, 3)], 1)
    R, t = align_se3(ep, rp)
    r = ep @ R.T + t - rp
    tg = np.arange(rt[0], rt[-1], 0.1)
    rg = np.stack([np.interp(tg, rt, r[:, i]) for i in range(3)], 1)
    slow = uniform_filter1d(rg, 101, axis=0, mode='nearest')
    fast = rg - slow
    # only grid points with a reference sample within 0.1 s count (the prism is lost at times)
    j = np.clip(np.searchsorted(rt, tg), 1, len(rt) - 1)
    seen = np.minimum(np.abs(rt[j] - tg), np.abs(rt[j - 1] - tg)) < 0.1
    slow, fast = slow[seen], fast[seen]
    return (np.sqrt((r ** 2).sum(1).mean()), np.sqrt((slow ** 2).sum(1).mean()),
            np.sqrt((slow[:, 2] ** 2).mean()), np.sqrt((fast ** 2).sum(1).mean()))


def main():
    bench, labels, missions = sys.argv[1], sys.argv[2].split(','), sys.argv[3:]
    print('mission  ' + ''.join(f'| {lab[:30]:30s} ' for lab in labels))
    print('         ' + '| ATE    slow   (z)    fast   ' * len(labels))
    for m in missions:
        ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{m}/prism_position', mode='r')
        rt, rp = ref['timestamp'][:].astype(float), ref['point'][:]
        row = f'{m:8s} '
        for lab in labels:
            try:
                est = np.loadtxt(f'{bench}/{lab}/{m}/{m}.tum')
            except OSError:
                row += '| ' + ' ' * 29 + '- '
                continue
            ate, slow, slow_z, fast = split(est, rt, rp)
            row += f'| {ate:.4f} {slow:.4f} ({slow_z:.4f}) {fast:.4f} '
        print(row)


if __name__ == '__main__':
    main()
