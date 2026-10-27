#!/usr/bin/env python3
"""
Scan-to-scan jitter of a trajectory, without a reference (chapter 33).

The RMS of the position minus its centred 1-s moving average, after resampling at 10 Hz, horizontally and vertically.
A legged robot sways, so this is not zero for a perfect estimate; it compares methods on the same mission.
Usage: p1_grandtour_jitter.py <trajectory.tum> ...
"""
import sys

import numpy as np
from scipy.ndimage import uniform_filter1d


def jitter(path):
    a = np.loadtxt(path)
    t = np.arange(a[0, 0], a[-1, 0], 0.1)
    p = np.stack([np.interp(t, a[:, 0], a[:, k]) for k in (1, 2, 3)], 1)
    hf = p - uniform_filter1d(p, 11, axis=0, mode='nearest')
    return np.sqrt((hf[:, :2] ** 2).sum(1).mean()), np.sqrt((hf[:, 2] ** 2).mean())


if __name__ == '__main__':
    for path in sys.argv[1:]:
        h, v = jitter(path)
        print(f'jitter_xy {h * 1000:.1f} mm, jitter_z {v * 1000:.1f} mm  {path}')
