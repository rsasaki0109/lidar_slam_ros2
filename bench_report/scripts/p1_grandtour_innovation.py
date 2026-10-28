#!/usr/bin/env python3
"""
Per-scan registration innovation (chapter 36): how far the registration moved each scan from the IMU prediction it
started from (registered - predicted position, world frame), from output_filter_log runs with rko_lio 3403690 or later
('U' events: t, registered position, 36 Hessian values, predicted position). GT-free: reads only the run's own log.
Prints per mission the 99 %, 99.9 % and maximum of |dz| and |dxy| (mm) and the times (s from the first update) of the
largest |dz|.
Usage: p1_grandtour_innovation.py <label> <mission> ...   (runs at /home/user/bench/grandtour/<label>/<mission>/)
"""
import sys

import numpy as np


def innovations(path):
    rows = []
    for line in open(path):
        if line[0] != 'U':
            continue
        f = line.split()
        if len(f) < 44:
            sys.exit(f'{path}: no predicted positions (log from rko_lio before 3403690)')
        rows.append([float(f[1])] + [float(v) for v in f[2:5]] + [float(v) for v in f[41:44]])
    a = np.array(rows)
    return a[:, 0], a[:, 1:4] - a[:, 4:7]


def main():
    label = sys.argv[1]
    print('mission   scans   |dz| mm: 99%   99.9%    max  | |dxy| mm: 99%  99.9%    max  | largest |dz| (t s: mm)')
    for m in sys.argv[2:]:
        t, d = innovations(f'/home/user/bench/grandtour/{label}/{m}/run/front_0/output_filter_events.txt')
        dz = np.abs(d[:, 2]) * 1000
        dxy = np.linalg.norm(d[:, :2], axis=1) * 1000
        top = np.argsort(-dz)[:5]
        print(f'{m:8s} {len(t):6d}   {np.percentile(dz, 99):6.1f} {np.percentile(dz, 99.9):6.1f} {dz.max():6.1f}  |'
              f'  {np.percentile(dxy, 99):6.1f} {np.percentile(dxy, 99.9):6.1f} {dxy.max():6.1f}  | '
              + ' '.join(f'{t[i] - t[0]:.1f}:{d[i, 2] * 1000:+.0f}' for i in top))


if __name__ == '__main__':
    main()
