#!/usr/bin/env python3
"""p1_grandtour_imu_scale.py (chapter 36): a GT-free scale check that did not work. The horizontal accelerations of the
registered positions (twice differentiated) against the IMU world accelerations of the output filter log
(fsw_g15flog), band-passed, give a_imu / a_lidar - 1 per mission; compared with the Sim(3) scale against the
reference."""
import sys
import numpy as np
from scipy.signal import butter, sosfiltfilt
B = '/home/user/bench/grandtour'
def load(m):
    P, U = [], []
    for line in open(f'{B}/fsw_g15flog/{m}/run/front_0/output_filter_events.txt'):
        f = line.split()
        if f[0] == 'P':
            P.append([float(v) for v in f[1:5]])
        elif f[0] == 'U':
            U.append([float(v) for v in f[1:5]])
    return np.array(P), np.array(U)
def scale(m, lo, hi, fs=10.0):
    P, U = load(m)
    t0, t1 = max(P[0, 0], U[0, 0]) + 5, min(P[-1, 0], U[-1, 0]) - 1
    tg = np.arange(t0, t1, 1 / fs)
    pos = np.stack([np.interp(tg, U[:, 0], U[:, i]) for i in (1, 2, 3)], 1)
    # IMU acceleration averaged into the grid cells
    acc = np.zeros_like(pos)
    idx = np.searchsorted(tg, P[:, 0]) - 1
    ok = (idx >= 0) & (idx < len(tg))
    for i in range(3):
        acc[:, i] = np.bincount(idx[ok], P[ok, i + 1], len(tg)) / np.maximum(np.bincount(idx[ok], None, len(tg)), 1)
    sos = butter(4, [lo, hi], btype='band', fs=fs, output='sos')
    a_l = sosfiltfilt(sos, np.gradient(np.gradient(pos, tg, axis=0), tg, axis=0), axis=0)
    a_i = sosfiltfilt(sos, acc, axis=0)
    # cell averages of acceleration lag half a cell; shift IMU by 0.5/fs via interpolation
    a_i = np.stack([np.interp(tg, tg + 0.5 / fs, a_i[:, i]) for i in range(3)], 1)
    h = slice(0, 2)
    s = (a_i[:, h] * a_l[:, h]).sum() / (a_l[:, h] ** 2).sum()
    corr = (a_i[:, h] * a_l[:, h]).sum() / np.sqrt((a_i[:, h] ** 2).sum() * (a_l[:, h] ** 2).sum())
    return s, corr
sim3 = {'heap-1': -1077, 'eig-2': 554, 'pil-2': 77, 'arc-1': -241, 'snow-3': -357, 'arc-4': -96, 'arc-5': -1025,
        'con-1': 299, 'con-2': 197, 'spx-1': 697}
bands = [(0.1, 0.5), (0.2, 1.0), (0.5, 2.0), (1.0, 3.0)]
print('mission  sim3(ppm)  ' + '  '.join(f'{lo}-{hi}Hz: a_imu/a_lidar-1 (ppm), corr' for lo, hi in bands))
res = {b: [] for b in bands}
for m, s3 in sim3.items():
    row = []
    for b in bands:
        s, c = scale(m, *b)
        res[b].append(((s - 1) * 1e6, s3))
        row.append(f'{(s-1)*1e6:+8.0f} {c:.3f}')
    print(f'{m:8s} {s3:+6d}    ' + '   '.join(row), flush=True)
for b in bands:
    x = np.array(res[b])
    print(b, 'correlation with Sim(3) scale', np.corrcoef(x[:, 0], x[:, 1])[0, 1].round(2))
