#!/usr/bin/env python3
"""p1_grandtour_scale_lag.py (chapter 36): per mission, fit each method's SE(3)-aligned horizontal error as
s * (p - centroid) + tau * velocity + const (s a scale, tau a time lag; development references only) for HS-G15g
(the logged G15f runs fsw_g15flog with the G15g filter are G15g's registration; here the G15f outputs), FAST-LIO2 and
BIEVR-LIO, and print s (ppm), tau (ms) and the ATE before and after removing both."""
import sys
import numpy as np, zarr
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_ate import align_se3
B = '/home/user/bench/grandtour'
G = {m: 'fsw_g15flog' for m in ['heap-1', 'pil-2', 'con-1', 'eig-2', 'arc-3', 'snow-3', 'arc-5', 'arc-4', 'spx-1', 'arc-1', 'con-2', 'con-3', 'ice-1', 'arc-6', 'spx-3']}
print('mission   vs ref: scale(ppm) lag(ms) ATE->(mm)  ours | FAST | BIEVR')
for m, lab in G.items():
    ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{m}/prism_position', mode='r')
    rt0 = ref['timestamp'][:].astype(float); rp0 = ref['point'][:]
    row = []
    for path in (f'{B}/{lab}/{m}/{m}.tum', f'{B}/rival_fastlio_hs/{m}/{m}.tum', f'{B}/rival_bievr_hs/{m}/{m}.tum'):
        try:
            e = np.loadtxt(path)
        except OSError:
            row.append('   -'); continue
        k = (rt0 >= e[0, 0] + 1) & (rt0 <= e[-1, 0] - 1); rt, rp = rt0[k], rp0[k]
        gp = np.linalg.norm(rp[1:-1] - rp[:-2], axis=1); gn = np.linalg.norm(rp[1:-1] - rp[2:], axis=1)
        sp = np.zeros(len(rp), bool); sp[1:-1] = (gp > 1) & (gn > 1)
        rt, rp = rt[~sp], rp[~sp]
        p = np.stack([np.interp(rt, e[:, 0], e[:, i]) for i in (1, 2, 3)], 1)
        v = np.stack([np.interp(rt, e[:, 0], np.gradient(e[:, i], e[:, 0])) for i in (1, 2, 3)], 1)
        R, t = align_se3(p, rp); pa = p @ R.T + t; va = v @ R.T
        err = pa - rp
        if np.abs(err).max() > 5: row.append('  fail'); continue
        c = pa.mean(0)
        # err ~ s * (pa - c) + tau * va + const  (horizontal), least squares
        A = np.column_stack([(pa - c)[:, :2].reshape(-1), va[:, :2].reshape(-1),
                             np.tile([1, 0], len(pa)), np.tile([0, 1], len(pa))])
        x, *_ = np.linalg.lstsq(A, err[:, :2].reshape(-1), rcond=None)
        res = err[:, :2].reshape(-1) - A @ x
        before = np.sqrt((np.linalg.norm(err, axis=1) ** 2).mean()) * 1000
        after = np.sqrt((res.reshape(-1, 2) ** 2).sum(1).mean() + (err[:, 2] ** 2).mean()) * 1000
        row.append(f'{x[0]*1e6:+6.0f} {x[1]*1000:+5.0f} {before:5.1f}->{after:5.1f}')
    print(f'{m:8s} ' + ' | '.join(row), flush=True)
