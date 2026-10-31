"""p1_grandtour_common_error.py (chapter 35): split HS-G15c's position error on the GrandTour development missions
into the part all three LiDAR-inertial methods share (the mean of the three SE(3)-aligned residuals of HS-G15c,
BIEVR-LIO and FAST-LIO2: same Hesai + STIM320 input, same reference times) and the rest. With --offset, fit instead a
constant offset in each output's own frame (a lever arm) jointly with the SE(3) alignment. Development missions only;
the trajectories are the runs of decision_rule_G15c.txt and rivals/."""
import argparse
import os
import sys
import numpy as np
import zarr
from scipy.spatial.transform import Rotation as Rot, Slerp
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_ate import align_se3
B = '/home/user/bench/grandtour'
g15c = {m: 'hsf_of' for m in ['arc-1', 'eig-2', 'heap-1', 'pil-2']}
g15c.update({m: 'hsg_of2' for m in ['arc-4', 'arc-5', 'arc-6', 'ice-1', 'snow-3']})
g15c.update({m: 'G15c_g15c' for m in ['con-1', 'con-2', 'con-3', 'spx-1', 'spx-3']})
def resid(path, rt, rp):
    est = np.loadtxt(path)
    ep = np.stack([np.interp(rt, est[:, 0], est[:, i]) for i in (1, 2, 3)], 1)
    Ra, ta = align_se3(ep, rp)
    return ep @ Ra.T + ta - rp
def fit(est, short):
    ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{short}/prism_position', mode='r')
    rt0 = ref['timestamp'][:].astype(float); rp0 = ref['point'][:]
    k = (rt0 >= est[0, 0]) & (rt0 <= est[-1, 0]); rt, rp = rt0[k], rp0[k]
    ep = np.stack([np.interp(rt, est[:, 0], est[:, i]) for i in (1, 2, 3)], 1)
    _, uniq = np.unique(est[:, 0], return_index=True); e = est[uniq]
    R = Slerp(e[:, 0], Rot.from_quat(e[:, 4:8]))(rt).as_matrix()
    ate0 = None; d = np.zeros(3)
    for it in range(10):
        p = ep + np.einsum('nij,j->ni', R, d)
        Ra, ta = align_se3(p, rp)
        r = p @ Ra.T + ta - rp
        n = np.linalg.norm(r, axis=1); keep = n < max(10 * np.median(n), 0.5)
        if ate0 is None: ate0 = np.sqrt((n[keep] ** 2).mean())
        # residual linear in d: Ra R d + (Ra ep + ta - rp)
        A = np.einsum('ij,njk->nik', Ra, R)[keep].reshape(-1, 3)
        b = -(ep @ Ra.T + ta - rp)[keep].reshape(-1)
        d, *_ = np.linalg.lstsq(A, b, rcond=None)
    p = ep + np.einsum('nij,j->ni', R, d); Ra, ta = align_se3(p, rp)
    n = np.linalg.norm(p @ Ra.T + ta - rp, axis=1); keep = n < max(10 * np.median(n), 0.5)
    return d, ate0, np.sqrt((n[keep] ** 2).mean())

parser = argparse.ArgumentParser()
parser.add_argument('--offset', action='store_true')
if parser.parse_args().offset:
    print('mission   method   offset in the TUM frame (mm)   ATE before / after (mm)')
    for m, lab in g15c.items():
        for name, path in (('G15c', f'{B}/{lab}/{m}/{m}.tum'), ('BIEVR', f'{B}/rival_bievr_hs/{m}/{m}.tum'), ('FAST', f'{B}/rival_fastlio_hs/{m}/{m}.tum')):
            if not os.path.exists(path) or m == 'arc-3' and name != 'G15c': continue
            d, a0, a1 = fit(np.loadtxt(path), m)
            print(f'{m:8s} {name:6s} {d[0]*1000:7.1f} {d[1]*1000:7.1f} {d[2]*1000:7.1f}    {a0*1000:5.1f} / {a1*1000:5.1f}')
else:
    print('mission  ATE G15c | rms of (G15c - BIEVR) | rms of (G15c - FAST) | common part (mean of 3) | G15c minus common  (mm; xy / z)')
    for m, lab in g15c.items():
        ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{m}/prism_position', mode='r')
        rt0 = ref['timestamp'][:].astype(float); rp0 = ref['point'][:]
        paths = [f'{B}/{lab}/{m}/{m}.tum', f'{B}/rival_bievr_hs/{m}/{m}.tum', f'{B}/rival_fastlio_hs/{m}/{m}.tum']
        ests = [np.loadtxt(p) for p in paths]
        t0 = max(e[0, 0] for e in ests); t1 = min(e[-1, 0] for e in ests)
        k = (rt0 >= t0) & (rt0 <= t1); rt, rp = rt0[k], rp0[k]
        rs = [resid(p, rt, rp) for p in paths]
        n = np.linalg.norm(rs[0], axis=1); keep = n < max(10 * np.median(n), 0.5)
        for i in range(3):
            n = np.linalg.norm(rs[i], axis=1); keep &= n < max(10 * np.median(n), 0.5)
        rs = [r[keep] for r in rs]
        def rms(x): return f'{np.sqrt((x[:, :2]**2).sum(1).mean())*1000:5.1f}/{np.sqrt((x[:, 2]**2).mean())*1000:4.1f}'
        common = (rs[0] + rs[1] + rs[2]) / 3
        print(f'{m:8s} {rms(rs[0])} | {rms(rs[0]-rs[1])} | {rms(rs[0]-rs[2])} | {rms(common)} | {rms(rs[0]-common)}')
