#!/usr/bin/env python3
"""Break the KITTI 05 relative error of a trajectory down (report chapter 48).

Usage: p2_kitti_error_breakdown.py <est.tum>   (the base-frame TUM trajectory scored by p2_kitti_score.py)

Both trajectories are put in the Velodyne frame as in p2_kitti_score.py. For windows of 100/200/400/800 m along the
ground-truth path (a new window every 10 m) it prints the relative translation error split into along-track,
lateral and vertical parts (in the window's first ground-truth frame, % of the length; signed means and RMS), the
rotation error per metre (yaw, pitch, roll), the 200 m windows by turning, speed and position, then the constant
rotation C that best maps the ground-truth relative translations onto the estimate's (a frame offset: the same
angle at every length) and the error left after removing it. Development data only (reads the ground truth).
"""
import os
import sys
import numpy as np
from scipy.spatial.transform import Rotation
# The LiDAR-to-base extrinsic the trajectory was made with (qx qy qz qw x y z); EXT in the environment overrides it.
EXT = ([float(v) for v in os.environ['EXT'].split(',')] if os.environ.get('EXT') else
       [-0.007412088, 0.001015086, 0.000385188, 0.999971941, 0.810543972, -0.307054372, 0.802723995])
est_path = sys.argv[1]
GT = '/home/user/data/kitti_raw_05/gt'
def se3(q, t):
    T = np.eye(4); T[:3, :3] = Rotation.from_quat(q).as_matrix(); T[:3, 3] = t; return T
T_base_velo = se3(EXT[:4], EXT[4:])
Tr = np.eye(4)
Tr[:3, :] = np.array([float(v) for v in next(l for l in open(f'{GT}/calib_05.txt') if l.startswith('Tr:')).split()[1:]]).reshape(3, 4)
gt_cam = np.loadtxt(f'{GT}/05.txt').reshape(-1, 3, 4)
frames = np.loadtxt('/home/user/data/kitti_raw_05/ros2.frames.txt', dtype=np.int64)
stamp_of = {int(f): int(t) for f, t, _ in frames}
est = np.loadtxt(est_path); est_ns = np.round(est[:, 0] * 1e9).astype(np.int64)
gt, es, ts = [], [], []
for f in range(len(gt_cam)):
    j = np.searchsorted(est_ns, stamp_of[f])
    k = [k for k in (j - 1, j) if 0 <= k < len(est_ns) and abs(est_ns[k] - stamp_of[f]) <= 1_000_000][0]
    Tc = np.eye(4); Tc[:3, :] = gt_cam[f]
    gt.append(np.linalg.inv(Tr) @ Tc @ Tr); es.append(se3(est[k, 4:8], est[k, 1:4]) @ T_base_velo); ts.append(est[k, 0])
gt, es, ts = np.array(gt), np.array(es), np.array(ts)
n = len(gt)
step = np.linalg.norm(np.diff(gt[:, :3, 3], axis=0), axis=1)
dist = np.concatenate([[0], np.cumsum(step)])
est_step = np.linalg.norm(np.diff(es[:, :3, 3], axis=0), axis=1)
print(f'frames {n}, GT path {dist[-1]:.1f} m, estimate path {est_step.sum():.1f} m ({100*(est_step.sum()/dist[-1]-1):+.3f} %)')
dt = np.diff(ts); speed = step / dt
yaw_gt = np.array([Rotation.from_matrix(T[:3, :3]).as_euler('zyx')[0] for T in gt])
def rel(T, i, j): return np.linalg.inv(T[i]) @ T[j]
def windows(L):
    out = []
    i = 0
    while True:
        j = np.searchsorted(dist, dist[i] + L)
        if j >= n: break
        Rg, Re = rel(gt, i, j), rel(es, i, j)
        tg, te = Rg[:3, 3], Re[:3, 3]
        d = te - tg                       # in GT frame i (velodyne: x forward, y left, z up)
        h = np.array([tg[0], tg[1], 0.0]); hu = h / np.linalg.norm(h)
        along = d @ hu; lateral = d @ np.array([-hu[1], hu[0], 0.0]); vertical = d[2]
        scale = (np.linalg.norm(te) - np.linalg.norm(tg)) / np.linalg.norm(tg)
        E = np.linalg.inv(Rg) @ Re
        ypr = Rotation.from_matrix(E[:3, :3]).as_euler('zyx', degrees=True)
        turn = np.degrees(np.abs(np.unwrap(yaw_gt[i:j + 1])[-1] - np.unwrap(yaw_gt[i:j + 1])[0]))
        v = (dist[j] - dist[i]) / (ts[j] - ts[i])
        out.append(dict(i=i, j=j, d0=dist[i], terr=np.linalg.norm(d) / L * 100, along=along / L * 100,
                        lateral=lateral / L * 100, vertical=vertical / L * 100, scale=scale * 100,
                        yaw=ypr[0] / L, pitch=ypr[1] / L, roll=ypr[2] / L, turn=turn, v=v, dz=tg[2]))
        i = np.searchsorted(dist, dist[i] + 10.0)
    return out
for L in (100, 200, 400, 800):
    w = windows(L)
    a = {k: np.array([x[k] for x in w]) for k in w[0]}
    print(f'\nL={L} m, {len(w)} windows: t_err {a["terr"].mean():.3f} %  | mean signed: along {a["along"].mean():+.3f} % '
          f'lateral {a["lateral"].mean():+.3f} % vertical {a["vertical"].mean():+.3f} % ; RMS: along {np.sqrt((a["along"]**2).mean()):.3f} '
          f'lateral {np.sqrt((a["lateral"]**2).mean()):.3f} vertical {np.sqrt((a["vertical"]**2).mean()):.3f}')
    print(f'  rot err deg/m RMS: yaw {np.sqrt((a["yaw"]**2).mean()):.5f} pitch {np.sqrt((a["pitch"]**2).mean()):.5f} roll {np.sqrt((a["roll"]**2).mean()):.5f}')
    if L == 200:
        straight = a['turn'] < 10; turning = a['turn'] > 45
        for name, m in (('straight (<10 deg)', straight), ('turning (>45 deg)', turning)):
            print(f'  {name}: n={m.sum()} t_err {a["terr"][m].mean():.3f} % along {a["along"][m].mean():+.3f} lateral RMS {np.sqrt((a["lateral"][m]**2).mean()):.3f} vertical RMS {np.sqrt((a["vertical"][m]**2).mean()):.3f}')
        for lo, hi in ((0, 6), (6, 9), (9, 20)):
            m = (a['v'] >= lo) & (a['v'] < hi)
            if m.sum(): print(f'  speed {lo}-{hi} m/s: n={m.sum()} t_err {a["terr"][m].mean():.3f} % along(scale) {a["along"][m].mean():+.3f} %')
        print('  by position along the drive (200 m windows every ~200 m): d0, t_err %, along %, lateral %, vertical %, yaw deg/m, turn deg, v m/s')
        for x in w[::20]:
            print(f'   {x["d0"]:7.0f} {x["terr"]:6.2f} {x["along"]:+6.2f} {x["lateral"]:+6.2f} {x["vertical"]:+6.2f} {x["yaw"]:+.5f} {x["turn"]:6.1f} {x["v"]:5.1f}')

# A constant frame rotation C between the estimate's and the ground truth's frames turns every relative translation
# by the same angle: t_est ~ C t_gt for all windows. Fit C (Kabsch, no translation) on 100 m windows, then score again.
def rel_pairs(L):
    P, Q, idx = [], [], []
    i = 0
    while True:
        j = np.searchsorted(dist, dist[i] + L)
        if j >= n: break
        P.append(rel(gt, i, j)[:3, 3]); Q.append(rel(es, i, j)[:3, 3]); idx.append((i, j))
        i = np.searchsorted(dist, dist[i] + 10.0)
    return np.array(P), np.array(Q), idx
P, Q, _ = rel_pairs(100)
U, S, Vt = np.linalg.svd(Q.T @ P)
D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(U @ Vt))
C = U @ D @ Vt            # Q ~ C P
ypr = Rotation.from_matrix(C).as_euler('zyx', degrees=True)
print(f'\nbest constant rotation (est = C gt), yaw {ypr[0]:+.3f} pitch {ypr[1]:+.3f} roll {ypr[2]:+.3f} deg')
for L in (100, 200, 400, 800):
    P, Q, idx = rel_pairs(L)
    raw = np.linalg.norm(Q - P, axis=1) / L * 100
    cor = np.linalg.norm(Q - P @ C.T, axis=1) / L * 100
    print(f'  L={L}: mean translation error {raw.mean():.3f} % -> {cor.mean():.3f} % after removing C')
# signed rotation drift per metre (does heading drift accumulate?)
for L in (100, 400, 800):
    w = windows(L)
    print(f'  L={L}: signed mean yaw {np.mean([x["yaw"] for x in w]):+.5f} pitch {np.mean([x["pitch"] for x in w]):+.5f} roll {np.mean([x["roll"] for x in w]):+.5f} deg/m')
