#!/usr/bin/env python3
"""Moments where a MapsHD ground truth and the frontend disagree on velocity (chapter 28).

For each scored run: velocity of the GT and of the frontend (lidar frame) by central differences over 0.1 s at
the frontend's stamps, both in their own world frame, and the change of horizontal speed over the next 0.2 s.
Lists the moments where |dspeed_gt - dspeed_frontend| exceeds a threshold (m/s), merged into events, and the share
of the aligned squared error that falls within 1 s of such an event. A disagreement does not say which one is
wrong; an IMU check (corridor_imu_check.py) does.
Usage: gt_velocity_disagreement.py <score_root> <gt_sealed_dir> [threshold=1.0]
"""
import glob
import os
import sys

import numpy as np

root, gtdir = sys.argv[1:3]
thr = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0


def pos(traj, t):
    return np.column_stack([np.interp(t, traj[:, 0], traj[:, k]) for k in (1, 2, 3)])


def hspeed(traj, t, h=0.05):
    v = (pos(traj, t + h) - pos(traj, t - h)) / (2 * h)
    return np.hypot(v[:, 0], v[:, 1])


print('trial,events,event_times_rel_s,share_sq_error_near_events,rmse,rmse_outside_events')
for d in sorted(glob.glob(os.path.join(root, 'score_*'))):
    name = os.path.basename(d)[len('score_'):].rsplit('_', 1)[0]
    if 'trial-' not in name:
        continue
    sc, tr = name.split('_trial-')
    gt = np.loadtxt(os.path.join(gtdir, sc, 'trial-' + tr, 'ground_truth.tum'))
    est = np.loadtxt(os.path.join(d, 'frontend_lidar.tum'))
    t0 = gt[0, 0]
    t = est[(est[:, 0] > gt[0, 0] + 0.3) & (est[:, 0] < gt[-1, 0] - 0.3), 0]
    dg = hspeed(gt, t + 0.2) - hspeed(gt, t)
    de = hspeed(est, t + 0.2) - hspeed(est, t)
    bad = np.abs(dg - de) > thr
    events = []
    for ti in t[bad]:
        if not events or ti - events[-1][-1] > 1.0:
            events.append([ti])
        else:
            events[-1].append(ti)
    # aligned error at GT stamps
    g = gt[(gt[:, 0] >= est[0, 0]) & (gt[:, 0] <= est[-1, 0])]
    e = pos(est, g[:, 0])
    p = g[:, 1:4]
    ms, md = e.mean(0), p.mean(0)
    U, _, Vt = np.linalg.svd((p - md).T @ (e - ms))
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    sq = (((R @ (e - ms).T).T + md - p) ** 2).sum(1)
    near = np.zeros(len(g), bool)
    for ev in events:
        near |= (g[:, 0] >= ev[0] - 1.0) & (g[:, 0] <= ev[-1] + 1.0)
    share = sq[near].sum() / sq.sum()
    out = np.sqrt(sq[~near].mean()) if (~near).any() else float('nan')
    times = ' '.join(f'{ev[0] - t0:.1f}' for ev in events)
    print(f'{name},{len(events)},{times},{share:.3f},{np.sqrt(sq.mean()):.4f},{out:.4f}')
