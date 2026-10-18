#!/usr/bin/env python3
"""Per-axis RMSE (after the same SE(3) Umeyama alignment, positions interpolated at the ground-truth stamps) of the
MapsHD frontend and pipeline-B trajectories in the LiDAR frame, and the share of time with |dz| > 0.1 m.
Usage: mapshd_error_breakdown.py <scores_dir> <gt_sealed_dir>
"""
import glob
import os
import sys

import numpy as np

root, gtdir = sys.argv[1:3]
print('trial,trajectory,rmse_x,rmse_y,rmse_z,rmse_3d,share_abs_dz_gt_0.1m,gt_path_m,gt_duration_s')
for d in sorted(glob.glob(os.path.join(root, '*_B'))):
    trial = os.path.basename(d)[:-2]
    sc, tr = trial.split('_trial-')
    gt = np.loadtxt(os.path.join(gtdir, sc, 'trial-' + tr, 'ground_truth.tum'))
    for kind in ('frontend', 'pipeline'):
        est = np.loadtxt(os.path.join(d, f'{kind}_lidar.tum'))
        g = gt[(gt[:, 0] >= est[0, 0]) & (gt[:, 0] <= est[-1, 0])]
        e = np.column_stack([np.interp(g[:, 0], est[:, 0], est[:, k]) for k in (1, 2, 3)])
        p = g[:, 1:4]
        mu_e, mu_p = e.mean(0), p.mean(0)
        U, _, Vt = np.linalg.svd((p - mu_p).T @ (e - mu_e))
        R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
        err = (R @ (e - mu_e).T).T + mu_p - p
        r = np.sqrt((err ** 2).mean(0))
        length = np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
        print(f'{trial},{kind if kind == "frontend" else "pipeline_B"},{r[0]:.4f},{r[1]:.4f},{r[2]:.4f},'
              f'{np.sqrt((err ** 2).sum(1).mean()):.4f},{(np.abs(err[:, 2]) > 0.1).mean():.3f},{length:.0f},'
              f'{g[-1, 0] - g[0, 0]:.0f}')
