#!/usr/bin/env python3
"""
Chapter 30.9: one body-frame accelerometer bias for the whole run, one gravity direction per window.

Model, for every IMU sample i of window k (attitude R_i from a trajectory of the IMU frame):
    R_i a_i = v_k + R_i b,
v_k = g u_k the gravity the window should see (fixed in the world within a window, free between windows) and b a
constant body-frame bias. Eliminating every v_k leaves one 3x3 system for b, solved in the directions whose
normalised eigenvalue reaches --min-observability (the vertical axis is not observable against |g|). The tilts of
v_k then show the trajectory's attitude error per window, without the bias; their spread is the drift that a
gravity window could correct. Samples with a yaw rate above --max-yaw-rate are left out, as in RKO-LIO's window.
Usage: p1_gravity_bias_joint.py <bag> <imu_topic> <trajectory_tum> [--gt <gt_tum> --t-imu-gt <npy|identity>]
       [--window 40] [--accel-scale 1.0]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p1_gravity_window_check import quat_to_R, read_imu  # noqa: E402


def attitudes_at(t, traj, R_body_imu):
    idx = np.clip(np.searchsorted(traj[:, 0], t), 1, len(traj) - 1)
    near = np.where(np.abs(traj[idx - 1, 0] - t) < np.abs(traj[idx, 0] - t), idx - 1, idx)
    ok = np.abs(traj[near, 0] - t) < 0.1
    R = np.array([quat_to_R(q) for q in traj[near, 4:8]]) @ R_body_imu
    return R, ok


def smoothed(t, values, span=0.1):
    """Mean of `values` over consecutive bins of `span` seconds, given back per sample (RKO-LIO averages per scan)."""
    bins = np.floor((t - t[0]) / span).astype(int)
    sums = np.bincount(bins, weights=values)
    counts = np.bincount(bins)
    return (sums / np.maximum(counts, 1))[bins]


def solve(t, acc, gyro_z_world, R, window, min_obs, max_yaw_rate):
    keep = np.abs(smoothed(t, gyro_z_world)) <= max_yaw_rate
    t, acc, R = t[keep], acc[keep], R[keep]
    k = np.floor((t - t[0]) / window).astype(int)
    S = np.zeros((3, 3))
    rhs = np.zeros(3)
    for w in np.unique(k):
        sel = k == w
        n = sel.sum()
        if n < 50:
            continue
        sum_R = R[sel].sum(0)
        sum_world = np.einsum('nij,nj->i', R[sel], acc[sel])
        S += n * np.eye(3) - sum_R.T @ sum_R / n
        rhs += acc[sel].sum(0) - sum_R.T @ sum_world / n
    total = len(t)
    evals, evecs = np.linalg.eigh(S / total)
    bias = np.zeros(3)
    for j in range(3):
        if evals[j] >= min_obs:
            bias += (evecs[:, j] @ (rhs / total) / evals[j]) * evecs[:, j]
    tilts = []
    for w in np.unique(k):
        sel = k == w
        if sel.sum() < 50:
            continue
        v = np.einsum('nij,nj->ni', R[sel], acc[sel] - bias).mean(0)
        tilts.append(v / np.linalg.norm(v))
    return bias, np.array(tilts), evals


def tilt_deg(u):
    return np.degrees(np.arccos(np.clip(u[:, 2], -1, 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('imu_topic')
    ap.add_argument('trajectory')
    ap.add_argument('--gt')
    ap.add_argument('--t-imu-gt', default='identity')
    ap.add_argument('--window', type=float, default=40.0)
    ap.add_argument('--accel-scale', type=float, default=1.0)
    ap.add_argument('--min-observability', type=float, default=0.3)
    ap.add_argument('--max-yaw-rate', type=float, default=0.05)
    args = ap.parse_args()

    cache = os.path.join(os.path.dirname(os.path.abspath(args.trajectory)), 'imu_cache.npz')
    if os.path.exists(cache):
        loaded = np.load(cache)
        t, acc, gyro = loaded['t'], loaded['acc'], loaded['gyro']
    else:
        t, acc, gyro = read_imu(args.bag, args.imu_topic, with_gyro=True)
        np.savez(cache, t=t, acc=acc, gyro=gyro)
    acc = acc * args.accel_scale
    sources = [('trajectory', np.loadtxt(args.trajectory), np.eye(3))]
    if args.gt:
        T = np.eye(4) if args.t_imu_gt == 'identity' else np.load(args.t_imu_gt)
        sources.append(('ground truth', np.loadtxt(args.gt), T[:3, :3].T))
    for name, traj, R_body_imu in sources:
        R, ok = attitudes_at(t, traj, R_body_imu)
        gz = np.einsum('nij,nj->ni', R[ok], gyro[ok])[:, 2]
        bias, u, evals = solve(t[ok], acc[ok], gz, R[ok], args.window, args.min_observability, args.max_yaw_rate)
        if len(u) == 0:
            print(f'{name:12s} no window with enough low-yaw-rate samples')
            continue
        horizontal = np.degrees(np.arctan2(np.linalg.norm(bias[:2]), 9.80665))
        mean_u = u.mean(0) / np.linalg.norm(u.mean(0))
        spread = np.degrees(np.arccos(np.clip(u @ mean_u, -1, 1)))
        print(f'{name:12s} bias {np.round(bias, 4)} m/s^2 ({horizontal:.2f} deg)  observability {np.round(evals, 2)}  '
              f'window tilt median {np.median(tilt_deg(u)):.2f} deg  drift about the mean tilt: median '
              f'{np.median(spread):.2f} p90 {np.percentile(spread, 90):.2f} max {spread.max():.2f} deg '
              f'({len(u)} windows)')


if __name__ == '__main__':
    main()
