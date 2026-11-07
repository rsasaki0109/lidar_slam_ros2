#!/usr/bin/env python3
"""Score a two-session join on RTK-SLAM sequences that visit the same surveyed checkpoints.

Plan: bench_report/data/multisession_rtk_2026-11-07/plan.txt.

Each trajectory (TUM, base frame in the session's odometry frame) is read at every checkpoint of its
sequence: the nearest pose within 2 s of the checkpoint's time, moved by the lever arm. Sessions A and B
share the checkpoints at identical survey coordinates (the files label them differently).

- With --alone-a/--alone-b (single-session trajectories): the best rigid placement of B onto A at the
  shared checkpoints (SE(3) least squares) and its residual, the drift-only reference. The placement is
  printed as the 12 values of debug_exclude_transform (3x4, row-major, B's frame to A's).
- With --joint-a/--joint-b (the joined map, both in A's frame): the shared-checkpoint gap.
- SE(3)-aligned checkpoint RMSE against the survey (rtk_slam_checkpoint_eval.py's protocol) for every
  trajectory given, and B placed with the alignment fitted on A's joint checkpoints.

usage:
  p3_rtk_multisession_eval.py --gt-a A.csv --gt-b B.csv --origin ORIGIN.json \\
    [--alone-a A.tum --alone-b B.tum] [--joint-a A.tum --joint-b B.tum] [--lever-arm X Y Z]
"""
import argparse
import importlib.util
import json
import math
import pathlib

import numpy as np

_spec = importlib.util.spec_from_file_location(
    'rtk_slam_checkpoint_eval',
    pathlib.Path(__file__).resolve().parent / 'rtk_slam_checkpoint_eval.py')
rtk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rtk)


def quat_to_rot(x, y, z, w):
    """Rotation matrix of a unit quaternion (x, y, z, w)."""
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def at_checkpoints(tum_path, gt, lever_arm):
    """Checkpoint id -> the trajectory's lever-arm point at the checkpoint's time (local frame)."""
    traj = np.loadtxt(tum_path, comments='#')
    out = {}
    for pid, _, _, t in gt:
        i = int(np.argmin(np.abs(traj[:, 0] - t)))
        if abs(traj[i, 0] - t) > 2.0:
            continue
        out[pid] = traj[i, 1:4] + quat_to_rot(*traj[i, 4:8]) @ np.asarray(lever_arm)
    return out


def rigid_fit(src, dst):
    """R, t minimising |R src + t - dst| (Umeyama without scale)."""
    src, dst = np.asarray(src), np.asarray(dst)
    sc, dc = src.mean(0), dst.mean(0)
    u, _, vt = np.linalg.svd((src - sc).T @ (dst - dc))
    d = np.diag([1.0, 1.0, np.sign(np.linalg.det(vt.T @ u.T))])
    r = vt.T @ d @ u.T
    return r, dc - r @ sc


def stats(values):
    v = np.asarray(values)
    return {'rmse': float(math.sqrt(np.mean(v ** 2))), 'median': float(np.median(v)),
            'max': float(v.max()), 'n': int(len(v))}


def to_utm(local, origin):
    lat0, lon0, h0 = origin
    e, n, u = local
    lat, lon, h = rtk.enu_to_latlon(e, n, u - 48.22, lat0, lon0, h0)
    return np.array(rtk.latlon_to_utm(lat, lon, h))


def se3_score(points, gt, origin):
    """SE(3)-aligned RMSE of local checkpoint estimates against the survey, and the fitted alignment."""
    ids = [pid for pid, _, _, _ in gt if pid in points]
    truth = {pid: g for pid, g, _, _ in gt}
    est = [to_utm(points[pid], origin) for pid in ids]
    ref = [truth[pid] for pid in ids]
    r, t = rigid_fit(est, ref)
    errs = [float(np.linalg.norm(r @ e + t - g)) for e, g in zip(est, ref)]
    return stats(errs), (r, t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gt-a', required=True)
    ap.add_argument('--gt-b', required=True)
    ap.add_argument('--origin', required=True, help='ENU origin [lat, lon, h] of session A')
    ap.add_argument('--alone-a')
    ap.add_argument('--alone-b')
    ap.add_argument('--joint-a')
    ap.add_argument('--joint-b')
    ap.add_argument('--lever-arm', type=float, nargs=3, default=[-0.062, 0.0, -0.216])
    a = ap.parse_args()
    gt_a, gt_b = rtk.read_gt(a.gt_a), rtk.read_gt(a.gt_b)
    origin = json.loads(pathlib.Path(a.origin).read_text())
    res = {}
    # The files label the same surveyed point differently: pair points at identical coordinates.
    pairs = [(ia, ib) for ia, ga, _, _ in gt_a for ib, gb, _, _ in gt_b
             if np.linalg.norm(ga - gb) <= 0.001]
    res['shared_checkpoints'] = len(pairs)
    if a.alone_a and a.alone_b:
        pa, pb = at_checkpoints(a.alone_a, gt_a, a.lever_arm), at_checkpoints(a.alone_b, gt_b, a.lever_arm)
        ids = [(i, j) for i, j in pairs if i in pa and j in pb]
        r, t = rigid_fit([pb[j] for _, j in ids], [pa[i] for i, _ in ids])
        res['drift_only_reference'] = stats([np.linalg.norm(r @ pb[j] + t - pa[i]) for i, j in ids])
        res['debug_exclude_transform'] = [float(v) for v in np.hstack([r, t[:, None]]).ravel()]
        res['alone_a'] = se3_score(pa, gt_a, origin)[0]
        res['alone_b'] = se3_score(pb, gt_b, origin)[0]
    if a.joint_a and a.joint_b:
        pa, pb = at_checkpoints(a.joint_a, gt_a, a.lever_arm), at_checkpoints(a.joint_b, gt_b, a.lever_arm)
        ids = [(i, j) for i, j in pairs if i in pa and j in pb]
        res['joint_gap'] = stats([np.linalg.norm(pa[i] - pb[j]) for i, j in ids])
        res['joint_gap_per_point'] = {
            f'{i}/{j}': round(float(np.linalg.norm(pa[i] - pb[j])), 3) for i, j in ids}
        score_a, (r, t) = se3_score(pa, gt_a, origin)
        res['joint_a'] = score_a
        res['joint_b'] = se3_score(pb, gt_b, origin)[0]
        truth_b = {pid: g for pid, g, _, _ in gt_b}
        res['joint_b_with_a_alignment'] = stats(
            [np.linalg.norm(r @ to_utm(pb[i], origin) + t - truth_b[i]) for i in pb])
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
