#!/usr/bin/env python3
"""Checkpoint RMSE on RTK-SLAM, absolute (no alignment) and SE(3)-aligned.

Reimplements the published RTK-SLAM evaluation (github.com/Willyzw/rtk-slam-eval): ENU -> lat/lon on
WGS84, lat/lon -> UTM 32N on GRS80, geoid offset -48.22 m on the SLAM height and -48.5 m on GNSS
heights, the nearest pose within 2 s of each checkpoint. se3_rmse_3d is its "SE3-aligned ATE".
--lever-arm moves each pose from the trajectory frame to the checkpoint point (body frame); the
Stadtgarten rig's IMU-to-base_center offset is -0.062 0 -0.216
(docs/research/gnss-rtk-slam-absolute-accuracy.md). For the SE(3)-aligned score the ENU origin only
sets the tangent point.

usage:
  rtk_slam_checkpoint_eval.py GT_CSV --traj TUM --origin ENU_ORIGIN_JSON [--lever-arm X Y Z] [--label L]
  rtk_slam_checkpoint_eval.py GT_CSV --rtk RTK_TXT
"""
import argparse
import csv
import json
import math

import numpy as np


def enu_to_latlon(e, n, u, lat0, lon0, h0):
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f * f
    la, lo = math.radians(lat0), math.radians(lon0)
    N0 = a / math.sqrt(1 - e2 * math.sin(la) ** 2)
    x0 = (N0 + h0) * math.cos(la) * math.cos(lo)
    y0 = (N0 + h0) * math.cos(la) * math.sin(lo)
    z0 = (N0 * (1 - e2) + h0) * math.sin(la)
    sla, cla, slo, clo = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
    x = x0 - slo * e + clo * n * (-sla) + clo * u * cla
    y = y0 + clo * e + slo * n * (-sla) + slo * u * cla
    z = z0 + n * cla + u * sla
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - e2))
    h = 0.0
    for _ in range(10):
        N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - N
        nl = math.atan2(z, p * (1 - e2 * N / (N + h)))
        if abs(nl - lat) < 1e-12:
            lat = nl
            break
        lat = nl
    return math.degrees(lat), math.degrees(lon), h


def latlon_to_utm(lat, lon, h, cm=9.0):
    a = 6378137.0
    f = 1 / 298.257222101
    e2 = 2 * f - f * f
    ep2 = e2 / (1 - e2)
    la, lo, c = math.radians(lat), math.radians(lon), math.radians(cm)
    N = a / math.sqrt(1 - e2 * math.sin(la) ** 2)
    T = math.tan(la) ** 2
    C = ep2 * math.cos(la) ** 2
    A = math.cos(la) * (lo - c)
    M = a * ((1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256) * la
             - (3 * e2 / 8 + 3 * e2 ** 2 / 32 + 45 * e2 ** 3 / 1024) * math.sin(2 * la)
             + (15 * e2 ** 2 / 256 + 45 * e2 ** 3 / 1024) * math.sin(4 * la)
             - (35 * e2 ** 3 / 3072) * math.sin(6 * la))
    E = 500000 + 0.9996 * N * (A + (1 - T + C) * A ** 3 / 6
                               + (5 - 18 * T + T ** 2 + 72 * C - 58 * ep2) * A ** 5 / 120)
    Nn = 0.9996 * (M + N * math.tan(la) * (A ** 2 / 2 + (5 - T + 9 * C + 4 * C ** 2) * A ** 4 / 24
                                          + (61 - 58 * T + T ** 2 + 600 * C - 330 * ep2) * A ** 6 / 720))
    return E, Nn, h


def read_gt(path):
    with open(path, newline='', encoding='utf-8') as f:
        return [(r['point_id'], np.array([float(r['easting']), float(r['northing']), float(r['height'])]),
                 r.get('env', 'outdoor'), float(r['timestamp'])) for r in csv.DictReader(f)]


def rmse(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else float('nan')


def se3_rmse(src, dst):
    src, dst = np.asarray(src), np.asarray(dst)
    sc, dc = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - sc).T @ (dst - dc))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = dc - R @ sc
    return rmse([float(np.linalg.norm(R @ s + t - d)) for s, d in zip(src, dst)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('gt')
    ap.add_argument('--traj')
    ap.add_argument('--origin')
    ap.add_argument('--rtk')
    ap.add_argument('--label', default='')
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--per-point', action='store_true')
    ap.add_argument('--lever-arm', type=float, nargs=3, default=None,
                    help='offset from the trajectory frame to the checkpoint point, in the body frame')
    a = ap.parse_args()
    gt = read_gt(a.gt)
    pairs = []
    if a.traj:
        traj = np.loadtxt(a.traj, comments='#')
        ts, pos = traj[:, 0], traj[:, 1:4].copy()
        if a.lever_arm is not None:
            t = np.array(a.lever_arm)
            for k in range(len(traj)):
                x, y, z, w = traj[k, 4:8]
                R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                              [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                              [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
                pos[k] = pos[k] + R @ t
        lat0, lon0, h0 = json.loads(open(a.origin).read())
        for pid, g, env, t in gt:
            i = int(np.argmin(np.abs(ts - t)))
            if abs(ts[i] - t) > 2.0:
                continue
            e, n, u = pos[i]
            lat, lon, h = enu_to_latlon(e, n, u - 48.22, lat0, lon0, h0)
            pairs.append((pid, env, np.array(latlon_to_utm(lat, lon, h)), g))
    else:
        rows = np.loadtxt(a.rtk, comments='#')
        for pid, g, env, t in gt:
            i = int(np.argmin(np.abs(rows[:, 0] - t)))
            if abs(rows[i, 0] - t) > 2.0:
                continue
            E, N, h = latlon_to_utm(rows[i, 1], rows[i, 2], rows[i, 3])
            pairs.append((pid, env, np.array([E, N, h - 48.5]), g))
    errs = [float(np.linalg.norm(p - g)) for _, _, p, g in pairs]
    res = {'label': a.label, 'n': len(pairs), 'n_gt': len(gt), 'abs_rmse_3d': rmse(errs),
           'abs_rmse_2d': rmse([float(np.linalg.norm((p - g)[:2])) for _, _, p, g in pairs]),
           'abs_rmse_z': rmse([float(abs((p - g)[2])) for _, _, p, g in pairs]),
           'se3_rmse_3d': se3_rmse([p for _, _, p, _ in pairs], [g for _, _, _, g in pairs]) if len(pairs) >= 3 else None}
    for env in sorted({e for _, e, _, _ in pairs}):
        res[f'abs_rmse_3d_{env}'] = rmse([err for (_, e, _, _), err in zip(pairs, errs) if e == env])
    if a.per_point:
        res['per_point'] = [{'id': pid, 'err_3d': round(err, 3), 'err_2d': round(float(np.linalg.norm((p - g)[:2])), 3), 'dz': round(float((p - g)[2]), 3)} for (pid, env, p, g), err in zip(pairs, errs)]
    print(json.dumps(res, indent=None if not a.json else 1))


if __name__ == '__main__':
    main()
