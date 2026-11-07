#!/usr/bin/env python3
"""Relative transform of two sessions from RTK GNSS, against the join's sets and the checkpoint truth.

Plan: bench_report/data/multisession_rtk_2026-11-07/plan_gnss.txt.

Each session's RTK-fixed GNSS positions (status >= 0, horizontal standard deviation <= 0.3 m) are paired
with its frontend poses (nearest within 0.05 s, moved by the antenna offset) and one SE(3) fit per session
puts its odometry frame in a common local ENU frame. T_gnss = T_enu<-A^-1 T_enu<-B maps B's odometry frame
to A's. Transforms are compared as the join's ambiguity test does: the RMS over B's keyframe positions of
|T1 p - T2 p|, and the rotation angle between them.

usage:
  p3_rtk_gnss_relative.py --bag-a BAG --bag-b BAG --front-a TUM --front-b TUM --keyframes-b TUM \\
    --report join_report.yaml --drift-ref drift_only.json
"""
import argparse
import json
import math

import numpy as np
import yaml


def read_fixes(bag, max_std_m):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import NavSatFix
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/gnss/fix']))
    out = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, NavSatFix)
        c = m.position_covariance
        if m.status.status < 0 or c[0] > max_std_m ** 2 or c[4] > max_std_m ** 2:
            continue
        out.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec,
                    m.latitude, m.longitude, m.altitude))
    return np.asarray(out)


def to_ecef(lat, lon, h):
    a, f = 6378137.0, 1 / 298.257223563
    e2 = 2 * f - f * f
    la, lo = np.radians(lat), np.radians(lon)
    n = a / np.sqrt(1 - e2 * np.sin(la) ** 2)
    return np.stack([(n + h) * np.cos(la) * np.cos(lo), (n + h) * np.cos(la) * np.sin(lo),
                     (n * (1 - e2) + h) * np.sin(la)], -1)


def to_enu(fixes, origin):
    lat0, lon0 = math.radians(origin[1]), math.radians(origin[2])
    r = np.array([[-math.sin(lon0), math.cos(lon0), 0.0],
                  [-math.sin(lat0) * math.cos(lon0), -math.sin(lat0) * math.sin(lon0), math.cos(lat0)],
                  [math.cos(lat0) * math.cos(lon0), math.cos(lat0) * math.sin(lon0), math.sin(lat0)]])
    d = to_ecef(fixes[:, 1], fixes[:, 2], fixes[:, 3]) - to_ecef(origin[1], origin[2], origin[3])
    return d @ r.T


def quat_to_rot(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rigid_fit(src, dst):
    sc, dc = src.mean(0), dst.mean(0)
    u, _, vt = np.linalg.svd((src - sc).T @ (dst - dc))
    d = np.diag([1.0, 1.0, np.sign(np.linalg.det(vt.T @ u.T))])
    r = vt.T @ d @ u.T
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = r, dc - r @ sc
    return t


def session_to_enu(fixes_enu, fix_t, tum, antenna):
    traj = np.loadtxt(tum, comments='#')
    src, dst = [], []
    for t, p in zip(fix_t, fixes_enu):
        i = int(np.argmin(np.abs(traj[:, 0] - t)))
        if abs(traj[i, 0] - t) > 0.05:
            continue
        src.append(traj[i, 1:4] + quat_to_rot(*traj[i, 4:8]) @ antenna)
        dst.append(p)
    src, dst = np.asarray(src), np.asarray(dst)
    t = rigid_fit(src, dst)
    residual = np.linalg.norm(src @ t[:3, :3].T + t[:3, 3] - dst, axis=1)
    return t, {'pairs': int(len(src)), 'fit_rmse_m': float(math.sqrt(np.mean(residual ** 2))),
               'fit_max_m': float(residual.max())}


def from_row(row):
    t = np.eye(4)
    t[:3, :] = np.asarray(row, dtype=float).reshape(3, 4)
    return t


def disagreement(t1, t2, points):
    d = points @ t1[:3, :3].T + t1[:3, 3] - (points @ t2[:3, :3].T + t2[:3, 3])
    rms = float(math.sqrt(np.mean(np.sum(d * d, axis=1))))
    c = (np.trace(t1[:3, :3].T @ t2[:3, :3]) - 1.0) / 2.0
    return {'rms_m': round(rms, 3), 'angle_deg': round(math.degrees(math.acos(max(-1.0, min(1.0, c)))), 3)}


def main():
    ap = argparse.ArgumentParser()
    for name in ('--bag-a', '--bag-b', '--front-a', '--front-b', '--keyframes-b', '--report',
                 '--drift-ref'):
        ap.add_argument(name, required=True)
    ap.add_argument('--antenna', type=float, nargs=3, default=[0.034, 0.0, 0.046])
    ap.add_argument('--max-std-m', type=float, default=0.3)
    a = ap.parse_args()
    fa, fb = read_fixes(a.bag_a, a.max_std_m), read_fixes(a.bag_b, a.max_std_m)
    origin = fa[0]
    ta, qa = session_to_enu(to_enu(fa, origin), fa[:, 0], a.front_a, np.asarray(a.antenna))
    tb, qb = session_to_enu(to_enu(fb, origin), fb[:, 0], a.front_b, np.asarray(a.antenna))
    t_gnss = np.linalg.inv(ta) @ tb
    keyframes = np.loadtxt(a.keyframes_b, comments='#')[:, 1:4]
    drift = from_row(json.load(open(a.drift_ref))['debug_exclude_transform'])
    report = yaml.safe_load(open(a.report))
    sets = []
    for session in report['sessions']:
        for s in session.get('sets_tried') or []:
            t = from_row(s['transform'])
            sets.append({'size': s['size'], 'share_below': s['share_below'], 'passed': s['passed'],
                         'vs_gnss': disagreement(t, t_gnss, keyframes),
                         'vs_drift_reference': disagreement(t, drift, keyframes)})
    print(json.dumps({
        'kept_fixes': {'a': int(len(fa)), 'b': int(len(fb))},
        'fit_a': qa, 'fit_b': qb,
        'gnss_vs_drift_reference': disagreement(t_gnss, drift, keyframes),
        'sets': sets,
    }, indent=1))


if __name__ == '__main__':
    main()
