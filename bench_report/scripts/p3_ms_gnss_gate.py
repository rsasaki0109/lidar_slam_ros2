#!/usr/bin/env python3
"""The GNSS gate of plan multisession_gnss_2026-11-07, evaluated offline on a join's sets and on given transforms.

Run with the ROS environment sourced and `python3 -P` (rosbag2_py reads the bags).

pair:  p3_ms_gnss_gate.py pair --bag-a BAG --bag-b BAG --front-a TUM --front-b TUM --keyframes-b TUM
                               --report join_report.yaml --truth alone.json [--checks debug_checks.yaml perturb.csv]
split: p3_ms_gnss_gate.py split --bag BAG --front TUM --keyframes TUM --report join_report.yaml [--gap-s 30]

A split's halves are cut as the join cuts them: at the middle of the keyframe stamps, gap_s left out; the truth is the
identity. Prints one JSON document: the fits, T_gnss against the truth, and every set (and checked transform) with
its distance to T_gnss and to the truth and the gate's verdict.
"""
import argparse
import csv
import json
import math
import os

import numpy as np
import yaml

ANTENNA = np.array([0.034, 0.0, 0.046])
MAX_STD_M = 0.3
PAIR_MAX_DT_S = 0.05
OVERLAP_M = 10.0
MIN_OVERLAP_PAIRS = 50
AGREE_M, AGREE_DEG = 1.0, 2.0
FLOOR_MIN_PLACES, FLOOR_MAX_SHARE = 5, 0.25


def read_fix_rows(bag):
    """Every /gnss/fix of a bag (mcap or sqlite3): stamp, lat, lon, alt, status, covariance xx and yy."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import NavSatFix
    storage = 'sqlite3' if any(f.endswith('.db3') for f in os.listdir(bag)) else 'mcap'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/gnss/fix']))
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, NavSatFix)
        rows.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m.latitude, m.longitude, m.altitude,
                     m.status.status, m.position_covariance[0], m.position_covariance[4]))
    return rows


def read_fixes(bag):
    """The kept fixes (stamp, lat, lon, alt) of a bag directory or of a CSV written by `export`."""
    rows = np.loadtxt(bag, delimiter=',', skiprows=1, ndmin=2) if bag.endswith('.csv') else \
        np.asarray(read_fix_rows(bag), dtype=float).reshape(-1, 7)
    keep = (rows[:, 4] >= 0) & (rows[:, 5] <= MAX_STD_M ** 2) & (rows[:, 6] <= MAX_STD_M ** 2)
    return rows[keep, :4]


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


def pairs(fix_t, fix_enu, traj):
    """Antenna positions in the odometry frame and the GNSS positions they pair with (nearest pose within 0.05 s)."""
    i = np.clip(np.searchsorted(traj[:, 0], fix_t), 1, len(traj) - 1)
    i = np.where(np.abs(traj[i - 1, 0] - fix_t) <= np.abs(traj[i, 0] - fix_t), i - 1, i)
    keep = np.abs(traj[i, 0] - fix_t) <= PAIR_MAX_DT_S
    src = np.asarray([traj[j, 1:4] + quat_to_rot(*traj[j, 4:8]) @ ANTENNA for j in i[keep]])
    return src.reshape(-1, 3), fix_enu[keep]


def overlap(dst, other):
    from scipy.spatial import cKDTree
    d, _ = cKDTree(other).query(dst, k=1)
    return d <= OVERLAP_M


def fit_4dof(src, dst):
    sc, dc = src.mean(0), dst.mean(0)
    s, d = src - sc, dst - dc
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]), np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    t = np.eye(4)
    t[:3, :3] = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0, 0, 1.0]])
    t[:3, 3] = dc - t[:3, :3] @ sc
    return t


def trimmed_fit(src, dst):
    keep = np.ones(len(src), bool)
    for rounds in range(1, 11):
        t = fit_4dof(src[keep], dst[keep])
        r = np.linalg.norm(src @ t[:3, :3].T + t[:3, 3] - dst, axis=1)
        new = r <= max(0.3, 3 * 1.4826 * float(np.median(r[keep])))
        if np.array_equal(new, keep):
            break
        keep = new
    r = r[keep]
    return t, {'kept': int(keep.sum()), 'rounds': rounds, 'fit_rmse_m': round(float(math.sqrt(np.mean(r * r))), 3),
               'fit_max_m': round(float(r.max()), 3)}


def t_gnss(sa, da, sb, db):
    """T_gnss from B's odometry frame to A's, or None (too little overlap), and the fit summaries."""
    oa, ob = overlap(da, db), overlap(db, da)
    info = {'pairs_a': int(len(sa)), 'pairs_b': int(len(sb)), 'overlap_a': int(oa.sum()), 'overlap_b': int(ob.sum())}
    if oa.sum() < MIN_OVERLAP_PAIRS or ob.sum() < MIN_OVERLAP_PAIRS:
        return None, info
    ta, info['fit_a'] = trimmed_fit(sa[oa], da[oa])
    tb, info['fit_b'] = trimmed_fit(sb[ob], db[ob])
    return np.linalg.inv(ta) @ tb, info


def from_row(row):
    t = np.eye(4)
    t[:3, :] = np.asarray(row, dtype=float).reshape(3, 4)
    return t


def disagreement(t1, t2, points):
    d = points @ t1[:3, :3].T + t1[:3, 3] - (points @ t2[:3, :3].T + t2[:3, 3])
    c = (np.trace(t1[:3, :3].T @ t2[:3, :3]) - 1.0) / 2.0
    return round(float(math.sqrt(np.mean(np.sum(d * d, axis=1)))), 3), \
        round(math.degrees(math.acos(max(-1.0, min(1.0, c)))), 3)


def judge(t, places, share_below, gnss, truth, keyframes):
    g_m, g_deg = disagreement(t, gnss, keyframes)
    t_m, t_deg = disagreement(t, truth, keyframes)
    agree = g_m <= AGREE_M and g_deg <= AGREE_DEG
    floor = places >= FLOOR_MIN_PLACES and share_below <= FLOOR_MAX_SHARE
    return {'vs_gnss_m': g_m, 'vs_gnss_deg': g_deg, 'vs_truth_m': t_m, 'vs_truth_deg': t_deg, 'places': places,
            'share_below': round(share_below, 4), 'gnss_agrees': agree, 'floor': floor, 'gate': agree and floor}


def evaluate(sets, gnss, truth, keyframes, info, checks=None):
    out = {'gnss': info}
    if gnss is None:
        out['gnss_available'] = False
        return out
    m, deg = disagreement(gnss, truth, keyframes)
    out.update({'gnss_available': True, 'gnss_vs_truth_m': m, 'gnss_vs_truth_deg': deg, 'sets': [], 'joins': None})
    for i, s in enumerate(sets):
        j = judge(from_row(s['transform']), s['places'], s['share_below'], gnss, truth, keyframes)
        j.update({'set': i, 'size': s['size'], 'whole_map_passed': s['passed']})
        out['sets'].append(j)
        if j['gate'] and out['joins'] is None:
            out['joins'] = i
    if checks:
        out['checks'] = []
        for c in checks:
            j = judge(from_row(c['transform']), c['places'], c['share_below'], gnss, truth, keyframes)
            j.update({k: c[k] for k in ('index', 'kind', 'axis', 'amount')})
            out['checks'].append(j)
    return out


def read_checks(path, labels_path):
    labels = {int(r['index']): r for r in csv.DictReader(open(labels_path))}
    seen, out = set(), []
    for c in yaml.safe_load(open(path))['debug_checks']['checks']:
        if c['index'] in seen or c.get('bar', 0.5) != 0.5:  # the bar 0.5 row of each transform
            continue
        seen.add(c['index'])
        lab = labels[c['index']]
        out.append({'index': c['index'], 'kind': lab['kind'], 'axis': lab['axis'], 'amount': float(lab['amount']),
                    'places': c['places'], 'share_below': c['share_below'], 'transform': c['transform']})
    return out


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='mode', required=True)
    p = sub.add_parser('pair')
    for name in ('--bag-a', '--bag-b', '--front-a', '--front-b', '--keyframes-b', '--report', '--truth'):
        p.add_argument(name, required=True)
    p.add_argument('--checks', nargs=2)
    e = sub.add_parser('export', help='write every /gnss/fix of a bag to a CSV (full precision)')
    e.add_argument('--bag', required=True)
    e.add_argument('--out', required=True)
    s = sub.add_parser('split')
    for name in ('--bag', '--front', '--keyframes', '--report'):
        s.add_argument(name, required=True)
    s.add_argument('--gap-s', type=float, default=30.0)
    a = ap.parse_args()
    if a.mode == 'export':
        with open(a.out, 'w') as f:
            f.write('stamp,latitude,longitude,altitude,status,cov_xx,cov_yy\n')
            for r in read_fix_rows(a.bag):
                f.write(','.join(repr(float(v)) for v in r) + '\n')
        return
    if a.mode == 'pair':
        fa, fb = read_fixes(a.bag_a), read_fixes(a.bag_b)
        origin = fa[0]
        sa, da = pairs(fa[:, 0], to_enu(fa, origin), np.loadtxt(a.front_a, comments='#'))
        sb, db = pairs(fb[:, 0], to_enu(fb, origin), np.loadtxt(a.front_b, comments='#'))
        gnss, info = t_gnss(sa, da, sb, db)
        keyframes = np.loadtxt(a.keyframes_b, comments='#')[:, 1:4]
        truth = from_row(json.load(open(a.truth))['debug_exclude_transform'])
        session = next(x for x in yaml.safe_load(open(a.report))['sessions'] if x.get('session') == 1)
        checks = read_checks(*a.checks) if a.checks else None
        out = evaluate(session.get('sets_tried') or [], gnss, truth, keyframes, info, checks)
        out['whole_map_joined'] = session['joined']
    else:
        f = read_fixes(a.bag)
        enu = to_enu(f, f[0])
        kf = np.loadtxt(a.keyframes, comments='#')
        middle = 0.5 * (kf[0, 0] + kf[-1, 0])
        lo, hi = middle - 0.5 * a.gap_s, middle + 0.5 * a.gap_s
        traj = np.loadtxt(a.front, comments='#')
        ia, ib = f[:, 0] < lo, f[:, 0] > hi
        sa, da = pairs(f[ia, 0], enu[ia], traj)
        sb, db = pairs(f[ib, 0], enu[ib], traj)
        gnss, info = t_gnss(sa, da, sb, db)
        keyframes = kf[kf[:, 0] > hi, 1:4]
        split = yaml.safe_load(open(a.report))['split']
        out = evaluate(split.get('sets_tried') or [], gnss, np.eye(4), keyframes, info)
        out['whole_map_joined'] = split['joined']
        out['excluded_by_debug_hook'] = split['excluded_by_debug_hook']
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
