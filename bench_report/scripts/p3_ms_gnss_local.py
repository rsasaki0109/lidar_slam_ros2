#!/usr/bin/env python3
"""The local GNSS gate of plan multisession_gnss_2026-11-07/plan_local.txt, evaluated offline.

Run with the ROS environment sourced and `python3 -P`. Fix reading, pairing and the trimmed 4-DoF fit come from
p3_ms_gnss_gate.py (the same directory).

pair:  p3_ms_gnss_local.py pair --bag-a BAG --bag-b BAG --front-a TUM --front-b TUM --keyframes-a TUM
                                --keyframes-b TUM --report join_report.yaml --truth alone.json
                                [--checks debug_checks.yaml perturb.csv]
split: p3_ms_gnss_local.py split --bag BAG --front TUM --keyframes TUM --report join_report.yaml [--gap-s 30]

For a transform T, the places are B's keyframes that T puts within 3 m of one of A's keyframes. At each place, A's and
B's pairs within 30 m (odometry) of the two keyframes are fitted (at least 50 each) and the place's disagreement is
|T p_b - T_loc p_b|. The gate: at least 5 measured places, median disagreement at most 1.0 m, and the whole-map floor.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p3_ms_gnss_gate as g  # noqa: E402

PLACE_RADIUS_M = 3.0
WINDOW_M = 30.0
MIN_WINDOW_PAIRS = 50
MIN_MEASURED = 5
MAX_MEDIAN_M = 1.0


class Local:
    """Local fits around keyframes, kept per keyframe."""

    def __init__(self, src, dst, keyframes):
        self.src, self.dst, self.keyframes = src, dst, keyframes
        self.tree = cKDTree(src) if len(src) else None
        self.cache = {}

    def fit(self, k):
        if k not in self.cache:
            idx = self.tree.query_ball_point(self.keyframes[k], WINDOW_M) if self.tree is not None else []
            self.cache[k] = g.trimmed_fit(self.src[idx], self.dst[idx])[0] if len(idx) >= MIN_WINDOW_PAIRS else None
        return self.cache[k]


def local_check(t, kfa, kfb, la, lb):
    placed = kfb @ t[:3, :3].T + t[:3, 3]
    dist, nearest = cKDTree(kfa).query(placed, k=1)
    places = np.flatnonzero(dist <= PLACE_RADIUS_M)
    d = []
    for b in places:
        ta, tb = la.fit(int(nearest[b])), lb.fit(int(b))
        if ta is None or tb is None:
            continue
        loc = np.linalg.inv(ta) @ tb
        d.append(float(np.linalg.norm(placed[b] - (loc[:3, :3] @ kfb[b] + loc[:3, 3]))))
    out = {'local_places': int(len(places)), 'measured': len(d)}
    if d:
        out.update({'median_m': round(float(np.median(d)), 3), 'p90_m': round(float(np.percentile(d, 90)), 3)})
    out['local_ok'] = len(d) >= MIN_MEASURED and out.get('median_m', math.inf) <= MAX_MEDIAN_M
    return out


def judge(t, places, share_below, truth, kfa, kfb, la, lb):
    out = local_check(t, kfa, kfb, la, lb)
    out['vs_truth_m'], out['vs_truth_deg'] = g.disagreement(t, truth, kfb)
    out['places'], out['share_below'] = places, round(share_below, 4)
    out['floor'] = places >= g.FLOOR_MIN_PLACES and share_below <= g.FLOOR_MAX_SHARE
    out['gate'] = out['local_ok'] and out['floor']
    return out


def evaluate(sets, truth, kfa, kfb, la, lb, checks=None):
    out = {'pairs_a': int(len(la.src)), 'pairs_b': int(len(lb.src)),
           'truth': local_check(truth, kfa, kfb, la, lb), 'sets': [], 'joins': None}
    for i, s in enumerate(sets):
        j = judge(g.from_row(s['transform']), s['places'], s['share_below'], truth, kfa, kfb, la, lb)
        j.update({'set': i, 'size': s['size'], 'whole_map_passed': s['passed']})
        out['sets'].append(j)
        if j['gate'] and out['joins'] is None:
            out['joins'] = i
    if checks:
        out['checks'] = []
        for c in checks:
            j = judge(g.from_row(c['transform']), c['places'], c['share_below'], truth, kfa, kfb, la, lb)
            j.update({k: c[k] for k in ('index', 'kind', 'axis', 'amount')})
            out['checks'].append(j)
    return out


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='mode', required=True)
    p = sub.add_parser('pair')
    for name in ('--bag-a', '--bag-b', '--front-a', '--front-b', '--keyframes-a', '--keyframes-b', '--report',
                 '--truth'):
        p.add_argument(name, required=True)
    p.add_argument('--checks', nargs=2)
    s = sub.add_parser('split')
    for name in ('--bag', '--front', '--keyframes', '--report'):
        s.add_argument(name, required=True)
    s.add_argument('--gap-s', type=float, default=30.0)
    a = ap.parse_args()
    if a.mode == 'pair':
        fa, fb = g.read_fixes(a.bag_a), g.read_fixes(a.bag_b)
        origin = fa[0]
        sa, da = g.pairs(fa[:, 0], g.to_enu(fa, origin), np.loadtxt(a.front_a, comments='#'))
        sb, db = g.pairs(fb[:, 0], g.to_enu(fb, origin), np.loadtxt(a.front_b, comments='#'))
        kfa = np.loadtxt(a.keyframes_a, comments='#')[:, 1:4]
        kfb = np.loadtxt(a.keyframes_b, comments='#')[:, 1:4]
        truth = g.from_row(json.load(open(a.truth))['debug_exclude_transform'])
        session = next(x for x in yaml.safe_load(open(a.report))['sessions'] if x.get('session') == 1)
        checks = g.read_checks(*a.checks) if a.checks else None
        out = evaluate(session.get('sets_tried') or [], truth, kfa, kfb, Local(sa, da, kfa), Local(sb, db, kfb),
                       checks)
        out['whole_map_joined'] = session['joined']
    else:
        f = g.read_fixes(a.bag)
        enu = g.to_enu(f, f[0])
        kf = np.loadtxt(a.keyframes, comments='#')
        middle = 0.5 * (kf[0, 0] + kf[-1, 0])
        lo, hi = middle - 0.5 * a.gap_s, middle + 0.5 * a.gap_s
        traj = np.loadtxt(a.front, comments='#')
        ia, ib = f[:, 0] < lo, f[:, 0] > hi
        sa, da = g.pairs(f[ia, 0], enu[ia], traj)
        sb, db = g.pairs(f[ib, 0], enu[ib], traj)
        kfa, kfb = kf[kf[:, 0] < lo, 1:4], kf[kf[:, 0] > hi, 1:4]
        split = yaml.safe_load(open(a.report))['split']
        out = evaluate(split.get('sets_tried') or [], np.eye(4), kfa, kfb, Local(sa, da, kfa), Local(sb, db, kfb))
        out['whole_map_joined'] = split['joined']
        out['excluded_by_debug_hook'] = split['excluded_by_debug_hook']
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
