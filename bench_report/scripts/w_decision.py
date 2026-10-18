#!/usr/bin/env python3
"""Chapter 21.4's rule: W (MapsHD pipeline runs scored by mapshd_score.py) against chapter 20's V; Oxford and exp04
are unchanged by construction and take V's values. Also the frontend md5 inside each W run against the screening.
Usage: w_decision.py <W_scores.csv> <W_rows.csv>"""
import csv
import math
import sys

w_scores, w_rows = sys.argv[1:3]
V = '/home/user/lidar_slam_ros2/bench_report/data/frontend_jerk'
seqs = ['calibration-room_trial-1', 'calibration-room_trial-2', 'flat-room_trial-1', 'long-corridor_trial-1',
        'outdoor-urk_trial-1', 'underground-garage_trial-1', 'underground-garage_trial-2', 'underground-garage_trial-3']
load = lambda path, tag, kind: {r[0][:-2]: float(r[5]) for r in csv.reader(open(path))
                                if r[0].endswith('_' + tag) and r[1] == kind and r[2] == 'lidar'}
vp, vf = load(f'{V}/mapshd_scores.csv', 'V', 'pipeline'), load(f'{V}/mapshd_scores.csv', 'V', 'frontend')
wp, wf = load(w_scores, 'W', 'pipeline'), load(w_scores, 'W', 'frontend')
for d in (vp, wp):
    d['oxford'], d['exp04'] = 0.046020142540087225, 0.052096
all10 = seqs + ['oxford', 'exp04']
gm = lambda d: math.exp(sum(math.log(d[s]) for s in all10) / len(all10))
screen = {}
for r in csv.reader(open('/home/user/lidar_slam_ros2/bench_report/data/frontend_corr/sweep_mapshd.csv')):
    if r[1] == 'max_correspondence_distance:=0.25':
        screen[r[0]] = r[-1]
rows = {r[0][:-2]: r for r in csv.reader(open(w_rows))}
print('| sequence | V frontend | W frontend | V pipeline | W pipeline | W / V | frontend md5 = screening |')
print('|---|---:|---:|---:|---:|---:|---|')
md5_ok = True
for s in seqs:
    same = rows[s][7].startswith(screen[s])
    md5_ok &= same
    print(f'| {s} | {vf[s]:.4f} | {wf[s]:.4f} | {vp[s]:.4f} | {wp[s]:.4f} | {wp[s] / vp[s]:.3f} | {"yes" if same else "no"} |')
print(f'| oxford, exp04 | (unchanged) | | {vp["oxford"]:.4f}, {vp["exp04"]:.4f} | same | 1.000 | |')
print(f'| geometric mean (10) | | | {gm(vp):.4f} | {gm(wp):.4f} | {gm(wp) / gm(vp):.3f} | |')
worst = max(wp[s] / vp[s] for s in seqs)
print()
print(f'1. GM lower: {gm(wp) < gm(vp)} ({gm(vp):.4f} -> {gm(wp):.4f})')
print(f'2. no MapsHD trial >= 5% worse: {worst < 1.05} (worst W/V {worst:.3f})')
print(f'3. frontend md5 = screening on every trial: {md5_ok}')
