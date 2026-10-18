#!/usr/bin/env python3
"""Summary of the MapsHD scores (mapshd_score.py rows) and the preregistered decision of report chapter 14.6.

B (NDT 2 m) is adopted only if (1) the geometric mean of the pipeline APE over all trials is lower than A's and
(2) no trial is 10% or more worse than A. The APE is evo_ape -a in the LiDAR frame (the benchmark's method).
Usage: mapshd_summary.py <scores.csv> <rows.csv>
"""
import csv
import math
import sys

scores = list(csv.DictReader(open(sys.argv[1])))
runs = {r[0]: r for r in csv.reader(open(sys.argv[2]))}


def get(run, kind, frame, col='evo_rmse'):
    for s in scores:
        if s['run'] == run and s['trajectory'] == kind and s['frame'] == frame:
            return float(s[col])
    return float('nan')


trials = sorted({s['run'][:-2] for s in scores if s['run'].endswith(('_A', '_B'))})
print('| trial | frontend | A (NDT 5 m) | B (NDT 2 m) | B / A | evo pairs | loops A / B | complete A / B (s) |')
print('|---|---:|---:|---:|---:|---:|---:|---:|')
ratios, a_all, b_all = [], [], []
for t in trials:
    f = get(t + '_A', 'frontend', 'lidar')
    a, b = get(t + '_A', 'pipeline', 'lidar'), get(t + '_B', 'pipeline', 'lidar')
    pairs = int(get(t + '_A', 'pipeline', 'lidar', 'evo_pairs'))
    ra, rb = runs.get(t + '_A'), runs.get(t + '_B')
    ratios.append(b / a); a_all.append(a); b_all.append(b)
    print(f'| {t} | {f:.4f} | {a:.4f} | {b:.4f} | {b / a:.3f} | {pairs} | {ra[6]} / {rb[6]} | {float(ra[2]):.1f} / {float(rb[2]):.1f} |')
gm = lambda v: math.exp(sum(math.log(x) for x in v) / len(v))
print(f'\ngeometric mean: A {gm(a_all):.4f} m, B {gm(b_all):.4f} m (B / A {gm(b_all) / gm(a_all):.3f}); '
      f'worst trial B / A {max(ratios):.3f}')
ok = gm(b_all) < gm(a_all) and max(ratios) < 1.10
print('decision: B adopted' if ok else 'decision: B not adopted')
