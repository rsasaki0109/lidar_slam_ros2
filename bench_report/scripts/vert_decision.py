#!/usr/bin/env python3
"""Chapter 20.4's rule: V (vert_score.sh) against chapter 18's G (gate_score.sh). Usage: vert_decision.py <V_dir> <G_dir>"""
import csv
import math
import sys

V, G = sys.argv[1:3]
seqs = ['calibration-room_trial-1', 'calibration-room_trial-2', 'flat-room_trial-1', 'long-corridor_trial-1',
        'outdoor-urk_trial-1', 'underground-garage_trial-1', 'underground-garage_trial-2', 'underground-garage_trial-3',
        'oxford', 'exp04']


def load(d, tag):
    a, f = {}, {}
    for r in csv.reader(open(f'{d}/mapshd_scores.csv')):
        if r[0].endswith('_' + tag) and r[2] == 'lidar':
            (a if r[1] == 'pipeline' else f)[r[0][:-2]] = float(r[5])
    a['oxford'] = float([l.split()[1] for l in open(f'{d}/ape_oxford_{tag}.txt') if l.startswith('rmse:')][0])
    for r in csv.reader(open(f'{d}/ape_exp04.csv')):
        if r[0].endswith('_' + tag):
            a['exp04'], f['exp04'] = float(r[3]), float(r[1])
    return a, f


va, vf = load(V, 'V')
ga, gf = load(G, 'G')
vf['oxford'] = float([l.split()[1] for l in open(f'{V}/ape_oxford_V_front.txt') if l.startswith('rmse:')][0])
gf['oxford'] = 0.05339362131793644  # chapter 20.3, default frontend alone (same as G's frontend)
gm = lambda d: math.exp(sum(math.log(d[s]) for s in seqs) / len(seqs))
print('| sequence | G frontend | V frontend | G pipeline | V pipeline | V / G (pipeline) |')
print('|---|---:|---:|---:|---:|---:|')
for s in seqs:
    print(f'| {s} | {gf[s]:.4f} | {vf[s]:.4f} | {ga[s]:.4f} | {va[s]:.4f} | {va[s] / ga[s]:.3f} |')
print(f'| geometric mean | {gm(gf):.4f} | {gm(vf):.4f} | {gm(ga):.4f} | {gm(va):.4f} | {gm(va) / gm(ga):.3f} |')
worst = max(va[s] / ga[s] for s in seqs)
n = sum(1 for l in open('/home/user/bench/vert/pipe/mid360_V/trajectory_optimized.tum') if l.strip() and not l.startswith('#'))
start = [l.split() for l in open(f'{V}/mid360_V_loop_residuals.txt')]
start = [l for l in start if int(l[0]) < 30 and int(l[1]) >= n - 60]
th = float(next(csv.reader(open(f'{V}/mid360_map_quality.csv')))[4])
print()
print(f'1. GM lower: {gm(va) < gm(ga)} ({gm(ga):.4f} -> {gm(va):.4f})')
print(f'2. no sequence >= 5% worse: {worst < 1.05} (worst V/G {worst:.3f})')
print(f'3. MID-360 start loop ({n} submaps): {[" ".join(l) for l in start] or "NONE"}')
print(f'4. MID-360 thickness: G 0.044438, V {th:.6f} ({th / 0.044438 - 1:+.2%})')
