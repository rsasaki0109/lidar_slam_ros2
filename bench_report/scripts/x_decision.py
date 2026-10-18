#!/usr/bin/env python3
"""Chapter 22.2's rule: X (x_score.sh) against W (MapsHD, chapter 21) and V (Oxford, exp04, MID-360, chapter 20), with
the replay prediction of chapter 22.1 alongside. Usage: x_decision.py <X_dir>"""
import csv
import math
import sys

X = sys.argv[1]
BR = '/home/user/lidar_slam_ros2/bench_report/data'
seqs = ['calibration-room_trial-1', 'calibration-room_trial-2', 'flat-room_trial-1', 'long-corridor_trial-1',
        'outdoor-urk_trial-1', 'underground-garage_trial-1', 'underground-garage_trial-2', 'underground-garage_trial-3']
load = lambda path, tag: {r[0][:-2]: float(r[5]) for r in csv.reader(open(path))
                          if r[0].endswith('_' + tag) and r[1] == 'pipeline' and r[2] == 'lidar'}
base = load(f'{BR}/frontend_corr/mapshd_scores.csv', 'W')
base['oxford'] = float([l.split()[1] for l in open(f'{BR}/frontend_jerk/ape_oxford_V.txt') if l.startswith('rmse:')][0])
base['exp04'] = [float(r[3]) for r in csv.reader(open(f'{BR}/frontend_jerk/ape_exp04.csv')) if r[0] == 'exp04_V'][0]
x = load(f'{X}/mapshd_scores.csv', 'X')
x['oxford'] = float([l.split()[1] for l in open(f'{X}/ape_oxford_X.txt') if l.startswith('rmse:')][0])
x['exp04'] = [float(r[3]) for r in csv.reader(open(f'{X}/ape_exp04.csv')) if r[0] == 'exp04_X'][0]
pred = {r[0][:-2]: float(r[2]) for r in csv.reader(open(f'{BR}/adjacent_trans/replay_w.csv')) if r[1] == 'adj_t=10'}
all10 = seqs + ['oxford', 'exp04']
gm = lambda d: math.exp(sum(math.log(d[s]) for s in all10) / len(all10))
print('| sequence | W / V (run) | X (replay prediction) | X (run) | X / W,V |')
print('|---|---:|---:|---:|---:|')
for s in all10:
    print(f'| {s} | {base[s]:.4f} | {pred[s]:.4f} | {x[s]:.4f} | {x[s] / base[s]:.3f} |')
print(f'| geometric mean | {gm(base):.4f} | {gm(pred):.4f} | {gm(x):.4f} | {gm(x) / gm(base):.3f} |')
worst = max(x[s] / base[s] for s in all10)
n = sum(1 for l in open('/home/user/bench/adj/pipe/mid360_X/trajectory_optimized.tum') if l.strip() and not l.startswith('#'))
start = [l.split() for l in open(f'{X}/mid360_X_loop_residuals.txt')]
start = [l for l in start if int(l[0]) < 30 and int(l[1]) >= n - 60]
th = float(next(csv.reader(open(f'{X}/mid360_map_quality.csv')))[4])
print()
print(f'1. GM lower than W: {gm(x) < gm(base)} ({gm(base):.4f} -> {gm(x):.4f})')
print(f'2. no sequence >= 5% worse: {worst < 1.05} (worst {worst:.3f})')
print(f'3. MID-360 start loop ({n} submaps): {[" ".join(l) for l in start] or "NONE"}')
print(f'4. MID-360 thickness: V 0.044402, X {th:.6f} ({th / 0.044402 - 1:+.2%})')
