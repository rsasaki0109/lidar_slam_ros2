#!/usr/bin/env python3
"""Chapter 23.2's rule: Y (y_score.sh) against X (chapter 22), with the replay prediction of chapter 23.1 alongside.
Usage: y_decision.py <Y_dir>"""
import csv
import math
import sys

Y = sys.argv[1]
BR = '/home/user/lidar_slam_ros2/bench_report/data'
seqs = ['calibration-room_trial-1', 'calibration-room_trial-2', 'flat-room_trial-1', 'long-corridor_trial-1',
        'outdoor-urk_trial-1', 'underground-garage_trial-1', 'underground-garage_trial-2', 'underground-garage_trial-3']
load = lambda path, tag: {r[0][:-2]: float(r[5]) for r in csv.reader(open(path))
                          if r[0].endswith('_' + tag) and r[1] == 'pipeline' and r[2] == 'lidar'}
base = load(f'{BR}/adjacent_trans/mapshd_scores.csv', 'X')
base['oxford'] = float([l.split()[1] for l in open(f'{BR}/adjacent_trans/ape_oxford_X.txt') if l.startswith('rmse:')][0])
base['exp04'] = [float(r[3]) for r in csv.reader(open(f'{BR}/adjacent_trans/ape_exp04.csv')) if r[0] == 'exp04_X'][0]
y = load(f'{Y}/mapshd_scores.csv', 'Y')
y['oxford'] = float([l.split()[1] for l in open(f'{Y}/ape_oxford_Y.txt') if l.startswith('rmse:')][0])
y['exp04'] = [float(r[3]) for r in csv.reader(open(f'{Y}/ape_exp04.csv')) if r[0] == 'exp04_Y'][0]
pred = {r[0][:-2]: float(r[2]) for r in csv.reader(open(f'{BR}/loop_travel/replay_x.csv')) if r[1] == 'travel_min=60'}
all10 = seqs + ['oxford', 'exp04']
gm = lambda d: math.exp(sum(math.log(d[s]) for s in all10) / len(all10))
print('| sequence | X (run) | Y (replay prediction) | Y (run) | Y / X |')
print('|---|---:|---:|---:|---:|')
for s in all10:
    print(f'| {s} | {base[s]:.4f} | {pred[s]:.4f} | {y[s]:.4f} | {y[s] / base[s]:.3f} |')
print(f'| geometric mean | {gm(base):.4f} | {gm(pred):.4f} | {gm(y):.4f} | {gm(y) / gm(base):.3f} |')
worst = max(y[s] / base[s] for s in all10)
n = sum(1 for l in open('/home/user/bench/loopdist/pipe/mid360_Y/trajectory_optimized.tum') if l.strip() and not l.startswith('#'))
start = [l.split() for l in open(f'{Y}/mid360_Y_loop_residuals.txt')]
start = [l for l in start if int(l[0]) < 30 and int(l[1]) >= n - 60]
th = float(next(csv.reader(open(f'{Y}/mid360_map_quality.csv')))[4])
print()
print(f'1. GM lower than X: {gm(y) < gm(base)} ({gm(base):.4f} -> {gm(y):.4f})')
print(f'2. no sequence >= 5% worse: {worst < 1.05} (worst {worst:.3f})')
print(f'3. MID-360 start loop ({n} submaps): {[" ".join(l) for l in start] or "NONE"}')
print(f'4. MID-360 thickness: X 0.044223, Y {th:.6f} ({th / 0.044223 - 1:+.2%})')
