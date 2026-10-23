#!/usr/bin/env python3
"""Chapter 27.3's rule: D (d_score.sh) against C (chapter 25, data/loop_constant), with the replay of 27.1 alongside
for the two garage trials it covers. Usage: d_decision.py <D_dir>"""
import csv
import math
import sys

D = sys.argv[1]
BR = '/home/user/lidar_slam_ros2/bench_report/data'
seqs = ['calibration-room_trial-1', 'calibration-room_trial-2', 'flat-room_trial-1', 'long-corridor_trial-1',
        'outdoor-urk_trial-1', 'underground-garage_trial-1', 'underground-garage_trial-2', 'underground-garage_trial-3']
load = lambda path, tag: {r[0][:-2]: float(r[5]) for r in csv.reader(open(path))
                          if r[0].endswith('_' + tag) and r[1] == 'pipeline' and r[2] == 'lidar'}
base = load(f'{BR}/loop_constant/mapshd_scores.csv', 'C')
base['oxford'] = float([l.split()[1] for l in open(f'{BR}/loop_constant/ape_oxford_C.txt') if l.startswith('rmse:')][0])
base['exp04'] = [float(r[3]) for r in csv.reader(open(f'{BR}/loop_constant/ape_exp04.csv')) if r[0] == 'exp04_C'][0]
d = load(f'{D}/mapshd_scores.csv', 'D')
d['oxford'] = float([l.split()[1] for l in open(f'{D}/ape_oxford_D.txt') if l.startswith('rmse:')][0])
d['exp04'] = [float(r[3]) for r in csv.reader(open(f'{D}/ape_exp04.csv')) if r[0] == 'exp04_D'][0]
pred = {r[0][:-2]: float(r[2]) for r in csv.reader(open(f'{BR}/loop_ndt1/replay_garage.csv'))
        if r[1] == 'meas=offline_res1'}
all10 = seqs + ['oxford', 'exp04']
gm = lambda x: math.exp(sum(math.log(x[s]) for s in all10) / len(all10))
print('| sequence | C (run) | D (replay, garage only) | D (run) | D / C |')
print('|---|---:|---:|---:|---:|')
for s in all10:
    p = f'{pred[s]:.4f}' if s in pred else '–'
    print(f'| {s} | {base[s]:.4f} | {p} | {d[s]:.4f} | {d[s] / base[s]:.3f} |')
print(f'| geometric mean | {gm(base):.4f} | – | {gm(d):.4f} | {gm(d) / gm(base):.3f} |')
worst = max(d[s] / base[s] for s in all10)
n = sum(1 for l in open('/home/user/bench/loopd/pipe/mid360_D/trajectory_optimized.tum') if l.strip() and not l.startswith('#'))
start = [l.split() for l in open(f'{D}/mid360_D_loop_residuals.txt')]
start = [l for l in start if int(l[0]) < 30 and int(l[1]) >= n - 60]
th = float(next(csv.reader(open(f'{D}/mid360_map_quality.csv')))[4])
print()
print(f'1. GM lower than C: {gm(d) < gm(base)} ({gm(base):.4f} -> {gm(d):.4f})')
print(f'2. no sequence >= 5% worse: {worst < 1.05} (worst {worst:.3f})')
print(f'3. MID-360 start loop ({n} submaps): {[" ".join(l) for l in start] or "NONE"}')
print(f'4. MID-360 thickness: C 0.044162, D {th:.6f} ({th / 0.044162 - 1:+.2%})')
cr = {r[0][:-2]: r for r in csv.reader(open(f'{BR}/loop_constant/runs.csv')) if r[0].endswith('_C')}
dr = {r[0][:-2]: r for r in csv.reader(open('/home/user/bench/loopd/pipe/rows.csv'))}
same = [k for k in cr if k in dr and cr[k][7] == dr[k][7]]
print(f'5. frontend md5 equal to C: {len(same) == len(cr) == len(dr)} ({len(same)}/{len(cr)})')
