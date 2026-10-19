#!/usr/bin/env python3
"""Chapter 25.2's rule: C (c_score.sh) against Z (chapter 24), with the replay prediction of chapter 25.1 alongside.
Usage: c_decision.py <C_dir>"""
import csv
import math
import sys

C = sys.argv[1]
BR = '/home/user/lidar_slam_ros2/bench_report/data'
seqs = ['calibration-room_trial-1', 'calibration-room_trial-2', 'flat-room_trial-1', 'long-corridor_trial-1',
        'outdoor-urk_trial-1', 'underground-garage_trial-1', 'underground-garage_trial-2', 'underground-garage_trial-3']
load = lambda path, tag: {r[0][:-2]: float(r[5]) for r in csv.reader(open(path))
                          if r[0].endswith('_' + tag) and r[1] == 'pipeline' and r[2] == 'lidar'}
base = load(f'{BR}/frontend_loc/mapshd_scores.csv', 'Z')
base['oxford'] = float([l.split()[1] for l in open(f'{BR}/frontend_loc/ape_oxford_Z.txt') if l.startswith('rmse:')][0])
base['exp04'] = [float(r[3]) for r in csv.reader(open(f'{BR}/frontend_loc/ape_exp04.csv')) if r[0] == 'exp04_Z'][0]
c = load(f'{C}/mapshd_scores.csv', 'C')
c['oxford'] = float([l.split()[1] for l in open(f'{C}/ape_oxford_C.txt') if l.startswith('rmse:')][0])
c['exp04'] = [float(r[3]) for r in csv.reader(open(f'{C}/ape_exp04.csv')) if r[0] == 'exp04_C'][0]
pred = {r[0][:-2]: float(r[2]) for r in csv.reader(open(f'{BR}/loop_constant/replay_z.csv'))
        if r[1] == 'loop_t=200 loop_r=100000'}
all10 = seqs + ['oxford', 'exp04']
gm = lambda d: math.exp(sum(math.log(d[s]) for s in all10) / len(all10))
print('| sequence | Z (run) | C (replay prediction) | C (run) | C / Z |')
print('|---|---:|---:|---:|---:|')
for s in all10:
    print(f'| {s} | {base[s]:.4f} | {pred[s]:.4f} | {c[s]:.4f} | {c[s] / base[s]:.3f} |')
print(f'| geometric mean | {gm(base):.4f} | {gm(pred):.4f} | {gm(c):.4f} | {gm(c) / gm(base):.3f} |')
worst = max(c[s] / base[s] for s in all10)
n = sum(1 for l in open('/home/user/bench/loopc/pipe/mid360_C/trajectory_optimized.tum') if l.strip() and not l.startswith('#'))
start = [l.split() for l in open(f'{C}/mid360_C_loop_residuals.txt')]
start = [l for l in start if int(l[0]) < 30 and int(l[1]) >= n - 60]
th = float(next(csv.reader(open(f'{C}/mid360_map_quality.csv')))[4])
print()
print(f'1. GM lower than Z: {gm(c) < gm(base)} ({gm(base):.4f} -> {gm(c):.4f})')
print(f'2. no sequence >= 5% worse: {worst < 1.05} (worst {worst:.3f})')
print(f'3. MID-360 start loop ({n} submaps): {[" ".join(l) for l in start] or "NONE"}')
print(f'4. MID-360 thickness: Z 0.044251, C {th:.6f} ({th / 0.044251 - 1:+.2%})')
zr = {r[0][:-2]: r for r in csv.reader(open('/home/user/bench/locz/pipe/rows.csv'))}
cr = {r[0][:-2]: r for r in csv.reader(open('/home/user/bench/loopc/pipe/rows.csv'))}
same = [k for k in zr if k in cr and zr[k][7] == cr[k][7]]
print(f'5. frontend md5 equal to Z: {len(same) == len(zr) == len(cr)} ({len(same)}/{len(zr)})')
