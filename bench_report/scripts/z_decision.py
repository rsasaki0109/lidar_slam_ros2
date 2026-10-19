#!/usr/bin/env python3
"""Chapter 24.4's rule: Z (z_score.sh) against Y (chapter 23), with the frontend-only screening of chapter 24.3
alongside (MapsHD; Oxford and exp04 keep their frontend settings). Usage: z_decision.py <Z_dir>"""
import csv
import math
import sys

Z = sys.argv[1]
BR = '/home/user/lidar_slam_ros2/bench_report/data'
seqs = ['calibration-room_trial-1', 'calibration-room_trial-2', 'flat-room_trial-1', 'long-corridor_trial-1',
        'outdoor-urk_trial-1', 'underground-garage_trial-1', 'underground-garage_trial-2', 'underground-garage_trial-3']
load = lambda path, tag, kind: {r[0][:-2]: float(r[5]) for r in csv.reader(open(path))
                                if r[0].endswith('_' + tag) and r[1] == kind and r[2] == 'lidar'}
base = load(f'{BR}/loop_travel/mapshd_scores.csv', 'Y', 'pipeline')
base['oxford'] = float([l.split()[1] for l in open(f'{BR}/loop_travel/ape_oxford_Y.txt') if l.startswith('rmse:')][0])
base['exp04'] = [float(r[3]) for r in csv.reader(open(f'{BR}/loop_travel/ape_exp04.csv')) if r[0] == 'exp04_Y'][0]
z = load(f'{Z}/mapshd_scores.csv', 'Z', 'pipeline')
z['oxford'] = float([l.split()[1] for l in open(f'{Z}/ape_oxford_Z.txt') if l.startswith('rmse:')][0])
z['exp04'] = [float(r[3]) for r in csv.reader(open(f'{Z}/ape_exp04.csv')) if r[0] == 'exp04_Z'][0]
zf = load(f'{Z}/mapshd_scores.csv', 'Z', 'frontend')
screen = {r[0]: (float(r[2]), r[7]) for r in csv.reader(open(f'{BR}/frontend_loc/screening_mapshd.csv'))
          if r[1].endswith('localizability_boost:=10.0 localizability_normal_only:=true')}
all10 = seqs + ['oxford', 'exp04']
gm = lambda d: math.exp(sum(math.log(d[s]) for s in all10) / len(all10))
print('| sequence | Y (run) | Z frontend (screening) | Z frontend (run) | Z (run) | Z / Y |')
print('|---|---:|---:|---:|---:|---:|')
for s in all10:
    sc = f'{screen[s][0]:.4f}' if s in screen else '-'
    fr = f'{zf[s]:.4f}' if s in zf else '-'
    print(f'| {s} | {base[s]:.4f} | {sc} | {fr} | {z[s]:.4f} | {z[s] / base[s]:.3f} |')
print(f'| geometric mean | {gm(base):.4f} | | | {gm(z):.4f} | {gm(z) / gm(base):.3f} |')
worst = max(z[s] / base[s] for s in all10)
n = sum(1 for l in open('/home/user/bench/locz/pipe/mid360_Z/trajectory_optimized.tum') if l.strip() and not l.startswith('#'))
start = [l.split() for l in open(f'{Z}/mid360_Z_loop_residuals.txt')]
start = [l for l in start if int(l[0]) < 30 and int(l[1]) >= n - 60]
th = float(next(csv.reader(open(f'{Z}/mid360_map_quality.csv')))[4])
print()
print(f'1. GM lower than Y: {gm(z) < gm(base)} ({gm(base):.4f} -> {gm(z):.4f})')
print(f'2. no sequence >= 5% worse: {worst < 1.05} (worst {worst:.3f})')
print(f'3. MID-360 start loop ({n} submaps): {[" ".join(l) for l in start] or "NONE"}')
print(f'4. MID-360 thickness: Y 0.044251, Z {th:.6f} ({th / 0.044251 - 1:+.2%})')
rows = {r[0]: r for r in csv.reader(open('/home/user/bench/locz/pipe/rows.csv'))}
md5 = {s: rows[f'{s}_Z'][7][:8] for s in seqs if f'{s}_Z' in rows}
print(f'5. MapsHD frontend md5 equal to the screening: {all(md5[s] == screen[s][1] for s in seqs)} '
      f'({sum(md5[s] == screen[s][1] for s in seqs)}/8)')
