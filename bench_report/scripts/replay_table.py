#!/usr/bin/env python3
"""Table of replay_sweep.sh rows: APE per trial, geometric mean, worst ratio to the frontend alone.
Usage: replay_table.py <csv>"""
import collections
import csv
import math
import sys

FRONT = {'calibration-room_trial-1': 0.0749, 'calibration-room_trial-2': 0.0804, 'flat-room_trial-1': 0.0766,
         'long-corridor_trial-1': 0.0772, 'outdoor-urk_trial-1': 0.0610, 'underground-garage_trial-1': 0.1277,
         'underground-garage_trial-2': 0.0932, 'underground-garage_trial-3': 0.1047}
by = collections.OrderedDict()
for run, cfg, v in csv.reader(open(sys.argv[1])):
    by.setdefault(cfg, {})[run.replace('_B', '')] = float(v)
gm = lambda d: math.exp(sum(math.log(d[t]) for t in FRONT) / len(FRONT))
print('| config | ' + ' | '.join(t.replace('underground-garage', 'garage').replace('calibration-room', 'calib')
                                    .replace('_trial-', ' ') for t in FRONT) + ' | GM | worst / frontend |')
print('|---|' + '---:|' * (len(FRONT) + 2))
print('| frontend alone | ' + ' | '.join(f'{FRONT[t]:.4f}' for t in FRONT) + f' | {gm(FRONT):.4f} | 1.000 |')
for cfg, d in by.items():
    if not all(t in d for t in FRONT):
        print(f'| {cfg} | incomplete ({len(d)} trials) |')
        continue
    print(f'| {cfg} | ' + ' | '.join(f'{d[t]:.4f}' for t in FRONT) +
          f' | {gm(d):.4f} | {max(d[t] / FRONT[t] for t in FRONT):.3f} |')
