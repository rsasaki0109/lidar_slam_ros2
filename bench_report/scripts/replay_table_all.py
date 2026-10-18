#!/usr/bin/env python3
"""Table of replay_sweep_all.sh rows: APE per sequence, geometric mean over the ten scored sequences, worst ratio to
the as-run replay, and the MID-360 start-loop residual. Usage: replay_table_all.py <csv>"""
import collections
import csv
import math
import sys

SEQ = ['calibration-room_trial-1', 'calibration-room_trial-2', 'flat-room_trial-1', 'long-corridor_trial-1',
       'outdoor-urk_trial-1', 'underground-garage_trial-1', 'underground-garage_trial-2', 'underground-garage_trial-3',
       'oxford', 'exp04']
FRONT = [0.0749, 0.0804, 0.0766, 0.0772, 0.0610, 0.1277, 0.0932, 0.1047, 0.053, 0.051967]
by = collections.OrderedDict()
for run, cfg, v in csv.reader(open(sys.argv[1])):
    by.setdefault(cfg, {})[run[:-2]] = float(v)
short = lambda t: t.replace('underground-garage', 'garage').replace('calibration-room', 'calib').replace('_trial-', ' ')
gm = lambda vals: math.exp(sum(math.log(v) for v in vals) / len(vals))
print('| config | ' + ' | '.join(short(t) for t in SEQ) + ' | GM | worst / as run | MID-360 start loop residual |')
print('|---|' + '---:|' * (len(SEQ) + 3))
print('| frontend alone | ' + ' | '.join(f'{v:.4f}' for v in FRONT) + f' | {gm(FRONT):.4f} | – | 3.915 |')
base = by.get('as_run')
for cfg, d in by.items():
    if not all(t in d for t in SEQ):
        print(f'| {cfg} | incomplete |')
        continue
    worst = max(d[t] / base[t] for t in SEQ) if base else float('nan')
    print(f'| {cfg} | ' + ' | '.join(f'{d[t]:.4f}' for t in SEQ) + f' | {gm([d[t] for t in SEQ]):.4f} | {worst:.3f} | '
          f'{d.get("mid360", float("nan")):.3f} |')
