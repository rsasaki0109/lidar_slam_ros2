#!/usr/bin/env python3
"""Chapter 18.4's rule on gate_score.sh's output. Usage: gate_decision.py <score_dir> <rows.csv>"""
import csv
import math
import sys

O, rows_csv = sys.argv[1:3]
FRONT = {'calibration-room_trial-1': 0.0749, 'calibration-room_trial-2': 0.0804, 'flat-room_trial-1': 0.0766,
         'long-corridor_trial-1': 0.0772, 'outdoor-urk_trial-1': 0.0610, 'underground-garage_trial-1': 0.1277,
         'underground-garage_trial-2': 0.0932, 'underground-garage_trial-3': 0.1047, 'oxford': 0.053,
         'exp04': 0.051967}
ape = {c: {} for c in 'RG'}
for r in csv.reader(open(f'{O}/mapshd_scores.csv')):
    if r[1] == 'pipeline' and r[2] == 'lidar':
        ape[r[0][-1]][r[0][:-2]] = float(r[5])
for c in 'RG':
    ape[c]['oxford'] = float([l.split()[1] for l in open(f'{O}/ape_oxford_{c}.txt') if l.startswith('rmse:')][0])
for r in csv.reader(open(f'{O}/ape_exp04.csv')):
    ape[r[0][-1]]['exp04'] = float(r[3])
seqs = list(FRONT)
gm = lambda d: math.exp(sum(math.log(d[s]) for s in seqs) / len(seqs))
rows = {r[0]: r for r in csv.reader(open(rows_csv))}
feat = list(csv.reader(open(f'{O}/loop_features.csv')))
over = {}
for f in feat:
    if f[0].endswith('_R') and float(f[6]) > 0.5 + 0.02 * float(f[4]):
        over.setdefault(f[0][:-2], []).append(f'{f[1]}-{f[2]} ({f[6]} m over {f[4]} m)')
print('| sequence | frontend alone | R | G | G / R | R loops over the cap | G md5 = R md5 |')
print('|---|---:|---:|---:|---:|---|---|')
same_ok = True
for s in seqs + ['mid360']:
    a, b = rows[f'{s}_R'], rows[f'{s}_G']
    same = a[8:11] == b[8:11]
    if s not in over and not same:
        same_ok = False
    if s == 'mid360':
        print(f'| mid360 | – | – | – | – | {", ".join(over.get(s, [])) or "none"} | {"yes" if same else "no"} |')
        continue
    print(f'| {s} | {FRONT[s]:.4f} | {ape["R"][s]:.4f} | {ape["G"][s]:.4f} | {ape["G"][s] / ape["R"][s]:.3f} | '
          f'{", ".join(over.get(s, [])) or "none"} | {"yes" if same else "no"} |')
print(f'| geometric mean | {gm(FRONT):.4f} | {gm(ape["R"]):.4f} | {gm(ape["G"]):.4f} | '
      f'{gm(ape["G"]) / gm(ape["R"]):.3f} | | |')
worst = max(ape['G'][s] / ape['R'][s] for s in seqs)
mq = {r[0]: r for r in csv.reader(open(f'{O}/mid360_map_quality.csv'))}
th_r, th_g = float(mq['mid360_R'][4]), float(mq['mid360_G'][4])
res = [l for l in open(f'{O}/mid360_G_loop_residuals.txt') if l.split()[:2] == ['9', '602']]
print()
print(f'1. GM lower: {gm(ape["G"]) < gm(ape["R"])} ({gm(ape["R"]):.4f} -> {gm(ape["G"]):.4f})')
print(f'2. no sequence >= 2% worse: {worst < 1.02} (worst G/R {worst:.3f})')
print(f'3. md5 equal wherever no R loop is over the cap: {same_ok}')
print(f'4. MID-360 start loop: {res[0].strip() if res else "NOT ACCEPTED"}; thickness R {th_r:.6f} G {th_g:.6f} '
      f'({th_g / th_r - 1:+.2%})')
for c in 'RG':
    print(f'exp04 6 points {c}: {[r for r in csv.reader(open(f"{O}/ape_exp04.csv")) if r[0].endswith(c)][0][4]}')
