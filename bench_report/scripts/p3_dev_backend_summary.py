#!/usr/bin/env python3
"""
Summary of p3_dev_backend.sh runs (Phase 3, robust loops on MapsHD, Oxford, exp04; plan in
data/loops_robust_dev_2026-10-29/plan.txt): per variant, each sequence's APE and its ratio to the frontend's, the
largest ratio, the geometric mean of the APEs, and the plan's two readings (no sequence more than 5 % above the
frontend; geometric mean below the frontend's).
Usage: p3_dev_backend_summary.py <label> [scores.csv]
"""
import csv
import math
import sys

label = sys.argv[1]
path = sys.argv[2] if len(sys.argv) > 2 else '/home/user/bench/dev_loops/scores.csv'
rows = [r for r in csv.reader(open(path)) if r and r[0] == label]
ape, loops = {}, {}
for r in rows:
    ape[(r[1], r[2])] = float(r[3])
    loops[(r[1], r[2])] = r[4]
seqs = [s for s in dict.fromkeys(r[1] for r in rows)]
variants = [v for v in dict.fromkeys(r[2] for r in rows)]


def gmean(values):
    return math.exp(sum(math.log(v) for v in values) / len(values))


front = {s: ape[(s, 'front')] for s in seqs if (s, 'front') in ape}
print('sequence'.ljust(28) + ''.join(v.rjust(18) for v in variants))
for s in seqs:
    cells = []
    for v in variants:
        if (s, v) not in ape:
            cells.append('-'.rjust(18))
        elif v == 'front':
            cells.append(f'{ape[(s, v)]:.4f}'.rjust(18))
        else:
            cells.append(f'{ape[(s, v)]:.4f} {ape[(s, v)] / front[s]:5.3f} ({loops[(s, v)]})'.rjust(18))
    print(s.ljust(28) + ''.join(cells))
complete = [s for s in seqs if all((s, v) in ape for v in variants)]
print(f'geometric mean over {len(complete)}'.ljust(28) +
      ''.join(f'{gmean([ape[(s, v)] for s in complete]):.4f}'.rjust(18) for v in variants))
g_front = gmean([front[s] for s in complete])
for v in variants:
    if v == 'front':
        continue
    worst = max(complete, key=lambda s: ape[(s, v)] / front[s])
    g = gmean([ape[(s, v)] for s in complete])
    print(f'{v}: largest ratio to the frontend {ape[(worst, v)] / front[worst]:.3f} ({worst}); '
          f'does not hurt (<= 1.05): {ape[(worst, v)] / front[worst] <= 1.05}; '
          f'helps (gmean {g:.4f} < frontend {g_front:.4f}): {g < g_front}')
