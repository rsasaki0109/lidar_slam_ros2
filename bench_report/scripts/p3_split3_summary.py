#!/usr/bin/env python3
"""
Summary of p3_split3_join_eval.sh results (multi-session step 7, data/grandtour/multisession_2026-10-29/plan.txt).

Per session: the parts joined to the map and to the first part (positive runs); the parts that join only with the map
or to a map of more than part 0; every wrong join (a join in a stress run, or a part more than 0.3 m from the
identity); the plan's decision.
Usage: p3_split3_summary.py <results.csv>
"""
import csv
import re
import sys

rows = [r for r in csv.reader(open(sys.argv[1])) if len(r) >= 7]
by = {(r[0], r[1], r[2]): r for r in rows}
names = list(dict.fromkeys(r[0] for r in rows))
wrong, fewer = [], []
total = {'map': 0, 'first': 0}
print('session                       parts        map: part 1 / part 2                       first: part 1 / part 2')
for n in names:
    joined = {}
    for to in ('map', 'first'):
        r = by.get((n, 'positive', to))
        if r is None:
            continue
        joined[to] = [c for c in r[4:6] if c.startswith('joined')]
        total[to] += len(joined[to])
        for c in r[4:6]:
            m = re.match(r'joined p\d+ map\[[\d ]+\] ([\d.]+)', c)
            if m and float(m.group(1)) > 0.3:
                wrong.append(f'{n} positive {to}: {c}')
    for to in ('map', 'first'):
        r = by.get((n, 'stress', to))
        if r is not None:
            wrong += [f'{n} stress {to}: {c}' for c in r[4:6] if c.startswith('joined')]
    if 'map' in joined and 'first' in joined and len(joined['map']) < len(joined['first']):
        fewer.append(n)
    pm, pf = by.get((n, 'positive', 'map')), by.get((n, 'positive', 'first'))
    print(f'{n:30s}{(pm or pf)[3]:13s}{" / ".join(pm[4:6]) if pm else "-":43s}{" / ".join(pf[4:6]) if pf else "-"}')
print(f'parts joined: map {total["map"]}, first {total["first"]}')
print('joined with the map but not with the first part, and joined to a map of more than part 0:')
for n in names:
    rm, rf = by.get((n, 'positive', 'map')), by.get((n, 'positive', 'first'))
    if rm and rf:
        for k, (cm, cf) in enumerate(zip(rm[4:6], rf[4:6]), 1):
            if cm.startswith('joined') and not cf.startswith('joined'):
                print(f'  {n} part {k}: only with the map: {cm}')
            elif cm.startswith('joined') and not re.match(r'joined p\d+ map\[0\]', cm):
                print(f'  {n} part {k}: map {cm}; first {cf}')
print(f'wrong joins: {len(wrong)}', *wrong, sep='\n  ')
print(f'sessions where the map joins fewer parts than first: {fewer or "none"}')
print('decision (with (a) met): map stays the default' if not wrong and not fewer else
      'decision: first becomes the default')
