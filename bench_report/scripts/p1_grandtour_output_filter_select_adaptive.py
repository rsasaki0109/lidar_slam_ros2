#!/usr/bin/env python3
"""
Pick an adaptive output filter setting (chapter 36, screen_hs_p_2026-10-28/plan.txt) from the replayed grid.

Reads grid.csv (p1_grandtour_output_filter_sweep.py --csv, settings 'mode sigma q_a q_b alpha lo hi' and HS-G15g's
'sigma q_a q_b' first); mean ATE over the tuning half; the lowest, then among those within 0.1 mm the simplest
(sigma 0.02, then alpha 0.5, then [0.25, 4], then scalar < shape < marginal); a candidate only if 0.1 mm below
HS-G15g. Prints the ten best with tuning / held-out / all means, and the pick per mission.
Usage: p1_grandtour_output_filter_select_adaptive.py <grid.csv>
"""
import csv
import sys
from collections import defaultdict

TUNING = ['heap-1', 'pil-2', 'con-1', 'eig-2', 'arc-3', 'snow-3', 'arc-5', 'arc-4']
HELD_OUT = ['spx-3', 'spx-1', 'arc-1', 'con-2', 'con-3', 'ice-1', 'arc-6']
BASE = '0.02 0.003 1e-05'


def simplicity(setting):
    f = setting.split()
    if len(f) != 7:
        return (9,)
    return (f[1] != '0.02', f[4] != '0.5', f[5] != '0.25', ['scalar', 'shape', 'marginal'].index(f[0]))


def main():
    ate = defaultdict(dict)
    for row in csv.DictReader(open(sys.argv[1])):
        ate[row['setting']][row['mission']] = float(row['ate_mm'])
    missing = [m for m in TUNING if m not in ate[BASE]]
    if missing:
        sys.exit(f'tuning missions not in the grid yet: {missing}')

    def mean(s, missions):
        v = [ate[s][m] for m in missions if m in ate[s]]
        return sum(v) / len(v) if v else float('nan')

    order = sorted(ate, key=lambda s: mean(s, TUNING))
    best = mean(order[0], TUNING)
    pick = min((s for s in order if mean(s, TUNING) <= best + 0.1), key=simplicity)
    held = [m for m in HELD_OUT if m in ate[BASE]]
    print(f'held-out missions in the grid: {len(held)} of {len(HELD_OUT)}')
    print(f'{"setting":34s} tuning  held-out  all  (mean ATE, mm)')
    for s in [BASE] + order[:10] + ([pick] if pick not in order[:10] else []):
        mark = ' HS-G15g' if s == BASE else (' picked' if s == pick else '')
        print(f'{s:34s} {mean(s, TUNING):6.2f}  {mean(s, held):8.2f}  {mean(s, TUNING + held):5.2f}{mark}')
    gain = mean(BASE, TUNING) - mean(pick, TUNING)
    print(f'pick {pick}: tuning gain {gain:.2f} mm -> {"candidate" if gain >= 0.1 else "no candidate (below 0.1 mm)"}')
    for m in TUNING + held:
        print(f'  {m:7s} {ate[BASE][m]:5.1f} -> {ate[pick][m]:5.1f}{"  (held out)" if m in held else ""}')


if __name__ == '__main__':
    main()
