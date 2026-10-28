#!/usr/bin/env python3
"""
Pick an output filter setting from the replayed grid (chapter 35, filter_sweep_2026-10-28/plan.txt).

Reads grid.csv (p1_grandtour_output_filter_sweep.py --csv): the mean ATE over the tuning half for every setting; the
lowest, and among the settings within 0.1 mm of it the one nearest to the current setting (fewest grid steps, the
sum over the three axes); prints the ten best settings with their tuning, held-out and all-mission means, and the
picked one per mission.
Usage: p1_grandtour_output_filter_select.py <grid.csv>
"""
import csv
import sys
from collections import defaultdict

TUNING = ['heap-1', 'pil-2', 'con-1', 'eig-2', 'arc-3', 'snow-3', 'arc-5', 'arc-4']
HELD_OUT = ['spx-3', 'spx-1', 'arc-1', 'con-2', 'con-3', 'ice-1', 'arc-6']
AXES = ([0.005, 0.01, 0.015, 0.02, 0.03], [0.0003, 0.001, 0.003, 0.01], [1e-5, 1e-4, 1e-3])
CURRENT = (0.02, 0.001, 1e-4)


def steps(setting):
    return sum(abs(min(range(len(a)), key=lambda i: abs(a[i] - v)) - min(range(len(a)), key=lambda i: abs(a[i] - c)))
               for a, v, c in zip(AXES, setting, CURRENT))


def main():
    ate = defaultdict(dict)
    for row in csv.DictReader(open(sys.argv[1])):
        setting = (float(row['position_sigma']), float(row['acceleration_noise']), float(row['bias_noise']))
        ate[setting][row['mission']] = float(row['ate_mm'])
    missing = [m for m in TUNING if m not in ate[CURRENT]]
    if missing:
        sys.exit(f'tuning missions not in the grid yet: {missing}')

    def mean(setting, missions):
        values = [ate[setting][m] for m in missions if m in ate[setting]]
        return sum(values) / len(values) if values else float('nan')

    settings = sorted(ate, key=lambda s: mean(s, TUNING))
    best = mean(settings[0], TUNING)
    picked = min((s for s in settings if mean(s, TUNING) <= best + 0.1), key=lambda s: (steps(s), mean(s, TUNING)))
    held = [m for m in HELD_OUT if m in ate[CURRENT]]
    print(f'held-out missions in the grid: {len(held)} of {len(HELD_OUT)}')
    print('sigma   q_a     q_b     steps  tuning  held-out  all  (mean ATE, mm)')
    for s in [CURRENT] + settings[:10] + ([picked] if picked not in settings[:10] else []):
        mark = ' current' if s == CURRENT else (' picked' if s == picked else '')
        print(f'{s[0]:<7g} {s[1]:<7g} {s[2]:<7g} {steps(s):5d}  {mean(s, TUNING):6.2f}  {mean(s, held):8.2f}  '
              f'{mean(s, TUNING + held):5.2f}{mark}')
    print('per mission, current -> picked (mm):')
    for m in TUNING + held:
        print(f'  {m:7s} {ate[CURRENT][m]:5.1f} -> {ate[picked][m]:5.1f}{"  (held out)" if m in held else ""}')


if __name__ == '__main__':
    main()
