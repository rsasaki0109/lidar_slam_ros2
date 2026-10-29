#!/usr/bin/env python3
"""
Summary of loop registration variants on GrandTour development missions (Phase 3, robust loops): per variant the
loop edges found and scored (p3_grandtour_loop_errors.py rows in <run>/loops.csv), the median and largest loop error,
how many are above 0.1 m, the frontend's error for the same pairs, and the causal ATE against the frontend's.
Usage: p3_grandtour_loop_summary.py <frontend label> <variant label> ... -- <mission> ...
"""
import csv
import os
import subprocess
import sys

import numpy as np

B = '/home/user/bench/grandtour'
S = '/home/user/lidar_slam_ros2/bench_report/scripts'
PY = '/home/user/mapshd/venv/bin/python'


def ate(tum, mission):
    out = subprocess.run([PY, '-I', f'{S}/p1_grandtour_ate.py', tum,
                          f'/home/user/data/grandtour/reference_dev/{mission}/prism_position'],
                         capture_output=True, text=True).stdout
    return float(out.strip().split()[-1]) if out.strip() else float('nan')


def main():
    args = sys.argv[1:]
    split = args.index('--')
    front_label, variants, missions = args[0], args[1:split], args[split + 1:]
    front = {m: ate(f'{B}/{front_label}/{m}/{m}.tum', m) for m in missions}
    print('variant      edges scored  loop med / max mm  >0.1m  frontend med mm  loop<front  ATE causal - frontend (mm) per mission')
    for v in variants:
        rows, ates = [], []
        for m in missions:
            p = f'{B}/{v}/{m}/loops.csv'
            if os.path.exists(p):
                rows += [r for r in csv.reader(open(p)) if r]
            t = f'{B}/{v}/{m}/{m}_causal.tum'
            ates.append((ate(t, m) - front[m]) * 1000 if os.path.exists(t) else float('nan'))
        sc = [r for r in rows if r[7] != 'nan']
        le = np.array([float(r[7]) for r in sc]) * 1000
        fe = np.array([float(r[8]) for r in sc]) * 1000
        s = (f'{v:12s} {len(rows):5d} {len(sc):6d}  ' + (f'{np.median(le):6.1f} / {le.max():6.0f}  {int((le > 100).sum()):5d}  '
             f'{np.median(fe):10.1f}       {int((le < fe).sum()):3d}/{len(sc):<3d}' if len(sc) else ' ' * 48))
        print(s + '   ' + ' '.join(f'{m}:{a:+.1f}' for m, a in zip(missions, ates)))
    print('frontend ATE (mm): ' + ' '.join(f'{m}:{front[m] * 1000:.1f}' for m in missions))


if __name__ == '__main__':
    main()
