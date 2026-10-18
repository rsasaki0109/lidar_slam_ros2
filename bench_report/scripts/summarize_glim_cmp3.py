#!/usr/bin/env python3
"""Summarize glim_vs_ours3.sh (timing), ape_glim_cmp2.py (accuracy) and map_quality_cmp.sh (maps) as Markdown.

Aggregation as docs/comparison.md: time / RTF / CPU / APE by median, peak RSS by maximum.
Usage: summarize_glim_cmp3.py <glim_vs_ours3.csv> <pipe_rows.csv> <ape.csv> <map_quality.csv>
"""
import csv
import statistics
import sys

DUR = 125.814128037
SYSTEMS = ['glim', 'ours_off', 'ours_loop']
runs = [r for r in csv.reader(open(sys.argv[1])) if r]
pipe = {r[0]: r for r in csv.reader(open(sys.argv[2])) if r}
apes = list(csv.DictReader(open(sys.argv[3])))
maps = [r for r in csv.reader(open(sys.argv[4])) if r]


def med(values):
    values = [float(v) for v in values if v not in ('NA', '')]
    return statistics.median(values) if values else float('nan')


print('| # | system | round | exit | frontend exit s | results saved s | RTF | peak RSS MB | CPU s | took s '
      '| save s | result md5 |')
print('|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|')
for i, r in enumerate(runs, 1):
    p = pipe.get(f'{r[0]}_{r[1]}')
    took, save = (p[4], p[3]) if p else ('–', '–')
    print(f'| {i} | {r[0]} | {r[1]} | {r[2]} | {r[3] if r[3] != "NA" else "–"} | {r[4]} | {r[5]} | {r[6]} '
          f'| {r[7]} | {took} | {save} | {r[12][:8]} |')
print()
s = {}
for system in SYSTEMS:
    sel = [r for r in runs if r[0] == system]
    s[system] = {
        'complete': med(r[4] for r in sel), 'front': med(r[3] for r in sel),
        'rss': max(float(r[6]) for r in sel), 'cpu': med(r[7] for r in sel),
        'n': len(sel), 'exits': sorted({r[2] for r in sel}), 'md5': len({r[12] for r in sel}),
        'took': med(pipe[f'{system}_{r[1]}'][4] for r in sel) if system != 'glim' else float('nan'),
        'save': med(pipe[f'{system}_{r[1]}'][3] for r in sel) if system != 'glim' else float('nan'),
    }
    kind = {'glim': 'traj_imu', 'ours_off': 'corrected_dense', 'ours_loop': 'corrected_dense'}[system]
    rows = [a for a in apes if a['system'] == system and a['trajectory'] == kind]
    s[system]['ape6'] = med(a['ape6_rmse'] for a in rows)
    s[system]['ape6_range'] = (min(float(a['ape6_rmse']) for a in rows), max(float(a['ape6_rmse']) for a in rows))
    s[system]['ape7'] = med(a['ape7_rmse'] for a in rows)
    s[system]['pairs7'] = sorted({a['ape7_pairs'] for a in rows})
    if system == 'glim':
        odo = [a for a in apes if a['system'] == system and a['trajectory'] == 'odom_imu']
        s[system]['ape6_odom'] = med(a['ape6_rmse'] for a in odo)
    m = [r for r in maps if r[0].startswith(system + '_')]
    s[system]['map'] = tuple(med(r[k] for r in m) for k in (4, 5, 6))
    s[system]['map_points'] = med(r[1] for r in m)
g = s['glim']
print('| system | n | results saved s (median) | RTF | vs GLIM | peak RSS MB (max) | CPU s (median) '
      '| APE 6 pts m (median) | APE 7 pts m | map thickness mean / p95 m | planar coverage | map points |')
print('|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|')
for system in SYSTEMS:
    v = s[system]
    lo, hi = v['ape6_range']
    rng = f' ({lo:.4f}–{hi:.4f})' if hi > lo else ''
    print(f'| {system} | {v["n"]} | {v["complete"]:.2f} | {v["complete"] / DUR:.4f} | {v["complete"] / g["complete"]:.3f} '
          f'| {v["rss"]:.1f} | {v["cpu"]:.1f} | {v["ape6"]:.4f}{rng} | {v["ape7"]:.4f} (pairs {"/".join(v["pairs7"])}) '
          f'| {v["map"][0]:.4f} / {v["map"][1]:.4f} | {v["map"][2]:.3f} | {v["map_points"]:.0f} |')
print()
for system in SYSTEMS:
    v = s[system]
    print(f'- {system}: exits {v["exits"]}, distinct result trajectories {v["md5"]}, frontend exit median '
          f'{v["front"]:.2f} s, offline node took median {v["took"]:.2f} s, map_save median {v["save"]:.3f} s')
print(f'- GLIM odometry only (odom_imu) APE 6 pts median {g["ape6_odom"]:.4f}')
for system in ('ours_off', 'ours_loop'):
    v = s[system]
    print(f'- {system}/glim: time {v["complete"] / g["complete"]:.3f}, RSS {v["rss"] / g["rss"]:.3f}, '
          f'CPU {v["cpu"] / g["cpu"]:.3f}, APE6 {v["ape6"] / g["ape6"]:.3f} ({(1 - v["ape6"] / g["ape6"]) * 100:.1f}% lower), '
          f'thickness mean {v["map"][0] / g["map"][0]:.3f}, p95 {v["map"][1] / g["map"][1]:.3f}, '
          f'coverage {v["map"][2] / g["map"][2]:.3f}')
