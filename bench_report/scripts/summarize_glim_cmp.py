#!/usr/bin/env python3
"""Summarize glim_vs_ours2.sh (timing) and ape_glim_cmp2.py (accuracy) CSVs as Markdown.

Aggregation as docs/comparison.md: time / RTF / CPU / APE by median, peak RSS by maximum.
Usage: summarize_glim_cmp.py <glim_vs_ours2.csv> <ape.csv>
"""
import csv
import statistics
import sys

SYSTEMS = ['glim', 'ours_readme', 'ours_loop']
runs = [r for r in csv.reader(open(sys.argv[1])) if r]
apes = list(csv.DictReader(open(sys.argv[2])))


def med(values):
    values = [float(v) for v in values if v not in ('NA', '')]
    return statistics.median(values) if values else float('nan')


print('| # | system | round | exit | time s | time incl. save s | RTF | RTF incl. save | peak RSS MB '
      '| CPU s | FS inputs | cached GB before | poses | frontend md5 |')
print('|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|')
for i, r in enumerate(runs, 1):
    print(f'| {i} | {r[0]} | {r[1]} | {r[2]} | {r[3]} | {r[4]} | {r[5]} | {r[6]} | {r[7]} | {r[8]} | {r[9]} '
          f'| {int(r[10]) / 1048576:.2f} | {r[11]} | {r[12][:8]} |')
print()
summary = {}
for system in SYSTEMS:
    sel = [r for r in runs if r[0] == system]
    if not sel:
        continue
    s = {
        'n': len(sel), 'exits': sorted({r[2] for r in sel}),
        'time': med(r[3] for r in sel), 'time_save': med(r[4] for r in sel),
        'times': [r[3] for r in sel], 'times_save': [r[4] for r in sel],
        'rss_max': max(float(r[7]) for r in sel), 'cpu': med(r[8] for r in sel),
        'front_md5': sorted({r[12] for r in sel}), 'poses': sorted({r[11] for r in sel}),
    }
    for kind in sorted({a['trajectory'] for a in apes if a['system'] == system}):
        rows = [a for a in apes if a['system'] == system and a['trajectory'] == kind]
        s[f'ape6_{kind}'] = med(a['ape6_rmse'] for a in rows)
        s[f'ape7_{kind}'] = med(a['ape7_rmse'] for a in rows)
        s[f'ape6_{kind}_range'] = (min(float(a['ape6_rmse']) for a in rows), max(float(a['ape6_rmse']) for a in rows))
        s[f'md5_{kind}'] = len({a['md5'] for a in rows})
        s[f'pairs7_{kind}'] = sorted({a['ape7_pairs'] for a in rows})
    summary[system] = s
DUR = 125.814128037
for system, s in summary.items():
    print(f'- {system}: n={s["n"]} exits={s["exits"]} time median {s["time"]:.2f} s (RTF {s["time"] / DUR:.4f}) '
          f'runs {s["times"]}; incl. save {s["time_save"]:.2f} s (RTF {s["time_save"] / DUR:.4f}) '
          f'runs {s["times_save"]}; peak RSS max {s["rss_max"]:.1f} MB; CPU median {s["cpu"]:.1f} s; '
          f'poses {s["poses"]}; frontend md5 {s["front_md5"]}')
    for key in sorted(k for k in s if k.startswith('ape6_') and not k.endswith('_range')):
        kind = key[5:]
        lo, hi = s[f'{key}_range']
        print(f'    - APE {kind}: common6 median {s[key]:.6f} (range {lo:.6f}..{hi:.6f}), all7 median '
              f'{s["ape7_" + kind]:.6f} (pairs {s["pairs7_" + kind]}), distinct trajectories {s["md5_" + kind]}')
g = summary.get('glim')
if g:
    print()
    for system in ('ours_readme', 'ours_loop'):
        s = summary.get(system)
        if not s:
            continue
        kind = 'corrected_dense' if system == 'ours_loop' else 'frontend'
        print(f'- {system}/glim: time {s["time"] / g["time"]:.3f}, incl. save {s["time_save"] / g["time_save"]:.3f}, '
              f'peak RSS {s["rss_max"] / g["rss_max"]:.3f}, CPU {s["cpu"] / g["cpu"]:.3f}, '
              f'APE common6 {s["ape6_" + kind] / g["ape6_traj_imu"]:.3f} '
              f'({(1 - s["ape6_" + kind] / g["ape6_traj_imu"]) * 100:.1f}% lower), '
              f'APE all7 {s["ape7_" + kind] / g["ape7_traj_imu"]:.3f}')
