#!/usr/bin/env python3
"""Score the sealed blind run (bench_report/data/blind_v7_2026-11-08/preregistration.txt), only after the ground truth
is opened with the maintainer's approval.

usage: p5_blind_score.py RUNS_DIR H4_GT_DIR H5_REF_DIR OUT_PREFIX

RUNS_DIR: /home/user/bench/blind_v7/runs (<seq>/<method>_r<k>/traj.tum). H4_GT_DIR holds exp14_*_imu.txt,
exp16_*_imu.txt, exp18_*_imu.txt; H5_REF_DIR holds <short>/prism_position (zarr). Writes OUT_PREFIX.csv (one row per
run) and OUT_PREFIX.md (per sequence: each method's median of its complete runs, G against the best rival, G's
real-time factor and peak RSS; per holdout and overall: geometric means and the win condition of the plan).
H4: scripts/ape_from_tum.py --interpolate; H5: bench_report/scripts/p1_grandtour_ate.py (the spike-free score beside).
A run is complete if it exited by itself (exit status 0, or FAST-LIO2 stopped by the script after the bag), its
trajectory has no NaN, and it spans at least 95 % of the reference's time span.
"""
import glob
import math
import os
import re
import statistics
import subprocess
import sys

import numpy as np

REPO = '/home/user/lidar_slam_ros2'
PY = '/home/user/mapshd/venv/bin/python'
METHODS = ['G', 'rko_upstream', 'bievr', 'fastlio', 'glim']
NAMES = {'G': 'G (G12)', 'rko_upstream': 'RKO-LIO upstream', 'bievr': 'BIEVR-LIO', 'fastlio': 'FAST-LIO2',
         'glim': 'GLIM'}
H4 = ['exp14', 'exp16', 'exp18']


def ref_times(seq, h4_dir, h5_dir):
    if seq in H4:
        path = glob.glob(os.path.join(h4_dir, f'{seq}_*imu*.txt'))[0]
        return path, np.loadtxt(path, comments='#', ndmin=2)[:, 0]
    import zarr
    path = os.path.join(h5_dir, seq, 'prism_position')
    return path, zarr.open(path, mode='r')['timestamp'][:].astype(float)


def score(seq, traj, ref, out_dir):
    if seq in H4:
        report = os.path.join(out_dir, 'ape.txt')
        subprocess.run([sys.executable, f'{REPO}/scripts/ape_from_tum.py', '--ref', ref, '--est', traj, '--out', report,
                        '--interpolate'], check=False, capture_output=True)
        m = re.search(r'^rmse:\s*([0-9.eE+-]+)', open(report).read(), re.M) if os.path.exists(report) else None
        return (float(m.group(1)), None) if m else (math.nan, None)
    r = subprocess.run([PY, '-I', f'{REPO}/bench_report/scripts/p1_grandtour_ate.py', traj, ref], capture_output=True,
                       text=True, check=False)
    m = re.search(r'ATE rmse ([0-9.]+) m.*spikes ([0-9.]+)', r.stdout)
    return (float(m.group(1)), float(m.group(2))) if m else (math.nan, None)


def gmean(values):
    return math.exp(sum(math.log(v) for v in values) / len(values)) if values else math.nan


def main():
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    runs, h4_dir, h5_dir, out = sys.argv[1:]
    rows, best = [], {}
    seqs = [s for s in sorted(os.listdir(runs)) if os.path.isdir(os.path.join(runs, s))]
    seqs = [s for s in H4 if s in seqs] + [s for s in seqs if s not in H4]
    for seq in seqs:
        ref, rt = ref_times(seq, h4_dir, h5_dir)
        for m in METHODS:
            for k in (1, 2, 3):
                d = os.path.join(runs, seq, f'{m}_r{k}')
                if not os.path.isdir(d):
                    continue
                traj = os.path.join(d, 'traj.tum')
                exit_code = int(open(os.path.join(d, 'exit.txt')).read().split()[0]) \
                    if os.path.exists(os.path.join(d, 'exit.txt')) else -1
                info = open(os.path.join(d, 'run_info.txt')).read() if os.path.exists(os.path.join(d, 'run_info.txt')) \
                    else ''
                wall = float(re.search(r'wall_seconds ([0-9.]+)', info).group(1)) if 'wall_seconds' in info \
                    else math.nan
                tfile = os.path.join(d, 'time.txt')
                rss = re.search(r'Maximum resident set size \(kbytes\): (\d+)', open(tfile).read()) \
                    if os.path.exists(tfile) else None
                node = re.search(r'Offline Node took ([0-9.]+)', open(os.path.join(d, 'stdout.log')).read()) \
                    if m == 'G' and os.path.exists(os.path.join(d, 'stdout.log')) else None
                est = np.loadtxt(traj, ndmin=2) if os.path.exists(traj) and os.path.getsize(traj) else np.zeros((0, 8))
                span = ((rt >= est[0, 0]) & (rt <= est[-1, 0])).mean() if len(est) else 0.0
                exited = exit_code == 0 or (m == 'fastlio' and exit_code in (0, 130, -2))
                complete = bool(exited and len(est) and np.isfinite(est).all() and span >= 0.95)
                ape, ape_nospike = score(seq, traj, ref, d) if len(est) else (math.nan, None)
                rows.append({'seq': seq, 'method': m, 'rep': k, 'complete': complete, 'span': round(float(span), 4),
                             'ape': ape, 'ape_nospike': ape_nospike, 'exit': exit_code, 'wall_s': wall,
                             'node_s': float(node.group(1)) if node else None,
                             'peak_rss_mb': round(int(rss.group(1)) / 1024, 1) if rss else None})
        g1 = os.path.join(runs, seq, 'G_r1', 'traj_imu.tum')
        if os.path.exists(g1) and os.path.getsize(g1):
            t = np.loadtxt(g1, ndmin=2)[:, 0]
            best[seq] = {'duration_s': float(t[-1] - t[0])}  # the data span the frontend covered
    with open(out + '.csv', 'w') as f:
        keys = list(rows[0].keys())
        f.write(','.join(keys) + '\n')
        for r in rows:
            f.write(','.join(str(r[k]) for k in keys) + '\n')
    med = {}
    for seq in seqs:
        for m in METHODS:
            ok = [r['ape'] for r in rows if r['seq'] == seq and r['method'] == m and r['complete']]
            med[(seq, m)] = statistics.median(ok) if ok else None
    lines = ['| sequence | ' + ' | '.join(NAMES[m] for m in METHODS) + ' | G / best rival | G RTF | G peak RSS (MB) |',
             '|' + '---|' * (len(METHODS) + 4)]
    worse = {}
    for seq in seqs:
        rivals = [med[(seq, m)] for m in METHODS[1:] if med[(seq, m)] is not None]
        g = med[(seq, 'G')]
        ratio = g / min(rivals) if g is not None and rivals else None
        worse[seq] = ratio is None or ratio > 1.10
        g_rows = [r for r in rows if r['seq'] == seq and r['method'] == 'G']
        dur = best.get(seq, {}).get('duration_s')
        rtf = max(r['node_s'] for r in g_rows if r['node_s']) / dur if dur and g_rows else None
        rss = max(r['peak_rss_mb'] for r in g_rows if r['peak_rss_mb']) if g_rows else None
        cells = [f'{med[(seq, m)]:.4f}' if med[(seq, m)] is not None else 'failed' for m in METHODS]
        lines.append(f'| {seq} | ' + ' | '.join(cells) + f" | {f'{ratio:.2f}' if ratio else '-'} | "
                     f"{f'{rtf:.3f}' if rtf else '-'} | {rss if rss else '-'} |")
    lines.append('')
    for label, group in (('H4', [s for s in seqs if s in H4]), ('H5', [s for s in seqs if s not in H4]),
                         ('all', seqs)):
        if not group:
            continue
        gm = {m: gmean([med[(s, m)] for s in group if med[(s, m)] is not None]) for m in METHODS}
        fails = {m: sum(med[(s, m)] is None for s in group) for m in METHODS}
        lines.append(f'{label} ({len(group)} sequences): geometric means ' + ', '.join(
            f'{NAMES[m]} {gm[m]:.4f}' + (f' ({fails[m]} failed)' if fails[m] else '') for m in METHODS) +
            f'; G more than 10 % worse than its best rival on {sum(worse[s] for s in group)}')
    with open(out + '.md', 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
