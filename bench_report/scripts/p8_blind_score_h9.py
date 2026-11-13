#!/usr/bin/env python3
"""Score the sealed H9 (Odyssey) runs of the second blind run, only after the reference is opened with the
maintainer's approval (bench_report/data/blind_v7_run2_2026-11-10/preregistration_h9.txt).

usage: p8_blind_score_h9.py RUNS_DIR GT_DIR OUT_PREFIX [SCORE_H8_CSV]

RUNS_DIR: <seq>/<method>_r<k>/traj_imu.tum (T_world_imu; the IMU frame is the LiDAR frame) and <seq>/bag_summary.json.
GT_DIR: <seq>/lidar_poses_ref.txt (KITTI rows, the LiDAR pose relative to the first) and <seq>/lidar_stamps.txt
(ns, sorted ouster file names), from p8_odyssey_fetch_reference.py.
ATE RMSE of position: the estimate interpolated at the reference stamps, rigid SE(3) Umeyama fit without scale, no
time offset. A run is complete if it exited 0, has no NaN and spans at least 95 % of the time between the first and
the last reference stamp. RTF = wall time / the bag's LiDAR duration.
Beside the preregistered score, column ate_dt is the same ATE with one constant time offset per run chosen in
[-0.15, 0.15] s (step 5 ms) to minimise it: a diagnostic of the stamp convention, written after the reference was
opened, never used for the win condition.
With SCORE_H8_CSV (score_h8.csv) the geometric means over H8 + H9 are added.
Writes OUT_PREFIX.csv (one row per run) and OUT_PREFIX.md. Run with `python3 -I`: the reference is untrusted input.
"""
import json
import math
import os
import re
import statistics
import sys

import numpy as np

METHODS = ['G', 'rko_upstream', 'bievr', 'fastlio', 'glim']
NAMES = {'G': 'G (G22)', 'rko_upstream': 'RKO-LIO upstream', 'bievr': 'BIEVR-LIO', 'fastlio': 'FAST-LIO2',
         'glim': 'GLIM'}


def ate(est_t, est_p, ref_t, ref_p):
    keep = (ref_t >= est_t[0]) & (ref_t <= est_t[-1])
    if keep.sum() < 3:
        return math.nan
    t, r = ref_t[keep], ref_p[keep]
    e = np.stack([np.interp(t, est_t, est_p[:, i]) for i in range(3)], 1)
    mu_e, mu_r = e.mean(0), r.mean(0)
    u, _, vt = np.linalg.svd((r - mu_r).T @ (e - mu_e))
    R = u @ np.diag([1, 1, np.sign(np.linalg.det(u @ vt))]) @ vt
    d = (e - mu_e) @ R.T + mu_r - r
    return float(np.sqrt((d ** 2).sum(1).mean()))


def gmean(v):
    return math.exp(sum(math.log(x) for x in v) / len(v)) if v else math.nan


def main():
    if len(sys.argv) not in (4, 5):
        sys.exit(__doc__)
    runs, gt, out = sys.argv[1:4]
    seqs = sorted(s for s in os.listdir(runs) if os.path.isdir(os.path.join(runs, s)))
    rows, dur, notes = [], {}, []
    for seq in seqs:
        stamps = np.loadtxt(os.path.join(gt, seq, 'lidar_stamps.txt'), dtype=np.int64)
        poses = np.loadtxt(os.path.join(gt, seq, 'lidar_poses_ref.txt'), ndmin=2)
        if poses.shape != (len(stamps), 12):
            sys.exit(f'{seq}: {poses.shape} reference rows for {len(stamps)} stamps')
        t0 = int(stamps[0])
        ref_t, ref_p = (stamps - t0) * 1e-9, poses[:, [3, 7, 11]]
        w = json.load(open(os.path.join(runs, seq, 'bag_summary.json')))['written']['/ouster/points']
        if (w['first_stamp'], w['last_stamp'], w['messages']) != (int(stamps[0]), int(stamps[-1]), len(stamps)):
            sys.exit(f'{seq}: the bag\'s LiDAR stamps are not the reference stamps')
        dur[seq] = ref_t[-1] + 0.1  # the last revolution is 0.1 s long
        notes.append(f'{seq}: {len(stamps)} reference poses over {ref_t[-1]:.1f} s, path '
                     f'{np.linalg.norm(np.diff(ref_p, axis=0), axis=1).sum():.0f} m')
        for m in METHODS:
            for k in (1, 2, 3):
                d = os.path.join(runs, seq, f'{m}_r{k}')
                code = int(open(os.path.join(d, 'exit.txt')).read().split()[0])
                info = open(os.path.join(d, 'run_info.txt')).read()
                wall = float(re.search(r'wall_seconds ([0-9.]+)', info).group(1))
                rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',
                                    open(os.path.join(d, 'time.txt')).read()).group(1))
                traj = os.path.join(d, 'traj_imu.tum')
                a = a_dt = math.nan
                span = dt_best = 0.0
                finite = False
                if os.path.exists(traj) and os.path.getsize(traj):
                    # the stamp column is read apart and reduced by the first reference stamp before float64
                    sec = np.array([(int(round(float(x.split('.')[0]))) * 10**9
                                     + int((x.split('.')[1] if '.' in x else '0').ljust(9, '0')[:9]) - t0) * 1e-9
                                    for x in (line.split(None, 1)[0] for line in open(traj) if line.strip())])
                    est = np.loadtxt(traj, ndmin=2)
                    finite = bool(len(est) > 2 and np.isfinite(est).all())
                    if finite:
                        span = (min(sec[-1], ref_t[-1]) - max(sec[0], ref_t[0])) / (ref_t[-1] - ref_t[0])
                        a = ate(sec, est[:, 1:4], ref_t, ref_p)
                        cand = [(ate(sec + dt, est[:, 1:4], ref_t, ref_p), dt) for dt in np.arange(-0.15, 0.1501, 0.005)]
                        a_dt, dt_best = min(c for c in cand if not math.isnan(c[0]))
                complete = bool(code == 0 and finite and span >= 0.95)
                rows.append({'seq': seq, 'method': m, 'rep': k, 'complete': complete, 'span': round(span, 4),
                             'ate': round(a, 5), 'ate_dt': round(a_dt, 5), 'dt_s': round(float(dt_best), 3),
                             'exit': code, 'wall_s': wall, 'rtf': round(wall / dur[seq], 3),
                             'peak_rss_mb': round(rss / 1024, 1)})
    with open(out + '.csv', 'w') as f:
        keys = list(rows[0])
        f.write(','.join(keys) + '\n')
        for r in rows:
            f.write(','.join(str(r[k]) for k in keys) + '\n')

    def med(seq, m, key='ate', need_complete=True):
        ok = [r[key] for r in rows if r['seq'] == seq and r['method'] == m and (r['complete'] or not need_complete)
              and not math.isnan(r[key])]
        return statistics.median(ok) if ok else None

    def table(key, need_complete):
        lines = ['| sequence | ' + ' | '.join(NAMES[m] for m in METHODS) + ' | G / best rival |',
                 '|' + '---|' * (len(METHODS) + 2)]
        for seq in seqs:
            v = [med(seq, m, key, need_complete) for m in METHODS]
            rivals = [x for x in v[1:] if x is not None]
            ratio = f'{v[0] / min(rivals):.2f}' if v[0] is not None and rivals else '-'
            lines.append(f'| {seq} | ' + ' | '.join(f'{x:.4f}' if x is not None else 'failed' for x in v)
                         + f' | {ratio} |')
        return lines

    lines = notes + ['', 'Preregistered score (median of the completed repetitions, m):', ''] + table('ate', True)
    m9 = {(s, m): med(s, m) for s in seqs for m in METHODS}
    fails = {m: sum(m9[(s, m)] is None for s in seqs) for m in METHODS}
    common = [s for s in seqs if all(m9[(s, m)] is not None for m in METHODS)]
    worse = [s for s in seqs if m9[(s, 'G')] is None or not [m9[(s, m)] for m in METHODS[1:] if m9[(s, m)] is not None]
             or m9[(s, 'G')] / min(m9[(s, m)] for m in METHODS[1:] if m9[(s, m)] is not None) > 1.10]
    lines += ['', f'H9 ({len(seqs)} sequences): geometric means over each method\'s completed sequences: ' + ', '.join(
        f'{NAMES[m]} {gmean([m9[(s, m)] for s in seqs if m9[(s, m)] is not None]):.4f}'
        + (f' ({fails[m]} failed)' if fails[m] else '') for m in METHODS),
        f'Over the {len(common)} sequences every method completed: ' + ', '.join(
            f'{NAMES[m]} {gmean([m9[(s, m)] for s in common]):.4f}' for m in METHODS),
        f'G more than 10 % worse than its best rival (or failed) on {len(worse)} of {len(seqs)}: ' + ', '.join(worse),
        '', '| method | completed runs | worst RTF | worst peak RSS (MB) |', '|---|---|---|---|']
    for m in METHODS:
        r = [x for x in rows if x['method'] == m]
        lines.append(f"| {NAMES[m]} | {sum(x['complete'] for x in r)}/{len(r)} | {max(x['rtf'] for x in r):.2f} | "
                     f"{max(x['peak_rss_mb'] for x in r):.0f} |")
    lines += ['', 'G per sequence (worst of three): ' + ', '.join(
        f"{s} RTF {max(x['rtf'] for x in rows if x['seq'] == s and x['method'] == 'G'):.2f} / "
        f"{max(x['peak_rss_mb'] for x in rows if x['seq'] == s and x['method'] == 'G'):.0f} MB" for s in seqs)]
    lines += ['', 'Diagnostic, not the preregistered score: ATE with one constant time offset per run '
              '(every run with a finite trajectory, complete or not), and the median offset chosen (s):', '']
    lines += table('ate_dt', False)
    lines += ['', '| sequence | ' + ' | '.join(NAMES[m] for m in METHODS) + ' |', '|' + '---|' * (len(METHODS) + 1)]
    for seq in seqs:
        lines.append(f'| {seq} | ' + ' | '.join(
            (lambda v: f'{v:+.3f}' if v is not None else '-')(med(seq, m, 'dt_s', False)) for m in METHODS) + ' |')
    lines += ['', 'Spans (median fraction of the reference time span): ' + '; '.join(
        f'{s} ' + ' '.join(f"{m} {med(s, m, 'span', False):.3f}" for m in METHODS) for s in seqs)]
    if len(sys.argv) == 5:
        h8 = {}
        for line in open(sys.argv[4]).read().splitlines()[1:]:
            c = line.split(',')
            if c[3] == 'True':
                h8.setdefault((c[0], c[1]), []).append(float(c[5]))
        s8 = sorted({k[0] for k in h8}, key=int)
        allm = {('H8 ' + s, m): statistics.median(h8[(s, m)]) for s in s8 for m in METHODS if (s, m) in h8}
        allm.update({('H9 ' + s, m): v for (s, m), v in m9.items() if v is not None})
        names = sorted({k[0] for k in allm})
        both = [s for s in names if all((s, m) in allm for m in METHODS)]
        lines += ['', f'H8 + H9: geometric means over each method\'s completed sequences: ' + ', '.join(
            f'{NAMES[m]} {gmean([allm[(s, m)] for s in names if (s, m) in allm]):.4f} '
            f'({sum((s, m) in allm for s in names)} sequences)' for m in METHODS),
            f'Over the {len(both)} sequences every method completed: ' + ', '.join(
                f'{NAMES[m]} {gmean([allm[(s, m)] for s in both]):.4f}' for m in METHODS)]
    with open(out + '.md', 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
