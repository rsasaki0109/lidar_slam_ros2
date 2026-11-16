#!/usr/bin/env python3
"""Score the sealed H8 (PASTEL) runs of the second blind run, only after the ground truth is opened with the
maintainer's approval (bench_report/data/blind_v7_run2_2026-11-10/preregistration_h8.txt).

usage: p8_blind_score_h8.py RUNS_DIR GT_DIR CALIB_DIR OUT_PREFIX

RUNS_DIR: <seq>/<method>_r<k>/traj_imu.tum (T_world_imu). GT_DIR: <seq>.csv of ground_truth.zip (time in ns, prism
position). CALIB_DIR: os0_128_lidar.yaml and leica_prism.yaml. ATE RMSE of the prism position: the estimate's prism
(T_world_imu * t) interpolated at the reference stamps, rigid SE(3) Umeyama fit without scale, no time offset.
A run is complete if it exited 0, has no NaN and spans at least 95 % of the reference's time span.
Writes OUT_PREFIX.csv (one row per run) and OUT_PREFIX.md.
Run with `python3 -I`: the ground truth is untrusted input.
"""
import math
import os
import re
import statistics
import sys

import numpy as np

METHODS = ['G', 'rko_upstream', 'bievr', 'fastlio', 'glim']
NAMES = {'G': 'G (G22)', 'rko_upstream': 'RKO-LIO upstream', 'bievr': 'BIEVR-LIO', 'fastlio': 'FAST-LIO2',
         'glim': 'GLIM'}


def matrix(path, key):
    text = open(path).read()
    block = text[text.index(key):]
    data = re.search(r'data:\s*\[([^\]]*)\]', block).group(1)
    T = np.array([float(v) for v in data.replace('\n', ' ').split(',')]).reshape(4, 4)
    R = T[:3, :3].copy()
    if np.linalg.det(R) < 0:  # printed as a reflection: the sign of [2, 2] (preregistration_h8.txt)
        R[2, 2] = -R[2, 2]
    u, _, vt = np.linalg.svd(R)
    T[:3, :3] = u @ vt
    return T


def rot(q):
    x, y, z, w = (q / np.linalg.norm(q, axis=1, keepdims=True)).T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                     np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                     np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], 1)


def ate(est, ref_t, ref_p, lever):
    p = est[:, 1:4] + np.einsum('nij,j->ni', rot(est[:, 4:8]), lever)
    keep = (ref_t >= est[0, 0]) & (ref_t <= est[-1, 0])
    t, r = ref_t[keep], ref_p[keep]
    e = np.stack([np.interp(t, est[:, 0], p[:, i]) for i in range(3)], 1)
    mu_e, mu_r = e.mean(0), r.mean(0)
    u, _, vt = np.linalg.svd((r - mu_r).T @ (e - mu_e))
    S = np.diag([1, 1, np.sign(np.linalg.det(u @ vt))])
    R = u @ S @ vt
    d = (e - mu_e) @ R.T + mu_r - r
    return float(np.sqrt((d ** 2).sum(1).mean())), float(keep.mean())


def gmean(v):
    return math.exp(sum(math.log(x) for x in v) / len(v)) if v else math.nan


def main():
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    runs, gt, calib, out = sys.argv[1:]
    T_i_l = matrix(os.path.join(calib, 'os0_128_lidar.yaml'), 'T_OS1_IMU_OS0')
    T_b_l = matrix(os.path.join(calib, 'os0_128_lidar.yaml'), 'T_Body_OS0')
    T_b_p = matrix(os.path.join(calib, 'leica_prism.yaml'), 'T_Body_Prism')
    lever = (T_i_l @ np.linalg.inv(T_b_l) @ T_b_p)[:3, 3]
    seqs = sorted((s for s in os.listdir(runs) if os.path.isdir(os.path.join(runs, s))), key=int)
    rows, dur = [], {}
    for seq in seqs:
        g = np.loadtxt(os.path.join(gt, f'{seq}.csv'), delimiter=',', skiprows=1, ndmin=2)
        ref_t, ref_p = g[:, 2] * 1e-9, g[:, 3:6]
        dur[seq] = None
        for m in METHODS:
            for k in (1, 2, 3):
                d = os.path.join(runs, seq, f'{m}_r{k}')
                code = int(open(os.path.join(d, 'exit.txt')).read().split()[0])
                info = open(os.path.join(d, 'run_info.txt')).read()
                wall = float(re.search(r'wall_seconds ([0-9.]+)', info).group(1))
                rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',
                                    open(os.path.join(d, 'time.txt')).read()).group(1))
                traj = os.path.join(d, 'traj_imu.tum')
                est = np.loadtxt(traj, ndmin=2) if os.path.exists(traj) and os.path.getsize(traj) else np.zeros((0, 8))
                finite = bool(len(est) and np.isfinite(est).all())
                a, span = ate(est, ref_t, ref_p, lever) if finite else (math.nan, 0.0)
                # the span is of the reference's whole time span, not of the part the estimate covers
                complete = bool(code == 0 and finite and span >= 0.95)
                if m == 'G' and k == 1 and len(est):
                    dur[seq] = float(est[-1, 0] - est[0, 0])
                rows.append({'seq': seq, 'method': m, 'rep': k, 'complete': complete, 'span': round(span, 4),
                             'ate': round(a, 5), 'exit': code, 'wall_s': wall, 'peak_rss_mb': round(rss / 1024, 1)})
    with open(out + '.csv', 'w') as f:
        keys = list(rows[0])
        f.write(','.join(keys) + '\n')
        for r in rows:
            f.write(','.join(str(r[k]) for k in keys) + '\n')
    med = {}
    for seq in seqs:
        for m in METHODS:
            ok = [r['ate'] for r in rows if r['seq'] == seq and r['method'] == m and r['complete']]
            med[(seq, m)] = statistics.median(ok) if ok else None
    lines = [f'Prism in the OS1-16 IMU frame: {lever.round(4).tolist()} m', '',
             '| sequence | ' + ' | '.join(NAMES[m] for m in METHODS) + ' | G / best rival | G RTF | G peak RSS (MB) |',
             '|' + '---|' * (len(METHODS) + 4)]
    worse, rtfs, rsss = {}, [], []
    for seq in seqs:
        rivals = [med[(seq, m)] for m in METHODS[1:] if med[(seq, m)] is not None]
        g = med[(seq, 'G')]
        ratio = g / min(rivals) if g is not None and rivals else None
        worse[seq] = ratio is None or ratio > 1.10
        g_rows = [r for r in rows if r['seq'] == seq and r['method'] == 'G']
        rtf = max(r['wall_s'] for r in g_rows) / dur[seq] if dur[seq] else math.nan
        rss = max(r['peak_rss_mb'] for r in g_rows)
        rtfs.append(rtf)
        rsss.append(rss)
        cells = [f'{med[(seq, m)]:.4f}' if med[(seq, m)] is not None else 'failed' for m in METHODS]
        lines.append(f'| {seq} | ' + ' | '.join(cells) + f" | {f'{ratio:.2f}' if ratio else '-'} | {rtf:.3f} | {rss} |")
    gm = {m: gmean([med[(s, m)] for s in seqs if med[(s, m)] is not None]) for m in METHODS}
    fails = {m: sum(med[(s, m)] is None for s in seqs) for m in METHODS}
    common = [s for s in seqs if all(med[(s, m)] is not None for m in METHODS)]
    lines += ['', f'H8 ({len(seqs)} sequences): geometric means over each method\'s completed sequences: ' + ', '.join(
        f'{NAMES[m]} {gm[m]:.4f}' + (f' ({fails[m]} failed)' if fails[m] else '') for m in METHODS),
        f'Over the {len(common)} sequences every method completed: ' + ', '.join(
            f'{NAMES[m]} {gmean([med[(s, m)] for s in common]):.4f}' for m in METHODS),
        f'G more than 10 % worse than its best rival on {sum(worse.values())} of {len(seqs)}: '
        + ', '.join(s for s in seqs if worse[s]),
        f'G: completion {len(seqs) - fails["G"]}/{len(seqs)}, worst RTF {max(rtfs):.3f}, worst peak RSS {max(rsss)} MB']
    with open(out + '.md', 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
