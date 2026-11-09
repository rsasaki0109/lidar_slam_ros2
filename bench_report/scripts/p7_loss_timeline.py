#!/usr/bin/env python3
"""Where along a sequence a method's error grows (report chapter 60; the opened blind sequences of chapter 57).

usage: p7_loss_timeline.py SEQ RUNS_DIR REF OUT_PREFIX [METHOD ...]

REF: the HILTI ground-truth TUM file, or a GrandTour prism_position zarr directory. Each method's trajectory
(RUNS_DIR/SEQ/<method>_r1/traj.tum) is interpolated at the reference times, aligned to them (SE(3), no scale), and the
error e(t) is split into horizontal and vertical parts. The 10 s growth |e(t + 10) - e(t)| marks where an error is made
(a global alignment spreads an end drift over the whole run, the growth does not). Writes OUT_PREFIX.png (error and
growth over time, every method) and OUT_PREFIX.txt (the 10 s windows where the first method's growth is largest, with
the other methods' growth in the same window).
"""
import os
import sys

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from p1_grandtour_ate import align_se3  # noqa: E402

WINDOW_S = 10.0


def reference(path):
    if os.path.isdir(path):
        import zarr
        z = zarr.open(path, mode='r')
        return z['timestamp'][:].astype(float), z['point'][:].astype(float)
    r = np.loadtxt(path, comments='#', ndmin=2)
    return r[:, 0], r[:, 1:4]


def errors(traj, rt, rp):
    est = np.loadtxt(traj, ndmin=2)
    inside = (rt >= est[0, 0]) & (rt <= est[-1, 0])
    t, p = rt[inside], rp[inside]
    ep = np.stack([np.interp(t, est[:, 0], est[:, k]) for k in (1, 2, 3)], 1)
    R, tr = align_se3(ep, p)
    return t, ep @ R.T + tr - p


def growth(t, e):
    j = np.searchsorted(t, t + WINDOW_S)
    ok = j < len(t)
    g = np.full(len(t), np.nan)
    g[ok] = np.linalg.norm(e[j[ok]] - e[ok], axis=1)
    return g


def main():
    if len(sys.argv) < 5:
        sys.exit(__doc__)
    seq, runs, ref, out = sys.argv[1:5]
    methods = sys.argv[5:] or ['G', 'bievr', 'fastlio', 'glim', 'rko_upstream']
    rt, rp = reference(ref)
    t0 = rt[0]
    res = {}
    for m in methods:
        f = os.path.join(runs, seq, f'{m}_r1', 'traj.tum')
        if os.path.exists(f) and os.path.getsize(f):
            t, e = errors(f, rt, rp)
            res[m] = (t, e, growth(t, e))
    fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True, dpi=110)
    for m, (t, e, g) in res.items():
        ax[0].plot(t - t0, np.linalg.norm(e[:, :2], axis=1), lw=0.8, label=m)
        ax[1].plot(t - t0, e[:, 2], lw=0.8, label=m)
        ax[2].plot(t - t0, g, lw=0.8, label=m)
    ax[0].set_ylabel('horizontal error (m)')
    ax[1].set_ylabel('vertical error (m)')
    ax[2].set_ylabel(f'{WINDOW_S:.0f} s growth (m)')
    ax[2].set_xlabel('time from the first reference point (s)')
    for a in ax:
        a.grid(alpha=0.3)
        a.set_yscale('symlog', linthresh=0.05)
    ax[0].legend(ncol=len(res), fontsize=8)
    ax[0].set_title(seq)
    fig.tight_layout()
    fig.savefig(out + '.png')
    first = methods[0]
    t, e, g = res[first]
    lines = [f'{seq}: windows of {WINDOW_S:.0f} s with the largest growth of {first}\n'
             'start_s  ' + '  '.join(f'{m:>12s}' for m in res)]
    order = np.argsort(-np.nan_to_num(g))
    taken = []
    for i in order:
        if len(taken) == 8:
            break
        if any(abs(t[i] - t[k]) < WINDOW_S for k in taken):
            continue
        taken.append(i)
        cells = []
        for m, (tm, em, gm) in res.items():
            k = min(np.searchsorted(tm, t[i]), len(tm) - 1)
            cells.append(f'{gm[k]:12.3f}')
        lines.append(f'{t[i] - t0:7.1f}  ' + '  '.join(cells))
    lines.append('final horizontal / vertical error: ' + ', '.join(
        f'{m} {np.linalg.norm(em[-1, :2]):.2f} / {em[-1, 2]:.2f}' for m, (tm, em, gm) in res.items()))
    open(out + '.txt', 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
