#!/usr/bin/env python3
# Height error across the reference gaps where the height changes (stairs out of the total station's view):
# for each run, the estimate is fitted to the reference (rigid; points more than 1 m off, the spikes, left out),
# then for each gap with
# |dz_ref| > 0.5 m the change of the height error across the gap (mean over 2 s at each edge) is printed, in mm.
# For development missions only (reads the reference). Chapter 35.
# Usage: p1_grandtour_climb_err.py <mission>[,<mission>...] [label glob]  (runs at /home/user/bench/grandtour/<label>/<mission>/)
import sys, glob, numpy as np, zarr
def align(A, B):
    ca, cb = A.mean(0), B.mean(0); H = (A-ca).T @ (B-cb); U, S, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]); R = Vt.T @ D @ U.T; return R, cb - R @ ca
missions = sys.argv[1].split(',')
pattern = sys.argv[2] if len(sys.argv) > 2 else '*'
for m in missions:
    ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{m}/prism_position', mode='r')
    rt = ref['timestamp'][:].astype(float); rp = ref['point'][:]
    gaps = [i for i in np.where(np.diff(rt) > 1.0)[0]]
    edges = []
    for i in gaps:
        b = (rt > rt[i] - 2) & (rt <= rt[i]); a = (rt >= rt[i+1]) & (rt < rt[i+1] + 2)
        dz = rp[a, 2].mean() - rp[b, 2].mean()
        if abs(dz) > 0.5: edges.append((i, b, a, dz))
    print(f'{m}: gaps with height change: ' + ', '.join(f'{rt[i]-rt[0]:.0f}-{rt[i+1]-rt[0]:.0f} s dz {dz:+.2f}' for i, b, a, dz in edges))
    for f in sorted(glob.glob(f'/home/user/bench/grandtour/{pattern}/{m}/{m}.tum')):
        try: T = np.loadtxt(f)
        except Exception: continue
        if len(T) < 100 or T[-1,0] < rt[-1] - 30: continue
        k = (rt > T[0,0]) & (rt < T[-1,0])
        e = np.column_stack([np.interp(rt, T[:,0], T[:,c]) for c in (1,2,3)])
        R, t = align(e[k], rp[k]); err = e @ R.T + t - rp
        # drop the reference spike (16 m on arc-3) from the fit: refit without points > 1 m off
        good = k & (np.linalg.norm(err, axis=1) < 1.0); R, t = align(e[good], rp[good]); err = e @ R.T + t - rp
        out = [f'{(err[a,2].mean() - err[b,2].mean())*1000:+5.0f}' for i, b, a, dz in edges]
        print(f'  {f.split("/")[-3]:28s} ' + ' '.join(out) + f'   rms {np.sqrt((err[good]**2).sum(1).mean())*1000:5.1f} mm')
