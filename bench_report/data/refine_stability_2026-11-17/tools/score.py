"""Refinement stability screen scoring (plan.txt): ATE vs development references, 1 um sensitivity."""
import importlib.util, json, re, sys
from pathlib import Path
import numpy as np
import zarr

spec = importlib.util.spec_from_file_location('ate', '/home/user/lidar_slam_ros2/bench_report/scripts/p1_grandtour_ate.py')
ate = importlib.util.module_from_spec(spec); spec.loader.exec_module(ate)
B = Path('/home/user/bench/refine/runs')

def ref_of(ds):
    if ds in ('c1', 'c2'):
        z = zarr.open(f'/home/user/data/grandtour/reference_dev/con-{ds[1]}/prism_position', mode='r')
        return z['timestamp'][:].astype(float), z['point'][:]
    if ds == 'm1':
        a = np.loadtxt('/home/user/data/mapshd/gt/calibration-room_trial-1/ground_truth.tum')
        return a[:, 0], a[:, 1:4]
    if ds == 'h4':
        a = np.loadtxt('/home/user/data/hilti2022/exp04_construction_upper_level_gt.txt')
        return a[:, 0], a[:, 1:4]
    if ds == 's2':
        rows = [l.strip().split(',') for l in open('/home/user/data/rtk_slam/rtk_slam_eval/ground_truth/stadtgarten_seq2.csv')][1:]
        a = np.array([[float(r[5]), float(r[1]), float(r[2]), float(r[3])] for r in rows])
        a = a[np.argsort(a[:, 0])]
        return a[:, 0], a[:, 1:4] - a[0, 1:4]

def ate_rms(tum, rt, rp):
    est = np.loadtxt(tum)
    m = (rt >= est[0, 0]) & (rt <= est[-1, 0])
    ep = np.stack([np.interp(rt[m], est[:, 0], est[:, k]) for k in (1, 2, 3)], 1)
    R, t = ate.align_se3(ep, rp[m])
    e = np.linalg.norm(ep @ R.T + t - rp[m], axis=1)
    return float(np.sqrt((e ** 2).mean())), int(m.sum())

def maxdiff(a, b):
    A = {round(r[0], 6): r[1:4] for r in np.loadtxt(a)}; Bm = {round(r[0], 6): r[1:4] for r in np.loadtxt(b)}
    k = sorted(set(A) & set(Bm))
    return float(max(np.linalg.norm(A[x] - Bm[x]) for x in k))

out = {}
for ds in ('c1', 'c2', 'm1', 'h4', 's2'):
    rt, rp = ref_of(ds)
    row = {}
    for v in ('R0', 'RB', 'RA'):
        d = B / ds / v
        if not (d / '0' / 'done').exists():
            continue
        r = {}
        r['ate_refined'], r['n_ref'] = ate_rms(d / '0/out/trajectory_refined.tum', rt, rp)
        r['ate_optimized'], _ = ate_rms(d / '0/out/trajectory_optimized.tum', rt, rp)
        if (d / '1um' / 'done').exists():
            r['sensitivity_m'] = maxdiff(d / '0/out/trajectory_refined.tum', d / '1um/out/trajectory_refined.tum')
        rep = (d / '0/out/map_refinement_report.yaml').read_text()
        r['report'] = {k: v2 for k, v2 in re.findall(r'(\w+): (\S+)', rep) if k in ('status', 'improved_windows', 'gated_windows', 'windows')}
        err = (d / '0/stderr.log').read_text()
        w = re.search(r'Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): ([\d:.]+)', err).group(1)
        r['wall_s'] = sum(float(x) * 60 ** i for i, x in enumerate(reversed(w.split(':'))))
        row[v] = r
    out[ds] = row
json.dump(out, sys.stdout, indent=1)
