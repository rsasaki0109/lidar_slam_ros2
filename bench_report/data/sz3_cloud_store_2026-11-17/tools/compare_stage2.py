"""Stage 2 comparison (plan_stage2.txt): T1 byte identity, T2 trajectory / loops / bytes, report RSS."""
import filecmp, json, math, re, sys
from pathlib import Path
import numpy as np

S2 = Path('/home/user/bench/sz3/s2')

def tum(p):
    a = np.loadtxt(p, ndmin=2)
    return {round(r[0], 6): r[1:] for r in a}

def quat_angle_deg(q1, q2):
    d = abs(float(np.dot(q1 / np.linalg.norm(q1), q2 / np.linalg.norm(q2))))
    return math.degrees(2 * math.acos(min(1.0, d)))

def traj_diff(a, b):
    A, B = tum(a), tum(b)
    keys = sorted(set(A) & set(B))
    dp = np.array([np.linalg.norm(A[k][:3] - B[k][:3]) for k in keys])
    dr = np.array([quat_angle_deg(A[k][3:7], B[k][3:7]) for k in keys])
    return {'matched': len(keys), 'only_a': len(A) - len(keys), 'only_b': len(B) - len(keys),
            'pos_rms': float(np.sqrt(np.mean(dp ** 2))), 'pos_max': float(dp.max()),
            'rot_max_deg': float(dr.max())}

def loops(d):
    return max(0, sum(1 for _ in open(d / 'out' / 'loop_edges.csv')) - 1)

def stats(d):
    err = (d / 'stderr.log').read_text() + (d / 'run.log').read_text()
    m = re.search(r'Submap clouds kept compressed: ([\d.]+) MiB', err)
    q = re.search(r'within [\d.]+ m \(submap_cloud_max_error_m\): (\d+) of (\d+), (\d+) bytes', err)
    rss = re.search(r'Maximum resident set size \(kbytes\): (\d+)', err)
    wall = re.search(r'Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): ([\d:.]+)', err)
    w = sum(float(x) * 60 ** i for i, x in enumerate(reversed(wall.group(1).split(':'))))
    return {'stored_mib': float(m.group(1)), 'stored_bytes': int(q.group(3)) if q else None,
            'quantized': f'{q.group(1)}/{q.group(2)}' if q else None,
            'rss_mb': int(rss.group(1)) / 1024, 'wall_s': w}

out = {'T1': {}, 'T2': {}}
for ds in ('d1', 'd3'):
    ref = S2 / 'before' / ds / 'none' / 'out'
    for arm in ('none', '0.0'):
        cmp = filecmp.dircmp(ref, S2 / 'after' / ds / arm / 'out')
        files = sorted(p.name for p in ref.iterdir())
        same = all(filecmp.cmp(ref / f, S2 / 'after' / ds / arm / 'out' / f, shallow=False) for f in files)
        out['T1'][f'{ds}/{arm}'] = {'files': len(files), 'identical': same and not cmp.left_only and not cmp.right_only}
for ds in ('d1', 'd2', 'd3', 'd4', 'd5', 'd6', 'd7', 'd8'):
    base = S2 / 'after' / ds / 'none'
    if not (base / 'done').exists():
        continue
    row = {'lossless': {'loops': loops(base), **stats(base)}}
    for err in ('0.001', '0.005'):
        d = S2 / 'after' / ds / err
        if not (d / 'done').exists():
            continue
        st = stats(d)
        row[err] = {'opt': traj_diff(base / 'out' / 'trajectory_optimized.tum', d / 'out' / 'trajectory_optimized.tum'),
                    'refined': traj_diff(base / 'out' / 'trajectory_refined.tum', d / 'out' / 'trajectory_refined.tum')
                    if (d / 'out' / 'trajectory_refined.tum').exists() else None,
                    'loops': loops(d), **st,
                    'bytes_ratio': st['stored_bytes'] / (row['lossless']['stored_mib'] * 1048576)}
        o = row[err]['opt']; lb = row['lossless']['loops']
        row[err]['pass'] = {'a': o['pos_rms'] <= 0.01 and o['pos_max'] <= 0.05 and o['rot_max_deg'] <= 0.2,
                            'b': abs(row[err]['loops'] - lb) <= max(2, 0.1 * lb),
                            'c': row[err]['bytes_ratio'] <= 0.6}
    out['T2'][ds] = row
json.dump(out, sys.stdout, indent=1)
