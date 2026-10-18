#!/usr/bin/env python3
"""Re-solve a run's final pose graph with other edge weights and score it like the run.

Reads <run>/pose_graph.g2o (every edge with its measurement and information) and <run>/loop_edges.csv (which edges
are loop edges), starts every vertex from the frontend pose at its submap stamp (as optimizePoseGraph does), rescales
the information of the adjacent and loop edges, solves with the g2o command-line tool (Levenberg-Marquardt, 10
iterations, Huber with delta 1 as for the loop edges of the run), writes trajectory_optimized.tum and scores it
like scripts/mapshd_score.py (densified onto the frontend, LiDAR frame, evo_ape -a) or with ape_oxford.py.

Weights (key=value, all optional):
  adj_t, adj_r   factors on the adjacent edges' translation / rotation information (default 1)
  loop           factor on the loop edges' information as stored (100 / fitness), default 1
  loop_t, loop_r if given, a fixed loop information for translation / rotation instead of 100 / fitness * loop
  drop           comma-separated 'i-j' loop edges to leave out
  travel_min     leave out the loop edges whose frontend path between the two submaps is shorter than this (m)
  travel_l0      scale each loop edge's information by s^2 / (s^2 + travel_l0^2), s its frontend path (m)
  meas           CSV (from,to,tx,ty,tz,qx,qy,qz,qw) of loop measurements to use instead of the run's, e.g. the
                 ground truth from loop_gt_measurements.py (an upper bound on what better loops could give)
Usage: graph_replay.py <run_dir> <out_dir> <scorer: mapshd:<gt_tum> | oxford:<gt_tum>,<T_imu_base.npy> |
       exp04:<gt_tum> | none>
       [key=value ...]
"""
import os
import re
import subprocess
import sys

import numpy as np

G2O = '/opt/ros/jazzy/bin/g2o'
REPO = '/home/user/lidar_slam_ros2'
EVO_APE = '/home/user/mapshd/venv/bin/evo_ape'
T_IMU_LIDAR = np.array([-0.011, -0.02329, 0.04412])


def read_tum(path):
    return np.array([list(map(float, l.split())) for l in open(path) if l.strip() and not l.startswith('#')])


def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def slerp(q0, q1, u):
    if np.dot(q0, q1) < 0:
        q1 = -q1
    d = min(1.0, float(np.dot(q0, q1)))
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
    else:
        th = np.arccos(d)
        q = (np.sin((1 - u) * th) * q0 + np.sin(u * th) * q1) / np.sin(th)
    return q / np.linalg.norm(q)


def at(traj, t):
    i = int(np.clip(np.searchsorted(traj[:, 0], t), 1, len(traj) - 1))
    a, b = traj[i - 1], traj[i]
    u = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
    return np.concatenate([a[1:4] + u * (b[1:4] - a[1:4]), slerp(a[4:8], b[4:8], u)])


def info_from(vals):
    m = np.zeros((6, 6))
    k = 0
    for r in range(6):
        for c in range(r, 6):
            m[r, c] = m[c, r] = vals[k]
            k += 1
    return m


def info_to(m):
    return [m[r, c] for r in range(6) for c in range(r, 6)]


def main():
    run, out, scorer = sys.argv[1:4]
    opts = dict(a.split('=', 1) for a in sys.argv[4:])
    adj_t, adj_r = float(opts.get('adj_t', 1)), float(opts.get('adj_r', 1))
    loop_f = float(opts.get('loop', 1))
    loop_t, loop_r = opts.get('loop_t'), opts.get('loop_r')
    drop = {tuple(map(int, d.split('-'))) for d in opts.get('drop', '').split(',') if d}
    travel_min, travel_l0 = float(opts.get('travel_min', 0)), float(opts.get('travel_l0', 0))
    meas_override = {}
    if 'meas' in opts:
        for l in open(opts['meas']):
            f = l.strip().split(',')
            meas_override[(int(f[0]), int(f[1]))] = [float(v) for v in f[2:9]]
    os.makedirs(out, exist_ok=True)

    stamps = read_tum(f'{run}/trajectory_optimized.tum')[:, 0]
    front = read_tum(f'{run}/front_0/front_tum_0.txt')
    # frontend path length up to each submap, as in loop_features.py (frontend positions at the submap stamps)
    pos = np.array([at(front, t)[:3] for t in stamps])
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))])
    loops = set()
    for l in list(open(f'{run}/loop_edges.csv'))[1:]:
        f = l.split(',')
        loops.add((int(f[0]), int(f[1])))
    lines = []
    for l in open(f'{run}/pose_graph.g2o'):
        f = l.split()
        if not f:
            continue
        if f[0] == 'VERTEX_SE3:QUAT':
            i = int(f[1])
            p = at(front, stamps[i])
            lines.append('VERTEX_SE3:QUAT %d %s' % (i, ' '.join('%.12g' % v for v in p)))
        elif f[0] == 'EDGE_SE3:QUAT':
            i, j = int(f[1]), int(f[2])
            meas = f[3:10]
            info = info_from(list(map(float, f[10:31])))
            if (i, j) in loops or (j, i) in loops:
                travel = abs(cum[j] - cum[i])
                if (i, j) in drop or travel < travel_min:
                    continue
                if (i, j) in meas_override:
                    meas = ['%.12g' % v for v in meas_override[(i, j)]]
                elif (j, i) in meas_override:
                    m = meas_override[(j, i)]
                    Rm = quat_to_R(np.asarray(m[3:7]))
                    tinv = -Rm.T @ np.asarray(m[0:3])
                    meas = ['%.12g' % v for v in [*tinv, -m[3], -m[4], -m[5], m[6]]]
                if loop_t is not None:
                    info = np.diag([float(loop_t)] * 3 + [float(loop_r)] * 3)
                else:
                    info = info * loop_f
                if travel_l0 > 0:
                    info = info * travel ** 2 / (travel ** 2 + travel_l0 ** 2)
            else:
                info[:3, :3] *= adj_t
                info[3:, 3:] *= adj_r
                info[:3, 3:] *= np.sqrt(adj_t * adj_r)
                info[3:, :3] *= np.sqrt(adj_t * adj_r)
            lines.append('EDGE_SE3:QUAT %d %d %s %s' % (i, j, ' '.join(meas), ' '.join('%.12g' % v for v in info_to(info))))
        else:
            lines.append(l.rstrip('\n'))
    src, dst = os.path.join(out, 'in.g2o'), os.path.join(out, 'out.g2o')
    open(src, 'w').write('\n'.join(lines) + '\n')
    subprocess.run([G2O, '-i', '10', '-solver', 'lm_var', '-robustKernel', 'Huber', '-robustKernelWidth', '1',
                    '-o', dst, src], check=True, capture_output=True)
    poses = {}
    for l in open(dst):
        f = l.split()
        if f and f[0] == 'VERTEX_SE3:QUAT':
            poses[int(f[1])] = list(map(float, f[2:9]))
    traj = os.path.join(out, 'trajectory_optimized.tum')
    with open(traj, 'w') as fo:
        for i in sorted(poses):
            fo.write('%.9f %s\n' % (stamps[i], ' '.join('%.9f' % v for v in poses[i])))
    dense = os.path.join(out, 'pipeline_imu.tum')
    subprocess.run(['python3', f'{REPO}/scripts/densify_corrected_trajectory.py', '--raw',
                    f'{run}/front_0/front_tum_0.txt', '--corrected', traj, '--output', dense], check=True,
                   capture_output=True, env=dict(os.environ, PYTHONPATH=REPO))
    if scorer.startswith('mapshd:'):
        gt = scorer.split(':', 1)[1]
        rows = read_tum(dense)
        lidar = os.path.join(out, 'pipeline_lidar.tum')
        with open(lidar, 'w') as fo:
            for r in rows:
                p = r[1:4] + quat_to_R(r[4:8]) @ T_IMU_LIDAR
                fo.write('%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n' % (r[0], *p, *r[4:8]))
        res = subprocess.run([EVO_APE, 'tum', gt, lidar, '-a'], capture_output=True, text=True,
                             env=dict(os.environ, MPLBACKEND='Agg'))
        rmse = float(re.search(r'^\s*rmse\s+([0-9.eE+-]+)', res.stdout, re.M).group(1))
        print(f'{os.path.basename(run.rstrip("/"))},{" ".join(sys.argv[4:]) or "as_run"},{rmse:.6f}')
    elif scorer.startswith('exp04:'):
        gt = scorer.split(':', 1)[1]
        ape = os.path.join(out, 'ape_exp04.txt')
        subprocess.run(['python3', f'{REPO}/scripts/ape_from_tum.py', '--ref', gt, '--est', dense, '--out', ape,
                        '--interpolate'], check=True, capture_output=True)
        rmse = float(re.search(r'^rmse:\s+([0-9.eE+-]+)', open(ape).read(), re.M).group(1))
        print(f'{os.path.basename(run.rstrip("/"))},{" ".join(sys.argv[4:]) or "as_run"},{rmse:.6f}')
    elif scorer.startswith('oxford:'):
        gt, extrinsic = scorer.split(':', 1)[1].split(',')
        res = subprocess.run(['python3', f'{REPO}/bench_report/scripts/ape_oxford.py', dense, gt, extrinsic,
                              os.path.join(out, 'ape')], capture_output=True, text=True, check=True)
        rmse = float(re.search(r'^rmse:\s+([0-9.eE+-]+)', res.stdout, re.M).group(1))
        print(f'{os.path.basename(run.rstrip("/"))},{" ".join(sys.argv[4:]) or "as_run"},{rmse:.6f}')


if __name__ == '__main__':
    main()
