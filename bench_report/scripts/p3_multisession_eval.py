#!/usr/bin/env python3
"""
Scores a graph_slam_multisession_join run of two GrandTour development sessions (plan in
data/grandtour/multisession_2026-10-29/plan.txt, step 2).
  - join error: session B's frontend trajectory moved by the join's transform against the true transform (each
    session's frontend fitted to its reference, p3_multisession_join.truth_transform): RMS distance and angle;
  - single-session ATE of each session: the runner's own optimized submap poses densified onto the session's frontend
    trajectory (scripts/densify_corrected_trajectory.py), at the prism, fitted to the session's reference;
  - joint ATE: the same for the joined graph's poses (trajectory_session0/1.tum, both in A's frame), with A fitted to
    its reference and B scored with A's fit: one frame for the joined map.
ATE is without reference spikes (as p1_grandtour_ate.py finds them).
Usage: p3_multisession_eval.py <session A dir> <session B dir> <mission A> <mission B> <runner dir A> <runner dir B>
       <join output dir>
Prints one JSON line.
"""
import json
import os
import subprocess
import sys
import tempfile

import numpy as np
import yaml
import zarr

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import matrix  # noqa: E402
from p3_grandtour_loop_errors import read_tum  # noqa: E402
from p3_multisession_join import rot_deg, truth_transform  # noqa: E402

REPO = '/home/user/lidar_slam_ros2'


def densify(front, corrected, out):
    env = dict(os.environ, PYTHONPATH=REPO)
    subprocess.run(['python3', f'{REPO}/scripts/densify_corrected_trajectory.py', '--raw', front, '--corrected',
                    corrected, '--output', out], check=True, env=env, capture_output=True)
    return read_tum(out)


def prism_positions(traj, sdir):
    q_t = yaml.safe_load(open(f'{sdir}/extrinsics.txt'))['T_imu_prism_quat_xyzw_xyz']
    lever = matrix(q_t[4:], q_t[:4])[:3, 3]
    return np.stack([matrix(r[1:4], r[4:8])[:3, :3] @ lever + r[1:4] for r in traj])


def fit(a, b):
    ca, cb = a.mean(0), b.mean(0)
    U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def ate(traj, sdir, mission, alignment=None):
    """ATE without reference spikes; the fit is made unless alignment (R, t) is given. Returns (ate, (R, t))."""
    ref = zarr.open(f'/home/user/data/grandtour/reference_dev/{mission}/prism_position', mode='r')
    rt, rp = ref['timestamp'][:].astype(float), ref['point'][:]
    prism = prism_positions(traj, sdir)
    inside = (rt >= traj[0, 0]) & (rt <= traj[-1, 0])
    rt, rp = rt[inside], rp[inside]
    ep = np.stack([np.interp(rt, traj[:, 0], prism[:, k]) for k in range(3)], 1)
    spike = np.zeros(len(rp), dtype=bool)
    if len(rp) > 2:
        gap_prev = np.linalg.norm(rp[1:-1] - rp[:-2], axis=1)
        gap_next = np.linalg.norm(rp[1:-1] - rp[2:], axis=1)
        across = np.linalg.norm(rp[2:] - rp[:-2], axis=1)
        spike[1:-1] = (gap_prev > 1.0) & (gap_next > 1.0) & (across < 0.5 + 2.0 * (rt[2:] - rt[:-2]))
    R, t = alignment if alignment is not None else fit(ep[~spike], rp[~spike])
    err = np.linalg.norm(ep[~spike] @ R.T + t - rp[~spike], axis=1)
    return float(np.sqrt((err ** 2).mean())), (R, t)


def main():
    a_dir, b_dir, ma, mb, run_a, run_b, join_dir = sys.argv[1:8]
    report = yaml.safe_load(open(f'{join_dir}/join_report.yaml'))
    session = next(s for s in report['sessions'] if s['session'] == 1)
    out = {'join': join_dir, 'joined': bool(session['joined']), 'candidates': session['candidates'],
           'accepted': session['accepted'], 'sets_tried': session['sets_tried'],
           'ambiguous': bool(session.get('ambiguous')), 'self_similar': session.get('self_similar') or ''}
    front_a = f'{a_dir}/run/front_0/front_tum_0.txt'
    front_b = f'{b_dir}/run/front_0/front_tum_0.txt'
    with tempfile.TemporaryDirectory() as tmp:
        single_a, _ = ate(densify(front_a, f'{run_a}/trajectory_optimized.tum', f'{tmp}/a.tum'), a_dir, ma)
        single_b, _ = ate(densify(front_b, f'{run_b}/trajectory_optimized.tum', f'{tmp}/b.tum'), b_dir, mb)
        out['single_ate_a'], out['single_ate_b'] = round(single_a, 4), round(single_b, 4)
        if out['joined']:
            S = np.eye(4)
            S[:3] = np.array(session['transform_to_session_0']).reshape(3, 4)
            S_true = np.linalg.inv(truth_transform(a_dir, ma)) @ truth_transform(b_dir, mb)
            pos = read_tum(front_b)[:, 1:4]
            diff = (pos @ S[:3, :3].T + S[:3, 3]) - (pos @ S_true[:3, :3].T + S_true[:3, 3])
            out['join_error_rms_m'] = round(float(np.sqrt((np.linalg.norm(diff, axis=1) ** 2).mean())), 4)
            out['join_error_deg'] = round(rot_deg(S[:3, :3].T @ S_true[:3, :3]), 3)
            joint_a, fit_a = ate(densify(front_a, f'{join_dir}/trajectory_session0.tum', f'{tmp}/ja.tum'), a_dir, ma)
            dense_b = densify(front_b, f'{join_dir}/trajectory_session1.tum', f'{tmp}/jb.tum')
            joint_b, _ = ate(dense_b, b_dir, mb, alignment=fit_a)
            own_b, _ = ate(dense_b, b_dir, mb)
            out['joint_ate_a'], out['joint_ate_b_in_a_frame'] = round(joint_a, 4), round(joint_b, 4)
            out['joint_ate_b_own_fit'] = round(own_b, 4)
            out['consistent'] = joint_b <= single_b + 0.02 and joint_a <= single_a + 0.005
    print(json.dumps(out))


if __name__ == '__main__':
    main()
