#!/usr/bin/env python3
"""Score LUH relocalization, causal localization and replay runtime."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

LIDAR_IN_BASE = [-.006253, .011775, .028535]
BASE_FROM_LIDAR = Rotation.from_quat([0, 0, 1, 0])


def read(path, truth=False):
    poses = np.loadtxt(path, ndmin=2, skiprows=int(truth))
    if poses.shape[1] != 8 or not np.isfinite(poses).all():
        raise ValueError(f'{path}: expected finite stamp, xyz and xyzw columns')
    if truth and poses[0, 0] > 1e10:
        poses[:, 0] /= 10
    if truth:
        if np.any(np.diff(poses[:, 0]) < 0):
            raise ValueError(f'{path}: truth timestamps are out of order')
        # The published 0.1 s timestamps can repeat. Keep the first row in a bin.
        first = np.r_[True, np.diff(poses[:, 0]) > 0]
        poses = poses[first]
    if np.any(np.diff(poses[:, 0]) <= 0):
        raise ValueError(f'{path}: expected increasing timestamps')
    return poses


def pair(poses, truth):
    stamps = poses[:, 0]
    right = np.searchsorted(truth[:, 0], stamps).clip(0, len(truth) - 1)
    left = (right - 1).clip(0)
    closest = np.where(abs(truth[left, 0] - stamps) < abs(truth[right, 0] - stamps),
                       left, right)
    mask = abs(truth[closest, 0] - stamps) <= .050001
    return poses[mask], closest[mask]


def lidar(poses):
    rotations = Rotation.from_quat(poses[:, 4:])
    return poses[:, 1:4] + rotations.apply(LIDAR_IN_BASE), rotations * BASE_FROM_LIDAR


def fit(positions, truth):
    """Rigid Umeyama fit with no scaling."""
    if len(positions) < 3:
        raise ValueError('need at least three matched poses')
    a, b = positions.mean(axis=0), truth.mean(axis=0)
    u, _, vt = np.linalg.svd((truth - b).T @ (positions - a))
    rotation = u @ np.diag([1., 1., np.sign(np.linalg.det(u @ vt))]) @ vt
    return rotation, b - rotation @ a


def score(base, base_truth, estimate, alone, truth, run, start=None):
    base, paired = pair(base, base_truth)
    r, t = fit(lidar(base)[0], base_truth[paired, 1:4])
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = r, t
    runtime = run['mean_processing_seconds_per_scan']
    drops = run['dropped_trailing_frames']
    input_to_pose_gap = run['lidar_messages'] - run['live_poses']
    comparable = run.get('realtime') is True and run.get('speed') == 1.
    result = dict(store_to_tls=transform.tolist(), R={'passed': False, 'status': 'no anchor'}, L=None,
                  T=dict(mean_seconds_per_scan=runtime, dropped_trailing_frames=drops,
                         input_to_pose_gap=input_to_pose_gap,
                         peak_rss_mib=run['process_peak_rss_mib'],
                         passed=bool(comparable and drops == 0 and 0 <= input_to_pose_gap <= 1 and
                                     runtime is not None and
                                     runtime < .10 and run['process_peak_rss_mib'] is not None)))
    anchor = run.get('store_anchor_stamp_sec')
    if anchor is None:
        return result
    start = run.get('store_start_stamp_sec') if start is None else start
    if start is None:
        raise ValueError('missing first input stamp for R. Supply --start-stamp.')
    if not len(estimate):
        raise ValueError('run reports an anchor but store trajectory is empty')
    first, first_id = pair(estimate[:1], truth)
    if len(first):
        first_xyz, first_rotation = lidar(first)
        delta = first_xyz[0] @ r.T + t - truth[first_id[0], 1:4]
        angle = (Rotation.from_matrix(r) * first_rotation).as_matrix()[0]
        true = Rotation.from_quat(truth[first_id[0], 4:]).as_matrix()
        heading = np.degrees(np.arctan2(angle[1, 0], angle[0, 0]) -
                             np.arctan2(true[1, 0], true[0, 0]))
        heading = abs((heading + 180.) % 360. - 180.)
        distance = float(np.linalg.norm(delta))
        elapsed = anchor - start
        result['R'] = dict(seconds=elapsed, error_m=distance, heading_error_deg=heading,
                           passed=bool(0 <= elapsed <= 10 and distance <= .5 and heading <= 5))
    else:
        result['R'] = {'passed': False, 'status': 'first pose has no matching truth'}

    update, updated = pair(estimate, truth)
    standalone, single = pair(alone, truth)
    ids, i, j = np.intersect1d(updated, single, return_indices=True)
    reference = truth[ids, 1:4]
    solo_xyz, _ = lidar(standalone[j])
    sr, st = fit(solo_xyz, reference)
    live_error = float(np.sqrt(np.mean(np.square(lidar(update[i])[0] @ r.T + t -
                                                  reference).sum(axis=1))))
    solo_error = float(np.sqrt(np.mean(np.square(solo_xyz @ sr.T + st - reference).sum(axis=1))))
    result['L'] = dict(position_rmse_m=live_error, standalone_rmse_m=solo_error,
                       ratio=live_error / solo_error if solo_error else None,
                       matched_poses=len(ids),
                       passed=bool(live_error <= .10 and live_error <= 1.2 * solo_error))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ('base', 'base-truth', 'update-truth', 'trajectory', 'standalone', 'live-summary'):
        parser.add_argument('--' + flag, type=Path, required=True)
    parser.add_argument('--start-stamp', type=float, help='first input stamp for older summaries')
    args = parser.parse_args()
    run = json.loads(args.live_summary.read_text())
    estimate = read(args.trajectory) if run.get('store_anchor_stamp_sec') is not None else []
    report = score(read(args.base), read(args.base_truth, truth=True), estimate,
                   read(args.standalone), read(args.update_truth, truth=True), run, args.start_stamp)
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
