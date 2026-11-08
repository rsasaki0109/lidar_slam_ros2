#!/usr/bin/env python3
"""Stale points of an earlier session that a later session saw through (map update for joined sessions).

Run with the ROS environment sourced and `python3 -P`. Plan: bench_report/data/map_update_2026-11-08/plan.txt.

p6_map_change.py --old-bag BAG --old-poses TUM [--old-transform 12] --new-bag BAG --new-poses TUM [--new-transform 12]
                 --out DIR [--ghosts]

The bags are backend inputs (deskewed scans /rko_lio/frame in the base frame, one per odometry message); the poses are a
session's corrected per-frame trajectory (densify_corrected_trajectory.py), mapped into the reference frame by the
session's transform_to_session_0 (12 values, row-major 3 x 4; identity when omitted).

The old map: every STRIDE-th old frame, one point per 0.1 m voxel. The new session's rays: every STRIDE-th new frame,
one ray per 0.2 m voxel of the scan, from the sensor to the point, sampled every 0.1 m up to MARGIN before the point.
An old point is stale when its 0.2 m voxel was crossed by rays of at least MIN_FREE new frames and no new point lies in
that voxel or one of its 26 neighbours. With --ghosts, 0.4 m cubes of points are added to the old map on the new
trajectory every 10 m of path (where the new sensor itself was, so free space), and their removal is counted apart.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

STRIDE = 5
MAP_VOXEL = 0.1
VOXEL = 0.2
STEP = 0.1
MARGIN = 0.3
MIN_RANGE, MAX_RANGE = 1.5, 40.0  # step 1: 1.0 (the carrier just beyond it)
MIN_FREE = 2
GHOST_SPACING_M, GHOST_SIDE = 10.0, 0.4


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_frames(bag, stride):
    """(stamp, points Nx3 float64) of every stride-th /rko_lio/frame message."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/rko_lio/frame']))
    k = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        k += 1
        if (k - 1) % stride:
            continue
        m = deserialize_message(data, PointCloud2)
        off = {f.name: f.offset for f in m.fields}
        raw = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
        xyz = np.stack([raw[:, off[c]:off[c] + 4].copy().view('<f4').ravel() for c in 'xyz'], axis=1).astype(float)
        yield m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, xyz[np.isfinite(xyz).all(axis=1)]


def read_poses(path, transform):
    rows = np.loadtxt(path, comments='#', ndmin=2)
    T = np.eye(4)
    if transform is not None:
        T[:3, :] = np.asarray(transform, dtype=float).reshape(3, 4)
    poses = []
    for r in rows:
        P = np.eye(4)
        P[:3, :3], P[:3, 3] = quat_to_R(r[4:8]), r[1:4]
        poses.append(T @ P)
    return rows[:, 0], np.asarray(poses)


def pose_at(stamps, poses, t):
    i = int(np.argmin(np.abs(stamps - t)))
    return poses[i] if abs(stamps[i] - t) < 0.02 else None


def keys(points, voxel):
    v = np.floor(points / voxel).astype(np.int64) + (1 << 20)
    return (v[:, 0] << 42) | (v[:, 1] << 21) | v[:, 2]


def first_per_voxel(points, voxel):
    _, idx = np.unique(keys(points, voxel), return_index=True)
    return points[np.sort(idx)]


def frames_in_ref(bag, stamps, poses, extrinsic):
    for t, xyz in read_frames(bag, STRIDE):
        xyz = xyz @ extrinsic[:3, :3].T + extrinsic[:3, 3]
        P = pose_at(stamps, poses, t)
        if P is None:
            continue
        r = np.linalg.norm(xyz - extrinsic[:3, 3], axis=1)  # range from the LiDAR
        xyz = xyz[(r >= MIN_RANGE) & (r <= MAX_RANGE)]
        yield P @ extrinsic[:, 3], xyz @ P[:3, :3].T + P[:3, 3]


def ghost_cubes(positions):
    g = np.arange(-GHOST_SIDE / 2, GHOST_SIDE / 2 + 1e-9, 0.05)
    a, b = np.meshgrid(g, g)
    a, b = a.ravel(), b.ravel()
    h = np.full_like(a, GHOST_SIDE / 2)
    face = np.concatenate([np.stack(p, axis=1) for p in
                           ((a, b, h), (a, b, -h), (a, h, b), (a, -h, b), (h, a, b), (-h, a, b))])
    face = np.unique(np.round(face, 4), axis=0)
    path = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(positions, axis=0), axis=1))])
    centers, nxt = [], GHOST_SPACING_M
    for p, s in zip(positions, path):
        if s >= nxt:
            centers.append(p)
            nxt += GHOST_SPACING_M
    centers = np.asarray(centers)
    return np.concatenate([face + c for c in centers]) if len(centers) else np.zeros((0, 3)), len(centers)


def main():
    ap = argparse.ArgumentParser()
    for side in ('old', 'new'):
        ap.add_argument(f'--{side}-bag', required=True)
        ap.add_argument(f'--{side}-poses', required=True)
        ap.add_argument(f'--{side}-transform', type=float, nargs=12)
    ap.add_argument('--extrinsic', type=float, nargs=7, help='LiDAR in the base frame: qx qy qz qw x y z')
    ap.add_argument('--out', required=True)
    ap.add_argument('--ghosts', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    ext = np.eye(4)
    if a.extrinsic:
        ext[:3, :3], ext[:3, 3] = quat_to_R(a.extrinsic[:4]), a.extrinsic[4:]
    os_, op = read_poses(a.old_poses, a.old_transform)
    ns, npo = read_poses(a.new_poses, a.new_transform)
    old = first_per_voxel(np.concatenate([xyz for _, xyz in frames_in_ref(a.old_bag, os_, op, ext)]), MAP_VOXEL)
    n_real = len(old)
    n_ghost_cubes = 0
    if a.ghosts:
        g, n_ghost_cubes = ghost_cubes((npo @ ext[:, 3])[:, :3])
        old = np.concatenate([old, g])
    ok = keys(old, VOXEL)
    uk, inv = np.unique(ok, return_inverse=True)
    free = np.zeros(len(uk), dtype=np.int64)
    hits = []
    frames = 0
    for origin, pts in frames_in_ref(a.new_bag, ns, npo, ext):
        frames += 1
        pts = first_per_voxel(pts, VOXEL)
        hits.append(np.unique(keys(pts, VOXEL)))
        d = np.linalg.norm(pts - origin, axis=1)
        n = np.maximum(np.floor((d - MARGIN) / STEP).astype(np.int64), 0)
        ray = np.repeat(np.arange(len(pts)), n)
        step = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n) + 1
        samples = origin + (pts[ray] - origin) * (step * STEP / d[ray])[:, None]
        sk = np.unique(keys(samples, VOXEL))
        pos = np.minimum(np.searchsorted(uk, sk), len(uk) - 1)
        free[pos[uk[pos] == sk]] += 1  # sk is unique, so each crossed old voxel counts once per frame
    hit = np.unique(np.concatenate(hits)) if hits else np.zeros(0, dtype=np.int64)
    protected = np.zeros(len(uk), dtype=bool)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                shifted = uk + (dx << 42) + (dy << 21) + dz
                protected |= np.isin(shifted, hit)
    stale_voxel = (free >= MIN_FREE) & ~protected
    stale = stale_voxel[inv]
    observed = (free >= MIN_FREE)[inv]
    real, ghost = slice(0, n_real), slice(n_real, len(old))
    out = {'old_points': n_real, 'new_frames_used': frames, 'old_frames_stride': STRIDE,
           'removed': int(stale[real].sum()), 'removed_fraction': round(float(stale[real].mean()), 5),
           'observed_free_fraction': round(float(observed[real].mean()), 5),
           'removed_of_observed': round(float(stale[real].sum() / max(observed[real].sum(), 1)), 5),
           'seconds': round(time.time() - t0, 1)}
    if a.ghosts:
        out.update({'ghost_cubes': n_ghost_cubes, 'ghost_points': int(len(old) - n_real),
                    'ghost_recall': round(float(stale[ghost].mean()), 4) if len(old) > n_real else None})
    np.save(f'{a.out}/old_points.npy', old[real].astype(np.float32))
    np.save(f'{a.out}/stale.npy', stale[real])
    with open(f'{a.out}/summary.json', 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out))


if __name__ == '__main__':
    sys.exit(main())
