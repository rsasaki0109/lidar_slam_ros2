#!/usr/bin/env python3
"""
Livox MID-360 range error against the Hesai XT-32 on a moving GrandTour mission, with no reference (chapter 33).

The Hesai frontend trajectory (T_world_imu at every Hesai scan, from a Hesai + STIM320 run) places the Hesai points
of every k-th scan in the world, point by point at its own time (deskewed by interpolating the trajectory); that
cloud, thinned to a voxel grid with a surface normal per point, is the reference surface. The Livox points of every
k-th Livox scan are placed with the same trajectory and the Livox extrinsic, and each is compared with the nearest
Hesai surface along its own ray: d_ray = (n . (p - q)) / (n . u) for points whose ray meets the surface at less than
60 deg from its normal. The result is binned by range and by elevation in the Livox frame.

Usage: p2_livox_range_model.py <mission dir with livox_points/ hesai_points/> <hesai run front_tum_0.txt (T_world_imu)>
           <hesai run extrinsics.txt> <livox run extrinsics.txt> [scan stride, default 20] [out.npz]
"""
import sys

import numpy as np

import yaml
import zarr
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import iter_scans, matrix  # noqa: E402


def trajectory(path):
    a = np.loadtxt(path)
    a = a[np.r_[True, np.diff(a[:, 0]) > 0]]
    return a[:, 0], a[:, 1:4], Rotation.from_quat(a[:, 4:8])


def poses_at(traj, t):
    times, pos, rot = traj
    t = np.clip(t, times[0], times[-1])
    p = np.stack([np.interp(t, times, pos[:, k]) for k in range(3)], 1)
    return p, Slerp(times, rot)(t)


def extrinsic(path):
    q = yaml.safe_load(open(path))['extrinsic_lidar2base_quat_xyzw_xyz']
    return matrix(q[4:], q[:4])


def to_world(traj, T_imu_lidar, xyz, t):
    p_imu = xyz @ T_imu_lidar[:3, :3].T + T_imu_lidar[:3, 3]
    pos, rot = poses_at(traj, t)
    return rot.apply(p_imu) + pos


def voxel(p, v):
    _, idx = np.unique(np.floor(p / v).astype(np.int64), axis=0, return_index=True)
    return p[idx]


def main():
    mission, tum, ext_h, ext_l = sys.argv[1:5]
    stride = int(sys.argv[5]) if len(sys.argv) > 5 else 20
    out = sys.argv[6] if len(sys.argv) > 6 else None
    traj = trajectory(tum)
    T_ih, T_il = extrinsic(ext_h), extrinsic(ext_l)
    hesai = zarr.open(f'{mission}/hesai_points', mode='r')
    livox = zarr.open(f'{mission}/livox_points', mode='r')
    t_lo, t_hi = traj[0][0] + 20.0, traj[0][-1]  # leave out the start (Hesai start failures)

    world = []
    for i, (xyz, t, _) in enumerate(iter_scans(hesai)):
        if i % stride or t.mean() < t_lo or t.mean() > t_hi:
            continue
        r = np.linalg.norm(xyz, axis=1)
        keep = (r > 1.0) & (r < 60.0)
        world.append(voxel(to_world(traj, T_ih, xyz[keep], t[keep]), 0.03))
    world = voxel(np.concatenate(world), 0.03)
    tree = cKDTree(world)
    _, nn = tree.query(world, k=10)
    nb = world[nn] - world[nn].mean(1, keepdims=True)
    w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', nb, nb))
    normals, planar = v[:, :, 0], (w[:, 1] - w[:, 0]) / np.maximum(w[:, 2], 1e-12)
    print(f'hesai surface: {len(world)} points')

    rows = []
    for i, (xyz, t, _) in enumerate(iter_scans(livox)):
        if i % stride or t.mean() < t_lo or t.mean() > t_hi:
            continue
        r = np.linalg.norm(xyz, axis=1)
        keep = (r > 1.0) & (r < 60.0)
        xyz, t, r = xyz[keep], t[keep], r[keep]
        pw = to_world(traj, T_il, xyz, t)
        dist, idx = tree.query(pw, distance_upper_bound=0.3)
        ok = np.isfinite(dist)
        idx = np.minimum(idx, len(world) - 1)
        ok &= planar[idx] > 0.6
        _, rot = poses_at(traj, t)
        u = rot.apply((xyz / r[:, None]) @ T_il[:3, :3].T)
        n = normals[idx]
        nu = np.einsum('ij,ij->i', n, u)
        ok &= np.abs(nu) > 0.5
        d_ray = np.einsum('ij,ij->i', n, pw - world[idx]) / np.where(np.abs(nu) > 0, nu, 1)
        elev = np.degrees(np.arcsin(np.clip(xyz[:, 2] / r, -1, 1)))
        rows.append(np.stack([r[ok], elev[ok], d_ray[ok], np.degrees(np.arccos(np.clip(np.abs(nu[ok]), 0, 1)))], 1))
    R = np.concatenate(rows)
    print(f'livox points compared: {len(R)}')
    rng, elev, d, inc = R.T
    inlier = np.abs(d) < 0.15
    print('along-ray residual (m, + = Livox measures the range too long), median per bin:')
    print('  range    n       median    p25      p75')
    for lo, hi in ((1, 3), (3, 5), (5, 8), (8, 12), (12, 18), (18, 25), (25, 35), (35, 60)):
        k = inlier & (rng >= lo) & (rng < hi)
        if k.sum() > 100:
            q1, q2, q3 = np.percentile(d[k], [25, 50, 75])
            print(f'  {lo:2d}-{hi:<2d} {k.sum():7d}  {q2 * 1000:+7.1f} mm {q1 * 1000:+7.1f} {q3 * 1000:+7.1f}')
    print('  elevation (Livox frame, deg):')
    for lo, hi in ((-10, 0), (0, 10), (10, 20), (20, 30), (30, 40), (40, 50), (50, 60)):
        k = inlier & (elev >= lo) & (elev < hi)
        if k.sum() > 100:
            print(f'  {lo:+3d}..{hi:+3d}  n {k.sum():7d}  median {np.median(d[k]) * 1000:+6.1f} mm   '
                  f'median range {np.median(rng[k]):5.1f} m')
    print('  incidence (deg):')
    for lo, hi in ((0, 20), (20, 40), (40, 60)):
        k = inlier & (inc >= lo) & (inc < hi)
        if k.sum() > 100:
            print(f'  {lo:2d}..{hi:<2d}  n {k.sum():7d}  median {np.median(d[k]) * 1000:+6.1f} mm')
    A = np.stack([rng[inlier], np.ones(inlier.sum())], 1)
    coef = np.linalg.lstsq(A, d[inlier], rcond=None)[0]
    print(f'least squares d_ray = {coef[0] * 100:+.3f} % * range {coef[1] * 1000:+.1f} mm')
    if out:
        np.savez_compressed(out, rows=R)


if __name__ == '__main__':
    main()
