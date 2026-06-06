#!/usr/bin/env python3
"""Build a LiDAR-primed Gaussian init cloud from a bag + SLAM trajectory.

Accumulates PointCloud2 scans into the world (map) frame using the SLAM TUM
trajectory, voxel-downsamples, and writes a PLY that ``train_gsplat.py`` seeds
Gaussians from. This is the geometric prior at the heart of "LiDAR-primed 3DGS"
(``docs/research/3dgs-postprocess-map-design.md``): the metric LiDAR geometry
removes the need for COLMAP SfM and gives the optimiser correct positions.

The point cloud's ``header.stamp`` is already on the trajectory clock (the
trajectory is logged from the same scans), so no ``--time-offset`` is needed.
The transform/accumulation maths is delegated to ``posed_images`` /
``pointcloud_io`` (pure, tested); only the bag/PointCloud2 reading needs ROS.
"""

from __future__ import annotations

import argparse
from typing import Optional, Sequence

import numpy as np

import pointcloud_io as pcio
import posed_images as pi


def transform_points(points: np.ndarray, world_T_body: np.ndarray) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to ``points`` (N,3)."""
    pts = np.asarray(points, dtype=np.float64)
    return (pts @ world_T_body[:3, :3].T) + world_T_body[:3, 3]


def _read_pointcloud_xyz(msg) -> np.ndarray:
    """Extract finite XYZ (N,3) from a sensor_msgs/PointCloud2 message."""
    from sensor_msgs_py import point_cloud2

    pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                         skip_nans=True)
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
    return pts[np.isfinite(pts).all(axis=1)]


def build(args: argparse.Namespace) -> dict:
    """Accumulate scans into world points and write the init PLY."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    samples = pi.read_tum_trajectory(args.traj)

    import rosbag2_py
    # Reuse the extractor's reader factory so FILE-compressed (zstd) bags work.
    from extract_posed_images import _open_reader
    reader = _open_reader(args.bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.points_topic]))

    chunks: list[np.ndarray] = []
    used = 0
    skipped = 0
    seen = 0
    t0 = None
    while reader.has_next():
        tname, raw, bagt = reader.read_next()
        if tname != args.points_topic:
            continue
        rel_t = 0.0 if t0 is None else (bagt * 1e-9 - t0)
        if t0 is None:
            t0 = bagt * 1e-9
        if args.end_time >= 0 and rel_t > args.end_time:
            break
        if rel_t < args.start_time:
            continue
        if args.stride > 1 and seen % args.stride != 0:
            seen += 1
            continue
        seen += 1
        msg = deserialize_message(raw, PointCloud2)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            world_T_body = pi.interpolate_pose(
                samples, ts, max_extrapolation=args.max_extrapolation)
        except ValueError:
            skipped += 1
            continue
        pts = _read_pointcloud_xyz(msg)
        if args.max_range > 0:
            rng = np.linalg.norm(pts, axis=1)
            pts = pts[rng <= args.max_range]
        world_pts = transform_points(pts, world_T_body).astype(np.float32)
        # Downsample each scan before accumulating so peak memory is bounded by
        # the downsampled cloud, not the sum of every raw scan (a long bag is
        # tens of millions of points before any reduction).
        world_pts, _ = pcio.voxel_downsample(world_pts, args.voxel)
        chunks.append(world_pts)
        used += 1

    if not chunks:
        raise RuntimeError('no scans accumulated; check --points-topic / trajectory')
    world = np.concatenate(chunks, axis=0)
    # Final pass dedups points that fell in the same voxel across scan boundaries.
    world, _ = pcio.voxel_downsample(world, args.voxel)
    if args.max_points > 0 and world.shape[0] > args.max_points:
        rng = np.random.default_rng(0)
        world = world[rng.choice(world.shape[0], args.max_points, replace=False)]
    rgb = None
    colored = 0
    if args.color_transforms:
        rgb, seen = _colorize(world, args.color_transforms)
        colored = int(seen.sum())
    out = pcio.write_ply(args.out, world, rgb)
    return {'scans_used': used, 'scans_skipped': skipped,
            'points': int(world.shape[0]), 'colored': colored, 'out': str(out)}


def _colorize(world: np.ndarray, transforms_path: str):
    """Project ``world`` points into the posed images of a transforms.json."""
    import imageio.v3 as iio
    import train_gsplat as tg

    ds = tg.load_transforms(transforms_path)
    images = [np.asarray(iio.imread(p)) for p in ds['image_paths']]
    return pcio.colorize_by_projection(
        world, ds['viewmats'], ds['K'], images, ds['width'], ds['height'])


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--bag', required=True)
    p.add_argument('--traj', required=True, help='SLAM trajectory (TUM, world<-body)')
    p.add_argument('--points-topic', default='/livox/points')
    p.add_argument('--out', required=True, help='output init .ply')
    p.add_argument('--voxel', type=float, default=0.1, help='voxel size (m)')
    p.add_argument('--max-range', type=float, default=80.0, help='drop points beyond (m)')
    p.add_argument('--max-points', type=int, default=300000, help='cap final point count')
    p.add_argument('--max-extrapolation', type=float, default=0.1)
    p.add_argument('--stride', type=int, default=1, help='use every Nth scan')
    p.add_argument('--start-time', type=float, default=0.0,
                   help='use scans at/after this many seconds from bag start')
    p.add_argument('--end-time', type=float, default=-1.0,
                   help='use scans up to this many seconds from bag start (-1 = all)')
    p.add_argument('--color-transforms', default=None,
                   help='transforms.json (+ images) to colour the init cloud by '
                        'projection; seeds Gaussian colour instead of flat grey')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    summary = build(args)
    print(f"accumulated {summary['scans_used']} scans "
          f"({summary['scans_skipped']} skipped) -> "
          f"{summary['points']} points "
          f"({summary['colored']} coloured) -> {summary['out']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
