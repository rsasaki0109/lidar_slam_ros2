#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Turn an Oxford Spires sequence into a colour benchmark for the map tools.

Oxford Spires (https://dynamic.robots.ox.ac.uk/datasets/oxford-spires/,
CC BY-NC-SA 4.0) pairs a handheld rig (three fisheye cameras and a Hesai QT64)
with a terrestrial laser scan of each site whose points carry RGB from
separate high-resolution cameras. That scan is an independent colour
reference for ``scripts/evaluate_colored_map_reference.py``. This script
writes, in the scan's frame:

- ``posed/transforms.json`` and ``posed/images``: every ``--image-stride``-th
  frame of each camera, rectified from the equidistant fisheye model to one
  shared pinhole (``--focal`` and the image centre) so ``train_gsplat``'s
  loader and ``recolor_pointcloud.py`` accept them. Poses are the
  ground-truth ``base`` trajectory interpolated at the image stamp, composed
  with ``T_base_lidar`` and the camera-LiDAR extrinsics.
- ``map.ply``: the dataset's undistorted clouds placed with the same
  trajectory and voxel-thinned, standing in for a SLAM map. The image-synced
  clouds under ``processed/lidar-undistortion`` are in the LiDAR frame
  (``--clouds-frame lidar``, the default; checked against the scan: 0.035 m
  median versus 0.36 m if read as ``base``); VILENS's own clouds are in
  ``base``.
- ``reference.ply``: the scan, cropped to ``--reference-radius`` around the
  trajectory.

Inputs are the unpacked ``raw/images.zip`` (``cam0/<stamp>.jpg`` ...),
the unpacked ``processed/lidar-undistortion/undist-clouds-image-synced.zip``,
``processed/trajectory/gt-tum.txt``, the site's ``merged-clouds-5cm.pcd``,
and the ``calibration`` folder (``cam<n>.yaml``, ``cam-lidar.yaml`` or
``cam-lidar-imu.yaml``).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_DIR = REPO_ROOT / 'tools' / 'colored_map'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import pointcloud_io as pcio  # noqa: E402
import posed_images as pi  # noqa: E402

# Dataset convention: base frame 0.124 m above the LiDAR, rotated 180 deg
# about z (calibration/README.md).
T_BASE_LIDAR = pi.make_transform([0.0, 0.0, 0.124], [0.0, 0.0, 1.0, 0.0])


def stamp_from_name(name: str) -> float:
    """Parse ``1710338098.943278592`` or ``cloud_1710338098_943278592``."""
    digits = re.findall(r'\d+', Path(name).stem)
    if len(digits) >= 2 and len(digits[-1]) == 9:
        return int(digits[-2]) + int(digits[-1]) * 1.0e-9
    if len(digits) >= 2:
        return float(f'{digits[-2]}.{digits[-1]}')
    if digits:
        return float(digits[-1])
    raise ValueError(f'no timestamp in {name}')


def load_calibration(calibration_dir: Path, cameras) -> dict:
    """Return per-camera ``K``, fisheye ``D`` and ``T_cam_lidar``."""
    import yaml

    extrinsics_path = next(
        (calibration_dir / name for name in ('cam-lidar.yaml', 'cam-lidar-imu.yaml')
         if (calibration_dir / name).is_file()), None)
    if extrinsics_path is None:
        raise SystemExit(f'{calibration_dir}: no cam-lidar(-imu).yaml')
    extrinsics = yaml.safe_load(extrinsics_path.read_text())
    out = {}
    for camera in cameras:
        intrinsics = yaml.safe_load((calibration_dir / f'{camera}.yaml').read_text())
        if intrinsics.get('distortion_model') != 'equidistant':
            raise SystemExit(f'{camera}: expected the equidistant model')
        out[camera] = {
            'K': np.asarray(intrinsics['camera_matrix']['data'],
                            dtype=np.float64).reshape(3, 3),
            'D': np.asarray(intrinsics['distortion_coefficients']['data'],
                            dtype=np.float64),
            'size': (int(intrinsics['image_width']),
                     int(intrinsics['image_height'])),
            'T_cam_lidar': np.asarray(extrinsics[camera]['T_cam_lidar'],
                                      dtype=np.float64),
        }
    return out


def world_T_camera(trajectory, stamp: float, T_cam_lidar: np.ndarray,
                   max_gap: float) -> np.ndarray:
    """``world <- camera optical`` at ``stamp`` from the base trajectory."""
    world_T_base = pi.interpolate_pose(trajectory, stamp,
                                       max_extrapolation=max_gap)
    return world_T_base @ T_BASE_LIDAR @ np.linalg.inv(T_cam_lidar)


def main(argv=None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--images', type=Path, required=True,
                        help='unpacked raw images (cam0/ cam1/ cam2/)')
    parser.add_argument('--clouds', type=Path, required=True,
                        help='unpacked undistorted clouds')
    parser.add_argument('--clouds-frame', choices=('lidar', 'base'),
                        default='lidar')
    parser.add_argument('--trajectory', type=Path, required=True,
                        help='processed/trajectory/gt-tum.txt')
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True,
                        help='ground_truth_map/<site>/merged-clouds-5cm.pcd')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--cameras', default='cam0,cam1,cam2')
    parser.add_argument('--image-stride', type=int, default=20)
    parser.add_argument('--start', type=float, default=0.0,
                        help='seconds after the first image')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='seconds to keep (0 keeps all)')
    parser.add_argument('--focal', type=float, default=530.0,
                        help='shared pinhole focal length (px) after rectification')
    parser.add_argument('--cloud-stride', type=int, default=2)
    parser.add_argument('--voxel', type=float, default=0.02)
    parser.add_argument('--min-range', type=float, default=1.0)
    parser.add_argument('--max-range', type=float, default=40.0)
    parser.add_argument('--reference-radius', type=float, default=30.0)
    parser.add_argument('--max-pose-gap', type=float, default=0.05)
    args = parser.parse_args(argv)
    import cv2
    from scipy.spatial import cKDTree

    cameras = [c.strip() for c in args.cameras.split(',') if c.strip()]
    calibration = load_calibration(args.calibration, cameras)
    trajectory = pi.read_tum_trajectory(args.trajectory)
    width, height = calibration[cameras[0]]['size']
    shared = np.array([[args.focal, 0.0, width / 2.0],
                       [0.0, args.focal, height / 2.0], [0.0, 0.0, 1.0]])
    image_dir = args.out / 'posed' / 'images'
    image_dir.mkdir(parents=True, exist_ok=True)

    stamps = {camera: sorted((args.images / camera).glob('*.jpg'),
                             key=lambda p: stamp_from_name(p.name))
              for camera in cameras}
    first = min(stamp_from_name(paths[0].name) for paths in stamps.values())
    start = first + args.start
    stop = start + args.duration if args.duration > 0.0 else float('inf')
    frames = []
    for camera in cameras:
        cal = calibration[camera]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            cal['K'], cal['D'], np.eye(3), shared, (width, height),
            cv2.CV_16SC2)
        selected = [p for p in stamps[camera]
                    if start <= stamp_from_name(p.name) <= stop]
        for path in selected[::args.image_stride]:
            stamp = stamp_from_name(path.name)
            try:
                pose = world_T_camera(trajectory, stamp, cal['T_cam_lidar'],
                                      args.max_pose_gap)
            except ValueError:
                continue
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            rectified = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
            name = f'{camera}_{path.stem}.jpg'
            cv2.imwrite(str(image_dir / name), rectified,
                        [cv2.IMWRITE_JPEG_QUALITY, 95])
            frames.append(pi.PosedImage(f'images/{name}', pose, stamp))
    frames.sort(key=lambda frame: frame.stamp)
    pi.write_transforms(
        args.out / 'posed' / 'transforms.json',
        pi.CameraIntrinsics(width, height, args.focal, args.focal,
                            width / 2.0, height / 2.0), frames)

    clouds = sorted(args.clouds.rglob('*.pcd'), key=lambda p: stamp_from_name(p.name))
    clouds = [p for p in clouds if start <= stamp_from_name(p.name) <= stop]
    chunks = []
    used = 0
    for path in clouds[::args.cloud_stride]:
        try:
            world_T_base = pi.interpolate_pose(
                trajectory, stamp_from_name(path.name),
                max_extrapolation=args.max_pose_gap)
        except ValueError:
            continue
        xyz, _ = pcio.read_point_cloud_xyz(path)
        xyz = np.asarray(xyz, dtype=np.float64)
        rng = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(rng >= args.min_range) & (rng <= args.max_range)]
        world_T_cloud = (world_T_base @ T_BASE_LIDAR
                         if args.clouds_frame == 'lidar' else world_T_base)
        chunks.append(xyz @ world_T_cloud[:3, :3].T + world_T_cloud[:3, 3])
        used += 1
        if len(chunks) >= 50:
            merged, _ = pcio.voxel_downsample(np.concatenate(chunks), args.voxel)
            chunks = [merged]
    if not chunks:
        raise SystemExit('no cloud overlaps the trajectory window')
    world, _ = pcio.voxel_downsample(np.concatenate(chunks), args.voxel)
    pcio.write_ply(args.out / 'map.ply', world)

    reference_xyz, reference_rgb = pcio.read_point_cloud_xyz(args.reference)
    positions = np.array([
        sample.translation for sample in trajectory if start <= sample.stamp <= stop])
    distance, _ = cKDTree(positions).query(
        reference_xyz, distance_upper_bound=args.reference_radius)
    keep = np.isfinite(distance)
    pcio.write_ply(args.out / 'reference.ply', reference_xyz[keep],
                   None if reference_rgb is None else reference_rgb[keep])
    summary = {'frames': len(frames), 'clouds_used': used,
               'map_points': int(len(world)),
               'reference_points': int(keep.sum()),
               'window': [start, None if stop == float('inf') else stop]}
    (args.out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
