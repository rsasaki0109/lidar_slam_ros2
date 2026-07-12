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

"""Build a robust coloured SLAM map from one bag and a TUM trajectory.

This is the user-facing entry point over ``extract_posed_images.py`` and
``build_lidar_init.py``. Existing posed images and maps are reused by default,
so an interrupted or repeated run only performs missing work.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Sequence


TOOL_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOL_DIR.parents[1]


def effective_trajectory(args) -> Path:
    """Return the trajectory consumed by image and map generation."""
    if args.raw_traj is None:
        return Path(args.traj)
    return Path(args.out) / 'dense_corrected_trajectory.tum'


def is_stale(output: Path, inputs: Sequence[Path]) -> bool:
    """Return whether an existing output predates any available input."""
    if not output.is_file():
        return True
    output_mtime = output.stat().st_mtime_ns
    return any(path.is_file() and path.stat().st_mtime_ns > output_mtime
               for path in inputs)


def validate_trajectory_density(path: Path, max_gap: float) -> None:
    """Reject sparse graph keyframes that cannot register individual scans."""
    if max_gap <= 0:
        return
    stamps = []
    for line in path.read_text().splitlines():
        fields = line.split()
        if fields and not fields[0].startswith('#'):
            stamps.append(float(fields[0]))
    if len(stamps) < 2:
        raise ValueError(f'{path}: trajectory needs at least two poses')
    largest = max(b - a for a, b in zip(stamps, stamps[1:]))
    if largest > max_gap:
        raise ValueError(
            f'{path}: trajectory pose gap {largest:.3f}s exceeds '
            f'--max-trajectory-gap {max_gap:.3f}s; use a dense SLAM '
            'trajectory rather than sparse pose-graph keyframes')


def build_commands(args) -> list[tuple[str, list[str]]]:
    """Return the missing/forced pipeline stages as ``(name, argv)`` pairs."""
    out_dir = Path(args.out)
    posed_dir = out_dir / 'posed_images'
    transforms = posed_dir / 'transforms.json'
    colored_map = out_dir / 'colored_map.ply'
    extrinsic_path = (Path(args.extrinsic) if args.extrinsic is not None else
                      out_dir / 'generated_body_camera_extrinsic.json')
    commands = []
    trajectory = effective_trajectory(args)

    rebuild_trajectory = (args.raw_traj is not None and (
        args.force_trajectory or is_stale(
            trajectory, [Path(args.raw_traj), Path(args.traj)])))
    if rebuild_trajectory:
        commands.append(('dense corrected trajectory', [
            sys.executable,
            str(REPO_ROOT / 'scripts' / 'densify_corrected_trajectory.py'),
            '--raw', str(args.raw_traj), '--corrected', str(args.traj),
            '--output', str(trajectory),
            '--max-anchor-offset', str(args.max_anchor_offset),
        ]))

    rebuild_images = (rebuild_trajectory or args.force_images or
                      is_stale(transforms, [trajectory]))
    if rebuild_images:
        extract = [
            sys.executable, str(TOOL_DIR / 'extract_posed_images.py'),
            '--bag', str(args.bag), '--traj', str(trajectory),
            '--camera-topic', args.camera_topic,
            '--camera-info-topic', args.camera_info_topic,
            '--extrinsic', str(extrinsic_path), '--out', str(posed_dir),
            '--time-offset', args.time_offset,
            '--clock-reference-topic', args.points_topic,
            '--stride', str(args.image_stride),
            '--start-time', str(args.start_time), '--end-time', str(args.end_time),
        ]
        if not args.no_undistort:
            extract.append('--undistort')
        if args.intrinsics_yaml is not None:
            extract.extend(['--intrinsics-yaml', str(args.intrinsics_yaml)])
        commands.append(('posed images', extract))

    if (rebuild_images or args.force_map or
            is_stale(colored_map, [trajectory, transforms])):
        build = [
            sys.executable, str(TOOL_DIR / 'build_lidar_init.py'),
            '--bag', str(args.bag), '--traj', str(trajectory),
            '--points-topic', args.points_topic, '--out', str(colored_map),
            '--voxel', str(args.voxel), '--max-points', str(args.max_points),
            '--min-range', str(args.min_range), '--max-range', str(args.max_range),
            '--stride', str(args.scan_stride),
            '--start-time', str(args.start_time), '--end-time', str(args.end_time),
            '--color-transforms', str(transforms), '--color-robust',
        ]
        if args.lidar_calibration is not None:
            build.extend([
                '--lidar-calibration', str(args.lidar_calibration),
                '--lidar-key', args.lidar_key,
            ])
        commands.append(('coloured map', build))
    return commands


def run_pipeline(args) -> dict:
    """Execute missing stages and return paths plus the stages that ran."""
    out_dir = Path(args.out)
    if args.kalibr_camchain is not None and args.lidar_calibration is None:
        raise ValueError('--kalibr-camchain requires --lidar-calibration')
    if not args.dry_run:
        out_dir.mkdir(parents=True, exist_ok=True)
        if args.kalibr_camchain is not None:
            from extract_posed_images import load_kalibr_body_camera_extrinsic
            matrix = load_kalibr_body_camera_extrinsic(
                args.kalibr_camchain, args.lidar_calibration,
                camera_key=args.camera_key, lidar_key=args.lidar_key)
            generated = out_dir / 'generated_body_camera_extrinsic.json'
            generated.write_text(json.dumps({'matrix': matrix.tolist()}, indent=2))
    commands = build_commands(args)
    trajectory = effective_trajectory(args)
    trajectory_validated = False
    for name, command in commands:
        if (not args.dry_run and name != 'dense corrected trajectory' and
                not trajectory_validated):
            validate_trajectory_density(
                trajectory, args.max_trajectory_gap)
            trajectory_validated = True
        print(f'[{name}]', ' '.join(command))
        if not args.dry_run:
            subprocess.run(command, check=True)
    if not args.dry_run and not trajectory_validated:
        validate_trajectory_density(trajectory, args.max_trajectory_gap)
    return {
        'stages': [name for name, _ in commands],
        'transforms': out_dir / 'posed_images' / 'transforms.json',
        'colored_map': out_dir / 'colored_map.ply',
        'trajectory': trajectory,
    }


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('bag', type=Path, help='rosbag2 directory')
    p.add_argument('traj', type=Path,
                   help='corrected SLAM trajectory (TUM, world<-body)')
    p.add_argument('out', type=Path, help='output directory')
    p.add_argument('--raw-traj', type=Path,
                   help='dense pre-optimization TUM trajectory; propagate the '
                        'corrections from traj before colouring')
    p.add_argument('--max-anchor-offset', type=float, default=0.2,
                   help='allowed corrected-anchor extrapolation into the raw '
                        'trajectory (s)')
    calibration = p.add_mutually_exclusive_group(required=True)
    calibration.add_argument('--extrinsic', type=Path,
                             help='body<-camera YAML/JSON or vlcal calib.json')
    calibration.add_argument('--kalibr-camchain', type=Path,
                             help='Kalibr camera chain with T_cam_imu')
    p.add_argument('--lidar-calibration', type=Path,
                   help='parented LiDAR calibration paired with Kalibr camchain')
    p.add_argument('--camera-key', default='cam0')
    p.add_argument('--lidar-key', default='PandarXT-32')
    p.add_argument('--points-topic', default='/livox/points')
    p.add_argument('--camera-topic', default='/image')
    p.add_argument('--camera-info-topic', default='/camera_info')
    p.add_argument('--intrinsics-yaml', type=Path,
                   help='camera intrinsics YAML when the bag has no CameraInfo')
    p.add_argument('--time-offset', default='auto',
                   help='camera-to-trajectory clock offset or auto')
    p.add_argument('--max-trajectory-gap', type=float, default=0.5,
                   help='reject sparse pose streams with a larger gap (s); '
                        'set <=0 to disable')
    p.add_argument('--image-stride', type=int, default=1)
    p.add_argument('--scan-stride', type=int, default=1)
    p.add_argument('--voxel', type=float, default=0.1)
    p.add_argument('--max-points', type=int, default=300000)
    p.add_argument('--min-range', type=float, default=0.0)
    p.add_argument('--max-range', type=float, default=80.0)
    p.add_argument('--start-time', type=float, default=0.0)
    p.add_argument('--end-time', type=float, default=-1.0)
    p.add_argument('--no-undistort', action='store_true')
    p.add_argument('--force-images', action='store_true')
    p.add_argument('--force-map', action='store_true')
    p.add_argument('--force-trajectory', action='store_true')
    p.add_argument('--dry-run', action='store_true')
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    summary = run_pipeline(args)
    if summary['stages']:
        print('completed stages:', ', '.join(summary['stages']))
    else:
        print('everything is up to date; nothing to do')
    print('trajectory:', summary['trajectory'])
    print('coloured map:', summary['colored_map'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
