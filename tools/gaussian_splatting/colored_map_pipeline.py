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
from pathlib import Path
import subprocess
import sys
from typing import Sequence


TOOL_DIR = Path(__file__).resolve().parent


def build_commands(args) -> list[tuple[str, list[str]]]:
    """Return the missing/forced pipeline stages as ``(name, argv)`` pairs."""
    out_dir = Path(args.out)
    posed_dir = out_dir / 'posed_images'
    transforms = posed_dir / 'transforms.json'
    colored_map = out_dir / 'colored_map.ply'
    commands = []

    rebuild_images = args.force_images or not transforms.is_file()
    if rebuild_images:
        extract = [
            sys.executable, str(TOOL_DIR / 'extract_posed_images.py'),
            '--bag', str(args.bag), '--traj', str(args.traj),
            '--camera-topic', args.camera_topic,
            '--camera-info-topic', args.camera_info_topic,
            '--extrinsic', str(args.extrinsic), '--out', str(posed_dir),
            '--time-offset', args.time_offset,
            '--clock-reference-topic', args.points_topic,
            '--stride', str(args.image_stride),
            '--start-time', str(args.start_time), '--end-time', str(args.end_time),
        ]
        if not args.no_undistort:
            extract.append('--undistort')
        commands.append(('posed images', extract))

    if rebuild_images or args.force_map or not colored_map.is_file():
        build = [
            sys.executable, str(TOOL_DIR / 'build_lidar_init.py'),
            '--bag', str(args.bag), '--traj', str(args.traj),
            '--points-topic', args.points_topic, '--out', str(colored_map),
            '--voxel', str(args.voxel), '--max-points', str(args.max_points),
            '--min-range', str(args.min_range), '--max-range', str(args.max_range),
            '--stride', str(args.scan_stride),
            '--start-time', str(args.start_time), '--end-time', str(args.end_time),
            '--color-transforms', str(transforms), '--color-robust',
        ]
        commands.append(('coloured map', build))
    return commands


def run_pipeline(args) -> dict:
    """Execute missing stages and return paths plus the stages that ran."""
    out_dir = Path(args.out)
    if not args.dry_run:
        out_dir.mkdir(parents=True, exist_ok=True)
    commands = build_commands(args)
    for name, command in commands:
        print(f'[{name}]', ' '.join(command))
        if not args.dry_run:
            subprocess.run(command, check=True)
    return {
        'stages': [name for name, _ in commands],
        'transforms': out_dir / 'posed_images' / 'transforms.json',
        'colored_map': out_dir / 'colored_map.ply',
    }


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('bag', type=Path, help='rosbag2 directory')
    p.add_argument('traj', type=Path, help='SLAM trajectory (TUM, world<-body)')
    p.add_argument('out', type=Path, help='output directory')
    p.add_argument('--extrinsic', type=Path, required=True,
                   help='body<-camera YAML/JSON or vlcal calib.json')
    p.add_argument('--points-topic', default='/livox/points')
    p.add_argument('--camera-topic', default='/image')
    p.add_argument('--camera-info-topic', default='/camera_info')
    p.add_argument('--time-offset', default='auto',
                   help='camera-to-trajectory clock offset or auto')
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
    p.add_argument('--dry-run', action='store_true')
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    summary = run_pipeline(args)
    if summary['stages']:
        print('completed stages:', ', '.join(summary['stages']))
    else:
        print('everything is up to date; nothing to do')
    print('coloured map:', summary['colored_map'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
