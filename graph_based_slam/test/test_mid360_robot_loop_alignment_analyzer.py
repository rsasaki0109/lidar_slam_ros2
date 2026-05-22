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

"""Tests for MID-360 loop-alignment cloud analysis."""

from __future__ import annotations

import json
from pathlib import Path
import struct
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'analyze_mid360_robot_loop_alignment.py'


def _write_binary_xyz_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = '\n'.join([
        '# .PCD v0.7 - Point Cloud Data file format',
        'VERSION 0.7',
        'FIELDS x y z',
        'SIZE 4 4 4',
        'TYPE F F F',
        'COUNT 1 1 1',
        f'WIDTH {len(points)}',
        'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {len(points)}',
        'DATA binary',
        '',
    ])
    payload = b''.join(struct.pack('<fff', *point) for point in points)
    path.write_bytes(header.encode('ascii') + payload)


def _lzf_literal_payload(data: bytes) -> bytes:
    payload = bytearray()
    for offset in range(0, len(data), 32):
        chunk = data[offset:offset + 32]
        payload.append(len(chunk) - 1)
        payload.extend(chunk)
    return bytes(payload)


def _write_binary_compressed_xyz_pcd(
    path: Path,
    points: list[tuple[float, float, float]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = '\n'.join([
        '# .PCD v0.7 - Point Cloud Data file format',
        'VERSION 0.7',
        'FIELDS x y z',
        'SIZE 4 4 4',
        'TYPE F F F',
        'COUNT 1 1 1',
        f'WIDTH {len(points)}',
        'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {len(points)}',
        'DATA binary_compressed',
        '',
    ])
    uncompressed = b''.join(
        struct.pack('<f', point[axis])
        for axis in range(3)
        for point in points
    )
    compressed = _lzf_literal_payload(uncompressed)
    sizes = struct.pack('<II', len(compressed), len(uncompressed))
    path.write_bytes(header.encode('ascii') + sizes + compressed)


def _write_map(run_dir: Path, points: list[tuple[float, float, float]]) -> None:
    map_dir = run_dir / 'pointcloud_map'
    map_dir.mkdir(parents=True, exist_ok=True)
    (map_dir / 'pointcloud_map_metadata.yaml').write_text(
        yaml.safe_dump({
            'x_resolution': 20,
            'y_resolution': 20,
            '0_0.pcd': [0, 0],
        }),
        encoding='utf-8',
    )
    _write_binary_xyz_pcd(map_dir / '0_0.pcd', points)


def _write_compressed_map(run_dir: Path, points: list[tuple[float, float, float]]) -> None:
    map_dir = run_dir / 'pointcloud_map'
    map_dir.mkdir(parents=True, exist_ok=True)
    (map_dir / 'pointcloud_map_metadata.yaml').write_text(
        yaml.safe_dump({
            'x_resolution': 20,
            'y_resolution': 20,
            '0_0.pcd': [0, 0],
        }),
        encoding='utf-8',
    )
    _write_binary_compressed_xyz_pcd(map_dir / '0_0.pcd', points)


def _connected_cloud_points() -> list[tuple[float, float, float]]:
    points = []
    for xi in range(-12, 13):
        for yi in range(-12, 13):
            points.append((xi * 0.2, yi * 0.2, 0.0))
    return points


def _split_cloud_points() -> list[tuple[float, float, float]]:
    points = []
    for offset in (-3.0, 3.0):
        for xi in range(-5, 6):
            for yi in range(-5, 6):
                points.append((offset + xi * 0.15, yi * 0.15, 0.0))
    return points


def _write_loop_trajectory(path: Path, *, end_offset_y: float = 0.2) -> None:
    lines = []
    for index in range(60):
        lines.append(f'{index} {index * 0.1:.3f} 0.0 0.0 0 0 0 1')
    for index in range(60, 120):
        x = (119 - index) * 0.1
        lines.append(f'{index} {x:.3f} {end_offset_y:.3f} 0.0 0 0 0 1')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def test_loop_alignment_cli_passes_connected_loop_cloud(tmp_path: Path):
    run_dir = tmp_path / 'run'
    trajectory = run_dir / 'trajectory_tum.txt'
    _write_map(run_dir, _connected_cloud_points())
    _write_loop_trajectory(trajectory, end_offset_y=0.2)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(run_dir),
            '--trajectory',
            str(trajectory),
            '--max-loop-distance-m',
            '0.5',
            '--cloud-radius-m',
            '4.0',
            '--write',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['status'] == 'PASS'
    assert report['loop_candidates']
    assert report['counts']['fail'] == 0
    assert (run_dir / 'mid360_robot_loop_alignment.json').is_file()
    assert (run_dir / 'mid360_robot_loop_alignment.md').is_file()


def test_loop_alignment_cli_reads_binary_compressed_cloud(tmp_path: Path):
    run_dir = tmp_path / 'run'
    trajectory = run_dir / 'trajectory_tum.txt'
    _write_compressed_map(run_dir, _connected_cloud_points())
    _write_loop_trajectory(trajectory, end_offset_y=0.2)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(run_dir),
            '--trajectory',
            str(trajectory),
            '--max-loop-distance-m',
            '0.5',
            '--cloud-radius-m',
            '4.0',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['status'] == 'PASS'
    assert report['cloud']['tiles'][0]['data'] == 'binary_compressed'
    assert report['cloud']['sampled_points'] == len(_connected_cloud_points())


def test_loop_alignment_cli_fails_on_trajectory_loop_distance(tmp_path: Path):
    run_dir = tmp_path / 'run'
    trajectory = run_dir / 'trajectory_tum.txt'
    _write_map(run_dir, _connected_cloud_points())
    _write_loop_trajectory(trajectory, end_offset_y=1.5)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(run_dir),
            '--trajectory',
            str(trajectory),
            '--loop-search-radius-m',
            '2.0',
            '--max-loop-distance-m',
            '0.5',
            '--cloud-radius-m',
            '4.0',
            '--json',
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert any(check['id'] == 'loop_distance_gate' and check['status'] == 'FAIL'
               for check in report['checks'])


def test_loop_alignment_cli_fails_on_split_loop_cloud(tmp_path: Path):
    run_dir = tmp_path / 'run'
    trajectory = run_dir / 'trajectory_tum.txt'
    _write_map(run_dir, _split_cloud_points())
    _write_loop_trajectory(trajectory, end_offset_y=0.2)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(run_dir),
            '--trajectory',
            str(trajectory),
            '--max-loop-distance-m',
            '0.5',
            '--cloud-radius-m',
            '5.0',
            '--max-connected-components',
            '1',
            '--json',
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert any(check['id'].startswith('local_cloud_connected_')
               and check['status'] == 'FAIL' for check in report['checks'])
