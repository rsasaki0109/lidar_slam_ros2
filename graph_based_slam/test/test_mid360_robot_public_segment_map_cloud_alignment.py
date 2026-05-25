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

"""Tests for public MID-360 reset segment map cloud alignment."""

from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_segment_map_cloud_alignment import (  # noqa: E402
    estimate_rigid_alignment,
    PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_JSON,
    PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_MARKDOWN,
    PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_PLY,
    PublicSegmentMapCloudAlignmentAnalyzer,
    SegmentMapCloudAlignmentOptions,
    transform_points,
)

import numpy as np  # noqa: E402


def _asymmetric_points() -> np.ndarray:
    points = []
    for x_index in range(8):
        for y_index in range(5):
            x = float(x_index)
            y = float(y_index)
            if x_index >= 5 and y_index >= 3:
                continue
            z = 0.05 * x + 0.12 * y
            points.append((x, y, z))
    points.extend([(8.5, 0.25, 0.6), (2.25, 5.5, 0.4), (-0.75, 1.5, -0.1)])
    return np.asarray(points, dtype=np.float64)


def _write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
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
        'DATA ascii',
    ]
    lines.extend(f'{point[0]} {point[1]} {point[2]}' for point in points)
    path.write_text('\n'.join(lines) + '\n', encoding='ascii')


def test_estimate_rigid_alignment_recovers_known_transform():
    source = _asymmetric_points()
    yaw = 0.35
    rotation = np.array([
        [np.cos(yaw), -np.sin(yaw), 0.0],
        [np.sin(yaw), np.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    translation = np.array([2.0, -1.25, 0.4])
    target = transform_points(source, rotation, translation)

    result = estimate_rigid_alignment(
        source,
        target,
        yaw_samples=72,
        max_iterations=30,
        trim_fraction=1.0,
    )
    aligned = transform_points(
        source,
        np.asarray(result['rotation'], dtype=np.float64),
        np.asarray(result['translation'], dtype=np.float64),
    )

    assert result['status'] == 'PASS'
    assert np.median(np.linalg.norm(aligned - target, axis=1)) < 1e-6


def test_segment_map_cloud_alignment_analyzer_writes_pass_report(tmp_path: Path):
    source = _asymmetric_points()
    yaw = -0.25
    rotation = np.array([
        [np.cos(yaw), -np.sin(yaw), 0.0],
        [np.sin(yaw), np.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    target = transform_points(source, rotation, np.array([4.0, 1.5, -0.2]))
    start_run = tmp_path / 'segment_000'
    end_run = tmp_path / 'segment_012'
    _write_ascii_pcd(start_run / 'pointcloud_map' / 'start.pcd', source)
    _write_ascii_pcd(end_run / 'pointcloud_map' / 'end.pcd', target)

    report = PublicSegmentMapCloudAlignmentAnalyzer().analyze(
        SegmentMapCloudAlignmentOptions(
            start_run_dir=start_run,
            end_run_dir=end_run,
            output_dir=tmp_path / 'alignment',
            voxel_size_m=0.01,
            max_points_per_cloud=1000,
            icp_yaw_samples=72,
            icp_trim_fraction=1.0,
            pass_median_nn_m=0.05,
            pass_p90_nn_m=0.05,
            pass_coverage_within_1m=1.0,
            min_cloud_points=10,
        )
    )

    assert report['status'] == 'PASS'
    assert report['aligned_overlap']['symmetric_median_nn_m'] < 0.05
    assert (tmp_path / 'alignment' / PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_JSON).is_file()
    assert (tmp_path / 'alignment' / PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_MARKDOWN).is_file()
    assert (tmp_path / 'alignment' / PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_PLY).is_file()


def test_segment_map_cloud_alignment_can_crop_around_loop_stamps(tmp_path: Path):
    source = _asymmetric_points()
    distractor = source + np.array([60.0, -20.0, 0.0])
    yaw = 0.4
    rotation = np.array([
        [np.cos(yaw), -np.sin(yaw), 0.0],
        [np.sin(yaw), np.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    translation = np.array([3.0, -2.0, 0.0])
    target = transform_points(source, rotation, translation)
    start_run = tmp_path / 'segment_000'
    end_run = tmp_path / 'segment_012'
    _write_ascii_pcd(start_run / 'pointcloud_map' / 'start.pcd', np.vstack([source, distractor]))
    _write_ascii_pcd(
        end_run / 'pointcloud_map' / 'end.pcd',
        np.vstack([target, distractor + np.array([-20.0, 70.0, 0.0])]),
    )
    (start_run / 'traj.tum').write_text('10.0 2.0 2.0 0.2 0 0 0 1\n', encoding='ascii')
    center = transform_points(np.asarray([[2.0, 2.0, 0.2]]), rotation, translation)[0]
    (end_run / 'traj.tum').write_text(
        f'20.0 {center[0]} {center[1]} {center[2]} 0 0 0 1\n',
        encoding='ascii',
    )

    report = PublicSegmentMapCloudAlignmentAnalyzer().analyze(
        SegmentMapCloudAlignmentOptions(
            start_run_dir=start_run,
            end_run_dir=end_run,
            output_dir=tmp_path / 'alignment_crop',
            start_center_stamp=10.0,
            end_center_stamp=20.0,
            crop_radius_m=6.0,
            voxel_size_m=0.01,
            icp_yaw_samples=72,
            icp_trim_fraction=1.0,
            pass_median_nn_m=0.05,
            pass_p90_nn_m=0.05,
            pass_coverage_within_1m=1.0,
            min_cloud_points=10,
        )
    )

    assert report['status'] == 'PASS'
    assert report['crop']['start_cropped_points'] < report['crop']['start_raw_points']
    assert report['crop']['end_cropped_points'] < report['crop']['end_raw_points']
