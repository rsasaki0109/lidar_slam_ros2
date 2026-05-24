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

"""Tests for public MID-360 loop cloud-overlap analysis."""

from __future__ import annotations

import json
from pathlib import Path
import sys
from zipfile import ZipFile

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_loop_cloud_analyzer import (  # noqa: E402
    cloud_overlap_metrics,
    LoopCloudAnalysisOptions,
    PUBLIC_LOOP_CLOUD_ANALYSIS_JSON,
    PUBLIC_LOOP_CLOUD_ANALYSIS_MARKDOWN,
    PublicLoopCloudAnalyzer,
    ScanPoints,
    voxel_downsample,
)

import numpy as np  # noqa: E402


class FakeWindowReader:
    """Return deterministic start/end loop clouds for unit tests."""

    def __init__(self, base_points: np.ndarray) -> None:
        self.base_points = base_points

    def read_window(
        self,
        bag_path: Path,
        topic: str,
        start_stamp: float,
        end_stamp: float,
        *,
        max_scans: int,
        max_points_per_scan: int,
        min_range_m: float,
        max_range_m: float,
    ) -> list[ScanPoints]:
        del bag_path, topic, end_stamp, max_scans, max_points_per_scan
        del min_range_m, max_range_m
        if start_stamp < 15.0:
            return [
                ScanPoints(
                    stamp=10.0,
                    receive_time_ns=10_000_000_000,
                    points=self.base_points,
                )
            ]
        shifted = self.base_points + np.array([0.05, 0.0, 0.0])
        return [ScanPoints(stamp=20.0, receive_time_ns=20_000_000_000, points=shifted)]


def _candidate_payload() -> dict:
    return {
        'sequences': [
            {
                'sequence_id': 'outdoor_kidnap',
                'trajectory_file': 'gt/traj_lidar_outdoor_kidnap.txt',
                'loop_candidates': [
                    {
                        'start_stamp': 10.0,
                        'end_stamp': 20.0,
                        'distance_m': 0.05,
                        'start_index': 0,
                        'end_index': 1,
                        'midpoint': [0.0, 0.0, 0.0],
                    }
                ],
            }
        ]
    }


def _write_gt_zip(path: Path) -> None:
    with ZipFile(path, 'w') as archive:
        archive.writestr(
            'gt/traj_lidar_outdoor_kidnap.txt',
            '\n'.join([
                '10.0 0 0 0 0 0 0 1',
                '20.0 0 0 0 0 0 0 1',
            ]),
        )


def test_cloud_overlap_metrics_detects_close_clouds():
    start = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    end = start + np.array([0.1, 0.0, 0.0])

    metrics = cloud_overlap_metrics(start, end)

    assert metrics['symmetric_median_nn_m'] < 0.11
    assert metrics['coverage_within_1m'] == 1.0


def test_voxel_downsample_limits_points():
    points = np.array([[0.01 * index, 0.0, 0.0] for index in range(100)], dtype=np.float64)

    downsampled = voxel_downsample(points, voxel_size_m=0.1, max_points=5)

    assert len(downsampled) == 5


def test_analyzer_writes_pass_report(tmp_path: Path):
    candidates_path = tmp_path / 'loop_candidates.json'
    candidates_path.write_text(json.dumps(_candidate_payload()), encoding='utf-8')
    gt_zip = tmp_path / 'gt.zip'
    _write_gt_zip(gt_zip)
    base_points = np.array(
        [[float(x), float(y), 0.0] for x in range(4) for y in range(4)],
        dtype=np.float64,
    )

    report = PublicLoopCloudAnalyzer(FakeWindowReader(base_points)).analyze(
        LoopCloudAnalysisOptions(
            loop_candidates_json=candidates_path,
            gt_zip=gt_zip,
            bag_path=tmp_path / 'fake_bag',
            output_dir=tmp_path / 'analysis',
            pass_median_nn_m=0.2,
            pass_coverage_within_1m=0.9,
        )
    )

    assert report['status'] == 'PASS'
    assert report['windows']['start']['scan_count'] == 1
    assert report['windows']['end']['scan_count'] == 1
    assert report['overlap']['symmetric_median_nn_m'] < 0.1
    assert (tmp_path / 'analysis' / PUBLIC_LOOP_CLOUD_ANALYSIS_JSON).is_file()
    assert (tmp_path / 'analysis' / PUBLIC_LOOP_CLOUD_ANALYSIS_MARKDOWN).is_file()
