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

"""Tests for public MID-360 loop segment-reset planning."""

from __future__ import annotations

import json
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_bag_segments import PUBLIC_BAG_SEGMENTS_JSON  # noqa: E402
from mid360_robot_public_loop_segment_reset import (  # noqa: E402
    LoopSegmentResetOptions,
    LoopSegmentResetPlanner,
    PUBLIC_LOOP_SEGMENT_RESET_JSON,
    PUBLIC_LOOP_SEGMENT_RESET_MARKDOWN,
    render_loop_segment_reset_markdown,
)


def _timing(index: int, stamp: float, keypoints: int = 20) -> dict:
    receive_ns = int(stamp * 1_000_000_000)
    return {
        'scan_index': index,
        'receive_time_ns': receive_ns,
        'header_stamp_ns': receive_ns,
        'point_count': 100,
        'timestamp_field': 't',
        'timestamp_datatype': 6,
        'raw_time_min': 0.0,
        'raw_time_max': 0.1,
        'timestamp_mode': 'relative',
        'timestamp_multiplier': 1.0,
        'rko_timestamp_supported': True,
        'processed_min_sec': stamp,
        'processed_max_sec': stamp + 0.1,
        'clipped_point_count': 100,
        'keypoint_count': keypoints,
    }


class FakeTimingReader:
    def __init__(self, timings: list[dict]) -> None:
        self.timings = timings
        self.calls = []

    def read_scan_timings(
        self,
        bag_path: Path,
        pointcloud_topic: str,
        max_scans: int = 0,
        **kwargs,
    ):
        self.calls.append((bag_path, pointcloud_topic, max_scans, kwargs))
        return self.timings[:max_scans] if max_scans else self.timings


def _loop_candidates(path: Path, *, end_index: int = 6) -> Path:
    payload = {
        'sequences': [
            {
                'sequence_id': 'outdoor_kidnap',
                'dataset_id': 'hard_pointcloud_mid360_outdoor_kidnap_a',
                'trajectory_file': 'gt/traj_lidar_outdoor_kidnap.txt',
                'loop_candidates': [
                    {
                        'start_index': 1,
                        'start_stamp': 1.0,
                        'end_index': end_index,
                        'end_stamp': float(end_index),
                        'distance_m': 0.05,
                    }
                ],
            }
        ]
    }
    path.write_text(json.dumps(payload), encoding='utf-8')
    return path


def test_segment_reset_plan_maps_loop_endpoints_to_segments(tmp_path: Path):
    timings = [
        _timing(0, 0.0),
        _timing(1, 1.0),
        _timing(2, 2.0),
        _timing(3, 3.0, keypoints=0),
        _timing(4, 4.0),
        _timing(5, 5.0),
        _timing(6, 6.0),
    ]
    loop_path = _loop_candidates(tmp_path / 'loop_candidates.json')
    output_dir = tmp_path / 'plan'

    report = LoopSegmentResetPlanner(FakeTimingReader(timings)).plan(
        LoopSegmentResetOptions(
            loop_candidates_json=loop_path,
            bag_path=tmp_path / 'bag',
            output_dir=output_dir,
            min_segment_duration_sec=0.1,
            max_scan_gap_sec=10.0,
            clip_output_root=tmp_path / 'clips',
            rko_output_root=tmp_path / 'rko',
        )
    )

    assert report['status'] == 'PASS'
    assert report['segment_count'] == 2
    assert report['reset_pair']['start']['segment']['segment_id'] == 'segment_000'
    assert report['reset_pair']['end']['segment']['segment_id'] == 'segment_001'
    assert '--segment segment_000' in report['commands']['clip_start']
    assert '--segment segment_001' in report['commands']['clip_end']
    assert 'run_rko_lio_graph_autoware_dogfood.sh' in report['commands']['run_end_rko']
    assert (output_dir / PUBLIC_LOOP_SEGMENT_RESET_JSON).is_file()
    assert (output_dir / PUBLIC_LOOP_SEGMENT_RESET_MARKDOWN).is_file()
    assert (output_dir / PUBLIC_BAG_SEGMENTS_JSON).is_file()
    markdown = render_loop_segment_reset_markdown(report)
    assert 'segment_000' in markdown
    assert 'segment_001' in markdown


def test_segment_reset_plan_warns_when_endpoint_has_no_ready_segment(tmp_path: Path):
    timings = [
        _timing(0, 0.0),
        _timing(1, 1.0),
        _timing(2, 2.0),
        _timing(3, 3.0, keypoints=0),
    ]
    loop_path = _loop_candidates(tmp_path / 'loop_candidates.json', end_index=3)

    report = LoopSegmentResetPlanner(FakeTimingReader(timings)).plan(
        LoopSegmentResetOptions(
            loop_candidates_json=loop_path,
            bag_path=tmp_path / 'bag',
            output_dir=tmp_path / 'plan',
            min_segment_duration_sec=0.1,
            max_scan_gap_sec=10.0,
        )
    )

    assert report['status'] == 'WARN'
    assert report['reset_pair']['start']['status'] == 'PASS'
    assert report['reset_pair']['end']['status'] == 'FAIL'
