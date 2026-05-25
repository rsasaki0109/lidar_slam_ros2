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

"""Tests for public MID-360 bag segment analysis."""

from __future__ import annotations

import json
from pathlib import Path
import sys
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_bag_segments import (  # noqa: E402
    pointcloud_scan_timing,
    process_rko_lio_timestamps,
    PUBLIC_BAG_SEGMENTS_JSON,
    PUBLIC_BAG_SEGMENTS_MARKDOWN,
    PublicBagSegmentOptions,
    PublicBagSegmentReportBuilder,
    render_public_bag_segments_markdown,
    split_contiguous_scan_segments,
)

import numpy as np  # noqa: E402


def _timing(index: int, receive_ns: int, processed_min: float, processed_max: float) -> dict:
    return {
        'scan_index': index,
        'receive_time_ns': receive_ns,
        'header_stamp_ns': receive_ns,
        'point_count': 10,
        'timestamp_field': 't',
        'timestamp_datatype': 6,
        'raw_time_min': 0,
        'raw_time_max': 100_000_000,
        'timestamp_mode': 'relative',
        'timestamp_multiplier': 1e-9,
        'rko_timestamp_supported': True,
        'processed_min_sec': processed_min,
        'processed_max_sec': processed_max,
    }


class FakeTimingReader:
    def __init__(self, timings: list[dict]) -> None:
        self.timings = timings
        self.calls: list[tuple[Path, str, int]] = []

    def read_scan_timings(
        self,
        bag_path: Path,
        pointcloud_topic: str,
        max_scans: int = 0,
        **kwargs,
    ) -> list[dict]:
        del kwargs
        self.calls.append((bag_path, pointcloud_topic, max_scans))
        return self.timings[:max_scans] if max_scans > 0 else self.timings


def _manifest(root: Path) -> dict:
    return {
        'candidates': [
            {
                'dataset_id': 'hard_pointcloud_mid360_outdoor_kidnap_a',
                'title': 'Hard',
                'selected_bag_path': str(root / 'bag'),
                'selected_topics': {'pointcloud': '/livox/points', 'imu': '/livox/imu'},
            }
        ]
    }


def test_process_rko_lio_timestamps_detects_relative_nanosecond_scan_times():
    result = process_rko_lio_timestamps(
        raw_min=0,
        raw_max=100_000_000,
        header_stamp_ns=1_000_000_000,
    )

    assert result['timestamp_mode'] == 'relative'
    assert result['timestamp_multiplier'] == 1e-9
    assert result['processed_min_sec'] == 1.0
    assert result['processed_max_sec'] == 1.1


def test_pointcloud_scan_timing_reads_t_field_min_max():
    data = np.zeros(3 * 16, dtype=np.uint8)
    points = np.ndarray(
        shape=(3, 4),
        dtype=np.dtype('<f4'),
        buffer=data,
    )
    points[:, 0] = [2.0, 3.0, 4.0]
    points[:, 1] = [0.0, 0.0, 0.0]
    points[:, 2] = [0.0, 0.0, 0.0]
    values = np.ndarray(
        shape=(3,),
        dtype=np.dtype('<u4'),
        buffer=data,
        offset=12,
        strides=(16,),
    )
    values[:] = [0, 50_000_000, 100_000_000]
    msg = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=10, nanosec=0),
            frame_id='livox_frame',
        ),
        height=1,
        width=3,
        fields=[
            SimpleNamespace(name='x', offset=0, datatype=7, count=1),
            SimpleNamespace(name='y', offset=4, datatype=7, count=1),
            SimpleNamespace(name='z', offset=8, datatype=7, count=1),
            SimpleNamespace(name='t', offset=12, datatype=6, count=1),
        ],
        is_bigendian=False,
        point_step=16,
        row_step=48,
        data=data,
    )

    timing = pointcloud_scan_timing(0, 10_100_000_000, msg)

    assert timing['raw_time_min'] == 0.0
    assert timing['raw_time_max'] == 100_000_000.0
    assert timing['timestamp_mode'] == 'relative'
    assert timing['processed_max_sec'] == 10.1
    assert timing['clipped_point_count'] == 3
    assert timing['keypoint_count'] == 2


def test_split_segments_breaks_on_rko_lio_scan_delta():
    timings = [
        _timing(0, 1_000_000_000, 1.0, 1.1),
        _timing(1, 1_100_000_000, 1.1, 1.2),
        _timing(2, 1_200_000_000, 1.2, 1.3),
        _timing(3, 3_000_000_000, 3.0, 3.1),
    ]

    split = split_contiguous_scan_segments(
        timings,
        PublicBagSegmentOptions(max_scan_gap_sec=1.0, min_segment_duration_sec=0.2),
    )

    assert len(split['segments']) == 2
    assert split['gaps'][0]['previous_scan_index'] == 2
    assert split['gaps'][0]['current_scan_index'] == 3
    assert split['segments'][0]['ready_for_clip'] is True
    assert split['segments'][0]['clip_end_offset_sec'] == 0.2


def test_report_builder_writes_recommended_segment(tmp_path: Path):
    timings = [
        _timing(0, 1_000_000_000, 1.0, 1.1),
        _timing(1, 1_100_000_000, 1.1, 1.2),
        _timing(2, 1_200_000_000, 1.2, 1.3),
        _timing(3, 3_000_000_000, 3.0, 3.1),
    ]
    manifest_path = tmp_path / 'manifest.json'
    manifest_path.write_text(json.dumps(_manifest(tmp_path)), encoding='utf-8')
    output_dir = tmp_path / 'out'

    builder = PublicBagSegmentReportBuilder(
        manifest_path=manifest_path,
        output_dir=output_dir,
        timing_reader=FakeTimingReader(timings),
    )
    report = builder.build(
        options=PublicBagSegmentOptions(max_scan_gap_sec=1.0, min_segment_duration_sec=0.2)
    )
    paths = builder.write(report)
    markdown = render_public_bag_segments_markdown(report)

    row = report['datasets'][0]
    assert report['status'] == 'PASS'
    assert row['recommended_segment']['segment_id'] == 'segment_000'
    assert row['recommended_segment']['scan_count'] == 3
    assert 'hard_pointcloud_mid360_outdoor_kidnap_a' in markdown
    assert paths['json'] == output_dir / PUBLIC_BAG_SEGMENTS_JSON
    assert paths['markdown'] == output_dir / PUBLIC_BAG_SEGMENTS_MARKDOWN
    assert json.loads(paths['json'].read_text(encoding='utf-8'))['status'] == 'PASS'
