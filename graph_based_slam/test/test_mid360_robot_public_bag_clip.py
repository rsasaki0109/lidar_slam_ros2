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

"""Tests for public MID-360 bag segment clipping."""

from __future__ import annotations

import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_bag_clip import (  # noqa: E402
    PUBLIC_BAG_SEGMENT_CLIP_JSON,
    PUBLIC_BAG_SEGMENT_CLIP_MARKDOWN,
    PublicBagSegmentClipOptions,
    PublicBagSegmentClipper,
)
from mid360_robot_sample_bag import Mid360SampleBagWriter, SampleBagConfig  # noqa: E402

import yaml  # noqa: E402


def _segments_report(source_bag: Path) -> dict:
    return {
        'datasets': [
            {
                'dataset_id': 'sample_public_mid360',
                'selected_bag_path': str(source_bag),
                'selected_topics': {'pointcloud': '/livox/lidar', 'imu': '/livox/imu'},
                'recommended_segment': {
                    'segment_id': 'segment_000',
                    'clip_start_time_ns': 1_000_000_000,
                    'clip_end_time_ns': 2_000_000_000,
                    'duration_sec': 1.0,
                },
                'segments': [],
            }
        ]
    }


def test_clipper_writes_segment_bag_and_sidecars(tmp_path: Path):
    source_bag = tmp_path / 'source_bag'
    Mid360SampleBagWriter(
        SampleBagConfig(
            output_path=source_bag,
            duration_sec=3.0,
            pointcloud_rate_hz=10.0,
            imu_rate_hz=100.0,
            force=True,
        )
    ).write()
    segments_path = tmp_path / 'segments.json'
    segments_path.write_text(json.dumps(_segments_report(source_bag)), encoding='utf-8')

    summary = PublicBagSegmentClipper(segments_path).clip(
        PublicBagSegmentClipOptions(
            dataset_id='sample_public_mid360',
            output_root=tmp_path / 'clips',
            force=True,
        )
    )

    output_dir = tmp_path / 'clips' / 'sample_public_mid360' / 'segment_000'
    output_bag = output_dir / 'rosbag2'
    metadata = yaml.safe_load((output_bag / 'metadata.yaml').read_text(encoding='utf-8'))
    info = metadata['rosbag2_bagfile_information']

    assert summary['status'] == 'PASS'
    assert summary['margin_sec'] == 0.0
    assert summary['copy']['message_counts']['/livox/lidar'] == 11
    assert summary['copy']['message_counts']['/livox/imu'] == 101
    assert info['message_count'] == 112
    assert (output_dir / PUBLIC_BAG_SEGMENT_CLIP_JSON).is_file()
    assert (output_dir / PUBLIC_BAG_SEGMENT_CLIP_MARKDOWN).is_file()
