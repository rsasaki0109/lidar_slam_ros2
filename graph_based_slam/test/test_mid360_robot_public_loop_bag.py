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

"""Tests for multi-bag MID-360 public loop bag builder."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
CLI_PATH = SCRIPT_DIR / 'build_mid360_robot_public_loop_bag.py'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_loop_bag import (  # noqa: E402
    PUBLIC_LOOP_BAG_JSON,
    PUBLIC_LOOP_BAG_MARKDOWN,
    PublicLoopBagBuilder,
    PublicLoopBagOptions,
)
from mid360_robot_sample_bag import Mid360SampleBagWriter, SampleBagConfig  # noqa: E402

import pytest  # noqa: E402
import yaml  # noqa: E402


def _write_bag(path: Path, *, start_offset_sec: float, duration_sec: float) -> None:
    Mid360SampleBagWriter(
        SampleBagConfig(
            output_path=path,
            duration_sec=duration_sec,
            pointcloud_rate_hz=10.0,
            imu_rate_hz=100.0,
            force=True,
            start_offset_sec=start_offset_sec,
        )
    ).write()


def test_builder_concatenates_two_bags_preserving_timestamps(tmp_path: Path):
    bag_a = tmp_path / 'bag_a'
    bag_b = tmp_path / 'bag_b'
    _write_bag(bag_a, start_offset_sec=0.0, duration_sec=2.0)
    _write_bag(bag_b, start_offset_sec=2.0, duration_sec=2.0)

    output_bag = tmp_path / 'loop_bag'
    summary = PublicLoopBagBuilder().build(
        PublicLoopBagOptions(
            input_bags=(bag_a, bag_b),
            output_bag=output_bag,
            topics=('/livox/lidar', '/livox/imu'),
            include_tf=True,
            force=True,
        )
    )

    assert summary['status'] == 'PASS'
    assert summary['copy']['message_counts']['/livox/lidar'] == 40
    assert summary['copy']['message_counts']['/livox/imu'] == 400
    assert summary['copy']['per_bag_total'] == [221, 221]
    assert summary['copy']['first_time_ns'] == 0
    expected_last_ns = int(round(3.99 * 1_000_000_000))
    assert summary['copy']['last_time_ns'] == expected_last_ns

    metadata = yaml.safe_load((output_bag / 'metadata.yaml').read_text(encoding='utf-8'))
    info = metadata['rosbag2_bagfile_information']
    assert info['message_count'] == 442  # 40 lidar + 400 imu + 2 tf_static
    assert (output_bag.parent / PUBLIC_LOOP_BAG_JSON).is_file()
    assert (output_bag.parent / PUBLIC_LOOP_BAG_MARKDOWN).is_file()


def test_builder_time_window_filters_messages(tmp_path: Path):
    bag_a = tmp_path / 'bag_a'
    bag_b = tmp_path / 'bag_b'
    _write_bag(bag_a, start_offset_sec=0.0, duration_sec=2.0)
    _write_bag(bag_b, start_offset_sec=2.0, duration_sec=2.0)

    output_bag = tmp_path / 'loop_bag'
    summary = PublicLoopBagBuilder().build(
        PublicLoopBagOptions(
            input_bags=(bag_a, bag_b),
            output_bag=output_bag,
            topics=('/livox/lidar', '/livox/imu'),
            include_tf=False,
            time_window_sec=(1.0, 3.0),
            force=True,
        )
    )

    assert summary['status'] == 'PASS'
    assert summary['copy']['message_counts']['/livox/lidar'] == 21
    assert summary['copy']['message_counts']['/livox/imu'] == 201


def test_builder_rejects_overlapping_bags(tmp_path: Path):
    bag_a = tmp_path / 'bag_a'
    bag_b = tmp_path / 'bag_b'
    _write_bag(bag_a, start_offset_sec=0.0, duration_sec=2.0)
    _write_bag(bag_b, start_offset_sec=1.0, duration_sec=2.0)

    with pytest.raises(ValueError, match='overlap'):
        PublicLoopBagBuilder().build(
            PublicLoopBagOptions(
                input_bags=(bag_a, bag_b),
                output_bag=tmp_path / 'loop_bag',
                topics=('/livox/lidar', '/livox/imu'),
                force=True,
            )
        )


def test_cli_writes_summary_and_bag(tmp_path: Path):
    bag_a = tmp_path / 'bag_a'
    bag_b = tmp_path / 'bag_b'
    _write_bag(bag_a, start_offset_sec=0.0, duration_sec=1.0)
    _write_bag(bag_b, start_offset_sec=1.0, duration_sec=1.0)

    output_bag = tmp_path / 'loop_bag'
    result = subprocess.run(
        [
            sys.executable,
            str(CLI_PATH),
            '--input-bag', str(bag_a),
            '--input-bag', str(bag_b),
            '--output-bag', str(output_bag),
            '--topic', '/livox/lidar',
            '--topic', '/livox/imu',
            '--no-tf',
            '--force',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    summary = json.loads(result.stdout)
    assert summary['status'] == 'PASS'
    assert summary['copy']['per_bag_total'] == [110, 110]
    assert (output_bag / 'metadata.yaml').is_file()
