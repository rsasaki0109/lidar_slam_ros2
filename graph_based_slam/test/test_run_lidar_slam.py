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

"""Tests for the user-facing LiDAR SLAM launcher."""

from __future__ import annotations

from pathlib import Path
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'scripts'))

from run_lidar_slam import (  # noqa: E402
    _clock,
    _latest_map_points,
    _progress_line,
    default_output_dir,
    discover_bags,
    main,
    read_bag_summary,
    resolve_bag,
)


def _bag(path: Path, topic: str = '/hesai/pandar') -> Path:
    path.mkdir(parents=True)
    payload = {'rosbag2_bagfile_information': {
        'duration': {'nanoseconds': 12_500_000_000},
        'message_count': 42,
        'topics_with_message_count': [
            {'topic_metadata': {'name': topic}, 'message_count': 42},
        ],
    }}
    (path / 'metadata.yaml').write_text(yaml.safe_dump(payload), encoding='utf-8')
    return path


def test_reads_summary_and_detects_supported_hilti_topic(tmp_path: Path):
    bag = _bag(tmp_path / 'exp01_ros2')
    summary = read_bag_summary(bag)
    assert summary['duration_sec'] == 12.5
    assert summary['message_count'] == 42
    assert summary['supported'] is True


def test_discovers_and_resolves_sequence_short_name(tmp_path: Path):
    bag = _bag(tmp_path / 'datasets' / 'exp01_ros2')
    candidates = discover_bags([tmp_path])
    assert candidates[0][0] == bag.resolve()
    assert resolve_bag('exp01', candidates) == bag.resolve()


def test_default_output_stays_on_media_drive():
    bag = Path('/media/sasaki/aiueo/datasets/hilti2022/exp01_ros2')
    assert default_output_dir(bag) == Path('/media/sasaki/aiueo/lidarslam_work/output/maps')


def test_no_argument_lists_bags_and_shows_example(tmp_path: Path, capsys):
    _bag(tmp_path / 'exp01_ros2')
    _bag(tmp_path / 'other_ros2', '/livox/lidar')
    assert main(['--scan-root', str(tmp_path)]) == 0
    output = capsys.readouterr().out
    assert 'exp01_ros2' in output
    assert 'other_ros2' not in output
    assert '1件は非表示' in output
    assert '実行可能' in output
    assert './scripts/run_lidar_slam.py exp01' in output


def test_dry_run_does_not_create_output(tmp_path: Path, capsys):
    bag = _bag(tmp_path / 'exp01_ros2')
    out = tmp_path / 'output'
    assert main([str(bag), '-o', str(out), '--dry-run']) == 0
    assert not out.exists()
    assert 'dry-runのため実行していません' in capsys.readouterr().out


def test_unsupported_sensor_explains_current_limit(tmp_path: Path, capsys):
    bag = _bag(tmp_path / 'other_ros2', '/livox/lidar')
    assert main([str(bag), '--dry-run']) == 2
    assert '/hesai/pandar' in capsys.readouterr().err


def test_progress_helpers_report_eta_and_latest_map_points(tmp_path: Path):
    log = tmp_path / 'slam.log'
    log.write_text(
        'number of points in the map : 1200\n'
        'other line\nnumber of points in the map : 345678\n', encoding='utf-8')
    assert _latest_map_points(log) == 345678
    assert _clock(65) == '01:05'
    line = _progress_line(25, 100, 345678)
    assert '25.0%' in line
    assert '01:15' in line
    assert '345,678点' in line
