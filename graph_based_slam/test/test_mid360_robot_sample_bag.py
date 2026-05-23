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

"""Tests for the MID-360 sample rosbag generator."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


pytestmark = pytest.mark.skipif(
    importlib.util.find_spec('rosbags') is None,
    reason='rosbags python module not available (pip install rosbags)',
)

REPO_ROOT = Path(__file__).resolve().parents[2]
SAMPLE_SCRIPT = REPO_ROOT / 'scripts' / 'generate_mid360_robot_sample_bag.py'
READINESS_SCRIPT = REPO_ROOT / 'scripts' / 'check_mid360_robot_readiness.py'
DEFAULT_PROFILE = REPO_ROOT / 'configs' / 'mid360_robot' / 'livox_mid360_default.yaml'


def test_sample_bag_generator_writes_readable_mid360_bag(tmp_path: Path):
    bag_path = tmp_path / 'mid360_sample'
    result = subprocess.run(
        [
            sys.executable,
            str(SAMPLE_SCRIPT),
            str(bag_path),
            '--duration-sec',
            '2.0',
            '--pointcloud-rate-hz',
            '10',
            '--imu-rate-hz',
            '100',
            '--point-count',
            '16',
            '--force',
            '--json',
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    summary = json.loads(result.stdout)

    assert bag_path.is_dir()
    assert (bag_path / 'metadata.yaml').is_file()
    assert summary['pointcloud_messages'] == 20
    assert summary['imu_messages'] == 200
    assert summary['topics']['pointcloud'] == '/livox/lidar'

    output_dir = tmp_path / 'readiness'
    readiness = subprocess.run(
        [
            sys.executable,
            str(READINESS_SCRIPT),
            str(bag_path),
            '--robot-profile',
            str(DEFAULT_PROFILE),
            '--output-dir',
            str(output_dir),
            '--json',
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    report = json.loads(readiness.stdout)

    assert report['status'] == 'PASS'
    assert report['ready_for_mid360_launch'] is True
    diagnostics = report['bag_diagnostics']
    assert diagnostics['sample_reader']['available'] is True
    assert diagnostics['topics']['pointcloud']['sampled_frame_ids'] == ['livox_frame']
    assert diagnostics['topics']['imu']['sampled_frame_ids'] == ['livox_frame']
    assert diagnostics['tf']['base_to_lidar_connected'] is True
    assert diagnostics['tf']['base_to_imu_connected'] is True


def test_sample_bag_generator_requires_force_for_existing_output(tmp_path: Path):
    bag_path = tmp_path / 'mid360_sample'
    first = [
        sys.executable,
        str(SAMPLE_SCRIPT),
        str(bag_path),
        '--duration-sec',
        '1.0',
    ]
    subprocess.run(first, check=True, text=True, capture_output=True)

    second = subprocess.run(first, text=True, capture_output=True)

    assert second.returncode == 1
    assert 'use --force' in second.stderr
