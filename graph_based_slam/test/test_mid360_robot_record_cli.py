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

"""CLI tests for MID-360 robot recording helper."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'plan_mid360_robot_record.py'
WRAPPER_PATH = REPO_ROOT / 'scripts' / 'record_mid360_robot_bag.sh'


def _write_profile(tmp_path: Path) -> Path:
    profile_path = tmp_path / 'profile.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'cli_record_robot',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
            'imu_frame': 'livox_frame',
            'expected_pointcloud_topic': '/livox/lidar',
            'expected_imu_topic': '/livox/imu',
        }),
        encoding='utf-8',
    )
    return profile_path


def test_record_cli_dry_run_writes_manifest_and_json(tmp_path: Path):
    profile_path = _write_profile(tmp_path)
    bag_root = tmp_path / 'bags'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--robot-profile',
            str(profile_path),
            '--bag-root',
            str(bag_root),
            '--run-id',
            'field01',
            '--duration-sec',
            '10',
            '--extra-topic',
            '/diagnostics',
            '--dry-run',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    payload = json.loads(result.stdout)

    assert payload['run_id'] == 'field01'
    assert payload['bag_path'] == str(bag_root.resolve() / 'field01')
    assert payload['topics'] == ['/livox/lidar', '/livox/imu', '/tf', '/tf_static', '/diagnostics']
    assert payload['command'][:2] == ['timeout', '10']
    assert (bag_root / 'field01_record_plan.json').is_file()
    assert (bag_root / 'field01_record_plan.md').is_file()
    assert (bag_root / 'field01_profile.yaml').is_file()


def test_record_shell_wrapper_dry_run_outputs_command(tmp_path: Path):
    profile_path = _write_profile(tmp_path)
    bag_root = tmp_path / 'bags'

    result = subprocess.run(
        [
            'bash',
            str(WRAPPER_PATH),
            '--robot-profile',
            str(profile_path),
            '--bag-root',
            str(bag_root),
            '--run-id',
            'field02',
            '--no-tf',
            '--dry-run',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    assert 'Record command:' in result.stdout
    assert 'ros2 bag record' in result.stdout
    assert '/tf_static' in result.stdout
    assert '/tf ' not in result.stdout
    assert (bag_root / 'field02_record_plan.json').is_file()
