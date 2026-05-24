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

"""CLI tests for the MID-360 robot map planner."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'plan_mid360_robot_map.py'


def _write_metadata(tmp_path: Path) -> Path:
    bag_dir = tmp_path / 'mid360_robot_bag'
    bag_dir.mkdir()
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 5_000_000_000},
            'message_count': 550,
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': '/livox/lidar',
                        'type': 'sensor_msgs/msg/PointCloud2',
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': 50,
                },
                {
                    'topic_metadata': {
                        'name': '/livox/imu',
                        'type': 'sensor_msgs/msg/Imu',
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': 500,
                },
            ],
        },
    }
    (bag_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
    return bag_dir


def test_plan_cli_json_contains_commands_without_shell_wrapper(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    output_dir = tmp_path / 'map_out'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--base-frame',
            'trunk',
            '--foxglove',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    payload = json.loads(result.stdout)
    dogfood_command = payload['plan']['dogfood_command']

    assert payload['preflight']['ready_for_mid360_launch'] is True
    assert '--base-frame' in dogfood_command
    assert 'trunk' in dogfood_command
    assert '--skip-viewer' in dogfood_command
    assert payload['plan']['foxglove_command']


def test_plan_cli_uses_robot_profile_and_allows_frame_override(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    profile_path = tmp_path / 'profile.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'profile_robot',
            'base_frame': 'profile_base',
            'lidar_frame': 'profile_lidar',
            'imu_frame': 'profile_imu',
            'expected_pointcloud_topic': '/livox/lidar',
            'expected_imu_topic': '/livox/imu',
        }),
        encoding='utf-8',
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(bag_dir),
            '--robot-profile',
            str(profile_path),
            '--base-frame',
            'override_base',
            '--json',
            '--dry-run',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    payload = json.loads(result.stdout)
    dogfood_command = payload['plan']['dogfood_command']

    assert payload['preflight']['robot_profile']['robot_name'] == 'profile_robot'
    assert payload['preflight']['frames']['base_frame'] == 'override_base'
    assert payload['preflight']['frames']['lidar_frame'] == 'profile_lidar'
    assert 'override_base' in dogfood_command
    assert 'profile_lidar' in dogfood_command


def test_plan_cli_write_manifest_in_dry_run(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    output_dir = tmp_path / 'map_out'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--write-manifest',
            '--dry-run',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    json_path = output_dir / 'mid360_robot_run_plan.json'
    markdown_path = output_dir / 'mid360_robot_run_plan.md'
    manifest = json.loads(json_path.read_text(encoding='utf-8'))

    assert json_path.is_file()
    assert markdown_path.is_file()
    assert manifest['bag_path'] == str(bag_dir.resolve())
    assert manifest['selected_topics']['pointcloud'] == '/livox/lidar'
    assert 'Manifest JSON:' in result.stderr
    assert 'Run command:' in result.stdout


def test_plan_cli_json_stays_machine_readable_when_writing_manifest(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    output_dir = tmp_path / 'map_out'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--write-manifest',
            '--json',
            '--dry-run',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    payload = json.loads(result.stdout)

    assert payload['plan']['output_dir'] == str(output_dir.resolve())
    assert (output_dir / 'mid360_robot_run_plan.json').is_file()
    assert 'Manifest JSON:' in result.stderr


def test_plan_cli_write_diagnosis_is_planned_but_not_run_in_dry_run(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    output_dir = tmp_path / 'map_out'

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--write-manifest',
            '--write-diagnosis',
            '--dry-run',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    manifest = json.loads(
        (output_dir / 'mid360_robot_run_plan.json').read_text(encoding='utf-8')
    )

    assert manifest['diagnosis']['ran'] is False
    assert 'diagnose_autoware_map_run.py' in manifest['diagnosis']['command_shell']
    assert not (output_dir / 'autoware_map_diagnosis.md').exists()


def test_plan_cli_json_includes_diagnosis_plan(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    output_dir = tmp_path / 'map_out'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--write-diagnosis',
            '--json',
            '--dry-run',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    payload = json.loads(result.stdout)

    assert payload['diagnosis']['ran'] is False
    assert payload['diagnosis']['markdown_path'] == str(
        output_dir.resolve() / 'autoware_map_diagnosis.md'
    )
