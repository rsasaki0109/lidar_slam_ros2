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

"""Tests for the repo-local golden-path product CLI."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
CLI = REPO_ROOT / 'scripts' / 'lidarslam'
VERSION = (REPO_ROOT / 'VERSION').read_text(encoding='utf-8').strip()


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(CLI), *args],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def _write_bag_metadata(path: Path) -> None:
    path.mkdir()
    payload = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 2_000_000_000},
            'message_count': 220,
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': '/points',
                        'type': 'sensor_msgs/msg/PointCloud2',
                    },
                    'message_count': 20,
                },
                {
                    'topic_metadata': {
                        'name': '/imu',
                        'type': 'sensor_msgs/msg/Imu',
                    },
                    'message_count': 200,
                },
            ],
        },
    }
    (path / 'metadata.yaml').write_text(
        __import__('yaml').safe_dump(payload),
        encoding='utf-8',
    )


def test_global_help_version_and_usage_contract():
    help_result = _run('--help')
    assert help_result.returncode == 0
    assert 'doctor <rosbag2_dir>' in help_result.stdout
    assert 'run <rosbag2_dir>' in help_result.stdout
    assert 'inspect <output_dir>' in help_result.stdout

    version_result = _run('--version')
    assert version_result.returncode == 0
    assert version_result.stdout.strip() == f'lidarslam_ros2 {VERSION}'

    missing_result = _run()
    assert missing_result.returncode == 2
    assert 'Usage:' in missing_result.stderr

    unknown_result = _run('unknown')
    assert unknown_result.returncode == 2
    assert 'unknown command: unknown' in unknown_result.stderr


def test_doctor_emits_machine_readable_preflight(tmp_path: Path):
    bag = tmp_path / 'sample_bag'
    _write_bag_metadata(bag)

    result = _run('doctor', str(bag), '--json')

    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    assert payload['recommended_profile_id'] == 'rko_lio_graph_public_path'
    assert payload['summary']['capabilities']['has_pointcloud2'] is True
    assert payload['summary']['capabilities']['has_imu'] is True


def test_run_dry_run_and_inspect_delegate_to_proven_tools(tmp_path: Path):
    bag = tmp_path / 'sample_bag'
    output = tmp_path / 'map_output'
    _write_bag_metadata(bag)

    run_result = _run(
        'run',
        str(bag),
        '--output-dir',
        str(output),
        '--dry-run',
    )
    assert run_result.returncode == 0, run_result.stderr
    assert 'Selected profile:' in run_result.stdout
    assert str(output) in run_result.stdout
    assert not output.exists()

    output.mkdir()
    inspect_result = _run('inspect', str(output), '--json')
    assert inspect_result.returncode == 0, inspect_result.stderr
    diagnosis = json.loads(inspect_result.stdout)
    assert diagnosis['status'] == 'incomplete'
    assert diagnosis['run_dir'] == str(output)


def test_child_input_error_is_propagated(tmp_path: Path):
    result = _run('doctor', str(tmp_path / 'missing'))

    assert result.returncode == 2
    assert 'rosbag2 directory does not exist' in result.stderr
