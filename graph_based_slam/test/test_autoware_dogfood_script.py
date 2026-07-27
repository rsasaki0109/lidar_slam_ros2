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

"""CLI regression tests for the RKO-LIO Autoware dogfood wrapper."""

from __future__ import annotations

from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
DOGFOOD_SCRIPT = REPO_ROOT / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh'


def _run_dogfood(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['bash', str(DOGFOOD_SCRIPT), *args],
        check=False,
        capture_output=True,
        text=True,
    )


def _write_minimal_bag(tmp_path: Path) -> Path:
    bag_dir = tmp_path / 'demo_bag'
    bag_dir.mkdir()
    (bag_dir / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: {}\n',
        encoding='utf-8',
    )
    return bag_dir


def test_dogfood_help_exits_successfully():
    result = _run_dogfood('--help')

    assert result.returncode == 0
    assert 'run_rko_lio_graph_autoware_dogfood.sh' in result.stderr
    assert '--skip-viewer' in result.stderr
    assert '--capture-raw-odometry' in result.stderr
    assert '--raw-odometry-topic' in result.stderr


def test_dogfood_rejects_missing_option_value_before_realpath():
    result = _run_dogfood('--bag', '--skip-viewer')

    assert result.returncode == 2
    assert 'error: option requires a value: --bag' in result.stderr
    assert 'realpath' not in result.stderr


def test_dogfood_rejects_invalid_bool_without_usage_dump():
    result = _run_dogfood('--capture-corrected-path', 'maybe')

    assert result.returncode == 2
    assert 'error: --capture-corrected-path expects true or false.' in result.stderr
    assert 'Usage:' not in result.stderr


def test_dogfood_rejects_invalid_raw_capture_bool():
    result = _run_dogfood('--capture-raw-odometry', 'maybe')

    assert result.returncode == 2
    assert 'error: --capture-raw-odometry expects true or false.' in result.stderr


def test_dogfood_rejects_missing_bag_dir_without_realpath(tmp_path: Path):
    result = _run_dogfood(
        '--bag',
        str(tmp_path / 'missing_bag'),
        '--skip-viewer',
    )

    assert result.returncode == 2
    assert 'error: rosbag2 directory not found:' in result.stderr
    assert 'metadata.yaml' in result.stderr
    assert 'realpath' not in result.stderr


def test_dogfood_rejects_missing_metadata_without_ros_launch(tmp_path: Path):
    bag_dir = tmp_path / 'bag_without_metadata'
    bag_dir.mkdir()

    result = _run_dogfood('--bag', str(bag_dir), '--skip-viewer')

    assert result.returncode == 2
    assert 'error: metadata.yaml not found under' in result.stderr
    assert 'ros2 not found' not in result.stderr


def test_dogfood_rejects_output_file_before_ros_launch(tmp_path: Path):
    bag_dir = _write_minimal_bag(tmp_path)
    output_file = tmp_path / 'map_output'
    output_file.write_text('', encoding='utf-8')

    result = _run_dogfood(
        '--bag',
        str(bag_dir),
        '--output-dir',
        str(output_file),
        '--skip-viewer',
    )

    assert result.returncode == 2
    assert 'error: output directory path is a file' in result.stderr
    assert 'ros2 not found' not in result.stderr


def test_dogfood_omits_empty_optional_frame_launch_arguments():
    script = DOGFOOD_SCRIPT.read_text(encoding='utf-8')

    assert 'LAUNCH_ARGS=(' in script
    assert 'if [[ -n "$LIDAR_FRAME" ]]; then' in script
    assert 'LAUNCH_ARGS+=("lidar_frame:=${LIDAR_FRAME}")' in script
    assert 'if [[ -n "$IMU_FRAME" ]]; then' in script
    assert 'LAUNCH_ARGS+=("imu_frame:=${IMU_FRAME}")' in script
    assert '"${LAUNCH_ARGS[@]}"' in script
    assert '"lidar_frame:=${LIDAR_FRAME}" \\' not in script
    assert '"imu_frame:=${IMU_FRAME}" \\' not in script
