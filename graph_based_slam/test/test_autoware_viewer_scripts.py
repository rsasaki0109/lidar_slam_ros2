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

"""CLI regression tests for Autoware map viewer shell entrypoints."""

from __future__ import annotations

from pathlib import Path
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'

GRAPH_VIEWER_WRAPPERS = (
    'run_graph_slam_pointcloud_map_in_autoware.sh',
    'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh',
)
MAP_VIEWER_SCRIPTS = (
    'run_autoware_pointcloud_map_viewer_docker.sh',
    'run_autoware_pointcloud_map_foxglove.sh',
)
ALL_VIEWER_SCRIPTS = GRAPH_VIEWER_WRAPPERS + MAP_VIEWER_SCRIPTS


def _run_script(script_name: str, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['bash', str(SCRIPT_DIR / script_name), *args],
        check=False,
        capture_output=True,
        text=True,
    )


@pytest.mark.parametrize('script_name', ALL_VIEWER_SCRIPTS)
def test_viewer_script_help_exits_successfully(script_name: str):
    result = _run_script(script_name, '--help')

    assert result.returncode == 0
    assert 'Usage:' in result.stderr
    assert script_name in result.stderr
    assert 'GNU coreutils' not in result.stderr


@pytest.mark.parametrize('script_name', GRAPH_VIEWER_WRAPPERS)
def test_graph_viewer_wrapper_requires_source_before_options(script_name: str):
    result = _run_script(script_name, '--stage-dir', '/tmp/stage')

    assert result.returncode == 2
    assert 'error: graph_slam_output_dir is required before options.' in result.stderr
    assert 'realpath' not in result.stderr


@pytest.mark.parametrize('script_name', MAP_VIEWER_SCRIPTS)
def test_map_viewer_script_requires_map_dir_before_options(script_name: str):
    result = _run_script(script_name, '--run-dir', '/tmp/runtime')

    assert result.returncode == 2
    assert 'error: autoware_map_dir is required before options.' in result.stderr
    assert 'realpath' not in result.stderr


@pytest.mark.parametrize('script_name', GRAPH_VIEWER_WRAPPERS)
def test_graph_viewer_wrapper_rejects_missing_option_value(
    tmp_path: Path,
    script_name: str,
):
    source_dir = tmp_path / 'graph_output'
    source_dir.mkdir()

    result = _run_script(script_name, str(source_dir), '--stage-dir', '--rebuild')

    assert result.returncode == 2
    assert 'error: option requires a value: --stage-dir' in result.stderr
    assert 'realpath' not in result.stderr


@pytest.mark.parametrize('script_name', MAP_VIEWER_SCRIPTS)
def test_map_viewer_script_rejects_missing_option_value(
    tmp_path: Path,
    script_name: str,
):
    map_dir = tmp_path / 'map_bundle'
    map_dir.mkdir()

    result = _run_script(script_name, str(map_dir), '--run-dir', '--rebuild')

    assert result.returncode == 2
    assert 'error: option requires a value: --run-dir' in result.stderr
    assert 'realpath' not in result.stderr


@pytest.mark.parametrize('script_name', GRAPH_VIEWER_WRAPPERS)
def test_graph_viewer_wrapper_reports_missing_source_directory(
    tmp_path: Path,
    script_name: str,
):
    result = _run_script(script_name, str(tmp_path / 'missing_output'))

    assert result.returncode == 2
    assert 'error: graph_based_slam output directory not found:' in result.stderr
    assert 'realpath' not in result.stderr


@pytest.mark.parametrize('script_name', MAP_VIEWER_SCRIPTS)
def test_map_viewer_script_reports_missing_map_directory(
    tmp_path: Path,
    script_name: str,
):
    result = _run_script(script_name, str(tmp_path / 'missing_map'))

    assert result.returncode == 2
    assert 'error: Autoware map directory not found:' in result.stderr
    assert 'realpath' not in result.stderr
