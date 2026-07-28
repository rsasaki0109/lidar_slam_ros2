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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for the dedicated map viewer product command."""

from __future__ import annotations

import importlib.util
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'view_autoware_map.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('view_autoware_map', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _map_output(tmp_path: Path) -> Path:
    output = tmp_path / 'map_output'
    pointcloud_map = output / 'pointcloud_map'
    pointcloud_map.mkdir(parents=True)
    (pointcloud_map / 'pointcloud_map_metadata.yaml').write_text(
        'version: 1\n',
        encoding='utf-8',
    )
    (output / 'map_projector_info.yaml').write_text(
        'projector_type: Local\n',
        encoding='utf-8',
    )
    return output


def test_help_has_a_small_viewer_specific_surface():
    """The optional viewer command should not expose map-generation options."""
    completed = subprocess.run(
        ['python3', str(SCRIPT_PATH), '--help'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0
    assert 'completed map-run output' in completed.stdout
    assert '--viewer {autoware,foxglove}' in completed.stdout
    assert '--runtime-dir' in completed.stdout
    assert '--viewer-run-dir' not in completed.stdout
    assert '--rebuild' in completed.stdout


def test_missing_or_incomplete_output_is_rejected_before_launch(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
):
    """Incomplete bundles should fail before an expensive viewer launch."""
    module = _load_module()

    assert module.main([str(tmp_path / 'missing')]) == 2
    assert 'map output directory does not exist' in capsys.readouterr().err

    incomplete = tmp_path / 'incomplete'
    incomplete.mkdir()
    assert module.main([str(incomplete)]) == 2
    error = capsys.readouterr().err
    assert 'map output is incomplete' in error
    assert 'lidarslam-map inspect <output_dir>' in error


@pytest.mark.parametrize(
    ('viewer', 'expected_script'),
    [
        ('autoware', 'run_graph_slam_pointcloud_map_in_autoware.sh'),
        ('foxglove', 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh'),
    ],
)
def test_viewer_command_maps_product_options_to_runtime_options(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    viewer: str,
    expected_script: str,
):
    """Product names should map deterministically to legacy helper names."""
    module = _load_module()
    output = _map_output(tmp_path)
    recorded: list[list[str]] = []

    def fake_run(command, **kwargs):
        recorded.append(command)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, 'run', fake_run)
    result = module.main([
        str(output),
        '--viewer',
        viewer,
        '--autoware-core-dir',
        '/opt/autoware-core',
        '--work-dir',
        '/tmp/view-work',
        '--runtime-dir',
        '/tmp/view-runtime',
        '--rebuild',
        '--auto-exit-secs',
        '30',
    ])

    assert result == 0
    command = recorded[0]
    assert command[:2] == ['bash', str(module.SCRIPT_DIR / expected_script)]
    assert command[2] == str(output.resolve())
    assert command[command.index('--autoware-core-dir') + 1] == '/opt/autoware-core'
    assert command[command.index('--work-dir') + 1] == '/tmp/view-work'
    assert command[command.index('--run-dir') + 1] == '/tmp/view-runtime'
    assert '--rebuild' in command
    assert command[command.index('--auto-exit-secs') + 1] == '30'


def test_viewer_failure_is_propagated(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    """Automation should receive the underlying viewer failure code."""
    module = _load_module()
    output = _map_output(tmp_path)
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda command, **kwargs: subprocess.CompletedProcess(command, 42),
    )

    assert module.main([str(output), '--viewer', 'foxglove']) == 42
    assert 'foxglove viewer failed with exit code 42' in capsys.readouterr().err
