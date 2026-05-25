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

"""Tests for the synthetic MID-360 field-session runner."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import pytest


pytest.importorskip('rosbags')

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_mid360_robot_sample_session.py'
DEFAULT_PROFILE = REPO_ROOT / 'configs' / 'mid360_robot' / 'livox_mid360_default.yaml'


def _run_sample_session(
    tmp_path: Path,
    scenario: str,
) -> tuple[subprocess.CompletedProcess[str], dict, Path, Path]:
    bag_root = tmp_path / f'bags_{scenario}'
    output_dir = tmp_path / f'out_{scenario}'
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--robot-profile',
            str(DEFAULT_PROFILE),
            '--bag-root',
            str(bag_root),
            '--run-id',
            scenario,
            '--output-dir',
            str(output_dir),
            '--duration-sec',
            '2',
            '--scenario',
            scenario,
            '--force',
            '--json',
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    return result, json.loads(result.stdout), bag_root, output_dir


def test_sample_session_writes_full_dashboard_artifact_set(tmp_path: Path):
    bag_root = tmp_path / 'bags'
    output_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--robot-profile',
            str(DEFAULT_PROFILE),
            '--bag-root',
            str(bag_root),
            '--run-id',
            'sample_01',
            '--output-dir',
            str(output_dir),
            '--duration-sec',
            '2',
            '--pointcloud-rate-hz',
            '10',
            '--imu-rate-hz',
            '100',
            '--force',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['status'] == 'PASS'
    assert report['sample_session'] is True
    assert (bag_root / 'sample_01' / 'metadata.yaml').is_file()
    assert (bag_root / 'sample_01_record_plan.json').is_file()
    assert (bag_root / 'sample_01_profile.yaml').is_file()
    assert (output_dir / 'mid360_robot_field_session.json').is_file()
    assert (output_dir / 'mid360_robot_recording_check.json').is_file()
    assert (output_dir / 'mid360_robot_readiness.json').is_file()
    assert (output_dir / 'mid360_robot_run_plan.json').is_file()
    dashboard = output_dir / 'mid360_robot_session_dashboard.html'
    assert dashboard.is_file()
    assert 'Route Sketch' in dashboard.read_text(encoding='utf-8')

    readiness = json.loads((output_dir / 'mid360_robot_readiness.json').read_text())
    assert readiness['status'] == 'PASS'
    assert readiness['bag_diagnostics']['sample_reader']['available'] is True
    assert readiness['bag_diagnostics']['topics']['pointcloud']['sampled_frame_ids'] == [
        'livox_frame'
    ]
    assert any(step['id'] == 'recording' and step['status'] == 'ok'
               for step in report['steps'])
    assert any(step['id'] == 'map' and step['status'] == 'planned'
               for step in report['steps'])


def test_sample_session_low_rate_scenario_surfaces_warn(tmp_path: Path):
    result, report, _, output_dir = _run_sample_session(tmp_path, 'low-rate')

    assert result.returncode == 0
    assert report['status'] == 'WARN'
    assert report['scenario'] == 'low-rate'
    assert report['effective_sample_options']['pointcloud_rate_hz'] == 2.0
    assert report['effective_sample_options']['imu_rate_hz'] == 20.0
    readiness = json.loads((output_dir / 'mid360_robot_readiness.json').read_text())
    assert readiness['status'] == 'WARN'
    assert any(check['id'] == 'pointcloud_metadata_rate' and check['status'] == 'warn'
               for check in readiness['checks'])
    assert any(check['id'] == 'imu_metadata_rate' and check['status'] == 'warn'
               for check in readiness['checks'])
    assert any(step['id'] == 'post_recording_check' and step['status'] == 'warn'
               for step in report['steps'])
    assert any(step['id'] == 'map' and step['status'] == 'planned'
               for step in report['steps'])


def test_sample_session_missing_tf_scenario_surfaces_warn(tmp_path: Path):
    result, report, _, output_dir = _run_sample_session(tmp_path, 'missing-tf')

    assert result.returncode == 0
    assert report['status'] == 'WARN'
    assert report['sample_bag']['tf_static_messages'] == 0
    assert report['effective_sample_options']['write_tf_static'] is False
    readiness = json.loads((output_dir / 'mid360_robot_readiness.json').read_text())
    assert readiness['status'] == 'WARN'
    assert any(check['id'] == 'tf_metadata' and check['status'] == 'warn'
               for check in readiness['checks'])
    assert readiness['bag_diagnostics']['tf']['topics'] == []
    dashboard = (output_dir / 'mid360_robot_session_dashboard.html').read_text(encoding='utf-8')
    assert 'WARN' in dashboard


def test_sample_session_frame_mismatch_scenario_surfaces_fail(tmp_path: Path):
    result, report, _, output_dir = _run_sample_session(tmp_path, 'frame-mismatch')

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert report['effective_sample_options']['lidar_frame'] == 'sample_wrong_lidar_frame'
    readiness = json.loads((output_dir / 'mid360_robot_readiness.json').read_text())
    assert readiness['status'] == 'FAIL'
    assert any(check['id'] == 'pointcloud_frame_id' and check['status'] == 'fail'
               for check in readiness['checks'])
    assert any(step['id'] == 'post_recording_check' and step['status'] == 'fail'
               for step in report['steps'])
    assert any(step['id'] == 'map' and step['status'] == 'skipped'
               for step in report['steps'])
    assert not (output_dir / 'mid360_robot_run_plan.json').exists()


def test_sample_session_requires_force_for_existing_bag(tmp_path: Path):
    bag_root = tmp_path / 'bags'
    output_dir = tmp_path / 'out'
    command = [
        sys.executable,
        str(SCRIPT_PATH),
        '--robot-profile',
        str(DEFAULT_PROFILE),
        '--bag-root',
        str(bag_root),
        '--run-id',
        'sample_01',
        '--output-dir',
        str(output_dir),
        '--duration-sec',
        '1',
    ]
    subprocess.run(
        command + ['--force'],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    second = subprocess.run(command, capture_output=True, text=True, cwd=REPO_ROOT)

    assert second.returncode == 1
    assert 'use --force' in second.stderr
