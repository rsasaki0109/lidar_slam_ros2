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

"""Tests for the MID-360 sample-session QA matrix."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import pytest


pytest.importorskip('rosbags')

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_mid360_robot_sample_session_matrix.py'
DEFAULT_PROFILE = REPO_ROOT / 'configs' / 'mid360_robot' / 'livox_mid360_default.yaml'


def test_sample_session_matrix_matches_expected_statuses(tmp_path: Path):
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
            '--output-dir',
            str(output_dir),
            '--run-id-prefix',
            'matrix',
            '--duration-sec',
            '2',
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
    assert report['counts'] == {'matched': 4, 'mismatched': 0, 'total': 4}
    observed = {
        item['scenario']: item['observed_status']
        for item in report['scenarios']
    }
    assert observed == {
        'pass': 'PASS',
        'low-rate': 'WARN',
        'missing-tf': 'WARN',
        'frame-mismatch': 'FAIL',
    }
    assert (output_dir / 'mid360_robot_sample_session_matrix.json').is_file()
    assert (output_dir / 'mid360_robot_sample_session_matrix.md').is_file()
    matrix_html = output_dir / 'mid360_robot_sample_session_matrix.html'
    assert matrix_html.is_file()
    html = matrix_html.read_text(encoding='utf-8')
    assert 'MID-360 Sample Session Matrix' in html
    assert 'Scenario QA' in html
    assert 'pass' in html
    assert 'low-rate' in html
    assert 'missing-tf' in html
    assert 'frame-mismatch' in html
    assert 'PASS' in html
    assert 'WARN' in html
    assert 'FAIL' in html
    assert '<h2>Mismatched</h2><p>0</p>' in html
    assert 'pass/mid360_robot_session_dashboard.html' in html
    for item in report['scenarios']:
        assert Path(item['dashboard_html_path']).is_file()
        assert Path(item['field_session_json_path']).is_file()
        assert Path(item['readiness_json_path']).is_file()


def test_sample_session_matrix_reports_unexpected_scenario_as_failure(tmp_path: Path):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--robot-profile',
            str(DEFAULT_PROFILE),
            '--bag-root',
            str(tmp_path / 'bags'),
            '--output-dir',
            str(tmp_path / 'out'),
            '--scenario',
            'unknown-case',
            '--force',
            '--json',
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert report['counts'] == {'matched': 0, 'mismatched': 1, 'total': 1}
    assert report['scenarios'][0]['observed_status'] == 'ERROR'
    assert 'unknown sample session scenario' in report['scenarios'][0]['error']
    assert (tmp_path / 'out' / 'mid360_robot_sample_session_matrix.html').is_file()
