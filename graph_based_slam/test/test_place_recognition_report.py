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

"""Regression tests for the place-recognition report generator."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'generate_place_recognition_report.py'


def _write_metrics(path: Path, rmse: float, loop_count: int, attempted: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                'evo': {'ape': {'rmse': rmse}},
                'graph_based_slam': {
                    'loop_count': loop_count,
                    'loop_count_attempted': attempted,
                },
            },
            indent=2,
        ) + '\n',
        encoding='utf-8',
    )


def test_place_recognition_report_marks_scan_context_regression(tmp_path):
    """The report should describe whether Scan Context helped or regressed."""
    baseline_metrics = tmp_path / 'baseline' / 'metrics.json'
    candidate_metrics = tmp_path / 'candidate' / 'metrics.json'
    baseline_log = tmp_path / 'baseline' / 'slam.launch.log'
    candidate_log = tmp_path / 'candidate' / 'slam.launch.log'
    out_path = tmp_path / 'place_report.md'

    _write_metrics(baseline_metrics, rmse=3.64, loop_count=1, attempted=1)
    _write_metrics(candidate_metrics, rmse=4.48, loop_count=2, attempted=2)
    baseline_log.write_text(
        '\n'.join(
            [
                'use_scan_context:false',
                'loop_candidate_source:distance',
            ],
        ) + '\n',
        encoding='utf-8',
    )
    candidate_log.write_text(
        '\n'.join(
            [
                'use_scan_context:true',
                'ScanContext loop candidate: id=42 sc_dist=0.21',
                'loop_candidate_source:distance',
                'loop_candidate_source:distance',
            ],
        ) + '\n',
        encoding='utf-8',
    )

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--baseline-metrics',
            str(baseline_metrics),
            '--baseline-log',
            str(baseline_log),
            '--candidate-metrics',
            str(candidate_metrics),
            '--candidate-log',
            str(candidate_log),
            '--out',
            str(out_path),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    report = out_path.read_text(encoding='utf-8')
    assert 'Runtime `use_scan_context`' in report
    assert '`False`' in report
    assert '`True`' in report
    assert 'Observed Scan Context candidates' in report
    assert 'regressed APE RMSE' in report


def test_place_recognition_report_marks_scan_context_improvement(tmp_path):
    """The report should describe a fair rerun improvement as improvement."""
    baseline_metrics = tmp_path / 'baseline' / 'metrics.json'
    candidate_metrics = tmp_path / 'candidate' / 'metrics.json'
    baseline_log = tmp_path / 'baseline' / 'slam.launch.log'
    candidate_log = tmp_path / 'candidate' / 'slam.launch.log'
    out_path = tmp_path / 'place_report.md'

    _write_metrics(baseline_metrics, rmse=4.10, loop_count=1, attempted=2)
    _write_metrics(candidate_metrics, rmse=3.82, loop_count=1, attempted=2)
    baseline_log.write_text(
        '\n'.join(
            [
                'use_scan_context:false',
                'loop_candidate_source:distance',
            ],
        ) + '\n',
        encoding='utf-8',
    )
    candidate_log.write_text(
        '\n'.join(
            [
                'use_scan_context:true',
                'ScanContext loop candidate: id=121 sc_dist=0.4319',
                'ScanContext loop candidate: id=132 sc_dist=0.4339',
                'loop_candidate_source:distance',
            ],
        ) + '\n',
        encoding='utf-8',
    )

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--baseline-metrics',
            str(baseline_metrics),
            '--baseline-log',
            str(baseline_log),
            '--candidate-metrics',
            str(candidate_metrics),
            '--candidate-log',
            str(candidate_log),
            '--out',
            str(out_path),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    report = out_path.read_text(encoding='utf-8')
    assert 'improved APE RMSE' in report
    assert '`2`' in report
