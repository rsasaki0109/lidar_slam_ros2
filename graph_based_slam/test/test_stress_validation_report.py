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

"""Tests for the stress-validation report generator."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
REPORT_SCRIPT = REPO_ROOT / 'scripts' / 'generate_stress_validation_report.py'


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding='utf-8')


def test_generate_stress_validation_report(tmp_path):
    """The stress report should separate current and legacy evidence."""
    summary = tmp_path / 'output' / 'benchmark_summary.md'
    summary.parent.mkdir(parents=True, exist_ok=True)
    summary.write_text('# Benchmark summary\n', encoding='utf-8')

    fresh_metrics = tmp_path / 'output' / 'fresh' / 'metrics.json'
    best_metrics = tmp_path / 'output' / 'best' / 'metrics.json'
    _write_json(
        fresh_metrics,
        {
            'evo': {'ape': {'rmse': 0.952}},
            'graph_based_slam': {
                'map_verify': {
                    'ok': True,
                    'passes': ['Total points across all tiles: 265,551'],
                },
            },
        },
    )
    _write_json(
        best_metrics,
        {
            'evo': {'ape': {'rmse': 0.870}},
        },
    )

    mid360_metrics = tmp_path / 'output' / 'bench_rko_lio_mid360_v8' / 'metrics.json'
    _write_json(
        mid360_metrics,
        {
            'evo': {
                'ape': {'rmse': 4.004, 'pairs': 585},
                'raw_ape': {'rmse': 9.348},
            },
            'cross_validation': {
                'estimated_aligned_path_length_m': 1077.578492209529,
            },
        },
    )

    mid360_legacy_summary = tmp_path / 'output' / 'mid360_compare_summary.json'
    _write_json(
        mid360_legacy_summary,
        {
            'ape_rmse_m': 0.4570873505558762,
        },
    )

    mid360_log = tmp_path / 'output' / 'bench_rko_lio_mid360_v3' / 'graph_slam.log'
    mid360_log.parent.mkdir(parents=True, exist_ok=True)
    mid360_log.write_text(
        '\n'.join(
            [
                '[INFO] [x] [graph_based_slam]: Odom input: 120 submaps, distance: 82.6m',
                '[INFO] [x] [graph_based_slam]: Odom input: 600 submaps, distance: 1022.3m',
                'id_loop_point 1:0 id_loop_point 2:606',
            ],
        ),
        encoding='utf-8',
    )

    benchmark_readme = tmp_path / 'output' / 'BENCHMARK_README.md'
    benchmark_readme.write_text(
        '| Path length | 320m (loop closure available) |\n',
        encoding='utf-8',
    )

    newer_college_summary = tmp_path / 'output' / 'newer_college_mathhard_report_summary.json'
    _write_json(
        newer_college_summary,
        {
            'reference': {'path_length_m': 320.5639},
            'lidarslam': {'rmse': 12.1639},
        },
    )

    ntu_legacy_summary = tmp_path / 'output' / 'ntu_viral_tnp01_report_summary.json'
    _write_json(
        ntu_legacy_summary,
        {
            'lidarslam': {'rmse': 0.21556},
        },
    )

    ntu_prism_summary = tmp_path / 'output' / 'ntu_viral_tnp01_report_threads1_prism_summary.json'
    _write_json(
        ntu_prism_summary,
        {
            'lidarslam': {'rmse': 0.11726},
        },
    )

    out = tmp_path / 'output' / 'stress_validation_report.md'
    result = subprocess.run(
        [
            'python3',
            str(REPORT_SCRIPT),
            '--benchmark-summary',
            str(summary),
            '--fresh-metrics',
            str(fresh_metrics),
            '--best-metrics',
            str(best_metrics),
            '--mid360-metrics',
            str(mid360_metrics),
            '--mid360-log',
            str(mid360_log),
            '--mid360-legacy-summary',
            str(mid360_legacy_summary),
            '--benchmark-readme',
            str(benchmark_readme),
            '--newer-college-summary',
            str(newer_college_summary),
            '--ntu-legacy-summary',
            str(ntu_legacy_summary),
            '--ntu-prism-summary',
            str(ntu_prism_summary),
            '--out',
            str(out),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    report = out.read_text(encoding='utf-8')
    assert 'Stress Validation Report' in report
    assert '`0.952 m`' in report
    assert '`0.870 m`' in report
    assert '`4.004 m`' in report
    assert '`9.348 m`' in report
    assert '`1077.578 m`' in report
    assert '`1022.3 m`' in report
    assert '`0 -> 606`' in report
    assert '`0.457 m`' in report
    assert '`320m (loop closure available)`' in report
    assert '`12.164 m`' in report
    assert '`0.216 m`' in report
    assert '`0.117 m`' in report
