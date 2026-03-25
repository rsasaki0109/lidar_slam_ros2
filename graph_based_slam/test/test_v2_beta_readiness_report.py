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

"""Tests for the v2 beta readiness report generator."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
REPORT_SCRIPT = REPO_ROOT / 'scripts' / 'generate_v2_beta_readiness_report.py'


def _write_metrics(path: Path, *, ape_rmse: float, total_points: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                'evo': {'ape': {'rmse': ape_rmse}},
                'lidarslam': {'wall_sec': 12.5, 'rtf': 0.42},
                'graph_based_slam': {
                    'map_verify': {
                        'ok': True,
                        'passes': [
                            f'Total points across all tiles: {total_points:,}',
                        ],
                    },
                },
            },
        ),
        encoding='utf-8',
    )


def test_generate_v2_beta_readiness_report(tmp_path):
    """The report generator should summarize benchmark and dogfood artifacts."""
    summary = tmp_path / 'output' / 'benchmark_summary.md'
    summary.parent.mkdir(parents=True, exist_ok=True)
    summary.write_text('# Benchmark summary\n', encoding='utf-8')

    fresh_metrics = tmp_path / 'output' / 'fresh' / 'metrics.json'
    best_metrics = tmp_path / 'output' / 'best' / 'metrics.json'
    _write_metrics(fresh_metrics, ape_rmse=0.95, total_points=265_551)
    _write_metrics(best_metrics, ape_rmse=0.87, total_points=257_751)

    dogfood_dir = tmp_path / 'output' / 'dogfood'
    ros_log_dir = dogfood_dir / '.ros_log'
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    (ros_log_dir / 'rviz2_test.log').write_text(
        '[INFO] [rviz]: Subscribing to: /map/pointcloud_map\n',
        encoding='utf-8',
    )
    (dogfood_dir / 'slam.launch.log').write_text('Saved grid-divided map\n', encoding='utf-8')
    (dogfood_dir / 'map_projector_info.yaml').write_text(
        yaml.safe_dump({'projector_type': 'Local'}, sort_keys=False),
        encoding='utf-8',
    )
    pointcloud_map = dogfood_dir / 'pointcloud_map'
    pointcloud_map.mkdir(parents=True, exist_ok=True)
    (pointcloud_map / 'pointcloud_map_metadata.yaml').write_text(
        yaml.safe_dump(
            {
                'x_resolution': 20.0,
                'y_resolution': 20.0,
                '0_0.pcd': [0, 0],
                '20_0.pcd': [20, 0],
            },
            sort_keys=False,
        ),
        encoding='utf-8',
    )

    out = tmp_path / 'output' / 'v2_beta_readiness.md'
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
            '--dogfood-dir',
            str(dogfood_dir),
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
    assert 'v2 Beta Readiness Report' in report
    assert '`0.950 m`' in report
    assert '`0.870 m`' in report
    assert '`projector_type: Local`' in report
    assert '`2`' in report
    assert '`yes`' in report
