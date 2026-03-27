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

"""Regression tests for the classic-path report generator."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'generate_classic_path_report.py'


def _write_run_dir(
    root: Path,
    *,
    rmse: float,
    mean: float,
    max_value: float,
    pairs: int,
    loop_count: int,
    attempted: int,
    verify: str,
    projector_type: str,
) -> None:
    root.mkdir(parents=True, exist_ok=True)
    (root / 'metrics.json').write_text(
        json.dumps(
            {
                'evo': {
                    'ape': {
                        'rmse': rmse,
                        'mean': mean,
                        'max': max_value,
                        'pairs': pairs,
                    },
                },
                'graph_based_slam': {
                    'loop_count': loop_count,
                    'loop_count_attempted': attempted,
                },
            },
            indent=2,
        ) + '\n',
        encoding='utf-8',
    )
    (root / 'verify_autoware_map.log').write_text(
        f'RESULT: {verify} -- map is Autoware-compatible\n',
        encoding='utf-8',
    )
    (root / 'map_projector_info.yaml').write_text(
        f'projector_type: {projector_type}\n',
        encoding='utf-8',
    )


def test_classic_path_report_summarizes_gnss_gain_and_imu_delta(tmp_path):
    """The classic-path report should compare GNSS, odom-prior, and IMU variants."""
    no_gnss_dir = tmp_path / 'no_gnss'
    gnss_only_dir = tmp_path / 'gnss_only'
    gnss_odom_dir = tmp_path / 'gnss_odom'
    gnss_imu_dir = tmp_path / 'gnss_imu'
    out_md = tmp_path / 'classic.md'
    out_json = tmp_path / 'classic.json'
    out_svg = tmp_path / 'classic.svg'

    _write_run_dir(
        no_gnss_dir,
        rmse=313.69,
        mean=269.80,
        max_value=703.67,
        pairs=8724,
        loop_count=0,
        attempted=0,
        verify='PASS',
        projector_type='Local',
    )
    _write_run_dir(
        gnss_only_dir,
        rmse=195.28,
        mean=166.73,
        max_value=571.70,
        pairs=8124,
        loop_count=0,
        attempted=0,
        verify='PASS',
        projector_type='LocalCartesian',
    )
    _write_run_dir(
        gnss_odom_dir,
        rmse=184.10,
        mean=160.10,
        max_value=555.55,
        pairs=8200,
        loop_count=0,
        attempted=0,
        verify='PASS',
        projector_type='LocalCartesian',
    )
    _write_run_dir(
        gnss_imu_dir,
        rmse=271.14,
        mean=234.55,
        max_value=777.25,
        pairs=8574,
        loop_count=0,
        attempted=0,
        verify='PASS',
        projector_type='LocalCartesian',
    )

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--no-gnss-dir',
            str(no_gnss_dir),
            '--gnss-only-dir',
            str(gnss_only_dir),
            '--gnss-odom-dir',
            str(gnss_odom_dir),
            '--gnss-imu-dir',
            str(gnss_imu_dir),
            '--out',
            str(out_md),
            '--write-json',
            str(out_json),
            '--write-svg',
            str(out_svg),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    report = out_md.read_text(encoding='utf-8')
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert 'Backend GNSS improves APE RMSE' in report
    assert 'GNSS + odom prior' in report
    assert 'GNSS + IMU' in report
    assert payload['gnss_gain_m'] > 100.0
    assert payload['odom_delta_vs_gnss_only_m'] < 0.0
    assert payload['imu_delta_vs_gnss_only_m'] > 0.0
    assert out_svg.is_file()
    assert 'Classic path APE RMSE comparison' in out_svg.read_text(encoding='utf-8')
