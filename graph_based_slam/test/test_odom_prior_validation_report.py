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

"""Regression tests for the odom-prior validation report generator."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'generate_odom_prior_validation_report.py'


def _write_run_dir(
    root: Path,
    *,
    rmse: float,
    pairs: int,
    tum_lines: int,
    est_len: float,
    ref_len: float,
    gnss_edges: int,
    rtk_edges: int,
    projector_type: str,
) -> None:
    root.mkdir(parents=True, exist_ok=True)
    (root / 'metrics.json').write_text(
        json.dumps(
            {
                'evo': {'ape': {'rmse': rmse, 'pairs': pairs}},
                'cross_validation': {
                    'estimated_path_length_m': est_len,
                    'reference_path_length_m': ref_len,
                },
                'lidarslam': {'tum_lines': tum_lines},
            },
            indent=2,
        ) + '\n',
        encoding='utf-8',
    )
    (root / 'verify_autoware_map.log').write_text(
        'RESULT: PASS -- map is Autoware-compatible\n',
        encoding='utf-8',
    )
    (root / 'map_projector_info.yaml').write_text(
        f'projector_type: {projector_type}\n',
        encoding='utf-8',
    )
    (root / 'lidarslam.launch.log').write_text(
        (
            '[graph_based_slam_node-2] [INFO] [0.0] [graph_based_slam]: '
            f'Added {gnss_edges} GNSS position constraint edges '
            f'({rtk_edges} RTK-like by covariance)\n'
        ),
        encoding='utf-8',
    )


def test_odom_prior_validation_report_captures_dataset_specific_behavior(tmp_path):
    """The report should show that odom-prior behavior differs by dataset."""
    driving30_gnss = tmp_path / 'driving30_gnss'
    driving30_always = tmp_path / 'driving30_always'
    driving30_recovery = tmp_path / 'driving30_recovery'
    bag6_gnss = tmp_path / 'bag6_gnss'
    bag6_always = tmp_path / 'bag6_always'
    bag6_recovery = tmp_path / 'bag6_recovery'
    out_md = tmp_path / 'odom_prior.md'
    out_json = tmp_path / 'odom_prior.json'
    out_svg = tmp_path / 'odom_prior.svg'

    _write_run_dir(
        driving30_gnss,
        rmse=195.285,
        pairs=8124,
        tum_lines=813,
        est_len=2549.938,
        ref_len=4515.553,
        gnss_edges=640,
        rtk_edges=520,
        projector_type='LocalCartesian',
    )
    _write_run_dir(
        driving30_always,
        rmse=175.732,
        pairs=6564,
        tum_lines=657,
        est_len=2051.587,
        ref_len=4515.553,
        gnss_edges=657,
        rtk_edges=535,
        projector_type='LocalCartesian',
    )
    _write_run_dir(
        driving30_recovery,
        rmse=218.846,
        pairs=7025,
        tum_lines=703,
        est_len=1898.892,
        ref_len=4515.553,
        gnss_edges=677,
        rtk_edges=505,
        projector_type='LocalCartesian',
    )
    _write_run_dir(
        bag6_gnss,
        rmse=28.171,
        pairs=243,
        tum_lines=25,
        est_len=66.169,
        ref_len=120.519,
        gnss_edges=0,
        rtk_edges=0,
        projector_type='LocalCartesian',
    )
    _write_run_dir(
        bag6_always,
        rmse=35.202,
        pairs=253,
        tum_lines=26,
        est_len=50.424,
        ref_len=120.519,
        gnss_edges=0,
        rtk_edges=0,
        projector_type='LocalCartesian',
    )
    _write_run_dir(
        bag6_recovery,
        rmse=0.466,
        pairs=393,
        tum_lines=40,
        est_len=118.434,
        ref_len=120.519,
        gnss_edges=0,
        rtk_edges=0,
        projector_type='LocalCartesian',
    )

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--driving30-gnss-only-dir',
            str(driving30_gnss),
            '--driving30-velocity-always-dir',
            str(driving30_always),
            '--driving30-velocity-recovery-dir',
            str(driving30_recovery),
            '--bag6-gnss-only-dir',
            str(bag6_gnss),
            '--bag6-velocity-always-dir',
            str(bag6_always),
            '--bag6-velocity-recovery-dir',
            str(bag6_recovery),
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
    assert 'driving_30_kmh' in report
    assert 'bag6_front' in report
    assert 'bag6_front` runs show `GNSS edges=0`' in report
    assert payload['driving30_best_label'] == 'velocity_always'
    assert payload['bag6_best_label'] == 'velocity_recovery'
    assert out_svg.is_file()
    assert 'Classic-path velocity-prior validation' in out_svg.read_text(encoding='utf-8')
