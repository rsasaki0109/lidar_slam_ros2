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

"""Regression tests for the exploration closeout report generator."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'generate_exploration_closeout_report.py'


def _write_metrics(
    root: Path,
    *,
    rmse: float,
    pairs: int = 100,
    tum_lines: int = 10,
    est_len: float = 100.0,
    ref_len: float = 120.0,
) -> None:
    root.mkdir(parents=True, exist_ok=True)
    (root / 'metrics.json').write_text(
        json.dumps(
            {
                'evo': {'ape': {'rmse': rmse, 'pairs': pairs}},
                'lidarslam': {'tum_lines': tum_lines},
                'cross_validation': {
                    'estimated_path_length_m': est_len,
                    'reference_path_length_m': ref_len,
                },
            },
            indent=2,
        ) + '\n',
        encoding='utf-8',
    )


def _write_place_log(root: Path, lines: list[str]) -> None:
    (root / 'slam.launch.log').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _write_classic_log(root: Path, gnss_edges: int, rtk_edges: int) -> None:
    (root / 'lidarslam.launch.log').write_text(
        (
            '[graph_based_slam_node-2] [INFO] [0.0] [graph_based_slam]: '
            f'Added {gnss_edges} GNSS position constraint edges '
            f'({rtk_edges} RTK-like by covariance)\n'
        ),
        encoding='utf-8',
    )


def test_exploration_closeout_report_summarizes_current_decisions(tmp_path):
    """The report should encode the default/opt-in/fallback decisions."""
    place = {
        'distance_default': tmp_path / 'distance_default',
        'scan_context': tmp_path / 'scan_context',
        'bev_rerank': tmp_path / 'bev_rerank',
        'solid_descriptor': tmp_path / 'solid_descriptor',
    }
    classic = {
        'driving30_gnss_only': tmp_path / 'driving30_gnss_only',
        'driving30_velocity_always': tmp_path / 'driving30_velocity_always',
        'driving30_velocity_recovery': tmp_path / 'driving30_velocity_recovery',
        'bag6_gnss_only': tmp_path / 'bag6_gnss_only',
        'bag6_velocity_always': tmp_path / 'bag6_velocity_always',
        'bag6_velocity_recovery': tmp_path / 'bag6_velocity_recovery',
    }
    out_md = tmp_path / 'closeout.md'
    out_json = tmp_path / 'closeout.json'

    _write_metrics(place['distance_default'], rmse=3.641)
    _write_metrics(place['scan_context'], rmse=3.568)
    _write_metrics(place['bev_rerank'], rmse=3.607)
    _write_metrics(place['solid_descriptor'], rmse=3.632)
    _write_place_log(place['distance_default'], ['loop_candidate_source:distance'])
    _write_place_log(
        place['scan_context'],
        ['ScanContext loop candidate: id=1 sc_dist=0.42', 'loop_candidate_source:distance'],
    )
    _write_place_log(
        place['bev_rerank'],
        ['Distance candidate reranked by BEV: id=7 dist_m=45.0 bev_score=0.40'],
    )
    _write_place_log(place['solid_descriptor'], ['SOLiD rerank candidate: id=12 score=0.83'])

    _write_metrics(
        classic['driving30_gnss_only'],
        rmse=195.285,
        pairs=8124,
        tum_lines=813,
        est_len=2549.9,
        ref_len=4515.5,
    )
    _write_metrics(
        classic['driving30_velocity_always'],
        rmse=175.732,
        pairs=6564,
        tum_lines=657,
        est_len=2051.6,
        ref_len=4515.5,
    )
    _write_metrics(
        classic['driving30_velocity_recovery'],
        rmse=218.846,
        pairs=7025,
        tum_lines=703,
        est_len=1898.9,
        ref_len=4515.5,
    )
    _write_metrics(
        classic['bag6_gnss_only'],
        rmse=28.171,
        pairs=243,
        tum_lines=25,
        est_len=66.2,
        ref_len=120.5,
    )
    _write_metrics(
        classic['bag6_velocity_always'],
        rmse=35.202,
        pairs=253,
        tum_lines=26,
        est_len=50.4,
        ref_len=120.5,
    )
    _write_metrics(
        classic['bag6_velocity_recovery'],
        rmse=0.466,
        pairs=393,
        tum_lines=40,
        est_len=118.4,
        ref_len=120.5,
    )
    _write_classic_log(classic['driving30_gnss_only'], 802, 635)
    _write_classic_log(classic['driving30_velocity_always'], 657, 535)
    _write_classic_log(classic['driving30_velocity_recovery'], 677, 505)
    _write_classic_log(classic['bag6_gnss_only'], 0, 0)
    _write_classic_log(classic['bag6_velocity_always'], 0, 0)
    _write_classic_log(classic['bag6_velocity_recovery'], 0, 0)

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--distance-default-dir',
            str(place['distance_default']),
            '--scan-context-dir',
            str(place['scan_context']),
            '--bev-rerank-dir',
            str(place['bev_rerank']),
            '--solid-descriptor-dir',
            str(place['solid_descriptor']),
            '--driving30-gnss-only-dir',
            str(classic['driving30_gnss_only']),
            '--driving30-velocity-always-dir',
            str(classic['driving30_velocity_always']),
            '--driving30-velocity-recovery-dir',
            str(classic['driving30_velocity_recovery']),
            '--bag6-gnss-only-dir',
            str(classic['bag6_gnss_only']),
            '--bag6-velocity-always-dir',
            str(classic['bag6_velocity_always']),
            '--bag6-velocity-recovery-dir',
            str(classic['bag6_velocity_recovery']),
            '--out',
            str(out_md),
            '--write-json',
            str(out_json),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    report = out_md.read_text(encoding='utf-8')
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert 'Place Recognition Decision' in report
    assert 'Classic Path Decision' in report
    assert 'Public default remains `distance`' in report
    assert '`classic path` remains a fallback path' in report
    assert payload['recommendations']['place_recognition_default'] == 'distance_default'
    assert payload['recommendations']['classic_path_position'] == 'fallback_only'
