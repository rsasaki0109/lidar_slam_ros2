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

"""Tests for deterministic graph-SLAM ablation comparison."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'ablation', ROOT / 'scripts/compare_graph_slam_offline_ablation.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write_run(root: Path, *, loop: str, rotation: float, thickness: float,
               coverage: float, elapsed: str, rss_kb: int) -> None:
    run = root / 'run1'
    run.mkdir(parents=True)
    (run / 'loop_edges.csv').write_text(loop)
    (run / 'pose_graph_loop_residuals.json').write_text(
        '{"status":"PASS","loop_edge_count":1,"summary":'
        f'{{"translation_residual_mean_m":0.01,"rotation_residual_mean_deg":{rotation}}}}}')
    quality = {'map_quality_report': {
        'mean_map_entropy': {'value_nats': -1.0},
        'plane_metrics': {
            'thickness_rms_mean_m': thickness,
            'thickness_rms_p95_m': thickness * 2.0,
            'planar_coverage': coverage,
        },
    }}
    for stage in ('optimized', 'refined'):
        path = run / f'map_quality_{stage}/run1/map_quality_report.yaml'
        path.parent.mkdir(parents=True)
        path.write_text(yaml.safe_dump(quality))
    (root / 'process_time.txt').write_text(
        f'Elapsed (wall clock) time (h:mm:ss or m:ss): {elapsed}\n'
        f'Maximum resident set size (kbytes): {rss_kb}\nExit status: 0\n')


def test_fixed_input_improvement_still_requires_multiple_datasets(tmp_path: Path):
    baseline = tmp_path / 'baseline'
    candidate = tmp_path / 'candidate'
    _write_run(baseline, loop='same\n', rotation=2.0, thickness=0.05,
               coverage=0.4, elapsed='2:00.00', rss_kb=102400)
    _write_run(candidate, loop='same\n', rotation=1.5, thickness=0.049,
               coverage=0.41, elapsed='1:00:00', rss_kb=103424)

    report = MODULE.build_report(
        baseline, candidate, baseline_runs=2, candidate_runs=60,
        dataset='public', parameter='weight', baseline_value=200,
        candidate_value=400, improved_datasets=1, minimum_improved_datasets=2,
        max_geometry_regression_percent=2.0)

    assert report['fixed_input']['passed'] is True
    assert report['loop_constraint_residuals']['rotation_residual_mean_deg'][
        'change_percent'] == pytest.approx(-25.0)
    assert report['resources']['wall_seconds_per_run']['baseline'] == 60.0
    assert report['resources']['wall_seconds_per_run']['candidate'] == 60.0
    assert report['trajectory_accuracy_claimed'] is False
    assert report['verdict'] == 'DO_NOT_ADOPT'
    assert report['verdict_reasons'] == ['minimum improved-dataset count is not met']


def test_geometry_regression_is_a_hard_rejection(tmp_path: Path):
    baseline = tmp_path / 'baseline'
    candidate = tmp_path / 'candidate'
    _write_run(baseline, loop='same\n', rotation=2.0, thickness=0.05,
               coverage=0.4, elapsed='1:00', rss_kb=1000)
    _write_run(candidate, loop='same\n', rotation=1.0, thickness=0.052,
               coverage=0.4, elapsed='1:00', rss_kb=1000)

    report = MODULE.build_report(
        baseline, candidate, baseline_runs=1, candidate_runs=1,
        dataset='public', parameter='weight', baseline_value=200,
        candidate_value=400, improved_datasets=2, minimum_improved_datasets=2,
        max_geometry_regression_percent=2.0)

    assert report['verdict'] == 'DO_NOT_ADOPT'
    assert {row['metric'] for row in report['gates']['geometry_regressions']} == {
        'thickness_rms_mean_m', 'thickness_rms_p95_m'}
