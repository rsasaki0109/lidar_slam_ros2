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

"""Tests for optimized pose-graph loop residual analysis."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'analyze_pose_graph_loop_residuals.py'
SPEC = importlib.util.spec_from_file_location('loop_residuals', SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write_graph(path: Path, loop_tx: float, vertex_tx: float) -> None:
    path.write_text(
        '\n'.join([
            'VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1',
            f'VERTEX_SE3:QUAT 100 {vertex_tx} 0 0 0 0 0 1',
            'EDGE_SE3:QUAT 0 1 1 0 0 0 0 0 1 ' + ' '.join(['1'] * 21),
            f'EDGE_SE3:QUAT 0 100 {loop_tx} 0 0 0 0 0 1 ' + ' '.join(['1'] * 21),
        ]) + '\n',
        encoding='utf-8',
    )


def test_exact_loop_constraint_has_zero_residual(tmp_path: Path):
    graph = tmp_path / 'exact.g2o'
    _write_graph(graph, loop_tx=5.0, vertex_tx=5.0)

    report = MODULE.analyze(graph, adjacency_window=20)

    assert report['status'] == 'PASS'
    assert report['vertex_count'] == 2
    assert report['edge_count'] == 2
    assert report['loop_edge_count'] == 1
    assert math.isclose(report['loop_edges'][0]['translation_residual_m'], 0.0)
    assert math.isclose(report['loop_edges'][0]['rotation_residual_deg'], 0.0)


def test_translation_residual_uses_measurement_inverse(tmp_path: Path):
    graph = tmp_path / 'offset.g2o'
    _write_graph(graph, loop_tx=5.0, vertex_tx=7.5)

    report = MODULE.analyze(graph, adjacency_window=20)

    assert math.isclose(report['loop_edges'][0]['translation_residual_m'], 2.5)
    assert math.isclose(report['loop_edges'][0]['optimized_pair_distance_m'], 7.5)
    assert math.isclose(report['loop_edges'][0]['measurement_translation_m'], 5.0)


def test_no_long_baseline_edge_fails_gate(tmp_path: Path):
    graph = tmp_path / 'adjacent_only.g2o'
    graph.write_text(
        'VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\n'
        'VERTEX_SE3:QUAT 1 1 0 0 0 0 0 1\n'
        'EDGE_SE3:QUAT 0 1 1 0 0 0 0 0 1 ' + ' '.join(['1'] * 21) + '\n',
        encoding='utf-8',
    )

    report = MODULE.analyze(graph, adjacency_window=20)

    assert report['status'] == 'FAIL'
    assert report['loop_edge_count'] == 0


def test_offline_csv_constraint_against_tum_vertices(tmp_path: Path):
    trajectory = tmp_path / 'trajectory_optimized.tum'
    trajectory.write_text(
        '0 0 0 0 0 0 0 1\n'
        '1 7.5 0 0 0 0 0 1\n',
        encoding='utf-8',
    )
    loops = tmp_path / 'loop_edges.csv'
    loops.write_text(
        'from,to,fitness,tx,ty,tz,qx,qy,qz,qw\n'
        '0,1,0.25,5,0,0,0,0,0,1\n',
        encoding='utf-8',
    )

    report = MODULE.analyze_offline(trajectory, loops)

    assert report['status'] == 'PASS'
    assert report['loop_edges'][0]['fitness'] == 0.25
    assert math.isclose(report['loop_edges'][0]['translation_residual_m'], 2.5)


def test_offline_csv_constraint_against_timestamp_matched_reference(tmp_path: Path):
    trajectory = tmp_path / 'trajectory_raw.tum'
    trajectory.write_text(
        '10.0 100 0 0 0 0 0 1\n'
        '20.0 200 0 0 0 0 0 1\n',
        encoding='utf-8',
    )
    reference = tmp_path / 'ground_truth.tum'
    reference.write_text(
        '9.99 1 0 0 0 0 0 1\n'
        '20.01 8.5 0 0 0 0 0 1\n',
        encoding='utf-8',
    )
    loops = tmp_path / 'loop_edges.csv'
    loops.write_text(
        'from,to,fitness,tx,ty,tz,qx,qy,qz,qw\n'
        '0,1,0.25,5,0,0,0,0,0,1\n',
        encoding='utf-8',
    )

    report = MODULE.analyze_offline_reference(
        trajectory, reference, loops, max_time_diff=0.02)

    assert report['status'] == 'PASS'
    assert report['loop_edges_missing_vertices'] == 0
    assert math.isclose(report['loop_edges'][0]['translation_residual_m'], 2.5)
    assert math.isclose(report['loop_edges'][0]['reference_pair_distance_m'], 7.5)
