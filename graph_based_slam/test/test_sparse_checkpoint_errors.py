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
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the distribution.
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
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
# OF SUCH DAMAGE.

"""Tests for sparse-checkpoint trajectory error attribution."""

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'checkpoint_errors', ROOT / 'scripts/analyze_sparse_checkpoint_errors.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write_tum(path: Path, positions: list[tuple[float, float, float]]) -> None:
    path.write_text('\n'.join(
        f'{index:.3f} {x} {y} {z} 0 0 0 1'
        for index, (x, y, z) in enumerate(positions)) + '\n')


def test_report_attributes_checkpoint_regression_after_independent_alignment(tmp_path: Path):
    reference = tmp_path / 'reference.tum'
    baseline = tmp_path / 'baseline.tum'
    candidate = tmp_path / 'candidate.tum'
    positions = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
    _write_tum(reference, positions)
    _write_tum(baseline, [(x + 4, y - 2, z + 1) for x, y, z in positions])
    _write_tum(candidate, [(x + 4, y - 2, z + (1.4 if i == 3 else 1))
                           for i, (x, y, z) in enumerate(positions)])

    report = MODULE.analyze(
        reference, [('baseline', baseline), ('candidate', candidate)],
        'baseline', 0.01)

    assert report['methods'][0]['rmse_m'] == pytest.approx(0.0, abs=1e-12)
    assert report['methods'][1]['rmse_m'] > 0.05
    assert report['checkpoints'][3]['delta_from_baseline_m']['candidate'] > 0.0


def test_reference_csv_labels_are_preserved(tmp_path: Path):
    reference = tmp_path / 'reference.tum'
    estimate = tmp_path / 'estimate.tum'
    positions = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
    _write_tum(reference, positions)
    _write_tum(estimate, positions)
    labels = tmp_path / 'labels.csv'
    labels.write_text('point_id,env,timestamp\nA,outdoor,0\nB,indoor,1\nC,transition,2\n')

    report = MODULE.analyze(
        reference, [('raw', estimate)], 'raw', 0.01, labels)

    assert [(row['point_id'], row['env']) for row in report['checkpoints']] == [
        ('A', 'outdoor'), ('B', 'indoor'), ('C', 'transition')]
