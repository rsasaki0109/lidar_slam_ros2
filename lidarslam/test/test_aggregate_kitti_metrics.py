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

"""Regression tests for KITTI metric aggregation."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'aggregate_kitti_metrics.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('aggregate_kitti_metrics', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding='utf-8')


def test_label_prefix_overrides_json_label(tmp_path: Path):
    module = _load_module()
    p = tmp_path / 'm.json'
    _write_json(p, {'label': 'from_json', 't_rel_percent_avg': 1.0,
                    'r_rel_deg_per_m_avg': 0.001, 'sequence': '00'})
    [rec] = module.load_metric_files([f'forced::{p}'])
    assert rec['label'] == 'forced'
    assert rec['sequence'] == '00'


def test_sequence_inferred_from_path_when_missing(tmp_path: Path):
    module = _load_module()
    p = tmp_path / 'kitti_bench_05_run' / 'metrics.json'
    _write_json(p, {'t_rel_percent_avg': 1.2, 'r_rel_deg_per_m_avg': 0.002})
    [rec] = module.load_metric_files([str(p)])
    assert rec['sequence'] == '05'


def test_sequence_inferred_from_seq_prefix(tmp_path: Path):
    module = _load_module()
    p = tmp_path / 'seq07_kiss.json'
    _write_json(p, {'t_rel_percent_avg': 0.5, 'r_rel_deg_per_m_avg': 0.001})
    [rec] = module.load_metric_files([str(p)])
    assert rec['sequence'] == '07'


def test_missing_input_file_raises(tmp_path: Path):
    module = _load_module()
    with pytest.raises(FileNotFoundError):
        module.load_metric_files([str(tmp_path / 'does_not_exist.json')])


def test_single_estimator_uses_flat_table(tmp_path: Path):
    module = _load_module()
    p00 = tmp_path / 'seq00.json'
    p05 = tmp_path / 'seq05.json'
    _write_json(p00, {'sequence': '00', 't_rel_percent_avg': 0.50,
                      'r_rel_deg_per_m_avg': 0.002, 'pairs_total': 1200, 'frames': 4541})
    _write_json(p05, {'sequence': '05', 't_rel_percent_avg': 0.70,
                      'r_rel_deg_per_m_avg': 0.003, 'pairs_total': 800, 'frames': 2761})
    recs = module.load_metric_files([f'ours::{p00}', f'ours::{p05}'])
    md = module.render_markdown(recs)
    assert '## Per-run metrics' in md
    assert '0.500' in md
    assert '0.700' in md
    assert 'average t_rel: 0.600%' in md


def test_two_estimators_render_side_by_side(tmp_path: Path):
    module = _load_module()
    p_ours_00 = tmp_path / 'ours_00.json'
    p_kiss_00 = tmp_path / 'kiss_00.json'
    p_ours_05 = tmp_path / 'ours_05.json'
    p_kiss_05 = tmp_path / 'kiss_05.json'
    _write_json(p_ours_00, {'sequence': '00', 't_rel_percent_avg': 0.50,
                            'r_rel_deg_per_m_avg': 0.002})
    _write_json(p_kiss_00, {'sequence': '00', 't_rel_percent_avg': 0.60,
                            'r_rel_deg_per_m_avg': 0.0025})
    _write_json(p_ours_05, {'sequence': '05', 't_rel_percent_avg': 0.70,
                            'r_rel_deg_per_m_avg': 0.003})
    _write_json(p_kiss_05, {'sequence': '05', 't_rel_percent_avg': 0.80,
                            'r_rel_deg_per_m_avg': 0.0035})
    recs = module.load_metric_files(
        [
            f'ours::{p_ours_00}', f'kiss::{p_kiss_00}',
            f'ours::{p_ours_05}', f'kiss::{p_kiss_05}',
        ]
    )
    md = module.render_markdown(recs)
    assert '## Per-sequence comparison' in md
    assert '## Aggregate per estimator' in md
    # Aggregate row for ours: avg t_rel = (0.50 + 0.70)/2 = 0.600
    assert '| ours | 2 | 0.600 |' in md
    # Aggregate row for kiss: avg t_rel = (0.60 + 0.80)/2 = 0.700
    assert '| kiss | 2 | 0.700 |' in md


def test_render_with_no_records_returns_placeholder():
    module = _load_module()
    md = module.render_markdown([])
    assert 'No input metrics' in md


def test_cli_writes_markdown(tmp_path: Path):
    module = _load_module()
    p = tmp_path / 'seq00.json'
    _write_json(p, {'sequence': '00', 't_rel_percent_avg': 0.42,
                    'r_rel_deg_per_m_avg': 0.001, 'pairs_total': 100, 'frames': 4000})
    out_md = tmp_path / 'report.md'
    rc = module.main(['--input', f'ours::{p}', '--out-md', str(out_md)])
    assert rc == 0
    body = out_md.read_text()
    assert 'KITTI Odometry aggregate report' in body
    assert '0.420' in body
