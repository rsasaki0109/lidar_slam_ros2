# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for the combined plane-revisit real-data A/B gate."""

import argparse
import importlib.util
import json
from pathlib import Path

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'evaluate_plane_revisit_ab.py'
SPEC = importlib.util.spec_from_file_location('evaluate_plane_revisit_ab', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write_fixtures(tmp_path: Path) -> argparse.Namespace:
    off_ape = tmp_path / 'off_ape.txt'
    on_ape = tmp_path / 'on_ape.txt'
    off_ape.write_text('pairs: 20\nrmse: 1.0\nmean: 0.8\nmax: 2.0\n')
    on_ape.write_text('pairs: 20\nrmse: 0.7\nmean: 0.6\nmax: 1.5\n')
    off_map = tmp_path / 'off_map.yaml'
    on_map = tmp_path / 'on_map.yaml'
    off_map.write_text(yaml.safe_dump({'map_quality_report': {'plane_metrics': {
        'thickness_rms_mean_m': 0.05, 'thickness_rms_p95_m': 0.10,
        'planar_coverage': 0.40}}}))
    on_map.write_text(yaml.safe_dump({'map_quality_report': {'plane_metrics': {
        'thickness_rms_mean_m': 0.04, 'thickness_rms_p95_m': 0.09,
        'planar_coverage': 0.42}}}))
    off_bim = tmp_path / 'off_bim.json'
    on_bim = tmp_path / 'on_bim.json'
    off_bim.write_text(json.dumps({'element_fit': {
        'coverage_ratio': {'mean': 0.20}, 'distance_rmse_m': {'mean': 0.06},
        'distance_p95_m': {'mean': 0.10}, 'distribution_ratio': {'mean': 0.95}}}))
    on_bim.write_text(json.dumps({'element_fit': {
        'coverage_ratio': {'mean': 0.25}, 'distance_rmse_m': {'mean': 0.05},
        'distance_p95_m': {'mean': 0.09}, 'distribution_ratio': {'mean': 0.9495}}}))
    plane = tmp_path / 'plane.yaml'
    plane.write_text(yaml.safe_dump({'plane_revisit': {
        'candidate_constraints': 100, 'constraints': 20,
        'constraints_rejected_initial_residual': 80,
        'chi2_before': 1.0, 'chi2_after': 0.5}}))
    return argparse.Namespace(
        off_ape=off_ape, on_ape=on_ape, off_map_quality=off_map,
        on_map_quality=on_map, off_bim=off_bim, on_bim=on_bim,
        on_plane_report=plane, distribution_tolerance=0.001)


def test_all_improvements_pass(tmp_path):
    result = MODULE.evaluate(_write_fixtures(tmp_path))
    assert result['passed'] is True
    assert all(result['checks'].values())
    assert result['metrics']['trajectory_rmse_m']['change_percent'] == pytest.approx(-30.0)


def test_map_regression_fails(tmp_path):
    args = _write_fixtures(tmp_path)
    report = yaml.safe_load(args.on_map_quality.read_text())
    report['map_quality_report']['plane_metrics']['thickness_rms_mean_m'] = 0.051
    args.on_map_quality.write_text(yaml.safe_dump(report))
    result = MODULE.evaluate(args)
    assert result['passed'] is False
    assert result['checks']['map_thickness_mean_improved'] is False
