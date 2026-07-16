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


"""Tests for conservative repeated map-quality aggregation."""

import importlib.util
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'summarize_repeated_map_quality.py'
SPEC = importlib.util.spec_from_file_location('map_repeat_summary', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _report(path, mean, p95, coverage, meaningful=True):
    path.write_text(yaml.safe_dump({'map_quality_report': {
        'input_points': 100, 'evaluated_points': 80,
        'plane_metrics': {'meaningful': meaningful,
                          'thickness_rms_mean_m': mean,
                          'thickness_rms_p95_m': p95,
                          'planar_coverage': coverage}}}))


def test_worst_case_uses_max_thickness_and_min_coverage(tmp_path):
    paths = [tmp_path / f'r{i}.yaml' for i in range(3)]
    _report(paths[0], 0.03, 0.08, 0.6)
    _report(paths[1], 0.04, 0.07, 0.5)
    _report(paths[2], 0.02, 0.09, 0.7)
    result = MODULE.summarize(paths)
    assert result['aggregate']['plane_thickness_mean_worst_m'] == 0.04
    assert result['aggregate']['plane_thickness_p95_worst_m'] == 0.09
    assert result['aggregate']['planar_coverage_worst'] == 0.5


def test_non_meaningful_plane_report_produces_invalid_evidence(tmp_path):
    path = tmp_path / 'bad.yaml'
    _report(path, 0.0, 0.0, 0.0, meaningful=False)
    result = MODULE.summarize([path])
    assert result['aggregation_valid'] is False
    assert result['valid_repetitions'] == 1
    assert result['meaningful_repetitions'] == 0
    assert result['aggregate'] is None
    assert 'not meaningful' in result['failure_reasons'][0]
    assert result['runs'][0]['plane_metrics_meaningful'] is False
