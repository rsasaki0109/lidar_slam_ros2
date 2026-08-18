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

"""Tests for conservative repeated held-out visual aggregation."""

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'summarize_repeated_visual_quality.py'
SPEC = importlib.util.spec_from_file_location('visual_summary', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _report(path, median, inlier, scored=100):
    body = {
        'train_views': 10, 'heldout_views': 10, 'heldout_views_scored': 2,
        'color_source': 'pointcloud', 'normalize_exposure': True,
        'exposure_scale_limit': 1.5, 'visible_points': 100,
        'scored_points': scored, 'heldout_scored_fraction': scored / 100,
        'rgb_l2_median': median, 'rgb_l2_inlier_20': inlier,
    }
    path.write_text(json.dumps(body))
    return path


def test_uses_conservative_worst_case_and_hashes_reports(tmp_path):
    reports = [
        _report(tmp_path / 'one.json', 10, 0.8),
        _report(tmp_path / 'two.json', 12, 0.7),
        _report(tmp_path / 'three.json', 11, 0.9),
    ]
    maps = [_report(tmp_path / f'map{index}.pcd', 1, 1)
            for index in range(3)]
    trajectories = [_report(tmp_path / f'traj{index}.tum', 1, 1)
                    for index in range(3)]
    transforms = [_report(tmp_path / f'transforms{index}.json', 1, 1)
                  for index in range(3)]
    calibration = tmp_path / 'calibration.yaml'
    calibration.write_text('calibration')
    summary = MODULE.summarize(
        reports, maps=maps, trajectories=trajectories,
        transforms=transforms, calibrations=[calibration])
    assert summary['valid_repetitions'] == 3
    assert summary['aggregate']['heldout_rgb_l2_median'] == 12
    assert summary['aggregate']['heldout_rgb_inlier_20'] == 0.7
    assert all(len(run['report_sha256']) == 64 for run in summary['runs'])
    assert all(len(run['map_sha256']) == 64 for run in summary['runs'])
    assert len(summary['calibrations'][0]['sha256']) == 64


def test_rejects_wrong_count_or_different_protocol(tmp_path):
    reports = [
        _report(tmp_path / 'one.json', 10, 0.8),
        _report(tmp_path / 'two.json', 12, 0.7),
        _report(tmp_path / 'three.json', 11, 0.9),
    ]
    with pytest.raises(ValueError, match='exactly three'):
        MODULE.summarize(reports[:2])
    with pytest.raises(ValueError, match='map and report counts differ'):
        MODULE.summarize(reports, maps=reports[:2])
    body = json.loads(reports[2].read_text())
    body['normalize_exposure'] = False
    reports[2].write_text(json.dumps(body))
    with pytest.raises(ValueError, match='protocols differ'):
        MODULE.summarize(reports)
