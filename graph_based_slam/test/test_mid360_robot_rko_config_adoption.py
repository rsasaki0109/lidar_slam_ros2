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

"""Tests for MID-360 RKO-LIO config adoption checks."""

from __future__ import annotations

import importlib
import json
from pathlib import Path
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
ADOPTION_SCRIPT = SCRIPT_DIR / 'check_mid360_robot_rko_config_adoption.py'


def _adoption_module():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    return importlib.import_module('mid360_robot_rko_config_adoption')


BEST_PARAMETERS = {
    'voxel_size': 0.5,
    'min_range': 1.0,
    'max_range': 100.0,
    'deskew': False,
    'double_downsample': True,
    'initialization_phase': False,
}


def _write_quality_report(tmp_path: Path) -> Path:
    quality = {
        'status': 'PASS',
        'cases': [
            {
                'case_id': 'voxel_0p50_min_1p00_dd_on',
                'rank': 1,
                'status': 'MAP_VERIFIED',
                'quality_score': 95,
                'parameters': BEST_PARAMETERS,
                'quality_gate': {'status': 'PASS'},
                'output_dir': str(tmp_path / 'sweep' / 'voxel_0p50_min_1p00_dd_on'),
            },
            {
                'case_id': 'voxel_0p30_min_1p00_dd_on',
                'rank': 2,
                'status': 'MAP_VERIFIED',
                'quality_score': 92,
                'parameters': {**BEST_PARAMETERS, 'voxel_size': 0.3},
                'quality_gate': {'status': 'PASS'},
                'output_dir': str(tmp_path / 'sweep' / 'voxel_0p30_min_1p00_dd_on'),
            },
        ],
    }
    path = tmp_path / 'mid360_robot_public_rko_quality_report.json'
    path.write_text(json.dumps(quality), encoding='utf-8')
    return path


def _write_config(tmp_path: Path, parameters: dict) -> Path:
    path = tmp_path / 'rko_config.yaml'
    path.write_text(yaml.safe_dump(parameters, sort_keys=False), encoding='utf-8')
    return path


def test_adoption_checker_passes_for_best_gate_config(tmp_path: Path):
    module = _adoption_module()
    quality_path = _write_quality_report(tmp_path)
    config_path = _write_config(tmp_path, BEST_PARAMETERS)

    report = module.RkoConfigAdoptionChecker(
        quality_report_path=quality_path,
        config_path=config_path,
        require_best=True,
    ).build_report()

    assert report['status'] == 'PASS'
    assert report['matched_case']['case_id'] == 'voxel_0p50_min_1p00_dd_on'
    assert report['recommended_case']['case_id'] == 'voxel_0p50_min_1p00_dd_on'
    assert report['parameter_diff_to_recommended'] == []
    assert all(check['status'] == 'PASS' for check in report['checks'])


def test_adoption_checker_fails_when_config_is_not_best(tmp_path: Path):
    module = _adoption_module()
    quality_path = _write_quality_report(tmp_path)
    config_path = _write_config(tmp_path, {**BEST_PARAMETERS, 'voxel_size': 0.3})

    report = module.RkoConfigAdoptionChecker(
        quality_report_path=quality_path,
        config_path=config_path,
        require_best=True,
    ).build_report()

    assert report['status'] == 'FAIL'
    assert report['matched_case']['case_id'] == 'voxel_0p30_min_1p00_dd_on'
    assert any(check['id'] == 'matched_case_is_best' and check['status'] == 'FAIL'
               for check in report['checks'])
    assert report['parameter_diff_to_recommended'][0]['key'] == 'voxel_size'


def test_adoption_report_writes_json_and_markdown(tmp_path: Path):
    module = _adoption_module()
    quality_path = _write_quality_report(tmp_path)
    config_path = _write_config(tmp_path, BEST_PARAMETERS)
    report = module.RkoConfigAdoptionChecker(
        quality_path, config_path, require_best=True,
    ).build_report()
    output_dir = tmp_path / 'out'

    paths = module.write_rko_config_adoption_report(report, output_dir)
    markdown = module.render_rko_config_adoption_markdown(report)

    assert paths['json'] == output_dir / module.RKO_CONFIG_ADOPTION_JSON
    assert paths['markdown'] == output_dir / module.RKO_CONFIG_ADOPTION_MARKDOWN
    assert 'matched_case' in markdown
    assert json.loads(paths['json'].read_text(encoding='utf-8'))['status'] == 'PASS'


def test_adoption_cli_returns_nonzero_for_non_best_config(tmp_path: Path):
    module = _adoption_module()
    quality_path = _write_quality_report(tmp_path)
    config_path = _write_config(tmp_path, {**BEST_PARAMETERS, 'voxel_size': 0.3})
    output_dir = tmp_path / 'adoption'

    result = subprocess.run(
        [
            sys.executable,
            str(ADOPTION_SCRIPT),
            '--quality-report',
            str(quality_path),
            '--config',
            str(config_path),
            '--output-dir',
            str(output_dir),
            '--require-best',
            '--json',
        ],
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert (output_dir / module.RKO_CONFIG_ADOPTION_JSON).is_file()
    assert (output_dir / module.RKO_CONFIG_ADOPTION_MARKDOWN).is_file()
