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

"""Tests for MID-360 robot production-readiness gate."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
PRODUCTION_SCRIPT = SCRIPT_DIR / 'check_mid360_robot_production_readiness.py'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_production_readiness import (  # noqa: E402
    Mid360ProductionReadinessGate,
    PRODUCTION_READINESS_JSON,
    PRODUCTION_READINESS_MARKDOWN,
    ProductionReadinessInputs,
    ProductionReadinessThresholds,
    render_production_readiness_markdown,
    write_production_readiness_report,
)


def _write_json(path: Path, payload: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    return path


def _production_artifacts(tmp_path: Path, *, bag_path: str | None = None) -> dict[str, Path]:
    artifact_dir = tmp_path / 'robot_artifacts'
    bag = bag_path or '/data/robot_bags/field_route_01'
    paths = {
        'host': artifact_dir / 'jetson_mid360_host_readiness.json',
        'recording': artifact_dir / 'mid360_robot_recording_check.json',
        'readiness': artifact_dir / 'mid360_robot_readiness.json',
        'map': artifact_dir / 'autoware_map_diagnosis.json',
        'adoption': tmp_path / 'public_gate' / 'mid360_robot_public_rko_adoption_gate.json',
    }
    _write_json(paths['host'], {
        'status': 'PASS',
        'checks': [{'id': 'jetson_model', 'status': 'ok', 'message': 'Jetson'}],
    })
    _write_json(paths['recording'], {
        'status': 'PASS',
        'bag_path': bag,
        'readiness_status': 'PASS',
        'checks': [{'id': 'readiness_status', 'status': 'ok', 'message': 'passed'}],
    })
    _write_json(paths['readiness'], {
        'status': 'PASS',
        'bag_path': bag,
        'ready_for_mid360_launch': True,
        'checks': [{'id': 'pointcloud2', 'status': 'ok', 'message': 'ok'}],
        'bag_diagnostics': {
            'topics': {
                'pointcloud': {
                    'metadata_message_count': 6000,
                    'metadata_rate_hz': 10.0,
                    'sampled_message_count': 20,
                    'stable_frame_id': True,
                    'matches_expected_frame': True,
                },
                'imu': {
                    'metadata_message_count': 120000,
                    'metadata_rate_hz': 200.0,
                    'sampled_message_count': 20,
                    'stable_frame_id': True,
                    'matches_expected_frame': True,
                },
            }
        },
    })
    _write_json(paths['map'], {
        'status': 'success',
        'verify': {'result': 'PASS'},
    })
    _write_json(paths['adoption'], {
        'status': 'PASS',
        'decision': {
            'matched_case': 'voxel_0p50_min_1p00_dd_on',
            'recommended_case': 'voxel_0p50_min_1p00_dd_on',
            'gate_pass_cases': 4,
        },
    })
    return paths


def _inputs(paths: dict[str, Path], output_dir: Path) -> ProductionReadinessInputs:
    return ProductionReadinessInputs(
        host_readiness=paths['host'],
        recording_check=paths['recording'],
        readiness=paths['readiness'],
        map_diagnosis=paths['map'],
        adoption_gate=paths['adoption'],
        output_dir=output_dir,
    )


def test_production_readiness_passes_with_robot_artifacts(tmp_path: Path):
    paths = _production_artifacts(tmp_path)

    report = Mid360ProductionReadinessGate(
        _inputs(paths, tmp_path / 'out'),
        thresholds=ProductionReadinessThresholds(min_bag_duration_sec=600.0),
    ).build_report()

    assert report['status'] == 'PASS'
    assert report['production_ready'] is True
    assert report['evidence']['estimated_bag_duration_sec'] == 600.0
    assert report['evidence']['adoption_matched_case'] == 'voxel_0p50_min_1p00_dd_on'
    assert all(check['status'] == 'PASS' for check in report['checks'])


def test_production_readiness_fails_for_public_bag_evidence(tmp_path: Path):
    paths = _production_artifacts(
        tmp_path,
        bag_path=str(REPO_ROOT / 'datasets' / 'mid360_public' / 'bag'),
    )

    report = Mid360ProductionReadinessGate(
        _inputs(paths, tmp_path / 'out'),
        thresholds=ProductionReadinessThresholds(min_bag_duration_sec=600.0),
    ).build_report()

    assert report['status'] == 'FAIL'
    assert report['production_ready'] is False
    assert any(check['id'] == 'real_robot_bag' and check['status'] == 'FAIL'
               for check in report['checks'])
    assert any('actual robot field bag' in action for action in report['next_actions'])


def test_production_readiness_fails_when_map_is_not_verified(tmp_path: Path):
    paths = _production_artifacts(tmp_path)
    _write_json(paths['map'], {'status': 'map_saved', 'verify': {'result': 'unknown'}})

    report = Mid360ProductionReadinessGate(
        _inputs(paths, tmp_path / 'out'),
        thresholds=ProductionReadinessThresholds(min_bag_duration_sec=600.0),
    ).build_report()

    assert report['status'] == 'FAIL'
    assert any(check['id'] == 'map_run_verified' and check['status'] == 'FAIL'
               for check in report['checks'])


def test_production_readiness_writes_json_and_markdown(tmp_path: Path):
    paths = _production_artifacts(tmp_path)
    output_dir = tmp_path / 'out'
    report = Mid360ProductionReadinessGate(_inputs(paths, output_dir)).build_report()

    written = write_production_readiness_report(report, output_dir)
    markdown = render_production_readiness_markdown(report)

    assert written['json'] == output_dir / PRODUCTION_READINESS_JSON
    assert written['markdown'] == output_dir / PRODUCTION_READINESS_MARKDOWN
    assert 'production_ready' in markdown
    assert json.loads(written['json'].read_text(encoding='utf-8'))['status'] == 'PASS'


def test_production_readiness_cli_outputs_json(tmp_path: Path):
    paths = _production_artifacts(tmp_path)
    output_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            sys.executable,
            str(PRODUCTION_SCRIPT),
            '--host-readiness',
            str(paths['host']),
            '--recording-check',
            str(paths['recording']),
            '--readiness',
            str(paths['readiness']),
            '--map-diagnosis',
            str(paths['map']),
            '--adoption-gate',
            str(paths['adoption']),
            '--output-dir',
            str(output_dir),
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['status'] == 'PASS'
    assert report['production_ready'] is True
    assert (output_dir / PRODUCTION_READINESS_JSON).is_file()
    assert (output_dir / PRODUCTION_READINESS_MARKDOWN).is_file()
