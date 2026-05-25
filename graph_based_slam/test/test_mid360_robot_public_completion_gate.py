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

"""Tests for public MID-360 completion gate."""

from __future__ import annotations

import json
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_completion_gate import (  # noqa: E402
    PUBLIC_COMPLETION_GATE_JSON,
    PUBLIC_COMPLETION_GATE_MARKDOWN,
    PublicCompletionGate,
    PublicCompletionGateOptions,
)


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding='utf-8')


def _write_minimal_map(run_dir: Path, *, tum_poses: int) -> None:
    pointcloud_map = run_dir / 'pointcloud_map'
    pointcloud_map.mkdir(parents=True, exist_ok=True)
    (run_dir / 'map_projector_info.yaml').write_text(
        'projector_type: local\n',
        encoding='utf-8',
    )
    (pointcloud_map / 'pointcloud_map_metadata.yaml').write_text(
        'x_resolution: 20\n'
        'y_resolution: 20\n'
        '0_0.pcd: [0, 0]\n',
        encoding='utf-8',
    )
    (pointcloud_map / '0_0.pcd').write_text(
        '# .PCD v0.7\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        'WIDTH 1\n'
        'HEIGHT 1\n'
        'POINTS 1\n'
        'DATA ascii\n'
        '1.0 2.0 0.0\n',
        encoding='ascii',
    )
    tum = run_dir / 'fake_tum_0.txt'
    tum.write_text(
        ''.join(f'{float(index):.1f} {index} 0 0 0 0 0 1\n' for index in range(tum_poses)),
        encoding='ascii',
    )
    (run_dir / 'map_save.log').write_text('response\n', encoding='utf-8')


def _write_entrypoints(root: Path) -> None:
    for relative in (
        'scripts/run_mid360_robot_production_candidate_session.sh',
        'scripts/generate_mid360_robot_session_dashboard.py',
        'scripts/export_mid360_robot_production_candidate_bundle.py',
        'scripts/import_mid360_robot_production_candidate_bundle.py',
        'scripts/run_release_readiness_checks.sh',
        'scripts/run_mid360_robot_public_completion_gate.py',
    ):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('#!/usr/bin/env bash\n', encoding='utf-8')


def test_public_completion_gate_passes_with_required_artifacts(tmp_path: Path):
    repo_root = tmp_path / 'repo'
    _write_entrypoints(repo_root)
    output_dir = tmp_path / 'completion'
    start_run = tmp_path / 'segment_000'
    end_run = tmp_path / 'segment_012'
    _write_minimal_map(start_run, tum_poses=60)
    _write_minimal_map(end_run, tum_poses=70)
    dashboard = tmp_path / 'dashboard.html'
    dashboard.write_text('<html>dashboard</html>\n', encoding='utf-8')

    loop_cloud = tmp_path / 'loop_cloud.json'
    _write_json(loop_cloud, {'status': 'PASS', 'overlap': {'symmetric_median_nn_m': 0.2}})
    segment_plan = tmp_path / 'segment_plan.json'
    _write_json(
        segment_plan,
        {
            'status': 'PASS',
            'reset_pair': {
                'start': {'status': 'PASS', 'segment': {'segment_id': 'segment_000'}},
                'end': {'status': 'PASS', 'segment': {'segment_id': 'segment_012'}},
            },
        },
    )
    alignment = tmp_path / 'alignment.json'
    _write_json(
        alignment,
        {
            'status': 'PASS',
            'aligned_overlap': {
                'symmetric_median_nn_m': 0.632,
                'symmetric_p90_nn_m': 2.107,
                'coverage_within_1m': 0.690,
            },
        },
    )
    adoption = tmp_path / 'adoption.json'
    _write_json(
        adoption,
        {
            'status': 'PASS',
            'decision': {
                'matched_case': 'voxel_0p50_min_1p00_dd_on',
                'recommended_case': 'voxel_0p50_min_1p00_dd_on',
                'gate_pass_cases': 4,
            },
        },
    )

    report = PublicCompletionGate().build_report(
        PublicCompletionGateOptions(
            repo_root=repo_root,
            output_dir=output_dir,
            loop_cloud_json=loop_cloud,
            segment_reset_plan_json=segment_plan,
            start_run_dir=start_run,
            end_run_dir=end_run,
            segment_map_alignment_json=alignment,
            adoption_gate_json=adoption,
            dashboard_html=dashboard,
            min_segment_rko_poses=50,
        )
    )

    assert report['status'] == 'PASS'
    assert report['completion_ready'] is True
    assert report['counts']['fail'] == 0
    assert (output_dir / PUBLIC_COMPLETION_GATE_JSON).is_file()
    assert (output_dir / PUBLIC_COMPLETION_GATE_MARKDOWN).is_file()


def test_public_completion_gate_fails_when_segment_trajectory_is_too_short(tmp_path: Path):
    repo_root = tmp_path / 'repo'
    _write_entrypoints(repo_root)
    start_run = tmp_path / 'segment_000'
    end_run = tmp_path / 'segment_012'
    _write_minimal_map(start_run, tum_poses=10)
    _write_minimal_map(end_run, tum_poses=70)
    dashboard = tmp_path / 'dashboard.html'
    dashboard.write_text('<html>dashboard</html>\n', encoding='utf-8')
    loop_cloud = tmp_path / 'loop_cloud.json'
    segment_plan = tmp_path / 'segment_plan.json'
    alignment = tmp_path / 'alignment.json'
    adoption = tmp_path / 'adoption.json'
    _write_json(loop_cloud, {'status': 'PASS'})
    _write_json(
        segment_plan,
        {
            'status': 'PASS',
            'reset_pair': {
                'start': {'status': 'PASS', 'segment': {'segment_id': 'segment_000'}},
                'end': {'status': 'PASS', 'segment': {'segment_id': 'segment_012'}},
            },
        },
    )
    _write_json(alignment, {'status': 'PASS', 'aligned_overlap': {}})
    _write_json(
        adoption,
        {
            'status': 'PASS',
            'decision': {
                'matched_case': 'voxel_0p50_min_1p00_dd_on',
                'recommended_case': 'voxel_0p50_min_1p00_dd_on',
                'gate_pass_cases': 4,
            },
        },
    )

    report = PublicCompletionGate().build_report(
        PublicCompletionGateOptions(
            repo_root=repo_root,
            output_dir=tmp_path / 'completion',
            loop_cloud_json=loop_cloud,
            segment_reset_plan_json=segment_plan,
            start_run_dir=start_run,
            end_run_dir=end_run,
            segment_map_alignment_json=alignment,
            adoption_gate_json=adoption,
            dashboard_html=dashboard,
            min_segment_rko_poses=50,
        )
    )

    assert report['status'] == 'FAIL'
    assert any(
        check['id'] == 'start_segment_rko_complete' and check['status'] == 'FAIL'
        for check in report['checks']
    )


def test_public_completion_gate_fails_when_tracked_config_is_not_top_case(tmp_path: Path):
    repo_root = tmp_path / 'repo'
    _write_entrypoints(repo_root)
    start_run = tmp_path / 'segment_000'
    end_run = tmp_path / 'segment_012'
    _write_minimal_map(start_run, tum_poses=60)
    _write_minimal_map(end_run, tum_poses=70)
    dashboard = tmp_path / 'dashboard.html'
    dashboard.write_text('<html>dashboard</html>\n', encoding='utf-8')
    loop_cloud = tmp_path / 'loop_cloud.json'
    segment_plan = tmp_path / 'segment_plan.json'
    alignment = tmp_path / 'alignment.json'
    adoption = tmp_path / 'adoption.json'
    _write_json(loop_cloud, {'status': 'PASS'})
    _write_json(
        segment_plan,
        {
            'status': 'PASS',
            'reset_pair': {
                'start': {'status': 'PASS', 'segment': {'segment_id': 'segment_000'}},
                'end': {'status': 'PASS', 'segment': {'segment_id': 'segment_012'}},
            },
        },
    )
    _write_json(alignment, {'status': 'PASS', 'aligned_overlap': {}})
    _write_json(
        adoption,
        {
            'status': 'PASS',
            'decision': {
                'matched_case': 'case_b',
                'recommended_case': 'case_a',
                'gate_pass_cases': 2,
            },
        },
    )

    report = PublicCompletionGate().build_report(
        PublicCompletionGateOptions(
            repo_root=repo_root,
            output_dir=tmp_path / 'completion',
            loop_cloud_json=loop_cloud,
            segment_reset_plan_json=segment_plan,
            start_run_dir=start_run,
            end_run_dir=end_run,
            segment_map_alignment_json=alignment,
            adoption_gate_json=adoption,
            dashboard_html=dashboard,
            min_segment_rko_poses=50,
        )
    )

    assert report['status'] == 'FAIL'
    assert any(
        check['id'] == 'tracked_config_matches_top_gate' and check['status'] == 'FAIL'
        for check in report['checks']
    )
