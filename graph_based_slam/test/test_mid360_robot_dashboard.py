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

"""Tests for the MID-360 robot session dashboard."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'generate_mid360_robot_session_dashboard.py'


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding='utf-8')


def _write_dashboard_fixture(output_dir: Path) -> None:
    output_dir.mkdir()
    _write_json(
        output_dir / 'mid360_robot_field_session.json',
        {
            'status': 'PASS',
            'run_id': 'stand_01',
            'created_at': '2026-05-21T00:00:00+00:00',
            'bag_path': '/bags/stand_01',
            'output_dir': str(output_dir),
            'steps': [
                {
                    'id': 'recording',
                    'status': 'ok',
                    'message': 'Recording completed.',
                    'command': ['ros2', 'bag', 'record', '/livox/lidar'],
                },
                {
                    'id': 'map',
                    'status': 'planned',
                    'message': 'Map dry-run planned.',
                    'command': ['bash', 'scripts/run_mid360_robot_map.sh'],
                },
            ],
        },
    )
    _write_json(
        output_dir / 'mid360_robot_recording_check.json',
        {
            'status': 'PASS',
            'checks': [
                {'id': 'metadata_yaml', 'status': 'ok', 'message': 'metadata.yaml exists'},
            ],
        },
    )
    _write_json(
        output_dir / 'mid360_robot_readiness.json',
        {
            'status': 'PASS',
            'selected_topics': {'pointcloud': '/livox/lidar', 'imu': '/livox/imu'},
            'frames': {
                'base_frame': 'base_link',
                'lidar_frame': 'livox_frame',
                'imu_frame': 'livox_frame',
            },
            'bag_diagnostics': {
                'topics': {
                    'pointcloud': {'metadata_rate_hz': 10.0},
                    'imu': {'metadata_rate_hz': 100.0},
                }
            },
            'checks': [
                {'id': 'pointcloud2', 'status': 'ok', 'message': 'PointCloud2 topic'},
            ],
        },
    )
    _write_json(
        output_dir / 'mid360_robot_run_plan.json',
        {
            'status': 'PASS',
            'dogfood_command_shell': 'bash scripts/run_rko_lio_graph_autoware_dogfood.sh',
        },
    )
    (output_dir / 'mid360_robot_3d_map_preview.html').write_text(
        '<html>preview</html>\n',
        encoding='utf-8',
    )
    _write_json(
        output_dir / 'mid360_robot_3d_map_preview.json',
        {
            'status': 'PASS',
            'pointcloud_map_dir': '/maps/pointcloud_map',
            'artifacts': {
                'html': str(output_dir / 'mid360_robot_3d_map_preview.html'),
                'ply': str(output_dir / 'mid360_robot_3d_map_preview.ply'),
                'overlay_json': str(output_dir / 'mid360_robot_3d_map_preview_overlay.json'),
            },
            'counts': {
                'cloud_points': 1000,
                'html_points': 500,
                'trajectory_poses': 42,
                'loop_candidates': 2,
            },
        },
    )
    _write_json(
        output_dir / 'mid360_robot_public_segment_map_cloud_alignment.json',
        {
            'status': 'PASS',
            'clouds': {
                'start': {'analysis_points': 4525},
                'end': {'analysis_points': 7291},
            },
            'crop': {'crop_radius_m': 20.0},
            'aligned_overlap': {
                'symmetric_median_nn_m': 0.632,
                'symmetric_p90_nn_m': 2.107,
                'coverage_within_1m': 0.690,
            },
            'transform_start_to_end': {
                'translation_norm_m': 8.54,
                'yaw_deg': 73.12,
            },
            'artifacts': {
                'ply': str(output_dir / 'mid360_robot_public_segment_map_cloud_alignment.ply'),
            },
            'checks': [
                {'id': 'median_overlap', 'status': 'PASS', 'message': 'median=0.632m'},
            ],
        },
    )
    (output_dir / 'mid360_robot_public_segment_map_cloud_alignment.ply').write_text(
        'ply\n',
        encoding='utf-8',
    )


def test_dashboard_cli_generates_static_html(tmp_path: Path):
    output_dir = tmp_path / 'session'
    _write_dashboard_fixture(output_dir)

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(output_dir)],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    html_path = output_dir / 'mid360_robot_session_dashboard.html'
    html = html_path.read_text(encoding='utf-8')

    assert 'MID-360 session dashboard:' in result.stdout
    assert html_path.is_file()
    assert 'MID-360 Robot Session' in html
    assert 'Route Sketch' in html
    assert '3D Map Preview' in html
    assert 'Segment Map Cloud Alignment' in html
    assert 'Aligned median NN' in html
    assert 'median_overlap' in html
    assert str(output_dir / 'mid360_robot_public_segment_map_cloud_alignment.ply') in html
    assert 'Open 3D map preview' in html
    assert 'mid360_robot_3d_map_preview.html' in html
    assert 'Browser preview points' in html
    assert 'Record Bag' in html
    assert 'Post Check' in html
    assert 'Map Dry-Run' in html
    assert 'Map Run' in html
    assert 'route-node ok' in html
    assert 'route-node planned' in html
    assert 'stand_01' in html
    assert '/livox/lidar' in html
    assert '10.00 Hz' in html
    assert 'metadata_yaml' in html
    assert 'bash scripts/run_rko_lio_graph_autoware_dogfood.sh' in html


def test_dashboard_optional_segment_alignment_does_not_set_overall_status(tmp_path: Path):
    output_dir = tmp_path / 'session'
    _write_dashboard_fixture(output_dir)
    _write_json(
        output_dir / 'mid360_robot_public_segment_map_cloud_alignment.json',
        {
            'status': 'FAIL',
            'clouds': {
                'start': {'analysis_points': 4525},
                'end': {'analysis_points': 7291},
            },
            'crop': {'crop_radius_m': 20.0},
            'aligned_overlap': {
                'symmetric_median_nn_m': 3.2,
                'symmetric_p90_nn_m': 8.0,
                'coverage_within_1m': 0.1,
            },
            'checks': [
                {'id': 'median_overlap', 'status': 'FAIL', 'message': 'median=3.2m'},
            ],
        },
    )

    subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(output_dir)],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    html = (output_dir / 'mid360_robot_session_dashboard.html').read_text(encoding='utf-8')

    assert '<div class="status pass">PASS</div>' in html
    assert 'Segment Map Cloud Alignment' in html
    assert 'median=3.2m' in html


def test_dashboard_marks_missing_artifacts(tmp_path: Path):
    output_dir = tmp_path / 'session'
    output_dir.mkdir()
    _write_json(
        output_dir / 'mid360_robot_field_session.json',
        {'status': 'PASS', 'run_id': 'missing_case', 'steps': []},
    )

    subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(output_dir)],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    html = (output_dir / 'mid360_robot_session_dashboard.html').read_text(encoding='utf-8')

    assert 'MISSING' in html
    assert 'route-node missing' in html
    assert 'mid360_robot_readiness.json' in html
    assert 'Create missing artifacts' in html


def test_dashboard_prefers_production_candidate_session(tmp_path: Path):
    output_dir = tmp_path / 'session'
    output_dir.mkdir()
    public_gate_dir = output_dir / 'public_rko_adoption_gate'
    _write_json(
        output_dir / 'mid360_robot_field_session.json',
        {
            'status': 'PASS',
            'run_id': 'field_ignored',
            'bag_path': '/bags/field_ignored',
            'steps': [],
        },
    )
    _write_json(
        output_dir / 'mid360_robot_production_candidate_session.json',
        {
            'status': 'PASS',
            'run_id': 'production_candidate_01',
            'created_at': '2026-05-22T00:00:00+00:00',
            'bag_path': '/bags/production_candidate_01',
            'artifact_paths': {
                'map_diagnosis_json': str(output_dir / 'autoware_map_diagnosis.json'),
                'public_rko_adoption_gate_json': str(  # noqa: E501
                    public_gate_dir / 'mid360_robot_public_rko_adoption_gate.json',
                ),
                'production_readiness_json': str(
                    output_dir / 'mid360_robot_production_readiness.json',
                ),
            },
            'steps': [
                {
                    'id': 'host_readiness',
                    'status': 'ok',
                    'message': 'Host readiness completed.',
                    'command': ['python3', 'scripts/check_jetson_mid360_host_readiness.py'],
                },
                {
                    'id': 'recording',
                    'status': 'ok',
                    'message': 'Recording completed.',
                    'command': ['bash', 'scripts/record_mid360_robot_bag.sh'],
                },
                {
                    'id': 'map',
                    'status': 'ok',
                    'message': 'Mapping completed.',
                    'command': ['bash', 'scripts/run_mid360_robot_map.sh'],
                },
                {
                    'id': 'public_rko_adoption_gate',
                    'status': 'ok',
                    'message': 'Public RKO adoption gate completed.',
                    'command': ['python3', 'scripts/run_mid360_robot_public_rko_adoption_gate.py'],
                },
                {
                    'id': 'production_readiness',
                    'status': 'fail',
                    'message': 'Production readiness failed.',
                    'command': ['python3', 'scripts/check_mid360_robot_production_readiness.py'],
                },
            ],
        },
    )
    _write_json(
        output_dir / 'mid360_robot_recording_check.json',
        {
            'status': 'PASS',
            'checks': [{'id': 'readiness_status', 'status': 'ok', 'message': 'passed'}],
        },
    )
    _write_json(
        output_dir / 'mid360_robot_readiness.json',
        {
            'status': 'PASS',
            'selected_topics': {'pointcloud': '/livox/lidar', 'imu': '/livox/imu'},
            'frames': {
                'base_frame': 'base_link',
                'lidar_frame': 'livox_frame',
                'imu_frame': 'livox_frame',
            },
            'bag_diagnostics': {
                'topics': {
                    'pointcloud': {'metadata_rate_hz': 10.0},
                    'imu': {'metadata_rate_hz': 100.0},
                }
            },
            'checks': [{'id': 'pointcloud2', 'status': 'ok', 'message': 'PointCloud2 topic'}],
        },
    )
    _write_json(output_dir / 'mid360_robot_run_plan.json', {'status': 'PASS'})
    _write_json(
        output_dir / 'autoware_map_diagnosis.json',
        {'status': 'success', 'verify': {'result': 'PASS'}},
    )
    _write_json(
        public_gate_dir / 'mid360_robot_public_rko_adoption_gate.json',
        {
            'status': 'PASS',
            'checks': [{'id': 'matched_config', 'status': 'PASS', 'message': 'matched'}],
        },
    )
    _write_json(
        output_dir / 'mid360_robot_production_readiness.json',
        {
            'status': 'FAIL',
            'checks': [{'id': 'bag_duration', 'status': 'FAIL', 'message': 'too short'}],
            'next_actions': ['Record a longer production candidate bag.'],
        },
    )

    subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(output_dir)],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    html = (output_dir / 'mid360_robot_session_dashboard.html').read_text(encoding='utf-8')

    assert 'MID-360 Robot Production Candidate' in html
    assert 'production_candidate_01' in html
    assert 'field_ignored' not in html
    assert 'Host Check' in html
    assert 'Public Gate' in html
    assert 'Prod Gate' in html
    assert 'mid360_robot_public_rko_adoption_gate.json' in html
    assert 'mid360_robot_production_readiness.json' in html
    assert 'bag_duration' in html
    assert 'Record a longer production candidate bag.' in html
    assert 'route-node fail' in html


def test_dashboard_renders_loop_alignment_section(tmp_path: Path):
    output_dir = tmp_path / 'session'
    _write_dashboard_fixture(output_dir)
    _write_json(
        output_dir / 'mid360_robot_loop_alignment.json',
        {
            'status': 'WARN',
            'trajectory': {
                'poses': 581,
                'path_length_m': 1086.3,
                'duration_sec': 277.4,
            },
            'cloud': {
                'sampled_points': 104830,
                'tile_count': 368,
            },
            'nearest_revisit': {'distance_m': 4.25},
            'loop_candidates': [{'distance_m': 4.25}, {'distance_m': 4.30}, {'distance_m': 4.35}],
            'local_cloud_checks': [
                {'status': 'PASS', 'largest_component_ratio': 0.72},
                {'status': 'FAIL', 'largest_component_ratio': 0.55},
                {'status': 'PASS', 'largest_component_ratio': 0.81},
            ],
            'thresholds': {
                'max_loop_distance_m': 5.0,
                'min_largest_component_ratio': 0.6,
            },
            'checks': [
                {
                    'id': 'loop_alignment_local_cloud',
                    'status': 'WARN',
                    'message': 'mixed connectivity',
                },
            ],
            'next_actions': ['Inspect the local loop cloud in CloudCompare.'],
        },
    )

    subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(output_dir)],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    html = (output_dir / 'mid360_robot_session_dashboard.html').read_text(encoding='utf-8')

    assert 'Loop Alignment' in html
    assert 'Nearest revisit' in html
    # nearest_revisit_m=4.25 rendered with threshold annotation
    assert '4.25 m (≤ 5.0 m)' in html
    # best ratio is max of [0.72, 0.55, 0.81] = 0.810, with threshold annotation
    assert '0.810 (≥ 0.60)' in html
    # local cloud check summary "2 PASS / 1 FAIL / 3 total"
    assert '2 PASS / 1 FAIL / 3 total' in html
    # loop_alignment check propagated into Checks table
    assert 'loop_alignment_local_cloud' in html
    # mixed connectivity message present
    assert 'mixed connectivity' in html
    # next_actions surfaced
    assert 'Inspect the local loop cloud in CloudCompare.' in html


def test_dashboard_omits_loop_alignment_when_absent(tmp_path: Path):
    output_dir = tmp_path / 'session'
    _write_dashboard_fixture(output_dir)

    subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(output_dir)],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    html = (output_dir / 'mid360_robot_session_dashboard.html').read_text(encoding='utf-8')

    # Section header is always present, but the placeholder fires when no artifact
    assert 'Loop Alignment' in html
    assert 'No loop alignment analyzer artifact found.' in html


def test_dashboard_renders_continuous_relocalization_gate(tmp_path: Path):
    output_dir = tmp_path / 'session'
    _write_dashboard_fixture(output_dir)
    _write_json(
        output_dir / 'mid360_robot_public_continuous_relocalization_gate.json',
        {
            'status': 'PASS',
            'completion_ready': True,
            'scope': 'public_mid360_continuous_rko_lio_kidnap_relocalization',
            'evidence': {
                'trajectory': {'poses': 2896, 'duration_sec': 553.8},
                'recovery': {
                    'relocalization_events': 1,
                    'recovery_accept_events': 1,
                    'dropped_scan_events': 1121,
                },
                'loop_alignment': {
                    'loop_candidates': 20,
                    'nearest_revisit_distance_m': 0.162,
                    'max_loop_distance_m': 0.181,
                },
                'autoware_map_verify': {'status': 'PASS'},
                'config': {'matches_tracked_config': True},
            },
            'checks': [
                {
                    'id': 'kidnap_relocalization_event_present',
                    'status': 'PASS',
                    'message': 'relocalized',
                },
            ],
        },
    )

    subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(output_dir)],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    html = (output_dir / 'mid360_robot_session_dashboard.html').read_text(encoding='utf-8')

    assert 'Continuous Relocalization Gate' in html
    assert 'public_mid360_continuous_rko_lio_kidnap_relocalization' in html
    assert 'Dropped invalid scans' in html
    assert '1121' in html
    assert 'kidnap_relocalization_event_present' in html
