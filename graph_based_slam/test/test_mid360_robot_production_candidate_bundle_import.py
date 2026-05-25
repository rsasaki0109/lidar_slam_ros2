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

"""Tests for importing and rechecking MID-360 production-candidate bundles."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tarfile

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
EXPORT_SCRIPT = REPO_ROOT / 'scripts' / 'export_mid360_robot_production_candidate_bundle.py'
IMPORT_SCRIPT = REPO_ROOT / 'scripts' / 'import_mid360_robot_production_candidate_bundle.py'


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding='utf-8')


def _write_text(path: Path, text: str = 'ok\n') -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')


def _write_candidate_artifacts(
    root: Path,
    bag_root: Path,
    *,
    run_id: str = 'production_candidate_01',
    pointcloud_messages: int = 6000,
    imu_messages: int = 120000,
) -> None:
    profile = bag_root / f'{run_id}_profile.yaml'
    record_plan_json = bag_root / f'{run_id}_record_plan.json'
    record_plan_md = bag_root / f'{run_id}_record_plan.md'
    _write_text(
        profile,
        yaml.safe_dump({
            'robot_name': 'production_robot',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
            'imu_frame': 'livox_frame',
            'expected_pointcloud_topic': '/livox/lidar',
            'expected_imu_topic': '/livox/imu',
        }),
    )
    _write_json(record_plan_json, {'run_id': run_id, 'bag_path': str(bag_root / run_id)})
    _write_text(record_plan_md, '# Record Plan\n')

    artifact_paths = {
        'profile_snapshot': str(profile),
        'record_plan_json': str(record_plan_json),
        'record_plan_markdown': str(record_plan_md),
        'host_readiness_json': str(root / 'jetson_mid360_host_readiness.json'),
        'recording_check_json': str(root / 'mid360_robot_recording_check.json'),
        'readiness_json': str(root / 'mid360_robot_readiness.json'),
        'map_plan_json': str(root / 'mid360_robot_run_plan.json'),
        'map_diagnosis_json': str(root / 'autoware_map_diagnosis.json'),
        'public_rko_adoption_gate_json': str(
            root / 'public_rko_adoption_gate' / 'mid360_robot_public_rko_adoption_gate.json',
        ),
        'production_readiness_json': str(root / 'mid360_robot_production_readiness.json'),
        'dashboard_html': str(root / 'mid360_robot_session_dashboard.html'),
    }
    _write_json(
        root / 'mid360_robot_production_candidate_session.json',
        {
            'status': 'PASS',
            'run_id': run_id,
            'bag_root': str(bag_root),
            'bag_path': str(bag_root / run_id),
            'profile_snapshot_path': str(profile),
            'record_plan_json_path': str(record_plan_json),
            'record_plan_markdown_path': str(record_plan_md),
            'artifact_paths': artifact_paths,
            'thresholds': {
                'min_bag_duration_sec': 600.0,
                'min_pointcloud_hz': 5.0,
                'min_imu_hz': 50.0,
                'allow_warnings': False,
                'allow_public_bag': False,
            },
        },
    )
    _write_text(root / 'mid360_robot_production_candidate_session.md', '# Candidate\n')
    _write_json(root / 'jetson_mid360_host_readiness.json', {'status': 'PASS'})
    _write_json(
        root / 'mid360_robot_recording_check.json',
        {
            'status': 'PASS',
            'bag_path': str(bag_root / run_id),
            'readiness_status': 'PASS',
            'checks': [{'id': 'readiness_status', 'status': 'ok', 'message': 'passed'}],
        },
    )
    _write_json(
        root / 'mid360_robot_readiness.json',
        {
            'status': 'PASS',
            'bag_path': str(bag_root / run_id),
            'ready_for_mid360_launch': True,
            'selected_topics': {'pointcloud': '/livox/lidar', 'imu': '/livox/imu'},
            'frames': {
                'base_frame': 'base_link',
                'lidar_frame': 'livox_frame',
                'imu_frame': 'livox_frame',
            },
            'checks': [{'id': 'pointcloud2', 'status': 'ok', 'message': 'PointCloud2 topic'}],
            'bag_diagnostics': {
                'topics': {
                    'pointcloud': {
                        'metadata_message_count': pointcloud_messages,
                        'metadata_rate_hz': 10.0,
                        'sampled_message_count': 20,
                        'stable_frame_id': True,
                        'matches_expected_frame': True,
                    },
                    'imu': {
                        'metadata_message_count': imu_messages,
                        'metadata_rate_hz': 200.0,
                        'sampled_message_count': 20,
                        'stable_frame_id': True,
                        'matches_expected_frame': True,
                    },
                }
            },
        },
    )
    _write_json(root / 'mid360_robot_run_plan.json', {'status': 'PASS'})
    _write_json(
        root / 'autoware_map_diagnosis.json',
        {'status': 'success', 'verify': {'result': 'PASS'}},
    )
    _write_json(
        root / 'public_rko_adoption_gate' / 'mid360_robot_public_rko_adoption_gate.json',
        {
            'status': 'PASS',
            'decision': {
                'matched_case': 'voxel_0p50_min_1p00_dd_on',
                'recommended_case': 'voxel_0p50_min_1p00_dd_on',
                'gate_pass_cases': 4,
            },
        },
    )
    _write_json(root / 'mid360_robot_production_readiness.json', {'status': 'PASS'})
    _write_text(root / 'mid360_robot_session_dashboard.html', '<html>dashboard</html>\n')
    _write_json(
        root / 'mid360_robot_public_segment_map_cloud_alignment.json',
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
                'ply': str(root / 'mid360_robot_public_segment_map_cloud_alignment.ply'),
            },
            'checks': [
                {'id': 'median_overlap', 'status': 'PASS', 'message': 'median=0.632m'},
            ],
        },
    )
    _write_text(root / 'mid360_robot_public_segment_map_cloud_alignment.md', '# Alignment\n')
    _write_text(root / 'mid360_robot_public_segment_map_cloud_alignment.ply', 'ply\n')


def _export_bundle(artifact_dir: Path, tarball: Path) -> None:
    subprocess.run(
        [
            sys.executable,
            str(EXPORT_SCRIPT),
            str(artifact_dir),
            '--output',
            str(tarball),
            '--verify',
            '--force',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )


def test_import_bundle_recheck_passes_from_tarball(tmp_path: Path):
    artifact_dir = tmp_path / 'jetson_out'
    bag_root = tmp_path / 'bags'
    tarball = tmp_path / 'production_candidate_01.tar.gz'
    import_dir = tmp_path / 'imported_candidate'
    _write_candidate_artifacts(artifact_dir, bag_root)
    _export_bundle(artifact_dir, tarball)

    result = subprocess.run(
        [
            sys.executable,
            str(IMPORT_SCRIPT),
            str(tarball),
            '--output-dir',
            str(import_dir),
            '--recheck',
            '--verify',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)
    bundle_manifest = json.loads(
        (import_dir / 'mid360_robot_production_candidate_bundle.json').read_text(encoding='utf-8')
    )
    production_path = (
        import_dir / 'artifacts' / 'mid360_robot_production_readiness.json'
    )
    production = json.loads(production_path.read_text(encoding='utf-8'))
    dashboard = (import_dir / 'artifacts' / 'mid360_robot_session_dashboard.html').read_text(
        encoding='utf-8',
    )

    assert report['status'] == 'PASS'
    assert report['verification']['status'] == 'PASS'
    assert report['recheck']['status'] == 'PASS'
    assert production['status'] == 'PASS'
    assert production['production_ready'] is True
    assert (import_dir / 'artifacts' / 'mid360_robot_session_dashboard.html').is_file()
    assert (
        import_dir
        / 'artifacts'
        / 'mid360_robot_public_segment_map_cloud_alignment.json'
    ).is_file()
    assert 'Segment Map Cloud Alignment' in dashboard
    assert 'Aligned median NN' in dashboard
    assert (import_dir / 'mid360_robot_production_candidate_bundle_import.json').is_file()
    assert bundle_manifest['last_import']['status'] == 'PASS'
    assert bundle_manifest['last_import']['recheck_status'] == 'PASS'


def test_import_bundle_recheck_failure_is_reported(tmp_path: Path):
    artifact_dir = tmp_path / 'jetson_out'
    bag_root = tmp_path / 'bags'
    tarball = tmp_path / 'short_candidate.tar.gz'
    import_dir = tmp_path / 'imported_short_candidate'
    _write_candidate_artifacts(
        artifact_dir,
        bag_root,
        pointcloud_messages=3000,
        imu_messages=60000,
    )
    _export_bundle(artifact_dir, tarball)

    result = subprocess.run(
        [
            sys.executable,
            str(IMPORT_SCRIPT),
            str(tarball),
            '--output-dir',
            str(import_dir),
            '--recheck',
            '--verify',
            '--json',
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)
    production_path = (
        import_dir / 'artifacts' / 'mid360_robot_production_readiness.json'
    )
    production = json.loads(production_path.read_text(encoding='utf-8'))

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert report['verification']['status'] == 'PASS'
    assert report['recheck']['status'] == 'FAIL'
    assert production['status'] == 'FAIL'
    assert any(check['id'] == 'bag_duration' and check['status'] == 'FAIL'
               for check in production['checks'])


def test_import_bundle_fails_when_manifest_is_missing(tmp_path: Path):
    bad_root = tmp_path / 'bad_bundle'
    bad_root.mkdir()
    _write_text(bad_root / 'README.txt', 'missing manifest\n')
    tarball = tmp_path / 'bad_bundle.tar.gz'
    with tarfile.open(tarball, 'w:gz') as archive:
        archive.add(bad_root, arcname='bad_bundle')
    import_dir = tmp_path / 'imported_bad'

    result = subprocess.run(
        [
            sys.executable,
            str(IMPORT_SCRIPT),
            str(tarball),
            '--output-dir',
            str(import_dir),
            '--verify',
            '--json',
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert report['manifest_status'] == 'MISSING'
    assert report['verification']['status'] == 'FAIL'
    assert 'missing or unreadable' in report['verification']['errors'][0]
