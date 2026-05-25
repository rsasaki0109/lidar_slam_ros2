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

"""Tests for MID-360 production-candidate artifact bundle export."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tarfile


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'export_mid360_robot_production_candidate_bundle.py'


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
    omit_map_diagnosis: bool = False,
) -> None:
    run_id = 'production_candidate_01'
    profile = bag_root / f'{run_id}_profile.yaml'
    record_plan_json = bag_root / f'{run_id}_record_plan.json'
    record_plan_md = bag_root / f'{run_id}_record_plan.md'
    _write_text(profile, 'robot_name: production_robot\n')
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
            'thresholds': {'min_bag_duration_sec': 600.0},
        },
    )
    _write_text(root / 'mid360_robot_production_candidate_session.md', '# Candidate\n')
    _write_json(root / 'jetson_mid360_host_readiness.json', {'status': 'PASS'})
    _write_json(root / 'mid360_robot_recording_check.json', {'status': 'PASS'})
    _write_json(root / 'mid360_robot_readiness.json', {'status': 'PASS'})
    _write_json(root / 'mid360_robot_run_plan.json', {'status': 'PASS'})
    if not omit_map_diagnosis:
        _write_json(
            root / 'autoware_map_diagnosis.json',
            {'status': 'success', 'verify': {'result': 'PASS'}},
        )
    _write_json(
        root / 'public_rko_adoption_gate' / 'mid360_robot_public_rko_adoption_gate.json',
        {'status': 'PASS'},
    )
    _write_json(root / 'mid360_robot_production_readiness.json', {'status': 'PASS'})
    _write_text(root / 'mid360_robot_session_dashboard.html', '<html>dashboard</html>\n')


def test_export_bundle_writes_manifest_and_tarball(tmp_path: Path):
    artifact_dir = tmp_path / 'out'
    bag_root = tmp_path / 'bags'
    _write_candidate_artifacts(artifact_dir, bag_root)
    tarball = tmp_path / 'production_candidate_01.tar.gz'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(artifact_dir),
            '--output',
            str(tarball),
            '--label',
            'candidate_bundle',
            '--verify',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    manifest = json.loads(result.stdout)

    assert manifest['status'] == 'PASS'
    assert manifest['bundle_label'] == 'candidate_bundle'
    assert manifest['tarball_verified'] is True
    assert tarball.is_file()
    bundle_json = (
        tmp_path / 'production_candidate_01' / 'mid360_robot_production_candidate_bundle.json'
    )
    assert bundle_json.is_file()
    assert 'artifacts/autoware_map_diagnosis.json' in manifest['required_files']
    assert 'recording/production_candidate_01_profile.yaml' in manifest['required_files']
    assert '--from-existing-artifacts' in manifest['recheck_command']

    with tarfile.open(tarball, 'r:gz') as archive:
        names = set(archive.getnames())
    root = 'production_candidate_01'
    assert f'{root}/mid360_robot_production_candidate_bundle.json' in names
    assert f'{root}/artifacts/mid360_robot_production_candidate_session.json' in names
    assert (
        f'{root}/artifacts/public_rko_adoption_gate/'
        'mid360_robot_public_rko_adoption_gate.json'
    ) in names
    assert f'{root}/recording/production_candidate_01_profile.yaml' in names


def test_export_bundle_includes_optional_loop_alignment_when_present(tmp_path: Path):
    artifact_dir = tmp_path / 'out'
    bag_root = tmp_path / 'bags'
    _write_candidate_artifacts(artifact_dir, bag_root)
    _write_json(
        artifact_dir / 'mid360_robot_loop_alignment.json',
        {
            'status': 'WARN',
            'trajectory': {'poses': 500},
            'nearest_revisit': {'distance_m': 4.0},
            'loop_candidates': [],
            'local_cloud_checks': [],
            'checks': [],
        },
    )
    _write_text(artifact_dir / 'mid360_robot_loop_alignment.md', '# Loop\n')
    _write_json(
        artifact_dir / 'mid360_robot_3d_map_preview.json',
        {
            'status': 'PASS',
            'artifacts': {
                'html': str(artifact_dir / 'mid360_robot_3d_map_preview.html'),
                'ply': str(artifact_dir / 'mid360_robot_3d_map_preview.ply'),
                'overlay_json': str(
                    artifact_dir / 'mid360_robot_3d_map_preview_overlay.json',
                ),
            },
            'counts': {'cloud_points': 1000, 'html_points': 500},
        },
    )
    _write_text(artifact_dir / 'mid360_robot_3d_map_preview.html', '<html>preview</html>\n')
    _write_text(artifact_dir / 'mid360_robot_3d_map_preview.ply', 'ply\n')
    _write_json(artifact_dir / 'mid360_robot_3d_map_preview_overlay.json', {'trajectory': []})
    _write_json(
        artifact_dir / 'mid360_robot_public_segment_map_cloud_alignment.json',
        {
            'status': 'PASS',
            'aligned_overlap': {
                'symmetric_median_nn_m': 0.632,
                'symmetric_p90_nn_m': 2.107,
                'coverage_within_1m': 0.690,
            },
            'checks': [],
        },
    )
    _write_text(
        artifact_dir / 'mid360_robot_public_segment_map_cloud_alignment.md',
        '# Alignment\n',
    )
    _write_text(artifact_dir / 'mid360_robot_public_segment_map_cloud_alignment.ply', 'ply\n')

    tarball = tmp_path / 'production_candidate_01.tar.gz'
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(artifact_dir),
            '--output',
            str(tarball),
            '--verify',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    manifest = json.loads(result.stdout)

    # Loop alignment is optional but should be staged when present.
    assert manifest['status'] == 'PASS'
    assert manifest['tarball_verified'] is True
    included_dests = {item['destination'] for item in manifest['included']}
    assert 'artifacts/mid360_robot_loop_alignment.json' in included_dests
    assert 'artifacts/mid360_robot_loop_alignment.md' in included_dests
    assert 'artifacts/mid360_robot_3d_map_preview.json' in included_dests
    assert 'artifacts/mid360_robot_3d_map_preview.html' in included_dests
    assert 'artifacts/mid360_robot_3d_map_preview.ply' in included_dests
    assert 'artifacts/mid360_robot_3d_map_preview_overlay.json' in included_dests
    assert 'artifacts/mid360_robot_public_segment_map_cloud_alignment.json' in included_dests
    assert 'artifacts/mid360_robot_public_segment_map_cloud_alignment.md' in included_dests
    assert 'artifacts/mid360_robot_public_segment_map_cloud_alignment.ply' in included_dests
    # Optional artifacts must NOT appear in required_files so a missing analyzer
    # cannot fail verify on its own.
    assert 'artifacts/mid360_robot_loop_alignment.json' not in manifest['required_files']
    assert 'artifacts/mid360_robot_3d_map_preview.json' not in manifest['required_files']
    assert (
        'artifacts/mid360_robot_public_segment_map_cloud_alignment.json'
        not in manifest['required_files']
    )

    with tarfile.open(tarball, 'r:gz') as archive:
        names = set(archive.getnames())
    root = 'production_candidate_01'
    assert f'{root}/artifacts/mid360_robot_loop_alignment.json' in names
    assert f'{root}/artifacts/mid360_robot_loop_alignment.md' in names
    assert f'{root}/artifacts/mid360_robot_3d_map_preview.html' in names
    assert f'{root}/artifacts/mid360_robot_public_segment_map_cloud_alignment.json' in names


def test_export_bundle_skips_loop_alignment_when_absent(tmp_path: Path):
    artifact_dir = tmp_path / 'out'
    bag_root = tmp_path / 'bags'
    _write_candidate_artifacts(artifact_dir, bag_root)
    tarball = tmp_path / 'production_candidate_01.tar.gz'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(artifact_dir),
            '--output',
            str(tarball),
            '--verify',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    manifest = json.loads(result.stdout)

    # No loop_alignment present -> still PASS (optional artifact)
    assert manifest['status'] == 'PASS'
    assert manifest['tarball_verified'] is True
    included_dests = {item['destination'] for item in manifest['included']}
    assert 'artifacts/mid360_robot_loop_alignment.json' not in included_dests
    assert 'artifacts/mid360_robot_3d_map_preview.json' not in included_dests
    assert 'artifacts/mid360_robot_public_segment_map_cloud_alignment.json' not in included_dests
    # Not in missing_required either (because it's not required)
    missing_dests = {item['destination'] for item in manifest['missing_required']}
    assert 'artifacts/mid360_robot_loop_alignment.json' not in missing_dests
    assert 'artifacts/mid360_robot_3d_map_preview.json' not in missing_dests
    assert 'artifacts/mid360_robot_public_segment_map_cloud_alignment.json' not in missing_dests


def test_export_bundle_verify_fails_when_required_artifact_missing(tmp_path: Path):
    artifact_dir = tmp_path / 'out'
    bag_root = tmp_path / 'bags'
    _write_candidate_artifacts(artifact_dir, bag_root, omit_map_diagnosis=True)
    tarball = tmp_path / 'missing_map.tar.gz'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(artifact_dir),
            '--output',
            str(tarball),
            '--verify',
            '--json',
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    manifest = json.loads(result.stdout)

    assert result.returncode == 1
    assert manifest['status'] == 'FAIL'
    assert any(
        item['destination'] == 'artifacts/autoware_map_diagnosis.json'
        for item in manifest['missing_required']
    )
    assert tarball.is_file()
