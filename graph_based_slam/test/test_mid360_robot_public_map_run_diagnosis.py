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

"""Tests for public MID-360 map-run diagnosis."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
DIAGNOSE_SCRIPT = SCRIPT_DIR / 'diagnose_mid360_robot_public_map_run.py'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_map_run_diagnosis import (  # noqa: E402
    PUBLIC_MAP_RUN_DIAGNOSIS_JSON,
    PUBLIC_MAP_RUN_DIAGNOSIS_MARKDOWN,
    PublicDatasetMapRunDiagnosisBuilder,
    render_public_map_run_diagnosis_markdown,
)


def _candidate(output_dir: Path) -> dict:
    return {
        'dataset_id': 'hard_pointcloud_mid360_outdoor_kidnap_a',
        'title': 'Hard public dataset',
        'selected_bag_path': str(output_dir.parent / 'bag'),
        'output_dir': str(output_dir),
        'safety': {'output_dir': str(output_dir)},
    }


def _write_failed_run(output_dir: Path) -> None:
    output_dir.mkdir(parents=True)
    (output_dir / 'slam.launch.log').write_text(
        'RKO LIO Node is up!\n'
        '[graph_based_slam]: initialization end\n'
        'First cloud received, 158304 bytes\n'
        'First odom received: (-0.10, -0.74, -0.04)\n'
        "Failed to find match for field 'intensity'.\n"
        'Bag reader initialized with total message count: 42143\n'
        'Error: Keypoints for ICP registration = 1, this is too little for ICP\n'
        'Error: Received LiDAR scan with 1.400104 seconds delta to previous scan.\n'
        'Error: Received LiDAR scan with 2.500035 seconds delta to previous scan.\n',
        encoding='utf-8',
    )
    trajectory_dir = output_dir / 'hard_pointcloud_mid360_outdoor_kidnap_a_0'
    trajectory_dir.mkdir()
    (trajectory_dir / 'hard_pointcloud_mid360_outdoor_kidnap_a_tum_0.txt').write_text(
        '0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n',
        encoding='utf-8',
    )


def _manifest(output_dir: Path) -> dict:
    return {
        'status': 'FAIL',
        'candidates': [_candidate(output_dir)],
        'runs': [
            {
                'dataset_id': 'hard_pointcloud_mid360_outdoor_kidnap_a',
                'returncode': 124,
                'timed_out': True,
                'timeout_sec': 45,
                'duration_sec': 55.0,
                'stdout': '',
                'stderr': 'Timed out waiting for offline completion or quiescent map outputs',
            }
        ],
    }


def test_builder_summarizes_failed_public_run(tmp_path: Path):
    run_dir = tmp_path / 'out' / 'hard'
    _write_failed_run(run_dir)
    manifest_path = tmp_path / 'manifest.json'
    manifest_path.write_text(json.dumps(_manifest(run_dir)), encoding='utf-8')

    builder = PublicDatasetMapRunDiagnosisBuilder(
        manifest_path=manifest_path,
        output_dir=tmp_path / 'diagnosis',
    )
    report = builder.build()
    markdown = render_public_map_run_diagnosis_markdown(report)

    row = report['datasets'][0]
    assert report['status'] == 'FAIL'
    assert row['status'] == 'FAIL'
    assert row['run_result']['timed_out'] is True
    assert row['runtime']['rko_started'] is True
    assert row['runtime']['graph_initialized'] is True
    assert row['runtime']['lidar_delta_error_count'] == 2
    assert row['runtime']['lidar_delta_max_sec'] == 2.500035
    assert row['runtime']['keypoints_too_few_count'] == 1
    assert row['outputs']['trajectory']['total_lines'] == 2
    assert row['outputs']['map_saved'] is False
    assert 'scan timestamp deltas' in '\n'.join(row['problem_hints'])
    assert 'hard_pointcloud_mid360_outdoor_kidnap_a' in markdown
    assert 'LiDAR Delta Errors' in markdown


def test_builder_writes_json_and_markdown(tmp_path: Path):
    run_dir = tmp_path / 'out' / 'hard'
    _write_failed_run(run_dir)
    manifest_path = tmp_path / 'manifest.json'
    manifest_path.write_text(json.dumps(_manifest(run_dir)), encoding='utf-8')

    builder = PublicDatasetMapRunDiagnosisBuilder(
        manifest_path=manifest_path,
        output_dir=tmp_path / 'diagnosis',
    )
    paths = builder.write(builder.build())

    assert paths['json'] == tmp_path / 'diagnosis' / PUBLIC_MAP_RUN_DIAGNOSIS_JSON
    assert paths['markdown'] == tmp_path / 'diagnosis' / PUBLIC_MAP_RUN_DIAGNOSIS_MARKDOWN
    assert json.loads(paths['json'].read_text(encoding='utf-8'))['status'] == 'FAIL'
    assert 'MID-360 Public Map Run Diagnosis' in paths['markdown'].read_text(
        encoding='utf-8'
    )


def test_cli_outputs_json_and_writes_artifacts(tmp_path: Path):
    run_dir = tmp_path / 'out' / 'hard'
    _write_failed_run(run_dir)
    manifest_path = tmp_path / 'manifest.json'
    manifest_path.write_text(json.dumps(_manifest(run_dir)), encoding='utf-8')
    output_dir = tmp_path / 'diagnosis'

    result = subprocess.run(
        [
            sys.executable,
            str(DIAGNOSE_SCRIPT),
            '--manifest',
            str(manifest_path),
            '--output-dir',
            str(output_dir),
            '--datasets',
            'hard_pointcloud_mid360_outdoor_kidnap_a',
            '--write',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['status'] == 'FAIL'
    assert report['counts']['total'] == 1
    assert (output_dir / PUBLIC_MAP_RUN_DIAGNOSIS_JSON).is_file()
    assert (output_dir / PUBLIC_MAP_RUN_DIAGNOSIS_MARKDOWN).is_file()
