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

"""Tests for public MID-360 dataset comparison reports."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
REPORT_SCRIPT = SCRIPT_DIR / 'generate_mid360_robot_public_dataset_report.py'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_dataset_report import (  # noqa: E402
    PUBLIC_DATASET_REPORT_HTML,
    PUBLIC_DATASET_REPORT_JSON,
    PUBLIC_DATASET_REPORT_MARKDOWN,
    PublicDatasetReportBuilder,
    render_public_dataset_report_markdown,
    write_public_dataset_report,
)


def _write_public_artifacts(
    root: Path,
    output_root: Path,
    dataset_id: str,
    *,
    status: str = 'WARN',
    pointcloud_topic: str = '/livox/lidar',
    pointcloud_rate: float = 10.0,
) -> None:
    dataset_dir = root / dataset_id
    output_dir = output_root / dataset_id
    dataset_dir.mkdir(parents=True)
    output_dir.mkdir(parents=True)
    (dataset_dir / 'mid360_robot_public_dataset_intake.json').write_text(
        json.dumps({
            'status': 'READY',
            'selected_bag_path': str(dataset_dir / 'bag'),
        }),
        encoding='utf-8',
    )
    checks = [
        {'id': 'metadata_yaml', 'status': 'ok', 'message': 'metadata exists'},
        {'id': 'tf_metadata', 'status': 'warn', 'message': 'No TF/TF_STATIC topic found'},
    ]
    (output_dir / 'mid360_robot_recording_check.json').write_text(
        json.dumps({
            'status': status,
            'bag_path': str(dataset_dir / 'bag'),
            'selected_topics': {'pointcloud': pointcloud_topic, 'imu': '/livox/imu'},
            'checks': checks,
        }),
        encoding='utf-8',
    )
    (output_dir / 'mid360_robot_readiness.json').write_text(
        json.dumps({
            'status': status,
            'selected_topics': {'pointcloud': pointcloud_topic, 'imu': '/livox/imu'},
            'checks': checks,
            'bag_diagnostics': {
                'sample_reader': {'available': True},
                'topics': {
                    'pointcloud': {
                        'metadata_rate_hz': pointcloud_rate,
                        'metadata_message_count': 100,
                        'sampled_frame_ids': ['livox_frame'],
                    },
                    'imu': {
                        'metadata_rate_hz': 200.0,
                        'metadata_message_count': 2000,
                        'sampled_frame_ids': ['livox_frame'],
                    },
                },
            },
        }),
        encoding='utf-8',
    )
    (output_dir / 'mid360_robot_run_plan.json').write_text(
        json.dumps({
            'ready_for_mid360_launch': True,
            'dogfood_command_shell': 'bash scripts/run_mid360_robot_map.sh bag --dry-run',
        }),
        encoding='utf-8',
    )


def _write_map_sweep(output_root: Path, dataset_id: str) -> Path:
    sweep_dir = output_root / 'rko_sweep_no_quiet_all'
    sweep_dir.mkdir(parents=True)
    path = sweep_dir / 'mid360_robot_public_rko_sweep.json'
    path.write_text(
        json.dumps({
            'created_at': '2026-05-22T00:00:00+00:00',
            'status': 'PASS',
            'bag_path': str(output_root / dataset_id / 'segment_002' / 'rosbag2'),
            'output_dir': str(sweep_dir),
            'counts': {
                'map_saved': 4,
                'map_verified': 4,
                'verify_failed': 0,
                'keypoint_drop_cases': 0,
                'lidar_delta_cases': 0,
            },
            'diagnostics': [
                {
                    'case_id': 'voxel_0p50_min_1p00_dd_on',
                    'status': 'MAP_VERIFIED',
                    'output_dir': str(sweep_dir / 'voxel_0p50_min_1p00_dd_on'),
                    'runtime': {
                        'offline_completed': True,
                        'keypoints_too_few_count': 0,
                        'lidar_delta_error_count': 0,
                    },
                    'run_result': {'duration_sec': 13.8},
                    'verification': {'result': 'PASS'},
                }
            ],
        }),
        encoding='utf-8',
    )
    return path


def test_public_dataset_report_summarizes_two_public_bags(tmp_path: Path):
    dataset_root = tmp_path / 'datasets'
    output_root = tmp_path / 'output'
    _write_public_artifacts(dataset_root, output_root, 'driving_slam_mid360')
    _write_public_artifacts(
        dataset_root,
        output_root,
        'hard_pointcloud_mid360_outdoor_kidnap_a',
        pointcloud_topic='/livox/points',
        pointcloud_rate=7.0,
    )
    _write_map_sweep(output_root, 'hard_pointcloud_mid360_outdoor_kidnap_a')

    report = PublicDatasetReportBuilder(
        dataset_root=dataset_root,
        output_root=output_root,
        dataset_ids=['driving_slam_mid360', 'hard_pointcloud_mid360_outdoor_kidnap_a'],
    ).build_report()

    assert report['status'] == 'WARN'
    assert report['counts']['total'] == 2
    assert report['counts']['ready_for_mid360_launch'] == 2
    assert report['datasets'][0]['rates_hz']['pointcloud'] == 10.0
    assert report['datasets'][1]['selected_topics']['pointcloud'] == '/livox/points'
    assert report['datasets'][1]['warnings'][0]['id'] == 'tf_metadata'
    assert report['datasets'][1]['map_validation']['status'] == 'MAP_VERIFIED'
    assert report['datasets'][1]['map_validation']['map_verified'] == 4
    assert report['counts']['map_verified'] == 1


def test_public_dataset_report_writes_json_markdown_and_html(tmp_path: Path):
    dataset_root = tmp_path / 'datasets'
    output_root = tmp_path / 'output'
    output_dir = tmp_path / 'report'
    _write_public_artifacts(dataset_root, output_root, 'driving_slam_mid360')
    report = PublicDatasetReportBuilder(
        dataset_root=dataset_root,
        output_root=output_root,
        dataset_ids=['driving_slam_mid360'],
    ).build_report()

    paths = write_public_dataset_report(report, output_dir)
    markdown = render_public_dataset_report_markdown(report)
    html = (output_dir / PUBLIC_DATASET_REPORT_HTML).read_text(encoding='utf-8')

    assert paths['json'] == output_dir / PUBLIC_DATASET_REPORT_JSON
    assert paths['markdown'] == output_dir / PUBLIC_DATASET_REPORT_MARKDOWN
    assert paths['html'] == output_dir / PUBLIC_DATASET_REPORT_HTML
    assert 'driving_slam_mid360' in markdown
    assert 'map_verified' in markdown
    assert 'Real Bag Intake Comparison' in html
    assert json.loads(paths['json'].read_text(encoding='utf-8'))['status'] == 'WARN'


def test_public_dataset_report_marks_missing_artifacts(tmp_path: Path):
    report = PublicDatasetReportBuilder(
        dataset_root=tmp_path / 'datasets',
        output_root=tmp_path / 'output',
        dataset_ids=['driving_slam_mid360'],
    ).build_report()

    assert report['status'] == 'INCOMPLETE'
    assert report['counts']['missing'] == 1
    assert report['datasets'][0]['status'] == 'MISSING'


def test_public_dataset_report_cli_writes_files(tmp_path: Path):
    dataset_root = tmp_path / 'datasets'
    output_root = tmp_path / 'output'
    report_dir = tmp_path / 'report'
    _write_public_artifacts(dataset_root, output_root, 'driving_slam_mid360')

    result = subprocess.run(
        [
            sys.executable,
            str(REPORT_SCRIPT),
            '--dataset-root',
            str(dataset_root),
            '--output-root',
            str(output_root),
            '--output-dir',
            str(report_dir),
            '--datasets',
            'driving_slam_mid360',
            '--map-sweep',
            str(_write_map_sweep(output_root, 'driving_slam_mid360')),
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['status'] == 'WARN'
    assert report['datasets'][0]['map_validation']['status'] == 'MAP_VERIFIED'
    assert (report_dir / PUBLIC_DATASET_REPORT_JSON).is_file()
    assert (report_dir / PUBLIC_DATASET_REPORT_MARKDOWN).is_file()
    assert (report_dir / PUBLIC_DATASET_REPORT_HTML).is_file()
