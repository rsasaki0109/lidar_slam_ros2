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

"""Tests for public MID-360 RKO-LIO quality reports."""

from __future__ import annotations

import importlib
import json
from pathlib import Path
import struct
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
QUALITY_SCRIPT = SCRIPT_DIR / 'generate_mid360_robot_public_rko_quality_report.py'


def _quality_module():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    return importlib.import_module('mid360_robot_public_rko_quality_report')


def _write_binary_xyz_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    header = '\n'.join([
        '# .PCD v0.7 - Point Cloud Data file format',
        'VERSION 0.7',
        'FIELDS x y z',
        'SIZE 4 4 4',
        'TYPE F F F',
        'COUNT 1 1 1',
        f'WIDTH {len(points)}',
        'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {len(points)}',
        'DATA binary',
        '',
    ])
    payload = b''.join(struct.pack('<fff', *point) for point in points)
    path.write_bytes(header.encode('ascii') + payload)


def _write_case_output(root: Path, case_id: str, *, points: int, poses: int) -> Path:
    case_dir = root / case_id
    map_dir = case_dir / 'pointcloud_map'
    traj_dir = case_dir / f'{case_id}_0'
    map_dir.mkdir(parents=True)
    traj_dir.mkdir(parents=True)
    (case_dir / 'map_projector_info.yaml').write_text('projector_type: Local\n', encoding='utf-8')
    (map_dir / 'pointcloud_map_metadata.yaml').write_text(
        yaml.safe_dump({
            'x_resolution': 20,
            'y_resolution': 20,
            '0_0.pcd': [0, 0],
        }, sort_keys=False),
        encoding='utf-8',
    )
    _write_binary_xyz_pcd(
        map_dir / '0_0.pcd',
        [(float(index), 1.0, 0.0) for index in range(points)],
    )
    traj_lines = [
        f'{float(index):.1f} {float(index):.3f} 0.0 0.0 0.0 0.0 0.0 1.0'
        for index in range(poses)
    ]
    (traj_dir / f'{case_id}_tum_0.txt').write_text('\n'.join(traj_lines) + '\n', encoding='utf-8')
    return case_dir


def _write_sweep(tmp_path: Path) -> Path:
    sweep_dir = tmp_path / 'sweep'
    best_dir = _write_case_output(sweep_dir, 'voxel_0p50_min_1p00_dd_on', points=4, poses=5)
    weak_dir = _write_case_output(sweep_dir, 'voxel_0p30_min_1p00_dd_on', points=2, poses=3)
    manifest = {
        'status': 'PASS',
        'bag_path': str(tmp_path / 'bag'),
        'output_dir': str(sweep_dir),
        'diagnostics': [
            {
                'case_id': 'voxel_0p50_min_1p00_dd_on',
                'label': 'voxel_0p50_min_1p00_dd_on',
                'status': 'MAP_VERIFIED',
                'parameters': {'voxel_size': 0.5, 'min_range': 1.0},
                'output_dir': str(best_dir),
                'run_result': {'returncode': 0, 'timed_out': False, 'duration_sec': 13.5},
                'runtime': {
                    'offline_completed': True,
                    'keypoints_too_few_count': 0,
                    'lidar_delta_error_count': 0,
                },
                'outputs': {'map_saved': True},
                'verification': {'result': 'PASS'},
            },
            {
                'case_id': 'voxel_0p30_min_1p00_dd_on',
                'label': 'voxel_0p30_min_1p00_dd_on',
                'status': 'MAP_SAVED',
                'parameters': {'voxel_size': 0.3, 'min_range': 1.0},
                'output_dir': str(weak_dir),
                'run_result': {'returncode': 0, 'timed_out': False, 'duration_sec': 19.5},
                'runtime': {
                    'offline_completed': True,
                    'keypoints_too_few_count': 1,
                    'lidar_delta_error_count': 0,
                },
                'outputs': {'map_saved': True},
                'verification': {'result': 'SKIP'},
            },
        ],
    }
    path = sweep_dir / 'mid360_robot_public_rko_sweep.json'
    path.write_text(json.dumps(manifest), encoding='utf-8')
    return path


def test_quality_report_summarizes_map_and_trajectory(tmp_path: Path):
    module = _quality_module()
    sweep_path = _write_sweep(tmp_path)

    report = module.RkoQualityReportBuilder(
        sweep_path,
        thresholds=module.RkoQualityGateThresholds(
            min_trajectory_poses=5,
            min_trajectory_duration_sec=4.0,
            min_path_length_m=3.0,
            max_step_m=2.0,
            min_map_points=4,
            min_map_tiles=1,
            max_runtime_sec=20.0,
        ),
    ).build_report()

    assert report['status'] == 'PASS'
    assert report['counts']['cases'] == 2
    assert report['counts']['map_verified'] == 1
    assert report['counts']['gate_pass'] == 1
    assert report['counts']['total_map_points'] == 6
    assert report['best_case']['case_id'] == 'voxel_0p50_min_1p00_dd_on'
    assert report['best_case']['gate_status'] == 'PASS'
    assert report['cases'][0]['rank'] == 1
    assert report['cases'][0]['quality_gate']['status'] == 'PASS'
    assert report['cases'][0]['trajectory']['poses'] == 5
    assert report['cases'][0]['trajectory']['duration_sec'] == 4.0
    assert report['cases'][0]['trajectory']['path_length_m'] == 4.0
    assert report['cases'][0]['map_quality']['tile_count'] == 1
    assert report['cases'][0]['map_quality']['point_density_per_m2'] == 0.01


def test_quality_report_writes_json_markdown_and_html(tmp_path: Path):
    module = _quality_module()
    sweep_path = _write_sweep(tmp_path)
    report = module.RkoQualityReportBuilder(sweep_path).build_report()
    output_dir = tmp_path / 'report'

    paths = module.write_rko_quality_report(report, output_dir)
    markdown = module.render_rko_quality_markdown(report)
    html = paths['html'].read_text(encoding='utf-8')

    assert paths['json'] == output_dir / module.RKO_QUALITY_JSON
    assert paths['markdown'] == output_dir / module.RKO_QUALITY_MARKDOWN
    assert paths['html'] == output_dir / module.RKO_QUALITY_HTML
    assert 'Quality Report' in markdown
    assert 'Gate' in markdown
    assert 'Map Points' in html
    assert json.loads(paths['json'].read_text(encoding='utf-8'))['counts']['cases'] == 2


def test_quality_report_cli_outputs_json(tmp_path: Path):
    module = _quality_module()
    sweep_path = _write_sweep(tmp_path)
    output_dir = tmp_path / 'quality'

    result = subprocess.run(
        [
            sys.executable,
            str(QUALITY_SCRIPT),
            '--sweep',
            str(sweep_path),
            '--output-dir',
            str(output_dir),
            '--min-trajectory-poses',
            '5',
            '--min-trajectory-duration-sec',
            '4',
            '--min-path-length-m',
            '3',
            '--min-map-points',
            '4',
            '--min-map-tiles',
            '1',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['best_case']['case_id'] == 'voxel_0p50_min_1p00_dd_on'
    assert report['counts']['gate_pass'] == 1
    assert (output_dir / module.RKO_QUALITY_JSON).is_file()
    assert (output_dir / module.RKO_QUALITY_MARKDOWN).is_file()
    assert (output_dir / module.RKO_QUALITY_HTML).is_file()


def test_quality_gate_fails_short_trajectory_duration(tmp_path: Path):
    module = _quality_module()
    sweep_path = _write_sweep(tmp_path)

    report = module.RkoQualityReportBuilder(
        sweep_path,
        thresholds=module.RkoQualityGateThresholds(
            min_trajectory_poses=5,
            min_trajectory_duration_sec=10.0,
            min_path_length_m=3.0,
            max_step_m=2.0,
            min_map_points=4,
            min_map_tiles=1,
            max_runtime_sec=20.0,
        ),
    ).build_report()

    best = report['cases'][0]
    checks = {
        check['id']: check['status']
        for check in best['quality_gate']['checks']
    }
    assert report['status'] == 'WARN'
    assert best['quality_gate']['status'] == 'FAIL'
    assert checks['trajectory_duration'] == 'FAIL'
