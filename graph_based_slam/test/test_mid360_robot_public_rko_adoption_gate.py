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

"""Tests for the public MID-360 RKO-LIO adoption gate runner."""

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
GATE_SCRIPT = SCRIPT_DIR / 'run_mid360_robot_public_rko_adoption_gate.py'


def _ensure_script_path():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))


def _gate_module():
    _ensure_script_path()
    return importlib.import_module('mid360_robot_public_rko_adoption_gate')


def _quality_module():
    _ensure_script_path()
    return importlib.import_module('mid360_robot_public_rko_quality_report')


def _sweep_module():
    _ensure_script_path()
    return importlib.import_module('mid360_robot_public_rko_sweep')


BEST_PARAMETERS = {
    'voxel_size': 0.5,
    'min_range': 1.0,
    'max_range': 100.0,
    'deskew': False,
    'double_downsample': True,
    'initialization_phase': False,
}


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
    path.write_bytes(header.encode('ascii') + b''.join(struct.pack('<fff', *p) for p in points))


def _write_case_output(root: Path, case_id: str) -> Path:
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
    _write_binary_xyz_pcd(map_dir / '0_0.pcd', [(0.0, 0.0, 0.0)] * 4)
    lines = [
        f'{float(index):.1f} {float(index):.3f} 0.0 0.0 0.0 0.0 0.0 1.0'
        for index in range(5)
    ]
    (traj_dir / f'{case_id}_tum_0.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return case_dir


def _write_sweep(tmp_path: Path) -> Path:
    sweep_dir = tmp_path / 'sweep'
    case_dir = _write_case_output(sweep_dir, 'voxel_0p50_min_1p00_dd_on')
    manifest = {
        'status': 'PASS',
        'bag_path': str(tmp_path / 'bag'),
        'output_dir': str(sweep_dir),
        'counts': {'map_verified': 1},
        'diagnostics': [
            {
                'case_id': 'voxel_0p50_min_1p00_dd_on',
                'label': 'voxel_0p50_min_1p00_dd_on',
                'status': 'MAP_VERIFIED',
                'parameters': BEST_PARAMETERS,
                'output_dir': str(case_dir),
                'run_result': {'returncode': 0, 'timed_out': False, 'duration_sec': 13.5},
                'runtime': {
                    'offline_completed': True,
                    'keypoints_too_few_count': 0,
                    'lidar_delta_error_count': 0,
                },
                'outputs': {'map_saved': True},
                'verification': {'result': 'PASS'},
            }
        ],
    }
    path = sweep_dir / _sweep_module().RKO_SWEEP_JSON
    path.write_text(json.dumps(manifest), encoding='utf-8')
    return path


def _write_config(tmp_path: Path, parameters: dict) -> Path:
    path = tmp_path / 'rko_config.yaml'
    path.write_text(yaml.safe_dump(parameters, sort_keys=False), encoding='utf-8')
    return path


def _thresholds():
    quality = _quality_module()
    return quality.RkoQualityGateThresholds(
        min_trajectory_poses=5,
        min_trajectory_duration_sec=4.0,
        min_path_length_m=3.0,
        max_step_m=2.0,
        min_map_points=4,
        min_map_tiles=1,
        max_runtime_sec=20.0,
    )


def test_adoption_gate_runner_passes_from_existing_sweep(tmp_path: Path):
    gate = _gate_module()
    sweep_path = _write_sweep(tmp_path)
    config_path = _write_config(tmp_path, BEST_PARAMETERS)
    output_dir = tmp_path / 'gate'

    report = gate.RkoAdoptionGateRunner(
        gate.RkoAdoptionGateOptions(
            repo_root=REPO_ROOT,
            output_dir=output_dir,
            config_path=config_path,
            sweep_path=sweep_path,
            thresholds=_thresholds(),
        )
    ).run()

    assert report['status'] == 'PASS'
    assert report['decision']['matched_case'] == 'voxel_0p50_min_1p00_dd_on'
    assert report['decision']['recommended_case'] == 'voxel_0p50_min_1p00_dd_on'
    assert report['decision']['gate_pass_cases'] == 1
    assert (output_dir / gate.RKO_ADOPTION_GATE_JSON).is_file()
    assert (output_dir / gate.RKO_ADOPTION_GATE_MARKDOWN).is_file()


def test_adoption_gate_cli_fails_for_non_matching_config(tmp_path: Path):
    gate = _gate_module()
    sweep_path = _write_sweep(tmp_path)
    config_path = _write_config(tmp_path, {**BEST_PARAMETERS, 'voxel_size': 0.3})
    output_dir = tmp_path / 'gate'

    result = subprocess.run(
        [
            sys.executable,
            str(GATE_SCRIPT),
            '--from-existing',
            '--sweep',
            str(sweep_path),
            '--output-dir',
            str(output_dir),
            '--config',
            str(config_path),
            '--min-trajectory-poses',
            '5',
            '--min-trajectory-duration-sec',
            '4',
            '--min-path-length-m',
            '3',
            '--max-step-m',
            '2',
            '--min-map-points',
            '4',
            '--min-map-tiles',
            '1',
            '--max-runtime-sec',
            '20',
            '--json',
        ],
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert report['checks'][-1]['id'] == 'adoption_pass'
    assert report['checks'][-1]['status'] == 'FAIL'
    assert (output_dir / gate.RKO_ADOPTION_GATE_JSON).is_file()
