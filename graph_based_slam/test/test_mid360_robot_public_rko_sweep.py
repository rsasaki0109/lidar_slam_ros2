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

"""Tests for public MID-360 RKO-LIO parameter sweep tooling."""

from __future__ import annotations

import importlib
import json
from pathlib import Path
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
SWEEP_SCRIPT = SCRIPT_DIR / 'run_mid360_robot_public_rko_sweep.py'
LOW_VOXEL_CONFIG = (
    REPO_ROOT / 'configs' / 'mid360_robot' / 'rko_lio_mid360_low_voxel_no_deskew.yaml'
)


def _sweep_module():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    return importlib.import_module('mid360_robot_public_rko_sweep')


def _write_inputs(tmp_path: Path) -> tuple[Path, Path, Path, Path]:
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  duration:\n'
        '    nanoseconds: 10000000000\n',
        encoding='utf-8',
    )
    base_rko = tmp_path / 'base_rko.yaml'
    base_rko.write_text(
        'extrinsic_imu2base_quat_xyzw_xyz: [0, 0, 0, 1, 0, 0, 0]\n'
        'extrinsic_lidar2base_quat_xyzw_xyz: [0, 0, 0, 1, 0, 0, 0]\n'
        'initialization_phase: false\n'
        'deskew: false\n',
        encoding='utf-8',
    )
    lidarslam_param = tmp_path / 'lidarslam.yaml'
    lidarslam_param.write_text(
        '/**:\n  ros__parameters:\n    use_odom_input: true\n', encoding='utf-8',
    )
    output_dir = tmp_path / 'sweep'
    return bag, base_rko, lidarslam_param, output_dir


def test_case_parser_and_config_writer_apply_overrides(tmp_path: Path):
    module = _sweep_module()
    base_rko = tmp_path / 'base.yaml'
    output_config = tmp_path / 'case' / module.CASE_CONFIG_NAME
    base_rko.write_text(
        'extrinsic_imu2base_quat_xyzw_xyz: [0, 0, 0, 1, 0, 0, 0]\n'
        'deskew: true\n',
        encoding='utf-8',
    )
    case = module.parse_rko_sweep_case(
        'half:voxel=0.5,min=0.75,max=80,dd=false,deskew=false,init=false'
    )

    module.write_rko_case_config(base_rko, output_config, case)
    data = yaml.safe_load(output_config.read_text(encoding='utf-8'))

    assert case.case_id == 'half'
    assert data['voxel_size'] == 0.5
    assert data['min_range'] == 0.75
    assert data['max_range'] == 80.0
    assert data['double_downsample'] is False
    assert data['deskew'] is False


def test_builder_writes_plan_manifest_and_case_configs(tmp_path: Path):
    module = _sweep_module()
    bag, base_rko, lidarslam_param, output_dir = _write_inputs(tmp_path)
    options = module.RkoSweepOptions(
        repo_root=REPO_ROOT,
        bag_path=bag,
        output_dir=output_dir,
        base_rko_param=base_rko,
        lidarslam_param=lidarslam_param,
        limit=1,
    )
    builder = module.RkoSweepBuilder(
        options=options,
        cases=(module.RkoSweepCase(label='half', voxel_size=0.5, min_range=1.0),),
    )

    manifest = builder.build(run=False)
    paths = builder.write(manifest)
    markdown = module.render_rko_sweep_markdown(manifest)

    assert manifest['status'] == 'READY'
    assert manifest['counts']['cases'] == 1
    assert manifest['cases'][0]['safety']['can_run'] is True
    assert manifest['diagnostics'][0]['status'] == 'PLAN'
    assert '--wait-for-offline-completion' in manifest['cases'][0]['command']
    assert '--offline-quiet-log-secs' in manifest['cases'][0]['command']
    assert manifest['run_options']['offline_quiet_log_secs'] == 0
    assert (output_dir / 'half' / module.CASE_CONFIG_NAME).is_file()
    assert paths['json'] == output_dir / module.RKO_SWEEP_JSON
    assert paths['markdown'] == output_dir / module.RKO_SWEEP_MARKDOWN
    assert 'MID-360 Public RKO-LIO Sweep' in markdown


def test_diagnosis_extracts_keypoint_and_delta_signatures(tmp_path: Path):
    module = _sweep_module()
    output_dir = tmp_path / 'sweep' / 'half'
    output_dir.mkdir(parents=True)
    (output_dir / 'slam.launch.log').write_text(
        'RKO LIO Node is up!\n'
        '[graph_based_slam]: initialization end\n'
        'Deskewing is disabled.\n'
        'Bag reader initialized with total message count: 8567\n'
        'Error: Keypoints for ICP registration = 0, this is too little for ICP. '
        'Config voxel size = 0.500000\n'
        'Error: Received LiDAR scan with 1.200000 seconds delta to previous scan.\n',
        encoding='utf-8',
    )
    case_row = {
        'case_id': 'half',
        'label': 'half',
        'parameters': {'voxel_size': 0.5, 'min_range': 1.0},
        'output_dir': str(output_dir),
    }
    run_result = {
        'returncode': 124,
        'timed_out': True,
        'timeout_sec': 90,
        'duration_sec': 100.0,
        'stdout': '',
        'stderr': '',
    }

    diagnosis = module.diagnose_rko_sweep_case(case_row, run_result)

    assert diagnosis['status'] == 'FAIL'
    assert diagnosis['runtime']['rko_started'] is True
    assert diagnosis['runtime']['deskew_disabled'] is True
    assert diagnosis['runtime']['bag_message_count'] == 8567
    assert diagnosis['runtime']['keypoints_too_few_count'] == 1
    assert diagnosis['runtime']['keypoints_min'] == 0
    assert diagnosis['runtime']['lidar_delta_error_count'] == 1
    assert 'too few ICP keypoints' in '\n'.join(diagnosis['problem_hints'])


def test_diagnosis_verifies_saved_autoware_map(tmp_path: Path):
    module = _sweep_module()
    output_dir = tmp_path / 'sweep' / 'half'
    pointcloud_map = output_dir / 'pointcloud_map'
    pointcloud_map.mkdir(parents=True)
    (output_dir / 'map_projector_info.yaml').write_text(
        'projector_type: Local\n',
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
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        'POINTS 1\n'
        'DATA ascii\n'
        '1.0 1.0 0.0\n',
        encoding='utf-8',
    )
    (output_dir / 'slam.launch.log').write_text(
        'RKO LIO Node is up!\n'
        '[graph_based_slam]: initialization end\n'
        'First cloud received, 100 bytes\n'
        'First odom received: (0, 0, 0)\n',
        encoding='utf-8',
    )
    case_row = {
        'case_id': 'half',
        'label': 'half',
        'parameters': {'voxel_size': 0.5, 'min_range': 1.0},
        'output_dir': str(output_dir),
    }

    diagnosis = module.diagnose_rko_sweep_case(case_row, {'returncode': 0})

    assert diagnosis['status'] == 'MAP_VERIFIED'
    assert diagnosis['outputs']['map_saved'] is True
    assert diagnosis['verification']['result'] == 'PASS'
    assert diagnosis['files']['verify_log'] == str(output_dir / module.VERIFY_LOG_NAME)
    assert 'RESULT: PASS' in (output_dir / module.VERIFY_LOG_NAME).read_text(encoding='utf-8')


def test_cli_outputs_json_and_writes_artifacts(tmp_path: Path):
    module = _sweep_module()
    bag, base_rko, lidarslam_param, output_dir = _write_inputs(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SWEEP_SCRIPT),
            '--bag',
            str(bag),
            '--base-rko-param',
            str(base_rko),
            '--lidarslam-param',
            str(lidarslam_param),
            '--output-dir',
            str(output_dir),
            '--case',
            'half:voxel_size=0.5,min_range=1.0,double_downsample=true,deskew=false',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    manifest = json.loads(result.stdout)

    assert manifest['status'] == 'READY'
    assert manifest['counts']['cases'] == 1
    assert (output_dir / module.RKO_SWEEP_JSON).is_file()
    assert (output_dir / module.RKO_SWEEP_MARKDOWN).is_file()


def test_tracked_low_voxel_config_matches_successful_public_case():
    data = yaml.safe_load(LOW_VOXEL_CONFIG.read_text(encoding='utf-8'))

    assert data['voxel_size'] == 0.5
    assert data['min_range'] == 1.0
    assert data['deskew'] is False
    assert data['double_downsample'] is True
    assert data['initialization_phase'] is False
