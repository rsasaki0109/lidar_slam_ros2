# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""CLI tests for MID-360 robot profile validation."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'validate_mid360_robot_profile.py'


def test_profile_cli_emits_normalized_json(tmp_path: Path):
    profile_path = tmp_path / 'profile.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'go2_mid360',
            'base_frame': 'trunk',
            'lidar_frame': 'mid360_link',
            'imu_frame': 'mid360_imu',
            'expected_pointcloud_topic': '/livox/lidar',
            'expected_imu_topic': '/livox/imu',
            'mount': {
                'xyz': [1, 2, 3],
                'q_xyzw': [0, 0, 0, 1],
            },
        }),
        encoding='utf-8',
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(profile_path), '--json'],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    payload = json.loads(result.stdout)

    assert payload['robot_name'] == 'go2_mid360'
    assert payload['frames']['base_frame'] == 'trunk'
    assert payload['mount']['xyz'] == [1.0, 2.0, 3.0]


def test_profile_cli_rejects_bad_mount_shape(tmp_path: Path):
    profile_path = tmp_path / 'bad_profile.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'bad_robot',
            'mount': {
                'xyz': [0.0, 0.0],
            },
        }),
        encoding='utf-8',
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(profile_path)],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    assert result.returncode != 0
    assert 'mount.xyz' in result.stderr
