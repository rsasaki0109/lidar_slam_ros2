# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Regression tests for the MID-360 robot map runner."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_mid360_robot_map.sh'


def _write_metadata(tmp_path: Path) -> Path:
    bag_dir = tmp_path / 'mid360_robot_bag'
    bag_dir.mkdir()
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 5_000_000_000},
            'message_count': 551,
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': '/livox/lidar',
                        'type': 'sensor_msgs/msg/PointCloud2',
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': 50,
                },
                {
                    'topic_metadata': {
                        'name': '/livox/imu',
                        'type': 'sensor_msgs/msg/Imu',
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': 500,
                },
                {
                    'topic_metadata': {
                        'name': '/tf_static',
                        'type': 'tf2_msgs/msg/TFMessage',
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': 1,
                },
            ],
        },
    }
    (bag_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
    return bag_dir


def test_mid360_robot_map_runner_dry_run_uses_topics_and_frames(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    output_dir = tmp_path / 'map_output'

    result = subprocess.run(
        [
            'bash',
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--base-frame',
            'trunk',
            '--lidar-frame',
            'mid360_link',
            '--imu-frame',
            'mid360_imu',
            '--dry-run',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    assert '--lidar-topic /livox/lidar' in result.stdout
    assert '--imu-topic /livox/imu' in result.stdout
    assert '--base-frame trunk' in result.stdout
    assert '--lidar-frame mid360_link' in result.stdout
    assert '--imu-frame mid360_imu' in result.stdout
    assert 'lidarslam_mid360_rko_graph.yaml' in result.stdout
    assert 'rko_lio_mid360.yaml' in result.stdout


def test_mid360_robot_map_runner_forwards_foxglove_alias(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)

    result = subprocess.run(
        [
            'bash',
            str(SCRIPT_PATH),
            str(bag_dir),
            '--foxglove',
            '--dry-run',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    assert 'Foxglove command:' in result.stdout
    assert 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh' in result.stdout


def test_mid360_robot_map_runner_accepts_robot_profile(tmp_path: Path):
    bag_dir = _write_metadata(tmp_path)
    profile_path = tmp_path / 'profile.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'runner_profile',
            'base_frame': 'profile_base',
            'lidar_frame': 'profile_lidar',
            'imu_frame': 'profile_imu',
            'expected_pointcloud_topic': '/livox/lidar',
            'expected_imu_topic': '/livox/imu',
        }),
        encoding='utf-8',
    )

    result = subprocess.run(
        [
            'bash',
            str(SCRIPT_PATH),
            str(bag_dir),
            '--robot-profile',
            str(profile_path),
            '--dry-run',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    assert '--base-frame profile_base' in result.stdout
    assert '--lidar-frame profile_lidar' in result.stdout
    assert '--imu-frame profile_imu' in result.stdout
