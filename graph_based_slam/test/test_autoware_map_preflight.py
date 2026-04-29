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

"""Regression tests for the Autoware map preflight helper."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'preflight_autoware_map_bag.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('preflight_autoware_map_bag', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_metadata(tmp_path: Path, topics: list[tuple[str, str, int]]) -> Path:
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 12_500_000_000},
            'message_count': sum(count for _, _, count in topics),
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': name,
                        'type': msg_type,
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': count,
                }
                for name, msg_type, count in topics
            ],
        },
    }
    (bag_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
    return bag_dir


def test_rko_lio_public_path_is_preferred_for_pointcloud_and_imu(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 200),
            ('/imu/data', 'sensor_msgs/msg/Imu', 2000),
            ('/gnss/fix', 'sensor_msgs/msg/NavSatFix', 500),
        ],
    )

    payload = module.build_preflight_payload(bag_dir)

    assert payload['recommended_profile_id'] == 'rko_lio_graph_public_path'
    assert payload['beginner_commands'][0]['command'].startswith(
        'bash scripts/run_autoware_map_beginner.sh'
    )
    assert payload['recommendations'][0]['command'].startswith(
        'ros2 launch lidarslam rko_lio_slam.launch.py'
    )
    assert any(item['id'] == 'pointcloud_gnss_smoke' for item in payload['recommendations'])
    report = module.render_text_report(payload)
    assert 'Recommended path: RKO-LIO + graph_based_slam public path' in report
    assert 'Beginner command:' in report
    assert 'Beginner command with browser viewer:' in report
    assert 'run_autoware_map_beginner.sh' in report
    assert 'inspect_navsatfix_covariance.py' in report


def test_packet_applanix_path_is_recommended_when_packet_topics_exist(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/front/packets', 'velodyne_msgs/msg/VelodyneScan', 300),
            ('/gsof49', 'applanix_msgs/msg/NavigationSolutionGsof49', 1500),
            ('/gsof50', 'applanix_msgs/msg/NavigationPerformanceGsof50', 150),
        ],
    )

    payload = module.build_preflight_payload(bag_dir)

    assert payload['recommended_profile_id'] == 'packet_applanix_smoke'
    assert payload['recommendations'][0]['command'].startswith(
        'bash scripts/run_open_data_applanix_velodyne_gnss_smoke.sh'
    )
    assert payload['advisory'][0]['command'].startswith(
        'python3 scripts/inspect_applanix_gsof50_quality.py'
    )


def test_cli_json_output_matches_machine_readable_payload(tmp_path: Path):
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 100),
            ('/tf', 'tf2_msgs/msg/TFMessage', 200),
        ],
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(bag_dir), '--json'],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(result.stdout)

    assert payload['recommended_profile_id'] is None
    assert payload['beginner_commands'] == []
    assert payload['summary']['capabilities']['has_pointcloud2'] is True
    assert payload['summary']['capabilities']['has_imu'] is False
    assert any('No Imu topic was found' in item for item in payload['missing_requirements'])


def test_livox_mid360_bag_emits_tuned_preset_hint(tmp_path: Path):
    module = _load_module()
    bag_dir = tmp_path / 'mid360_demo_bag'
    bag_dir.mkdir()
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 2_000_000_000},
            'message_count': 200,
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': '/livox/lidar',
                        'type': 'sensor_msgs/msg/PointCloud2',
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': 20,
                },
                {
                    'topic_metadata': {
                        'name': '/livox/imu',
                        'type': 'sensor_msgs/msg/Imu',
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': 180,
                },
            ],
        },
    }
    (bag_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')

    payload = module.build_preflight_payload(bag_dir)
    recommendation_ids = [item['id'] for item in payload['recommendations']]

    assert payload['recommended_profile_id'] == 'rko_lio_graph_public_path'
    assert 'rko_lio_graph_mid360_preset' in recommendation_ids
    tuned = next(
        item for item in payload['recommendations']
        if item['id'] == 'rko_lio_graph_mid360_preset'
    )
    assert 'lidarslam_mid360_rko_graph.yaml' in tuned['command']
    assert 'rko_lio_mid360.yaml' in tuned['command']
