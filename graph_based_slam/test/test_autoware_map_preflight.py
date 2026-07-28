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
import sqlite3
import subprocess
import sys

import jsonschema
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


def _write_metadata(
    tmp_path: Path,
    topics: list[tuple[str, str, int]],
    timestamps_by_topic: dict[str, list[int]] | None = None,
) -> Path:
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    storage_name = 'bag_0.db3'
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 12_500_000_000},
            'message_count': sum(count for _, _, count in topics),
            'storage_identifier': 'sqlite3',
            'relative_file_paths': [storage_name],
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
    connection = sqlite3.connect(bag_dir / storage_name)
    connection.executescript(
        'CREATE TABLE topics ('
        'id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, '
        'serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);'
        'CREATE TABLE messages ('
        'id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, '
        'timestamp INTEGER NOT NULL, data BLOB NOT NULL);'
    )
    message_id = 1
    for topic_id, (name, msg_type, count) in enumerate(topics, start=1):
        connection.execute(
            'INSERT INTO topics VALUES (?, ?, ?, ?, ?)',
            (topic_id, name, msg_type, 'cdr', ''),
        )
        timestamps = (
            timestamps_by_topic[name]
            if timestamps_by_topic is not None and name in timestamps_by_topic
            else list(range(1, count + 1))
        )
        assert len(timestamps) == count
        for timestamp in timestamps:
            connection.execute(
                'INSERT INTO messages VALUES (?, ?, ?, ?)',
                (message_id, topic_id, timestamp, b''),
            )
            message_id += 1
    connection.commit()
    connection.close()
    return bag_dir


def _compatible_inspection(_bag_path: Path, topic: str, _storage_id: str) -> dict:
    return {
        'status': 'inspected',
        'topic': topic,
        'fields': [
            {'name': 'x', 'datatype': 7, 'count': 1},
            {'name': 'y', 'datatype': 7, 'count': 1},
            {'name': 'z', 'datatype': 7, 'count': 1},
            {'name': 'time', 'datatype': 7, 'count': 1},
        ],
        'rko_lio_compatible': True,
        'timestamp_field': 'time',
        'reason': "RKO-LIO-compatible per-point timestamp field 'time' was found.",
    }


def _passing_timestamp_inspection(
    _bag_path: Path,
    topics: list[str],
    _bag_info: dict,
) -> dict:
    return {
        'status': 'passed',
        'storage_identifier': 'sqlite3',
        'ordering': 'sqlite_message_row_id',
        'checked_storage_files': ['fixture.db3'],
        'checked_topics': [
            {'topic': topic, 'checked_records': 1}
            for topic in sorted(topics)
        ],
        'checked_records': len(topics),
        'first_reversal': None,
        'reason': 'Fixture timestamps are non-decreasing.',
    }


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

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
    )

    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'preflight-v3.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate(payload, schema)
    assert payload['schema_version'] == 3
    assert payload['schema_uri'].endswith('/schemas/preflight-v3.schema.json')
    assert payload['summary']['record_timestamp_inspection']['status'] == 'passed'
    assert payload['summary']['pointcloud_inspection']['timestamp_field'] == 'time'
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


def test_cli_help_is_user_facing():
    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), '--help'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert 'Autoware-compatible map workflow' in result.stdout
    assert 'The input must be the rosbag2 directory' in result.stdout
    assert 'Pass /path/to/rosbag2, not /path/to/rosbag2_0.db3.' in result.stdout
    assert 'Profiles this tool can recommend:' in result.stdout
    assert 'rko_lio_graph_public_path' in result.stdout
    assert '--json' in result.stdout


def test_cli_rejects_missing_bag_without_traceback(tmp_path: Path):
    missing_bag = tmp_path / 'missing_bag'

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(missing_bag)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'rosbag2 directory does not exist' in result.stderr
    assert 'Traceback' not in result.stderr


def test_cli_rejects_db3_file_without_traceback(tmp_path: Path):
    db3_path = tmp_path / 'demo_0.db3'
    db3_path.write_text('', encoding='utf-8')

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(db3_path)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'not the .db3 file' in result.stderr
    assert 'Traceback' not in result.stderr


def test_cli_rejects_missing_metadata_without_traceback(tmp_path: Path):
    bag_dir = tmp_path / 'bag_without_metadata'
    bag_dir.mkdir()

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(bag_dir)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'metadata.yaml not found' in result.stderr
    assert 'Traceback' not in result.stderr


def test_cli_rejects_invalid_metadata_yaml_without_traceback(tmp_path: Path):
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    (bag_dir / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: [invalid',
        encoding='utf-8',
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(bag_dir)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'failed to parse' in result.stderr
    assert 'Traceback' not in result.stderr


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

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
        timestamp_inspector=_passing_timestamp_inspection,
    )
    recommendation_ids = [item['id'] for item in payload['recommendations']]

    assert payload['recommended_profile_id'] == 'rko_lio_graph_public_path'
    assert 'rko_lio_graph_mid360_preset' in recommendation_ids
    tuned = next(
        item for item in payload['recommendations']
        if item['id'] == 'rko_lio_graph_mid360_preset'
    )
    assert 'lidarslam_mid360_rko_graph.yaml' in tuned['command']
    assert 'rko_lio_mid360.yaml' in tuned['command']


def test_pointcloud_field_assessment_accepts_supported_timestamp():
    module = _load_module()

    assessment = module.assess_pointcloud_fields([
        {'name': 'x', 'datatype': 7, 'count': 1},
        {'name': 'y', 'datatype': 7, 'count': 1},
        {'name': 'z', 'datatype': 7, 'count': 1},
        {'name': 'timestamp', 'datatype': 8, 'count': 1},
    ])

    assert assessment['rko_lio_compatible'] is True
    assert assessment['timestamp_field'] == 'timestamp'


def test_pointcloud_field_assessment_rejects_missing_timestamp():
    module = _load_module()

    assessment = module.assess_pointcloud_fields([
        {'name': 'x', 'datatype': 7, 'count': 1},
        {'name': 'y', 'datatype': 7, 'count': 1},
        {'name': 'z', 'datatype': 7, 'count': 1},
        {'name': 'intensity', 'datatype': 7, 'count': 1},
    ])

    assert assessment['rko_lio_compatible'] is False
    assert assessment['timestamp_field'] is None
    assert 'expected t/timestamp/time/stamps' in assessment['reason']


def test_pointcloud_field_assessment_rejects_unsupported_timestamp_type():
    module = _load_module()

    assessment = module.assess_pointcloud_fields([
        {'name': 'x', 'datatype': 7, 'count': 1},
        {'name': 'y', 'datatype': 7, 'count': 1},
        {'name': 'z', 'datatype': 7, 'count': 1},
        {'name': 'time', 'datatype': 2, 'count': 1},
    ])

    assert assessment['rko_lio_compatible'] is False
    assert 'UINT32, FLOAT32, or FLOAT64' in assessment['reason']


def test_missing_point_timestamp_prevents_rko_recommendation(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 200),
            ('/imu/data', 'sensor_msgs/msg/Imu', 2000),
        ],
    )

    def incompatible(_bag_path: Path, topic: str, _storage_id: str) -> dict:
        return {
            'status': 'inspected',
            'topic': topic,
            'fields': [
                {'name': 'x', 'datatype': 7, 'count': 1},
                {'name': 'y', 'datatype': 7, 'count': 1},
                {'name': 'z', 'datatype': 7, 'count': 1},
            ],
            'rko_lio_compatible': False,
            'timestamp_field': None,
            'reason': (
                'PointCloud2 has no per-point timestamp field required for '
                'RKO-LIO deskewing (expected t/timestamp/time/stamps).'
            ),
        }

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=incompatible,
    )

    assert payload['recommended_profile_id'] is None
    assert payload['recommendations'] == []
    assert payload['beginner_commands'] == []
    assert any(
        'expected t/timestamp/time/stamps' in item
        for item in payload['missing_requirements']
    )


def test_record_timestamp_inspection_streams_real_sqlite_records(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 3),
            ('/imu', 'sensor_msgs/msg/Imu', 4),
        ],
        {
            '/points': [1_000, 1_000, 2_000],
            '/imu': [900, 1_100, 1_200, 1_300],
        },
    )

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
    )
    inspection = payload['summary']['record_timestamp_inspection']

    assert inspection['status'] == 'passed'
    assert inspection['ordering'] == 'sqlite_message_row_id'
    assert inspection['checked_records'] == 7
    assert inspection['first_reversal'] is None
    assert payload['recommended_profile_id'] == 'rko_lio_graph_public_path'


def test_record_timestamp_reversal_removes_all_workflow_recommendations(
    tmp_path: Path,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 3),
            ('/imu', 'sensor_msgs/msg/Imu', 3),
        ],
        {
            '/points': [1_000, 2_000, 1_500],
            '/imu': [900, 1_100, 1_200],
        },
    )

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
    )
    inspection = payload['summary']['record_timestamp_inspection']

    assert inspection['status'] == 'failed'
    assert inspection['first_reversal']['topic'] == '/points'
    assert inspection['first_reversal']['previous_timestamp_ns'] == 2_000
    assert inspection['first_reversal']['timestamp_ns'] == 1_500
    assert payload['recommendations'] == []
    assert payload['recommended_profile_id'] is None
    assert payload['beginner_commands'] == []
    assert any(
        'record timestamp reversal on /points' in reason
        for reason in payload['missing_requirements']
    )


def test_record_timestamp_reversal_is_detected_across_split_files(
    tmp_path: Path,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [('/points', 'sensor_msgs/msg/PointCloud2', 2)],
        {'/points': [1_000, 2_000]},
    )
    second_storage = bag_dir / 'bag_1.db3'
    connection = sqlite3.connect(second_storage)
    connection.executescript(
        'CREATE TABLE topics ('
        'id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, '
        'serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);'
        'CREATE TABLE messages ('
        'id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, '
        'timestamp INTEGER NOT NULL, data BLOB NOT NULL);'
    )
    connection.execute(
        'INSERT INTO topics VALUES (?, ?, ?, ?, ?)',
        (1, '/points', 'sensor_msgs/msg/PointCloud2', 'cdr', ''),
    )
    connection.execute(
        'INSERT INTO messages VALUES (?, ?, ?, ?)',
        (1, 1, 1_500, b''),
    )
    connection.commit()
    connection.close()
    metadata_path = bag_dir / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    metadata['rosbag2_bagfile_information']['relative_file_paths'].append(
        second_storage.name
    )
    metadata_path.write_text(yaml.safe_dump(metadata), encoding='utf-8')

    bag_info = module.load_bag_metadata(bag_dir)
    inspection = module.inspect_record_timestamps(
        bag_dir,
        ['/points'],
        bag_info,
    )

    assert inspection['status'] == 'failed'
    assert inspection['first_reversal']['previous_storage_file'] == 'bag_0.db3'
    assert inspection['first_reversal']['storage_file'] == 'bag_1.db3'
    assert inspection['first_reversal']['previous_timestamp_ns'] == 2_000
    assert inspection['first_reversal']['timestamp_ns'] == 1_500
