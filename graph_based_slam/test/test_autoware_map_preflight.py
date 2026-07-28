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

import jsonschema
import pytest
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


def _monotonic_timestamp_inspection(
    _bag_path: Path,
    topics,
    _storage_id: str,
    max_records_per_topic: int,
) -> dict:
    return {
        'status': 'passed',
        'timestamp_source': 'header.stamp',
        'max_records_per_topic': max_records_per_topic,
        'failed_topics': [],
        'topics': [
            {
                'topic': topic.name,
                'msg_type': topic.msg_type,
                'expected_records': topic.message_count,
                'records_scanned': topic.message_count,
                'complete': True,
                'readable': True,
                'first_stamp_ns': 1_000_000_000,
                'last_stamp_ns': 2_000_000_000,
                'reversal_count': 0,
                'invalid_stamp_count': 0,
                'max_backward_jump_ns': 0,
            }
            for topic in topics
        ],
        'reason': (
            'All selected PointCloud2/Imu header timestamps are monotonic.'
        ),
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
        timestamp_inspector=_monotonic_timestamp_inspection,
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
    assert payload['summary']['pointcloud_inspection']['timestamp_field'] == 'time'
    assert payload['summary']['timestamp_order']['status'] == 'passed'
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
        timestamp_inspector=_monotonic_timestamp_inspection,
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


def test_timestamp_scan_detects_reversal_and_maximum_jump():
    module = _load_module()
    topic = module.TopicRecord(
        '/points',
        'sensor_msgs/msg/PointCloud2',
        4,
    )
    states = module._timestamp_scan_state([topic], 10)

    for stamp_ns in (1_000, 2_000, 1_500, 3_000):
        module._record_timestamp_sample(states['/points'], stamp_ns)
    result = module._finalize_timestamp_scan(
        states,
        exhausted=True,
        max_records_per_topic=10,
    )

    assert result['status'] == 'failed'
    assert result['failed_topics'] == ['/points']
    assert result['topics'][0]['records_scanned'] == 4
    assert result['topics'][0]['complete'] is True
    assert result['topics'][0]['reversal_count'] == 1
    assert result['topics'][0]['max_backward_jump_ns'] == 500


def test_timestamp_scan_distinguishes_bounded_sample_from_full_pass():
    module = _load_module()
    topic = module.TopicRecord(
        '/imu',
        'sensor_msgs/msg/Imu',
        100,
    )
    states = module._timestamp_scan_state([topic], 3)

    for stamp_ns in (1_000, 2_000, 3_000):
        module._record_timestamp_sample(states['/imu'], stamp_ns)
    result = module._finalize_timestamp_scan(
        states,
        exhausted=False,
        max_records_per_topic=3,
    )

    assert result['status'] == 'sampled'
    assert result['failed_topics'] == []
    assert result['topics'][0]['complete'] is False
    assert result['topics'][0]['readable'] is True


def test_timestamp_scan_rejects_invalid_and_unreadable_stamps():
    module = _load_module()
    points = module.TopicRecord(
        '/points',
        'sensor_msgs/msg/PointCloud2',
        1,
    )
    imu = module.TopicRecord('/imu', 'sensor_msgs/msg/Imu', 1)
    states = module._timestamp_scan_state([points, imu], 10)
    module._record_timestamp_sample(states['/points'], None)

    result = module._finalize_timestamp_scan(
        states,
        exhausted=True,
        max_records_per_topic=10,
    )

    assert result['status'] == 'failed'
    assert result['failed_topics'] == ['/points', '/imu']
    assert result['topics'][0]['invalid_stamp_count'] == 1
    assert result['topics'][1]['readable'] is False


def test_timestamp_reversal_blocks_affected_mapping_profile(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 200),
            ('/imu/data', 'sensor_msgs/msg/Imu', 2000),
        ],
    )

    def reversed_timestamps(
        _bag_path: Path,
        topics,
        _storage_id: str,
        max_records_per_topic: int,
    ) -> dict:
        result = _monotonic_timestamp_inspection(
            _bag_path,
            topics,
            _storage_id,
            max_records_per_topic,
        )
        result['status'] = 'failed'
        result['failed_topics'] = ['/points']
        result['topics'][0]['reversal_count'] = 2
        result['topics'][0]['max_backward_jump_ns'] = 750_000_000
        result['reason'] = (
            'Non-monotonic header timestamps were detected on /points.'
        )
        return result

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
        timestamp_inspector=reversed_timestamps,
    )

    assert payload['recommended_profile_id'] is None
    assert payload['beginner_commands'] == []
    assert any(
        'Header timestamp disorder on /points' in item
        and '0.750000000 s' in item
        for item in payload['missing_requirements']
    )
    report = module.render_text_report(payload)
    assert 'Header timestamp order: failed' in report


@pytest.mark.skipif(
    importlib.util.find_spec('rosbags') is None,
    reason='rosbags is required for the real rosbag2 fallback fixture',
)
def test_rosbags_reader_detects_reversal_in_serialized_records(
    tmp_path: Path,
):
    module = _load_module()
    if str(REPO_ROOT / 'scripts') not in sys.path:
        sys.path.insert(0, str(REPO_ROOT / 'scripts'))
    from mid360_robot_sample_bag import (  # noqa: PLC0415
        Mid360SampleBagWriter,
        SampleBagConfig,
    )
    from rosbags.highlevel import AnyReader  # noqa: PLC0415
    from rosbags.rosbag2 import Writer  # noqa: PLC0415
    from rosbags.typesys import Stores, get_typestore  # noqa: PLC0415

    source = tmp_path / 'source'
    reversed_bag = tmp_path / 'reversed'
    Mid360SampleBagWriter(
        SampleBagConfig(
            output_path=source,
            duration_sec=0.5,
            pointcloud_rate_hz=10.0,
            imu_rate_hz=100.0,
            force=True,
        )
    ).write()

    typestore = get_typestore(Stores.LATEST)
    previous_point_stamp = None
    point_index = 0
    with AnyReader(
        [source],
        default_typestore=typestore,
    ) as reader, Writer(reversed_bag, version=9) as writer:
        output_connections = {
            connection.topic: writer.add_connection(
                connection.topic,
                connection.msgtype,
                typestore=typestore,
            )
            for connection in reader.connections
        }
        for connection, receive_stamp_ns, serialized in reader.messages():
            if connection.msgtype == 'sensor_msgs/msg/PointCloud2':
                message = reader.deserialize(
                    serialized,
                    connection.msgtype,
                )
                header_stamp_ns = (
                    int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec)
                )
                if point_index == 3:
                    header_stamp_ns = previous_point_stamp - 50_000_000
                    message.header.stamp.sec = (
                        header_stamp_ns // 1_000_000_000
                    )
                    message.header.stamp.nanosec = (
                        header_stamp_ns % 1_000_000_000
                    )
                    serialized = typestore.serialize_cdr(
                        message,
                        connection.msgtype,
                    )
                previous_point_stamp = header_stamp_ns
                point_index += 1
            writer.write(
                output_connections[connection.topic],
                receive_stamp_ns,
                serialized,
            )

    payload = module.build_preflight_payload(reversed_bag)
    timestamp_order = payload['summary']['timestamp_order']

    assert payload['summary']['pointcloud_inspection'][
        'rko_lio_compatible'
    ] is True
    assert timestamp_order['status'] == 'failed'
    assert timestamp_order['failed_topics'] == ['/livox/lidar']
    points = next(
        topic
        for topic in timestamp_order['topics']
        if topic['topic'] == '/livox/lidar'
    )
    assert points['reversal_count'] == 1
    assert points['max_backward_jump_ns'] == 50_000_000
    assert payload['recommended_profile_id'] is None
