#!/usr/bin/env python3
"""Preflight a rosbag2 for Autoware-compatible map authoring workflows."""

import argparse
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import shlex
import sys
import textwrap
from typing import Any, Callable

import yaml

try:
    from product_profiles import PROFILE_HELP, select_profile
except ModuleNotFoundError as exc:
    if exc.name != 'product_profiles':
        raise
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    try:
        from product_profiles import PROFILE_HELP, select_profile
    finally:
        sys.path.pop(0)


POINTCLOUD2 = 'sensor_msgs/msg/PointCloud2'
IMU = 'sensor_msgs/msg/Imu'
NAVSATFIX = 'sensor_msgs/msg/NavSatFix'
VELODYNE_SCAN = 'velodyne_msgs/msg/VelodyneScan'
TFMESSAGE = 'tf2_msgs/msg/TFMessage'
GSOF49 = 'applanix_msgs/msg/NavigationSolutionGsof49'
GSOF50 = 'applanix_msgs/msg/NavigationPerformanceGsof50'
VELOCITY_REPORT = 'autoware_auto_vehicle_msgs/msg/VelocityReport'
SCHEMA_VERSION = 4
SCHEMA_URI = 'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/preflight-v4.schema.json'
POINT_FIELD_UINT32 = 6
POINT_FIELD_FLOAT32 = 7
POINT_FIELD_FLOAT64 = 8
RKO_TIMESTAMP_FIELDS = ('t', 'timestamp', 'time', 'stamps')
RKO_TIMESTAMP_DATATYPES = (
    POINT_FIELD_UINT32,
    POINT_FIELD_FLOAT32,
    POINT_FIELD_FLOAT64,
)
TIMESTAMP_SCAN_MSGTYPES = (POINTCLOUD2, IMU)
MAX_TIMESTAMP_RECORDS_PER_TOPIC = 100_000


def _finding(code: str, message: str, next_action: str) -> dict[str, str]:
    """Build one stable, actionable preflight rejection finding."""
    return {
        'code': code,
        'message': message,
        'next_action': next_action,
    }


@dataclass(frozen=True)
class TopicRecord:
    """Topic metadata relevant to bag preflight."""

    name: str
    msg_type: str
    message_count: int


def _safe_quote(value: str) -> str:
    return shlex.quote(value)


def _topic_from_entry(entry: dict[str, Any]) -> TopicRecord:
    topic_metadata = entry.get('topic_metadata', {}) or {}
    return TopicRecord(
        name=str(topic_metadata.get('name', '')),
        msg_type=str(topic_metadata.get('type', '')),
        message_count=int(entry.get('message_count', 0)),
    )


def load_bag_metadata(bag_path: Path) -> dict[str, Any]:
    """Load rosbag2 metadata.yaml."""
    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.is_file():
        raise FileNotFoundError(
            f'metadata.yaml not found under {bag_path}. '
            'Pass the rosbag2 directory that contains metadata.yaml.'
        )
    try:
        data = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    except yaml.YAMLError as exc:
        raise ValueError(f'failed to parse {metadata_path}: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError(
            f'{metadata_path} must contain a rosbag2 metadata YAML mapping.'
        )
    bag_info = data.get('rosbag2_bagfile_information', {}) or {}
    if not isinstance(bag_info, dict) or not bag_info:
        raise ValueError(
            f'rosbag2_bagfile_information missing in {metadata_path}. '
            'Use a rosbag2 metadata.yaml file, not a topic list or arbitrary YAML file.'
        )
    return bag_info


def _duration_seconds(bag_info: dict[str, Any]) -> float | None:
    duration = bag_info.get('duration', {}) or {}
    nanoseconds = duration.get('nanoseconds')
    if nanoseconds is None:
        return None
    return float(nanoseconds) / 1e9


def _collect_topics(bag_info: dict[str, Any]) -> list[TopicRecord]:
    topics = []
    for entry in bag_info.get('topics_with_message_count', []) or []:
        if not isinstance(entry, dict):
            continue
        record = _topic_from_entry(entry)
        if record.name and record.msg_type:
            topics.append(record)
    return topics


def _topic_group(records: list[TopicRecord], msg_type: str) -> list[TopicRecord]:
    return sorted(
        [record for record in records if record.msg_type == msg_type],
        key=lambda item: (-item.message_count, item.name),
    )


def _best_topic(records: list[TopicRecord], msg_type: str) -> TopicRecord | None:
    grouped = _topic_group(records, msg_type)
    return grouped[0] if grouped else None


def assess_pointcloud_fields(fields: list[dict[str, Any]]) -> dict[str, Any]:
    """Assess whether PointCloud2 fields satisfy RKO-LIO's reader contract."""
    normalized = [
        {
            'name': str(field.get('name', '')),
            'datatype': int(field.get('datatype', 0)),
            'count': int(field.get('count', 0)),
        }
        for field in fields
    ]
    by_name = {field['name']: field for field in normalized}

    invalid_xyz = [
        name
        for name in ('x', 'y', 'z')
        if name not in by_name
        or by_name[name]['datatype'] != POINT_FIELD_FLOAT32
        or by_name[name]['count'] <= 0
    ]
    if invalid_xyz:
        return {
            'rko_lio_compatible': False,
            'timestamp_field': None,
            'reason': (
                'RKO-LIO requires x, y, and z PointCloud2 fields with FLOAT32 '
                f'datatype and positive count; invalid or missing: {", ".join(invalid_xyz)}.'
            ),
        }

    timestamp_candidates = [
        by_name[name] for name in RKO_TIMESTAMP_FIELDS if name in by_name
    ]
    for field in timestamp_candidates:
        if (
            field['datatype'] in RKO_TIMESTAMP_DATATYPES
            and field['count'] > 0
        ):
            return {
                'rko_lio_compatible': True,
                'timestamp_field': field['name'],
                'reason': (
                    f'RKO-LIO-compatible per-point timestamp field '
                    f'{field["name"]!r} was found.'
                ),
            }

    expected = '/'.join(RKO_TIMESTAMP_FIELDS)
    if timestamp_candidates:
        return {
            'rko_lio_compatible': False,
            'timestamp_field': None,
            'reason': (
                f'PointCloud2 timestamp field must be one of {expected}, have '
                'positive count, and use UINT32, FLOAT32, or FLOAT64.'
            ),
        }
    return {
        'rko_lio_compatible': False,
        'timestamp_field': None,
        'reason': (
            'PointCloud2 has no per-point timestamp field required for '
            f'RKO-LIO deskewing (expected {expected}).'
        ),
    }


def inspect_pointcloud_record(
    bag_path: Path,
    topic: str,
    storage_id: str,
) -> dict[str, Any]:
    """Read the first selected PointCloud2 record and inspect its fields."""
    base = {
        'topic': topic,
        'frame_id': None,
        'fields': [],
        'rko_lio_compatible': None,
        'timestamp_field': None,
    }
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        return _inspect_pointcloud_record_with_rosbags(
            bag_path,
            topic,
            ros_import_error=exc,
        )

    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(
                uri=str(bag_path),
                storage_id=storage_id,
            ),
            rosbag2_py.ConverterOptions('', ''),
        )
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        while reader.has_next():
            record_topic, serialized, _ = reader.read_next()
            if record_topic != topic:
                continue
            message = deserialize_message(serialized, PointCloud2)
            fields = [
                {
                    'name': field.name,
                    'datatype': field.datatype,
                    'count': field.count,
                }
                for field in message.fields
            ]
            assessment = assess_pointcloud_fields(fields)
            return {
                **base,
                **assessment,
                'status': 'inspected',
                'frame_id': message.header.frame_id.strip() or None,
                'fields': fields,
            }
    except Exception as exc:  # rosbag2 storage plugins expose backend-specific errors
        return {
            **base,
            'status': 'error',
            'reason': f'Failed to inspect PointCloud2 record on {topic}: {exc}',
        }

    return {
        **base,
        'status': 'empty',
        'reason': f'No PointCloud2 record was found on selected topic {topic}.',
    }


def _timestamp_scan_state(
    topics: list[TopicRecord],
    max_records_per_topic: int,
) -> dict[str, dict[str, Any]]:
    return {
        topic.name: {
            'topic': topic.name,
            'msg_type': topic.msg_type,
            'expected_records': topic.message_count,
            'records_scanned': 0,
            'complete': False,
            'first_stamp_ns': None,
            'last_stamp_ns': None,
            'reversal_count': 0,
            'invalid_stamp_count': 0,
            'max_backward_jump_ns': 0,
            '_frame_ids': set(),
            '_limit': max_records_per_topic,
        }
        for topic in topics
    }


def _stamp_ns_from_message(message: Any) -> int | None:
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    sec = getattr(stamp, 'sec', None)
    nanosec = getattr(stamp, 'nanosec', None)
    if sec is None or nanosec is None:
        return None
    sec = int(sec)
    nanosec = int(nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return None
    return sec * 1_000_000_000 + nanosec


def _record_timestamp_sample(
    state: dict[str, Any],
    stamp_ns: int | None,
    frame_id: str | None = None,
) -> None:
    if state['records_scanned'] >= state['_limit']:
        return
    state['records_scanned'] += 1
    if frame_id and frame_id.strip():
        state['_frame_ids'].add(frame_id.strip())
    if stamp_ns is None:
        state['invalid_stamp_count'] += 1
        return
    if state['first_stamp_ns'] is None:
        state['first_stamp_ns'] = stamp_ns
    previous = state['last_stamp_ns']
    if previous is not None and stamp_ns < previous:
        state['reversal_count'] += 1
        state['max_backward_jump_ns'] = max(
            state['max_backward_jump_ns'],
            previous - stamp_ns,
        )
    state['last_stamp_ns'] = stamp_ns


def _timestamp_scan_satisfied(state: dict[str, Any]) -> bool:
    expected = state['expected_records']
    goal = min(
        expected if expected > 0 else state['_limit'],
        state['_limit'],
    )
    return state['records_scanned'] >= goal


def _finalize_timestamp_scan(
    states: dict[str, dict[str, Any]],
    *,
    exhausted: bool,
    max_records_per_topic: int,
) -> dict[str, Any]:
    topics = []
    failed_topics = []
    for state in states.values():
        expected = state['expected_records']
        state['complete'] = (
            exhausted
            or (expected > 0 and state['records_scanned'] >= expected)
        )
        state['readable'] = state['records_scanned'] > 0
        frame_ids = sorted(state.pop('_frame_ids'))
        state['frame_id'] = frame_ids[0] if len(frame_ids) == 1 else None
        state['frame_ids'] = frame_ids
        state.pop('_limit')
        if (
            not state['readable']
            or state['reversal_count']
            or state['invalid_stamp_count']
        ):
            failed_topics.append(state['topic'])
        topics.append(state)

    if failed_topics:
        status = 'failed'
        reason = (
            'Unreadable, non-monotonic, or invalid header timestamps '
            'were detected on: '
            + ', '.join(failed_topics)
            + '. Correct or rewrite the bag before mapping.'
        )
    elif all(topic['complete'] for topic in topics):
        status = 'passed'
        reason = 'All selected PointCloud2/Imu header timestamps are monotonic.'
    else:
        status = 'sampled'
        reason = (
            'No reversal was found in the bounded header timestamp sample; '
            'at least one selected topic was not scanned completely.'
        )
    return {
        'status': status,
        'timestamp_source': 'header.stamp',
        'max_records_per_topic': max_records_per_topic,
        'failed_topics': failed_topics,
        'topics': topics,
        'reason': reason,
    }


def inspect_timestamp_order(
    bag_path: Path,
    topics: list[TopicRecord],
    storage_id: str,
    max_records_per_topic: int = MAX_TIMESTAMP_RECORDS_PER_TOPIC,
) -> dict[str, Any]:
    """Boundedly inspect PointCloud2/Imu header timestamps before launch."""
    selected = [
        topic for topic in topics
        if topic.msg_type in TIMESTAMP_SCAN_MSGTYPES
    ]
    if not selected:
        return {
            'status': 'not_applicable',
            'timestamp_source': 'header.stamp',
            'max_records_per_topic': max_records_per_topic,
            'failed_topics': [],
            'topics': [],
            'reason': 'No PointCloud2 or Imu topic was selected for inspection.',
        }
    if max_records_per_topic <= 0:
        raise ValueError('max_records_per_topic must be positive')

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu, PointCloud2
    except ImportError as exc:
        return _inspect_timestamp_order_with_rosbags(
            bag_path,
            selected,
            max_records_per_topic,
            ros_import_error=exc,
        )

    message_types = {
        POINTCLOUD2: PointCloud2,
        IMU: Imu,
    }
    states = _timestamp_scan_state(selected, max_records_per_topic)
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(
                uri=str(bag_path),
                storage_id=storage_id,
            ),
            rosbag2_py.ConverterOptions('', ''),
        )
        reader.set_filter(
            rosbag2_py.StorageFilter(topics=list(states))
        )
        exhausted = True
        while reader.has_next():
            if all(_timestamp_scan_satisfied(state) for state in states.values()):
                exhausted = False
                break
            topic, serialized, _ = reader.read_next()
            state = states.get(topic)
            if state is None or _timestamp_scan_satisfied(state):
                continue
            message = deserialize_message(
                serialized,
                message_types[state['msg_type']],
            )
            _record_timestamp_sample(
                state,
                _stamp_ns_from_message(message),
                getattr(getattr(message, 'header', None), 'frame_id', None),
            )
        return _finalize_timestamp_scan(
            states,
            exhausted=exhausted,
            max_records_per_topic=max_records_per_topic,
        )
    except Exception as exc:  # storage plugins expose backend-specific errors
        return {
            'status': 'error',
            'timestamp_source': 'header.stamp',
            'max_records_per_topic': max_records_per_topic,
            'failed_topics': [],
            'topics': [],
            'reason': f'Failed to inspect header timestamp order: {exc}',
        }


def _inspect_timestamp_order_with_rosbags(
    bag_path: Path,
    topics: list[TopicRecord],
    max_records_per_topic: int,
    ros_import_error: ImportError,
) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as rosbags_error:
        return {
            'status': 'unavailable',
            'timestamp_source': 'header.stamp',
            'max_records_per_topic': max_records_per_topic,
            'failed_topics': [],
            'topics': [],
            'reason': (
                'Header timestamp inspection is unavailable: ROS 2 Python '
                f'bindings failed to import ({ros_import_error}); the rosbags '
                f'fallback also failed to import ({rosbags_error}).'
            ),
        }

    states = _timestamp_scan_state(topics, max_records_per_topic)
    try:
        typestore = get_typestore(Stores.LATEST)
        with AnyReader(
            [bag_path],
            default_typestore=typestore,
        ) as reader:
            connections = [
                connection
                for connection in reader.connections
                if connection.topic in states
                and connection.msgtype == states[connection.topic]['msg_type']
            ]
            exhausted = True
            for connection, _, serialized in reader.messages(
                connections=connections,
            ):
                if all(
                    _timestamp_scan_satisfied(state)
                    for state in states.values()
                ):
                    exhausted = False
                    break
                state = states[connection.topic]
                if _timestamp_scan_satisfied(state):
                    continue
                message = reader.deserialize(
                    serialized,
                    connection.msgtype,
                )
                _record_timestamp_sample(
                    state,
                    _stamp_ns_from_message(message),
                    getattr(getattr(message, 'header', None), 'frame_id', None),
                )
        return _finalize_timestamp_scan(
            states,
            exhausted=exhausted,
            max_records_per_topic=max_records_per_topic,
        )
    except Exception as exc:
        return {
            'status': 'error',
            'timestamp_source': 'header.stamp',
            'max_records_per_topic': max_records_per_topic,
            'failed_topics': [],
            'topics': [],
            'reason': (
                'Failed to inspect header timestamp order with rosbags: '
                f'{exc}'
            ),
        }


def _inspect_pointcloud_record_with_rosbags(
    bag_path: Path,
    topic: str,
    ros_import_error: ImportError,
) -> dict[str, Any]:
    """Use the pure-Python rosbags reader when ROS Python bindings are absent."""
    base = {
        'topic': topic,
        'frame_id': None,
        'fields': [],
        'rko_lio_compatible': None,
        'timestamp_field': None,
    }
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as rosbags_error:
        return {
            **base,
            'status': 'unavailable',
            'reason': (
                'PointCloud2 record inspection is unavailable: ROS 2 Python '
                f'bindings failed to import ({ros_import_error}); the rosbags '
                f'fallback also failed to import ({rosbags_error}). Source ROS 2 '
                'or install the Python rosbags package before selecting an '
                'RKO-LIO profile.'
            ),
        }

    try:
        typestore = get_typestore(Stores.LATEST)
        with AnyReader(
            [bag_path],
            default_typestore=typestore,
        ) as reader:
            connections = [
                connection
                for connection in reader.connections
                if connection.topic == topic and connection.msgtype == POINTCLOUD2
            ]
            for connection, _, serialized in reader.messages(
                connections=connections,
            ):
                message = reader.deserialize(serialized, connection.msgtype)
                fields = [
                    {
                        'name': field.name,
                        'datatype': int(field.datatype),
                        'count': int(field.count),
                    }
                    for field in message.fields
                ]
                assessment = assess_pointcloud_fields(fields)
                return {
                    **base,
                    **assessment,
                    'status': 'inspected',
                    'frame_id': str(message.header.frame_id).strip() or None,
                    'fields': fields,
                }
    except Exception as exc:
        return {
            **base,
            'status': 'error',
            'reason': (
                f'Failed to inspect PointCloud2 record on {topic} with rosbags: '
                f'{exc}'
            ),
        }

    return {
        **base,
        'status': 'empty',
        'reason': f'No PointCloud2 record was found on selected topic {topic}.',
    }


def summarize_bag(bag_path: Path) -> dict[str, Any]:
    """Summarize a rosbag2 in terms of map-authoring inputs."""
    bag_info = load_bag_metadata(bag_path)
    topic_records = _collect_topics(bag_info)
    summary = {
        'bag_path': str(bag_path),
        'duration_sec': _duration_seconds(bag_info),
        'message_count': int(bag_info.get('message_count', 0)),
        'topics': {
            'pointcloud2': [asdict(item) for item in _topic_group(topic_records, POINTCLOUD2)],
            'imu': [asdict(item) for item in _topic_group(topic_records, IMU)],
            'navsatfix': [asdict(item) for item in _topic_group(topic_records, NAVSATFIX)],
            'velodyne_scan': [asdict(item) for item in _topic_group(topic_records, VELODYNE_SCAN)],
            'applanix_gsof49': [asdict(item) for item in _topic_group(topic_records, GSOF49)],
            'applanix_gsof50': [asdict(item) for item in _topic_group(topic_records, GSOF50)],
            'tf': [asdict(item) for item in _topic_group(topic_records, TFMESSAGE)],
            'velocity_report': [
                asdict(item)
                for item in _topic_group(topic_records, VELOCITY_REPORT)
            ],
        },
    }

    summary['capabilities'] = {
        'has_pointcloud2': bool(summary['topics']['pointcloud2']),
        'has_imu': bool(summary['topics']['imu']),
        'has_navsatfix': bool(summary['topics']['navsatfix']),
        'has_velodyne_scan': bool(summary['topics']['velodyne_scan']),
        'has_applanix_gsof49': bool(summary['topics']['applanix_gsof49']),
        'has_applanix_gsof50': bool(summary['topics']['applanix_gsof50']),
        'has_tf': bool(summary['topics']['tf']),
        'has_velocity_report': bool(summary['topics']['velocity_report']),
    }
    return summary


def build_recommendations(summary: dict[str, Any]) -> list[dict[str, Any]]:
    """Build compatible workflow recommendations for the bag."""
    bag_path = summary['bag_path']
    bag_q = _safe_quote(bag_path)
    capabilities = summary['capabilities']
    recommendations: list[dict[str, Any]] = []

    best_pointcloud = (
        summary['topics']['pointcloud2'][0]
        if summary['topics']['pointcloud2'] else None
    )
    best_imu = summary['topics']['imu'][0] if summary['topics']['imu'] else None
    best_navsat = summary['topics']['navsatfix'][0] if summary['topics']['navsatfix'] else None
    best_packet = (
        summary['topics']['velodyne_scan'][0]
        if summary['topics']['velodyne_scan'] else None
    )
    best_gsof49 = (
        summary['topics']['applanix_gsof49'][0]
        if summary['topics']['applanix_gsof49'] else None
    )
    best_gsof50 = (
        summary['topics']['applanix_gsof50'][0]
        if summary['topics']['applanix_gsof50'] else None
    )
    bag_path_lower = bag_path.lower()

    def looks_like_livox_mid360() -> bool:
        topic_names = []
        for key in ('pointcloud2', 'imu'):
            topic_names.extend(item['name'].lower() for item in summary['topics'][key])
        return 'mid360' in bag_path_lower or any('livox' in name for name in topic_names)

    pointcloud_inspection = summary['pointcloud_inspection']
    timestamp_order_ready = (
        summary['timestamp_order']['status'] in {'passed', 'sampled'}
    )
    failed_timestamp_topics = set(
        summary['timestamp_order']['failed_topics']
    )
    if (
        capabilities['has_pointcloud2']
        and capabilities['has_imu']
        and pointcloud_inspection['rko_lio_compatible'] is True
        and timestamp_order_ready
        and best_pointcloud['name'] not in failed_timestamp_topics
        and best_imu['name'] not in failed_timestamp_topics
    ):
        command = textwrap.dedent(
            f"""\
            ros2 launch lidarslam rko_lio_slam.launch.py \\
              bag_path:={bag_q} \\
              lidar_topic:={_safe_quote(best_pointcloud['name'])} \\
              imu_topic:={_safe_quote(best_imu['name'])}"""
        )
        notes = []
        if capabilities['has_navsatfix']:
            notes.append(
                'GNSS is present in the bag. Inspect covariance before enabling backend '
                'GNSS weighting.'
            )
        recommendations.append({
            'id': 'rko_lio_graph_public_path',
            'priority': 100,
            'label': 'RKO-LIO + graph_based_slam public path',
            'why': [
                f"PointCloud2 is available on {best_pointcloud['name']}",
                f"Imu is available on {best_imu['name']}",
                (
                    'The first PointCloud2 record has RKO-LIO-compatible '
                    f"per-point timestamps in {pointcloud_inspection['timestamp_field']!r}."
                ),
                'This is the main maintained map-authoring path in the repository.',
            ],
            'command': command,
            'notes': notes,
        })

        if looks_like_livox_mid360():
            tuned_command = textwrap.dedent(
                f"""\
                ros2 launch lidarslam rko_lio_slam.launch.py \\
                  main_param_dir:=lidarslam/param/lidarslam_mid360_rko_graph.yaml \\
                  rko_param_file:=lidarslam/param/rko_lio_mid360.yaml \\
                  bag_path:={bag_q} \\
                  lidar_topic:={_safe_quote(best_pointcloud['name'])} \\
                  imu_topic:={_safe_quote(best_imu['name'])}"""
            )
            recommendations.append({
                'id': 'rko_lio_graph_mid360_preset',
                'priority': 95,
                'label': 'RKO-LIO + graph_based_slam MID360/Livox preset',
                'why': [
                    (
                        f"PointCloud2 topic {best_pointcloud['name']} looks like "
                        'a Livox/MID360 source'
                    ),
                    (
                        'The repository tracks a tuned MID360 graph/backend YAML '
                        'for this sensor family.'
                    ),
                ],
                'command': tuned_command,
                'notes': [
                    (
                        'Use this when the bag is a Livox/MID360-style dataset and '
                        'you want the tracked tuned preset instead of the generic '
                        'default.'
                    ),
                ],
            })

    if (
        capabilities['has_pointcloud2']
        and capabilities['has_navsatfix']
        and timestamp_order_ready
        and best_pointcloud['name'] not in failed_timestamp_topics
    ):
        command = (
            'bash scripts/run_open_data_gnss_smoke.sh '
            f'--bag {bag_q} '
            f'--points-topic {_safe_quote(best_pointcloud["name"])} '
            f'--gnss-topic {_safe_quote(best_navsat["name"])}'
        )
        if capabilities['has_imu']:
            command += f' --imu-topic {_safe_quote(best_imu["name"])}'
        recommendations.append({
            'id': 'pointcloud_gnss_smoke',
            'priority': 80,
            'label': 'PointCloud2 + GNSS smoke path',
            'why': [
                f"PointCloud2 is available on {best_pointcloud['name']}",
                f"NavSatFix is available on {best_navsat['name']}",
                (
                    'This wrapper produces a verified pointcloud map with '
                    'GNSS-enabled backend constraints.'
                ),
            ],
            'command': command,
            'notes': [],
        })

    if capabilities['has_velodyne_scan'] and capabilities['has_applanix_gsof49']:
        command = (
            'bash scripts/run_open_data_applanix_velodyne_gnss_smoke.sh '
            f'--bag {bag_q} '
            f'--packet-topic {_safe_quote(best_packet["name"])}'
        )
        if best_gsof49 is not None:
            command += f' --gsof49-topic {_safe_quote(best_gsof49["name"])}'
        if best_gsof50 is not None:
            command += f' --gsof50-topic {_safe_quote(best_gsof50["name"])}'
        recommendations.append({
            'id': 'packet_applanix_smoke',
            'priority': 70,
            'label': 'Velodyne packet + Applanix smoke path',
            'why': [
                f"VelodyneScan is available on {best_packet['name']}",
                f"Applanix GSOF49 is available on {best_gsof49['name']}",
                (
                    'This wrapper converts packet and Applanix data into the '
                    'maintained pointcloud-map path.'
                ),
            ],
            'command': command,
            'notes': [],
        })

    return sorted(recommendations, key=lambda item: item['priority'], reverse=True)


def build_preflight_payload(
    bag_path: Path,
    pointcloud_inspector: Callable[[Path, str, str], dict[str, Any]] | None = None,
    timestamp_inspector: Callable[
        [Path, list[TopicRecord], str, int],
        dict[str, Any],
    ] | None = None,
    max_timestamp_records_per_topic: int = MAX_TIMESTAMP_RECORDS_PER_TOPIC,
) -> dict[str, Any]:
    """Create the machine-readable preflight result."""
    summary = summarize_bag(bag_path)
    bag_info = load_bag_metadata(bag_path)
    storage_id = str(bag_info.get('storage_identifier', ''))
    if summary['topics']['pointcloud2']:
        topic = summary['topics']['pointcloud2'][0]['name']
        inspector = pointcloud_inspector or inspect_pointcloud_record
        summary['pointcloud_inspection'] = inspector(
            bag_path,
            topic,
            storage_id,
        )
    else:
        summary['pointcloud_inspection'] = {
            'status': 'not_applicable',
            'topic': None,
            'fields': [],
            'rko_lio_compatible': None,
            'timestamp_field': None,
            'reason': 'No PointCloud2 topic was found.',
        }
    timestamp_topics = []
    for msg_type in TIMESTAMP_SCAN_MSGTYPES:
        topic = _best_topic(_collect_topics(bag_info), msg_type)
        if topic is not None:
            timestamp_topics.append(topic)
    order_inspector = timestamp_inspector or inspect_timestamp_order
    summary['timestamp_order'] = order_inspector(
        bag_path,
        timestamp_topics,
        storage_id,
        max_timestamp_records_per_topic,
    )
    recommendations = build_recommendations(summary)
    bag_q = _safe_quote(summary['bag_path'])
    advisory = []
    if summary['capabilities']['has_navsatfix']:
        navsat_topic = summary['topics']['navsatfix'][0]['name']
        advisory.append({
            'label': 'Inspect NavSatFix covariance',
            'command': (
                'python3 scripts/inspect_navsatfix_covariance.py '
                f'{_safe_quote(summary["bag_path"])} --topic {_safe_quote(navsat_topic)}'
            ),
        })
    if summary['capabilities']['has_applanix_gsof50']:
        gsof50_topic = summary['topics']['applanix_gsof50'][0]['name']
        advisory.append({
            'label': 'Inspect Applanix GSOF50 quality',
            'command': (
                'python3 scripts/inspect_applanix_gsof50_quality.py '
                f'{_safe_quote(summary["bag_path"])} --topic {_safe_quote(gsof50_topic)}'
            ),
        })

    findings = []
    if (
        not summary['capabilities']['has_pointcloud2']
        and not summary['capabilities']['has_velodyne_scan']
    ):
        findings.append(_finding(
            'range-input-missing',
            'No PointCloud2 or VelodyneScan topic was found.',
            'Record or convert a sensor_msgs/msg/PointCloud2 or '
            'velodyne_msgs/msg/VelodyneScan topic, then rerun '
            'lidarslam-map doctor <rosbag2_dir>.',
        ))
    inspection = summary['pointcloud_inspection']
    if (
        summary['capabilities']['has_pointcloud2']
        and summary['capabilities']['has_imu']
        and inspection['rko_lio_compatible'] is not True
    ):
        if inspection['status'] in {'error', 'unavailable'}:
            findings.append(_finding(
                'pointcloud-inspection-unavailable',
                f"PointCloud2 topic {inspection['topic']} could not be inspected: "
                f"{inspection['reason']}",
                'Run ros2 bag info <rosbag2_dir>; repair invalid metadata or '
                'install the reported storage/message support, then rerun '
                'lidarslam-map doctor <rosbag2_dir>.',
            ))
        elif inspection['status'] == 'empty':
            findings.append(_finding(
                'pointcloud-record-missing',
                f"PointCloud2 topic {inspection['topic']} had no readable record: "
                f"{inspection['reason']}",
                'Record at least one readable PointCloud2 message, then rerun '
                'lidarslam-map doctor <rosbag2_dir>.',
            ))
        else:
            findings.append(_finding(
                'pointcloud-layout-incompatible',
                f"PointCloud2 topic {inspection['topic']} is not verified as compatible "
                f"with RKO-LIO: {inspection['reason']}",
                'Rewrite the selected PointCloud2 with x/y/z fields and a supported '
                'per-point timestamp field (t, timestamp, time, or stamps), then '
                'rerun lidarslam-map doctor <rosbag2_dir>.',
            ))
    timestamp_order = summary['timestamp_order']
    if timestamp_order['status'] in {'error', 'unavailable'}:
        findings.append(_finding(
            'timestamp-inspection-unavailable',
            'Header timestamp order could not be inspected before launch: '
            f"{timestamp_order['reason']}",
            'Run ros2 bag info <rosbag2_dir>; repair invalid metadata or install '
            'the reported storage/message support, then rerun '
            'lidarslam-map doctor <rosbag2_dir>.',
        ))
    if timestamp_order['status'] == 'failed':
        for topic in timestamp_order['topics']:
            if not topic['readable']:
                findings.append(_finding(
                    'timestamp-unreadable',
                    f"Header timestamps on {topic['topic']} could not be read. "
                    'Correct or rewrite the bag before mapping.',
                    f"Repair or rewrite header.stamp on {topic['topic']}, then "
                    'rerun lidarslam-map doctor <rosbag2_dir>.',
                ))
            elif topic['reversal_count'] or topic['invalid_stamp_count']:
                findings.append(_finding(
                    'timestamp-order-invalid',
                    f"Header timestamp disorder on {topic['topic']}: "
                    f"{topic['reversal_count']} reversal(s), "
                    f"{topic['invalid_stamp_count']} invalid stamp(s), "
                    f"maximum backward jump "
                    f"{topic['max_backward_jump_ns'] / 1e9:.9f} s. "
                    'Correct or rewrite the bag before mapping.',
                    f"Rewrite header.stamp on {topic['topic']} so it is valid and "
                    'monotonic, then rerun lidarslam-map doctor <rosbag2_dir>.',
                ))
    if not recommendations:
        if not summary['capabilities']['has_imu']:
            findings.append(_finding(
                'imu-input-missing',
                'No Imu topic was found for the main RKO-LIO public path.',
                'Record a sensor_msgs/msg/Imu topic synchronized with the '
                'PointCloud2 input, then rerun lidarslam-map doctor '
                '<rosbag2_dir>.',
            ))
        if not summary['capabilities']['has_navsatfix']:
            findings.append(_finding(
                'navsatfix-input-missing',
                'No NavSatFix topic was found for the PointCloud2 + GNSS smoke path.',
                'If using the GNSS smoke path, record a sensor_msgs/msg/NavSatFix '
                'topic, then rerun lidarslam-map doctor <rosbag2_dir>.',
            ))
        if not summary['capabilities']['has_applanix_gsof49']:
            findings.append(_finding(
                'applanix-gsof49-input-missing',
                'No Applanix GSOF49 topic was found for the packet + Applanix path.',
                'If using the packet path, record an Applanix GSOF49 navigation '
                'topic, then rerun lidarslam-map doctor <rosbag2_dir>.',
            ))

    missing = [finding['message'] for finding in findings]

    beginner_commands = []
    if recommendations:
        beginner_commands = [
            {
                'label': 'Beginner one-command path',
                'command': f'bash scripts/run_autoware_map_beginner.sh {bag_q}',
            },
            {
                'label': 'Beginner path with Foxglove viewer',
                'command': f'bash scripts/run_autoware_map_beginner.sh {bag_q} --foxglove',
            },
            {
                'label': 'Beginner dry-run to inspect the chosen public path',
                'command': f'bash scripts/run_autoware_map_beginner.sh {bag_q} --dry-run',
            },
        ]

    return {
        'schema_version': SCHEMA_VERSION,
        'schema_uri': SCHEMA_URI,
        'summary': summary,
        'recommendations': recommendations,
        'recommended_profile_id': recommendations[0]['id'] if recommendations else None,
        'beginner_commands': beginner_commands,
        'advisory': advisory,
        'findings': findings,
        'missing_requirements': missing,
    }


def _format_topic_list(records: list[dict[str, Any]]) -> str:
    if not records:
        return 'none'
    rendered = [f"{record['name']} ({record['message_count']})" for record in records[:3]]
    remaining = len(records) - min(len(records), 3)
    if remaining > 0:
        rendered.append(f'+{remaining} more')
    return ', '.join(rendered)


def render_text_report(payload: dict[str, Any]) -> str:
    """Render a human-readable preflight report."""
    summary = payload['summary']
    recommendations = payload['recommendations']
    duration_sec = summary['duration_sec']
    duration_text = f'{duration_sec:.3f}s' if duration_sec is not None else 'unknown'

    lines = [
        'Autoware-Compatible Map Preflight',
        f"bag: {summary['bag_path']}",
        f'duration: {duration_text}',
        f"messages: {summary['message_count']}",
        '',
        'Detected inputs:',
        f"  PointCloud2: {_format_topic_list(summary['topics']['pointcloud2'])}",
        f"  Imu: {_format_topic_list(summary['topics']['imu'])}",
        f"  NavSatFix: {_format_topic_list(summary['topics']['navsatfix'])}",
        f"  VelodyneScan: {_format_topic_list(summary['topics']['velodyne_scan'])}",
        f"  Applanix GSOF49: {_format_topic_list(summary['topics']['applanix_gsof49'])}",
        f"  Applanix GSOF50: {_format_topic_list(summary['topics']['applanix_gsof50'])}",
        f"  TF/TF_STATIC: {_format_topic_list(summary['topics']['tf'])}",
        f"  VelocityReport: {_format_topic_list(summary['topics']['velocity_report'])}",
    ]
    inspection = summary['pointcloud_inspection']
    if inspection['status'] != 'not_applicable':
        lines.append(
            '  PointCloud2 record: '
            f"{inspection['status']} ({inspection['reason']})"
        )
    timestamp_order = summary['timestamp_order']
    lines.append(
        '  Header timestamp order: '
        f"{timestamp_order['status']} ({timestamp_order['reason']})"
    )

    if recommendations:
        primary = recommendations[0]
        lines.extend([
            '',
            f"Recommended path: {primary['label']}",
            f"Recommended profile: {primary['id']}",
            'Why:',
        ])
        for reason in primary['why']:
            lines.append(f'  - {reason}')
        lines.extend([
            'Beginner command:',
            textwrap.indent(payload['beginner_commands'][0]['command'], '  '),
            'Beginner command with browser viewer:',
            textwrap.indent(payload['beginner_commands'][1]['command'], '  '),
            'Next command:',
            textwrap.indent(primary['command'], '  '),
        ])

        if len(recommendations) > 1:
            lines.append('')
            lines.append('Other compatible paths:')
            for alternative in recommendations[1:]:
                lines.append(f"  - {alternative['label']} [{alternative['id']}]")
                lines.append(textwrap.indent(alternative['command'], '    '))
    else:
        lines.extend([
            '',
            'Recommended path: none',
            'Findings:',
        ])
        findings = payload.get('findings') or []
        if findings:
            for finding in findings:
                lines.append(f"  [{finding['code']}] {finding['message']}")
                lines.append(f"    Next: {finding['next_action']}")
        else:
            for item in payload['missing_requirements']:
                lines.append(f'  - {item}')

    if payload['advisory']:
        lines.append('')
        lines.append('Advisory commands:')
        for item in payload['advisory']:
            lines.append(f"  - {item['label']}")
            lines.append(textwrap.indent(item['command'], '    '))

    return '\n'.join(lines)


def validate_bag_path(bag_path: Path) -> None:
    """Validate that a CLI input points at a rosbag2 directory."""
    if bag_path.is_file():
        if bag_path.suffix == '.db3':
            raise FileNotFoundError(
                f'rosbag2 path points to a .db3 file: {bag_path}. '
                'Pass the rosbag2 directory that contains metadata.yaml, not the .db3 file.'
            )
        raise FileNotFoundError(
            f'rosbag2 path is a file, not a directory: {bag_path}. '
            'Pass the rosbag2 directory that contains metadata.yaml.'
        )
    if not bag_path.exists():
        raise FileNotFoundError(
            f'rosbag2 directory does not exist: {bag_path}. '
            'Pass the directory that contains metadata.yaml.'
        )
    if not bag_path.is_dir():
        raise FileNotFoundError(
            f'rosbag2 path is not a directory: {bag_path}. '
            'Pass the directory that contains metadata.yaml.'
        )
    if not (bag_path / 'metadata.yaml').is_file():
        raise FileNotFoundError(
            f'metadata.yaml not found under {bag_path}. '
            'Pass the rosbag2 directory that contains metadata.yaml.'
        )


def _profile_help_text() -> str:
    lines = ['Profiles this tool can recommend:']
    for profile_id, description in PROFILE_HELP:
        lines.append(f'  {profile_id}: {description}')
    return '\n'.join(lines)


def _help_epilog() -> str:
    command = os.environ.get(
        'LIDARSLAM_CLI_COMMAND',
        'python3 scripts/preflight_autoware_map_bag.py',
    )
    product_command = (
        command.rsplit(' ', 1)[0]
        if 'LIDARSLAM_CLI_COMMAND' in os.environ
        else 'lidarslam-map'
    )
    return '\n'.join([
        'The input must be the rosbag2 directory that contains metadata.yaml.',
        'Pass /path/to/rosbag2, not /path/to/rosbag2_0.db3.',
        '',
        _profile_help_text(),
        '',
        'Typical commands:',
        f'  {command} /path/to/rosbag2',
        f'  {command} /path/to/rosbag2 --json',
        f'  {product_command} run /path/to/rosbag2 --dry-run',
    ])


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Inspect a rosbag2 directory and suggest the shortest supported '
            'Autoware-compatible map workflow.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_help_epilog(),
    )
    parser.add_argument(
        'bag',
        metavar='rosbag2_dir',
        help='Directory containing metadata.yaml.',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show all options (this command has no advanced options).',
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Emit machine-readable JSON instead of the human report.',
    )
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    try:
        validate_bag_path(bag_path)
        payload = build_preflight_payload(bag_path)
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(render_text_report(payload))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
