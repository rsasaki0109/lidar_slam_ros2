#!/usr/bin/env python3
"""Preflight a rosbag2 for Autoware-compatible map authoring workflows."""

import argparse
import json
import shlex
import textwrap
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import yaml


POINTCLOUD2 = 'sensor_msgs/msg/PointCloud2'
IMU = 'sensor_msgs/msg/Imu'
NAVSATFIX = 'sensor_msgs/msg/NavSatFix'
VELODYNE_SCAN = 'velodyne_msgs/msg/VelodyneScan'
TFMESSAGE = 'tf2_msgs/msg/TFMessage'
GSOF49 = 'applanix_msgs/msg/NavigationSolutionGsof49'
GSOF50 = 'applanix_msgs/msg/NavigationPerformanceGsof50'
VELOCITY_REPORT = 'autoware_auto_vehicle_msgs/msg/VelocityReport'


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
        raise FileNotFoundError(f'metadata.yaml not found under {bag_path}')
    data = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    bag_info = data.get('rosbag2_bagfile_information', {}) or {}
    if not bag_info:
        raise ValueError(f'rosbag2_bagfile_information missing in {metadata_path}')
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
            'velocity_report': [asdict(item) for item in _topic_group(topic_records, VELOCITY_REPORT)],
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

    best_pointcloud = summary['topics']['pointcloud2'][0] if summary['topics']['pointcloud2'] else None
    best_imu = summary['topics']['imu'][0] if summary['topics']['imu'] else None
    best_navsat = summary['topics']['navsatfix'][0] if summary['topics']['navsatfix'] else None
    best_packet = summary['topics']['velodyne_scan'][0] if summary['topics']['velodyne_scan'] else None
    best_gsof49 = summary['topics']['applanix_gsof49'][0] if summary['topics']['applanix_gsof49'] else None
    best_gsof50 = summary['topics']['applanix_gsof50'][0] if summary['topics']['applanix_gsof50'] else None
    bag_path_lower = bag_path.lower()

    def looks_like_livox_mid360() -> bool:
        topic_names = []
        for key in ('pointcloud2', 'imu'):
            topic_names.extend(item['name'].lower() for item in summary['topics'][key])
        return 'mid360' in bag_path_lower or any('livox' in name for name in topic_names)

    if capabilities['has_pointcloud2'] and capabilities['has_imu']:
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
                    f"PointCloud2 topic {best_pointcloud['name']} looks like a Livox/MID360 source",
                    'The repository tracks a tuned MID360 graph/backend YAML for this sensor family.',
                ],
                'command': tuned_command,
                'notes': [
                    'Use this when the bag is a Livox/MID360-style dataset and you want the tracked tuned preset instead of the generic default.',
                ],
            })

    if capabilities['has_pointcloud2'] and capabilities['has_navsatfix']:
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
                'This wrapper produces a verified pointcloud map with GNSS-enabled backend constraints.',
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
                'This wrapper converts packet and Applanix data into the maintained pointcloud-map path.',
            ],
            'command': command,
            'notes': [],
        })

    return sorted(recommendations, key=lambda item: item['priority'], reverse=True)


def build_preflight_payload(bag_path: Path) -> dict[str, Any]:
    """Create the machine-readable preflight result."""
    summary = summarize_bag(bag_path)
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

    missing = []
    if not summary['capabilities']['has_pointcloud2'] and not summary['capabilities']['has_velodyne_scan']:
        missing.append('No PointCloud2 or VelodyneScan topic was found.')
    if not recommendations:
        if not summary['capabilities']['has_imu']:
            missing.append('No Imu topic was found for the main RKO-LIO public path.')
        if not summary['capabilities']['has_navsatfix']:
            missing.append('No NavSatFix topic was found for the PointCloud2 + GNSS smoke path.')
        if not summary['capabilities']['has_applanix_gsof49']:
            missing.append('No Applanix GSOF49 topic was found for the packet + Applanix path.')

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
        'summary': summary,
        'recommendations': recommendations,
        'recommended_profile_id': recommendations[0]['id'] if recommendations else None,
        'beginner_commands': beginner_commands,
        'advisory': advisory,
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
        f"duration: {duration_text}",
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

    if recommendations:
        primary = recommendations[0]
        lines.extend([
            '',
            f"Recommended path: {primary['label']}",
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
                lines.append(f"  - {alternative['label']}")
                lines.append(textwrap.indent(alternative['command'], '    '))
    else:
        lines.extend([
            '',
            'Recommended path: none',
            'Why:',
        ])
        for item in payload['missing_requirements']:
            lines.append(f'  - {item}')

    if payload['advisory']:
        lines.append('')
        lines.append('Advisory commands:')
        for item in payload['advisory']:
            lines.append(f"  - {item['label']}")
            lines.append(textwrap.indent(item['command'], '    '))

    return '\n'.join(lines)


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Inspect a rosbag2 and suggest the shortest supported map-authoring path.'
    )
    parser.add_argument('bag', help='Path to a rosbag2 directory that contains metadata.yaml.')
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
    payload = build_preflight_payload(bag_path)
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(render_text_report(payload))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
