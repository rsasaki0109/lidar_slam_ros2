#!/usr/bin/env python3
"""Shared MID-360 robot preflight and run-planning helpers."""

from __future__ import annotations

import importlib.util
import json
import shlex
import textwrap
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


MID360_PROFILE_ID = 'rko_lio_graph_mid360_preset'
DEFAULT_SAMPLE_MESSAGES = 20
POINTCLOUD_MIN_METADATA_HZ = 5.0
IMU_MIN_METADATA_HZ = 50.0


@dataclass(frozen=True)
class RobotFrames:
    """Frame names used by the robot mapping launch."""

    base_frame: str = 'base_link'
    lidar_frame: str = 'livox_frame'
    imu_frame: str = 'livox_frame'


@dataclass(frozen=True)
class RobotProfile:
    """Robot-specific MID-360 mapping defaults."""

    robot_name: str
    frames: RobotFrames
    expected_pointcloud_topic: str = ''
    expected_imu_topic: str = ''
    mount: dict[str, Any] | None = None
    source_path: str = ''

    def to_dict(self) -> dict[str, Any]:
        return {
            'robot_name': self.robot_name,
            'frames': asdict(self.frames),
            'expected_pointcloud_topic': self.expected_pointcloud_topic,
            'expected_imu_topic': self.expected_imu_topic,
            'mount': self.mount or {},
            'source_path': self.source_path,
        }


@dataclass(frozen=True)
class TopicSelection:
    """Topic names selected from rosbag2 metadata."""

    pointcloud: str | None
    imu: str | None

    @property
    def ready(self) -> bool:
        return bool(self.pointcloud and self.imu)


@dataclass(frozen=True)
class MessageSample:
    """Small, standard-message sample extracted from a rosbag2 topic."""

    topic: str
    msg_type: str
    timestamp_ns: int | None = None
    header_stamp_ns: int | None = None
    frame_id: str = ''
    tf_pairs: tuple[tuple[str, str], ...] = ()


@dataclass(frozen=True)
class PreflightCheck:
    """A single robot preflight check."""

    id: str
    status: str
    message: str


@dataclass(frozen=True)
class MapRunOptions:
    """Options that affect the MID-360 map runner command."""

    output_dir: Path | None = None
    run_name: str = ''
    save_timeout_secs: str = ''
    startup_timeout_secs: str = ''
    viewer: str = 'none'
    viewer_rebuild: bool = False
    viewer_run_dir: str = ''
    autoware_core_dir: str = ''
    work_dir: str = ''
    auto_exit_secs: str = ''
    keep_launch: bool = False


@dataclass(frozen=True)
class MapRunPlan:
    """Commands needed for one MID-360 map run."""

    output_dir: Path
    dogfood_command: list[str]
    foxglove_command: list[str]

    def to_dict(self) -> dict[str, Any]:
        return {
            'output_dir': str(self.output_dir),
            'dogfood_command': self.dogfood_command,
            'dogfood_command_shell': shlex.join(self.dogfood_command),
            'foxglove_command': self.foxglove_command,
            'foxglove_command_shell': (
                shlex.join(self.foxglove_command) if self.foxglove_command else ''
            ),
        }


@dataclass(frozen=True)
class DiagnosisPlan:
    """Command and expected outputs for map-run diagnosis."""

    command: list[str]
    markdown_path: Path
    json_path: Path
    ran: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            'command': self.command,
            'command_shell': shlex.join(self.command),
            'markdown_path': str(self.markdown_path),
            'json_path': str(self.json_path),
            'ran': self.ran,
        }


class Mid360RunDiagnosisPlanner:
    """Build diagnosis commands for completed MID-360 map runs."""

    def __init__(self, repo_root: Path) -> None:
        self._repo_root = repo_root

    def build_plan(self, output_dir: Path, bag_path: Path) -> DiagnosisPlan:
        command = [
            'python3',
            str(self._repo_root / 'scripts' / 'diagnose_autoware_map_run.py'),
            str(output_dir),
            '--bag',
            str(bag_path),
            '--write',
        ]
        return DiagnosisPlan(
            command=command,
            markdown_path=output_dir / 'autoware_map_diagnosis.md',
            json_path=output_dir / 'autoware_map_diagnosis.json',
        )


class Mid360RunManifestWriter:
    """Write reproducible run-plan manifests for MID-360 robot map runs."""

    JSON_NAME = 'mid360_robot_run_plan.json'
    MARKDOWN_NAME = 'mid360_robot_run_plan.md'

    def build_manifest(self, payload: dict[str, Any]) -> dict[str, Any]:
        preflight = payload['preflight']
        plan = payload['plan']
        summary = preflight['summary']
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'bag_path': summary['bag_path'],
            'output_dir': plan['output_dir'],
            'selected_topics': preflight['selected_topics'],
            'frames': preflight['frames'],
            'robot_profile': preflight.get('robot_profile', {}),
            'bag_diagnostics': preflight.get('bag_diagnostics', {}),
            'preflight_checks': preflight['checks'],
            'ready_for_mid360_launch': preflight['ready_for_mid360_launch'],
            'dogfood_command': plan['dogfood_command'],
            'dogfood_command_shell': plan['dogfood_command_shell'],
            'foxglove_command': plan['foxglove_command'],
            'foxglove_command_shell': plan['foxglove_command_shell'],
            'diagnosis': payload.get('diagnosis', {}),
        }

    def write(self, payload: dict[str, Any]) -> dict[str, Path]:
        manifest = self.build_manifest(payload)
        output_dir = Path(manifest['output_dir'])
        output_dir.mkdir(parents=True, exist_ok=True)
        json_path = output_dir / self.JSON_NAME
        markdown_path = output_dir / self.MARKDOWN_NAME
        json_path.write_text(payload_to_json(manifest) + '\n', encoding='utf-8')
        markdown_path.write_text(self.render_markdown(manifest) + '\n', encoding='utf-8')
        return {'json': json_path, 'markdown': markdown_path}

    @staticmethod
    def render_markdown(manifest: dict[str, Any]) -> str:
        lines = [
            '# MID-360 Robot Run Plan',
            '',
            f"- created_at: `{manifest['created_at']}`",
            f"- bag_path: `{manifest['bag_path']}`",
            f"- output_dir: `{manifest['output_dir']}`",
            f"- ready_for_mid360_launch: `{manifest['ready_for_mid360_launch']}`",
            '',
            '## Selected Topics',
            '',
            f"- pointcloud: `{manifest['selected_topics'].get('pointcloud')}`",
            f"- imu: `{manifest['selected_topics'].get('imu')}`",
            '',
            '## Frames',
            '',
            f"- base_frame: `{manifest['frames'].get('base_frame')}`",
            f"- lidar_frame: `{manifest['frames'].get('lidar_frame')}`",
            f"- imu_frame: `{manifest['frames'].get('imu_frame')}`",
            '',
            '## Robot Profile',
            '',
        ]
        profile = manifest.get('robot_profile') or {}
        if profile:
            lines.extend([
                f"- robot_name: `{profile.get('robot_name')}`",
                f"- source_path: `{profile.get('source_path')}`",
                f"- expected_pointcloud_topic: `{profile.get('expected_pointcloud_topic')}`",
                f"- expected_imu_topic: `{profile.get('expected_imu_topic')}`",
            ])
        else:
            lines.append('- none')

        lines.extend([
            '',
            '## Bag Diagnostics',
            '',
        ])
        Mid360RunManifestWriter._append_bag_diagnostics(lines, manifest.get('bag_diagnostics') or {})

        lines.extend([
            '',
            '## Preflight Checks',
            '',
        ])
        for check in manifest['preflight_checks']:
            lines.append(f"- `{check['status']}` `{check['id']}`: {check['message']}")

        lines.extend([
            '',
            '## Commands',
            '',
            '```bash',
            manifest['dogfood_command_shell'],
            '```',
        ])
        if manifest['foxglove_command_shell']:
            lines.extend([
                '',
                '```bash',
                manifest['foxglove_command_shell'],
                '```',
            ])
        diagnosis = manifest.get('diagnosis') or {}
        if diagnosis:
            lines.extend([
                '',
                '## Diagnosis',
                '',
                f"- ran: `{diagnosis.get('ran')}`",
                f"- markdown_path: `{diagnosis.get('markdown_path')}`",
                f"- json_path: `{diagnosis.get('json_path')}`",
                '',
                '```bash',
                diagnosis.get('command_shell', ''),
                '```',
            ])
        return '\n'.join(lines)

    @staticmethod
    def _append_bag_diagnostics(lines: list[str], diagnostics: dict[str, Any]) -> None:
        if not diagnostics:
            lines.append('- missing')
            return
        topics = diagnostics.get('topics') or {}
        for key in ('pointcloud', 'imu'):
            topic = topics.get(key) or {}
            rate_hz = topic.get('metadata_rate_hz')
            frame_ids = topic.get('sampled_frame_ids') or []
            rate_text = f'{float(rate_hz):.2f}' if isinstance(rate_hz, (int, float)) else 'unknown'
            lines.append(f"- {key}_metadata_rate_hz: `{rate_text}`")
            lines.append(
                f"- {key}_sampled_frame_ids: "
                f"`{', '.join(frame_ids) if frame_ids else 'not sampled'}`"
            )
        sample_reader = diagnostics.get('sample_reader') or {}
        lines.append(f"- sample_reader_available: `{sample_reader.get('available')}`")


class Mid360ReadinessReporter:
    """Build and write pre-run readiness reports for MID-360 robot mapping."""

    JSON_NAME = 'mid360_robot_readiness.json'
    MARKDOWN_NAME = 'mid360_robot_readiness.md'

    def build_report(
        self,
        payload: dict[str, Any],
        output_dir: Path,
        plan_error: str = '',
    ) -> dict[str, Any]:
        preflight = payload['preflight']
        checks = list(preflight['checks'])
        if plan_error:
            checks.append({
                'id': 'run_plan',
                'status': 'fail',
                'message': plan_error,
            })
        status = self._status_from_checks(checks)
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'bag_path': preflight['summary']['bag_path'],
            'output_dir': str(output_dir),
            'selected_topics': preflight['selected_topics'],
            'frames': preflight['frames'],
            'robot_profile': preflight.get('robot_profile', {}),
            'bag_diagnostics': preflight.get('bag_diagnostics', {}),
            'checks': checks,
            'counts': self._count_checks(checks),
            'ready_for_mid360_launch': preflight['ready_for_mid360_launch'],
            'plan': payload.get('plan', {}),
            'plan_error': plan_error,
        }

    def build_error_report(
        self,
        bag_path: Path,
        output_dir: Path,
        message: str,
    ) -> dict[str, Any]:
        check = {
            'id': 'readiness_setup',
            'status': 'fail',
            'message': message,
        }
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': 'FAIL',
            'bag_path': str(bag_path),
            'output_dir': str(output_dir),
            'selected_topics': {'pointcloud': None, 'imu': None},
            'frames': asdict(RobotFrames()),
            'robot_profile': {},
            'bag_diagnostics': {},
            'checks': [check],
            'counts': self._count_checks([check]),
            'ready_for_mid360_launch': False,
            'plan': {},
            'plan_error': message,
        }

    def write(self, report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        json_path = output_dir / self.JSON_NAME
        markdown_path = output_dir / self.MARKDOWN_NAME
        json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
        markdown_path.write_text(self.render_markdown(report) + '\n', encoding='utf-8')
        return {'json': json_path, 'markdown': markdown_path}

    @staticmethod
    def _status_from_checks(checks: list[dict[str, Any]]) -> str:
        if any(check['status'] == 'fail' for check in checks):
            return 'FAIL'
        if any(check['status'] == 'warn' for check in checks):
            return 'WARN'
        return 'PASS'

    @staticmethod
    def _count_checks(checks: list[dict[str, Any]]) -> dict[str, int]:
        return {
            'ok': sum(1 for check in checks if check['status'] == 'ok'),
            'warn': sum(1 for check in checks if check['status'] == 'warn'),
            'fail': sum(1 for check in checks if check['status'] == 'fail'),
        }

    @staticmethod
    def render_markdown(report: dict[str, Any]) -> str:
        lines = [
            '# MID-360 Robot Readiness',
            '',
            f"- status: `{report['status']}`",
            f"- created_at: `{report['created_at']}`",
            f"- bag_path: `{report['bag_path']}`",
            f"- output_dir: `{report['output_dir']}`",
            f"- ready_for_mid360_launch: `{report['ready_for_mid360_launch']}`",
            '',
            '## Counts',
            '',
            f"- ok: `{report['counts']['ok']}`",
            f"- warn: `{report['counts']['warn']}`",
            f"- fail: `{report['counts']['fail']}`",
            '',
            '## Checks',
            '',
        ]
        for check in report['checks']:
            lines.append(f"- `{check['status']}` `{check['id']}`: {check['message']}")

        lines.extend([
            '',
            '## Selected Topics',
            '',
            f"- pointcloud: `{report['selected_topics'].get('pointcloud')}`",
            f"- imu: `{report['selected_topics'].get('imu')}`",
            '',
            '## Frames',
            '',
            f"- base_frame: `{report['frames'].get('base_frame')}`",
            f"- lidar_frame: `{report['frames'].get('lidar_frame')}`",
            f"- imu_frame: `{report['frames'].get('imu_frame')}`",
            '',
            '## Bag Diagnostics',
            '',
        ])
        Mid360ReadinessReporter._append_bag_diagnostics(lines, report.get('bag_diagnostics') or {})

        lines.extend([
            '',
            '## Run Plan',
            '',
        ])
        plan = report.get('plan') or {}
        if plan.get('dogfood_command_shell'):
            lines.extend([
                '```bash',
                plan['dogfood_command_shell'],
                '```',
            ])
        elif report.get('plan_error'):
            lines.append(f"- plan_error: `{report['plan_error']}`")
        else:
            lines.append('- missing')
        return '\n'.join(lines)

    @staticmethod
    def _append_bag_diagnostics(lines: list[str], diagnostics: dict[str, Any]) -> None:
        if not diagnostics:
            lines.append('- missing')
            return
        topics = diagnostics.get('topics') or {}
        for key in ('pointcloud', 'imu'):
            topic = topics.get(key) or {}
            rate_hz = topic.get('metadata_rate_hz')
            frame_ids = topic.get('sampled_frame_ids') or []
            rate_text = f'{float(rate_hz):.2f}' if isinstance(rate_hz, (int, float)) else 'unknown'
            frames_text = ', '.join(frame_ids) if frame_ids else 'not sampled'
            lines.append(f"- {key}_metadata_rate_hz: `{rate_text}`")
            lines.append(f"- {key}_sampled_frame_ids: `{frames_text}`")
        sample_reader = diagnostics.get('sample_reader') or {}
        lines.append(f"- sample_reader_available: `{sample_reader.get('available')}`")


class AutowarePreflightAdapter:
    """Local adapter for the existing Autoware map preflight script."""

    def __init__(self, repo_root: Path) -> None:
        self._repo_root = repo_root
        self._module: Any | None = None

    def build_payload(self, bag_path: Path) -> dict[str, Any]:
        return self._load_module().build_preflight_payload(bag_path)

    def _load_module(self) -> Any:
        if self._module is not None:
            return self._module

        script_path = self._repo_root / 'scripts' / 'preflight_autoware_map_bag.py'
        spec = importlib.util.spec_from_file_location('preflight_autoware_map_bag', script_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f'failed to load {script_path}')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self._module = module
        return module


class RobotProfileLoader:
    """Load robot mapping defaults from YAML."""

    def load(self, path: Path) -> RobotProfile:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        if not isinstance(data, dict):
            raise ValueError(f'robot profile must be a YAML mapping: {path}')

        frame_data = data.get('frames') or {}
        if not isinstance(frame_data, dict):
            raise ValueError(f'robot profile frames must be a mapping: {path}')

        frames = self._load_frames(data, frame_data)
        robot_name = self._load_string(data, 'robot_name', path.stem, path)
        mount = self._load_mount(data, path)
        return RobotProfile(
            robot_name=robot_name,
            frames=frames,
            expected_pointcloud_topic=self._load_string(
                data,
                'expected_pointcloud_topic',
                str(data.get('pointcloud_topic') or ''),
                path,
            ),
            expected_imu_topic=self._load_string(
                data,
                'expected_imu_topic',
                str(data.get('imu_topic') or ''),
                path,
            ),
            mount=mount,
            source_path=str(path),
        )

    def _load_frames(self, data: dict[str, Any], frame_data: dict[str, Any]) -> RobotFrames:
        return RobotFrames(
            base_frame=self._frame_value(data, frame_data, 'base_frame', 'base_link'),
            lidar_frame=self._frame_value(data, frame_data, 'lidar_frame', 'livox_frame'),
            imu_frame=self._frame_value(data, frame_data, 'imu_frame', 'livox_frame'),
        )

    @staticmethod
    def _frame_value(
        data: dict[str, Any],
        frame_data: dict[str, Any],
        key: str,
        default: str,
    ) -> str:
        value = data.get(key) if key in data else frame_data.get(key, default)
        if not isinstance(value, str) or not value:
            raise ValueError(f'robot profile {key} must be a non-empty string')
        return value

    @staticmethod
    def _load_string(
        data: dict[str, Any],
        key: str,
        default: str,
        path: Path,
    ) -> str:
        value = data.get(key, default)
        if value is None:
            return ''
        if not isinstance(value, str):
            raise ValueError(f'robot profile {key} must be a string: {path}')
        return value

    @staticmethod
    def _load_mount(data: dict[str, Any], path: Path) -> dict[str, Any]:
        mount = data.get('mount') or {}
        if not isinstance(mount, dict):
            raise ValueError(f'robot profile mount must be a mapping: {path}')

        normalized = dict(mount)
        if 'xyz' in normalized:
            normalized['xyz'] = RobotProfileLoader._numeric_vector(
                normalized['xyz'], length=3, field='mount.xyz', path=path
            )
        if 'q_xyzw' in normalized:
            normalized['q_xyzw'] = RobotProfileLoader._numeric_vector(
                normalized['q_xyzw'], length=4, field='mount.q_xyzw', path=path
            )
        return normalized

    @staticmethod
    def _numeric_vector(value: Any, length: int, field: str, path: Path) -> list[float]:
        if not isinstance(value, list) or len(value) != length:
            raise ValueError(f'robot profile {field} must be a length-{length} list: {path}')
        result = []
        for item in value:
            if not isinstance(item, (int, float)):
                raise ValueError(f'robot profile {field} entries must be numeric: {path}')
            result.append(float(item))
        return result


def render_robot_profile_report(profile: RobotProfile) -> str:
    """Render a short profile validation report."""
    lines = [
        'MID-360 Robot Profile',
        f'profile: {profile.source_path}',
        f'robot_name: {profile.robot_name}',
        f'base_frame: {profile.frames.base_frame}',
        f'lidar_frame: {profile.frames.lidar_frame}',
        f'imu_frame: {profile.frames.imu_frame}',
        f'expected_pointcloud_topic: {profile.expected_pointcloud_topic or "not set"}',
        f'expected_imu_topic: {profile.expected_imu_topic or "not set"}',
    ]
    mount = profile.mount or {}
    if mount:
        lines.extend([
            f"mount.xyz: {mount.get('xyz', 'not set')}",
            f"mount.q_xyzw: {mount.get('q_xyzw', 'not set')}",
        ])
    return '\n'.join(lines)


def resolve_robot_frames(
    base_frame: str = '',
    lidar_frame: str = '',
    imu_frame: str = '',
    profile: RobotProfile | None = None,
) -> RobotFrames:
    """Resolve CLI frame overrides against an optional robot profile."""
    profile_frames = profile.frames if profile else RobotFrames()
    return RobotFrames(
        base_frame=base_frame or profile_frames.base_frame,
        lidar_frame=lidar_frame or profile_frames.lidar_frame,
        imu_frame=imu_frame or profile_frames.imu_frame,
    )


class RosbagMessageSampler:
    """Sample standard ROS messages from a rosbag2 directory."""

    def read_samples(
        self,
        bag_path: Path,
        topics: list[str],
        limit_per_topic: int,
    ) -> dict[str, list[MessageSample]]:
        if limit_per_topic <= 0:
            return {topic: [] for topic in topics}

        try:
            from rosbags.highlevel import AnyReader
            from rosbags.typesys import Stores, get_typestore
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError('rosbags is required for message sampling') from exc

        remaining = {topic: limit_per_topic for topic in topics if topic}
        samples: dict[str, list[MessageSample]] = {topic: [] for topic in remaining}
        if not remaining:
            return samples

        typestore = get_typestore(Stores.LATEST)
        with AnyReader([bag_path], default_typestore=typestore) as reader:
            connections = [conn for conn in reader.connections if conn.topic in remaining]
            for connection, timestamp_ns, raw in reader.messages(connections=connections):
                topic = connection.topic
                if remaining[topic] <= 0:
                    continue
                msg = reader.deserialize(raw, connection.msgtype)
                samples[topic].append(
                    self._sample_message(topic, connection.msgtype, timestamp_ns, msg)
                )
                remaining[topic] -= 1
                if all(count <= 0 for count in remaining.values()):
                    break
        return samples

    @staticmethod
    def _sample_message(
        topic: str,
        msg_type: str,
        timestamp_ns: int | None,
        msg: Any,
    ) -> MessageSample:
        header = getattr(msg, 'header', None)
        frame_id = str(getattr(header, 'frame_id', '') or '') if header else ''
        tf_pairs = RosbagMessageSampler._tf_pairs(msg)
        return MessageSample(
            topic=topic,
            msg_type=msg_type,
            timestamp_ns=int(timestamp_ns) if timestamp_ns is not None else None,
            header_stamp_ns=RosbagMessageSampler._header_stamp_ns(header),
            frame_id=frame_id,
            tf_pairs=tuple(tf_pairs),
        )

    @staticmethod
    def _header_stamp_ns(header: Any) -> int | None:
        if header is None:
            return None
        stamp = getattr(header, 'stamp', None)
        if stamp is None:
            return None
        sec = getattr(stamp, 'sec', None)
        nanosec = getattr(stamp, 'nanosec', None)
        if sec is None or nanosec is None:
            return None
        return int(sec) * 1_000_000_000 + int(nanosec)

    @staticmethod
    def _tf_pairs(msg: Any) -> list[tuple[str, str]]:
        pairs = []
        for transform in getattr(msg, 'transforms', []) or []:
            header = getattr(transform, 'header', None)
            parent = str(getattr(header, 'frame_id', '') or '') if header else ''
            child = str(getattr(transform, 'child_frame_id', '') or '')
            if parent and child:
                pairs.append((parent, child))
        return pairs


class Mid360BagDiagnosticsBuilder:
    """Build metadata-rate and message-sample diagnostics for a robot bag."""

    def __init__(
        self,
        sample_reader: Any | None = None,
        sample_limit: int = DEFAULT_SAMPLE_MESSAGES,
    ) -> None:
        self._sample_reader = sample_reader or RosbagMessageSampler()
        self._sample_limit = sample_limit

    def build(
        self,
        bag_path: Path,
        summary: dict[str, Any],
        topics: TopicSelection,
        frames: RobotFrames,
    ) -> dict[str, Any]:
        sample_topics = self._sample_topics(summary, topics)
        samples_by_topic, sample_reader_payload = self._read_samples(bag_path, sample_topics)
        pointcloud = self._topic_diagnostics(
            summary=summary,
            key='pointcloud2',
            topic=topics.pointcloud,
            expected_frame=frames.lidar_frame,
            min_rate_hz=POINTCLOUD_MIN_METADATA_HZ,
            samples=samples_by_topic.get(topics.pointcloud or '', []),
        )
        imu = self._topic_diagnostics(
            summary=summary,
            key='imu',
            topic=topics.imu,
            expected_frame=frames.imu_frame,
            min_rate_hz=IMU_MIN_METADATA_HZ,
            samples=samples_by_topic.get(topics.imu or '', []),
        )
        return {
            'sample_reader': sample_reader_payload,
            'topics': {
                'pointcloud': pointcloud,
                'imu': imu,
            },
            'tf': self._tf_diagnostics(summary, samples_by_topic, frames),
        }

    @staticmethod
    def _sample_topics(summary: dict[str, Any], topics: TopicSelection) -> list[str]:
        result = []
        if topics.pointcloud:
            result.append(topics.pointcloud)
        if topics.imu:
            result.append(topics.imu)
        result.extend(item['name'] for item in summary['topics'].get('tf', []))
        return list(dict.fromkeys(result))

    def _read_samples(
        self,
        bag_path: Path,
        topics: list[str],
    ) -> tuple[dict[str, list[MessageSample]], dict[str, Any]]:
        payload = {
            'attempted': False,
            'available': False,
            'limit_per_topic': self._sample_limit,
            'reason': '',
        }
        if not topics or self._sample_limit <= 0:
            payload['reason'] = 'no topics selected for sampling'
            return {}, payload

        payload['attempted'] = True
        try:
            samples = self._sample_reader.read_samples(bag_path, topics, self._sample_limit)
        except Exception as exc:  # Sampling is advisory; metadata checks still run.
            payload['reason'] = str(exc)
            return {}, payload

        sampled_count = sum(len(items) for items in samples.values())
        payload['available'] = sampled_count > 0
        if sampled_count == 0:
            payload['reason'] = 'no readable messages sampled'
        return samples, payload

    @staticmethod
    def _topic_diagnostics(
        summary: dict[str, Any],
        key: str,
        topic: str | None,
        expected_frame: str,
        min_rate_hz: float,
        samples: list[MessageSample],
    ) -> dict[str, Any]:
        record = Mid360BagDiagnosticsBuilder._find_topic(summary, key, topic)
        metadata_message_count = int(record.get('message_count', 0)) if record else 0
        metadata_rate_hz = Mid360BagDiagnosticsBuilder._metadata_rate_hz(
            metadata_message_count,
            summary.get('duration_sec'),
        )
        frame_ids = [sample.frame_id for sample in samples if sample.frame_id]
        unique_frame_ids = sorted(set(frame_ids))
        sample_span_sec = Mid360BagDiagnosticsBuilder._sample_span_sec(samples)
        return {
            'topic': topic,
            'metadata_message_count': metadata_message_count,
            'metadata_rate_hz': metadata_rate_hz,
            'min_metadata_rate_hz': min_rate_hz,
            'sampled_message_count': len(samples),
            'sample_time_span_sec': sample_span_sec,
            'sample_observed_rate_hz': Mid360BagDiagnosticsBuilder._sample_rate_hz(
                len(samples),
                sample_span_sec,
            ),
            'sampled_frame_ids': unique_frame_ids,
            'frame_id_changes': Mid360BagDiagnosticsBuilder._frame_id_changes(frame_ids),
            'stable_frame_id': None if not frame_ids else len(unique_frame_ids) == 1,
            'expected_frame_id': expected_frame,
            'matches_expected_frame': (
                None if not frame_ids else expected_frame in unique_frame_ids
            ),
        }

    @staticmethod
    def _find_topic(
        summary: dict[str, Any],
        key: str,
        topic: str | None,
    ) -> dict[str, Any] | None:
        if not topic:
            return None
        for item in summary['topics'].get(key, []):
            if item['name'] == topic:
                return item
        return None

    @staticmethod
    def _metadata_rate_hz(message_count: int, duration_sec: Any) -> float | None:
        if duration_sec is None:
            return None
        duration = float(duration_sec)
        if duration <= 0.0:
            return None
        return message_count / duration

    @staticmethod
    def _sample_span_sec(samples: list[MessageSample]) -> float | None:
        timestamps = [sample.timestamp_ns for sample in samples if sample.timestamp_ns is not None]
        if len(timestamps) < 2:
            return None
        span_ns = int(timestamps[-1]) - int(timestamps[0])
        if span_ns <= 0:
            return None
        return span_ns / 1e9

    @staticmethod
    def _sample_rate_hz(message_count: int, span_sec: float | None) -> float | None:
        if message_count < 2 or span_sec is None or span_sec <= 0.0:
            return None
        return (message_count - 1) / span_sec

    @staticmethod
    def _frame_id_changes(frame_ids: list[str]) -> int:
        if len(frame_ids) < 2:
            return 0
        return sum(1 for before, after in zip(frame_ids, frame_ids[1:]) if before != after)

    @staticmethod
    def _tf_diagnostics(
        summary: dict[str, Any],
        samples_by_topic: dict[str, list[MessageSample]],
        frames: RobotFrames,
    ) -> dict[str, Any]:
        tf_topics = [item['name'] for item in summary['topics'].get('tf', [])]
        samples = []
        for topic in tf_topics:
            samples.extend(samples_by_topic.get(topic, []))

        pairs = []
        seen = set()
        for sample in samples:
            for parent, child in sample.tf_pairs:
                if (parent, child) not in seen:
                    seen.add((parent, child))
                    pairs.append({'parent': parent, 'child': child})

        graph_pairs = [(item['parent'], item['child']) for item in pairs]
        return {
            'topics': tf_topics,
            'sampled_message_count': len(samples),
            'frame_pairs': pairs,
            'base_to_lidar_connected': Mid360BagDiagnosticsBuilder._frames_connected(
                graph_pairs,
                frames.base_frame,
                frames.lidar_frame,
            ),
            'base_to_imu_connected': Mid360BagDiagnosticsBuilder._frames_connected(
                graph_pairs,
                frames.base_frame,
                frames.imu_frame,
            ),
        }

    @staticmethod
    def _frames_connected(
        pairs: list[tuple[str, str]],
        source_frame: str,
        target_frame: str,
    ) -> bool | None:
        if source_frame == target_frame:
            return True
        if not pairs:
            return None
        adjacency: dict[str, list[str]] = {}
        for parent, child in pairs:
            adjacency.setdefault(parent, []).append(child)
            adjacency.setdefault(child, []).append(parent)

        queue = [source_frame]
        visited = {source_frame}
        while queue:
            current = queue.pop(0)
            for next_frame in adjacency.get(current, []):
                if next_frame == target_frame:
                    return True
                if next_frame in visited:
                    continue
                visited.add(next_frame)
                queue.append(next_frame)
        return False


class Mid360RobotPreflight:
    """Build and render MID-360 robot preflight reports."""

    def __init__(
        self,
        adapter: AutowarePreflightAdapter,
        diagnostics_builder: Mid360BagDiagnosticsBuilder | None = None,
    ) -> None:
        self._adapter = adapter
        self._diagnostics_builder = diagnostics_builder or Mid360BagDiagnosticsBuilder()

    def build_payload(
        self,
        bag_path: Path,
        frames: RobotFrames,
        profile: RobotProfile | None = None,
    ) -> dict[str, Any]:
        """Build a robot-focused preflight payload from rosbag2 metadata."""
        autoware_payload = self._adapter.build_payload(bag_path)
        summary = autoware_payload['summary']
        topics = self._select_topics(summary, profile)
        has_mid360 = self._has_mid360_recommendation(autoware_payload)
        bag_diagnostics = self._diagnostics_builder.build(bag_path, summary, topics, frames)
        checks = self._build_checks(
            summary,
            topics,
            has_mid360,
            profile,
            bag_diagnostics,
        )

        return {
            'summary': summary,
            'checks': [asdict(check) for check in checks],
            'ready_for_mid360_launch': (
                topics.ready
                and self._profile_topics_ok(summary, profile)
                and not any(check.status == 'fail' for check in checks)
            ),
            'launch_command': self._build_launch_command(summary, topics, frames),
            'frames': asdict(frames),
            'selected_topics': asdict(topics),
            'bag_diagnostics': bag_diagnostics,
            'robot_profile': profile.to_dict() if profile else {},
            'next_actions': [
                'Confirm base_link -> livox_frame static transform from measurement.',
                'Record one stationary bag and one short walking bag before a full route.',
                'Run /map_save after the offline launch completes and verify pointcloud_map.',
            ],
            'autoware_preflight': autoware_payload,
        }

    def render_text_report(self, payload: dict[str, Any]) -> str:
        """Render a human-readable robot preflight report."""
        summary = payload['summary']
        duration = summary['duration_sec']
        duration_text = f'{duration:.3f}s' if duration is not None else 'unknown'

        lines = [
            'MID-360 Robot Bag Preflight',
            f"bag: {summary['bag_path']}",
            f"duration: {duration_text}",
            f"messages: {summary['message_count']}",
            '',
            'Checks:',
        ]
        for check in payload['checks']:
            lines.append(f"  [{check['status'].upper()}] {check['message']}")

        diagnostics = payload.get('bag_diagnostics') or {}
        if diagnostics:
            pointcloud = diagnostics['topics']['pointcloud']
            imu = diagnostics['topics']['imu']
            lines.extend([
                '',
                'Bag diagnostics:',
                f"  pointcloud metadata rate: {self._format_hz(pointcloud['metadata_rate_hz'])}",
                f"  imu metadata rate: {self._format_hz(imu['metadata_rate_hz'])}",
                f"  pointcloud sampled frames: {self._format_frames(pointcloud)}",
                f"  imu sampled frames: {self._format_frames(imu)}",
                (
                    '  message sampling: available'
                    if diagnostics['sample_reader']['available']
                    else f"  message sampling: unavailable ({diagnostics['sample_reader']['reason']})"
                ),
            ])

        if payload['launch_command']:
            lines.extend([
                '',
                'Recommended MID-360 launch:',
                textwrap.indent(payload['launch_command'], '  '),
            ])
        else:
            lines.extend([
                '',
                'Recommended MID-360 launch: unavailable until PointCloud2 and Imu topics exist.',
            ])

        lines.append('')
        lines.append('Next actions:')
        for action in payload['next_actions']:
            lines.append(f'  - {action}')

        return '\n'.join(lines)

    @staticmethod
    def _format_hz(value: float | None) -> str:
        return f'{value:.2f} Hz' if value is not None else 'unknown'

    @staticmethod
    def _format_frames(topic_diagnostics: dict[str, Any]) -> str:
        frames = topic_diagnostics.get('sampled_frame_ids') or []
        return ', '.join(frames) if frames else 'not sampled'

    def _select_topics(
        self,
        summary: dict[str, Any],
        profile: RobotProfile | None,
    ) -> TopicSelection:
        pointcloud = self._select_topic_name(
            summary,
            'pointcloud2',
            profile.expected_pointcloud_topic if profile else '',
        )
        imu = self._select_topic_name(
            summary,
            'imu',
            profile.expected_imu_topic if profile else '',
        )
        return TopicSelection(pointcloud=pointcloud, imu=imu)

    @staticmethod
    def _select_topic_name(summary: dict[str, Any], key: str, expected: str) -> str | None:
        topics = summary['topics'][key]
        if expected:
            return expected if any(item['name'] == expected for item in topics) else None
        return topics[0]['name'] if topics else None

    @staticmethod
    def _has_mid360_recommendation(payload: dict[str, Any]) -> bool:
        return any(item['id'] == MID360_PROFILE_ID for item in payload['recommendations'])

    @staticmethod
    def _build_checks(
        summary: dict[str, Any],
        topics: TopicSelection,
        has_mid360: bool,
        profile: RobotProfile | None,
        bag_diagnostics: dict[str, Any],
    ) -> list[PreflightCheck]:
        checks = [
            PreflightCheck(
                id='pointcloud2',
                status='ok' if topics.pointcloud else 'fail',
                message=(
                    f'PointCloud2 topic: {topics.pointcloud}'
                    if topics.pointcloud else 'No PointCloud2 topic found.'
                ),
            ),
            PreflightCheck(
                id='imu',
                status='ok' if topics.imu else 'fail',
                message=f'Imu topic: {topics.imu}' if topics.imu else 'No Imu topic found.',
            ),
            PreflightCheck(
                id='mid360_preset',
                status='ok' if has_mid360 else 'warn',
                message=(
                    'Livox/MID-360 preset recommendation is available.'
                    if has_mid360
                    else 'Bag path/topics do not look Livox/MID-360-specific.'
                ),
            ),
            PreflightCheck(
                id='tf_metadata',
                status='ok' if summary['capabilities']['has_tf'] else 'warn',
                message=(
                    'TF or TF_STATIC topic exists in the bag metadata.'
                    if summary['capabilities']['has_tf']
                    else 'No TF/TF_STATIC topic found in metadata; pass static frames explicitly.'
                ),
            ),
        ]
        if profile and profile.expected_pointcloud_topic:
            checks.append(
                Mid360RobotPreflight._expected_topic_check(
                    summary,
                    key='pointcloud2',
                    check_id='expected_pointcloud_topic',
                    expected=profile.expected_pointcloud_topic,
                )
            )
        if profile and profile.expected_imu_topic:
            checks.append(
                Mid360RobotPreflight._expected_topic_check(
                    summary,
                    key='imu',
                    check_id='expected_imu_topic',
                    expected=profile.expected_imu_topic,
                )
            )
        checks.extend(Mid360RobotPreflight._bag_diagnostic_checks(bag_diagnostics))
        return checks

    @staticmethod
    def _bag_diagnostic_checks(bag_diagnostics: dict[str, Any]) -> list[PreflightCheck]:
        topics = bag_diagnostics.get('topics') or {}
        checks = []
        for key, label in (
            ('pointcloud', 'PointCloud2'),
            ('imu', 'Imu'),
        ):
            topic_diagnostics = topics.get(key) or {}
            rate_check = Mid360RobotPreflight._metadata_rate_check(
                topic_diagnostics,
                check_id=f'{key}_metadata_rate',
                label=label,
            )
            if rate_check:
                checks.append(rate_check)
            frame_check = Mid360RobotPreflight._sample_frame_check(
                topic_diagnostics,
                check_id=f'{key}_frame_id',
                label=label,
            )
            if frame_check:
                checks.append(frame_check)
        checks.extend(Mid360RobotPreflight._tf_sample_checks(bag_diagnostics.get('tf') or {}))
        return checks

    @staticmethod
    def _metadata_rate_check(
        topic_diagnostics: dict[str, Any],
        check_id: str,
        label: str,
    ) -> PreflightCheck | None:
        topic = topic_diagnostics.get('topic')
        if not topic:
            return None
        rate_hz = topic_diagnostics.get('metadata_rate_hz')
        min_rate_hz = float(topic_diagnostics.get('min_metadata_rate_hz') or 0.0)
        if rate_hz is None:
            return PreflightCheck(
                id=check_id,
                status='warn',
                message=f'{label} metadata rate is unknown because bag duration is missing.',
            )
        if float(rate_hz) < min_rate_hz:
            return PreflightCheck(
                id=check_id,
                status='warn',
                message=(
                    f'{label} metadata rate is {float(rate_hz):.2f} Hz on {topic}; '
                    f'expected at least {min_rate_hz:.2f} Hz for a field check.'
                ),
            )
        return PreflightCheck(
            id=check_id,
            status='ok',
            message=f'{label} metadata rate is {float(rate_hz):.2f} Hz on {topic}.',
        )

    @staticmethod
    def _sample_frame_check(
        topic_diagnostics: dict[str, Any],
        check_id: str,
        label: str,
    ) -> PreflightCheck | None:
        frame_ids = topic_diagnostics.get('sampled_frame_ids') or []
        if not frame_ids:
            return None
        expected = topic_diagnostics.get('expected_frame_id') or ''
        if expected and topic_diagnostics.get('matches_expected_frame') is False:
            return PreflightCheck(
                id=check_id,
                status='fail',
                message=(
                    f'{label} sampled frame_id does not match expected {expected}: '
                    f'{", ".join(frame_ids)}'
                ),
            )
        if topic_diagnostics.get('stable_frame_id') is False:
            return PreflightCheck(
                id=check_id,
                status='warn',
                message=(
                    f'{label} sampled frame_id changed '
                    f"{topic_diagnostics.get('frame_id_changes')} times: "
                    f'{", ".join(frame_ids)}'
                ),
            )
        return PreflightCheck(
            id=check_id,
            status='ok',
            message=f'{label} sampled frame_id is stable: {frame_ids[0]}',
        )

    @staticmethod
    def _tf_sample_checks(tf_diagnostics: dict[str, Any]) -> list[PreflightCheck]:
        if not tf_diagnostics.get('sampled_message_count'):
            return []
        checks = []
        for key, label in (
            ('base_to_lidar_connected', 'base frame to lidar frame'),
            ('base_to_imu_connected', 'base frame to imu frame'),
        ):
            connected = tf_diagnostics.get(key)
            if connected is None:
                continue
            checks.append(
                PreflightCheck(
                    id=f'tf_{key}',
                    status='ok' if connected else 'warn',
                    message=(
                        f'TF samples connect {label}.'
                        if connected
                        else f'TF samples do not connect {label}; verify static extrinsics.'
                    ),
                )
            )
        return checks

    @staticmethod
    def _expected_topic_check(
        summary: dict[str, Any],
        key: str,
        check_id: str,
        expected: str,
    ) -> PreflightCheck:
        available = [item['name'] for item in summary['topics'][key]]
        if expected in available:
            return PreflightCheck(
                id=check_id,
                status='ok',
                message=f'Profile expected topic is present: {expected}',
            )
        return PreflightCheck(
            id=check_id,
            status='fail',
            message=(
                f'Profile expected topic is missing: {expected}. '
                f'Available: {", ".join(available) if available else "none"}'
            ),
        )

    @staticmethod
    def _profile_topics_ok(summary: dict[str, Any], profile: RobotProfile | None) -> bool:
        if profile is None:
            return True
        if profile.expected_pointcloud_topic:
            available = [item['name'] for item in summary['topics']['pointcloud2']]
            if profile.expected_pointcloud_topic not in available:
                return False
        if profile.expected_imu_topic:
            available = [item['name'] for item in summary['topics']['imu']]
            if profile.expected_imu_topic not in available:
                return False
        return True

    @staticmethod
    def _build_launch_command(
        summary: dict[str, Any],
        topics: TopicSelection,
        frames: RobotFrames,
    ) -> str:
        if not topics.ready:
            return ''

        return textwrap.dedent(
            f"""\
            ros2 launch lidarslam rko_lio_slam.launch.py \\
              main_param_dir:=lidarslam/param/lidarslam_mid360_rko_graph.yaml \\
              rko_param_file:=lidarslam/param/rko_lio_mid360.yaml \\
              bag_path:={shlex.quote(summary['bag_path'])} \\
              lidar_topic:={shlex.quote(topics.pointcloud or '')} \\
              imu_topic:={shlex.quote(topics.imu or '')} \\
              base_frame:={shlex.quote(frames.base_frame)} \\
              lidar_frame:={shlex.quote(frames.lidar_frame)} \\
              imu_frame:={shlex.quote(frames.imu_frame)}"""
        )


class Mid360MapRunPlanner:
    """Create executable commands for the MID-360 robot map wrapper."""

    def __init__(self, repo_root: Path) -> None:
        self._repo_root = repo_root

    def build_plan(
        self,
        bag_path: Path,
        payload: dict[str, Any],
        frames: RobotFrames,
        options: MapRunOptions,
    ) -> MapRunPlan:
        if not payload['ready_for_mid360_launch']:
            raise ValueError('MID-360 robot mapping prerequisites are not satisfied.')

        output_dir = options.output_dir
        if output_dir is None:
            output_dir = self._default_output_dir(bag_path)

        topics = payload['selected_topics']
        dogfood_command = [
            'bash',
            str(self._repo_root / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh'),
            '--bag',
            str(bag_path),
            '--lidar-topic',
            topics['pointcloud'],
            '--imu-topic',
            topics['imu'],
            '--lidarslam-param',
            str(self._repo_root / 'lidarslam' / 'param' / 'lidarslam_mid360_rko_graph.yaml'),
            '--rko-param',
            str(self._repo_root / 'lidarslam' / 'param' / 'rko_lio_mid360.yaml'),
            '--base-frame',
            frames.base_frame,
            '--lidar-frame',
            frames.lidar_frame,
            '--imu-frame',
            frames.imu_frame,
            '--output-dir',
            str(output_dir),
            '--wait-for-offline-completion',
        ]

        self._append_optional_dogfood_args(dogfood_command, options)
        foxglove_command = self._build_foxglove_command(output_dir, options)
        return MapRunPlan(
            output_dir=output_dir,
            dogfood_command=dogfood_command,
            foxglove_command=foxglove_command,
        )

    def _default_output_dir(self, bag_path: Path) -> Path:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return self._repo_root / 'output' / f'mid360_robot_map_{bag_path.name}_{timestamp}'

    @staticmethod
    def _append_optional_dogfood_args(command: list[str], options: MapRunOptions) -> None:
        if options.run_name:
            command.extend(['--run-name', options.run_name])
        if options.save_timeout_secs:
            command.extend(['--save-timeout-secs', options.save_timeout_secs])
        if options.startup_timeout_secs:
            command.extend(['--startup-timeout-secs', options.startup_timeout_secs])
        if options.keep_launch:
            command.append('--keep-launch')
        if options.viewer in ('none', 'foxglove'):
            command.append('--skip-viewer')
        if options.viewer_rebuild:
            command.append('--viewer-rebuild')
        if options.viewer_run_dir:
            command.extend(['--viewer-run-dir', options.viewer_run_dir])
        if options.autoware_core_dir:
            command.extend(['--autoware-core-dir', options.autoware_core_dir])
        if options.work_dir:
            command.extend(['--work-dir', options.work_dir])
        if options.auto_exit_secs:
            command.extend(['--auto-exit-secs', options.auto_exit_secs])

    def _build_foxglove_command(self, output_dir: Path, options: MapRunOptions) -> list[str]:
        if options.viewer != 'foxglove':
            return []

        command = [
            'bash',
            str(self._repo_root / 'scripts' / 'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh'),
            str(output_dir),
        ]
        if options.work_dir:
            command.extend(['--work-dir', options.work_dir])
        if options.viewer_run_dir:
            command.extend(['--run-dir', options.viewer_run_dir])
        if options.viewer_rebuild:
            command.append('--rebuild')
        if options.auto_exit_secs:
            command.extend(['--auto-exit-secs', options.auto_exit_secs])
        return command


def payload_to_json(payload: dict[str, Any]) -> str:
    """Serialize payloads consistently for CLIs."""
    return json.dumps(payload, indent=2, sort_keys=True)
