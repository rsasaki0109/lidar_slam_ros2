"""Data contracts shared by MID-360 planning and preflight tools."""

from __future__ import annotations

import shlex
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class RobotFrames:
    base_frame: str = 'base_link'
    lidar_frame: str = 'livox_frame'
    imu_frame: str = 'livox_frame'


@dataclass(frozen=True)
class RobotProfile:
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
    pointcloud: str | None
    imu: str | None

    @property
    def ready(self) -> bool:
        return bool(self.pointcloud and self.imu)


@dataclass(frozen=True)
class MessageSample:
    topic: str
    msg_type: str
    timestamp_ns: int | None = None
    header_stamp_ns: int | None = None
    frame_id: str = ''
    tf_pairs: tuple[tuple[str, str], ...] = ()


@dataclass(frozen=True)
class PreflightCheck:
    id: str
    status: str
    message: str


@dataclass(frozen=True)
class MapRunOptions:
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
