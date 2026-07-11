"""MID-360 robot profile loading, validation, and frame resolution."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .mid360_models import RobotFrames, RobotProfile


class RobotProfileLoader:
    """Load robot mapping defaults from YAML."""

    def load(self, path: Path) -> RobotProfile:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        if not isinstance(data, dict):
            raise ValueError(f'robot profile must be a YAML mapping: {path}')
        frame_data = data.get('frames') or {}
        if not isinstance(frame_data, dict):
            raise ValueError(f'robot profile frames must be a mapping: {path}')
        return RobotProfile(
            robot_name=self._load_string(data, 'robot_name', path.stem, path),
            frames=self._load_frames(data, frame_data),
            expected_pointcloud_topic=self._load_string(
                data, 'expected_pointcloud_topic', str(data.get('pointcloud_topic') or ''), path),
            expected_imu_topic=self._load_string(
                data, 'expected_imu_topic', str(data.get('imu_topic') or ''), path),
            mount=self._load_mount(data, path),
            source_path=str(path),
        )

    def _load_frames(self, data: dict[str, Any], frame_data: dict[str, Any]) -> RobotFrames:
        return RobotFrames(
            base_frame=self._frame_value(data, frame_data, 'base_frame', 'base_link'),
            lidar_frame=self._frame_value(data, frame_data, 'lidar_frame', 'livox_frame'),
            imu_frame=self._frame_value(data, frame_data, 'imu_frame', 'livox_frame'),
        )

    @staticmethod
    def _frame_value(data: dict[str, Any], frame_data: dict[str, Any],
                     key: str, default: str) -> str:
        value = data.get(key) if key in data else frame_data.get(key, default)
        if not isinstance(value, str) or not value:
            raise ValueError(f'robot profile {key} must be a non-empty string')
        return value

    @staticmethod
    def _load_string(data: dict[str, Any], key: str, default: str, path: Path) -> str:
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
                normalized['xyz'], length=3, field='mount.xyz', path=path)
        if 'q_xyzw' in normalized:
            normalized['q_xyzw'] = RobotProfileLoader._numeric_vector(
                normalized['q_xyzw'], length=4, field='mount.q_xyzw', path=path)
        return normalized

    @staticmethod
    def _numeric_vector(value: Any, length: int, field: str, path: Path) -> list[float]:
        if not isinstance(value, list) or len(value) != length:
            raise ValueError(f'robot profile {field} must be a length-{length} list: {path}')
        if not all(isinstance(item, (int, float)) for item in value):
            raise ValueError(f'robot profile {field} entries must be numeric: {path}')
        return [float(item) for item in value]


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


def resolve_robot_frames(base_frame: str = '', lidar_frame: str = '', imu_frame: str = '',
                         profile: RobotProfile | None = None) -> RobotFrames:
    """Resolve CLI frame overrides against an optional robot profile."""
    profile_frames = profile.frames if profile else RobotFrames()
    return RobotFrames(
        base_frame=base_frame or profile_frames.base_frame,
        lidar_frame=lidar_frame or profile_frames.lidar_frame,
        imu_frame=imu_frame or profile_frames.imu_frame,
    )
