"""Command planning for MID-360 mapping and post-run diagnosis."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any

from .mid360_models import DiagnosisPlan, MapRunOptions, MapRunPlan, RobotFrames


class Mid360RunDiagnosisPlanner:
    def __init__(self, repo_root: Path) -> None:
        self._repo_root = repo_root

    def build_plan(self, output_dir: Path, bag_path: Path) -> DiagnosisPlan:
        command = [
            'python3',
            str(self._repo_root / 'scripts' / 'diagnose_autoware_map_run.py'),
            str(output_dir), '--bag', str(bag_path), '--write',
        ]
        return DiagnosisPlan(
            command=command,
            markdown_path=output_dir / 'autoware_map_diagnosis.md',
            json_path=output_dir / 'autoware_map_diagnosis.json',
        )


class Mid360MapRunPlanner:
    """Create executable commands for the MID-360 robot map wrapper."""

    def __init__(self, repo_root: Path) -> None:
        self._repo_root = repo_root

    def build_plan(self, bag_path: Path, payload: dict[str, Any], frames: RobotFrames,
                   options: MapRunOptions) -> MapRunPlan:
        if not payload['ready_for_mid360_launch']:
            raise ValueError('MID-360 robot mapping prerequisites are not satisfied.')
        output_dir = options.output_dir or self._default_output_dir(bag_path)
        topics = payload['selected_topics']
        dogfood_command = [
            'bash', str(self._repo_root / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh'),
            '--bag', str(bag_path),
            '--lidar-topic', topics['pointcloud'],
            '--imu-topic', topics['imu'],
            '--lidarslam-param',
            str(self._repo_root / 'lidarslam' / 'param' / 'lidarslam_mid360_rko_graph.yaml'),
            '--rko-param', str(self._repo_root / 'lidarslam' / 'param' / 'rko_lio_mid360.yaml'),
            '--base-frame', frames.base_frame,
            '--lidar-frame', frames.lidar_frame,
            '--imu-frame', frames.imu_frame,
            '--output-dir', str(output_dir),
            '--wait-for-offline-completion',
        ]
        self._append_optional_dogfood_args(dogfood_command, options)
        return MapRunPlan(
            output_dir=output_dir,
            dogfood_command=dogfood_command,
            foxglove_command=self._build_foxglove_command(output_dir, options),
        )

    def _default_output_dir(self, bag_path: Path) -> Path:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return self._repo_root / 'output' / f'mid360_robot_map_{bag_path.name}_{timestamp}'

    @staticmethod
    def _append_optional_dogfood_args(command: list[str], options: MapRunOptions) -> None:
        pairs = (
            ('--run-name', options.run_name),
            ('--save-timeout-secs', options.save_timeout_secs),
            ('--startup-timeout-secs', options.startup_timeout_secs),
        )
        for flag, value in pairs:
            if value:
                command.extend([flag, value])
        if options.keep_launch:
            command.append('--keep-launch')
        if options.viewer in ('none', 'foxglove'):
            command.append('--skip-viewer')
        optional_pairs = (
            ('--viewer-run-dir', options.viewer_run_dir),
            ('--autoware-core-dir', options.autoware_core_dir),
            ('--work-dir', options.work_dir),
            ('--auto-exit-secs', options.auto_exit_secs),
        )
        if options.viewer_rebuild:
            command.append('--viewer-rebuild')
        for flag, value in optional_pairs:
            if value:
                command.extend([flag, value])

    def _build_foxglove_command(self, output_dir: Path,
                                options: MapRunOptions) -> list[str]:
        if options.viewer != 'foxglove':
            return []
        command = [
            'bash',
            str(self._repo_root / 'scripts' /
                'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh'),
            str(output_dir),
        ]
        for flag, value in (
            ('--work-dir', options.work_dir),
            ('--run-dir', options.viewer_run_dir),
        ):
            if value:
                command.extend([flag, value])
        if options.viewer_rebuild:
            command.append('--rebuild')
        if options.auto_exit_secs:
            command.extend(['--auto-exit-secs', options.auto_exit_secs])
        return command
