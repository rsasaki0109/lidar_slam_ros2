"""MID-360 run manifest and readiness report builders."""

from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Any

from .mid360_models import RobotFrames
from .mid360_reporting import (
    append_bag_diagnostics, count_checks, status_from_checks, utc_timestamp, write_report,
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
            'created_at': utc_timestamp(),
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
        return write_report(
            output_dir, self.JSON_NAME, self.MARKDOWN_NAME,
            manifest, self.render_markdown(manifest))

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
        append_bag_diagnostics(lines, manifest.get('bag_diagnostics') or {})

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
        status = status_from_checks(checks)
        return {
            'created_at': utc_timestamp(),
            'status': status,
            'bag_path': preflight['summary']['bag_path'],
            'output_dir': str(output_dir),
            'selected_topics': preflight['selected_topics'],
            'frames': preflight['frames'],
            'robot_profile': preflight.get('robot_profile', {}),
            'bag_diagnostics': preflight.get('bag_diagnostics', {}),
            'checks': checks,
            'counts': count_checks(checks),
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
            'created_at': utc_timestamp(),
            'status': 'FAIL',
            'bag_path': str(bag_path),
            'output_dir': str(output_dir),
            'selected_topics': {'pointcloud': None, 'imu': None},
            'frames': asdict(RobotFrames()),
            'robot_profile': {},
            'bag_diagnostics': {},
            'checks': [check],
            'counts': count_checks([check]),
            'ready_for_mid360_launch': False,
            'plan': {},
            'plan_error': message,
        }

    def write(self, report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
        return write_report(
            output_dir, self.JSON_NAME, self.MARKDOWN_NAME,
            report, self.render_markdown(report))

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
        append_bag_diagnostics(lines, report.get('bag_diagnostics') or {})

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
