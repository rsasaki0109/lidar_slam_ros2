#!/usr/bin/env python3
"""Stable product command surface over the proven map-authoring tools."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
from typing import Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
VERSION_PATH = REPO_ROOT / 'VERSION'
COMMANDS = {
    'doctor': 'preflight_autoware_map_bag.py',
    'run': 'run_autoware_map_from_bag.py',
    'inspect': 'diagnose_autoware_map_run.py',
    'view': 'view_autoware_map.py',
    'migrate-manifest': 'migrate_run_manifest.py',
    'rollback-plan': 'plan_image_rollback.py',
}
EX_USAGE = 2
EX_SOFTWARE = 70
HELP_MODE_ENV = 'LIDARSLAM_CLI_HELP_MODE'


def command_name() -> str:
    """Return the source or installed command spelling."""
    configured = os.environ.get('LIDARSLAM_CLI_NAME')
    if configured:
        return configured
    return './scripts/lidarslam'


def render_help(*, include_all: bool = False) -> str:
    """Return top-level CLI help."""
    executable = command_name()
    lines = [
        f'Usage: {executable} <command> [options]',
        '',
        'Core rosbag2-to-map commands:',
        '  doctor <rosbag2_dir>   Check inputs and select a compatible profile',
        '  run <rosbag2_dir>      Build and verify a map bundle',
        '  inspect <output_dir>   Diagnose an existing map-authoring output',
        '',
        'Optional post-processing:',
        '  view <output_dir>      Open a completed map in an optional viewer',
        '',
        'Global options:',
        '  --version              Print the repository product version',
        '  --help                 Show this help',
        '  --help-all             Show advanced and deprecated help',
        '',
        f'Run "{executable} <command> --help" for command options.',
        '',
        'Exit codes:',
        '  0   command completed successfully',
        '  2   invalid usage, input, profile, or output path',
        '  70  the command could not start because of an internal/tooling error',
        '  other non-zero values are propagated from a delegated workflow',
    ]
    if include_all:
        lines.extend([
            '',
            'Advanced recovery commands:',
            (
                '  migrate-manifest <output_dir>'
                '  Convert terminal v1 metadata for inspection'
            ),
            (
                '  rollback-plan <release_image_record>'
                '  Plan an immutable image rollback'
            ),
            '',
            'Help levels:',
            f'  {executable} <command> --help',
            '      Show the stable options needed for normal operation.',
            f'  {executable} <command> --help-all',
            '      Also show advanced runtime and deprecated compatibility options.',
        ])
    return '\n'.join(lines)


def read_version() -> str:
    """Read the root VERSION source of truth."""
    try:
        version = VERSION_PATH.read_text(encoding='utf-8').strip()
    except OSError as exc:
        raise RuntimeError(f'failed to read product version: {exc}') from exc
    if not version:
        raise RuntimeError(f'product version is empty: {VERSION_PATH}')
    return version


def command_argv(command: str, args: Sequence[str]) -> list[str]:
    """Build a delegated product command."""
    helper_name = COMMANDS[command]
    helper_path = SCRIPT_DIR / helper_name
    if not helper_path.is_file():
        raise RuntimeError(f'product helper is missing: {helper_path}')
    return [sys.executable, str(helper_path), *args]


def main(argv: Sequence[str] | None = None) -> int:
    """Dispatch the stable product command surface."""
    args = list(sys.argv[1:] if argv is None else argv)
    if args in (['--help'], ['-h']):
        print(render_help())
        return 0
    if args == ['--help-all']:
        print(render_help(include_all=True))
        return 0
    if args == ['--version']:
        try:
            print(f'lidarslam_ros2 {read_version()}')
        except RuntimeError as exc:
            print(f'error: {exc}', file=sys.stderr)
            return EX_SOFTWARE
        return 0
    if not args:
        print(render_help(), file=sys.stderr)
        return EX_USAGE

    command = args.pop(0)
    if command not in COMMANDS:
        print(f'error: unknown command: {command}', file=sys.stderr)
        print(f'hint: run "{command_name()} --help".', file=sys.stderr)
        return EX_USAGE

    try:
        child_env = os.environ.copy()
        child_env.pop(HELP_MODE_ENV, None)
        if '--help-all' in args:
            if args != ['--help-all']:
                print(
                    'error: --help-all cannot be combined with other options',
                    file=sys.stderr,
                )
                return EX_USAGE
            args = ['--help']
            child_env[HELP_MODE_ENV] = 'all'
        elif '--help' in args or '-h' in args:
            child_env[HELP_MODE_ENV] = 'core'
        child_env['LIDARSLAM_CLI_COMMAND'] = f'{command_name()} {command}'
        completed = subprocess.run(
            command_argv(command, args),
            check=False,
            env=child_env,
        )
    except (OSError, RuntimeError) as exc:
        print(f'error: failed to start {command}: {exc}', file=sys.stderr)
        return EX_SOFTWARE
    return completed.returncode


if __name__ == '__main__':
    raise SystemExit(main())
