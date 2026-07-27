#!/usr/bin/env python3
"""Stable product command surface over the proven map-authoring tools."""

from __future__ import annotations

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
}
EX_USAGE = 2
EX_SOFTWARE = 70


def render_help() -> str:
    """Return top-level CLI help."""
    return '\n'.join([
        'Usage: ./scripts/lidarslam <command> [options]',
        '',
        'Offline rosbag2-to-map product commands:',
        '  doctor <rosbag2_dir>   Check inputs and select a compatible profile',
        '  run <rosbag2_dir>      Build and verify a map bundle',
        '  inspect <output_dir>   Diagnose an existing map-authoring output',
        '',
        'Global options:',
        '  --version              Print the repository product version',
        '  --help                 Show this help',
        '',
        'Run "./scripts/lidarslam <command> --help" for command options.',
        '',
        'Exit codes:',
        '  0   command completed successfully',
        '  2   invalid usage, input, profile, or output path',
        '  70  the command could not start because of an internal/tooling error',
        '  other non-zero values are propagated from the map workflow',
    ])


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
        print('hint: run "./scripts/lidarslam --help".', file=sys.stderr)
        return EX_USAGE

    try:
        completed = subprocess.run(command_argv(command, args), check=False)
    except (OSError, RuntimeError) as exc:
        print(f'error: failed to start {command}: {exc}', file=sys.stderr)
        return EX_SOFTWARE
    return completed.returncode


if __name__ == '__main__':
    raise SystemExit(main())
