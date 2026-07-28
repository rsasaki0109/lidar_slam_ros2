#!/usr/bin/env python3
"""Open a verified map-run output in an optional product viewer."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
EX_USAGE = 2
EX_SOFTWARE = 70


def _positive_seconds(value: str) -> int:
    try:
        seconds = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be a positive integer') from exc
    if seconds <= 0:
        raise argparse.ArgumentTypeError('must be a positive integer')
    return seconds


def _help_epilog(*, show_advanced: bool = True) -> str:
    command = os.environ.get(
        'LIDARSLAM_CLI_COMMAND',
        'python3 scripts/view_autoware_map.py',
    )
    lines = [
        'The input must be a completed map-run output containing:',
        '  pointcloud_map/pointcloud_map_metadata.yaml',
        '  map_projector_info.yaml',
        '',
        'Examples:',
        f'  {command} output/my_map',
        f'  {command} output/my_map --viewer foxglove',
    ]
    if show_advanced:
        lines.append(f'  {command} output/my_map --rebuild')
    return '\n'.join(lines)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the public viewer command options."""
    show_all_help = os.environ.get('LIDARSLAM_CLI_HELP_MODE') != 'core'

    def advanced_help(text: str) -> str:
        return text if show_all_help else argparse.SUPPRESS

    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Validate an existing map-run output, stage its Autoware map '
            'bundle, and open an optional viewer.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_help_epilog(show_advanced=show_all_help),
    )
    parser.add_argument(
        'output_dir',
        help='Completed map-run output directory.',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show advanced viewer-runtime options.',
    )
    viewer_options = parser.add_argument_group('viewer')
    viewer_options.add_argument(
        '--viewer',
        choices=['autoware', 'foxglove'],
        default='autoware',
        help='Viewer to open (default: autoware).',
    )
    runtime_options = parser.add_argument_group('viewer runtime')
    runtime_options.add_argument(
        '--autoware-core-dir',
        metavar='<dir>',
        help=advanced_help('autoware_core checkout used by the map loaders.'),
    )
    runtime_options.add_argument(
        '--work-dir',
        metavar='<dir>',
        help=advanced_help('Runtime workspace directory used by the viewer.'),
    )
    runtime_options.add_argument(
        '--runtime-dir',
        metavar='<dir>',
        help=advanced_help('Existing built Docker viewer runtime to reuse.'),
    )
    runtime_options.add_argument(
        '--rebuild',
        action='store_true',
        help=advanced_help('Rebuild the viewer runtime before opening.'),
    )
    runtime_options.add_argument(
        '--auto-exit-secs',
        type=_positive_seconds,
        metavar='<seconds>',
        help=advanced_help('Automatically close the viewer after N seconds.'),
    )
    return parser.parse_args(argv)


def validate_output_dir(output_dir: Path) -> None:
    """Reject paths that are not complete map-run outputs."""
    if not output_dir.is_dir():
        raise ValueError(f'map output directory does not exist: {output_dir}')

    required = (
        output_dir / 'pointcloud_map' / 'pointcloud_map_metadata.yaml',
        output_dir / 'map_projector_info.yaml',
    )
    missing = [str(path.relative_to(output_dir)) for path in required if not path.is_file()]
    if missing:
        raise ValueError(
            'map output is incomplete; missing: '
            + ', '.join(missing)
            + '. Run "lidarslam-map inspect <output_dir>" for diagnosis.'
        )


def build_viewer_command(
    args: argparse.Namespace,
    output_dir: Path,
) -> list[str]:
    """Translate product options into the maintained viewer helper command."""
    script_name = (
        'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh'
        if args.viewer == 'foxglove'
        else 'run_graph_slam_pointcloud_map_in_autoware.sh'
    )
    command = ['bash', str(SCRIPT_DIR / script_name), str(output_dir)]
    if args.autoware_core_dir:
        command.extend(['--autoware-core-dir', args.autoware_core_dir])
    if args.work_dir:
        command.extend(['--work-dir', args.work_dir])
    if args.runtime_dir:
        command.extend(['--run-dir', args.runtime_dir])
    if args.rebuild:
        command.append('--rebuild')
    if args.auto_exit_secs is not None:
        command.extend(['--auto-exit-secs', str(args.auto_exit_secs)])
    return command


def main(argv: Sequence[str] | None = None) -> int:
    """Validate the map output and propagate the selected viewer result."""
    args = parse_args(argv)
    output_dir = Path(args.output_dir).expanduser().resolve()
    try:
        validate_output_dir(output_dir)
        completed = subprocess.run(
            build_viewer_command(args, output_dir),
            check=False,
        )
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return EX_USAGE
    except OSError as exc:
        print(f'error: failed to start viewer: {exc}', file=sys.stderr)
        return EX_SOFTWARE

    if completed.returncode != 0:
        print(
            f'error: {args.viewer} viewer failed with exit code '
            f'{completed.returncode}.',
            file=sys.stderr,
        )
    return completed.returncode


if __name__ == '__main__':
    raise SystemExit(main())
