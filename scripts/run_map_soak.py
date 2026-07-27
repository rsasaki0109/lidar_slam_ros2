#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Run the product map command repeatedly under a bounded soak contract."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import time
from typing import Callable


ROOT = Path(__file__).resolve().parents[1]
PROFILE_DURATIONS = {
    'one-hour': 3600.0,
    'eight-hour': 28800.0,
}
REPORT_NAME = 'soak_report.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/soak-report-v1.schema.json'
)
DROP_SIGNATURES = {
    'message_filter': re.compile(r'Message Filter.*dropp', re.IGNORECASE),
    'scan_drop': re.compile(r'dropp(?:ed|ing).*scan', re.IGNORECASE),
    'queue_overflow': re.compile(r'(?:queue|buffer).*(?:overflow|full)', re.IGNORECASE),
}
GNU_TIME_KEYS = {
    'elapsed': 'Elapsed (wall clock) time (h:mm:ss or m:ss)',
    'peak_rss': 'Maximum resident set size (kbytes)',
    'exit_status': 'Exit status',
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _positive_finite(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be a positive number') from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError('must be finite and greater than zero')
    return parsed


def _nonnegative_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be a non-negative integer') from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError('must be a non-negative integer')
    return parsed


def _nonempty(value: str) -> str:
    parsed = value.strip()
    if not parsed:
        raise argparse.ArgumentTypeError('must not be empty')
    return parsed


def _elapsed_seconds(value: str) -> float:
    fields = value.strip().split(':')
    if len(fields) == 2:
        return float(fields[0]) * 60.0 + float(fields[1])
    if len(fields) == 3:
        return (
            float(fields[0]) * 3600.0
            + float(fields[1]) * 60.0
            + float(fields[2])
        )
    raise ValueError(f'invalid GNU time elapsed value: {value!r}')


def parse_gnu_time(text: str) -> dict[str, float | int]:
    """Parse the stable fields used by the soak evidence contract."""
    fields: dict[str, str] = {}
    for line in text.splitlines():
        stripped = line.strip()
        for name, key in GNU_TIME_KEYS.items():
            prefix = f'{key}:'
            if stripped.startswith(prefix):
                fields[name] = stripped[len(prefix):].strip()
    missing = sorted(set(GNU_TIME_KEYS) - set(fields))
    if missing:
        raise ValueError(f'GNU time report lacks fields: {", ".join(missing)}')
    return {
        'wall_time_sec': _elapsed_seconds(fields['elapsed']),
        'peak_rss_mib': float(fields['peak_rss']) / 1024.0,
        'exit_code': int(fields['exit_status']),
    }


def count_dropped_inputs(text: str) -> dict[str, int]:
    """Count documented log signatures without treating silence as proof."""
    return {
        name: sum(1 for line in text.splitlines() if pattern.search(line))
        for name, pattern in DROP_SIGNATURES.items()
    }


def directory_size_bytes(path: Path) -> int:
    total = 0
    if not path.exists():
        return total
    for candidate in path.rglob('*'):
        if candidate.is_file():
            try:
                total += candidate.stat().st_size
            except FileNotFoundError:
                continue
    return total


def _write_report_atomic(output_root: Path, report: dict[str, object]) -> None:
    destination = output_root / REPORT_NAME
    temporary = output_root / f'.{REPORT_NAME}.tmp'
    try:
        temporary.write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        os.replace(temporary, destination)
    except OSError:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise


def evaluate_report(report: dict[str, object]) -> dict[str, bool]:
    thresholds = report['thresholds']
    metrics = report['metrics']
    return {
        'target_duration_reached': (
            metrics['wall_time_sec'] >= report['target_duration_sec']
        ),
        'all_iterations_succeeded': (
            metrics['iterations_completed'] > 0
            and metrics['iterations_failed'] == 0
        ),
        'peak_rss_within_budget': (
            metrics['peak_rss_mib'] <= thresholds['max_peak_rss_mib']
        ),
        'output_size_within_budget': (
            metrics['output_size_bytes'] <= thresholds['max_output_bytes']
        ),
        'dropped_inputs_within_budget': (
            metrics['dropped_inputs_total'] <= thresholds['max_dropped_inputs']
        ),
    }


def _resolve_cli(value: str | None) -> Path:
    if value:
        candidate = Path(value).expanduser().resolve()
    else:
        installed = shutil.which('lidarslam-map')
        candidate = Path(installed) if installed else ROOT / 'scripts/lidarslam'
    if not candidate.is_file():
        raise ValueError(f'product CLI not found: {candidate}')
    if not os.access(candidate, os.X_OK):
        raise ValueError(f'product CLI is not executable: {candidate}')
    return candidate


def _run_iteration(
    command: list[str],
    log_path: Path,
    time_path: Path,
) -> tuple[int, dict[str, float | int], dict[str, int]]:
    gnu_time = Path('/usr/bin/time')
    if not gnu_time.is_file():
        raise ValueError('GNU /usr/bin/time is required for soak telemetry')
    environment = os.environ.copy()
    environment['LC_ALL'] = 'C'
    timed_command = [
        str(gnu_time),
        '--verbose',
        '--output',
        str(time_path),
        *command,
    ]
    with log_path.open('w', encoding='utf-8') as stream:
        completed = subprocess.run(
            timed_command,
            stdout=stream,
            stderr=subprocess.STDOUT,
            check=False,
            env=environment,
        )
    log_text = log_path.read_text(encoding='utf-8', errors='replace')
    metrics = parse_gnu_time(time_path.read_text(encoding='utf-8'))
    return completed.returncode, metrics, count_dropped_inputs(log_text)


def _initial_report(args: argparse.Namespace, cli: Path) -> dict[str, object]:
    return {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': 'running',
        'last_error': None,
        'profile': args.soak_profile,
        'target_duration_sec': PROFILE_DURATIONS[args.soak_profile],
        'started_at': _utc_now(),
        'completed_at': None,
        'hardware_label': args.hardware_label,
        'bag_path': str(Path(args.bag).expanduser().resolve()),
        'map_profile': args.map_profile,
        'product_cli': str(cli),
        'thresholds': {
            'max_peak_rss_mib': args.max_peak_rss_mib,
            'max_output_bytes': math.ceil(args.max_output_gib * 1024**3),
            'max_dropped_inputs': args.max_dropped_inputs,
        },
        'metrics': {
            'wall_time_sec': 0.0,
            'iterations_completed': 0,
            'iterations_failed': 0,
            'peak_rss_mib': 0.0,
            'output_size_bytes': 0,
            'dropped_inputs_total': 0,
            'minimum_free_space_bytes': None,
        },
        'checks': {},
        'iterations': [],
    }


def run_soak(
    args: argparse.Namespace,
    *,
    clock: Callable[[], float] = time.monotonic,
    run_iteration: Callable[
        [list[str], Path, Path],
        tuple[int, dict[str, float | int], dict[str, int]],
    ] = _run_iteration,
) -> tuple[int, dict[str, object]]:
    """Execute iterations until the profile duration or first failed run."""
    cli = _resolve_cli(args.cli)
    bag = Path(args.bag).expanduser().resolve()
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError(f'rosbag2 metadata.yaml not found under: {bag}')
    output_root = Path(args.output_root).expanduser().resolve()
    if output_root.exists():
        raise ValueError(f'soak output already exists: {output_root}')
    if run_iteration is _run_iteration and not Path('/usr/bin/time').is_file():
        raise ValueError('GNU /usr/bin/time is required for soak telemetry')
    output_root.mkdir(parents=True)
    logs_dir = output_root / 'logs'
    runs_dir = output_root / 'runs'
    logs_dir.mkdir()
    runs_dir.mkdir()

    report = _initial_report(args, cli)
    target = report['target_duration_sec']
    start = clock()
    _write_report_atomic(output_root, report)
    iteration_number = 0
    while clock() - start < target:
        iteration_number += 1
        iteration_id = f'iteration-{iteration_number:04d}'
        run_dir = runs_dir / iteration_id
        log_path = logs_dir / f'{iteration_id}.log'
        time_path = logs_dir / f'{iteration_id}.time'
        command = [
            str(cli),
            'run',
            str(bag),
            '--output-dir',
            str(run_dir),
            '--min-free-space-gib',
            str(args.min_free_space_gib),
        ]
        if args.map_profile:
            command.extend(['--profile', args.map_profile])
        iteration_started = _utc_now()
        try:
            return_code, resources, dropped = run_iteration(
                command,
                log_path,
                time_path,
            )
        except (OSError, ValueError) as exc:
            report['metrics']['wall_time_sec'] = clock() - start
            report['status'] = 'failed'
            report['last_error'] = (
                f'{iteration_id} telemetry or execution failed: {exc}'
            )
            report['completed_at'] = _utc_now()
            report['checks'] = evaluate_report(report)
            _write_report_atomic(output_root, report)
            return 1, report
        elapsed = clock() - start
        output_size = directory_size_bytes(run_dir)
        free_bytes = shutil.disk_usage(output_root).free
        dropped_total = sum(dropped.values())
        iteration = {
            'id': iteration_id,
            'started_at': iteration_started,
            'completed_at': _utc_now(),
            'command': command,
            'command_display': shlex.join(command),
            'exit_code': return_code,
            'wall_time_sec': resources['wall_time_sec'],
            'peak_rss_mib': resources['peak_rss_mib'],
            'output_size_bytes': output_size,
            'free_space_bytes': free_bytes,
            'dropped_input_signatures': dropped,
            'log': str(log_path.relative_to(output_root)),
            'time_report': str(time_path.relative_to(output_root)),
        }
        report['iterations'].append(iteration)
        metrics = report['metrics']
        metrics['wall_time_sec'] = elapsed
        metrics['iterations_completed'] += int(return_code == 0)
        metrics['iterations_failed'] += int(return_code != 0)
        metrics['peak_rss_mib'] = max(
            metrics['peak_rss_mib'],
            resources['peak_rss_mib'],
        )
        metrics['output_size_bytes'] = directory_size_bytes(runs_dir)
        metrics['dropped_inputs_total'] += dropped_total
        previous_minimum = metrics['minimum_free_space_bytes']
        metrics['minimum_free_space_bytes'] = (
            free_bytes
            if previous_minimum is None
            else min(previous_minimum, free_bytes)
        )
        report['checks'] = evaluate_report(report)
        _write_report_atomic(output_root, report)
        if return_code != 0:
            report['last_error'] = (
                f'{iteration_id} exited with code {return_code}'
            )
            break
        budget_checks = (
            'peak_rss_within_budget',
            'output_size_within_budget',
            'dropped_inputs_within_budget',
        )
        failed_budget = next(
            (name for name in budget_checks if not report['checks'][name]),
            None,
        )
        if failed_budget is not None:
            report['last_error'] = f'{iteration_id} failed {failed_budget}'
            break

    report['completed_at'] = _utc_now()
    report['checks'] = evaluate_report(report)
    report['status'] = (
        'passed' if all(report['checks'].values()) else 'failed'
    )
    _write_report_atomic(output_root, report)
    return (0 if report['status'] == 'passed' else 1), report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='rosbag2 directory containing metadata.yaml')
    parser.add_argument('--output-root', required=True)
    parser.add_argument(
        '--soak-profile',
        choices=tuple(PROFILE_DURATIONS),
        required=True,
    )
    parser.add_argument(
        '--hardware-label',
        type=_nonempty,
        required=True,
        help='Stable operator-defined hardware identity recorded in evidence',
    )
    parser.add_argument('--map-profile')
    parser.add_argument('--cli', help='Product CLI path (default: lidarslam-map)')
    parser.add_argument(
        '--max-peak-rss-mib',
        type=_positive_finite,
        required=True,
    )
    parser.add_argument(
        '--max-output-gib',
        type=_positive_finite,
        required=True,
    )
    parser.add_argument(
        '--max-dropped-inputs',
        type=_nonnegative_int,
        required=True,
    )
    parser.add_argument(
        '--min-free-space-gib',
        type=_positive_finite,
        default=5.0,
    )
    return parser.parse_args()


def main() -> int:
    try:
        exit_code, report = run_soak(parse_args())
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=os.sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True))
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
