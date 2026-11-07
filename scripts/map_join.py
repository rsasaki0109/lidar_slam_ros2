#!/usr/bin/env python3
"""Place-recognition alignment of map bundles with graph_slam_multisession_join.

`lidarslam-map merge --place-recognition` uses this to find how each later map bundle sits in the
first one's frame before the usual alignment and overlap gates run. It needs the deterministic
backend input that an editable run records (`backend_input/` in the bundle) and the graph parameter
file of those runs (`--params`, else the anchor bundle's `graph_params.ros.yaml`, as map edits do):

1. Each bundle's backend is replayed offline (run_offline_determinism_check.sh, one run), which
   gives the runner outputs the join reads (trajectory_raw.tum, loop_edges.csv).
2. graph_slam_multisession_join joins every session to the map of the sessions joined before it,
   by place recognition, with its refusals (ambiguous sets, self-similar sessions, the whole-map
   check). docs/architecture/multisession-join.md describes the method.
3. A session that does not join stops the merge with the join's reason; a joined session's
   transform to the first session becomes its initial transform (x, y, z, yaw).
"""
from __future__ import annotations

import math
import shlex
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import yaml

from map_edit import MapEditError, resolve_bundle_dir


class MapJoinError(ValueError):
    """Place-recognition alignment cannot place every session."""


@dataclass(frozen=True)
class JoinSettings:
    """Inputs of a place-recognition alignment."""

    params_path: Path | None = None
    setup_path: Path | None = None
    num_threads: int = 4
    replay_script: Path | None = None
    join_command: Sequence[str] | None = None


def bundle_dir_of(source: Path) -> Path:
    """Return the map bundle of a completed output (the bundle itself or its run directory)."""
    try:
        return resolve_bundle_dir(source)
    except MapEditError as exc:
        raise MapJoinError(str(exc)) from exc


def graph_params(anchor_bundle: Path, settings: JoinSettings) -> Path:
    """Return --params, else the graph parameters an editable anchor run recorded."""
    if settings.params_path is not None:
        path = settings.params_path.expanduser().resolve()
    else:
        path = anchor_bundle / 'graph_params.ros.yaml'
    if not path.is_file():
        raise MapJoinError(
            f'graph parameter file not found: {path}; pass --params or use an editable output '
            'that recorded graph_params.ros.yaml')
    return path


def backend_input_dir(bundle_dir: Path) -> Path:
    """Return the bundle's recorded backend input, or raise naming how to record one."""
    backend = bundle_dir / 'backend_input'
    if not (backend / 'metadata.yaml').is_file():
        raise MapJoinError(
            f'{bundle_dir} has no backend_input/; place recognition needs the backend input of an '
            'editable run (record the session again with --editable)'
        )
    return backend


def _run(command: list[str], what: str) -> None:
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    if completed.returncode != 0:
        tail = '\n'.join((completed.stdout + '\n' + completed.stderr).splitlines()[-20:])
        raise MapJoinError(f'{what} failed with exit code {completed.returncode}:\n{tail}')


def replay_backend(bundle_dir: Path, work_dir: Path, params: Path,
                   settings: JoinSettings) -> Path:
    """Replay one bundle's backend offline; returns the run directory with the runner outputs."""
    script = (
        settings.replay_script.expanduser().resolve() if settings.replay_script
        else Path(__file__).resolve().parent / 'run_offline_determinism_check.sh'
    )
    if not script.is_file():
        raise MapJoinError(f'offline replay helper is missing: {script}')
    command = [
        'bash', str(script),
        '--bag', str(backend_input_dir(bundle_dir).resolve()),
        '--params', str(params),
        '--runs', '1',
        '--output-dir', str(work_dir),
    ]
    if settings.setup_path is not None:
        command[2:2] = ['--setup', str(settings.setup_path.expanduser().resolve())]
    _run(command, f'offline replay of {bundle_dir}')
    run_dir = work_dir / 'run1'
    missing = [name for name in ('trajectory_raw.tum', 'loop_edges.csv')
               if not (run_dir / name).is_file()]
    if missing:
        raise MapJoinError(
            f'offline replay of {bundle_dir} did not produce: ' + ', '.join(missing))
    return run_dir


def _default_join_command(setup_path: Path | None) -> list[str]:
    source = ''
    if setup_path:
        source = f'source {shlex.quote(str(setup_path.expanduser().resolve()))} && '
    return [
        'bash', '-c',
        f'set -e; {source}prefix=$(ros2 pkg prefix graph_based_slam); '
        'exec "$prefix/lib/graph_based_slam/graph_slam_multisession_join" "$@"',
        'graph_slam_multisession_join',
    ]


def run_join(bags: list[Path], run_dirs: list[Path], output_dir: Path, params: Path,
             settings: JoinSettings) -> dict[str, Any]:
    """Run graph_slam_multisession_join and return its parsed join_report.yaml."""
    command = (list(settings.join_command) if settings.join_command
               else _default_join_command(settings.setup_path))
    quoted = ','.join(f"'{path.resolve()}'" for path in bags)
    runs = ','.join(f"'{path.resolve()}'" for path in run_dirs)
    command += [
        '--ros-args', '--params-file', str(params),
        '-p', f'session_bag_paths:=[{quoted}]',
        '-p', f'session_backend_dirs:=[{runs}]',
        '-p', f'multisession_num_threads:={settings.num_threads}',
        '-p', f'output_dir:={output_dir}',
    ]
    _run(command, 'graph_slam_multisession_join')
    report_path = output_dir / 'join_report.yaml'
    try:
        report = yaml.safe_load(report_path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise MapJoinError(f'cannot read {report_path}: {exc}') from exc
    if not isinstance(report, dict) or not isinstance(report.get('sessions'), list):
        raise MapJoinError(f'{report_path} has no sessions list')
    return report


def refusal_reason(session: dict[str, Any]) -> str:
    """Return why the join left a session out, in the join report's terms."""
    if session.get('ambiguous'):
        return 'two sets of matches fit with different transforms (ambiguous place)'
    if session.get('self_similar'):
        return (f"session {session['self_similar']} joins itself (its place repeats itself, so a "
                'join to it cannot be told apart from a wrong one)')
    sets = session.get('sets_tried') or []
    if not sets:
        return (f"{session.get('accepted', 0)} of {session.get('candidates', 0)} place matches "
                'were accepted, too few to agree on one transform at 3 places 5 m apart')
    best = sets[0]
    return (f"the whole-map check refused every set tried; the largest ({best.get('size')} "
            f"matches) left {100.0 * float(best.get('share_below', 0.0)):.1f} % of "
            f"{best.get('places')} places with less than half their points overlapping "
            '(at most 10 % allowed)')


def initial_transform(row: Sequence[float]) -> tuple[float, float, float, float]:
    """Return (x, y, z, yaw in degrees) of a 3x4 row-major transform."""
    if len(row) != 12:
        raise MapJoinError(f'expected a 3x4 transform, got {len(row)} values')
    return (float(row[3]), float(row[7]), float(row[11]),
            math.degrees(math.atan2(float(row[4]), float(row[0]))))


def place_recognition_alignments(
    source_dirs: list[Path], work_dir: Path, settings: JoinSettings
) -> tuple[dict[int, tuple[float, float, float, float]], dict[str, Any]]:
    """Return the initial transforms of sessions 1.. in session 0's frame and a summary."""
    bundles = [bundle_dir_of(path) for path in source_dirs]
    bags = [backend_input_dir(path) for path in bundles]
    params = graph_params(bundles[0], settings)
    run_dirs = [replay_backend(path, work_dir / f'session{index}', params, settings)
                for index, path in enumerate(bundles)]
    report = run_join(bags, run_dirs, work_dir / 'join', params, settings)
    transforms: dict[int, tuple[float, float, float, float]] = {}
    refused = []
    for session in report['sessions']:
        index = int(session.get('session', -1))
        if index <= 0:
            continue
        if not session.get('joined'):
            refused.append(f'session {index} ({source_dirs[index]}): {refusal_reason(session)}')
            continue
        transforms[index] = initial_transform(session.get('transform_to_session_0') or [])
    if refused:
        raise MapJoinError('place recognition did not join every session: ' + '; '.join(refused))
    missing = [index for index in range(1, len(source_dirs)) if index not in transforms]
    if missing:
        raise MapJoinError(f'join report has no result for sessions {missing}')
    summary = {
        'method': 'graph_slam_multisession_join',
        'params': str(params),
        'sessions': [
            {key: session.get(key) for key in (
                'session', 'joined', 'candidates', 'accepted', 'transform_to_session_0')}
            for session in report['sessions']
        ],
        'map_joins': report.get('map_joins'),
    }
    return transforms, summary
