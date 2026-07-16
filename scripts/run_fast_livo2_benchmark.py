#!/usr/bin/env python3
"""Run repeated, provenance-checked FAST-LIVO2 benchmarks in Docker."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def benchmark_machine_fingerprint() -> dict[str, Any]:
    """Return a stable, non-secret fingerprint of the benchmark host."""
    private_values = []
    for path in (Path('/etc/machine-id'),
                 Path('/sys/class/dmi/id/product_uuid'),
                 Path('/sys/class/dmi/id/board_serial')):
        try:
            value = path.read_text().strip()
        except OSError:
            value = ''
        if value:
            private_values.append(value)

    cpu_model = ''
    try:
        for line in Path('/proc/cpuinfo').read_text().splitlines():
            if line.lower().startswith('model name') and ':' in line:
                cpu_model = line.split(':', 1)[1].strip()
                break
    except OSError:
        pass
    memory_total_kb = None
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if line.startswith('MemTotal:'):
                memory_total_kb = int(line.split()[1])
                break
    except (OSError, ValueError, IndexError):
        pass

    public = {
        'architecture': platform.machine(),
        'cpu_model': cpu_model,
        'logical_cpu_count': os.cpu_count(),
        'memory_total_kb': memory_total_kb,
    }
    identity_payload = {
        'private_identifiers': private_values,
        **public,
    }
    machine_id = hashlib.sha256(json.dumps(
        identity_payload, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return {'machine_id': machine_id, **public}


def command_output(command: list[str]) -> str:
    return subprocess.run(command, check=True, text=True,
                          capture_output=True).stdout.strip()


def read_int(path: Path) -> int | None:
    try:
        return int(path.read_text().strip())
    except (FileNotFoundError, ValueError):
        return None


def parse_time_report(path: Path) -> dict[str, float | int]:
    if not path.exists():
        return {}
    text = path.read_text(errors='replace')
    result: dict[str, float | int] = {}
    rss = re.search(r'Maximum resident set size \(kbytes\):\s*(\d+)', text)
    elapsed = re.search(
        r'Elapsed \(wall clock\) time \(h:mm:ss or m:ss\):\s*([0-9:.]+)', text)
    if rss:
        result['peak_rss_kb'] = int(rss.group(1))
        result['peak_rss_mb'] = int(rss.group(1)) / 1024.0
    if elapsed:
        parts = [float(value) for value in elapsed.group(1).split(':')]
        result['wall_seconds'] = sum(
            value * 60 ** index for index, value in enumerate(reversed(parts)))
    return result


def parse_bag_bounds(path: Path) -> tuple[float | None, float | None]:
    if not path.exists():
        return None, None
    document = yaml.safe_load(path.read_text()) or {}
    try:
        start = float(document['start'])
        duration = float(document['duration'])
    except (KeyError, TypeError, ValueError):
        return None, None
    return start, start + duration


def processing_rtf_upper_bound(replay_wall_seconds: float | None,
                               drain_seconds: float,
                               sensor_duration: float | None) -> float | None:
    """Conservative processing bound from accelerated replay plus drain.

    This is only evidence when the accelerated run retains the baseline pose
    count and trajectory accuracy; the scorer records it but the gate checker
    is responsible for that cross-run validation.
    """
    if not replay_wall_seconds or not sensor_duration or sensor_duration <= 0.0:
        return None
    return (replay_wall_seconds + max(0.0, drain_seconds)) / sensor_duration


def first_present(row: dict[str, str], names: list[str]) -> float:
    for name in names:
        value = row.get(name)
        if value not in (None, ''):
            return float(value)
    raise KeyError(', '.join(names))


def odometry_csv_to_tum(source: Path, destination: Path) -> dict[str, Any]:
    """Convert ROS1 ``rostopic echo -p`` nav_msgs/Odometry output to TUM."""
    count, malformed = 0, 0
    first_stamp = last_stamp = None
    with destination.open('w', encoding='utf-8') as output:
        if not source.exists():
            return {'samples': 0, 'malformed_rows': 0,
                    'first_stamp': None, 'last_stamp': None}
        with source.open(newline='', errors='replace') as stream:
            for row in csv.DictReader(stream):
                try:
                    stamp = first_present(row, [
                        'field.header.stamp', 'field.header.stamp.secs'])
                    if row.get('field.header.stamp.nsecs'):
                        stamp += float(row['field.header.stamp.nsecs']) * 1e-9
                    elif stamp > 1.0e12:
                        # ROS1 ``rostopic echo -p`` serializes a Time field as
                        # one integer count of nanoseconds on Noetic.
                        stamp *= 1e-9
                    values = [first_present(row, [name]) for name in (
                        'field.pose.pose.position.x', 'field.pose.pose.position.y',
                        'field.pose.pose.position.z', 'field.pose.pose.orientation.x',
                        'field.pose.pose.orientation.y', 'field.pose.pose.orientation.z',
                        'field.pose.pose.orientation.w')]
                except (KeyError, TypeError, ValueError):
                    malformed += 1
                    continue
                output.write(f'{stamp:.9f} ' + ' '.join(
                    f'{value:.12g}' for value in values) + '\n')
                first_stamp = stamp if first_stamp is None else first_stamp
                last_stamp, count = stamp, count + 1
    return {'samples': count, 'malformed_rows': malformed,
            'first_stamp': first_stamp, 'last_stamp': last_stamp}


def git_state(source: Path) -> dict[str, Any]:
    return {
        'revision': command_output(['git', '-C', str(source), 'rev-parse', 'HEAD']),
        'tracked_dirty': bool(command_output([
            'git', '-C', str(source), 'status', '--porcelain',
            '--untracked-files=no'])),
    }


def load_contract(profile_path: Path) -> dict[str, Any]:
    return yaml.safe_load(profile_path.read_text())['competitive_slam_profile']


def bag_container_binding(bag: Path, asset_root: Path) -> tuple[str, list[str]]:
    """Return the in-container bag path and any additional read-only mount."""
    if bag.is_relative_to(asset_root):
        return '/bench/' + bag.relative_to(asset_root).as_posix(), []
    return '/input/input.bag', ['-v', f'{bag}:/input/input.bag:ro']


def fast_log_binding(run_dir: Path, save_map: bool) -> list[str]:
    """Isolate official trajectory logs, and maps when requested, per run."""
    destination = run_dir / 'fast_log'
    (destination / 'result').mkdir(parents=True, exist_ok=True)
    if save_map:
        (destination / 'pcd').mkdir(parents=True, exist_ok=True)
    return ['-v', f'{destination}:/bench/FAST-LIVO2/Log']


def select_fast_map(run_dir: Path) -> Path | None:
    """Select the official downsampled map, never a partial interval PCD."""
    candidates = (
        run_dir / 'fast_log/pcd/all_downsampled_points.pcd',
        run_dir / 'fast_log/pcd/all_raw_points.pcd',
    )
    return next((path for path in candidates
                 if path.is_file() and path.stat().st_size > 0), None)


def evaluate_map_quality(map_path: Path, run_dir: Path) -> dict[str, Any] | None:
    """Run the common geometry evaluator and return its normalized payload."""
    quality_dir = run_dir / 'map_quality'
    command = [
        'bash', str(ROOT / 'scripts/run_map_quality_check.sh'),
        '--input', str(map_path), '--output-dir', str(quality_dir),
        '--runs', '1', '--downsample', '0.1',
        '--setup', str(ROOT / 'install/setup.bash')]
    completed = subprocess.run(
        command, cwd=ROOT, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, check=False)
    report = quality_dir / 'run1/map_quality_report.yaml'
    if completed.returncode != 0 or not report.is_file():
        return None
    payload = yaml.safe_load(report.read_text())['map_quality_report']
    payload['source_map'] = {
        'path': str(map_path.resolve()), 'sha256': sha256(map_path),
        'bytes': map_path.stat().st_size}
    return payload


def tum_trajectory_info(path: Path) -> dict[str, Any]:
    count = malformed = 0
    first_stamp = last_stamp = None
    if not path.is_file():
        return {'samples': 0, 'malformed_rows': 0,
                'first_stamp': None, 'last_stamp': None}
    for line in path.read_text(errors='replace').splitlines():
        fields = line.split()
        try:
            if len(fields) != 8:
                raise ValueError
            stamp = float(fields[0])
            values = [float(value) for value in fields[1:]]
            if not all(value == value for value in [stamp, *values]):
                raise ValueError
        except ValueError:
            malformed += 1
            continue
        first_stamp = stamp if first_stamp is None else first_stamp
        last_stamp, count = stamp, count + 1
    return {'samples': count, 'malformed_rows': malformed,
            'first_stamp': first_stamp, 'last_stamp': last_stamp}


def collect_official_state_trajectory(run_dir: Path) -> tuple[dict[str, Any], str | None]:
    """Collect FAST-LIVO2's exact LiDAR-update-time IMU state trajectory."""
    candidates = sorted(path for path in (run_dir / 'fast_log/result').glob('*.txt')
                        if path.stat().st_size > 0)
    destination = run_dir / 'trajectory_imu.tum'
    if len(candidates) != 1:
        return tum_trajectory_info(destination), None
    shutil.copyfile(candidates[0], destination)
    return tum_trajectory_info(destination), str(candidates[0])


def load_reference_offset(path: Path, source_frame: str) -> tuple[float, float, float]:
    metadata = json.loads(path.read_text())
    key = f'{source_frame}_to_prism_translation_m'
    offset = metadata.get(key)
    if not isinstance(offset, dict) or any(axis not in offset for axis in 'xyz'):
        raise ValueError(f'reference metadata lacks {key}')
    return tuple(float(offset[axis]) for axis in 'xyz')


def apply_tum_translation_offset(source: Path, destination: Path,
                                 offset: tuple[float, float, float]) -> None:
    """Move each TUM pose origin by a fixed offset expressed in its local frame."""
    tx, ty, tz = offset
    lines = []
    for line in source.read_text(errors='replace').splitlines():
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f'invalid TUM line: {line}')
        stamp = fields[0]
        px, py, pz, qx, qy, qz, qw = map(float, fields[1:])
        norm = (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5
        if norm <= 0.0:
            raise ValueError('zero-norm TUM quaternion')
        qw, qx, qy, qz = (value / norm for value in (qw, qx, qy, qz))
        rx = ((1.0 - 2.0 * (qy * qy + qz * qz)) * tx +
              2.0 * (qx * qy - qz * qw) * ty +
              2.0 * (qx * qz + qy * qw) * tz)
        ry = (2.0 * (qx * qy + qz * qw) * tx +
              (1.0 - 2.0 * (qx * qx + qz * qz)) * ty +
              2.0 * (qy * qz - qx * qw) * tz)
        rz = (2.0 * (qx * qz - qy * qw) * tx +
              2.0 * (qy * qz + qx * qw) * ty +
              (1.0 - 2.0 * (qx * qx + qy * qy)) * tz)
        lines.append(
            f'{stamp} {px + rx:.9f} {py + ry:.9f} {pz + rz:.9f} '
            f'{fields[4]} {fields[5]} {fields[6]} {fields[7]}\n')
    destination.write_text(''.join(lines))


def validate_frozen_input_manifest(path: Path | None, hash_key: str,
                                   actual_hash: str) -> dict[str, Any] | None:
    if path is None:
        return None
    document = json.loads(path.read_text())
    if document.get('status') != 'frozen':
        raise ValueError(f'input manifest is not frozen: {path}')
    expected = document.get('hashes', {}).get(hash_key)
    if expected != actual_hash:
        raise ValueError(
            f'input hash differs from frozen manifest: {hash_key} '
            f'expected={expected} actual={actual_hash}')
    return {'path': str(path.resolve()), 'sha256': sha256(path),
            'slot': document.get('slot'), 'sequence': document.get('sequence')}


def run_once(args: argparse.Namespace, asset_root: Path, output: Path,
             run_index: int, shared: dict[str, Any]) -> dict[str, Any]:
    run_dir = output / f'run_{run_index:02d}'
    run_dir.mkdir(parents=True, exist_ok=False)
    bag_inside, bag_mount = bag_container_binding(args.bag, asset_root)
    log_mount = fast_log_binding(run_dir, args.save_map)
    launch_mounts: list[str] = []
    launch_environment: list[str] = []
    if args.mapping_launch is not None:
        launch_mounts += ['-v', f'{args.mapping_launch}:/benchmark_launch.launch:ro']
        launch_environment += ['-e', 'MAPPING_LAUNCH=/benchmark_launch.launch']
    if args.mapping_map_launch is not None:
        launch_mounts += [
            '-v', f'{args.mapping_map_launch}:/benchmark_map_launch.launch:ro']
        launch_environment += [
            '-e', 'MAPPING_MAP_LAUNCH=/benchmark_map_launch.launch']
    command = [
        'docker', 'run', '--rm', '--init', '--name',
        f'fast-livo2-bench-{run_index}-{os.getpid()}',
        '-e', f'BAG_PATH={bag_inside}', '-e', f'RATE={args.rate}',
        '-e', f'SHUTDOWN_GRACE_SECONDS={args.shutdown_grace_seconds}',
        '-e', f'SAVE_MAP={1 if args.save_map else 0}',
        *launch_environment,
        '-v', f'{asset_root}:/bench', *bag_mount, *log_mount, *launch_mounts,
        '-v', f'{ROOT}:/runner:ro',
        '-v', f'{run_dir}:/out', '--entrypoint', '/bin/bash', args.image,
        '/runner/scripts/fast_livo2_container_run.sh']
    started = dt.datetime.now(dt.timezone.utc)
    with (run_dir / 'container_stdout.log').open('w') as stdout, \
            (run_dir / 'container_stderr.log').open('w') as stderr:
        completed = subprocess.run(command, stdout=stdout, stderr=stderr,
                                   check=False)
    finished = dt.datetime.now(dt.timezone.utc)
    legacy_odometry = odometry_csv_to_tum(
        run_dir / 'odometry.csv', run_dir / 'odometry_now_stamp.tum')
    trajectory, official_path = collect_official_state_trajectory(run_dir)
    scoring_path = run_dir / 'trajectory_prism.tum'
    if trajectory['samples'] > 0:
        apply_tum_translation_offset(
            run_dir / 'trajectory_imu.tum', scoring_path, args.frame_offset)
    scoring_trajectory = tum_trajectory_info(scoring_path)
    bag_start, bag_end = parse_bag_bounds(run_dir / 'rosbag_info.yaml')
    end_gap = (None if bag_end is None or trajectory['last_stamp'] is None
               else bag_end - trajectory['last_stamp'])
    bag_time = parse_time_report(run_dir / 'bag_time.txt')
    mapper_time = parse_time_report(run_dir / 'mapper_time.txt')
    duration = None if bag_start is None or bag_end is None else bag_end - bag_start
    rtf = (float(bag_time['wall_seconds']) / duration
           if duration and bag_time.get('wall_seconds') else None)
    processing_bound = processing_rtf_upper_bound(
        bag_time.get('wall_seconds'), args.shutdown_grace_seconds, duration)
    bag_exit = read_int(run_dir / 'bag_exit_status.txt')
    alive_after_bag = read_int(run_dir / 'mapper_alive_after_bag.txt') == 1
    shutdown_exit = read_int(run_dir / 'mapper_shutdown_exit_status.txt')
    map_path = select_fast_map(run_dir) if args.save_map else None
    map_quality = (evaluate_map_quality(map_path, run_dir)
                   if map_path is not None else None)
    complete = (bag_exit == 0 and alive_after_bag and trajectory['samples'] > 0 and
                scoring_trajectory['samples'] == trajectory['samples'] and
                end_gap is not None and end_gap <= args.maximum_end_gap_seconds)
    if args.save_map and map_quality is None:
        complete = False
    report = {
        'schema_version': 1, 'system': 'fast_livo2', 'run_index': run_index,
        'started_at': started.isoformat(), 'finished_at': finished.isoformat(),
        'provenance': shared,
        'execution': {'container_exit_status': completed.returncode,
                      'bag_exit_status': bag_exit,
                      'mapper_alive_after_bag': alive_after_bag,
                      'mapper_shutdown_exit_status': shutdown_exit},
        'completion': {'trajectory_complete': complete,
                       'trajectory_end_gap_seconds': end_gap,
                       'process_exit_status': shutdown_exit},
        'trajectory': trajectory,
        'scoring_trajectory': scoring_trajectory,
        'trajectory_contract': {
            'source': 'official_FAST_LIVO2_Log_result',
            'source_path': official_path,
            'timestamp': 'LidarMeasures.last_lio_update_time',
            'source_frame': args.trajectory_source_frame,
            'target_frame': 'leica_prism',
            'offset_m': dict(zip('xyz', args.frame_offset)),
            'legacy_now_stamp_odometry': legacy_odometry,
        },
        'runtime': {'bag_duration_seconds': duration,
                    'replay_wall_realtime_factor': rtf,
                    'processing_realtime_factor_upper_bound': processing_bound,
                    'processing_measurement_method': (
                        'accelerated_replay_plus_fixed_drain_requires_'
                        'accuracy_and_count_validation'),
                    'mapper': mapper_time, 'bag_player': bag_time},
    }
    if map_quality is not None:
        report['mapping'] = map_quality
    (run_dir / 'run.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--asset-root', type=Path, required=True)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument(
        '--mapping-launch', type=Path,
        help='Optional official-source-compatible roslaunch file for this sensor')
    parser.add_argument(
        '--mapping-map-launch', type=Path,
        help='Map-export roslaunch variant used together with --save-map')
    parser.add_argument('--input-manifest', type=Path)
    parser.add_argument('--reference-meta', type=Path, required=True)
    parser.add_argument('--trajectory-source-frame', choices=('imu', 'body', 'lidar'),
                        default='imu')
    parser.add_argument('--image', default='fast-livo2-benchmark:noetic')
    parser.add_argument('--runs', type=int)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--shutdown-grace-seconds', type=float, default=5.0)
    parser.add_argument('--maximum-end-gap-seconds', type=float, default=0.25)
    parser.add_argument('--save-map', action='store_true')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.asset_root, args.bag = args.asset_root.resolve(), args.bag.resolve()
    args.output = args.output.resolve()
    args.reference_meta = args.reference_meta.resolve()
    if not args.reference_meta.is_file():
        raise ValueError(f'--reference-meta is not a file: {args.reference_meta}')
    args.frame_offset = load_reference_offset(
        args.reference_meta, args.trajectory_source_frame)
    if args.mapping_launch is not None:
        args.mapping_launch = args.mapping_launch.resolve()
        if not args.mapping_launch.is_file():
            raise ValueError(f'--mapping-launch is not a file: {args.mapping_launch}')
    if args.mapping_map_launch is not None:
        args.mapping_map_launch = args.mapping_map_launch.resolve()
        if not args.mapping_map_launch.is_file():
            raise ValueError(
                f'--mapping-map-launch is not a file: {args.mapping_map_launch}')
    if args.save_map and args.mapping_launch is not None and args.mapping_map_launch is None:
        raise ValueError(
            '--mapping-map-launch is required with --save-map when '
            '--mapping-launch is supplied')
    if not args.bag.is_file():
        raise ValueError(f'--bag is not a file: {args.bag}')
    contract = load_contract(args.profile)
    runs = contract['repetitions'] if args.runs is None else args.runs
    if runs < 1:
        raise ValueError('--runs must be positive')
    source_state = git_state(args.asset_root / 'FAST-LIVO2')
    expected = contract['rivals']['fast_livo2']['revision']
    if source_state['revision'] != expected or source_state['tracked_dirty']:
        raise RuntimeError(f'FAST-LIVO2 must be clean at {expected}; got {source_state}')
    args.output.mkdir(parents=True, exist_ok=False)
    bag_hash = sha256(args.bag)
    manifest = validate_frozen_input_manifest(
        args.input_manifest, 'raw_rosbag1_sha256', bag_hash)
    shared = {
        'profile': contract['name'], 'profile_sha256': sha256(args.profile),
        'machine': benchmark_machine_fingerprint(),
        'source': source_state, 'bag_path': str(args.bag),
        'bag_sha256': bag_hash, 'input_manifest': manifest,
        'container_image': args.image,
        'container_image_id': command_output([
            'docker', 'image', 'inspect', args.image, '--format', '{{.Id}}']),
        'rate': args.rate, 'map_export_enabled': args.save_map,
        'reference_metadata': {
            'path': str(args.reference_meta), 'sha256': sha256(args.reference_meta)},
        'trajectory_source_frame': args.trajectory_source_frame,
        'trajectory_to_prism_offset_m': dict(zip('xyz', args.frame_offset)),
        'mapping_launch': None if args.mapping_launch is None else {
            'path': str(args.mapping_launch), 'sha256': sha256(args.mapping_launch)},
        'mapping_map_launch': None if args.mapping_map_launch is None else {
            'path': str(args.mapping_map_launch),
            'sha256': sha256(args.mapping_map_launch)},
    }
    reports = []
    for index in range(1, runs + 1):
        print(f'FAST-LIVO2 repetition {index}/{runs}', flush=True)
        reports.append(run_once(args, args.asset_root, args.output, index, shared))
    summary = {
        'schema_version': 1, 'system': 'fast_livo2', 'requested_runs': runs,
        'completed_trajectories': sum(
            report['completion']['trajectory_complete'] for report in reports),
        'clean_shutdowns': sum(
            report['completion']['process_exit_status'] == 0 for report in reports),
        'completed_maps': sum('mapping' in report for report in reports),
        'runs': reports,
    }
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({key: summary[key] for key in (
        'requested_runs', 'completed_trajectories', 'clean_shutdowns')}, indent=2))
    return 0 if summary['completed_trajectories'] == runs else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
