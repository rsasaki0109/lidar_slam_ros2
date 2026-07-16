#!/usr/bin/env python3
"""Run the pinned official GLIM CPU track repeatedly in Docker."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

from run_fast_livo2_benchmark import (
    apply_tum_translation_offset, benchmark_machine_fingerprint,
    evaluate_map_quality, load_reference_offset, parse_time_report, read_int,
    validate_frozen_input_manifest)
import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'


def sha256_tree(path: Path) -> str:
    digest = hashlib.sha256()
    for candidate in sorted(item for item in path.rglob('*') if item.is_file()):
        digest.update(candidate.relative_to(path).as_posix().encode())
        digest.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def bag_bounds(metadata_path: Path) -> tuple[float, float]:
    info = yaml.safe_load(metadata_path.read_text())['rosbag2_bagfile_information']
    start = info['starting_time']['nanoseconds_since_epoch'] * 1e-9
    duration = info['duration']['nanoseconds'] * 1e-9
    return start, start + duration


def trajectory_info(path: Path) -> dict[str, Any]:
    rows = []
    if path.exists():
        for line in path.read_text(errors='replace').splitlines():
            if not line or line.startswith('#'):
                continue
            fields = line.split()
            if len(fields) >= 8:
                rows.append(fields)
    return {
        'samples': len(rows),
        'first_stamp': float(rows[0][0]) if rows else None,
        'last_stamp': float(rows[-1][0]) if rows else None,
    }


def image_labels(image: str) -> dict[str, str]:
    output = subprocess.run(
        ['docker', 'image', 'inspect', image], check=True, text=True,
        capture_output=True).stdout
    inspection = json.loads(output)[0]
    return inspection['Id'], inspection['Config'].get('Labels') or {}


def run_once(args: argparse.Namespace, output: Path, index: int,
             shared: dict[str, Any], duration: float, bag_end: float) -> dict[str, Any]:
    run_dir = output / f'run_{index:02d}'
    run_dir.mkdir(parents=True, exist_ok=False)
    command = [
        'docker', 'run', '--rm', '--init', '--name',
        f'glim-cpu-bench-{index}-{os.getpid()}',
        '-e', f'BAG_PATH=/data/{args.bag.name}',
        '-e', 'OVERRIDE_CONFIG=/benchmark_config',
        '-v', f'{args.bag.parent}:/data:ro',
        '-v', f'{args.config_dir}:/benchmark_config:ro',
        '-v', f'{ROOT}:/runner:ro', '-v', f'{run_dir}:/out',
        args.image, '/bin/bash', '/runner/scripts/glim_container_run.sh']
    started = dt.datetime.now(dt.timezone.utc)
    with (run_dir / 'container_stdout.log').open('w') as stdout, \
            (run_dir / 'container_stderr.log').open('w') as stderr:
        completed = subprocess.run(command, stdout=stdout, stderr=stderr,
                                   check=False)
    finished = dt.datetime.now(dt.timezone.utc)
    process_time = parse_time_report(run_dir / 'process_time.txt')
    raw_trajectory_path = run_dir / 'dump/traj_lidar.txt'
    trajectory = trajectory_info(raw_trajectory_path)
    scoring_path = run_dir / 'trajectory_prism.tum'
    if trajectory['samples'] > 0:
        apply_tum_translation_offset(
            raw_trajectory_path, scoring_path, args.frame_offset)
    scoring_trajectory = trajectory_info(scoring_path)
    end_gap = (None if trajectory['last_stamp'] is None
               else bag_end - trajectory['last_stamp'])
    processing_rtf = (float(process_time['wall_seconds']) / duration
                      if process_time.get('wall_seconds') else None)
    process_exit = read_int(run_dir / 'process_exit_status.txt')
    map_quality = None
    if args.save_maps and process_exit == 0:
        map_path = run_dir / 'map.pcd'
        exported = subprocess.run([
            sys.executable, str(ROOT / 'scripts/export_glim_dump_map.py'),
            '--dump', str(run_dir / 'dump'), '--output', str(map_path)],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            check=False)
        if exported.returncode == 0 and map_path.is_file():
            map_quality = evaluate_map_quality(map_path, run_dir)
    complete = (completed.returncode == 0 and process_exit == 0 and
                trajectory['samples'] > 0 and end_gap is not None and
                scoring_trajectory['samples'] == trajectory['samples'] and
                end_gap <= args.maximum_end_gap_seconds)
    if args.save_maps and map_quality is None:
        complete = False
    report = {
        'schema_version': 1, 'system': 'glim_cpu', 'run_index': index,
        'started_at': started.isoformat(), 'finished_at': finished.isoformat(),
        'provenance': shared,
        'execution': {'container_exit_status': completed.returncode,
                      'process_exit_status': process_exit},
        'completion': {'trajectory_complete': complete,
                       'trajectory_end_gap_seconds': end_gap,
                       'process_exit_status': process_exit},
        'trajectory': trajectory,
        'scoring_trajectory': scoring_trajectory,
        'trajectory_contract': {
            'source': 'official_GLIM_dump_traj_lidar',
            'source_frame': 'lidar', 'target_frame': 'leica_prism',
            'offset_m': dict(zip('xyz', args.frame_offset)),
        },
        'runtime': {'bag_duration_seconds': duration,
                    'processing_realtime_factor': processing_rtf,
                    **process_time},
    }
    if map_quality is not None:
        report['mapping'] = map_quality
    (run_dir / 'run.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument(
        '--config-dir', type=Path,
        default=ROOT / 'configs/glim/hilti2022_cpu',
        help='GLIM override directory containing the four benchmark JSON files')
    parser.add_argument('--input-manifest', type=Path)
    parser.add_argument('--reference-meta', type=Path, required=True)
    parser.add_argument('--image', default='glim-cpu-benchmark:competitive-v1')
    parser.add_argument('--runs', type=int)
    parser.add_argument('--save-maps', action='store_true')
    parser.add_argument('--maximum-end-gap-seconds', type=float, default=0.25)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.bag, args.output = args.bag.resolve(), args.output.resolve()
    args.config_dir = args.config_dir.resolve()
    args.reference_meta = args.reference_meta.resolve()
    if not args.reference_meta.is_file():
        raise ValueError(f'--reference-meta is not a file: {args.reference_meta}')
    args.frame_offset = load_reference_offset(args.reference_meta, 'lidar')
    contract = yaml.safe_load(args.profile.read_text())['competitive_slam_profile']
    runs = contract['repetitions'] if args.runs is None else args.runs
    if runs < 1 or not (args.bag / 'metadata.yaml').exists():
        raise ValueError('positive runs and a ROS2 bag directory are required')
    required_configs = {
        'config.json', 'config_sensors.json', 'config_ros.json',
        'config_logging.json'}
    missing_configs = sorted(
        name for name in required_configs if not (args.config_dir / name).is_file())
    if missing_configs:
        raise ValueError(
            f'--config-dir is missing required files: {missing_configs}')
    image_id, labels = image_labels(args.image)
    rival = contract['rivals']['glim']
    expected_labels = {
        'benchmark.glim.revision': rival['revision'],
        'benchmark.glim_ros2.revision': rival['ros2_revision'],
    }
    mismatches = {key: (labels.get(key), expected)
                  for key, expected in expected_labels.items()
                  if labels.get(key) != expected}
    if mismatches:
        raise RuntimeError(f'container revision labels mismatch: {mismatches}')
    args.output.mkdir(parents=True, exist_ok=False)
    start, end = bag_bounds(args.bag / 'metadata.yaml')
    bag_hash = sha256_tree(args.bag)
    manifest = validate_frozen_input_manifest(
        args.input_manifest, 'canonical_rosbag2_tree_sha256', bag_hash)
    shared = {
        'profile': contract['name'], 'image': args.image, 'image_id': image_id,
        'machine': benchmark_machine_fingerprint(),
        'image_labels': labels, 'bag_path': str(args.bag),
        'bag_sha256': bag_hash, 'input_manifest': manifest,
        'config_path': str(args.config_dir),
        'config_sha256': sha256_tree(args.config_dir),
        'reference_metadata': {
            'path': str(args.reference_meta),
            'sha256': hashlib.sha256(args.reference_meta.read_bytes()).hexdigest()},
        'trajectory_source_frame': 'lidar',
        'trajectory_to_prism_offset_m': dict(zip('xyz', args.frame_offset)),
    }
    reports = []
    for index in range(1, runs + 1):
        print(f'GLIM CPU repetition {index}/{runs}', flush=True)
        reports.append(run_once(args, args.output, index, shared, end - start, end))
    summary = {
        'schema_version': 1, 'system': 'glim_cpu', 'requested_runs': runs,
        'completed_trajectories': sum(
            report['completion']['trajectory_complete'] for report in reports),
        'clean_exits': sum(
            report['completion']['process_exit_status'] == 0 for report in reports),
        'completed_maps': sum('mapping' in report for report in reports),
        'runs': reports,
    }
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({key: summary[key] for key in (
        'requested_runs', 'completed_trajectories', 'clean_exits')}, indent=2))
    return 0 if summary['completed_trajectories'] == runs else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, KeyError,
            subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
