#!/usr/bin/env python3

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parent.parent
VERIFY_SCRIPT = REPO_ROOT / 'scripts' / 'verify_autoware_map.py'


def _load_verify_module():
    spec = importlib.util.spec_from_file_location(
        'verify_autoware_map',
        VERIFY_SCRIPT,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError('failed to load verify_autoware_map.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


VERIFY_MODULE = _load_verify_module()
MapVerifier = VERIFY_MODULE.MapVerifier


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _read_pose_count(path: Path) -> int:
    if not path.is_file():
        return 0
    return sum(
        1
        for line in path.read_text(encoding='utf-8', errors='replace').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    )


def _parse_ape_report(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    metrics: dict[str, Any] = {'path': str(path)}
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if ':' not in line:
            continue
        key, raw_value = line.split(':', 1)
        key = key.strip()
        value = raw_value.strip()
        if key == 'alignment':
            metrics[key] = value
            continue
        try:
            numeric = float(value)
        except ValueError:
            continue
        if key == 'pairs':
            metrics[key] = int(numeric)
        else:
            metrics[key] = numeric
    return metrics


def _bag_duration_seconds(metadata_path: Path) -> float | None:
    if not metadata_path.is_file():
        return None
    lines = metadata_path.read_text(encoding='utf-8', errors='replace').splitlines()
    in_duration = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('duration:'):
            in_duration = True
            continue
        if in_duration and stripped.startswith('nanoseconds:'):
            try:
                nanoseconds = int(stripped.split(':', 1)[1].strip())
            except ValueError:
                return None
            return nanoseconds / 1e9
        if in_duration and stripped and not line.startswith(' '):
            break
    return None


def _read_reference_meta(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    return _read_json(path)


def _verify_map(pointcloud_dir: Path) -> dict[str, Any] | None:
    if not pointcloud_dir.is_dir():
        return None
    verifier = MapVerifier(str(pointcloud_dir))
    ok = verifier.run()
    return {
        'ok': ok,
        'passes': verifier.passes,
        'warnings': verifier.warnings,
        'failures': verifier.failures,
    }


def _fmt_path(path: Path | None) -> str:
    return str(path) if path is not None else ''


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Write a metrics.json for an RKO-LIO + graph_based_slam benchmark '
            'run so it can be consumed by the repo reporting tools.'
        ),
    )
    parser.add_argument('--out-dir', required=True, help='Benchmark output directory')
    parser.add_argument('--bag', required=True, help='rosbag2 directory used for the run')
    parser.add_argument('--reference-tum', required=True, help='Reference TUM trajectory')
    parser.add_argument(
        '--reference-meta',
        default='',
        help='Optional JSON sidecar emitted by generate_ntu_viral_tnp01_reference.py',
    )
    parser.add_argument(
        '--reference-source',
        default='leica_prism_gt',
        help='Reference source label stored in metrics.json',
    )
    parser.add_argument(
        '--points-topic',
        default='/os1_cloud_node1/points',
        help='LiDAR topic used for the run',
    )
    parser.add_argument(
        '--imu-topic',
        default='/imu/imu',
        help='IMU topic used for the run',
    )
    parser.add_argument(
        '--lidarslam-param',
        default='lidarslam/param/lidarslam.yaml',
        help='graph_based_slam parameter YAML',
    )
    parser.add_argument(
        '--rko-param',
        default='lidarslam/param/rko_lio_ntu_viral.yaml',
        help='RKO-LIO parameter YAML',
    )
    parser.add_argument(
        '--run-name',
        default='',
        help='RKO-LIO run name tag',
    )
    parser.add_argument(
        '--raw-tum',
        default='',
        help='Raw odometry TUM path (default: auto-detect in out-dir)',
    )
    parser.add_argument(
        '--corrected-tum',
        default='',
        help='Corrected path TUM path (default: auto-detect in out-dir)',
    )
    parser.add_argument(
        '--raw-ape',
        default='',
        help='Raw APE report path (default: auto-detect in out-dir)',
    )
    parser.add_argument(
        '--corrected-ape',
        default='',
        help='Corrected APE report path (default: auto-detect in out-dir)',
    )
    parser.add_argument(
        '--launch-log',
        default='',
        help='Launch log path (default: <out-dir>/slam.launch.log)',
    )
    parser.add_argument(
        '--wall-sec',
        type=float,
        default=None,
        help='Measured wall time for the full benchmark run',
    )
    parser.add_argument(
        '--started-at',
        default='',
        help='Optional ISO-8601 start timestamp',
    )
    parser.add_argument(
        '--started-at-unix',
        type=int,
        default=None,
        help='Optional unix start timestamp',
    )
    parser.add_argument(
        '--metrics-out',
        default='',
        help='Output metrics path (default: <out-dir>/metrics.json)',
    )
    parser.add_argument(
        '--pipeline',
        default='rko_lio',
        choices=('rko_lio', 'lo', 'small_gicp'),
        help=(
            'rko_lio: RKO-LIO frontend; '
            'lo: scanmatcher LiDAR-only frontend; '
            'small_gicp: small_gicp ICP/GICP odometry frontend.'
        ),
    )
    parser.add_argument(
        '--robot-frame-id',
        default='base_link',
        help='Robot frame label stored in metrics (LO / visualization).',
    )
    parser.add_argument(
        '--raw-path-topic',
        default='/path',
        help='Scanmatcher Path topic (LO pipeline).',
    )
    parser.add_argument(
        '--corrected-path-topic',
        default='/modified_path',
        help='Graph Path topic (LO pipeline).',
    )
    args = parser.parse_args()

    out_dir = Path(args.out_dir).expanduser().resolve()
    bag_path = Path(args.bag).expanduser().resolve()
    reference_tum = Path(args.reference_tum).expanduser().resolve()
    reference_meta = (
        Path(args.reference_meta).expanduser().resolve()
        if args.reference_meta else None
    )
    lidarslam_param = Path(args.lidarslam_param).expanduser().resolve()
    rko_param = Path(args.rko_param).expanduser().resolve()
    metrics_path = (
        Path(args.metrics_out).expanduser().resolve()
        if args.metrics_out else out_dir / 'metrics.json'
    )

    raw_tum = (
        Path(args.raw_tum).expanduser().resolve()
        if args.raw_tum else out_dir / 'traj_raw_prism.tum'
    )
    corrected_tum = (
        Path(args.corrected_tum).expanduser().resolve()
        if args.corrected_tum else out_dir / 'traj_corrected_prism.tum'
    )
    if not raw_tum.is_file():
        raw_tum = out_dir / 'traj_raw.tum'
    if not corrected_tum.is_file():
        corrected_tum = out_dir / 'traj_corrected.tum'

    raw_ape_path = (
        Path(args.raw_ape).expanduser().resolve()
        if args.raw_ape else out_dir / 'ape_raw_vs_gt.txt'
    )
    corrected_ape_path = (
        Path(args.corrected_ape).expanduser().resolve()
        if args.corrected_ape else out_dir / 'ape_corrected_vs_gt.txt'
    )
    launch_log = (
        Path(args.launch_log).expanduser().resolve()
        if args.launch_log else out_dir / 'slam.launch.log'
    )

    bag_duration_sec = _bag_duration_seconds(bag_path / 'metadata.yaml')
    reference_meta_data = _read_reference_meta(reference_meta) if reference_meta else {}
    raw_ape = _parse_ape_report(raw_ape_path)
    corrected_ape = _parse_ape_report(corrected_ape_path)
    map_verify = _verify_map(out_dir / 'pointcloud_map')

    corrected_success = corrected_tum.is_file() and corrected_ape is not None
    raw_success = raw_tum.is_file() and raw_ape is not None
    wall_sec = args.wall_sec
    rtf = None
    if wall_sec is not None and bag_duration_sec and bag_duration_sec > 0.0:
        rtf = wall_sec / bag_duration_sec

    if args.pipeline in ('lo', 'small_gicp'):
        frames: dict[str, str] = {
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'robot_frame_id': args.robot_frame_id,
            'points_frame_id': args.robot_frame_id,
        }
    else:
        frames = {
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'robot_frame_id': 'os_sensor',
            'points_frame_id': 'os_sensor',
        }

    metrics: dict[str, Any] = {
        'started_at': args.started_at or None,
        'started_at_unix': args.started_at_unix,
        'pipeline': args.pipeline,
        'out_dir': str(out_dir),
        'bag_path': str(bag_path),
        'bag_duration_sec': bag_duration_sec,
        'points_topic': args.points_topic,
        'imu_topic': args.imu_topic,
        'frames': frames,
        'reference': {
            'source': reference_meta_data.get('source', args.reference_source),
            'tum_path': str(reference_tum),
            'topic': reference_meta_data.get('topic', '/leica/pose/relative'),
            'meta_path': _fmt_path(reference_meta),
            'source_bag': reference_meta_data.get('source_bag', ''),
        },
        'lidarslam': {
            'success': corrected_success or raw_success,
            'wall_sec': wall_sec,
            'rtf': rtf,
            'tum_path': str(corrected_tum if corrected_tum.is_file() else raw_tum),
            'tum_lines': _read_pose_count(corrected_tum if corrected_tum.is_file() else raw_tum),
            'log_path': str(launch_log) if launch_log.is_file() else '',
            'param_path': str(lidarslam_param),
            'out_dir': str(out_dir),
        },
        'glim': {
            'available': False,
            'success': False,
            'reference_source': reference_meta_data.get('source', args.reference_source),
        },
        'rko_lio': (
            {
                'available': False,
                'note': f'{args.pipeline} pipeline does not use RKO-LIO.',
            }
            if args.pipeline != 'rko_lio'
            else {
                'available': True,
                'run_name': args.run_name or out_dir.name,
                'param_path': str(rko_param),
                'raw_tum_path': str(raw_tum) if raw_tum.is_file() else '',
                'raw_tum_lines': _read_pose_count(raw_tum),
                'raw_ape': raw_ape,
                'corrected_tum_path': str(corrected_tum) if corrected_tum.is_file() else '',
                'corrected_tum_lines': _read_pose_count(corrected_tum),
                'corrected_ape': corrected_ape,
                'reference_meta_path': _fmt_path(reference_meta),
                'prism_offset_m': reference_meta_data.get('lidar_to_prism_translation_m'),
            }
        ),
        'scanmatcher_lo': (
            {
                'lidarslam_param_path': str(lidarslam_param),
                'raw_path_topic': args.raw_path_topic,
                'corrected_path_topic': args.corrected_path_topic,
                'raw_tum_path': str(raw_tum) if raw_tum.is_file() else '',
                'raw_tum_lines': _read_pose_count(raw_tum),
                'raw_ape': raw_ape,
                'corrected_tum_path': str(corrected_tum) if corrected_tum.is_file() else '',
                'corrected_tum_lines': _read_pose_count(corrected_tum),
                'corrected_ape': corrected_ape,
                'reference_meta_path': _fmt_path(reference_meta),
                'prism_offset_m': reference_meta_data.get('lidar_to_prism_translation_m'),
            }
            if args.pipeline == 'lo'
            else {
                'available': False,
            }
        ),
        'small_gicp_lo': (
            {
                'available': True,
                'frontend_param_path': str(rko_param),
                'raw_odom_topic': '/odom',
                'raw_tum_path': str(raw_tum) if raw_tum.is_file() else '',
                'raw_tum_lines': _read_pose_count(raw_tum),
                'raw_ape': raw_ape,
                'corrected_tum_path': str(corrected_tum) if corrected_tum.is_file() else '',
                'corrected_tum_lines': _read_pose_count(corrected_tum),
                'corrected_ape': corrected_ape,
                'reference_meta_path': _fmt_path(reference_meta),
                'prism_offset_m': reference_meta_data.get('lidar_to_prism_translation_m'),
            }
            if args.pipeline == 'small_gicp'
            else {
                'available': False,
            }
        ),
        'graph_based_slam': {
            'corrected_path_available': corrected_tum.is_file(),
            'map_projector_info_path': str(out_dir / 'map_projector_info.yaml')
            if (out_dir / 'map_projector_info.yaml').is_file() else '',
            'pointcloud_map_dir': str(out_dir / 'pointcloud_map')
            if (out_dir / 'pointcloud_map').is_dir() else '',
            'map_verify': map_verify,
        },
        'evo': {
            'ape_log_path': str(corrected_ape_path) if corrected_ape_path.is_file() else '',
            'ape': corrected_ape,
            'raw_ape_log_path': str(raw_ape_path) if raw_ape_path.is_file() else '',
            'raw_ape': raw_ape,
        },
    }

    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    metrics_path.write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    print(metrics_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
