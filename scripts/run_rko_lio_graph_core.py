#!/usr/bin/env python3
"""The RKO-LIO + graph_based_slam map without ROS 2 (the "core" engine of lidarslam-map run).

The counterpart of run_rko_lio_graph_autoware_dogfood.sh for an installation without ROS 2: the
same inputs (bag, topics, the two parameter files, frames) and the same map outputs
(pointcloud_map/, map_projector_info.yaml, map.pcd, traj_raw.tum, traj_corrected.tum,
lanelet2_map.osm), computed by the lidarslam Python package (pip install ./lidarslam_core):
RKO-LIO and the offline graph_based_slam backend in one pass (lidarslam.mapping).

The odometry parameters are assembled as rko_lio_slam.launch.py assembles the offline node's: the
launch defaults and the frames given here first, then the RKO-LIO file on top. The backend takes
the graph_based_slam section of the lidarslam file and the LiDAR extrinsic of the RKO-LIO file.
The backend is the offline pipeline (deterministic), not the live graph node, so its map is not
expected to equal a ROS 2 run's bytes.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent

# rko_lio_slam.launch.py: the offline node's parameters before the RKO-LIO file is applied.
LAUNCH_ODOMETRY_DEFAULTS: dict[str, Any] = {
    'odom_frame': 'odom',
    'lidar_frame': '',
    'imu_frame': '',
    'deskew': True,
    'voxel_size': 1.0,
    'max_range': 100.0,
    'min_range': 1.0,
    'initialization_phase': False,
    'skip_to_time': 0.0,
    'publish_deskewed_scan': True,
}


def core_engine_available() -> tuple[bool, str]:
    """Whether the lidarslam package with the odometry and the backend can be imported."""
    try:
        from lidarslam import _core  # noqa: F401
    except ImportError as exc:
        return False, f'the lidarslam Python package is not installed ({exc})'
    missing = [name for name in ('OdometryDriver', 'BackendPipeline') if not hasattr(_core, name)]
    if missing:
        return False, f'the lidarslam package was built without {", ".join(missing)}'
    return True, 'lidarslam ' + getattr(sys.modules.get('lidarslam'), '__version__', '?')


def odometry_parameters(rko_param: Path, lidar_topic: str, imu_topic: str, base_frame: str,
                        lidar_frame: str | None, imu_frame: str | None) -> dict[str, Any]:
    """Return the offline node's parameters: launch defaults and frames, then the RKO-LIO file."""
    from lidarslam.params import load_parameters

    params: dict[str, Any] = dict(LAUNCH_ODOMETRY_DEFAULTS)
    params.update({'lidar_topic': lidar_topic, 'imu_topic': imu_topic, 'base_frame': base_frame})
    if lidar_frame:
        params['lidar_frame'] = lidar_frame
    if imu_frame:
        params['imu_frame'] = imu_frame
    params.update(load_parameters(rko_param))
    return params


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--bag', required=True, help='rosbag2 directory or ROS 1 .bag file')
    parser.add_argument('--lidar-topic', required=True)
    parser.add_argument('--imu-topic', required=True)
    parser.add_argument('--lidarslam-param', required=True, type=Path,
                        help='graph_based_slam parameter YAML')
    parser.add_argument('--rko-param', required=True, type=Path, help='RKO-LIO parameter YAML')
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--lidar-frame')
    parser.add_argument('--imu-frame')
    parser.add_argument('--backend-input', type=Path,
                        help='also record the backend input (as --editable runs keep it)')
    parser.add_argument('--generate-lanelet2', default='true', choices=('true', 'false'))
    parser.add_argument('--lane-width', default='3.5')
    parser.add_argument('--origin-lat', default='0.0')
    parser.add_argument('--origin-lon', default='0.0')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    available, detail = core_engine_available()
    if not available:
        print(f'error: the core engine is not available: {detail}', file=sys.stderr)
        return 70
    from lidarslam import backend, mapping

    out = args.output_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    odometry = odometry_parameters(
        args.rko_param.expanduser().resolve(), args.lidar_topic, args.imu_topic,
        args.base_frame, args.lidar_frame, args.imu_frame)
    graph = backend.load_parameters(args.lidarslam_param.expanduser().resolve())
    (out / 'core_params.json').write_text(
        json.dumps({'odometry': odometry, 'backend': graph}, indent=2, sort_keys=True,
                   default=str) + '\n', encoding='utf-8')

    print(f'core engine ({detail}): RKO-LIO + graph_based_slam on {args.bag}', flush=True)
    started = time.monotonic()

    def log(level: str, message: str) -> None:
        print(f'[{level}] [graph_based_slam]: {message}', flush=True)

    summary = mapping.run(
        args.bag, odometry, graph, out, log=log,
        backend_input=args.backend_input.expanduser().resolve() if args.backend_input else None)
    print(f'core engine finished in {time.monotonic() - started:.1f} s: '
          f'{summary["frames"]} frames, {summary["lidar_messages"]} LiDAR and '
          f'{summary["imu_messages"]} IMU messages', flush=True)

    raw_tum, corrected_tum = out / 'traj_raw.tum', out / 'traj_corrected.tum'
    shutil.copyfile(out / 'odometry_tum.txt', raw_tum)
    shutil.copyfile(out / 'trajectory_optimized.tum', corrected_tum)
    print(f'Raw trajectory: {raw_tum} ({len(raw_tum.read_text().splitlines())} poses)')
    print(f'Corrected trajectory: {corrected_tum} '
          f'({len(corrected_tum.read_text().splitlines())} poses)')

    lanelet = out / 'lanelet2_map.osm'
    if args.generate_lanelet2 == 'true':
        # As the ROS 2 path: generated into a temporary file and moved in place on success.
        lanelet.unlink(missing_ok=True)
        tmp = out / 'lanelet2_map.osm.tmp'
        result = subprocess.run(
            [sys.executable, str(SCRIPT_DIR / 'simple_lanelet2_generator.py'),
             '--input', str(corrected_tum), '--output', str(tmp),
             '--lane-width', args.lane_width, '--origin-lat', args.origin_lat,
             '--origin-lon', args.origin_lon], check=False)
        if result.returncode == 0:
            tmp.replace(lanelet)
            print(f'Lanelet2 map written: {lanelet}')
        else:
            tmp.unlink(missing_ok=True)
            print('Warning: Lanelet2 map generation failed; continuing without '
                  'lanelet2_map.osm.', file=sys.stderr)

    print(f'Autoware map bundle under {out}:')
    metadata = out / 'pointcloud_map' / 'pointcloud_map_metadata.yaml'
    for name, path in (('pointcloud_map/', metadata),
                       ('map_projector_info.yaml', out / 'map_projector_info.yaml'),
                       ('lanelet2_map.osm', lanelet)):
        print(f'  {name:<24} {"OK" if path.is_file() else "MISSING"}')
    return 0 if metadata.is_file() and (out / 'map_projector_info.yaml').is_file() else 1


if __name__ == '__main__':
    sys.exit(main())
