"""python -m lidarslam: sensor logs and odometry without ROS 2.

  inspect <log>                         list topics, message types and counts
  odometry <log> --params P --out DIR   RKO-LIO odometry with a ROS 2 parameter file
  backend <log> --params P --out DIR    graph_based_slam offline backend on a backend input
  map <log> --odometry-params P --backend-params P --out DIR
                                        odometry and backend in one pass over a sensor log
"""

from __future__ import annotations

import argparse
import json
import sys

from lidarslam.io import SUPPORTED_TYPES, SensorLog


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog='lidarslam')
    sub = parser.add_subparsers(dest='command', required=True)
    inspect = sub.add_parser('inspect', help='list topics, message types and counts of a log')
    inspect.add_argument('log', help='rosbag2 directory or ROS 1 .bag file')
    odometry = sub.add_parser('odometry', help='RKO-LIO odometry on a log (no ROS 2)')
    odometry.add_argument('log', help='rosbag2 directory or ROS 1 .bag file')
    odometry.add_argument('--params', required=True,
                          help='RKO-LIO parameter file (ROS 2 ros__parameters layout or flat YAML)')
    odometry.add_argument('--out', required=True, help='output directory')
    odometry.add_argument('--lidar-topic', help='overrides lidar_topic')
    odometry.add_argument('--imu-topic', help='overrides imu_topic')
    odometry.add_argument('--run-name', default='odometry', help='output file stem')
    backend = sub.add_parser(
        'backend', help='graph_based_slam offline backend on a recorded backend input (no ROS 2)')
    backend.add_argument('log', help='backend-input log (odometry + deskewed clouds)')
    backend.add_argument('--params', required=True, action='append',
                         help='graph_based_slam parameter file; repeat to layer files')
    backend.add_argument('--out', required=True, help='output directory (output_dir)')
    backend.add_argument('-p', '--param', action='append', default=[], metavar='NAME:=VALUE',
                         help='parameter override, as ROS 2 -p (YAML value)')
    mapping = sub.add_parser(
        'map', help='odometry, then the backend on its deskewed scans, in one pass (no ROS 2)')
    mapping.add_argument('log', help='rosbag2 directory or ROS 1 .bag file (LiDAR + IMU)')
    mapping.add_argument('--odometry-params', required=True,
                         help='RKO-LIO parameter file (ROS 2 ros__parameters layout or flat YAML)')
    mapping.add_argument('--backend-params', required=True, action='append',
                         help='graph_based_slam parameter file; repeat to layer files')
    mapping.add_argument('--out', required=True, help='output directory')
    mapping.add_argument('--lidar-topic', help='overrides lidar_topic')
    mapping.add_argument('--imu-topic', help='overrides imu_topic')
    mapping.add_argument('-p', '--param', action='append', default=[], metavar='NAME:=VALUE',
                         help='backend parameter override, as ROS 2 -p (YAML value)')
    mapping.add_argument('--backend-input', metavar='DIR',
                         help='also record the backend input (rosbag2, MCAP) for later replays')
    args = parser.parse_args(argv)

    if args.command == 'inspect':
        with SensorLog(args.log) as log:
            for topic in log.topics():
                mark = '*' if topic.msgtype in SUPPORTED_TYPES else ' '
                print(f'{mark} {topic.name}  {topic.msgtype}  {topic.count}')
        print('* = read by the mapping pipeline')
    elif args.command == 'odometry':
        from lidarslam.odometry import load_parameters, run
        params = load_parameters(args.params)
        if args.lidar_topic:
            params['lidar_topic'] = args.lidar_topic
        if args.imu_topic:
            params['imu_topic'] = args.imu_topic
        summary = run(args.log, params, args.out, run_name=args.run_name)
        print(json.dumps(summary, indent=2))
    elif args.command == 'backend':
        from lidarslam import backend as backend_module
        from lidarslam.params import apply_overrides
        params = apply_overrides(backend_module.load_parameters(*args.params), args.param)
        summary = backend_module.run(args.log, params, args.out)
        print(json.dumps(summary, indent=2))
    elif args.command == 'map':
        from lidarslam import backend as backend_module, mapping as mapping_module
        from lidarslam.odometry import load_parameters as load_odometry_parameters
        from lidarslam.params import apply_overrides
        odometry_params = load_odometry_parameters(args.odometry_params)
        if args.lidar_topic:
            odometry_params['lidar_topic'] = args.lidar_topic
        if args.imu_topic:
            odometry_params['imu_topic'] = args.imu_topic
        backend_params = apply_overrides(
            backend_module.load_parameters(*args.backend_params), args.param)
        summary = mapping_module.run(args.log, odometry_params, backend_params, args.out,
                                     backend_input=args.backend_input)
        print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
