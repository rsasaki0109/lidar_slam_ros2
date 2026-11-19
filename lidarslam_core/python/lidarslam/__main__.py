"""python -m lidarslam: sensor logs and odometry without ROS 2.

  inspect <log>                         list topics, message types and counts
  odometry <log> --params P --out DIR   RKO-LIO odometry with a ROS 2 parameter file
  backend <log> --params P --out DIR    graph_based_slam offline backend on a backend input
  map <log> --odometry-params P --backend-params P --out DIR
                                        odometry and backend in one pass over a sensor log
  join --params P -p session_bag_paths:=[...] -p session_backend_dirs:=[...] -p output_dir:=DIR
                                        multi-session join (graph_slam_multisession_join)
  live <log> --odometry-params P --backend-params P --out DIR [--realtime] [--speed S]
       [--store STORE]                  live mapping: the log played message by message, live
                                        (causal) poses as they come (lidarslam.live); with
                                        --store the session is added to a long-term map store
  store info|export|rollback|candidates|edit|apply-edit|rollback-edit STORE [...]
                                        a long-term map store (lidarslam.store): list its
                                        sessions, export its Autoware bundle, take the latest
                                        session off the map
"""

from __future__ import annotations

import argparse
import json
import sys

from lidarslam.io import SUPPORTED_TYPES, SensorLog


def _store_review(map_store, args) -> int:
    """store candidates | edit | apply-edit | rollback-edit (lidarslam.map_edits)."""
    from lidarslam import map_edits
    if args.action == 'candidates':
        items = map_edits.candidates(map_store, args.min_voxels)
        print(json.dumps([{k: v for k, v in c.items() if k != 'voxel_keys'} for c in items],
                         indent=2))
    elif args.action == 'rollback-edit':
        print(json.dumps(map_store.rollback_edit(), indent=2))
    else:
        if args.action == 'apply-edit':
            if not args.file:
                raise SystemExit('apply-edit needs --file')
            edit = map_edits.read(args.file)
        else:
            if not (args.author and args.reason):
                raise SystemExit('edit needs --author and --reason')
            boxes = lambda values: [(v[:3], v[3:]) for v in values]  # noqa: E731
            operations = map_edits.resolve(
                map_store, remove=args.remove, keep=args.keep,
                remove_boxes=boxes(args.remove_box), keep_boxes=boxes(args.keep_box),
                min_voxels=args.min_voxels)
            if not operations:
                raise SystemExit('nothing to edit: no candidate or box with stored points')
            edit = map_edits.make_edit(args.author, args.reason, operations)
        print(json.dumps(map_store.add_edit(edit), indent=2))
    return 0


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
    live = sub.add_parser(
        'live', help='live mapping: play a log into a live session, poses as they come (no ROS 2)')
    live.add_argument('log', help='rosbag2 directory or ROS 1 .bag file (LiDAR + IMU)')
    live.add_argument('--odometry-params', required=True,
                      help='RKO-LIO parameter file (ROS 2 ros__parameters layout or flat YAML)')
    live.add_argument('--backend-params', required=True, action='append',
                      help='graph_based_slam parameter file; repeat to layer files')
    live.add_argument('--out', required=True, help='output directory')
    live.add_argument('-p', '--param', action='append', default=[], metavar='NAME:=VALUE',
                      help='backend parameter override, as ROS 2 -p (YAML value)')
    live.add_argument('--realtime', action='store_true',
                      help='hand each message over at its log time (sensor rate)')
    live.add_argument('--speed', type=float, default=1.0,
                      help='with --realtime: playback speed (2 = twice the sensor rate)')
    live.add_argument('--store', metavar='STORE',
                      help='add the session to this long-term map store (created when missing)')
    startup = live.add_mutually_exclusive_group()
    startup.add_argument('--initial-pose', nargs=7, type=float,
                         metavar=('X', 'Y', 'Z', 'QX', 'QY', 'QZ', 'QW'),
                         help='startup odometry frame in the store frame, as a pose')
    startup.add_argument('--start-near-first-submap', action='store_true',
                         help='search a launch area within 5 m of the first stored submap')
    live.add_argument('--track-store', action='store_true',
                      help='after startup, try experimental loop factors against the stored map')
    live.add_argument('--track-shared-surfaces', action='store_true',
                      help='track against shared surfaces in two or more stored sessions')
    live.add_argument('--detect-changes', action='store_true',
                      help='trace anchored keyframe scans and remove stale voxels from store exports')
    live.add_argument('--filter-new-points', action='store_true',
                      help='detect changes and filter new points without repeated scan support')
    live.add_argument('--refine-visibility', action='store_true',
                      help='use complete ray evidence for new submaps (experimental)')
    store = sub.add_parser(
        'store', help='a long-term map store: info, export, rollback, and reviewed edits')
    store.add_argument('action', choices=['info', 'export', 'rollback', 'candidates', 'edit',
                                          'apply-edit', 'rollback-edit'],
                       help='candidates: what the change evidence and the latest session '
                            'changed, as clusters to review; edit: keep or remove candidates or '
                            'boxes; apply-edit: an edit file (e.g. from the map editor); '
                            'rollback-edit: take the latest edit off the map')
    store.add_argument('store', help='map store directory')
    store.add_argument('--out', help='export: bundle directory (default: STORE/export)')
    store.add_argument('--min-voxels', type=int, default=5,
                       help='candidates / edit: smallest cluster listed (0.15 m voxels)')
    store.add_argument('--author', help="edit: 'human:<name>' or 'agent:<name>'")
    store.add_argument('--reason', help='edit: why (kept with the edit)')
    store.add_argument('--remove', action='append', default=[], metavar='CANDIDATE',
                       help='edit: remove the points of a candidate')
    store.add_argument('--keep', action='append', default=[], metavar='CANDIDATE',
                       help='edit: keep the points of a candidate')
    store.add_argument('--remove-box', action='append', default=[], nargs=6, type=float,
                       metavar=('X0', 'Y0', 'Z0', 'X1', 'Y1', 'Z1'),
                       help='edit: remove the points inside a store-frame box')
    store.add_argument('--keep-box', action='append', default=[], nargs=6, type=float,
                       metavar=('X0', 'Y0', 'Z0', 'X1', 'Y1', 'Z1'),
                       help='edit: keep the points inside a store-frame box')
    store.add_argument('--file', help='apply-edit: the edit file (JSON)')
    join = sub.add_parser(
        'join', help='multi-session join of backend inputs and their offline runs (no ROS 2)')
    join.add_argument('--params', action='append', default=[],
                      help='graph_based_slam parameter file; repeat to layer files')
    join.add_argument('-p', '--param', action='append', default=[], metavar='NAME:=VALUE',
                      help='parameter, as ROS 2 -p (YAML value): session_bag_paths, '
                           'session_backend_dirs, output_dir, ...')
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
    elif args.command == 'live':
        from lidarslam import backend as backend_module, live as live_module
        from lidarslam.odometry import load_parameters as load_odometry_parameters
        from lidarslam.params import apply_overrides
        backend_params = apply_overrides(
            backend_module.load_parameters(*args.backend_params), args.param)
        summary = live_module.replay(args.log, load_odometry_parameters(args.odometry_params),
                                     backend_params, args.out, realtime=args.realtime,
                                     speed=args.speed, store=args.store, initial_pose=args.initial_pose,
                                     track_store=args.track_store,
                                     track_shared_surfaces=args.track_shared_surfaces,
                                     detect_changes=args.detect_changes,
                                     filter_new_points=args.filter_new_points,
                                     start_near_first_submap=args.start_near_first_submap,
                                     refine_visibility=args.refine_visibility)
        print(json.dumps(summary, indent=2))
    elif args.command == 'store':
        from lidarslam.store import Store
        map_store = Store.open(args.store)
        if args.action in ('candidates', 'edit', 'apply-edit', 'rollback-edit'):
            return _store_review(map_store, args)
        if args.action == 'export':
            print(map_store.export(args.out))
        elif args.action == 'rollback':
            print(json.dumps(map_store.rollback(), indent=2))
        else:
            print(json.dumps({'store': str(map_store.path), 'frame': map_store.frame,
                              'sessions': map_store.sessions, 'edits': map_store.edits},
                             indent=2))
    elif args.command == 'join':
        from lidarslam import multisession
        from lidarslam.params import apply_overrides
        params = apply_overrides(multisession.load_parameters(*args.params), args.param)
        return multisession.run(params)
    return 0


if __name__ == '__main__':
    sys.exit(main())
