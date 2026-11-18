"""RKO-LIO odometry on a sensor log, without ROS 2.

Takes the same parameter file as the ROS 2 node (``ros__parameters`` layout or a flat mapping),
reads the log with :mod:`lidarslam.io` in log-time order, and runs the shared offline pipeline
(``rko_lio::driver::OfflineDriver``), which registers frames in the order the ROS 2 offline node
does. Writes the trajectory file of the node's ``dump_results`` and a full-precision CSV.
"""

from __future__ import annotations

import csv
from pathlib import Path
import sys
from typing import Any, Callable

from lidarslam import _core
from lidarslam import velodyne
from lidarslam.io import IMU, POINT_CLOUD2, SensorLog
from lidarslam.params import load_parameters as _load_parameters

# Parameters of the ROS 2 offline node itself (bag reading, ROS plumbing); not driver parameters.
OFFLINE_NODE_ONLY = {
    'bag_path', 'gnss_topic', 'output_subscriber_timeout_sec', 'publisher_queue_depth',
    'async.max_lidar_buffer_size', 'async.output_publish_delay_ms', 'use_sim_time',
}


def load_parameters(path: str | Path) -> dict[str, Any]:
    """Parameters from a ROS 2 parameter file (any node name) or a flat YAML mapping."""
    return _load_parameters(path)


def make_driver(params: dict[str, Any], velodyne_packets: bool = False) -> Any:
    """The RKO-LIO driver for the node's parameters (``params`` is changed: node-only names go).

    ``velodyne_packets``: the LiDAR topic is decoded from VelodyneScan packets.
    """
    buffer_size = int(params.get('async.max_lidar_buffer_size', 10))
    for name in OFFLINE_NODE_ONLY:
        params.pop(name, None)
    # The ROS 2 node's dump/output switches are not used here; callers write their own files.
    for name in ('dump_results', 'results_dir', 'run_name'):
        params.pop(name, None)
    if velodyne_packets and not {'lidar_timestamps.force_absolute',
                                 'lidar_timestamps.force_relative'} & set(params):
        # The decoder's per-point time counts from the scan stamp, which RKO-LIO's automatic
        # check does not take when the first point is more than 1 ms after it.
        params['lidar_timestamps.force_relative'] = True
    driver = _core.OdometryDriver(params, buffer_size)
    for name in driver.unused_parameters:
        print(f'[lidarslam] parameter not used by the odometry: {name}', file=sys.stderr)
    return driver


def drive(log_path: str | Path, params: dict[str, Any],
          on_output: Callable[[dict[str, Any], dict[str, Any]], None],
          passthrough: dict[str, Callable[[Any], None]] | None = None) -> dict[str, Any]:
    """Runs the odometry over a log and hands each output to ``on_output(output, state)`` in order.

    ``state`` (also returned) holds the driver and counters: ``{'driver', 'imu_messages',
    'lidar_messages', 'lidar_frame_id'}`` (``lidar_frame_id``: the header frame of the first LiDAR
    message). ``passthrough`` maps other topics of the log to a callable that gets each of their
    messages as it is read, as the ROS 2 offline node republishes ``gnss_topic`` (topics the log
    lacks are skipped). A ``velodyne_msgs/msg/VelodyneScan`` lidar_topic is decoded as
    velodyne_transform_node does, with the ``velodyne.*`` parameters (:mod:`lidarslam.velodyne`).
    """
    velodyne_settings, params = velodyne.split_parameters(params)
    skip_to_time = float(params.pop('skip_to_time', 0.0) or 0.0)
    with SensorLog(log_path) as probe:
        types = {t.name: t.msgtype for t in probe.topics()}
    driver = make_driver(params, types.get(params.get('lidar_topic')) == velodyne.VELODYNE_SCAN)

    result: dict[str, Any] = {'driver': driver, 'imu_messages': 0, 'lidar_messages': 0,
                              'lidar_frame_id': None}
    decoder = None
    if types.get(driver.lidar_topic) == velodyne.VELODYNE_SCAN:
        decoder = velodyne.Decoder.from_settings(velodyne_settings)
        result['velodyne_calibration'] = str(decoder.calibration_path)
    with SensorLog(log_path, velodyne=decoder) as log:
        for topic, expected in ((driver.lidar_topic, POINT_CLOUD2), (driver.imu_topic, IMU)):
            if types.get(topic) != expected and not (
                    topic == driver.lidar_topic and decoder is not None):
                raise KeyError(f'{topic}: not a {expected} topic of {log_path}'
                               + (f' (or {velodyne.VELODYNE_SCAN})'
                                  if topic == driver.lidar_topic else ''))
        passthrough = {topic: handler for topic, handler in (passthrough or {}).items()
                       if topic in types and topic not in (driver.lidar_topic, driver.imu_topic)}
        start_ns = None
        for record in log.records([driver.lidar_topic, driver.imu_topic, *passthrough]):
            if start_ns is None:
                start_ns = record.log_time_ns
            if record.log_time_ns < start_ns + int(skip_to_time * 1e9):
                continue
            if record.topic in passthrough:
                passthrough[record.topic](record)
                continue
            if record.topic == driver.imu_topic:
                driver.add_imu(record.message)
                result['imu_messages'] += 1
            else:
                if result['lidar_frame_id'] is None:
                    result['lidar_frame_id'] = record.message.header.frame_id
                driver.add_lidar(record.message)
                result['lidar_messages'] += 1
            for output in driver.take_outputs():
                on_output(output, result)
    return result


def run(log_path: str | Path, params: dict[str, Any], out_dir: str | Path,
        run_name: str = 'odometry') -> dict[str, Any]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rows: list[list[Any]] = []
    result = drive(log_path, params,
                   lambda output, _: rows.append([output['time_ns'], *output['pose']]))
    driver = result['driver']
    tum = out / f'{run_name}_tum.txt'
    driver.write_tum(str(tum))
    with (out / f'{run_name}.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['time_ns', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for row in rows:
            writer.writerow([row[0], *(repr(float(v)) for v in row[1:])])
    return {
        'tum': str(tum),
        'poses': len(rows),
        'imu_messages': result['imu_messages'],
        'lidar_messages': result['lidar_messages'],
        'dropped_trailing_frames': driver.pending_lidar_frames,
    }
