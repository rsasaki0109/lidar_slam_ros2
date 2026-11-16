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
from typing import Any

from lidarslam import _core
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


def run(log_path: str | Path, params: dict[str, Any], out_dir: str | Path,
        run_name: str = 'odometry') -> dict[str, Any]:
    params = dict(params)
    skip_to_time = float(params.pop('skip_to_time', 0.0) or 0.0)
    buffer_size = int(params.get('async.max_lidar_buffer_size', 10))
    for name in OFFLINE_NODE_ONLY:
        params.pop(name, None)
    # The ROS 2 node's dump/output switches are not used here; this function writes its own files.
    for name in ('dump_results', 'results_dir', 'run_name'):
        params.pop(name, None)

    driver = _core.OdometryDriver(params, buffer_size)
    for name in driver.unused_parameters:
        print(f'[lidarslam] parameter not used by the odometry: {name}', file=sys.stderr)

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rows: list[list[Any]] = []
    counts = {'imu': 0, 'lidar': 0}
    with SensorLog(log_path) as log:
        types = {t.name: t.msgtype for t in log.topics()}
        for topic, expected in ((driver.lidar_topic, POINT_CLOUD2), (driver.imu_topic, IMU)):
            if types.get(topic) != expected:
                raise KeyError(f'{topic}: not a {expected} topic of {log_path}')
        start_ns = None
        for record in log.records([driver.lidar_topic, driver.imu_topic]):
            if start_ns is None:
                start_ns = record.log_time_ns
            if record.log_time_ns < start_ns + int(skip_to_time * 1e9):
                continue
            if record.topic == driver.imu_topic:
                driver.add_imu(record.message)
                counts['imu'] += 1
            else:
                driver.add_lidar(record.message)
                counts['lidar'] += 1
            for output in driver.take_outputs():
                rows.append([output['time_ns'], *output['pose']])

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
        'imu_messages': counts['imu'],
        'lidar_messages': counts['lidar'],
        'dropped_trailing_frames': driver.pending_lidar_frames,
    }
