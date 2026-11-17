"""GNSS anchors in the package's backend (use_gnss), on a synthetic backend input (CI smoke test).

The walk of test_python_backend with RTK fixes of the sensor's position, in an ENU frame turned
40 deg from the odometry frame. With use_gnss the final optimization takes the fixes as anchors,
the yaw alignment moves the graph into the ENU frame and the map bundle gets the GNSS origin; the
backend input the core engine records keeps the fixes. Agreement with the ROS 2 offline runner,
and accuracy on real RTK data, are checked in bench_report/data/ros_free_gnss_2026-11-17.
"""

from __future__ import annotations

import math
from pathlib import Path
import re

import pytest

lidarslam = pytest.importorskip('lidarslam')
core = pytest.importorskip('lidarslam._core')
if not hasattr(core, 'BackendPipeline') or not hasattr(core.BackendPipeline, 'add_gnss_fix'):
    pytest.skip('lidarslam built without GNSS anchors', allow_module_level=True)

import numpy as np  # noqa: E402
from lidarslam import backend  # noqa: E402
from lidarslam.io import SensorLog  # noqa: E402
from lidarslam.mapping import BackendInputWriter  # noqa: E402
from rosbags.rosbag2 import StoragePlugin, Writer  # noqa: E402
from test_python_backend import _path, _write_input, M, TS  # noqa: E402

YAW = math.radians(40.0)  # ENU from odometry
LAT0, LON0, ALT0 = 48.78, 9.17, 300.0
PARAMS = {'registration_method': 'SMALL_GICP', 'gicp_max_correspondence_distance': 0.5,
          'ndt_num_threads': 1, 'voxel_leaf_size': 0.2, 'submap_distance_threshold': 1.5}


def _lat_lon(east: float, north: float) -> tuple[float, float]:
    return (LAT0 + north / 111132.0,
            LON0 + east / (111320.0 * math.cos(math.radians(LAT0))))


def _write_gnss_input(path: Path, frames: int = 160) -> None:
    """The backend input of test_python_backend plus a 5 Hz RTK fix of the sensor's position, at
    every other scan's stamp."""
    _write_input(path, frames)
    header, time = M['std_msgs/msg/Header'], M['builtin_interfaces/msg/Time']
    start = 1_700_000_000 * 10**9
    fixes = path.parent / 'fixes'
    with Writer(fixes, version=8, storage_plugin=StoragePlugin.MCAP) as writer:
        conn = writer.add_connection('/gnss/fix', 'sensor_msgs/msg/NavSatFix', typestore=TS)
        for k in range(0, frames, 2):
            stamp = start + k * 100_000_000  # at the scan's stamp
            pos, _ = _path(k, frames)
            c, s = math.cos(YAW), math.sin(YAW)
            east, north = c * pos[0] - s * pos[1], s * pos[0] + c * pos[1]
            lat, lon = _lat_lon(east, north)
            sec, nsec = divmod(stamp, 10**9)
            fix = M['sensor_msgs/msg/NavSatFix'](
                header=header(stamp=time(sec=sec, nanosec=nsec), frame_id='gnss'),
                status=M['sensor_msgs/msg/NavSatStatus'](status=2, service=1),
                latitude=lat, longitude=lon, altitude=ALT0,
                position_covariance=np.array([1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 4e-4]),
                position_covariance_type=2)
            writer.write(conn, stamp, TS.serialize_cdr(fix, 'sensor_msgs/msg/NavSatFix'))
    # One log: the backend input's messages and the fixes, in log-time order.
    merged = path.parent / 'merged'
    out = BackendInputWriter(merged)
    try:
        with SensorLog(path) as a, SensorLog(fixes) as b:
            records = sorted([*a.records(), *b.records()], key=lambda r: r.log_time_ns)
        pairs: dict[int, dict[str, object]] = {}
        for record in records:
            if record.topic == '/gnss/fix':
                out.write_fix('/gnss/fix', record.message, record.log_time_ns)
                continue
            pair = pairs.setdefault(record.log_time_ns, {})
            pair[record.topic] = record.message
            if len(pair) == 2:
                out.write(pair['/rko_lio/frame'], pair['/rko_lio/odometry'], record.log_time_ns)
                del pairs[record.log_time_ns]
    finally:
        out.close()
    path.rename(path.parent / 'without_gnss')
    merged.rename(path)


def test_gnss_anchors_move_the_graph_into_the_enu_frame(tmp_path: Path) -> None:
    log = tmp_path / 'backend_input'
    _write_gnss_input(log)
    with SensorLog(log) as sensor_log:
        counts = {t.name: t.count for t in sensor_log.topics()}
    assert counts == {'/gnss/fix': 80, '/rko_lio/frame': 160, '/rko_lio/odometry': 160}

    messages: list[tuple[str, str]] = []
    summary = backend.run(
        log, dict(PARAMS, use_gnss=True, gnss_yaw_alignment_min_anchors=5,
                  offline_map_bundle=True),
        tmp_path / 'gnss', log=lambda level, msg: messages.append((level, msg)))
    assert summary['gnss_fixes'] == 80
    text = '\n'.join(msg for _, msg in messages)
    assert 'GNSS origin set from 3 consistent fixes' in text
    yaw = re.search(r'GNSS yaw alignment applied: yaw=(-?[0-9.]+) deg', text)
    # _lat_lon is a spherical approximation of the backend's WGS84 ENU: 0.3 deg, a few cm
    assert yaw is not None and abs(float(yaw.group(1)) - 40.0) < 0.5
    assert not [msg for level, msg in messages if level == 'ERROR']

    raw = np.loadtxt(tmp_path / 'gnss' / 'trajectory_raw.tum', ndmin=2)
    optimized = np.loadtxt(tmp_path / 'gnss' / 'trajectory_optimized.tum', ndmin=2)
    c, s = math.cos(YAW), math.sin(YAW)
    expected = np.column_stack([c * raw[:, 1] - s * raw[:, 2], s * raw[:, 1] + c * raw[:, 2]])
    # the ENU origin is the mean of the first three fixes (k = 0, 2, 4)
    origin_odom = np.mean([_path(k, 160)[0] for k in (0, 2, 4)], axis=0)
    offset = np.array([c * origin_odom[0] - s * origin_odom[1],
                       s * origin_odom[0] + c * origin_odom[1]])
    # The fixes that set the origin are not anchors (as in the live node), so the first submaps
    # take the nearest later fix (0.4 s, 1 m along the walk) and pull the stiff odometry chain by
    # up to 0.4 m; the rest of the walk sits on its fixes.
    error = np.linalg.norm(optimized[:, 1:3] - (expected - offset), axis=1)
    assert error.max() < 0.5
    assert np.median(error) < 0.1
    projector = (tmp_path / 'gnss' / 'map_projector_info.yaml').read_text()
    assert 'projector_type: LocalCartesian' in projector

    # Without use_gnss the fixes are ignored: the run of the input without them.
    backend.run(log, dict(PARAMS), tmp_path / 'off', log=lambda level, msg: None)
    backend.run(tmp_path / 'without_gnss', dict(PARAMS), tmp_path / 'plain',
                log=lambda level, msg: None)
    for name in ('trajectory_optimized.tum', 'loop_edges.csv'):
        assert (tmp_path / 'off' / name).read_bytes() == (tmp_path / 'plain' / name).read_bytes()


def test_backend_input_writer_adds_the_fix_topic_only_with_fixes(tmp_path: Path) -> None:
    BackendInputWriter(tmp_path / 'none').close()
    with SensorLog(tmp_path / 'none') as sensor_log:
        assert '/gnss/fix' not in {t.name for t in sensor_log.topics()}
