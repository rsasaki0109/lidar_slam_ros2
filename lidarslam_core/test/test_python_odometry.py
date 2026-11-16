"""lidarslam odometry without ROS 2 on a synthetic log (CI smoke test).

A static sensor inside a box room: 3 s of 200 Hz IMU (gravity only) and 10 Hz LiDAR scans with
per-point times. The odometry must run end to end and stay near the origin. Byte identity with
the ROS 2 node is checked on real logs (docs/roadmap/ros-free-core-2026-11.md), not here.
"""

from __future__ import annotations

from pathlib import Path

import pytest

lidarslam = pytest.importorskip('lidarslam')
import numpy as np  # noqa: E402
core = pytest.importorskip('lidarslam._core')
if not hasattr(core, 'OdometryDriver'):
    pytest.skip('lidarslam built without the frontend', allow_module_level=True)

from lidarslam.odometry import load_parameters, run  # noqa: E402
from rosbags.rosbag2 import StoragePlugin, Writer  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

TS = get_typestore(Stores.ROS2_JAZZY)
M = TS.types


def _room_points(rng: np.random.Generator, n: int = 6000) -> np.ndarray:
    """Points on the walls, floor and ceiling of an 8 x 6 x 3 m room, plus a few boxes."""
    half = np.array([4.0, 3.0, 1.5])
    pts = rng.uniform(-half, half, size=(n, 3))
    axis = rng.integers(0, 3, size=n)
    sign = rng.choice([-1.0, 1.0], size=n)
    pts[np.arange(n), axis] = sign * half[axis]
    boxes = rng.uniform([-3, -2, -1.5], [3, 2, 0.0], size=(800, 3))
    boxes[:, 0] = np.round(boxes[:, 0])  # vertical planes at integer x
    return np.vstack([pts, boxes]).astype(np.float32)


def _write_log(path: Path) -> None:
    rng = np.random.default_rng(7)
    room = _room_points(rng)
    header = M['std_msgs/msg/Header']
    time = M['builtin_interfaces/msg/Time']
    pf = M['sensor_msgs/msg/PointField']
    fields = [pf(name='x', offset=0, datatype=7, count=1), pf(name='y', offset=4, datatype=7, count=1),
              pf(name='z', offset=8, datatype=7, count=1), pf(name='t', offset=12, datatype=7, count=1)]
    v3, q = M['geometry_msgs/msg/Vector3'], M['geometry_msgs/msg/Quaternion']
    start = 1_700_000_000 * 10**9
    with Writer(path, version=8, storage_plugin=StoragePlugin.MCAP) as writer:
        lidar = writer.add_connection('/points', 'sensor_msgs/msg/PointCloud2', typestore=TS)
        imu = writer.add_connection('/imu', 'sensor_msgs/msg/Imu', typestore=TS)
        events = [(start + k * 5_000_000, 'imu') for k in range(600)]
        events += [(start + 100_000_000 + k * 100_000_000, 'lidar') for k in range(28)]
        for stamp, kind in sorted(events):
            sec, nsec = divmod(stamp, 10**9)
            hdr = header(stamp=time(sec=sec, nanosec=nsec), frame_id=kind)
            if kind == 'imu':
                msg = M['sensor_msgs/msg/Imu'](
                    header=hdr, orientation=q(x=0.0, y=0.0, z=0.0, w=1.0),
                    orientation_covariance=np.zeros(9),
                    angular_velocity=v3(x=0.0, y=0.0, z=0.0), angular_velocity_covariance=np.zeros(9),
                    linear_acceleration=v3(x=0.0, y=0.0, z=9.81),
                    linear_acceleration_covariance=np.zeros(9))
                writer.write(imu, stamp, TS.serialize_cdr(msg, 'sensor_msgs/msg/Imu'))
            else:
                pts = room + rng.normal(0.0, 0.005, size=room.shape).astype(np.float32)
                rec = np.zeros(len(pts), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('t', '<f4')])
                rec['x'], rec['y'], rec['z'] = pts[:, 0], pts[:, 1], pts[:, 2]
                rec['t'] = np.linspace(-0.1, 0.0, len(pts), dtype=np.float32)  # relative, scan ends at stamp
                msg = M['sensor_msgs/msg/PointCloud2'](
                    header=hdr, height=1, width=len(pts), fields=fields, is_bigendian=False,
                    point_step=16, row_step=16 * len(pts), data=rec.view(np.uint8).copy(), is_dense=True)
                writer.write(lidar, stamp, TS.serialize_cdr(msg, 'sensor_msgs/msg/PointCloud2'))


def test_static_room(tmp_path: Path) -> None:
    log = tmp_path / 'log'
    _write_log(log)
    params_file = tmp_path / 'params.yaml'
    params_file.write_text(
        '/**:\n'
        '  ros__parameters:\n'
        '    imu_topic: /imu\n'
        '    lidar_topic: /points\n'
        '    base_frame: base_link\n'
        '    extrinsic_imu2base_quat_xyzw_xyz: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]\n'
        '    extrinsic_lidar2base_quat_xyzw_xyz: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]\n'
        '    voxel_size: 0.25\n'
        '    lidar_timestamps:\n'
        '      force_relative: true\n',
        encoding='utf-8')
    params = load_parameters(params_file)
    assert params['lidar_timestamps.force_relative'] is True
    summary = run(log, params, tmp_path / 'out')
    assert summary['lidar_messages'] == 28 and summary['imu_messages'] == 600
    poses = np.loadtxt(summary['tum'])
    assert len(poses) >= 20
    assert np.abs(poses[:, 1:4]).max() < 0.05  # a static sensor stays put


def test_bad_parameter_type_is_reported(tmp_path: Path) -> None:
    with pytest.raises(TypeError, match='voxel_size'):
        core.OdometryDriver({'imu_topic': '/imu', 'lidar_topic': '/points', 'base_frame': 'b',
                             'voxel_size': 'large'}, 10)
    with pytest.raises(KeyError, match='lidar_topic'):
        core.OdometryDriver({'imu_topic': '/imu', 'base_frame': 'b'}, 10)
