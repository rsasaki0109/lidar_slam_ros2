"""lidarslam.io on small logs written here (no ROS 2 needed).

Writes a rosbag2 (MCAP and SQLite3) and a ROS 1 bag with rosbags, reads them back with
SensorLog, and checks every converted field.
"""

from __future__ import annotations

from pathlib import Path

import pytest

lidarslam = pytest.importorskip('lidarslam')
import numpy as np  # noqa: E402
from lidarslam.io import SensorLog  # noqa: E402
from rosbags.rosbag1 import Writer as Writer1  # noqa: E402
from rosbags.rosbag2 import StoragePlugin, Writer as Writer2  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

TS2 = get_typestore(Stores.ROS2_JAZZY)
TS1 = get_typestore(Stores.ROS1_NOETIC)
N = 5


def _cloud_data() -> tuple[np.ndarray, np.ndarray]:
    xyz = np.arange(N * 3, dtype=np.float32).reshape(N, 3) * 0.5
    t = np.arange(N, dtype=np.uint32) * 1000
    rec = np.zeros(N, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('t', '<u4')])
    rec['x'], rec['y'], rec['z'], rec['t'] = xyz[:, 0], xyz[:, 1], xyz[:, 2], t
    return xyz, rec.view(np.uint8).copy()


def _messages(ts, ros1: bool):
    m = ts.types
    header_kwargs = {'seq': 0} if ros1 else {}
    stamp = (m['builtin_interfaces/msg/Time'] if not ros1 else m['builtin_interfaces/msg/Time'])(
        sec=12, nanosec=345)
    header = m['std_msgs/msg/Header'](**header_kwargs, stamp=stamp, frame_id='lidar')
    PF = m['sensor_msgs/msg/PointField']
    fields = [PF(name='x', offset=0, datatype=7, count=1), PF(name='y', offset=4, datatype=7, count=1),
              PF(name='z', offset=8, datatype=7, count=1), PF(name='t', offset=12, datatype=6, count=1)]
    _, data = _cloud_data()
    cloud = m['sensor_msgs/msg/PointCloud2'](
        header=header, height=1, width=N, fields=fields, is_bigendian=False, point_step=16,
        row_step=16 * N, data=data, is_dense=True)
    V3, Q = m['geometry_msgs/msg/Vector3'], m['geometry_msgs/msg/Quaternion']
    imu = m['sensor_msgs/msg/Imu'](
        header=header, orientation=Q(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=np.full(9, 0.1),
        angular_velocity=V3(x=0.01, y=-0.02, z=0.03), angular_velocity_covariance=np.zeros(9),
        linear_acceleration=V3(x=0.1, y=0.2, z=9.81), linear_acceleration_covariance=np.zeros(9))
    status = m['sensor_msgs/msg/NavSatStatus'](status=2, service=1)
    fix = m['sensor_msgs/msg/NavSatFix'](
        header=header, status=status, latitude=35.1, longitude=139.2, altitude=40.5,
        position_covariance=np.arange(9, dtype=np.float64), position_covariance_type=2)
    return [('/points', 'sensor_msgs/msg/PointCloud2', cloud),
            ('/imu', 'sensor_msgs/msg/Imu', imu),
            ('/fix', 'sensor_msgs/msg/NavSatFix', fix)]


def _write_rosbag2(path: Path, storage: StoragePlugin) -> None:
    with Writer2(path, version=8, storage_plugin=storage) as writer:
        for k, (topic, msgtype, msg) in enumerate(_messages(TS2, ros1=False)):
            conn = writer.add_connection(topic, msgtype, typestore=TS2)
            writer.write(conn, 1_000 + k, TS2.serialize_cdr(msg, msgtype))


def _write_rosbag1(path: Path) -> None:
    with Writer1(path) as writer:
        for k, (topic, msgtype, msg) in enumerate(_messages(TS1, ros1=True)):
            conn = writer.add_connection(topic, msgtype, typestore=TS1)
            writer.write(conn, 1_000 + k, TS1.serialize_ros1(msg, msgtype))


def _check(log_path: Path) -> None:
    with SensorLog(log_path) as log:
        assert {t.name: (t.msgtype, t.count) for t in log.topics()} == {
            '/points': ('sensor_msgs/msg/PointCloud2', 1),
            '/imu': ('sensor_msgs/msg/Imu', 1),
            '/fix': ('sensor_msgs/msg/NavSatFix', 1),
        }
        records = list(log.records())
    assert [r.topic for r in records] == ['/points', '/imu', '/fix']
    assert [r.log_time_ns for r in records] == [1000, 1001, 1002]

    cloud = records[0].message
    xyz, data = _cloud_data()
    assert (cloud.header.stamp.sec, cloud.header.stamp.nanosec, cloud.header.frame_id) == (
        12, 345, 'lidar')
    assert (cloud.height, cloud.width, cloud.point_step, cloud.row_step) == (1, N, 16, 16 * N)
    assert [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields] == [
        ('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1), ('t', 12, 6, 1)]
    assert cloud.data == data.tobytes()
    assert len(cloud) == N
    np.testing.assert_array_equal(cloud.field('x'), xyz[:, 0].astype(np.float64))
    np.testing.assert_array_equal(cloud.field('t'), np.arange(N) * 1000.0)
    with pytest.raises(KeyError):
        cloud.field('intensity')

    imu = records[1].message
    assert imu.orientation == [0.0, 0.0, 0.0, 1.0]
    assert imu.angular_velocity == [0.01, -0.02, 0.03]
    assert imu.linear_acceleration == [0.1, 0.2, 9.81]
    assert imu.orientation_covariance == [0.1] * 9

    fix = records[2].message
    assert (fix.status, fix.service, fix.position_covariance_type) == (2, 1, 2)
    assert (fix.latitude, fix.longitude, fix.altitude) == (35.1, 139.2, 40.5)
    assert fix.position_covariance == [float(v) for v in range(9)]


@pytest.mark.parametrize('storage', [StoragePlugin.MCAP, StoragePlugin.SQLITE3])
def test_rosbag2(tmp_path: Path, storage: StoragePlugin) -> None:
    path = tmp_path / 'log'
    _write_rosbag2(path, storage)
    _check(path)


def test_rosbag1(tmp_path: Path) -> None:
    path = tmp_path / 'log.bag'
    _write_rosbag1(path)
    _check(path)


def test_topic_filter_rejects_unknown_topics(tmp_path: Path) -> None:
    path = tmp_path / 'log'
    _write_rosbag2(path, StoragePlugin.MCAP)
    with SensorLog(path) as log:
        assert [r.topic for r in log.records(['/imu'])] == ['/imu']
        with pytest.raises(KeyError):
            list(log.records(['/missing']))
