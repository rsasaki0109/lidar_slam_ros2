"""Read sensor logs without ROS 2.

Bags are read with `rosbags` (pure Python): rosbag2 directories (MCAP or SQLite3) and ROS 1
``.bag`` files. Messages of the three types the mapping pipeline uses are converted into the
core's plain types, which mirror the ROS 2 messages field for field:

* ``sensor_msgs/msg/PointCloud2`` -> :class:`lidarslam._core.PackedCloud`
* ``sensor_msgs/msg/Imu`` -> :class:`lidarslam._core.ImuSample`
* ``sensor_msgs/msg/NavSatFix`` -> :class:`lidarslam._core.NavSatFix`
* ``nav_msgs/msg/Odometry`` -> :class:`lidarslam._core.Odometry`
* ``velodyne_msgs/msg/VelodyneScan`` -> :class:`lidarslam._core.PackedCloud`, decoded by the
  :class:`lidarslam.velodyne.Decoder` given to :class:`SensorLog` (as velodyne_transform_node)
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from lidarslam import _core
from lidarslam.velodyne import VELODYNE_SCAN, register_types as _register_velodyne_types

POINT_CLOUD2 = 'sensor_msgs/msg/PointCloud2'
IMU = 'sensor_msgs/msg/Imu'
NAV_SAT_FIX = 'sensor_msgs/msg/NavSatFix'
ODOMETRY = 'nav_msgs/msg/Odometry'
SUPPORTED_TYPES = (POINT_CLOUD2, IMU, NAV_SAT_FIX, ODOMETRY, VELODYNE_SCAN)


@dataclass(frozen=True)
class TopicInfo:
    name: str
    msgtype: str
    count: int


@dataclass(frozen=True)
class Record:
    topic: str
    log_time_ns: int
    message: Any  # PackedCloud (also for a decoded VelodyneScan), ImuSample, NavSatFix or Odometry


def _header(src: Any) -> _core.Header:
    header = _core.Header()
    header.stamp.sec = int(src.stamp.sec)
    header.stamp.nanosec = int(src.stamp.nanosec)
    header.frame_id = str(src.frame_id)
    return header


def to_packed_cloud(msg: Any) -> _core.PackedCloud:
    cloud = _core.PackedCloud()
    cloud.header = _header(msg.header)
    cloud.height = int(msg.height)
    cloud.width = int(msg.width)
    cloud.fields = [
        _core.PointField(str(f.name), int(f.offset), int(f.datatype), int(f.count))
        for f in msg.fields
    ]
    cloud.is_bigendian = bool(msg.is_bigendian)
    cloud.point_step = int(msg.point_step)
    cloud.row_step = int(msg.row_step)
    cloud.data = memoryview(msg.data).cast('B')
    cloud.is_dense = bool(msg.is_dense)
    return cloud


def to_imu_sample(msg: Any) -> _core.ImuSample:
    imu = _core.ImuSample()
    imu.header = _header(msg.header)
    q = msg.orientation
    imu.orientation = [float(q.x), float(q.y), float(q.z), float(q.w)]
    imu.orientation_covariance = [float(v) for v in msg.orientation_covariance]
    w = msg.angular_velocity
    imu.angular_velocity = [float(w.x), float(w.y), float(w.z)]
    imu.angular_velocity_covariance = [float(v) for v in msg.angular_velocity_covariance]
    a = msg.linear_acceleration
    imu.linear_acceleration = [float(a.x), float(a.y), float(a.z)]
    imu.linear_acceleration_covariance = [float(v) for v in msg.linear_acceleration_covariance]
    return imu


def to_nav_sat_fix(msg: Any) -> _core.NavSatFix:
    fix = _core.NavSatFix()
    fix.header = _header(msg.header)
    fix.status = int(msg.status.status)
    fix.service = int(msg.status.service)
    fix.latitude = float(msg.latitude)
    fix.longitude = float(msg.longitude)
    fix.altitude = float(msg.altitude)
    fix.position_covariance = [float(v) for v in msg.position_covariance]
    fix.position_covariance_type = int(msg.position_covariance_type)
    return fix


def to_odometry(msg: Any) -> _core.Odometry:
    odom = _core.Odometry()
    odom.header = _header(msg.header)
    odom.child_frame_id = str(msg.child_frame_id)
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    odom.position = [float(p.x), float(p.y), float(p.z)]
    odom.orientation = [float(q.x), float(q.y), float(q.z), float(q.w)]
    odom.pose_covariance = [float(v) for v in msg.pose.covariance]
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    odom.linear_velocity = [float(v.x), float(v.y), float(v.z)]
    odom.angular_velocity = [float(w.x), float(w.y), float(w.z)]
    odom.twist_covariance = [float(c) for c in msg.twist.covariance]
    return odom


_CONVERTERS = {
    POINT_CLOUD2: to_packed_cloud,
    IMU: to_imu_sample,
    NAV_SAT_FIX: to_nav_sat_fix,
    ODOMETRY: to_odometry,
}


class SensorLog:
    """A recorded sensor log (rosbag2 directory or ROS 1 bag), opened without ROS 2."""

    def __init__(self, path: str | Path, velodyne: Any = None):
        """``velodyne``: a :class:`lidarslam.velodyne.Decoder` for VelodyneScan topics."""
        self.path = Path(path)
        if not self.path.exists():
            raise FileNotFoundError(self.path)
        typestore = get_typestore(Stores.LATEST)
        _register_velodyne_types(typestore)
        self._reader = AnyReader([self.path], default_typestore=typestore)
        self._converters = dict(_CONVERTERS)
        if velodyne is not None:
            self._converters[VELODYNE_SCAN] = velodyne.decode

    def __enter__(self) -> 'SensorLog':
        self._reader.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self._reader.close()

    def topics(self) -> list[TopicInfo]:
        return sorted(
            (TopicInfo(name, info.msgtype or '', int(info.msgcount))
             for name, info in self._reader.topics.items()),
            key=lambda t: t.name)

    def records(self, topics: Iterable[str] | None = None) -> Iterator[Record]:
        """Messages of the supported types in log-time order, converted to core types."""
        wanted = set(topics) if topics is not None else None
        connections = [
            c for c in self._reader.connections
            if c.msgtype in self._converters and (wanted is None or c.topic in wanted)
        ]
        if wanted is not None:
            missing = wanted - {c.topic for c in connections}
            if missing:
                raise KeyError(
                    f'not in the log or not a supported type ({", ".join(SUPPORTED_TYPES)}; '
                    f'{VELODYNE_SCAN} with a Velodyne decoder): {sorted(missing)}')
        for connection, log_time_ns, raw in self._reader.messages(connections=connections):
            msg = self._reader.deserialize(raw, connection.msgtype)
            yield Record(connection.topic, int(log_time_ns),
                         self._converters[connection.msgtype](msg))
