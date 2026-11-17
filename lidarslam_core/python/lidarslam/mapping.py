"""Frontend and backend in one pass over a sensor log, without ROS 2.

RKO-LIO (:mod:`lidarslam.odometry`) registers each LiDAR scan; its deskewed scan and odometry go,
in memory, to the graph_based_slam offline backend (:mod:`lidarslam.backend`) as the messages the
ROS 2 offline node publishes on ``/rko_lio/frame`` and ``/rko_lio/odometry``: the same fields,
frames and stamps, so the backend sees what it would read from a recorded backend input. With
the backend's ``use_gnss``, the log's NavSatFix messages of ``gnss_topic`` go to the backend as
they are read, as the ROS 2 offline node republishes them.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

import numpy as np

from lidarslam import _core
from lidarslam.io import NAV_SAT_FIX, SensorLog
from lidarslam.odometry import drive

_XYZ_FIELDS = [('x', 0), ('y', 4), ('z', 8)]
_FLOAT32 = 7


def _header(time_ns: int, frame_id: str) -> _core.Header:
    header = _core.Header()
    header.stamp.sec, header.stamp.nanosec = divmod(int(time_ns), 1_000_000_000)
    header.frame_id = frame_id
    return header


def deskewed_cloud(output: dict[str, Any], lidar_frame: str) -> _core.PackedCloud:
    """The offline node's /rko_lio/frame: x, y, z as float32 with 4 bytes of padding a point."""
    scan = np.asarray(output['deskewed_scan'], dtype=np.float64).reshape(-1, 3)
    packed = np.zeros((len(scan), 4), dtype='<f4')
    packed[:, :3] = scan  # double -> float rounds to nearest, as static_cast<float>
    cloud = _core.PackedCloud()
    cloud.header = _header(output['time_ns'], lidar_frame)
    cloud.height = 1
    cloud.width = len(scan)
    cloud.fields = [_core.PointField(name, offset, _FLOAT32, 1) for name, offset in _XYZ_FIELDS]
    cloud.is_bigendian = False
    cloud.point_step = 16
    cloud.row_step = 16 * len(scan)
    cloud.data = packed.tobytes()
    cloud.is_dense = False
    return cloud


def odometry_message(output: dict[str, Any], odom_frame: str, base_frame: str) -> _core.Odometry:
    """The offline node's /rko_lio/odometry."""
    odom = _core.Odometry()
    odom.header = _header(output['time_ns'], odom_frame)
    odom.child_frame_id = base_frame
    pose = output['pose']
    odom.position = list(pose[:3])
    odom.orientation = list(pose[3:7])
    odom.pose_covariance = list(output['pose_covariance'])
    odom.linear_velocity = list(output['velocity'])
    odom.angular_velocity = list(output['angular_velocity'])
    return odom


class BackendInputWriter:
    """Records the messages the backend reads as a rosbag2 (MCAP) backend input.

    The same topics, types and stamps as ``record_backend_input.sh`` records from the ROS 2
    offline node (log time = message stamp), so the replay and merge tools read it.
    """

    def __init__(self, path: str | Path):
        from rosbags.rosbag2 import StoragePlugin, Writer
        from rosbags.typesys import Stores, get_typestore

        self._ts = get_typestore(Stores.ROS2_JAZZY)
        self._writer = Writer(Path(path), version=8, storage_plugin=StoragePlugin.MCAP)
        self._writer.open()
        self._odom = self._writer.add_connection(
            '/rko_lio/odometry', 'nav_msgs/msg/Odometry', typestore=self._ts)
        self._frame = self._writer.add_connection(
            '/rko_lio/frame', 'sensor_msgs/msg/PointCloud2', typestore=self._ts)
        self._fix = None  # added with the first fix, so an input without GNSS has no such topic

    def _header(self, header: _core.Header) -> Any:
        m = self._ts.types
        return m['std_msgs/msg/Header'](
            stamp=m['builtin_interfaces/msg/Time'](
                sec=header.stamp.sec, nanosec=header.stamp.nanosec),
            frame_id=header.frame_id)

    def write(self, cloud: _core.PackedCloud, odom: _core.Odometry, time_ns: int) -> None:
        m = self._ts.types
        fields = [m['sensor_msgs/msg/PointField'](
            name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
            for f in cloud.fields]
        frame = m['sensor_msgs/msg/PointCloud2'](
            header=self._header(cloud.header), height=cloud.height, width=cloud.width,
            fields=fields, is_bigendian=cloud.is_bigendian, point_step=cloud.point_step,
            row_step=cloud.row_step, data=np.frombuffer(cloud.data, dtype=np.uint8),
            is_dense=cloud.is_dense)
        vec, point = m['geometry_msgs/msg/Vector3'], m['geometry_msgs/msg/Point']
        odometry = m['nav_msgs/msg/Odometry'](
            header=self._header(odom.header), child_frame_id=odom.child_frame_id,
            pose=m['geometry_msgs/msg/PoseWithCovariance'](
                pose=m['geometry_msgs/msg/Pose'](
                    position=point(x=odom.position[0], y=odom.position[1], z=odom.position[2]),
                    orientation=m['geometry_msgs/msg/Quaternion'](
                        x=odom.orientation[0], y=odom.orientation[1], z=odom.orientation[2],
                        w=odom.orientation[3])),
                covariance=np.asarray(odom.pose_covariance, dtype=np.float64)),
            twist=m['geometry_msgs/msg/TwistWithCovariance'](
                twist=m['geometry_msgs/msg/Twist'](
                    linear=vec(x=odom.linear_velocity[0], y=odom.linear_velocity[1],
                               z=odom.linear_velocity[2]),
                    angular=vec(x=odom.angular_velocity[0], y=odom.angular_velocity[1],
                                z=odom.angular_velocity[2])),
                covariance=np.asarray(odom.twist_covariance, dtype=np.float64)))
        self._writer.write(
            self._frame, time_ns, self._ts.serialize_cdr(frame, 'sensor_msgs/msg/PointCloud2'))
        self._writer.write(
            self._odom, time_ns, self._ts.serialize_cdr(odometry, 'nav_msgs/msg/Odometry'))

    def write_fix(self, topic: str, fix: _core.NavSatFix, time_ns: int) -> None:
        """A NavSatFix of the backend's gnss_topic, at its log time in the sensor log."""
        m = self._ts.types
        if self._fix is None:
            self._fix = self._writer.add_connection(
                topic, 'sensor_msgs/msg/NavSatFix', typestore=self._ts)
        msg = m['sensor_msgs/msg/NavSatFix'](
            header=self._header(fix.header),
            status=m['sensor_msgs/msg/NavSatStatus'](status=fix.status, service=fix.service),
            latitude=fix.latitude, longitude=fix.longitude, altitude=fix.altitude,
            position_covariance=np.asarray(fix.position_covariance, dtype=np.float64),
            position_covariance_type=fix.position_covariance_type)
        self._writer.write(
            self._fix, time_ns, self._ts.serialize_cdr(msg, 'sensor_msgs/msg/NavSatFix'))

    def close(self) -> None:
        self._writer.close()


def run(log_path: str | Path, odometry_params: dict[str, Any], backend_params: dict[str, Any],
        out_dir: str | Path, log: Callable[[str, str], None] | None = None,
        backend_input: str | Path | None = None) -> dict[str, Any]:
    """Maps a sensor log: odometry, then the backend on its deskewed scans; returns a summary.

    ``odometry_params`` are the RKO-LIO node's, ``backend_params`` the ``graph_based_slam`` ones.
    As the ROS 2 launch file does, the backend takes ``extrinsic_lidar2base_quat_xyzw_xyz`` from
    the odometry parameters. Writes the backend's files and, unless ``offline_map_bundle`` is
    false, the Autoware map bundle to ``out_dir``, and the odometry trajectory (TUM) to
    ``out_dir/odometry_tum.txt``.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    odometry_params = dict(odometry_params)
    odometry_params['publish_deskewed_scan'] = True
    backend_params = dict(backend_params)
    extrinsic = odometry_params.get('extrinsic_lidar2base_quat_xyzw_xyz')
    if extrinsic is not None:
        backend_params['extrinsic_lidar2base_quat_xyzw_xyz'] = [float(v) for v in extrinsic]
    # The Autoware map bundle (pointcloud_map/, map.pcd, map_projector_info.yaml, ...) unless the
    # caller turns it off.
    backend_params.setdefault('offline_map_bundle', True)
    backend_params['bag_path'] = str(log_path)
    backend_params['output_dir'] = str(out)

    pipeline = _core.BackendPipeline(backend_params, log)
    if not pipeline.configure():
        raise RuntimeError('the backend could not be configured (see its log)')
    pipeline.begin()

    base_frame = str(odometry_params.get('base_frame', ''))
    odom_frame = str(odometry_params.get('odom_frame') or 'odom')
    frames = {'count': 0}
    recorder = BackendInputWriter(backend_input) if backend_input is not None else None

    def on_output(output: dict[str, Any], state: dict[str, Any]) -> None:
        lidar_frame = str(odometry_params.get('lidar_frame') or state['lidar_frame_id'] or '')
        if lidar_frame.startswith('/'):
            lidar_frame = lidar_frame[1:]
        # The node publishes the frame first, then the odometry.
        cloud = deskewed_cloud(output, lidar_frame)
        odom = odometry_message(output, odom_frame, base_frame)
        if recorder is not None:
            recorder.write(cloud, odom, output['time_ns'])
        pipeline.add_cloud(cloud)
        pipeline.add_odometry(odom)
        frames['count'] += 1

    passthrough = {}
    gnss_fixes = {'count': 0}
    if pipeline.uses_gnss:
        with SensorLog(log_path) as sensor_log:
            types = {t.name: t.msgtype for t in sensor_log.topics()}
        if types.get(pipeline.gnss_topic) == NAV_SAT_FIX:
            def on_fix(record: Any) -> None:
                if recorder is not None:
                    recorder.write_fix(pipeline.gnss_topic, record.message, record.log_time_ns)
                pipeline.add_gnss_fix(record.message)
                gnss_fixes['count'] += 1

            passthrough[pipeline.gnss_topic] = on_fix

    try:
        state = drive(log_path, odometry_params, on_output, passthrough)
    finally:
        if recorder is not None:
            recorder.close()
    state['driver'].write_tum(str(out / 'odometry_tum.txt'))
    if not pipeline.finish():
        raise RuntimeError('the backend failed (see its log)')
    return {
        'output_dir': str(out),
        'frames': frames['count'],
        'imu_messages': state['imu_messages'],
        'lidar_messages': state['lidar_messages'],
        'gnss_fixes': gnss_fixes['count'],
        'dropped_trailing_frames': state['driver'].pending_lidar_frames,
    }
