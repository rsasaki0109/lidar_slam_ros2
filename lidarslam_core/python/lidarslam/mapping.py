"""Frontend and backend in one pass over a sensor log, without ROS 2.

RKO-LIO (:mod:`lidarslam.odometry`) registers each LiDAR scan; its deskewed scan and odometry go,
in memory, to the graph_based_slam offline backend (:mod:`lidarslam.backend`) as the messages the
ROS 2 offline node publishes on ``/rko_lio/frame`` and ``/rko_lio/odometry``: the same fields,
frames and stamps, so the backend sees what it would read from a recorded backend input.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

import numpy as np

from lidarslam import _core
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


def run(log_path: str | Path, odometry_params: dict[str, Any], backend_params: dict[str, Any],
        out_dir: str | Path, log: Callable[[str, str], None] | None = None) -> dict[str, Any]:
    """Maps a sensor log: odometry, then the backend on its deskewed scans; returns a summary.

    ``odometry_params`` are the RKO-LIO node's, ``backend_params`` the ``graph_based_slam`` ones.
    As the ROS 2 launch file does, the backend takes ``extrinsic_lidar2base_quat_xyzw_xyz`` from
    the odometry parameters. Writes the backend's files to ``out_dir`` and the odometry
    trajectory (TUM) to ``out_dir/odometry_tum.txt``.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    odometry_params = dict(odometry_params)
    odometry_params['publish_deskewed_scan'] = True
    backend_params = dict(backend_params)
    extrinsic = odometry_params.get('extrinsic_lidar2base_quat_xyzw_xyz')
    if extrinsic is not None:
        backend_params['extrinsic_lidar2base_quat_xyzw_xyz'] = [float(v) for v in extrinsic]
    backend_params['bag_path'] = str(log_path)
    backend_params['output_dir'] = str(out)

    pipeline = _core.BackendPipeline(backend_params, log)
    if not pipeline.configure():
        raise RuntimeError('the backend could not be configured (see its log)')
    pipeline.begin()

    base_frame = str(odometry_params.get('base_frame', ''))
    odom_frame = str(odometry_params.get('odom_frame') or 'odom')
    frames = {'count': 0}

    def on_output(output: dict[str, Any], state: dict[str, Any]) -> None:
        lidar_frame = str(odometry_params.get('lidar_frame') or state['lidar_frame_id'] or '')
        if lidar_frame.startswith('/'):
            lidar_frame = lidar_frame[1:]
        # The node publishes the frame first, then the odometry.
        pipeline.add_cloud(deskewed_cloud(output, lidar_frame))
        pipeline.add_odometry(odometry_message(output, odom_frame, base_frame))
        frames['count'] += 1

    state = drive(log_path, odometry_params, on_output)
    state['driver'].write_tum(str(out / 'odometry_tum.txt'))
    if not pipeline.finish():
        raise RuntimeError('the backend failed (see its log)')
    return {
        'output_dir': str(out),
        'frames': frames['count'],
        'imu_messages': state['imu_messages'],
        'lidar_messages': state['lidar_messages'],
        'dropped_trailing_frames': state['driver'].pending_lidar_frames,
    }
