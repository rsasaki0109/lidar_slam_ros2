"""Live mapping without ROS 2: feed IMU and LiDAR messages as they arrive, get poses as they go.

A :class:`Session` is the one-pass mapping (:mod:`lidarslam.mapping`) driven message by message
instead of over a log: RKO-LIO registers each scan, its deskewed scan and odometry go to the
graph_based_slam backend at once, and the backend reports every scan's *live* pose: the odometry
pose corrected by the latest pose-graph optimization its background worker has finished, which
is what a robot could use at that moment (the backend's causal trajectory,
``trajectory_causal.tum``). :meth:`Session.finish` runs the backend's remaining stages and writes
the same files as ``python -m lidarslam map``.

:func:`replay` plays a sensor log into a session, at the sensor's rate with ``realtime`` (the
playback waits for each message's log time), and records how far behind the data the poses came.
This is the "live" mode of docs/roadmap/long-term-mapping-2026-11.md (milestone L1).
"""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any, Callable

from lidarslam import _core
from lidarslam.io import IMU, NAV_SAT_FIX, POINT_CLOUD2, SensorLog
from lidarslam.mapping import deskewed_cloud, odometry_message
from lidarslam.odometry import make_driver

LivePose = dict[str, Any]


class Session:
    """One live mapping session.

    ``odometry_params``: the RKO-LIO node's parameters; ``backend_params``: graph_based_slam's (as
    for :func:`lidarslam.mapping.run`). ``on_pose(pose)`` gets every scan's live pose as a dict:
    ``stamp_sec``, ``odometry`` and ``map`` (``[x, y, z, qx, qy, qz, qw]``), ``new_submap``,
    ``submaps``, ``corrections_applied``. The backend writes its files to ``out_dir`` at
    :meth:`finish`.
    """

    def __init__(self, odometry_params: dict[str, Any], backend_params: dict[str, Any],
                 out_dir: str | Path, on_pose: Callable[[LivePose], None] | None = None,
                 log: Callable[[str, str], None] | None = None):
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        odometry_params = dict(odometry_params)
        odometry_params['publish_deskewed_scan'] = True
        backend_params = dict(backend_params)
        extrinsic = odometry_params.get('extrinsic_lidar2base_quat_xyzw_xyz')
        if extrinsic is not None:
            backend_params['extrinsic_lidar2base_quat_xyzw_xyz'] = [float(v) for v in extrinsic]
        backend_params.setdefault('offline_map_bundle', True)
        backend_params.setdefault('bag_path', str(self.out / 'live'))
        backend_params['output_dir'] = str(self.out)
        backend_params['causal_trajectory_output'] = True

        self._base_frame = str(odometry_params.get('base_frame', ''))
        self._odom_frame = str(odometry_params.get('odom_frame') or 'odom')
        self._lidar_frame = str(odometry_params.get('lidar_frame') or '')
        self._driver = make_driver(odometry_params)
        self._pipeline = _core.BackendPipeline(backend_params, log)
        if not self._pipeline.configure():
            raise RuntimeError('the backend could not be configured (see its log)')
        self._on_pose = on_pose
        self._pipeline.set_live_pose_listener(self._pose)
        self._pipeline.begin()
        self.poses: list[LivePose] = []
        self.imu_messages = 0
        self.lidar_messages = 0
        self._finished = False

    @property
    def lidar_topic(self) -> str:
        return self._driver.lidar_topic

    @property
    def imu_topic(self) -> str:
        return self._driver.imu_topic

    @property
    def uses_gnss(self) -> bool:
        return self._pipeline.uses_gnss

    @property
    def gnss_topic(self) -> str:
        return self._pipeline.gnss_topic

    def _pose(self, pose: LivePose) -> None:
        self.poses.append(pose)
        if self._on_pose is not None:
            self._on_pose(pose)

    def _forward(self) -> None:
        for output in self._driver.take_outputs():
            lidar_frame = self._lidar_frame.lstrip('/')
            # The ROS 2 offline node publishes the frame first, then the odometry.
            self._pipeline.add_cloud(deskewed_cloud(output, lidar_frame))
            self._pipeline.add_odometry(odometry_message(output, self._odom_frame,
                                                         self._base_frame))

    def add_imu(self, imu: Any) -> None:
        """An IMU message (``lidarslam._core.Imu``, as :class:`lidarslam.io.SensorLog` reads)."""
        self._driver.add_imu(imu)
        self.imu_messages += 1
        self._forward()

    def add_lidar(self, cloud: Any) -> None:
        """A LiDAR scan (``lidarslam._core.PackedCloud`` with per-point time)."""
        if not self._lidar_frame:
            self._lidar_frame = cloud.header.frame_id
        self._driver.add_lidar(cloud)
        self.lidar_messages += 1
        self._forward()

    def add_gnss_fix(self, fix: Any) -> None:
        """A NavSatFix of the backend's ``gnss_topic`` (used with ``use_gnss``)."""
        self._pipeline.add_gnss_fix(fix)

    def finish(self) -> dict[str, Any]:
        """Runs the backend's remaining stages and writes its files; returns a summary."""
        if self._finished:
            raise RuntimeError('the session is already finished')
        self._finished = True
        self._driver.write_tum(str(self.out / 'odometry_tum.txt'))
        if not self._pipeline.finish():
            raise RuntimeError('the backend failed (see its log)')
        return {
            'output_dir': str(self.out),
            'live_poses': len(self.poses),
            'imu_messages': self.imu_messages,
            'lidar_messages': self.lidar_messages,
            'dropped_trailing_frames': self._driver.pending_lidar_frames,
        }


def replay(log_path: str | Path, odometry_params: dict[str, Any],
           backend_params: dict[str, Any], out_dir: str | Path, realtime: bool = False,
           speed: float = 1.0, log: Callable[[str, str], None] | None = None) -> dict[str, Any]:
    """Plays a sensor log into a :class:`Session` and writes its live trajectory and timing.

    With ``realtime`` each message is handed over no earlier than its log time after the first
    message, divided by ``speed``. Writes ``trajectory_live.tum`` (the live poses) and
    ``live_timing.json``. With ``realtime`` it records each pose's ``lag``: the wall time from
    the moment the pose's data time (the scan's end) was reached in the playback to the moment
    the pose was available; a live system keeps up when the lag stays small and does not grow.
    """
    out = Path(out_dir)
    lags: list[float] = []
    clock = {'wall_start': time.monotonic(), 'log_start_sec': None}

    def on_pose(pose: LivePose) -> None:
        if realtime and clock['log_start_sec'] is not None:
            reached = clock['wall_start'] + (pose['stamp_sec'] - clock['log_start_sec']) / speed
            lags.append(time.monotonic() - reached)

    session = Session(odometry_params, backend_params, out, on_pose=on_pose, log=log)
    topics = [session.lidar_topic, session.imu_topic]
    with SensorLog(log_path) as sensor_log:
        types = {t.name: t.msgtype for t in sensor_log.topics()}
        for topic, expected in ((session.lidar_topic, POINT_CLOUD2), (session.imu_topic, IMU)):
            if types.get(topic) != expected:
                raise KeyError(f'{topic}: not a {expected} topic of {log_path}')
        gnss = session.uses_gnss and types.get(session.gnss_topic) == NAV_SAT_FIX
        if gnss:
            topics.append(session.gnss_topic)
        log_start = None
        clock['wall_start'] = time.monotonic()
        for record in sensor_log.records(topics):
            if log_start is None:
                log_start = record.log_time_ns
                clock['log_start_sec'] = log_start * 1e-9
            if realtime:
                due = clock['wall_start'] + (record.log_time_ns - log_start) * 1e-9 / speed
                delay = due - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            if record.topic == session.imu_topic:
                session.add_imu(record.message)
            elif record.topic == session.lidar_topic:
                session.add_lidar(record.message)
            else:
                session.add_gnss_fix(record.message)
    data_sec = 0.0
    if session.poses:
        data_sec = session.poses[-1]['stamp_sec'] - session.poses[0]['stamp_sec']
    wall_sec = time.monotonic() - clock['wall_start']
    summary = session.finish()
    finish_sec = time.monotonic() - clock['wall_start'] - wall_sec

    with (out / 'trajectory_live.tum').open('w', encoding='utf-8') as handle:
        for pose in session.poses:
            handle.write(' '.join(f'{v:.9f}' for v in (pose['stamp_sec'], *pose['map'])) + '\n')
    lags.sort()

    def percentile(values: list[float], q: float) -> float:
        return values[min(len(values) - 1, int(q * len(values)))] if values else 0.0

    stats = {
        'realtime': realtime, 'speed': speed, 'live_poses': len(session.poses),
        'data_seconds': round(data_sec, 3), 'stream_wall_seconds': round(wall_sec, 3),
        'finish_wall_seconds': round(finish_sec, 3),
        'lag_seconds': ({'p50': percentile(lags, 0.5), 'p95': percentile(lags, 0.95),
                         'max': lags[-1] if lags else 0.0} if realtime else None),
    }
    (out / 'live_timing.json').write_text(json.dumps(stats, indent=2) + '\n', encoding='utf-8')
    summary.update(stats)
    return summary
