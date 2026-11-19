"""Live mapping without ROS 2: feed IMU and LiDAR messages as they arrive, get poses as they go.

A :class:`Session` is the one-pass mapping (:mod:`lidarslam.mapping`) driven message by message
instead of over a log: RKO-LIO registers each scan, its deskewed scan and odometry go to the
graph_based_slam backend at once, and the backend reports every scan's *live* pose: the odometry
pose corrected by the latest pose-graph optimization its background worker has finished, which
is what a robot could use at that moment (the backend's causal trajectory,
``trajectory_causal.tum``). :meth:`Session.finish` runs the backend's remaining stages and writes
the same files as ``python -m lidarslam map``. With ``store`` (:class:`lidarslam.store.Store`) the
finished session is added to that long-term map store and, as its first session, exported as the
store's Autoware bundle (milestone L2).

:func:`replay` plays a sensor log into a session, at the sensor's rate with ``realtime`` (the
playback waits for each message's log time), and records how far behind the data the poses came.
This is the "live" mode of docs/roadmap/long-term-mapping-2026-11.md (milestone L1).
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import time
from typing import Any, Callable

from lidarslam import _core
from lidarslam.io import IMU, NAV_SAT_FIX, POINT_CLOUD2, SensorLog
from lidarslam.mapping import deskewed_cloud, odometry_message
from lidarslam.odometry import make_driver
from lidarslam.store import Store

LivePose = dict[str, Any]


class Session:
    """One live mapping session.

    ``odometry_params``: the RKO-LIO node's parameters; ``backend_params``: graph_based_slam's (as
    for :func:`lidarslam.mapping.run`). ``on_pose(pose)`` gets every scan's live pose as a dict:
    ``stamp_sec``, ``odometry`` and ``map`` (``[x, y, z, qx, qy, qz, qw]``), ``new_submap``,
    ``submaps``, ``corrections_applied``. The backend writes its files to ``out_dir`` at
    :meth:`finish`; with ``store`` (a :class:`lidarslam.store.Store` or its path, created when
    missing) the session is also added to that map store. ``initial_pose`` places the new
    odometry frame in an existing store as position and quaternion. After startup registration
    is accepted, subsequent callbacks also carry ``store_pose`` and ``store_frame``.
    ``track_store`` enables experimental inter-session factors after startup and stores the
    resulting absolute submap poses with the session.
    ``track_shared_surfaces`` enables tracking and uses geometry seen in multiple stored
    sessions when a shared registration target can be built.
    ``detect_changes`` accumulates removal evidence from anchored keyframe scans.
    ``filter_new_points`` also masks new voxels without repeated scan support in exports.
    ``refine_visibility`` also counts free space before the first hit for new submaps.
    ``start_near_first_submap`` searches for a verified startup pose within 5 m of the first
    stored submap, when the operator knows that the recording starts in that area.
    """

    def __init__(self, odometry_params: dict[str, Any], backend_params: dict[str, Any],
                 out_dir: str | Path, on_pose: Callable[[LivePose], None] | None = None,
                 log: Callable[[str, str], None] | None = None,
                 store: Store | str | Path | None = None,
                 initial_pose: list[float] | None = None,
                 track_store: bool = False, track_shared_surfaces: bool = False,
                 detect_changes: bool = False, filter_new_points: bool = False,
                 start_near_first_submap: bool = False, refine_visibility: bool = False):
        filter_new_points = filter_new_points or refine_visibility
        detect_changes = detect_changes or filter_new_points
        if initial_pose is not None and start_near_first_submap:
            raise ValueError('choose either initial_pose or start_near_first_submap')
        localize = initial_pose is not None or start_near_first_submap
        if (track_store or track_shared_surfaces) and not localize:
            raise ValueError('store tracking requires a startup pose or launch-area search')
        self.out = Path(out_dir)
        self._store = (store if isinstance(store, Store) or store is None
                       else Store.open_or_create(store))
        if detect_changes and (self._store is None or (self._store.sessions and not localize)):
            raise ValueError('detect_changes requires a store and startup localization for updates')
        self._log = log
        self._odometry_params = dict(odometry_params)
        self.out.mkdir(parents=True, exist_ok=True)
        odometry_params = dict(odometry_params)
        odometry_params['publish_deskewed_scan'] = True
        backend_params = dict(backend_params)
        extrinsic = odometry_params.get('extrinsic_lidar2base_quat_xyzw_xyz')
        if extrinsic is not None:
            backend_params['extrinsic_lidar2base_quat_xyzw_xyz'] = [float(v) for v in extrinsic]
        self._startup = None
        self._windows = None
        self._changes = None
        self._change_rows = None
        if self._store is not None and hasattr(_core, 'StoreRelocalizer'):
            from lidarslam.relocalization import KeyframeWindows
            lidar_pose = (list(extrinsic[4:]) + list(extrinsic[:4]) if extrinsic is not None
                          else [0., 0., 0., 0., 0., 0., 1.])
            self._windows = KeyframeWindows(lidar_pose, self.out)
        if localize:
            if self._store is None or not self._store.sessions:
                raise ValueError('startup localization requires a store with a committed session')
            from lidarslam.relocalization import StartupRelocalization
            aligned = self._store._aligned_sessions()
            self._startup = StartupRelocalization(aligned, initial_pose,
                                                   track=track_store or track_shared_surfaces,
                                                   shared_surfaces=track_shared_surfaces)
            if detect_changes:
                from lidarslam.map_changes import MapChanges
                self._changes = MapChanges([path for path, _ in aligned],
                                           self._startup.matcher.session_poses(),
                                           self._windows.extrinsic, filter_new_points,
                                           refine_visibility)
        self._track_shared_surfaces = track_shared_surfaces
        backend_params.setdefault('offline_map_bundle', True)
        backend_params.setdefault('bag_path', str(self.out / 'live'))
        backend_params['output_dir'] = str(self.out)
        backend_params['causal_trajectory_output'] = True
        self._backend_params = backend_params

        self._base_frame = str(odometry_params.get('base_frame', ''))
        self._odom_frame = str(odometry_params.get('odom_frame') or 'odom')
        self._lidar_frame = str(odometry_params.get('lidar_frame') or '')
        self._driver = make_driver(odometry_params)
        self._pipeline = _core.BackendPipeline(backend_params, log)
        if not self._pipeline.configure():
            raise RuntimeError('the backend could not be configured (see its log)')
        self._on_pose = on_pose
        self._pipeline.set_live_pose_listener(self._pose)
        if self._store is not None:
            self._pipeline.keep_session_map(True)
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
        if self._startup is not None:
            if self._startup.transform is not None:
                pose['store_pose'] = self._startup.store_pose(pose['map'])
                pose['store_frame'] = self._store.frame
        if pose['new_submap']:
            if self._windows is not None:
                self._windows.add_keyframe(pose)
            if self._startup is not None:
                self._startup.add_keyframe(pose)
        self.poses.append(pose)
        if self._on_pose is not None:
            self._on_pose(pose)

    def _forward(self) -> None:
        for output in self._driver.take_outputs():
            if self._windows is not None:
                self._windows.add_scan(output)
            lidar_frame = self._lidar_frame.lstrip('/')
            previous_count = len(self.poses)
            # The ROS 2 offline node publishes the frame first, then the odometry.
            self._pipeline.add_cloud(deskewed_cloud(output, lidar_frame))
            self._pipeline.add_odometry(odometry_message(output, self._odom_frame,
                                                         self._base_frame))
            if self._windows is not None:
                stamp = output['time_ns'] * 1e-9
                for window in self._windows.advance(stamp):
                    if self._startup is not None:
                        self._startup.observe(stamp, *window)
            if self._changes is not None:
                pose = self.poses[-1] if len(self.poses) > previous_count else None
                self._changes.observe(pose, output['deskewed_scan'], self._startup)

    def add_imu(self, imu: Any) -> None:
        """An IMU message (``lidarslam._core.Imu``, as :class:`lidarslam.io.SensorLog` reads)."""
        if self._startup is not None:
            self._startup.receive(imu.header)
        self._driver.add_imu(imu)
        self.imu_messages += 1
        self._forward()

    def add_lidar(self, cloud: Any) -> None:
        """A LiDAR scan (``lidarslam._core.PackedCloud`` with per-point time)."""
        if self._startup is not None:
            self._startup.receive(cloud.header)
        if not self._lidar_frame:
            self._lidar_frame = cloud.header.frame_id
        self._driver.add_lidar(cloud)
        self.lidar_messages += 1
        self._forward()

    def add_gnss_fix(self, fix: Any) -> None:
        """A NavSatFix of the backend's ``gnss_topic`` (used with ``use_gnss``)."""
        self._pipeline.add_gnss_fix(fix)

    def _write_store_session(self, directory: str) -> int:
        count = self._pipeline.write_session_map(directory)
        if self._windows is not None:
            self._windows.write(directory)
        if self._startup is not None and self._startup.track:
            self._startup.write(directory, count)
        if self._change_rows is not None:
            self._changes.write(directory, self._change_rows)
        return count

    def changes(self) -> dict[str, Any]:
        """Return the current evidence for the committed reference sessions."""
        if self._changes is None:
            return {'references': [], 'rows': []}
        return {'references': list(self._changes.references), 'rows': self._changes.rows()}

    def finish(self) -> dict[str, Any]:
        """Runs the backend's remaining stages and writes its files; returns a summary."""
        if self._finished:
            raise RuntimeError('the session is already finished')
        self._finished = True
        self._driver.write_tum(str(self.out / 'odometry_tum.txt'))
        if not self._pipeline.finish():
            raise RuntimeError('the backend failed (see its log)')
        if self._windows is not None:
            for _ in self._windows.advance(float('inf')):
                pass
        summary = {
            'output_dir': str(self.out),
            'live_poses': len(self.poses),
            'imu_messages': self.imu_messages,
            'lidar_messages': self.lidar_messages,
            'dropped_trailing_frames': self._driver.pending_lidar_frames,
        }
        if self._store is not None:
            info = {'live_poses': len(self.poses), 'lidar_messages': self.lidar_messages,
                    'first_stamp_sec': self.poses[0]['stamp_sec'] if self.poses else None,
                    'last_stamp_sec': self.poses[-1]['stamp_sec'] if self.poses else None}
            if self._startup is not None:
                summary['store_start_stamp_sec'] = self._startup.first_stamp
                summary['store_anchor_stamp_sec'] = self._startup.anchor_stamp
                if self._startup.track:
                    summary['store_tracking_observations'] = self._startup.tracking_observations
                if self._track_shared_surfaces:
                    shared_count = self._startup.shared_observations
                    info['shared_surface_observations'] = shared_count
                    summary['store_shared_surface_observations'] = shared_count
                if self._startup.transform is not None:
                    info['store_from_session'] = self._startup.transform.tolist()
                    summary['store_from_session'] = info['store_from_session']
                    if self._changes is not None:
                        self._change_rows = self._changes.rows()
                        info['map_change_evidence'] = True
                        if self._changes.refined_visibility:
                            info['map_change_refined_visibility'] = True
                        summary['store_removed_voxels'] = int((self._change_rows[:, 8] == 2).sum())
            entry = self._store.add_session(self._write_store_session,
                                            self._odometry_params, self._backend_params, info,
                                            log=self._log)
            summary['store'] = str(self._store.path)
            summary['store_session'] = entry['id']
            if self._changes is not None and self._changes.suppressed_new_voxels is not None:
                summary['store_suppressed_new_voxels'] = self._changes.suppressed_new_voxels
        return summary


def replay(log_path: str | Path, odometry_params: dict[str, Any],
           backend_params: dict[str, Any], out_dir: str | Path, realtime: bool = False,
           speed: float = 1.0, log: Callable[[str, str], None] | None = None,
           store: Store | str | Path | None = None,
           initial_pose: list[float] | None = None,
           track_store: bool = False, track_shared_surfaces: bool = False,
           detect_changes: bool = False, filter_new_points: bool = False,
           start_near_first_submap: bool = False, refine_visibility: bool = False) -> dict[str, Any]:
    """Plays a sensor log into a :class:`Session` and writes its live trajectory and timing.

    With ``realtime`` each message is handed over no earlier than its log time after the first
    message, divided by ``speed``. Writes ``trajectory_live.tum`` (the live poses) and
    ``live_timing.json``. With ``realtime`` it records each pose's ``lag``: the wall time from
    the moment the pose's data time (the scan's end) was reached in the playback to the moment
    the pose was available; a live system keeps up when the lag stays small and does not grow.
    Store and tracking options are as for :class:`Session`. Once anchored,
    writes ``trajectory_store.tum`` for the poses available in the store frame after that decision.
    Processing time includes reading and dispatching sensor records and excludes playback sleep.
    Peak RSS is measured for the entire process on Linux and macOS.
    """
    out = Path(out_dir)
    lags: list[float] = []
    clock = {'wall_start': time.monotonic(), 'log_start_sec': None}
    anchor_wall_sec = None
    active_seconds = 0.0

    def on_pose(pose: LivePose) -> None:
        if realtime and clock['log_start_sec'] is not None:
            reached = clock['wall_start'] + (pose['stamp_sec'] - clock['log_start_sec']) / speed
            lags.append(time.monotonic() - reached)

    session = Session(odometry_params, backend_params, out, on_pose=on_pose, log=log,
                      store=store, initial_pose=initial_pose, track_store=track_store,
                      track_shared_surfaces=track_shared_surfaces, detect_changes=detect_changes,
                      filter_new_points=filter_new_points,
                      start_near_first_submap=start_near_first_submap,
                      refine_visibility=refine_visibility)
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
        dispatched = clock['wall_start']
        for record in sensor_log.records(topics):
            active_seconds += time.monotonic() - dispatched
            if log_start is None:
                log_start = record.log_time_ns
                clock['log_start_sec'] = log_start * 1e-9
            if realtime:
                due = clock['wall_start'] + (record.log_time_ns - log_start) * 1e-9 / speed
                delay = due - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            dispatch_start = time.monotonic()
            if record.topic == session.imu_topic:
                session.add_imu(record.message)
            elif record.topic == session.lidar_topic:
                session.add_lidar(record.message)
            else:
                session.add_gnss_fix(record.message)
            if (session._startup is not None and anchor_wall_sec is None and
                    session._startup.anchor_stamp is not None):
                anchor_wall_sec = time.monotonic() - clock['wall_start']
            dispatched = time.monotonic()
            active_seconds += dispatched - dispatch_start
    data_sec = 0.0
    if session.poses:
        data_sec = session.poses[-1]['stamp_sec'] - session.poses[0]['stamp_sec']
    wall_sec = time.monotonic() - clock['wall_start']
    summary = session.finish()
    if session._startup is not None:
        summary['store_anchor_wall_sec'] = anchor_wall_sec
    finish_sec = time.monotonic() - clock['wall_start'] - wall_sec

    with (out / 'trajectory_live.tum').open('w', encoding='utf-8') as handle:
        for pose in session.poses:
            handle.write(' '.join(f'{v:.9f}' for v in (pose['stamp_sec'], *pose['map'])) + '\n')
    if session._startup is not None:
        store_poses = [pose for pose in session.poses if 'store_pose' in pose]
        path = out / 'trajectory_store.tum'
        if store_poses:
            with path.open('w', encoding='utf-8') as handle:
                for pose in store_poses:
                    handle.write(' '.join(f'{v:.9f}' for v in
                                          (pose['stamp_sec'], *pose['store_pose'])) + '\n')
        else:
            path.unlink(missing_ok=True)
    lags.sort()

    def percentile(values: list[float], q: float) -> float:
        return values[min(len(values) - 1, int(q * len(values)))] if values else 0.0

    peak_rss_mib = None
    if sys.platform in ('linux', 'darwin'):
        import resource
        rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        peak_rss_mib = rss / (1024 ** (2 if sys.platform == 'darwin' else 1))
    stats = {
        'realtime': realtime, 'speed': speed, 'live_poses': len(session.poses),
        'data_seconds': round(data_sec, 3), 'stream_wall_seconds': round(wall_sec, 3),
        'finish_wall_seconds': round(finish_sec, 3),
        'processing_wall_seconds': round(active_seconds, 3),
        'mean_processing_seconds_per_scan': (
            active_seconds / session.lidar_messages if session.lidar_messages else None),
        'process_peak_rss_mib': peak_rss_mib,
        'lag_seconds': ({'p50': percentile(lags, 0.5), 'p95': percentile(lags, 0.95),
                         'max': lags[-1] if lags else 0.0} if realtime else None),
    }
    (out / 'live_timing.json').write_text(json.dumps(stats, indent=2) + '\n', encoding='utf-8')
    summary.update(stats)
    return summary
