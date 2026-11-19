"""Causal startup registration against the primary frame of a long-term store."""

from collections import deque
from pathlib import Path
import shutil
import tempfile
import zipfile

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from lidarslam import _core


def pose_matrix(pose):
    """Convert a finite [x, y, z, qx, qy, qz, qw] pose to a rigid transform."""
    values = np.asarray(pose, dtype=np.float64)
    if values.shape != (7,) or not np.isfinite(values).all() or not np.any(values[3:]):
        raise ValueError('expected a finite pose [x, y, z, qx, qy, qz, qw] with nonzero quaternion')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    transform[:3, 3] = values[:3]
    return transform


class KeyframeWindows:
    """Collect the deskewed scans around each committed backend keyframe."""

    def __init__(self, lidar_extrinsic, out_dir):
        self.extrinsic = pose_matrix(lidar_extrinsic)
        self.samples = deque()
        self.pending = deque()
        self._spool = tempfile.TemporaryFile(dir=out_dir)
        self._archive = zipfile.ZipFile(self._spool, mode='w', compression=zipfile.ZIP_DEFLATED)

    def add_scan(self, output):
        points = np.asarray(output['deskewed_scan']).reshape(-1, 3)
        points = points @ self.extrinsic[:3, :3].T + self.extrinsic[:3, 3]
        pose = pose_matrix(output['pose'])
        self.samples.append((output['time_ns'] * 1e-9,
                             points @ pose[:3, :3].T + pose[:3, 3]))

    def add_keyframe(self, pose):
        odometry = pose_matrix(pose['odometry'])
        frame = odometry.copy()
        yaw = np.arctan2(odometry[1, 0], odometry[0, 0])
        frame[:3, :3] = Rotation.from_rotvec([0, 0, yaw]).as_matrix()
        self.pending.append((pose['submaps'] - 1, pose['stamp_sec'], frame,
                             odometry, pose_matrix(pose['map'])))

    def advance(self, stamp):
        while self.pending and stamp > self.pending[0][1] + 1:
            index, key_time, frame, odometry, local_map = self.pending.popleft()
            scans = [(cloud - frame[:3, 3]) @ frame[:3, :3] for t, cloud in self.samples
                     if abs(t - key_time) <= 1]
            if not scans:
                continue
            points = np.concatenate(scans)
            points = np.ascontiguousarray(points[np.linalg.norm(points, axis=1) < 40], dtype='f4')
            base_from_frame = np.linalg.inv(odometry) @ frame
            local = points @ base_from_frame[:3, :3].T + base_from_frame[:3, 3]
            thinned = _core.StoreRelocalizer.thin(np.ascontiguousarray(local, dtype='f4'))
            with self._archive.open(f'{index}.npy', 'w') as handle:
                np.lib.format.write_array(handle, thinned, allow_pickle=False)
            yield index, frame, odometry, local_map, points, thinned
        keep_after = (self.pending[0][1] if self.pending else stamp) - 1
        while self.samples and self.samples[0][0] < keep_after:
            self.samples.popleft()

    def write(self, directory):
        try:
            self._archive.close()
            self._spool.seek(0)
            with (Path(directory) / 'keyframe_windows.npz').open('wb') as handle:
                shutil.copyfileobj(self._spool, handle)
        finally:
            self._spool.close()


def tracked_poses(directory):
    """Read optional absolute keyframe poses of a stored session."""
    path = Path(directory) / 'store_submap_poses.npy'
    return np.load(path, allow_pickle=False) if path.is_file() else []


class StartupRelocalization:
    """Verify an initial pose and optionally track it with fixed-reference loop factors."""

    def __init__(self, aligned_sessions, initial_pose, track=False, shared_surfaces=False):
        self.prior = pose_matrix(initial_pose) if initial_pose is not None else None
        directories, transforms = zip(*aligned_sessions)
        windows = []
        for directory in directories:
            path = Path(directory) / 'keyframe_windows.npz'
            if path.is_file():
                with np.load(path, allow_pickle=False) as data:
                    windows.append({int(index): data[index] for index in data.files})
            else:
                windows.append({})
        self.matcher = _core.StoreRelocalizer(directories, transforms, windows,
                                              [tracked_poses(path) for path in directories])
        self._surfaces = None
        if shared_surfaces and sum(bool(window) for window in windows) >= 2:
            from lidarslam.shared_surfaces import SharedSurfaces
            self._surfaces = SharedSurfaces.from_windows(windows, self.matcher.session_poses())
        self.first_stamp = None
        self.expired = False
        self.transform = None
        self.anchor_stamp = None
        self.track = track
        self._odometry = []
        self._observations = []
        self._initial_odometry = None

    def add_keyframe(self, pose):
        if self.track:
            if pose['submaps'] != len(self._odometry) + 1:
                raise RuntimeError('tracking keyframes must arrive in submap order')
            self._odometry.append(pose_matrix(pose['odometry']))

    def receive(self, header):
        if not self.active:
            return
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        if self.first_stamp is None:
            self.first_stamp = stamp
        elif stamp - self.first_stamp > 10:
            self.expired = True

    def _match(self, frame, odometry, local_map, points):
        if self.active:
            prior = self.prior @ frame if self.prior is not None else None
        elif self.track and self.transform is not None:
            prior = self.transform @ local_map @ np.linalg.inv(odometry) @ frame
        else:
            return
        result = (self.matcher.match_near_launch(points) if prior is None
                  else self.matcher.match_with_reference(points, prior))
        if result is None:
            return
        reference, proposal = result
        if self.transform is not None:
            yaw = np.arctan2(proposal[1, 0], proposal[0, 0]) - np.arctan2(prior[1, 0], prior[0, 0])
            if (np.linalg.norm(prior[:3, 3] - proposal[:3, 3]) > .20 or
                    abs(np.arctan2(np.sin(yaw), np.cos(yaw))) > np.deg2rad(2)):
                return
        return reference, proposal @ np.linalg.inv(frame) @ odometry

    def observe(self, stamp, index, frame, odometry, local_map, points, body_points):
        shared = self._surfaces is not None and self.transform is not None
        if shared:
            prior = self.transform @ local_map
            observed = self._surfaces.align(body_points, prior)
            if observed is None:
                return
            reference = None
        else:
            if self.active and self.prior is None:
                # Describe the incoming window at the same density as the stored archive.
                level_from_body = np.linalg.inv(frame) @ odometry
                points = body_points @ level_from_body[:3, :3].T + level_from_body[:3, 3]
            match = self._match(frame, odometry, local_map, points)
            if match is None:
                return
            reference, observed = match
        if self.transform is None:
            self.transform = observed @ np.linalg.inv(local_map)
            self._initial_odometry = observed @ np.linalg.inv(odometry)
            self.anchor_stamp = stamp
        if self.track:
            self._observations.append((index, reference, observed))
            poses, _ = self._solve(index + 1)
            corrected = poses[-1]
            if shared:
                # Dampen the live correction about the current keyframe; retain the full
                # measurement in the graph used to export and reopen the committed store.
                rotations = Rotation.from_matrix([prior[:3, :3], corrected[:3, :3]])
                corrected[:3, :3] = Slerp([0, 1], rotations)(.35).as_matrix()
                corrected[:3, 3] = .65 * prior[:3, 3] + .35 * corrected[:3, 3]
            self.transform = corrected @ np.linalg.inv(local_map)

    def _solve(self, count):
        return self.matcher.optimize_tracking(self._odometry[:count], self._initial_odometry,
                                              self._observations)

    def write(self, directory, count):
        if not self._observations:
            return
        if count != len(self._odometry):
            raise RuntimeError('tracking poses do not match the stored submaps')
        poses, graph = self._solve(count)
        directory = Path(directory)
        np.save(directory / 'store_submap_poses.npy', poses, allow_pickle=False)
        (directory / 'store_tracking_graph.g2o').write_text(graph, encoding='utf-8')

    @property
    def tracking_observations(self):
        return len(self._observations)

    @property
    def shared_observations(self):
        return sum(reference is None for _, reference, _ in self._observations)

    @property
    def active(self):
        return not self.expired and self.transform is None

    def store_pose(self, pose):
        transform = self.transform @ pose_matrix(pose)
        return [*transform[:3, 3], *Rotation.from_matrix(transform[:3, :3]).as_quat()]
