"""lidarslam backend without ROS 2 on a synthetic backend input (CI smoke test).

Odometry and clouds of a sensor walking twice around a 12 m x 8 m box room, as the frontend
records them for the backend (equal stamps, clouds in the odometry child frame). The backend must
run end to end, write the runner's files and give the same bytes twice. Byte identity with the
ROS 2 runner is checked on real logs (docs/roadmap/ros-free-core-2026-11.md), not here.
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path

import pytest

lidarslam = pytest.importorskip('lidarslam')
import numpy as np  # noqa: E402
core = pytest.importorskip('lidarslam._core')
if not hasattr(core, 'BackendPipeline'):
    pytest.skip('lidarslam built without the backend', allow_module_level=True)

from lidarslam.backend import load_parameters, run  # noqa: E402
from lidarslam.params import apply_overrides  # noqa: E402
from rosbags.rosbag2 import StoragePlugin, Writer  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

TS = get_typestore(Stores.ROS2_JAZZY)
M = TS.types


def _room(rng: np.random.Generator, n: int = 8000) -> np.ndarray:
    half = np.array([6.0, 4.0, 1.5])
    pts = rng.uniform(-half, half, size=(n, 3))
    axis = rng.integers(0, 3, size=n)
    pts[np.arange(n), axis] = rng.choice([-1.0, 1.0], size=n) * half[axis]
    pillars = np.repeat(np.array([[-3.0, -1.5], [2.0, 1.0], [4.0, -2.5]]), 300, axis=0)
    pillars = np.column_stack([pillars + rng.normal(0, 0.1, size=pillars.shape),
                               rng.uniform(-1.5, 1.5, size=len(pillars))])
    return np.vstack([pts, pillars])


def _path(k: int, n: int) -> tuple[np.ndarray, float]:
    """Two laps of an ellipse inside the room; position and yaw."""
    t = 4.0 * math.pi * k / n
    pos = np.array([4.0 * math.cos(t), 2.5 * math.sin(t), 0.0])
    yaw = math.atan2(2.5 * math.cos(t), -4.0 * math.sin(t))
    return pos, yaw


def _write_input(path: Path, frames: int = 160) -> None:
    rng = np.random.default_rng(3)
    room = _room(rng)
    header, time = M['std_msgs/msg/Header'], M['builtin_interfaces/msg/Time']
    pf = M['sensor_msgs/msg/PointField']
    fields = [pf(name=n, offset=4 * i, datatype=7, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
    point, quat = M['geometry_msgs/msg/Point'], M['geometry_msgs/msg/Quaternion']
    pose_cov = M['geometry_msgs/msg/PoseWithCovariance']
    twist_cov = M['geometry_msgs/msg/TwistWithCovariance']
    twist, vec = M['geometry_msgs/msg/Twist'], M['geometry_msgs/msg/Vector3']
    start = 1_700_000_000 * 10**9
    with Writer(path, version=8, storage_plugin=StoragePlugin.MCAP) as writer:
        odom_conn = writer.add_connection('/rko_lio/odometry', 'nav_msgs/msg/Odometry', typestore=TS)
        cloud_conn = writer.add_connection('/rko_lio/frame', 'sensor_msgs/msg/PointCloud2',
                                           typestore=TS)
        for k in range(frames):
            stamp = start + k * 100_000_000
            sec, nsec = divmod(stamp, 10**9)
            pos, yaw = _path(k, frames)
            c, s = math.cos(yaw), math.sin(yaw)
            rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            local = (room - pos) @ rot  # world -> sensor (R^T (p - t))
            local = local[np.linalg.norm(local[:, :2], axis=1) < 15.0]
            local = local + rng.normal(0.0, 0.01, size=local.shape)
            rec = np.zeros(len(local), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                              ('intensity', '<f4')])
            rec['x'], rec['y'], rec['z'] = local[:, 0], local[:, 1], local[:, 2]
            rec['intensity'] = 10.0
            cloud = M['sensor_msgs/msg/PointCloud2'](
                header=header(stamp=time(sec=sec, nanosec=nsec), frame_id='base_link'), height=1, width=len(rec), fields=fields, is_bigendian=False,
                point_step=16, row_step=16 * len(rec), data=rec.view(np.uint8).copy(),
                is_dense=True)
            odom = M['nav_msgs/msg/Odometry'](
                header=header(stamp=time(sec=sec, nanosec=nsec), frame_id='odom'),
                child_frame_id='base_link',
                pose=pose_cov(pose=M['geometry_msgs/msg/Pose'](
                    position=point(x=pos[0], y=pos[1], z=pos[2]),
                    orientation=quat(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))),
                    covariance=np.eye(6).ravel() * 1e-4),
                twist=twist_cov(twist=twist(linear=vec(x=0.0, y=0.0, z=0.0),
                                            angular=vec(x=0.0, y=0.0, z=0.0)),
                                covariance=np.zeros(36)))
            writer.write(odom_conn, stamp, TS.serialize_cdr(odom, 'nav_msgs/msg/Odometry'))
            writer.write(cloud_conn, stamp,
                         TS.serialize_cdr(cloud, 'sensor_msgs/msg/PointCloud2'))


def _digests(directory: Path) -> dict[str, str]:
    return {p.name: hashlib.md5(p.read_bytes()).hexdigest()
            for p in sorted(directory.iterdir()) if p.is_file()}


def test_walk_around_a_room(tmp_path: Path) -> None:
    log = tmp_path / 'backend_input'
    _write_input(log)
    params_file = tmp_path / 'params.yaml'
    params_file.write_text(
        'graph_based_slam:\n'
        '  ros__parameters:\n'
        '    registration_method: "SMALL_GICP"\n'
        '    gicp_max_correspondence_distance: 0.5\n'
        '    ndt_num_threads: 1\n'  # one registration thread: deterministic, as the presets
        '    voxel_leaf_size: 0.2\n'
        '    submap_distance_threshold: 1.5\n'
        '    refine: true\n'
        'scan_matcher:\n'
        '  ros__parameters:\n'
        '    registration_method: "NDT"\n',
        encoding='utf-8')
    params = apply_overrides(load_parameters(params_file), ['causal_trajectory_output:=true'])
    assert params['registration_method'] == 'SMALL_GICP'  # only the graph_based_slam section

    messages: list[tuple[str, str]] = []
    summary = run(log, params, tmp_path / 'out1', log=lambda level, msg: messages.append(
        (level, msg)))
    assert summary == {'output_dir': str(tmp_path / 'out1'), 'odometry_messages': 160,
                       'cloud_messages': 160, 'gnss_fixes': 0}
    assert any('Offline run complete: 160 pairs' in msg for _, msg in messages)
    assert not [msg for level, msg in messages if level == 'ERROR']

    out = tmp_path / 'out1'
    for name in ('trajectory_raw.tum', 'trajectory_optimized.tum', 'loop_edges.csv',
                 'registration_plugin_receipt.yaml', 'trajectory_causal.tum'):
        assert (out / name).is_file(), name
    submaps = np.loadtxt(out / 'trajectory_raw.tum', ndmin=2)
    assert len(submaps) >= 20  # ~50 m walked, a submap every 1.5 m
    assert 'resolved_class: "lidarslam_builtin/BackendSmallGicp"' in (
        out / 'registration_plugin_receipt.yaml').read_text()

    run(log, params, tmp_path / 'out2', log=lambda level, msg: None)
    # trajectory_causal.tum models the loop worker's delay with the measured compute time, so it
    # varies between runs by design; every other file is deterministic.
    first, second = _digests(tmp_path / 'out1'), _digests(tmp_path / 'out2')
    del first['trajectory_causal.tum'], second['trajectory_causal.tum']
    assert first == second


def test_parameter_types_follow_rclcpp(tmp_path: Path) -> None:
    log = tmp_path / 'backend_input'
    log.mkdir()
    with pytest.raises(TypeError, match='voxel_leaf_size'):
        core.BackendPipeline({'bag_path': str(log), 'output_dir': str(tmp_path / 'o'),
                              'voxel_leaf_size': 1}, None).configure()  # an integer, not a double
    messages: list[tuple[str, str]] = []
    pipeline = core.BackendPipeline({'output_dir': str(tmp_path / 'o')},
                                    lambda level, msg: messages.append((level, msg)))
    assert not pipeline.configure()
    assert messages == [('ERROR', 'bag_path parameter is required')]
    with pytest.raises(ValueError, match='empty list'):
        core.BackendPipeline({'session_bag_paths': []}, None)
