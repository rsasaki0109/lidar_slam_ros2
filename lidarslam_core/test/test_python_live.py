"""lidarslam live without ROS 2: a session fed message by message (CI smoke test).

Uses the static box-room log of test_python_odometry. A live session must give every scan a live
pose as it is processed, the live poses must be the backend's causal trajectory, and the files
the session writes at the end must be those of the one-pass mapping (``lidarslam map``) with
the causal trajectory switched on.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

lidarslam = pytest.importorskip('lidarslam')
core = pytest.importorskip('lidarslam._core')
if not (hasattr(core, 'OdometryDriver') and hasattr(core, 'BackendPipeline')):
    pytest.skip('lidarslam built without the frontend or the backend', allow_module_level=True)
if not hasattr(core.BackendPipeline, 'set_live_pose_listener'):
    pytest.skip('lidarslam built without live poses', allow_module_level=True)

from lidarslam import live, mapping  # noqa: E402
from lidarslam.io import SensorLog  # noqa: E402
from test_python_mapping import ODOMETRY_PARAMS  # noqa: E402
from test_python_odometry import _write_log  # noqa: E402

BACKEND_PARAMS = {'registration_method': 'SMALL_GICP', 'ndt_num_threads': 1,
                  'submap_distance_threshold': 0.0}


def test_session_gives_a_live_pose_per_scan(tmp_path: Path) -> None:
    log = tmp_path / 'log'
    _write_log(log)
    poses: list[dict] = []
    session = live.Session(ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'live',
                           on_pose=poses.append)
    with SensorLog(log) as sensor_log:
        for record in sensor_log.records([session.lidar_topic, session.imu_topic]):
            before = len(poses)
            if record.topic == session.imu_topic:
                session.add_imu(record.message)
            else:
                session.add_lidar(record.message)
            # Poses come out while the data goes in, never in a batch at the end.
            assert len(poses) - before <= 1
    summary = session.finish()
    assert summary['live_poses'] == len(poses) > 0
    for pose in poses:
        assert len(pose['odometry']) == len(pose['map']) == 7
        assert pose['submaps'] >= 1
    stamps = [pose['stamp_sec'] for pose in poses]
    assert stamps == sorted(stamps)

    # The live poses are the backend's causal trajectory.
    causal = (tmp_path / 'live' / 'trajectory_causal.tum').read_text().split('\n')
    causal = [[float(v) for v in line.split()] for line in causal if line]
    assert len(causal) == len(poses)
    for row, pose in zip(causal, poses):
        assert row[0] == pytest.approx(pose['stamp_sec'], abs=1e-9)
        assert row[1:] == pytest.approx(pose['map'], abs=1e-9)

    # And the files at the end are the one-pass mapping's.
    mapping.run(log, ODOMETRY_PARAMS, {**BACKEND_PARAMS, 'causal_trajectory_output': True},
                tmp_path / 'map')
    for name in ('odometry_tum.txt', 'trajectory_raw.tum', 'trajectory_optimized.tum',
                 'trajectory_causal.tum', 'map.pcd'):
        assert ((tmp_path / 'live' / name).read_bytes()
                == (tmp_path / 'map' / name).read_bytes()), name


def test_replay_writes_the_live_trajectory_and_timing(tmp_path: Path) -> None:
    log = tmp_path / 'log'
    _write_log(log)
    summary = live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'out',
                          realtime=True, speed=20.0)
    out = tmp_path / 'out'
    rows = [line for line in (out / 'trajectory_live.tum').read_text().split('\n') if line]
    assert len(rows) == summary['live_poses'] > 0
    timing = json.loads((out / 'live_timing.json').read_text())
    assert timing['realtime'] is True and timing['speed'] == 20.0
    assert timing['lag_seconds']['max'] >= timing['lag_seconds']['p50']
