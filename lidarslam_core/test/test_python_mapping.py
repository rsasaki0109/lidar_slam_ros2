"""lidarslam map without ROS 2: odometry and backend in one pass (CI smoke test).

Uses the static box-room log of test_python_odometry. The odometry part must give the same
trajectory as ``lidarslam odometry`` alone, and every registered scan must reach the backend.
Byte identity with the ROS 2 path (offline node, recorded backend input, offline runner) is
checked on a real log (docs/roadmap/ros-free-core-2026-11.md), not here.
"""

from __future__ import annotations

from pathlib import Path

import pytest

lidarslam = pytest.importorskip('lidarslam')
core = pytest.importorskip('lidarslam._core')
if not (hasattr(core, 'OdometryDriver') and hasattr(core, 'BackendPipeline')):
    pytest.skip('lidarslam built without the frontend or the backend', allow_module_level=True)

from lidarslam import mapping, odometry  # noqa: E402
from test_python_odometry import _write_log  # noqa: E402

ODOMETRY_PARAMS = {
    'imu_topic': '/imu', 'lidar_topic': '/points', 'base_frame': 'base_link',
    'extrinsic_imu2base_quat_xyzw_xyz': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    'extrinsic_lidar2base_quat_xyzw_xyz': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    'voxel_size': 0.25, 'lidar_timestamps.force_relative': True,
}


def test_one_pass_map(tmp_path: Path) -> None:
    log = tmp_path / 'log'
    _write_log(log)
    alone = odometry.run(log, ODOMETRY_PARAMS, tmp_path / 'odometry')

    messages: list[tuple[str, str]] = []
    summary = mapping.run(
        log, ODOMETRY_PARAMS,
        {'registration_method': 'SMALL_GICP', 'ndt_num_threads': 1,
         'submap_distance_threshold': 0.0},
        tmp_path / 'map', log=lambda level, msg: messages.append((level, msg)))
    out = tmp_path / 'map'
    assert summary['frames'] == alone['poses'] > 0
    assert (out / 'odometry_tum.txt').read_bytes() == Path(alone['tum']).read_bytes()
    assert any(f'Offline run complete: {summary["frames"]} pairs' in msg for _, msg in messages)
    for name in ('trajectory_raw.tum', 'trajectory_optimized.tum', 'loop_edges.csv', 'map.pcd',
                 'map_projector_info.yaml', 'map_bundle.yaml', 'pose_graph.g2o',
                 'pointcloud_map/pointcloud_map_metadata.yaml'):
        assert (out / name).is_file(), name
    assert (out / 'map_projector_info.yaml').read_text() == 'projector_type: Local\n'
