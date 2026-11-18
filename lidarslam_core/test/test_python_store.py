"""lidarslam long-term map store without ROS 2 (CI smoke test, milestone L2).

A live session on the static box-room log of test_python_odometry is added to a store. The
store's export of that one session must be, byte for byte, the map bundle the backend wrote for
it; the session must read back; rollback takes it off the map and leaves its files.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

lidarslam = pytest.importorskip('lidarslam')
core = pytest.importorskip('lidarslam._core')
if not (hasattr(core, 'OdometryDriver') and hasattr(core, 'BackendPipeline')):
    pytest.skip('lidarslam built without the frontend or the backend', allow_module_level=True)
if not hasattr(core, 'export_store_bundle'):
    pytest.skip('lidarslam built without the map store', allow_module_level=True)

from lidarslam import live  # noqa: E402
from lidarslam.store import Store  # noqa: E402
from test_python_mapping import ODOMETRY_PARAMS  # noqa: E402
from test_python_odometry import _write_log  # noqa: E402

BACKEND_PARAMS = {'registration_method': 'SMALL_GICP', 'ndt_num_threads': 1,
                  'submap_distance_threshold': 0.0}


def _bundle_files(directory: Path) -> dict[str, bytes]:
    names = ['map.pcd', 'map_projector_info.yaml', 'map_bundle.yaml', 'pose_graph.g2o']
    files = {name: (directory / name).read_bytes() for name in names}
    for path in sorted((directory / 'pointcloud_map').iterdir()):
        files[f'pointcloud_map/{path.name}'] = path.read_bytes()
    return files


@pytest.mark.parametrize('points_per_pass', [None, 0])
def test_store_export_is_the_session_bundle(tmp_path: Path, points_per_pass: int | None) -> None:
    log = tmp_path / 'log'
    _write_log(log)
    params = dict(BACKEND_PARAMS)
    if points_per_pass is not None:
        params['map_save_points_per_pass'] = points_per_pass  # the assembled-map path
    summary = live.replay(log, ODOMETRY_PARAMS, params, tmp_path / 'live',
                          store=tmp_path / 'store')
    assert summary['store_session'] == '0000'

    store = Store.open(tmp_path / 'store')
    assert [entry['id'] for entry in store.sessions] == ['0000']
    entry = store.sessions[0]
    assert entry['submaps'] >= 1 and entry['live_poses'] == summary['live_poses']
    session = store.session_dir('0000')
    for name in ('submaps.bin', 'pose_graph.g2o', 'backend_params.json',
                 'odometry_params.json', 'session.json'):
        assert (session / name).is_file(), name
    assert store.backend_params('0000')['causal_trajectory_output'] is True

    # The store's export (written when the session was added) is the backend's bundle.
    backend_bundle = _bundle_files(tmp_path / 'live')
    assert _bundle_files(tmp_path / 'store' / 'export') == backend_bundle
    # And so is an export from the files alone, in a fresh process state.
    again = Store.open(tmp_path / 'store').export(tmp_path / 'again')
    assert _bundle_files(again) == backend_bundle


def test_store_rollback_and_refusals(tmp_path: Path) -> None:
    log = tmp_path / 'log'
    _write_log(log)
    live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'a', store=tmp_path / 'store')
    live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'b', store=tmp_path / 'store')
    store = Store.open(tmp_path / 'store')
    assert [entry['id'] for entry in store.sessions] == ['0000', '0001']
    # Two sessions are not in one frame before relocalization (L3).
    with pytest.raises(ValueError):
        store.export(tmp_path / 'stacked')
    store.export(tmp_path / 'stacked', stack_sessions=True)
    manifest = (tmp_path / 'stacked' / 'map_bundle.yaml').read_text()
    assert f"submap_count: {2 * store.sessions[0]['submaps']}" in manifest
    assert not (tmp_path / 'stacked' / 'pose_graph.g2o').exists()

    removed = store.rollback()
    assert removed['id'] == '0001'
    assert [entry['id'] for entry in Store.open(tmp_path / 'store').sessions] == ['0000']
    assert (store.session_dir('0001') / 'submaps.bin').is_file()  # files stay
    index = json.loads((tmp_path / 'store' / 'store.json').read_text())
    assert index['format'] == 'lidarslam-map-store' and index['version'] == 1

    with pytest.raises(FileExistsError):
        Store.create(tmp_path / 'store')
    with pytest.raises(ValueError):
        store.commit('0000')


def test_corrupt_session_is_refused(tmp_path: Path) -> None:
    log = tmp_path / 'log'
    _write_log(log)
    live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'a', store=tmp_path / 'store')
    store = Store.open(tmp_path / 'store')
    submaps = store.session_dir('0000') / 'submaps.bin'
    submaps.write_bytes(submaps.read_bytes()[:-3])
    with pytest.raises(RuntimeError, match='ends early|trailing'):
        store.export(tmp_path / 'out')
