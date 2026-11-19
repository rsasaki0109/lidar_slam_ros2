"""lidarslam long-term map store without ROS 2 (CI smoke test, milestone L2).

A live session on the static box-room log of test_python_odometry is added to a store. The
store's export of that one session must be, byte for byte, the map bundle the backend wrote for
it; the session must read back; rollback takes it off the map and leaves its files.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
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


@pytest.mark.parametrize('track_store', [False, True])
def test_store_startup_registration_and_island(tmp_path: Path, track_store: bool) -> None:
    log = tmp_path / 'log'
    _write_log(log)
    store_path = tmp_path / 'store'
    live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'base', store=store_path)
    store = Store.open(store_path)
    assert store.sessions[0]['submaps'] >= 2
    if not track_store:
        # L2 stores predate the window archive and must still support a fixed anchor.
        (store.session_dir('0000') / 'keyframe_windows.npz').unlink()

    aligned = live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'aligned',
                          store=store_path, initial_pose=[0.05, 0, 0, 0, 0, 0, 1],
                          track_store=track_store)
    assert aligned['store_anchor_stamp_sec'] is not None
    store = Store.open(store_path)
    assert 'store_from_session' in store.sessions[1]
    assert (tmp_path / 'aligned' / 'trajectory_store.tum').stat().st_size > 0
    tracked = store.session_dir('0001') / 'store_submap_poses.npy'
    assert tracked.is_file() == track_store
    if track_store:
        assert aligned['store_tracking_observations'] >= 2
        graph = (store.session_dir('0001') / 'store_tracking_graph.g2o').read_text()
        assert 'FIX 0' in graph and 'EDGE_SE3:QUAT' in graph
    manifest = (store_path / 'export' / 'map_bundle.yaml').read_text()
    assert f"submap_count: {sum(entry['submaps'] for entry in store.sessions)}" in manifest

    refused = live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'refused',
                          store=store_path, initial_pose=[50, 0, 0, 0, 0, 0, 1],
                          track_store=track_store)
    assert refused['store_anchor_stamp_sec'] is None
    store = Store.open(store_path)
    assert 'store_from_session' not in store.sessions[2]
    assert not (store.session_dir('0002') / 'store_submap_poses.npy').exists()
    assert not (tmp_path / 'refused' / 'trajectory_store.tum').exists()
    with pytest.raises(ValueError, match='own frames'):
        store.export()


def test_shared_surface_graph_reopened_for_followup(tmp_path: Path) -> None:
    from lidarslam.relocalization import StartupRelocalization

    log = tmp_path / 'log'
    _write_log(log)
    store_path = tmp_path / 'store'
    prior = [0.05, 0, 0, 0, 0, 0, 1]

    live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'base', store=store_path)
    second = live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'second',
                         store=store_path, initial_pose=prior, track_shared_surfaces=True)
    assert second['store_shared_surface_observations'] == 0
    third = live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'third',
                        store=store_path, initial_pose=prior, track_shared_surfaces=True)
    assert third['store_shared_surface_observations'] > 0

    store = Store.open(store_path)
    assert store.sessions[2]['shared_surface_observations'] == (
        third['store_shared_surface_observations'])
    directory = store.session_dir('0002')
    graph_path = directory / 'store_tracking_graph.g2o'
    graph = graph_path.read_text()
    assert 'FIX 0' in graph
    assert 'VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1' in graph
    assert sum(line.startswith('EDGE_SE3:QUAT 0 ') for line in graph.splitlines()) == (
        third['store_shared_surface_observations'])
    poses_path = directory / 'store_submap_poses.npy'
    poses = np.load(poses_path, allow_pickle=False)
    vertices = [line.split() for line in graph.splitlines() if line.startswith('VERTEX_SE3:QUAT')]
    np.testing.assert_allclose(np.asarray([v[2:5] for v in vertices[-len(poses):]], dtype=float),
                               poses[:, :3, 3], atol=1e-6)

    reopened = StartupRelocalization(store._aligned_sessions(), prior, track=True,
                                     shared_surfaces=True)
    np.testing.assert_array_equal(reopened.matcher.session_poses()[2], poses)
    fourth = live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'fourth',
                         store=store_path, initial_pose=prior, track_shared_surfaces=True)
    assert fourth['store_session'] == '0003'
    assert fourth['store_shared_surface_observations'] > 0
    assert (tmp_path / 'fourth' / 'trajectory_store.tum').stat().st_size > 0
    assert graph_path.read_text() == graph
    np.testing.assert_array_equal(np.load(poses_path, allow_pickle=False), poses)
    committed = Store.open(store_path).sessions
    manifest = (store_path / 'export' / 'map_bundle.yaml').read_text()
    assert f"submap_count: {sum(entry['submaps'] for entry in committed)}" in manifest


def test_change_evidence_removal_and_rollback(tmp_path: Path) -> None:
    from lidarslam.map_changes import EVIDENCE_FILE, removal_masks
    from lidarslam.relocalization import StartupRelocalization

    log = tmp_path / 'log'
    _write_log(log)
    store_path = tmp_path / 'store'
    prior = [0.05, 0, 0, 0, 0, 0, 1]
    live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'base',
                store=store_path, detect_changes=True)
    original = _bundle_files(store_path / 'export')
    store = Store.open(store_path)
    reference = store.session_dir('0000')
    native = StartupRelocalization(store._aligned_sessions(), prior).matcher
    evidence = core.MapChangeEvidence([str(reference)], native.session_poses())
    yz = np.arange(-.95, 1.05, .15) + .025
    rays = np.array([[7., y, z] for y in yz for z in yz], dtype='f4')
    for frame in range(10):
        evidence.observe([.75 * (frame % 2), 0., 0.], rays)
    rows = np.asarray(evidence.snapshot(), dtype=np.int64)
    assert np.any(rows[:, 8] == 2)
    assert np.all(rows[:, 6] <= 10)

    update = live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'update',
                         store=store_path, initial_pose=prior, detect_changes=True)
    store = Store.open(store_path)
    path = store.session_dir(update['store_session']) / EVIDENCE_FILE
    with np.load(path, allow_pickle=False) as saved:
        assert saved['references'].tolist() == ['0000']
        assert np.any(saved['rows'][:, 5] > 0)
    # Feed the through-wall scenario to the real persistence and export path.
    np.savez_compressed(path, references=['0000'], rows=rows)
    masks = removal_masks(store, store.sessions)
    assert any(masks[0].values())
    mask_export = tmp_path / 'filtered'
    core.export_store_bundle([str(reference)], store.backend_params('0000'), str(mask_export),
                             removed=masks[:1])
    assert (mask_export / 'map.pcd').stat().st_size < len(original['map.pcd'])
    store.export()

    assert store.rollback()['id'] == '0001'
    assert path.is_file()
    assert _bundle_files(store_path / 'export') == original


def test_new_point_filter_keeps_repeated_hits_and_rolls_back(tmp_path: Path) -> None:
    from lidarslam.map_changes import EVIDENCE_FILE, removal_masks

    log = tmp_path / 'log'
    _write_log(log)
    store_path = tmp_path / 'store'
    live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'base', store=store_path)
    original = _bundle_files(store_path / 'export')
    update = live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'update',
                         store=store_path, initial_pose=[0.05, 0, 0, 0, 0, 0, 1],
                         filter_new_points=True)
    store = Store.open(store_path)
    directory = store.session_dir('0001')
    archive = directory / EVIDENCE_FILE
    with np.load(archive, allow_pickle=False) as data:
        persisted = data['new_rows']
    assert update['store_suppressed_new_voxels'] == len(persisted) > 0
    assert any(removal_masks(store, store.sessions)[1].values())

    line = np.arange(-1, 1, .10) + .025
    repeated = np.array([[4., y, z] for y in line for z in line], dtype='f4')
    transient = np.array([[-4., y, z] for y in line for z in line], dtype='f4')
    evidence = core.NewPointEvidence()
    evidence.observe([0., 0., 0.], np.concatenate((repeated, transient)))
    empty = np.empty((0, 3), dtype='f4')
    evidence.observe([0., 0., 0.], empty)
    evidence.observe([0., 0., 0.], empty)
    all_unconfirmed = {tuple(row) for row in evidence.suppression_mask(
        str(directory), {0: np.eye(4)}, [0., 0., 0.])}
    for _ in range(2):
        evidence.observe([0., 0., 0.], repeated)
    remaining = {tuple(row) for row in evidence.suppression_mask(
        str(directory), {0: np.eye(4)}, [0., 0., 0.])}
    assert remaining < all_unconfirmed
    assert all(row[1] > 0 for row in all_unconfirmed - remaining)
    assert any(row[1] < 0 for row in remaining)
    through_wall = repeated * 1.75
    for _ in range(4):
        evidence.observe([0., 0., 0.], through_wall)
    cleared = {tuple(row) for row in evidence.suppression_mask(
        str(directory), {0: np.eye(4)}, [0., 0., 0.])}
    assert remaining < cleared
    assert store.rollback()['id'] == '0001'
    assert archive.is_file()
    assert _bundle_files(store_path / 'export') == original


def test_live_store_launch_area_localizes_without_initial_pose(tmp_path: Path) -> None:
    log = tmp_path / 'log'
    _write_log(log)
    path = tmp_path / 'store'
    live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'base', store=path)
    out = tmp_path / 'update'
    summary = live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, out, store=path,
                          start_near_first_submap=True, track_store=True)
    assert 0 <= summary['store_anchor_stamp_sec'] - summary['store_start_stamp_sec'] <= 10
    assert summary['store_tracking_observations'] > 0
    assert summary['mean_processing_seconds_per_scan'] > 0
    assert (out / 'trajectory_store.tum').is_file()


def test_refined_visibility_counts_free_space_before_first_hit(tmp_path: Path) -> None:
    from lidarslam.map_changes import EVIDENCE_FILE

    log = tmp_path / 'log'
    _write_log(log)
    path = tmp_path / 'store'
    live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'base', store=path)
    original = _bundle_files(path / 'export')
    summary = live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'update',
                          store=path, initial_pose=[.05, 0, 0, 0, 0, 0, 1],
                          refine_visibility=True)
    store = Store.open(path)
    directory = store.session_dir('0001')
    assert store.sessions[1]['map_change_refined_visibility']
    with np.load(directory / EVIDENCE_FILE, allow_pickle=False) as data:
        assert summary['store_suppressed_new_voxels'] == len(data['new_rows'])

    values = np.arange(-1, 1, .075) + .025
    wall = np.array([[4., y, z] for y in values for z in values], dtype='f4')
    clear = wall * 1.75
    origin = [0., 0., 0.]
    together = core.RefinedNewPointEvidence()
    hits_only = core.RefinedNewPointEvidence()
    for _ in range(4):
        together.observe(origin, clear)
    for _ in range(3):
        together.observe(origin, wall)
        hits_only.observe(origin, wall)
    arguments = (str(directory), {0: np.eye(4)}, origin)
    masked = {tuple(row) for row in together.suppression_mask(*arguments)}
    supported = {tuple(row) for row in hits_only.suppression_mask(*arguments)}
    assert supported < masked

    assert store.rollback()['id'] == '0001'
    assert _bundle_files(path / 'export') == original
