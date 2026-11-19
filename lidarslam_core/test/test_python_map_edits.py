"""Reviewed edits of a long-term map store (lidarslam.map_edits; CI smoke test).

A two-session store on the static box-room log of test_python_odometry gets change evidence
that removes part of the first session (the through-wall scenario of test_python_store). The
removal must show up as a review candidate; keeping it must give the map without the removal;
a box removal must drop points; rolling back the edits must give the earlier exports back, byte
for byte; and a malformed edit must be refused.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

lidarslam = pytest.importorskip('lidarslam')
core = pytest.importorskip('lidarslam._core')
if not hasattr(core, 'store_points'):
    pytest.skip('lidarslam built without map store edits', allow_module_level=True)

from lidarslam import live, map_edits  # noqa: E402
from lidarslam.__main__ import main as cli  # noqa: E402
from lidarslam.map_changes import EVIDENCE_FILE  # noqa: E402
from lidarslam.relocalization import StartupRelocalization  # noqa: E402
from lidarslam.store import Store  # noqa: E402
from test_python_mapping import ODOMETRY_PARAMS  # noqa: E402
from test_python_odometry import _write_log  # noqa: E402
from test_python_store import BACKEND_PARAMS  # noqa: E402


def _bundle_files(directory: Path) -> dict[str, bytes]:
    return {str(p.relative_to(directory)): p.read_bytes()
            for p in sorted(directory.rglob('*')) if p.is_file()}


@pytest.fixture()
def store(tmp_path: Path) -> Store:
    log = tmp_path / 'log'
    _write_log(log)
    path = tmp_path / 'store'
    prior = [0.05, 0, 0, 0, 0, 0, 1]
    live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'base', store=path,
                detect_changes=True)
    base = Store.open(path)
    native = StartupRelocalization(base._aligned_sessions(), prior).matcher
    evidence = core.MapChangeEvidence([str(base.session_dir('0000'))], native.session_poses())
    yz = np.arange(-.95, 1.05, .15) + .025
    rays = np.array([[7., y, z] for y in yz for z in yz], dtype='f4')
    for frame in range(10):
        evidence.observe([.75 * (frame % 2), 0., 0.], rays)
    rows = np.asarray(evidence.snapshot(), dtype=np.int64)
    update = live.replay(log, ODOMETRY_PARAMS, BACKEND_PARAMS, tmp_path / 'update', store=path,
                         initial_pose=prior, detect_changes=True)
    updated = Store.open(path)
    np.savez_compressed(updated.session_dir(update['store_session']) / EVIDENCE_FILE,
                        references=['0000'], rows=rows)
    updated.export()
    return Store.open(path)


def test_candidates_keep_remove_and_rollback(store: Store, tmp_path: Path) -> None:
    points = store.points()
    assert points['xyz'].shape[0] == points['voxel'].shape[0] == points['state'].size > 0
    assert np.any(points['state'] == map_edits.REMOVED_BY_EVIDENCE)
    assert not np.any(points['state'] >= map_edits.REMOVED_BY_EDIT)

    items = map_edits.candidates(store, min_voxels=1)
    removed = [c for c in items if c['kind'] == 'removed_old']
    assert removed and all(c['sessions'] == ['0000'] for c in removed)
    assert sum(c['points'] for c in removed) == int(
        np.sum(points['state'] == map_edits.REMOVED_BY_EVIDENCE))
    # Ids are stable for the same store state.
    assert [c['id'] for c in map_edits.candidates(store, min_voxels=1)] == [c['id'] for c in items]

    evidence_export = _bundle_files(store.path / 'export')
    # The same sessions with no removal at all.
    dirs = [str(store.session_dir(e['id'])) for e in store.sessions]
    plain = tmp_path / 'plain'
    transforms = [np.eye(4).tolist(), store.sessions[1]['store_from_session']]
    core.export_store_bundle(dirs, store.backend_params('0000'), str(plain), transforms=transforms)

    # Keeping every removal candidate is the map without the removal.
    operations = map_edits.resolve(store, keep=[c['id'] for c in removed], min_voxels=1)
    entry = store.add_edit(map_edits.make_edit('agent:test', 'the wall is still there', operations))
    assert entry['id'] == '0000' and store.edits[0]['author'] == 'agent:test'
    after_keep = store.points()
    assert not np.any(after_keep['state'] == map_edits.REMOVED_BY_EVIDENCE)
    assert np.sum(after_keep['state'] == map_edits.KEPT_BY_EDIT) == np.sum(
        points['state'] == map_edits.REMOVED_BY_EVIDENCE)
    assert (_bundle_files(store.path / 'export')['map.pcd'] ==
            _bundle_files(plain)['map.pcd'])
    keep_export = _bundle_files(store.path / 'export')

    # A person removes a box of noise through the command line.
    lo = points['xyz'].min(axis=0)
    hi = lo + 0.5
    assert cli(['store', 'edit', str(store.path), '--author', 'human:tester',
                '--reason', 'noise in a corner', '--remove-box', *map(str, [*lo, *hi])]) == 0
    store = Store.open(store.path)
    after_box = store.points()
    assert np.any(after_box['state'] == map_edits.REMOVED_BY_EDIT)
    assert (store.path / 'export' / 'map.pcd').stat().st_size < len(keep_export['map.pcd'])
    document = store.edit_document('0001')
    assert document['author'] == 'human:tester' and document['operations'][0]['op'] == 'remove'
    assert document['sessions'] == ['0000', '0001']

    # Rolling the edits back gives each earlier export back, byte for byte.
    assert store.rollback_edit()['id'] == '0001'
    assert _bundle_files(store.path / 'export') == keep_export
    assert store.rollback_edit()['id'] == '0000'
    assert _bundle_files(store.path / 'export') == evidence_export
    assert (store.path / 'edits' / '0000.json').is_file()  # files stay
    with pytest.raises(ValueError):
        store.rollback_edit()


def test_apply_edit_file_and_refusals(store: Store, tmp_path: Path) -> None:
    voxel = store.points()['voxel'][0].tolist()
    edit = map_edits.make_edit('human:editor', 'from the map editor',
                               [{'op': 'remove', 'voxels': [voxel]}])
    path = tmp_path / 'edit.json'
    path.write_text(json.dumps(edit))
    assert cli(['store', 'apply-edit', str(store.path), '--file', str(path)]) == 0
    assert Store.open(store.path).edits[0]['voxels'] == 1

    bad = [
        {**edit, 'author': 'robot'},
        {**edit, 'reason': ' '},
        {**edit, 'operations': []},
        {**edit, 'operations': [{'op': 'erase', 'voxels': [voxel]}]},
        {**edit, 'operations': [{'op': 'keep', 'voxels': [[1, 2]]}]},
        {**edit, 'operations': [{'op': 'keep', 'voxels': [[1, 2, 1 << 30]]}]},
        {**edit, 'format': 'something-else'},
    ]
    for document in bad:
        with pytest.raises(ValueError):
            store.add_edit(document)
    with pytest.raises(KeyError):
        map_edits.resolve(store, keep=['removed_old-0000000000'])


def test_fold_last_edit_wins() -> None:
    first = {'operations': [{'op': 'remove', 'voxels': [[0, 0, 0], [1, 0, 0]]}]}
    second = {'operations': [{'op': 'keep', 'voxels': [[1, 0, 0]]}]}
    remove, keep = map_edits.fold([first, second])
    assert remove.tolist() == [[0, 0, 0]] and keep.tolist() == [[1, 0, 0]]
    remove, keep = map_edits.fold([second, first])
    assert remove.tolist() == [[0, 0, 0], [1, 0, 0]] and keep.tolist() == []


def test_review_pack_round_trip_and_stale_edit(store: Store, tmp_path: Path) -> None:
    path = tmp_path / 'map.lsreview'
    assert cli(['store', 'review-pack', str(store.path), '--out', str(path),
                '--min-voxels', '1']) == 0
    header, rows = map_edits.read_review_pack(path)
    assert header['sessions'] == ['0000', '0001'] and header['edits'] == []
    points = store.points()
    assert 0 < int(rows['points'].astype(np.int64).sum()) <= points['state'].size
    # Each candidate's rows hold exactly its voxels.
    for i, item in enumerate(header['candidates']):
        assert len(np.unique(rows['voxel'][rows['candidate'] == i], axis=0)) == item['voxels']
    # An edit made on this pack, as the map editor writes it (basis = the pack's store state).
    first = header['candidates'][0]
    voxels = np.unique(rows['voxel'][rows['candidate'] == 0], axis=0).tolist()
    edit = {**map_edits.make_edit('human:editor', 'reviewed', [{'op': 'keep', 'voxels': voxels}]),
            'basis': {'sessions': header['sessions'], 'edits': header['edits']},
            'decisions': [{'candidate': first['id'], 'op': 'keep'}]}
    store.add_edit(edit)
    with pytest.raises(ValueError, match='review a new pack'):
        store.add_edit(edit)  # the store has changed since the pack was written
    store.add_edit(edit, force=True)
    assert [e['id'] for e in store.edits] == ['0000', '0001']
