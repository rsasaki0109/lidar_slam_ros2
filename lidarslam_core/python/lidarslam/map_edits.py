"""Reviewed edits of a long-term map store, by a person or an agent.

The automatic change detection (:mod:`lidarslam.map_changes`) removes old points and suppresses
new ones from their ray evidence; it is not always right (docs/roadmap/long-term-mapping-2026-11.md,
L5). This module puts a review step on top of it:

* :func:`candidates` groups what the evidence changed, and what the latest session added, into
  clusters of store-frame voxels (0.15 m, the voxels the change evidence uses): ``removed_old``
  (old points the evidence removed), ``suppressed_new`` (new points the evidence held back) and
  ``added`` (new points where the earlier sessions had none).
* An edit keeps or removes voxels: a whole candidate, a box, or explicit voxels. Edits are
  append-only files ``edits/<id>.json`` listed in ``store.json`` under ``edits``, each with its
  author (``human:<name>`` or ``agent:<name>``) and reason. The export folds them in order, the
  last edit of a voxel deciding: ``keep`` keeps the voxel's points even where the evidence
  removed them, ``remove`` drops them. :meth:`lidarslam.store.Store.rollback_edit` takes the
  latest edit off the list and leaves its file.

Edits address the store frame, so they hold across sessions; a voxel kept or removed stays so
when a later session observes it again, until a later edit says otherwise.
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from lidarslam import _core

FORMAT = 'lidarslam-map-edit'
VERSION = 1
VOXEL_M = 0.15  # graph_based_slam/map_change_evidence.hpp
OPERATIONS = ('remove', 'keep')
# PointState of graph_based_slam/map_store.hpp
KEPT, REMOVED_BY_EVIDENCE, REMOVED_BY_EDIT, KEPT_BY_EDIT = 0, 1, 2, 3
_NEIGHBOURS = np.array([(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                        for dz in (-1, 0, 1) if (dx, dy, dz) > (0, 0, 0)], dtype=np.int64)


def _keys(voxels: np.ndarray) -> np.ndarray:
    """One int64 per voxel (21 bits per axis, offset to be non-negative)."""
    v = np.asarray(voxels, dtype=np.int64).reshape(-1, 3) + (1 << 20)
    if np.any((v < 0) | (v >= (1 << 21))):
        raise ValueError('a voxel index is out of range')
    return (v[:, 0] << 42) | (v[:, 1] << 21) | v[:, 2]


def _voxels(keys: np.ndarray) -> np.ndarray:
    keys = np.asarray(keys, dtype=np.int64)
    mask = (1 << 21) - 1
    return np.stack([(keys >> 42) & mask, (keys >> 21) & mask, keys & mask], axis=1) - (1 << 20)


def _clusters(keys: np.ndarray) -> list[np.ndarray]:
    """Connected groups (26-neighbourhood) of unique voxel keys, largest first."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

    keys = np.unique(keys)
    if keys.size == 0:
        return []
    voxels = _voxels(keys)
    rows, cols = [], []
    for offset in _NEIGHBOURS:
        neighbour = _keys(voxels + offset)
        at = np.searchsorted(keys, neighbour)
        at[at >= keys.size] = 0
        hit = keys[at] == neighbour
        rows.append(np.nonzero(hit)[0])
        cols.append(at[hit])
    rows, cols = np.concatenate(rows), np.concatenate(cols)
    graph = coo_matrix((np.ones(rows.size, dtype=np.int8), (rows, cols)),
                       shape=(keys.size, keys.size))
    count, labels = connected_components(graph, directed=False)
    order = np.argsort(labels, kind='stable')
    groups = np.split(keys[order], np.cumsum(np.bincount(labels, minlength=count))[:-1])
    return sorted(groups, key=lambda g: (-g.size, int(g[0])))


def _object_sized(cluster: np.ndarray, max_extent_m: float, tile_m: float) -> list[np.ndarray]:
    """A cluster wider than max_extent_m split into the connected parts of each tile_m floor
    tile (a whole room's floor is one cluster; a reviewer judges objects)."""
    voxels = _voxels(cluster)
    extent = (voxels[:, :2].max(axis=0) - voxels[:, :2].min(axis=0) + 1) * VOXEL_M
    if max_extent_m <= 0 or np.all(extent <= max_extent_m):
        return [cluster]
    tiles = np.floor(voxels[:, :2] * VOXEL_M / tile_m).astype(np.int64)
    order = np.lexsort((tiles[:, 1], tiles[:, 0]))
    tiles, part = tiles[order], cluster[order]
    starts = np.flatnonzero(np.r_[True, np.any(tiles[1:] != tiles[:-1], axis=1)])
    pieces = []
    for piece in np.split(part, starts[1:]):
        pieces.extend(_clusters(np.sort(piece)))
    return sorted(pieces, key=lambda g: (-g.size, int(g[0])))


def candidate_id(kind: str, keys: np.ndarray) -> str:
    """A stable id: the kind and a digest of the cluster's voxels."""
    digest = hashlib.sha1(np.sort(np.asarray(keys, dtype='<i8')).tobytes()).hexdigest()[:10]
    return f'{kind}-{digest}'


def _summary(kind: str, keys: np.ndarray, xyz: np.ndarray, sessions: np.ndarray,
             session_ids: list[str]) -> dict[str, Any]:
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    return {
        'id': candidate_id(kind, keys), 'kind': kind, 'voxels': int(keys.size),
        'points': int(len(xyz)), 'centroid': [round(float(v), 3) for v in xyz.mean(axis=0)],
        'min': [round(float(v), 3) for v in lo], 'max': [round(float(v), 3) for v in hi],
        'sessions': sorted({session_ids[int(s)] for s in np.unique(sessions)}),
    }


def candidates(store: Any, min_voxels: int = 5, points: dict[str, np.ndarray] | None = None,
               max_extent_m: float = 3.0, tile_m: float = 2.0) -> list[dict[str, Any]]:
    """Clusters of what to review, largest first within each kind; each with its ``id``, ``kind``,
    voxel and point counts, centroid and bounds (store frame, metres) and its sessions. The
    voxels of each candidate are under the key ``voxel_keys`` (see :func:`resolve`). A cluster
    wider than ``max_extent_m`` is split into the connected parts of ``tile_m`` floor tiles."""
    sessions = store.sessions
    if not sessions:
        return []
    points = points if points is not None else store.points()
    session_ids = [entry['id'] for entry in sessions]
    latest = len(sessions) - 1
    keys = _keys(points['voxel'])
    state, session = points['state'], points['session']
    groups: list[tuple[str, np.ndarray]] = []
    removed = state == REMOVED_BY_EVIDENCE
    groups.append(('removed_old', removed & (session < latest)))
    groups.append(('suppressed_new', removed & (session == latest)))
    if latest > 0:
        visible = (state == KEPT) | (state == KEPT_BY_EDIT)
        earlier = np.unique(keys[visible & (session < latest)])
        # A voxel next to an earlier one is the same surface seen again, not new structure.
        near = np.zeros(keys.size, dtype=bool)
        if earlier.size:
            voxels = points['voxel'].astype(np.int64)
            for offset in np.vstack([np.zeros((1, 3), dtype=np.int64), _NEIGHBOURS,
                                     -_NEIGHBOURS]):
                shifted = _keys(voxels + offset)
                at = np.searchsorted(earlier, shifted)
                at[at >= earlier.size] = 0
                near |= earlier[at] == shifted
        groups.append(('added', (state == KEPT) & (session == latest) & ~near))
    out = []
    for kind, select in groups:
        if not np.any(select):
            continue
        selected_keys = keys[select]
        order = np.argsort(selected_keys, kind='stable')
        sorted_keys = selected_keys[order]
        xyz = points['xyz'][select][order]
        sess = session[select][order]
        pieces = [piece for cluster in _clusters(sorted_keys)
                  for piece in _object_sized(cluster, max_extent_m, tile_m)]
        for cluster in sorted(pieces, key=lambda g: (-g.size, int(g[0]))):
            if cluster.size < min_voxels:
                continue
            lo = np.searchsorted(sorted_keys, cluster, side='left')
            hi = np.searchsorted(sorted_keys, cluster, side='right')
            idx = np.concatenate([np.arange(a, b) for a, b in zip(lo, hi)])
            item = _summary(kind, cluster, xyz[idx], sess[idx], session_ids)
            item['voxel_keys'] = cluster
            out.append(item)
    return out


def box_voxels(points: dict[str, np.ndarray], lo: Iterable[float], hi: Iterable[float]
               ) -> np.ndarray:
    """The voxels of the stored points inside the axis-aligned box [lo, hi] (store frame, m)."""
    lo, hi = np.asarray(list(lo), dtype=float), np.asarray(list(hi), dtype=float)
    if lo.shape != (3,) or hi.shape != (3,) or np.any(hi < lo):
        raise ValueError('a box is min x y z and max x y z with max >= min')
    inside = np.all((points['xyz'] >= lo) & (points['xyz'] <= hi), axis=1)
    return np.unique(points['voxel'][inside], axis=0)


def make_edit(author: str, reason: str, operations: list[dict[str, Any]]) -> dict[str, Any]:
    """An edit document; each operation is ``{'op': 'remove' | 'keep', 'voxels': [[i, j, k], ...]}``
    with optional ``candidate`` and ``note``."""
    edit = {'format': FORMAT, 'version': VERSION, 'author': author, 'reason': reason,
            'created': datetime.now(timezone.utc).isoformat(timespec='seconds'),
            'operations': operations}
    validate(edit)
    return edit


def validate(edit: dict[str, Any]) -> None:
    """Raises ValueError unless edit is a well-formed edit document."""
    if edit.get('format') != FORMAT or edit.get('version') != VERSION:
        raise ValueError(f'not a {FORMAT} version {VERSION}')
    author = edit.get('author')
    if not isinstance(author, str) or not author.split(':', 1)[0] in ('human', 'agent') \
            or ':' not in author or not author.split(':', 1)[1]:
        raise ValueError("author must be 'human:<name>' or 'agent:<name>'")
    if not isinstance(edit.get('reason'), str) or not edit['reason'].strip():
        raise ValueError('an edit needs a reason')
    operations = edit.get('operations')
    if not isinstance(operations, list) or not operations:
        raise ValueError('an edit needs at least one operation')
    for op in operations:
        if not isinstance(op, dict) or op.get('op') not in OPERATIONS:
            raise ValueError(f"an operation's op is one of {OPERATIONS}")
        voxels = op.get('voxels')
        if (not isinstance(voxels, list) or not voxels or not all(
                isinstance(v, list) and len(v) == 3 and all(
                    isinstance(c, int) and not isinstance(c, bool) for c in v)
                for v in voxels)):
            raise ValueError('an operation needs voxels as [[i, j, k], ...] integers')
        _keys(np.asarray(voxels))  # range check


def fold(edits: Iterable[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray]:
    """The final (remove, keep) voxels of edits applied in order; a voxel's last edit decides."""
    decision: dict[int, str] = {}
    for edit in edits:
        for op in edit['operations']:
            for key in _keys(np.asarray(op['voxels'])).tolist():
                decision[key] = op['op']
    remove = np.array(sorted(k for k, v in decision.items() if v == 'remove'), dtype=np.int64)
    keep = np.array(sorted(k for k, v in decision.items() if v == 'keep'), dtype=np.int64)
    return _voxels(remove), _voxels(keep)


def read(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding='utf-8') as handle:
        edit = json.load(handle)
    validate(edit)
    return edit


def resolve(store: Any, remove: Iterable[str] = (), keep: Iterable[str] = (),
            remove_boxes: Iterable[tuple] = (), keep_boxes: Iterable[tuple] = (),
            min_voxels: int = 5) -> list[dict[str, Any]]:
    """Operations for candidate ids and boxes ``(min_xyz, max_xyz)`` against the store's current
    state. Unknown candidate ids raise KeyError (the store changed, or the id is wrong)."""
    remove, keep = list(remove), list(keep)
    points = store.points()
    operations = []
    if remove or keep:
        by_id = {c['id']: c for c in candidates(store, min_voxels, points)}
        for name, op in [(i, 'remove') for i in remove] + [(i, 'keep') for i in keep]:
            if name not in by_id:
                raise KeyError(f'no candidate {name} in the store as it is now')
            operations.append({'op': op, 'candidate': name,
                               'voxels': _voxels(by_id[name]['voxel_keys']).tolist()})
    for boxes, op in ((remove_boxes, 'remove'), (keep_boxes, 'keep')):
        for lo, hi in boxes:
            voxels = box_voxels(points, lo, hi)
            if len(voxels):
                operations.append({'op': op, 'box': [list(map(float, lo)), list(map(float, hi))],
                                   'voxels': voxels.tolist()})
    return operations


PACK_MAGIC = b'LSREVIEW'
PACK_VERSION = 1


def review_pack(store: Any, path: str | Path, min_voxels: int = 5) -> dict[str, Any]:
    """Writes the store's review pack for the map editor (docs/map-editor/) and returns its header.

    Layout (little-endian): ``LSREVIEW``, u32 version, u32 header length, the header (JSON:
    frame, voxel size, sessions, edits, candidates without their voxels), u32 rows, then the
    arrays of the rows: voxel (int32 x 3), state (uint8), session (uint8), candidate (int32,
    -1 for none) and points (uint16, clipped). A row is one voxel, state and session; its
    candidate is the one whose kind matches the row.
    """
    sessions = store.sessions
    points = store.points()
    items = candidates(store, min_voxels, points)
    if len(sessions) > 255:
        raise ValueError('a review pack holds at most 255 sessions')
    columns = np.stack([_keys(points['voxel']), points['state'].astype(np.int64),
                        points['session'].astype(np.int64)], axis=1)
    unique, counts = np.unique(columns, axis=0, return_counts=True)
    row_voxel_keys = unique[:, 0]
    state = unique[:, 1].astype(np.uint8)
    session = unique[:, 2].astype(np.uint8)
    latest = len(sessions) - 1
    kind_of_row = np.full(len(unique), '', dtype=object)
    kind_of_row[(state == REMOVED_BY_EVIDENCE) & (session < latest)] = 'removed_old'
    kind_of_row[(state == REMOVED_BY_EVIDENCE) & (session == latest)] = 'suppressed_new'
    kind_of_row[(state == KEPT) & (session == latest)] = 'added'
    candidate = np.full(len(unique), -1, dtype=np.int32)
    for index, item in enumerate(items):
        rows = np.isin(row_voxel_keys, item['voxel_keys']) & (kind_of_row == item['kind'])
        candidate[rows] = index
    header = {
        'format': 'lidarslam-map-review', 'version': PACK_VERSION, 'voxel_m': VOXEL_M,
        'frame': store.frame, 'store': Path(store.path).name,
        'sessions': [entry['id'] for entry in sessions],
        'edits': [entry['id'] for entry in store.edits],
        'candidates': [{k: v for k, v in item.items() if k != 'voxel_keys'} for item in items],
    }
    encoded = json.dumps(header).encode('utf-8')
    with Path(path).open('wb') as handle:
        handle.write(PACK_MAGIC)
        handle.write(np.array([PACK_VERSION, len(encoded)], dtype='<u4').tobytes())
        handle.write(encoded)
        handle.write(np.array([len(unique)], dtype='<u4').tobytes())
        handle.write(_voxels(row_voxel_keys).astype('<i4').tobytes())
        handle.write(state.tobytes())
        handle.write(session.tobytes())
        handle.write(candidate.astype('<i4').tobytes())
        handle.write(np.minimum(counts, 65535).astype('<u2').tobytes())
    return header


def read_review_pack(path: str | Path) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    """The header and the rows of a review pack (the inverse of :func:`review_pack`)."""
    data = Path(path).read_bytes()
    if data[:8] != PACK_MAGIC:
        raise ValueError(f'{path}: not a review pack')
    version, length = np.frombuffer(data[8:16], dtype='<u4')
    if version != PACK_VERSION:
        raise ValueError(f'{path}: review pack version {version}')
    header = json.loads(data[16:16 + length])
    at = 16 + int(length)
    (rows,) = np.frombuffer(data[at:at + 4], dtype='<u4')
    at += 4
    out = {}
    for name, dtype, width in (('voxel', '<i4', 3), ('state', 'u1', 1), ('session', 'u1', 1),
                               ('candidate', '<i4', 1), ('points', '<u2', 1)):
        size = int(rows) * width * np.dtype(dtype).itemsize
        array = np.frombuffer(data[at:at + size], dtype=dtype)
        out[name] = array.reshape(-1, 3) if width == 3 else array
        at += size
    if at != len(data):
        raise ValueError(f'{path}: review pack has {len(data) - at} trailing bytes')
    return header, out
