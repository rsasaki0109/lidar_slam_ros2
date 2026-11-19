"""A long-term map store: the mapping sessions of one place, the Autoware map bundle an export.

docs/roadmap/long-term-mapping-2026-11.md (milestone L2). A store is a directory::

    store.json        format, version, frame and the committed sessions, in order
    sessions/<id>/    one session: submaps.bin (the backend's submaps: stamps, odometry and
                      optimized poses, compressed scans), pose_graph.g2o, the parameters it was
                      mapped with (odometry_params.json, backend_params.json), session.json
    export/           the Autoware map bundle exported from the committed sessions

Sessions are append-only. A session's directory is written first; the session is part of the
map only once ``store.json`` lists it, and ``store.json`` is replaced in one rename, so a crash
leaves either the old or the new list. :meth:`Store.rollback` takes the latest session off the
list and leaves every file as it was. The bundle in ``export/`` is rebuilt from the committed
sessions (:meth:`Store.export`); for a store of one session it is, byte for byte, the bundle the
backend wrote for that session.

A live session (:class:`lidarslam.live.Session` with ``store=``) adds itself when it finishes.
Sessions anchored to the primary map record ``store_from_session``. Other sessions keep their
own frame. :meth:`Store.export` requires a common frame by default.
When present, ``store_submap_poses.npy`` holds the session's absolute tracked submap poses in
``submaps.bin`` order, which take precedence over the rigid transform for export.

Reviewed edits (:mod:`lidarslam.map_edits`, by a person or an agent) are ``edits/<id>.json``,
listed in ``store.json`` under ``edits`` in the order they apply. The export applies them after
the change evidence; :meth:`Store.rollback_edit` takes the latest one off the list.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
from typing import Any, Callable

import numpy as np

from lidarslam import _core

FORMAT = 'lidarslam-map-store'
VERSION = 1
_INDEX = 'store.json'


def _write_json(path: Path, value: Any) -> None:
    """Writes path through a temporary file and one rename."""
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w', encoding='utf-8') as handle:
        json.dump(value, handle, indent=2, sort_keys=False)
        handle.write('\n')
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def _jsonable(params: dict[str, Any]) -> dict[str, Any]:
    out = {}
    for key, value in params.items():
        if isinstance(value, bytes):
            raise TypeError(f'parameter {key!r}: a bytes value cannot be stored')
        out[key] = list(value) if isinstance(value, tuple) else value
    return out


class Store:
    """A map store directory; :meth:`create` a new one or :meth:`open` an existing one."""

    def __init__(self, path: str | Path, index: dict[str, Any]):
        self.path = Path(path)
        self._index = index

    @classmethod
    def create(cls, path: str | Path, frame: str = 'map') -> Store:
        """A new, empty store at path (a missing or empty directory)."""
        path = Path(path)
        if path.exists() and any(path.iterdir()):
            raise FileExistsError(f'{path}: not empty; open it with Store.open')
        (path / 'sessions').mkdir(parents=True, exist_ok=True)
        index = {'format': FORMAT, 'version': VERSION, 'frame': frame, 'sessions': []}
        _write_json(path / _INDEX, index)
        return cls(path, index)

    @classmethod
    def open(cls, path: str | Path) -> Store:
        path = Path(path)
        with (path / _INDEX).open(encoding='utf-8') as handle:
            index = json.load(handle)
        if index.get('format') != FORMAT:
            raise ValueError(f'{path}: not a {FORMAT}')
        if index.get('version') != VERSION:
            raise ValueError(f'{path}: store version {index.get("version")} (this reads '
                             f'{VERSION})')
        return cls(path, index)

    @classmethod
    def open_or_create(cls, path: str | Path, frame: str = 'map') -> Store:
        path = Path(path)
        if (path / _INDEX).exists():
            return cls.open(path)
        return cls.create(path, frame)

    @property
    def frame(self) -> str:
        return self._index['frame']

    @property
    def sessions(self) -> list[dict[str, Any]]:
        """The committed sessions, oldest first (copies of their store.json entries)."""
        return [dict(entry) for entry in self._index['sessions']]

    def session_dir(self, session_id: str) -> Path:
        return self.path / 'sessions' / session_id

    def new_session(self) -> tuple[str, Path]:
        """An id and an empty directory for a new session (not yet part of the map)."""
        existing = {p.name for p in (self.path / 'sessions').iterdir()}
        number = len(existing)
        while f'{number:04d}' in existing:
            number += 1
        session_id = f'{number:04d}'
        directory = self.session_dir(session_id)
        directory.mkdir()
        return session_id, directory

    def commit(self, session_id: str, info: dict[str, Any] | None = None) -> dict[str, Any]:
        """Lists a written session in store.json: from now on it is part of the map."""
        directory = self.session_dir(session_id)
        if not (directory / 'submaps.bin').is_file():
            raise FileNotFoundError(f'{directory}: no submaps.bin')
        if any(entry['id'] == session_id for entry in self._index['sessions']):
            raise ValueError(f'session {session_id} is already committed')
        entry = {'id': session_id,
                 'committed': datetime.now(timezone.utc).isoformat(timespec='seconds')}
        entry.update(info or {})
        index = dict(self._index)
        index['sessions'] = [*self._index['sessions'], entry]
        _write_json(self.path / _INDEX, index)
        self._index = index
        return dict(entry)

    def rollback(self) -> dict[str, Any]:
        """Take the latest session off the map and rebuild an evidence-aware export."""
        if not self._index['sessions']:
            raise ValueError('the store has no session')
        index = dict(self._index)
        index['sessions'] = self._index['sessions'][:-1]
        _write_json(self.path / _INDEX, index)
        removed = self._index['sessions'][-1]
        rebuild = (any(e.get('map_change_evidence') for e in self._index['sessions'])
                   or bool(self._index.get('edits')))
        self._index = index
        if rebuild and index['sessions'] and len(self._aligned_sessions()) == len(index['sessions']):
            self.export()
        return dict(removed)

    @property
    def edits(self) -> list[dict[str, Any]]:
        """The applied edits, oldest first (copies of their store.json entries)."""
        return [dict(entry) for entry in self._index.get('edits', [])]

    def edit_document(self, edit_id: str) -> dict[str, Any]:
        from lidarslam import map_edits
        return map_edits.read(self.path / 'edits' / f'{edit_id}.json')

    def add_edit(self, edit: dict[str, Any], log: Callable[[str, str], None] | None = None,
                 force: bool = False) -> dict[str, Any]:
        """Writes a reviewed edit (:func:`lidarslam.map_edits.make_edit`), lists it in store.json
        and rebuilds the export when the sessions share a frame. Returns its entry.

        An edit made on a review pack records the store state it saw (``basis``: its sessions
        and edits); it is refused when the store has changed since, unless ``force``."""
        from lidarslam import map_edits
        map_edits.validate(edit)
        if not self._index['sessions']:
            raise ValueError('the store has no session to edit')
        basis = edit.get('basis')
        if basis is not None and not force:
            now = {'sessions': [e['id'] for e in self._index['sessions']],
                   'edits': [e['id'] for e in self._index.get('edits', [])]}
            if basis != now:
                raise ValueError(f'the edit was made on the store as {basis}, it is now {now}; '
                                 'review a new pack, or force it')
        directory = self.path / 'edits'
        directory.mkdir(exist_ok=True)
        existing = {p.stem for p in directory.glob('*.json')}
        number = len(existing)
        while f'{number:04d}' in existing:
            number += 1
        edit_id = f'{number:04d}'
        document = {**edit, 'id': edit_id,
                    'sessions': [entry['id'] for entry in self._index['sessions']]}
        _write_json(directory / f'{edit_id}.json', document)
        entry = {'id': edit_id, 'author': edit['author'], 'reason': edit['reason'],
                 'operations': len(edit['operations']),
                 'voxels': sum(len(op['voxels']) for op in edit['operations']),
                 'applied': datetime.now(timezone.utc).isoformat(timespec='seconds')}
        index = dict(self._index)
        index['edits'] = [*self._index.get('edits', []), entry]
        _write_json(self.path / _INDEX, index)
        self._index = index
        if len(self._aligned_sessions()) == len(index['sessions']):
            self.export(log=log)
        return dict(entry)

    def rollback_edit(self, log: Callable[[str, str], None] | None = None) -> dict[str, Any]:
        """Takes the latest edit off the list (its file stays) and rebuilds the export."""
        edits = self._index.get('edits', [])
        if not edits:
            raise ValueError('the store has no edit')
        index = dict(self._index)
        index['edits'] = edits[:-1]
        _write_json(self.path / _INDEX, index)
        self._index = index
        if index['sessions'] and len(self._aligned_sessions()) == len(index['sessions']):
            self.export(log=log)
        return dict(edits[-1])

    def _edit_voxels(self) -> dict[str, list[list[int]]]:
        from lidarslam import map_edits
        if not self._index.get('edits'):
            return {}
        remove, keep = map_edits.fold(
            self.edit_document(entry['id']) for entry in self._index['edits'])
        return {'edit_remove': remove.tolist(), 'edit_keep': keep.tolist()}

    def _session_args(self, sessions: list[dict[str, Any]]) -> tuple[list[str], dict[str, Any]]:
        """The session directories and the pose, evidence and edit arguments of the export."""
        aligned = self._aligned_sessions()
        dirs = [str(self.session_dir(entry['id'])) for entry in sessions]
        transforms = dict(aligned)
        extra = ({'transforms': [transforms.get(directory, np.eye(4).tolist())
                                 for directory in dirs]}
                 if any('store_from_session' in entry for entry in sessions) else {})
        if any((Path(directory) / 'store_submap_poses.npy').is_file() for directory in dirs):
            from lidarslam.relocalization import tracked_poses
            extra['optimized_poses'] = [tracked_poses(directory) for directory in dirs]
        if any(entry.get('map_change_evidence') for entry in sessions):
            from lidarslam.map_changes import removal_masks
            extra['removed'] = removal_masks(self, sessions)
        extra.update(self._edit_voxels())
        return dirs, extra

    def points(self) -> dict[str, np.ndarray]:
        """Every stored point at its exported pose (``_core.store_points``): ``xyz``, ``voxel``
        (the store-frame voxel edits address), ``session``, ``submap`` and ``state`` (0 kept,
        1 removed by change evidence, 2 removed by an edit, 3 kept by an edit)."""
        sessions = self._index['sessions']
        if not sessions:
            raise ValueError('the store has no session')
        if len(self._aligned_sessions()) != len(sessions):
            raise ValueError('the store contains sessions mapped in their own frames')
        dirs, extra = self._session_args(sessions)
        return _core.store_points(dirs, **extra)

    def backend_params(self, session_id: str) -> dict[str, Any]:
        with (self.session_dir(session_id) / 'backend_params.json').open(encoding='utf-8') as f:
            return json.load(f)

    def _aligned_sessions(self) -> list[tuple[str, list[list[float]]]]:
        """Sessions in the primary map frame. Older stores imply identity for the first session."""
        aligned = []
        for index, entry in enumerate(self._index['sessions']):
            transform = entry.get('store_from_session')
            if transform is None and index == 0:
                transform = np.eye(4).tolist()
            if transform is not None:
                aligned.append((str(self.session_dir(entry['id'])), transform))
        return aligned

    def export(self, out_dir: str | Path | None = None,
               params: dict[str, Any] | None = None, stack_sessions: bool = False,
               log: Callable[[str, str], None] | None = None) -> Path:
        """Writes the Autoware map bundle of the committed sessions; returns its directory.

        ``out_dir`` defaults to ``export/`` in the store, which is replaced only once the new
        bundle is complete. ``params`` are the map-saving parameters (``map_leaf_size``, ...);
        by default the first session's backend parameters. ``stack_sessions`` also exports
        unaligned sessions at their original local poses.
        """
        sessions = self._index['sessions']
        if not sessions:
            raise ValueError('the store has no session')
        aligned = self._aligned_sessions()
        if len(aligned) != len(sessions) and not stack_sessions:
            raise ValueError('the store contains sessions mapped in their own frames; '
                             'pass stack_sessions=True to export them stacked')
        if params is None:
            params = self.backend_params(sessions[0]['id'])
        dirs, extra = self._session_args(sessions)
        if out_dir is not None:
            _core.export_store_bundle(dirs, params, str(out_dir), log, **extra)
            return Path(out_dir)
        final = self.path / 'export'
        staging = self.path / 'export.tmp'
        if staging.exists():
            shutil.rmtree(staging)
        _core.export_store_bundle(dirs, params, str(staging), log, **extra)
        previous = self.path / 'export.old'
        if final.exists():
            os.replace(final, previous)
        os.replace(staging, final)
        if previous.exists():
            shutil.rmtree(previous)
        return final

    def add_session(self, write: Callable[[str], int], odometry_params: dict[str, Any],
                    backend_params: dict[str, Any], info: dict[str, Any] | None = None,
                    export: bool = True,
                    log: Callable[[str, str], None] | None = None) -> dict[str, Any]:
        """Writes a session with ``write(directory)`` (returns its submap count), commits it and,
        when ``export`` is true and the sessions share a frame, exports the bundle. Returns its entry."""
        session_id, directory = self.new_session()
        submaps = write(str(directory))
        _write_json(directory / 'odometry_params.json', _jsonable(odometry_params))
        _write_json(directory / 'backend_params.json', _jsonable(backend_params))
        entry_info = {'submaps': submaps, **(info or {})}
        _write_json(directory / 'session.json', {'id': session_id, **entry_info})
        entry = self.commit(session_id, entry_info)
        if export and len(self._aligned_sessions()) == len(self._index['sessions']):
            self.export(log=log)
        return entry
