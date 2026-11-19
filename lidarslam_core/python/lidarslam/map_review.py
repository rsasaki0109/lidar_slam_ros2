"""Reviewing a long-term map update, as an agent does it (the tools of ``python -m lidarslam mcp``).

:class:`Review` holds one reviewer's pass over a store: it lists the candidates of
:mod:`lidarslam.map_edits`, describes one with numbers and a picture (a top and a side view of
it in its surroundings), shows any region, collects keep / remove decisions, and commits them as
one edit with the reviewer's name and reason. With ``propose_only`` the commit writes a proposal
file instead (``STORE/proposals/<id>.json``) for a person to apply with
``python -m lidarslam store apply-edit``. Nothing here needs the MCP SDK;
:mod:`lidarslam.mcp_server` puts these methods behind MCP tools.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import struct
from typing import Any
import zlib

import numpy as np

from lidarslam import map_edits
from lidarslam.store import Store

KIND_RGB = {'removed_old': (239, 83, 80), 'suppressed_new': (255, 167, 38),
            'added': (102, 187, 106)}
EDIT_RGB = {'keep': (38, 198, 218), 'remove': (236, 64, 122)}
FOCUS_RGB = (255, 238, 88)
REMOVED_RGB = (126, 87, 194)


def png(rgb: np.ndarray) -> bytes:
    """A PNG of an H x W x 3 uint8 image."""
    height, width, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[row].tobytes() for row in range(height))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (struct.pack('>I', len(data)) + kind + data +
                struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff))

    header = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header) + chunk(b'IDAT', zlib.compress(raw, 6))
            + chunk(b'IEND', b''))


class Review:
    """One reviewer's pass over a map store."""

    def __init__(self, store: str | Path, author: str = 'agent:claude', propose_only: bool = False,
                 min_voxels: int = 5):
        if ':' not in author or author.split(':', 1)[0] not in ('human', 'agent'):
            raise ValueError("author must be 'human:<name>' or 'agent:<name>'")
        self.path = Path(store)
        self.author = author
        self.propose_only = propose_only
        self.min_voxels = min_voxels
        self.pending: list[dict[str, Any]] = []  # {'op', 'voxels', 'candidate'?, 'note'?}
        self._load()

    # --- State --------------------------------------------------------------------------------
    def _load(self) -> None:
        self.store = Store.open(self.path)
        self.points = self.store.points()
        self.candidates = map_edits.candidates(self.store, self.min_voxels, self.points)
        self.by_id = {c['id']: c for c in self.candidates}
        self.keys = map_edits._keys(self.points['voxel'])
        added = [c['voxel_keys'] for c in self.candidates if c['kind'] == 'added']
        self.added = (np.isin(self.keys, np.concatenate(added)) if added
                      else np.zeros(self.keys.size, dtype=bool))

    def _decided(self) -> dict[str, str]:
        return {op['candidate']: op['op'] for op in self.pending if 'candidate' in op}

    def info(self) -> dict[str, Any]:
        """The store's sessions and edits, the candidates by kind and the pending decisions."""
        kinds: dict[str, int] = {}
        for c in self.candidates:
            kinds[c['kind']] = kinds.get(c['kind'], 0) + 1
        sessions = [{'id': s['id'], 'submaps': s.get('submaps'),
                     'anchored': 'store_from_session' in s or i == 0,
                     'change_evidence': bool(s.get('map_change_evidence'))}
                    for i, s in enumerate(self.store.sessions)]
        lo, hi = self.points['xyz'].min(axis=0), self.points['xyz'].max(axis=0)
        return {
            'store': str(self.path), 'frame': self.store.frame, 'sessions': sessions,
            'edits': self.store.edits, 'candidates': kinds,
            'pending_decisions': len(self.pending), 'author': self.author,
            'propose_only': self.propose_only, 'voxel_m': map_edits.VOXEL_M,
            'map_bounds': {'min': [round(float(v), 2) for v in lo],
                           'max': [round(float(v), 2) for v in hi]},
            'kinds': {'removed_old': 'old points the change evidence removed',
                      'suppressed_new': 'new points the change evidence held back',
                      'added': 'new points where the earlier sessions had none'},
        }

    def list_candidates(self, kind: str | None = None, offset: int = 0, limit: int = 50,
                        undecided_only: bool = False) -> dict[str, Any]:
        decided = self._decided()
        chosen = [c for c in self.candidates if (kind is None or c['kind'] == kind)
                  and not (undecided_only and c['id'] in decided)]
        page = chosen[offset:offset + limit]
        return {'total': len(chosen), 'offset': offset,
                'candidates': [{**{k: v for k, v in c.items() if k != 'voxel_keys'},
                                'decision': decided.get(c['id'])} for c in page]}

    # --- Looking ------------------------------------------------------------------------------
    def _floor(self, center: np.ndarray, radius: float = 5.0) -> float:
        xyz = self.points['xyz']
        near = np.linalg.norm(xyz[:, :2] - center[:2], axis=1) < radius
        kept = near & np.isin(self.points['state'], (map_edits.KEPT, map_edits.KEPT_BY_EDIT))
        return float(np.percentile(xyz[kept, 2], 2)) if np.any(kept) else float(center[2])

    def describe(self, candidate_id: str) -> tuple[dict[str, Any], bytes]:
        """Numbers about a candidate and a PNG (top view left, side view right, 6 m around it)."""
        c = self._candidate(candidate_id)
        keys = self.keys
        inside = np.isin(keys, c['voxel_keys'])
        sessions = self.points['session'][inside]
        states = self.points['state'][inside]
        ids = [s['id'] for s in self.store.sessions]
        centroid = np.asarray(c['centroid'])
        floor = self._floor(centroid)
        # How much of it touches structure that stays in the map.
        kept_keys = np.unique(keys[np.isin(self.points['state'],
                                           (map_edits.KEPT, map_edits.KEPT_BY_EDIT)) & ~inside])
        voxels = map_edits._voxels(c['voxel_keys'])
        touching = np.zeros(len(voxels), dtype=bool)
        for offset in np.vstack([map_edits._NEIGHBOURS, -map_edits._NEIGHBOURS]):
            shifted = map_edits._keys(voxels + offset)
            at = np.searchsorted(kept_keys, shifted)
            at[at >= kept_keys.size] = 0
            touching |= kept_keys[at] == shifted if kept_keys.size else False
        lo, hi = np.asarray(c['min']), np.asarray(c['max'])
        info = {
            **{k: v for k, v in c.items() if k != 'voxel_keys'},
            'decision': self._decided().get(c['id']),
            'size_m': [round(float(v), 2) for v in hi - lo],
            'height_above_floor_m': [round(float(lo[2] - floor), 2),
                                     round(float(hi[2] - floor), 2)],
            'points_per_session': {ids[int(s)]: int((sessions == s).sum())
                                   for s in np.unique(sessions)},
            'points_by_state': {name: int((states == value).sum()) for name, value in (
                ('kept', map_edits.KEPT), ('removed_by_evidence', map_edits.REMOVED_BY_EVIDENCE),
                ('removed_by_edit', map_edits.REMOVED_BY_EDIT),
                ('kept_by_edit', map_edits.KEPT_BY_EDIT)) if (states == value).any()},
            'share_of_voxels_touching_kept_structure': round(float(touching.mean()), 3),
            'image': 'left: top view (x right, y up); right: side view (x right, z up); 6 m '
                     'around the candidate; yellow = this candidate, grey = map (lighter is '
                     'higher), red = removed old, orange = held-back new, green = added, '
                     'purple = removed by an earlier edit',
        }
        return info, self.render(centroid, 6.0, focus=inside)

    def view_region(self, center: list[float], radius: float = 6.0) -> bytes:
        """A PNG of the map around a store-frame point (top and side views)."""
        if not 0.5 <= radius <= 100.0:
            raise ValueError('radius is 0.5 to 100 m')
        return self.render(np.asarray(center, dtype=float), radius)

    def render(self, center: np.ndarray, radius: float, focus: np.ndarray | None = None,
               size: int = 384) -> bytes:
        xyz = self.points['xyz']
        near = np.all(np.abs(xyz[:, :2] - center[:2]) <= radius, axis=1)
        idx = np.flatnonzero(near)
        state, session = self.points['state'][idx], self.points['session'][idx]
        latest = len(self.store.sessions) - 1
        p = xyz[idx]
        z_lo, z_hi = (np.percentile(p[:, 2], [1, 99]) if len(p) else (0.0, 1.0))
        shade = np.clip((p[:, 2] - z_lo) / max(1e-6, z_hi - z_lo), 0, 1)
        colors = np.stack([90 + 120 * shade] * 3, axis=1).astype(np.uint8)
        removed = state == map_edits.REMOVED_BY_EVIDENCE
        colors[removed & (session < latest)] = KIND_RGB['removed_old']
        colors[removed & (session == latest)] = KIND_RGB['suppressed_new']
        colors[state == map_edits.REMOVED_BY_EDIT] = REMOVED_RGB
        keys = self.keys[idx]
        colors[self.added[idx] & (state == map_edits.KEPT)] = KIND_RGB['added']
        for op in self.pending:  # decisions not yet committed
            colors[np.isin(keys, map_edits._keys(np.asarray(op['voxels'])))] = EDIT_RGB[op['op']]
        order = np.argsort(p[:, 2], kind='stable')  # top view: higher points drawn last
        priority = np.zeros(len(idx), dtype=np.int8)
        if focus is not None:
            focused = focus[idx]
            colors[focused] = FOCUS_RGB
            priority[focused] = 1
        image = np.full((size, 2 * size + 8, 3), 255, dtype=np.uint8)
        scale = (size - 1) / (2 * radius)

        def draw(u: np.ndarray, v: np.ndarray, at: int, draw_order: np.ndarray) -> None:
            draw_order = draw_order[np.argsort(priority[draw_order], kind='stable')]
            col = np.clip(((u[draw_order]) * scale).astype(int), 0, size - 1) + at
            row = np.clip((size - 1 - v[draw_order] * scale).astype(int), 0, size - 1)
            image[row, col] = colors[draw_order]

        draw(p[:, 0] - center[0] + radius, p[:, 1] - center[1] + radius, 0, order)
        side = np.argsort(-(p[:, 1] - center[1]), kind='stable')  # nearer (lower y) drawn last
        z_base = center[2] - radius * 0.5
        draw(p[:, 0] - center[0] + radius, p[:, 2] - z_base, size + 8, side)
        image[:, size:size + 8] = 200
        return png(image)

    # --- Deciding -----------------------------------------------------------------------------
    def _candidate(self, candidate_id: str) -> dict[str, Any]:
        if candidate_id not in self.by_id:
            raise KeyError(f'no candidate {candidate_id}; list_candidates gives the current ids')
        return self.by_id[candidate_id]

    def decide(self, candidate_id: str, op: str, note: str = '') -> dict[str, Any]:
        """Keep or remove a candidate (``op``: keep, remove, or clear to drop the decision)."""
        c = self._candidate(candidate_id)
        self.pending = [o for o in self.pending if o.get('candidate') != candidate_id]
        if op == 'clear':
            return {'candidate': candidate_id, 'decision': None, 'pending': len(self.pending)}
        if op not in map_edits.OPERATIONS:
            raise ValueError("op is 'keep', 'remove' or 'clear'")
        entry = {'op': op, 'candidate': candidate_id,
                 'voxels': map_edits._voxels(c['voxel_keys']).tolist()}
        if note:
            entry['note'] = note
        self.pending.append(entry)
        return {'candidate': candidate_id, 'decision': op, 'pending': len(self.pending)}

    def decide_box(self, lo: list[float], hi: list[float], op: str,
                   note: str = '') -> dict[str, Any]:
        """Keep or remove every stored point inside a store-frame box."""
        if op not in map_edits.OPERATIONS:
            raise ValueError("op is 'keep' or 'remove'")
        voxels = map_edits.box_voxels(self.points, lo, hi)
        if not len(voxels):
            return {'voxels': 0, 'pending': len(self.pending)}
        entry = {'op': op, 'box': [list(map(float, lo)), list(map(float, hi))],
                 'voxels': voxels.tolist()}
        if note:
            entry['note'] = note
        self.pending.append(entry)
        return {'voxels': int(len(voxels)), 'pending': len(self.pending)}

    def commit(self, reason: str) -> dict[str, Any]:
        """Writes the pending decisions as one edit (or a proposal with ``propose_only``)."""
        if not self.pending:
            raise ValueError('no pending decision to commit')
        edit = map_edits.make_edit(self.author, reason, self.pending)
        edit['basis'] = {'sessions': [s['id'] for s in self.store.sessions],
                         'edits': [e['id'] for e in self.store.edits]}
        if self.propose_only:
            directory = self.path / 'proposals'
            directory.mkdir(exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            path = directory / f'{stamp}-{self.author.split(":", 1)[1]}.json'
            path.write_text(json.dumps(edit, indent=1) + '\n', encoding='utf-8')
            result = {'proposal': str(path), 'operations': len(self.pending),
                      'apply_with': f'python -m lidarslam store apply-edit {self.path} '
                                    f'--file {path}'}
        else:
            result = {'edit': self.store.add_edit(edit), 'export': str(self.path / 'export')}
            self._load()
        self.pending = []
        return result

    def rollback_edit(self) -> dict[str, Any]:
        """Takes the store's latest edit off the map (its file stays)."""
        if self.propose_only:
            raise PermissionError('this review only proposes edits; it does not change the store')
        removed = self.store.rollback_edit()
        self._load()
        return {'rolled_back': removed, 'edits': self.store.edits}
