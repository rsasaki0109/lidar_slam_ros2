"""Causal observations and append-only removal decisions for a map store."""

from pathlib import Path

import numpy as np

from lidarslam import _core
from lidarslam.relocalization import pose_matrix

EVIDENCE_FILE = 'map_change_evidence.npz'


class MapChanges:
    """Accumulate old-map visibility after the session has a verified store pose."""

    def __init__(self, directories, poses, extrinsic, filter_new_points=False,
                 refine_visibility=False):
        self.references = [Path(directory).name for directory in directories]
        self.evidence = _core.MapChangeEvidence(directories, poses)
        self.refined_visibility = refine_visibility
        self.extrinsic = extrinsic
        self.pending = []
        self.new_evidence = None
        if refine_visibility:
            self.new_evidence = _core.RefinedNewPointEvidence()
        elif filter_new_points:
            self.new_evidence = _core.NewPointEvidence()
        self.next_sample = -float('inf')
        self.capture_poses = {}
        self.suppressed_new_voxels = None

    def observe(self, pose, scan, startup):
        """Queue early scans and trace them once the session joins the store."""
        if pose is not None and (startup.active or startup.transform is not None):
            stamp = pose['stamp_sec']
            sample = self.new_evidence is not None and stamp + .001 >= self.next_sample
            if sample:
                self.next_sample = stamp + .5
            submap = pose['submaps'] - 1 if pose['new_submap'] else None
            if submap is not None or sample:
                endpoints = np.asarray(scan).reshape(-1, 3).copy()
                self.pending.append((pose_matrix(pose['map']), submap, sample, endpoints))
        if startup.transform is None:
            if not startup.active:
                self.pending.clear()
            return
        for local_from_base, submap, sample, endpoints in self.pending:
            world_from_base = startup.transform @ local_from_base
            if submap is not None and self.new_evidence is not None:
                self.capture_poses[submap] = world_from_base
            transform = world_from_base @ self.extrinsic
            world = endpoints @ transform[:3, :3].T + transform[:3, 3]
            world = np.ascontiguousarray(world, dtype='f4')
            if submap is not None:
                self.evidence.observe(transform[:3, 3], world)
            if sample and self.new_evidence is not None:
                self.new_evidence.observe(transform[:3, 3], world)
        self.pending.clear()

    def rows(self):
        """Return evidence rows in deterministic session, submap and local voxel order."""
        return np.asarray(sorted(self.evidence.snapshot()), dtype=np.int64).reshape(-1, 9)

    def write(self, directory, rows):
        """Save the scan evidence and decisions in the updating session's directory."""
        extra = {}
        if self.new_evidence is not None:
            mask = self.new_evidence.suppression_mask(
                directory, self.capture_poses, self.extrinsic[:3, 3])
            extra['new_rows'] = np.asarray(sorted(mask), dtype=np.int64).reshape(-1, 4)
            self.suppressed_new_voxels = len(mask)
        np.savez_compressed(Path(directory) / EVIDENCE_FILE, references=self.references,
                            rows=rows, **extra)


def removal_masks(store, sessions):
    """Replay committed decisions in session order into per-submap removal masks."""
    index = {entry['id']: i for i, entry in enumerate(sessions)}
    masks = [{} for _ in sessions]
    for i, entry in enumerate(sessions):
        if not entry.get('map_change_evidence'):
            continue
        path = store.session_dir(entry['id']) / EVIDENCE_FILE
        with np.load(path, allow_pickle=False) as evidence:
            references = [index[name] for name in evidence['references'].tolist()]
            rows = evidence['rows']
            new_rows = evidence['new_rows'] if 'new_rows' in evidence else None
        if any(ref >= i for ref in references):
            raise ValueError(f'{path}: evidence must refer to earlier committed sessions')
        if (rows.ndim != 2 or rows.shape[1] != 9 or rows.dtype.kind != 'i' or
                np.any((rows[:, 0] < 0) | (rows[:, 0] >= len(references))) or
                np.any((rows[:, 8] < 0) | (rows[:, 8] > 2))):
            raise ValueError(f'{path}: invalid change evidence rows')
        if new_rows is not None:
            if (new_rows.ndim != 2 or new_rows.shape[1] != 4 or new_rows.dtype.kind != 'i' or
                    np.any((new_rows[:, 0] < 0) | (new_rows[:, 0] >= entry['submaps']))):
                raise ValueError(f'{path}: invalid new-point suppression mask')
            for submap, x, y, z in new_rows.tolist():
                masks[i].setdefault(submap, set()).add((x, y, z))
        for ref, submap, x, y, z, _, _, _, decision in rows[rows[:, 8] != 0]:
            if not 0 <= submap < sessions[references[ref]]['submaps']:
                raise ValueError(f'{path}: evidence refers to a missing submap')
            mask = masks[references[ref]].setdefault(int(submap), set())
            voxel = (int(x), int(y), int(z))
            if decision == 2:
                mask.add(voxel)
            else:
                mask.discard(voxel)
    return masks
