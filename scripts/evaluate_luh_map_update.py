#!/usr/bin/env python3
"""Measure one LUH map update against the full TLS cloud and an optional reference map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct

import numpy as np
from scipy.spatial import cKDTree

from map_edit import _decompress_lzf, _pcd_layout

MATCH_M = .10
NEW_M = .20
CHUNK = 1_000_000


def xyz_chunks(path):
    """Read binary PCD XYZ, streaming interleaved files in bounded-size chunks."""
    layout = _pcd_layout(path)
    indices = [layout.fields.index(axis) for axis in 'xyz']
    if any(layout.types[i] != 'F' or layout.sizes[i] not in (4, 8) or
           layout.counts[i] != 1 for i in indices):
        raise ValueError(f'{path}: expected scalar floating-point XYZ')
    offsets = np.cumsum([0, *layout.field_widths[:-1]])
    if layout.data == 'binary':
        dtype = np.dtype(dict(names=list('xyz'),
                              formats=[f'<f{layout.sizes[i]}' for i in indices],
                              offsets=[offsets[i] for i in indices], itemsize=layout.row_size))
        raw = np.memmap(path, mode='r', dtype=dtype, offset=layout.header_bytes,
                        shape=(layout.points,))
        for start in range(0, len(raw), CHUNK):
            chunk = raw[start:start + CHUNK]
            yield np.column_stack([chunk[axis] for axis in 'xyz'])
    elif layout.data == 'binary_compressed':
        with path.open('rb') as stream:
            stream.seek(layout.header_bytes)
            compressed, expanded = struct.unpack('<II', stream.read(8))
            if expanded != layout.points * layout.row_size:
                raise ValueError(f'{path}: incorrect compressed PCD layout')
            raw = _decompress_lzf(stream.read(compressed), expanded)
        yield np.column_stack([np.frombuffer(raw, dtype=f'<f{layout.sizes[i]}',
                                             count=layout.points, offset=layout.points * offsets[i])
                               for i in indices])
    else:
        raise ValueError(f'{path}: expected binary or binary_compressed PCD')


def transform(path):
    """Read a rigid store-to-TLS transform as a JSON matrix."""
    matrix = np.asarray(json.loads(path.read_text()), dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1]) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-6)):
        raise ValueError(f'{path}: expected a rigid 4x4 transform')
    return matrix


class Region:
    """Hall floor polygon, specified floor elevation and update path in the TLS frame."""

    def __init__(self, polygon, floor, trajectory):
        if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3 or
                not np.isfinite(polygon).all() or not np.isfinite(floor)):
            raise ValueError('expected at least three finite XY polygon vertices and a floor elevation')
        if trajectory.shape[1] != 3 or not len(trajectory) or not np.isfinite(trajectory).all():
            raise ValueError('expected an update path with three finite XYZ columns')
        self.vertices = polygon
        self.floor = floor
        self.path = cKDTree(trajectory[:, :2])

    def points(self, points):
        valid = np.isfinite(points).all(axis=1)
        valid &= (points[:, 2] >= self.floor + .1) & (points[:, 2] <= self.floor + 2.5)
        points = points[valid]
        x, y = points[:, :2].T
        inside = np.zeros(len(points), dtype=bool)
        for (x0, y0), (x1, y1) in zip(self.vertices, np.roll(self.vertices, -1, axis=0)):
            if y0 != y1:
                crosses = (y0 > y) != (y1 > y)
                inside ^= crosses & (x < (x1 - x0) * (y - y0) / (y1 - y0) + x0)
        points = points[inside]
        return points[self.path.query(points[:, :2], distance_upper_bound=10.)[0] < 10.]

    def load_map(self, path, pose):
        points = np.concatenate([self.points(x @ pose[:3, :3].T + pose[:3, 3])
                                 for x in xyz_chunks(path)])
        if len(points) == 0:
            raise ValueError(f'{path}: no map points in the evaluation region')
        return points


def matches(tree, points, threshold):
    """Nearest-neighbour threshold with an inclusive boundary."""
    limit = np.nextafter(threshold, np.inf)
    return tree.query(points, distance_upper_bound=limit)[0] <= threshold


def measure(tls_file, pairs, region, reference=None):
    maps = {key: points for name, (before, after) in pairs.items()
            for key, points in ((name + '_before', before), (name + '_after', after))}
    trees = {name: cKDTree(points) for name, points in maps.items()}
    covered = dict.fromkeys(maps, 0)
    parts = {name: np.ones(len(after), dtype=bool) for name, (_, after) in pairs.items()}
    genuine_new = np.ones(len(reference), dtype=bool) if reference is not None else None
    count = 0
    for chunk in xyz_chunks(tls_file):
        tls = region.points(chunk)
        if len(tls) == 0:
            continue
        count += len(tls)
        tls_tree = cKDTree(tls)
        for key, tree in trees.items():
            covered[key] += int(np.count_nonzero(matches(tree, tls, MATCH_M)))
        for name, (_, after) in pairs.items():
            missing = parts[name]
            missing[missing] = ~matches(tls_tree, after[missing], MATCH_M)
        if genuine_new is not None:
            genuine_new[genuine_new] = ~matches(tls_tree, reference[genuine_new], NEW_M)
    if count == 0:
        raise ValueError('no TLS points in the evaluation region')
    selected = reference[genuine_new] if reference is not None else None
    if selected is not None and len(selected) == 0:
        raise ValueError('reference has no new structure in the evaluation region')
    result = {'tls_points': count, 'new_reference_points': len(selected) if selected is not None else None}
    for name, (_, after) in pairs.items():
        before = covered[name + '_before']
        after_count = covered[name + '_after']
        result[name] = dict(G=float(parts[name].mean()), S=after_count / before if before else None,
                            TLS_coverage_before=before / count, TLS_coverage_after=after_count / count,
                            map_points=len(after))
        if selected is not None:
            result[name]['N'] = float(matches(trees[name + '_after'], selected, MATCH_M).mean())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ('before', 'after', 'tls', 'store-to-tls', 'polygon', 'path-xyz'):
        parser.add_argument('--' + flag, type=Path, required=True)
    parser.add_argument('--floor', type=float, required=True, help='floor elevation in TLS metres')
    parser.add_argument('--reference', type=Path, help='independently mapped session of the same layout')
    parser.add_argument('--reference-to-tls', type=Path)
    baseline = parser.add_mutually_exclusive_group(required=True)
    baseline.add_argument('--baseline-after', type=Path)
    baseline.add_argument('--baseline-refusal', type=Path, help='log from merge --place-recognition')
    parser.add_argument('--baseline-before', type=Path,
                        help='pre-update baseline map, defaults to --before')
    args = parser.parse_args()
    if bool(args.reference) != bool(args.reference_to_tls):
        parser.error('--reference and --reference-to-tls must be supplied together')
    if args.baseline_before and not args.baseline_after:
        parser.error('--baseline-before requires --baseline-after')
    region = Region(np.asarray(json.loads(args.polygon.read_text()), dtype=float), args.floor,
                    np.loadtxt(args.path_xyz, ndmin=2))
    pose = transform(args.store_to_tls)
    pairs = {'live': (region.load_map(args.before, pose), region.load_map(args.after, pose))}
    if args.baseline_after:
        pairs['baseline'] = (region.load_map(args.baseline_before or args.before, pose),
                             region.load_map(args.baseline_after, pose))
    reference = (region.load_map(args.reference, transform(args.reference_to_tls))
                 if args.reference else None)
    report = measure(args.tls, pairs, region, reference)
    if args.baseline_refusal:
        reason = args.baseline_refusal.read_text().strip()
        if not reason:
            parser.error('--baseline-refusal log is empty')
        report['baseline'] = {'status': 'refused', 'log': str(args.baseline_refusal), 'reason': reason}
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
