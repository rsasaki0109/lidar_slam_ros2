#!/usr/bin/env python3
"""Convert a PLY point cloud to deterministic float32 XYZ binary PCD.

The common map-quality executable consumes PCD through PCL.  FAST-LIVO2's
Open3D export stores XYZ as float64 in a binary PLY, which some PCL PLY paths
misinterpret.  This converter deliberately keeps geometry only and emits the
small, unambiguous PCD schema expected by the common evaluator.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import sys

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    return parser.parse_args()


def load_xyz(path: Path) -> np.ndarray:
    try:
        import open3d as o3d
    except ImportError as exc:  # pragma: no cover - environment failure path
        raise RuntimeError('open3d is required to read PLY files') from exc

    cloud = o3d.io.read_point_cloud(str(path), remove_nan_points=False,
                                    remove_infinite_points=False)
    xyz64 = np.asarray(cloud.points)
    if xyz64.ndim != 2 or xyz64.shape[1:] != (3,) or len(xyz64) == 0:
        raise ValueError(f'PLY has no XYZ vertices: {path}')
    if not np.isfinite(xyz64).all():
        bad = int((~np.isfinite(xyz64).all(axis=1)).sum())
        raise ValueError(f'PLY contains {bad} non-finite XYZ vertices: {path}')
    xyz32 = np.ascontiguousarray(xyz64, dtype='<f4')
    if not np.isfinite(xyz32).all():
        raise ValueError('float32 conversion overflowed')
    return xyz32


def write_binary_pcd(path: Path, xyz: np.ndarray) -> None:
    count = len(xyz)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {count}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {count}\n'
        'DATA binary\n'
    ).encode('ascii')
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('wb') as stream:
        stream.write(header)
        stream.write(xyz.tobytes(order='C'))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    args = parse_args()
    if not args.input.is_file():
        print(f'input does not exist: {args.input}', file=sys.stderr)
        return 2
    try:
        xyz = load_xyz(args.input)
        write_binary_pcd(args.output, xyz)
    except (RuntimeError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(f'points: {len(xyz)}')
    print(f'output: {args.output}')
    print(f'sha256: {sha256(args.output)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
