#!/usr/bin/env python3
"""Export compact GLIM submaps as one binary XYZ PCD in world coordinates."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

import numpy as np


def world_transform(data_path: Path) -> np.ndarray:
    text = data_path.read_text(errors='replace')
    match = re.search(r'T_world_origin:\s*\n((?:[^\n]+\n){4})', text)
    if not match:
        raise ValueError(f'{data_path}: T_world_origin not found')
    values = [float(value) for value in match.group(1).split()]
    if len(values) != 16:
        raise ValueError(f'{data_path}: invalid T_world_origin')
    return np.asarray(values, dtype=np.float64).reshape(4, 4)


def load_world_points(submap: Path) -> np.ndarray:
    points = np.fromfile(submap / 'points_compact.bin', dtype=np.float32)
    if points.size % 3:
        raise ValueError(f'{submap}: compact point count is not divisible by 3')
    points = points.reshape(-1, 3).astype(np.float64)
    transform = world_transform(submap / 'data.txt')
    world = points @ transform[:3, :3].T + transform[:3, 3]
    return world[np.isfinite(world).all(axis=1)].astype(np.float32)


def export_dump(dump: Path, output: Path) -> int:
    submaps = sorted(path for path in dump.iterdir()
                     if path.is_dir() and path.name.isdigit() and
                     (path / 'points_compact.bin').exists())
    if not submaps:
        raise ValueError(f'{dump}: no compact GLIM submaps')
    clouds = [load_world_points(submap) for submap in submaps]
    points = np.concatenate(clouds)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\nDATA binary\n').encode()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(header + points.tobytes(order='C'))
    return len(points)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--dump', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    count = export_dump(args.dump, args.output)
    print(f'exported {count} points to {args.output}')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
