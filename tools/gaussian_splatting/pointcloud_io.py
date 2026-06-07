#!/usr/bin/env python3
"""Minimal PLY point-cloud I/O + voxel downsampling (numpy only, ROS/GPU-free).

Shared by ``build_lidar_init.py`` (writes a LiDAR-primed init cloud) and
``train_gsplat.py`` (seeds Gaussians from it). Supports the small subset of PLY
this pipeline needs: ``x y z`` floats with optional ``red green blue`` uchar, in
binary-little-endian or ascii. See
``docs/research/3dgs-postprocess-map-design.md``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np


def write_ply(path: str | Path, xyz: np.ndarray,
              rgb: Optional[np.ndarray] = None) -> Path:
    """Write ``xyz`` (N,3 float) and optional ``rgb`` (N,3 uint8) to a binary PLY."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    xyz = np.asarray(xyz, dtype=np.float32)
    n = xyz.shape[0]
    header = 'ply\nformat binary_little_endian 1.0\n'
    header += f'element vertex {n}\n'
    header += 'property float x\nproperty float y\nproperty float z\n'
    if rgb is not None:
        header += 'property uchar red\nproperty uchar green\nproperty uchar blue\n'
    header += 'end_header\n'
    with open(path, 'wb') as fh:
        fh.write(header.encode('ascii'))
        if rgb is None:
            fh.write(xyz.tobytes())
        else:
            rgb = np.asarray(rgb, dtype=np.uint8)
            dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                           ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
            rec = np.empty(n, dtype=dt)
            rec['x'], rec['y'], rec['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            rec['r'], rec['g'], rec['b'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
            fh.write(rec.tobytes())
    return path


def read_ply_xyz(path: str | Path) -> tuple[np.ndarray, Optional[np.ndarray]]:
    """Read a PLY into ``(xyz float32 (N,3), rgb uint8 (N,3) or None)``.

    Handles binary-little-endian and ascii with float ``x y z`` and optional
    uchar ``red green blue``. Other properties are tolerated (skipped) as long
    as their type is a known fixed-size scalar.
    """
    raw = Path(path).read_bytes()
    end = raw.index(b'end_header\n') + len(b'end_header\n')
    header = raw[:end].decode('ascii')
    fmt = 'ascii'
    count = 0
    props: list[tuple[str, str]] = []
    for line in header.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'format':
            fmt = parts[1]
        elif parts[0] == 'element' and parts[1] == 'vertex':
            count = int(parts[2])
        elif parts[0] == 'property':
            props.append((parts[1], parts[2]))  # (type, name)

    names = [n for _, n in props]
    np_types = {
        'float': np.float32, 'float32': np.float32, 'double': np.float64,
        'uchar': np.uint8, 'uint8': np.uint8, 'char': np.int8, 'int8': np.int8,
        'ushort': np.uint16, 'short': np.int16, 'uint': np.uint32,
        'int': np.int32, 'int32': np.int32,
    }
    has_rgb = all(c in names for c in ('red', 'green', 'blue'))

    if fmt == 'ascii':
        body = raw[end:].decode('ascii').split('\n')
        vals = [r.split() for r in body if r.strip()][:count]
        arr = np.array(vals, dtype=np.float64)
        idx = {n: i for i, n in enumerate(names)}
        xyz = arr[:, [idx['x'], idx['y'], idx['z']]].astype(np.float32)
        rgb = (arr[:, [idx['red'], idx['green'], idx['blue']]].astype(np.uint8)
               if has_rgb else None)
        return xyz, rgb

    dt = np.dtype([(n, np.dtype(np_types[t]).newbyteorder('<'))
                   for t, n in props])
    rec = np.frombuffer(raw[end:end + dt.itemsize * count], dtype=dt)
    xyz = np.stack([rec['x'], rec['y'], rec['z']], axis=1).astype(np.float32)
    rgb = (np.stack([rec['red'], rec['green'], rec['blue']], axis=1).astype(np.uint8)
           if has_rgb else None)
    return xyz, rgb


def colorize_by_projection(points: np.ndarray, viewmats: np.ndarray,
                           K: np.ndarray, images, width: int, height: int,
                           default_rgb=(128, 128, 128)
                           ) -> tuple[np.ndarray, np.ndarray]:
    """Colour points by projecting them into posed camera images and averaging.

    For each point, project into every camera (``viewmats`` are OpenCV
    world->camera, as ``train_gsplat.load_transforms`` returns), sample the pixel
    where it lands in front of and inside the image, and average the colours over
    all such views. This seeds Gaussian colour from the real images instead of a
    flat grey, so training starts far closer to the target. No occlusion test --
    averaging over many views is enough for an init (training refines it).

    Returns ``(rgb uint8 (N,3), seen bool (N,))``; unseen points get
    ``default_rgb``.
    """
    pts = np.asarray(points, dtype=np.float64)
    n = pts.shape[0]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    sum_rgb = np.zeros((n, 3), dtype=np.float64)
    cnt = np.zeros(n, dtype=np.int64)
    for vm, img in zip(viewmats, images):
        vm = np.asarray(vm, dtype=np.float64)
        cam = pts @ vm[:3, :3].T + vm[:3, 3]
        z = cam[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = fx * cam[:, 0] / z + cx
            v = fy * cam[:, 1] / z + cy
        ui = np.round(u).astype(np.int64)
        vi = np.round(v).astype(np.int64)
        inb = (z > 1e-6) & (ui >= 0) & (ui < width) & (vi >= 0) & (vi < height)
        idx = np.nonzero(inb)[0]
        if idx.size == 0:
            continue
        cols = np.asarray(img)[vi[idx], ui[idx]]
        if cols.ndim == 1:
            cols = np.repeat(cols[:, None], 3, axis=1)
        sum_rgb[idx] += cols[:, :3]
        cnt[idx] += 1
    seen = cnt > 0
    rgb = np.tile(np.asarray(default_rgb, dtype=np.uint8), (n, 1))
    rgb[seen] = np.round(sum_rgb[seen] / cnt[seen, None]).astype(np.uint8)
    return rgb, seen


def voxel_downsample(xyz: np.ndarray, voxel_size: float,
                     rgb: Optional[np.ndarray] = None
                     ) -> tuple[np.ndarray, Optional[np.ndarray]]:
    """Keep one representative point per ``voxel_size`` cube (first occurrence).

    Returns downsampled ``(xyz, rgb)``. A non-positive ``voxel_size`` is a no-op.
    """
    xyz = np.asarray(xyz, dtype=np.float32)
    if voxel_size <= 0 or xyz.shape[0] == 0:
        return xyz, rgb
    keys = np.ascontiguousarray(np.floor(xyz / voxel_size).astype(np.int64))
    # Uniquify the (N,3) voxel keys via a 1D structured (void) view rather than
    # np.unique(axis=0): the latter lexicographically sorts a 2D array
    # row-by-row, which is far slower and more memory-hungry on the multi-
    # million-point clouds build_lidar_init accumulates.
    key_view = keys.view([('k', keys.dtype, 3)]).ravel()
    _, first_idx = np.unique(key_view, return_index=True)
    first_idx.sort()
    return xyz[first_idx], (None if rgb is None else np.asarray(rgb)[first_idx])
