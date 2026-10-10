#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Minimal PLY/PCD point-cloud I/O + voxel downsampling (numpy only, ROS/GPU-free).

Shared by ``build_lidar_init.py`` (writes a LiDAR-primed init cloud) and
``train_gsplat.py`` (seeds Gaussians from it). Supports the small subset of PLY
this pipeline needs: ``x y z`` floats with optional ``red green blue`` uchar, in
binary-little-endian or ascii. See
``docs/research/3dgs-postprocess-map-design.md``.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional, Sequence

# Sibling helpers stay importable regardless of which directory hosts the
# caller (same pattern as build_lidar_init.py).
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.append(str(_HERE))

import geometry_aware_fusion as gaf
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


def read_pcd_xyz(path: str | Path) -> tuple[np.ndarray, Optional[np.ndarray]]:
    """Read fixed-width ASCII or binary PCD into ``(xyz, rgb)``.

    This deliberately covers the uncompressed PCD files emitted by the
    offline graph runner. ``binary_compressed`` remains unsupported because
    decoding PCL's LZF stream would add a non-numpy dependency.
    """
    path = Path(path)
    metadata: dict[str, list[str]] = {}
    with path.open('rb') as fh:
        while True:
            line = fh.readline()
            if not line:
                raise ValueError(f'{path}: PCD DATA header is missing')
            decoded = line.decode('ascii').strip()
            if not decoded or decoded.startswith('#'):
                continue
            key, *values = decoded.split()
            metadata[key.upper()] = values
            if key.upper() == 'DATA':
                payload = fh.read()
                break

    fields = metadata.get('FIELDS', metadata.get('FIELD', []))
    sizes = [int(value) for value in metadata.get('SIZE', [])]
    types = metadata.get('TYPE', [])
    counts = [int(value) for value in metadata.get('COUNT', ['1'] * len(fields))]
    points = int(metadata.get('POINTS', metadata.get('WIDTH', ['0']))[0])
    if not fields or not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError(f'{path}: inconsistent PCD field metadata')
    if any(count != 1 for count in counts):
        raise ValueError(f'{path}: PCD COUNT > 1 is not supported')
    if not all(axis in fields for axis in ('x', 'y', 'z')):
        raise ValueError(f'{path}: PCD must contain x y z fields')

    scalar_types = {
        ('F', 4): '<f4', ('F', 8): '<f8',
        ('I', 1): 'i1', ('I', 2): '<i2', ('I', 4): '<i4',
        ('U', 1): 'u1', ('U', 2): '<u2', ('U', 4): '<u4',
    }
    try:
        dtype = np.dtype([
            (name, scalar_types[(kind.upper(), size)])
            for name, size, kind in zip(fields, sizes, types)
        ])
    except KeyError as exc:
        raise ValueError(f'{path}: unsupported PCD scalar type {exc.args[0]}') from exc

    def decode_rgb(values: np.ndarray) -> np.ndarray:
        """Decode PCL's packed 0x00RRGGBB ``rgb`` scalar."""
        index = fields.index('rgb')
        if sizes[index] != 4 or types[index].upper() not in ('F', 'U', 'I'):
            raise ValueError(f'{path}: packed rgb must be a 4-byte F/U/I scalar')
        if types[index].upper() == 'F':
            packed = np.asarray(values, dtype='<f4').view('<u4')
        else:
            packed = np.asarray(values, dtype='<u4')
        return np.stack([
            (packed >> 16) & 0xff,
            (packed >> 8) & 0xff,
            packed & 0xff,
        ], axis=1).astype(np.uint8)

    data_kind = metadata['DATA'][0].lower()
    if data_kind == 'binary':
        expected = points * dtype.itemsize
        if len(payload) < expected:
            raise ValueError(
                f'{path}: truncated PCD payload ({len(payload)} < {expected} bytes)')
        records = np.frombuffer(payload[:expected], dtype=dtype, count=points)
        xyz = np.stack([records['x'], records['y'], records['z']], axis=1)
        if all(channel in fields for channel in ('red', 'green', 'blue')):
            rgb = np.stack(
                [records['red'], records['green'], records['blue']], axis=1)
        elif 'rgb' in fields:
            rgb = decode_rgb(records['rgb'])
        else:
            rgb = None
    elif data_kind == 'ascii':
        rows = np.loadtxt(payload.splitlines(), dtype=np.float64, ndmin=2,
                          max_rows=points)
        indices = {name: index for index, name in enumerate(fields)}
        xyz = rows[:, [indices['x'], indices['y'], indices['z']]]
        if all(channel in fields for channel in ('red', 'green', 'blue')):
            rgb = rows[:, [indices['red'], indices['green'], indices['blue']]]
        elif 'rgb' in fields:
            rgb = decode_rgb(rows[:, indices['rgb']])
        else:
            rgb = None
    else:
        raise ValueError(f'{path}: unsupported PCD DATA {data_kind}')
    return (np.asarray(xyz, dtype=np.float32),
            None if rgb is None else np.asarray(rgb, dtype=np.uint8))


def read_point_cloud_xyz(
        path: str | Path) -> tuple[np.ndarray, Optional[np.ndarray]]:
    """Read a supported ``.ply`` or ``.pcd`` cloud by file extension."""
    suffix = Path(path).suffix.lower()
    if suffix == '.ply':
        return read_ply_xyz(path)
    if suffix == '.pcd':
        return read_pcd_xyz(path)
    raise ValueError(f'unsupported point-cloud extension: {suffix or "<none>"}')


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
        image = np.asarray(img)
        if image.shape[:2] != (height, width):
            raise ValueError('image dimensions must match projection calibration')
        cols = image[vi[idx], ui[idx]]
        if cols.ndim == 1:
            cols = np.repeat(cols[:, None], 3, axis=1)
        sum_rgb[idx] += cols[:, :3]
        cnt[idx] += 1
    seen = cnt > 0
    rgb = np.tile(np.asarray(default_rgb, dtype=np.uint8), (n, 1))
    rgb[seen] = np.round(sum_rgb[seen] / cnt[seen, None]).astype(np.uint8)
    return rgb, seen


def _sample_pixels(img: np.ndarray, uf: np.ndarray, vf: np.ndarray,
                   width: int, height: int, interp: str,
                   edge_threshold: float = 48.0) -> np.ndarray:
    """Sample ``img`` at float pixel coords ``(uf, vf)`` -> float32 ``(M,3)``.

    ``interp='nearest'`` rounds to the pixel centre; ``'bilinear'`` blends the
    four surrounding pixels (edge coords clamp, so the border never wraps).
    ``'edge-aware'`` uses bilinear on smooth patches but switches to the nearest
    real pixel when the local RGB range exceeds ``edge_threshold``, preventing
    foreground/background colour bleed. A 2-D image is broadcast to RGB.
    """
    im = np.asarray(img)
    if im.shape[:2] != (height, width):
        raise ValueError('image dimensions must match projection calibration')
    if im.ndim == 2:
        im = np.broadcast_to(im[:, :, None], (*im.shape, 3))
    im = im[:, :, :3]
    if interp == 'nearest':
        ui = np.clip(np.round(uf).astype(np.int64), 0, width - 1)
        vi = np.clip(np.round(vf).astype(np.int64), 0, height - 1)
        return im[vi, ui].astype(np.float32)
    if interp not in ('bilinear', 'edge-aware'):
        raise ValueError(
            "interp must be 'nearest', 'bilinear', or 'edge-aware', "
            f'got {interp!r}')
    if edge_threshold < 0.0:
        raise ValueError('edge_threshold must be >= 0')
    x0 = np.clip(np.floor(uf).astype(np.int64), 0, width - 1)
    y0 = np.clip(np.floor(vf).astype(np.int64), 0, height - 1)
    x1 = np.minimum(x0 + 1, width - 1)
    y1 = np.minimum(y0 + 1, height - 1)
    wx = np.clip(uf - x0, 0.0, 1.0)[:, None].astype(np.float32)
    wy = np.clip(vf - y0, 0.0, 1.0)[:, None].astype(np.float32)
    p00, p10 = im[y0, x0].astype(np.float32), im[y0, x1].astype(np.float32)
    if interp == 'edge-aware':
        local_minimum = np.minimum(p00, p10)
        local_maximum = np.maximum(p00, p10)
    top = p00 * (1.0 - wx) + p10 * wx
    del p00, p10
    p01, p11 = im[y1, x0].astype(np.float32), im[y1, x1].astype(np.float32)
    if interp == 'edge-aware':
        np.minimum(local_minimum, p01, out=local_minimum)
        np.minimum(local_minimum, p11, out=local_minimum)
        np.maximum(local_maximum, p01, out=local_maximum)
        np.maximum(local_maximum, p11, out=local_maximum)
    bot = p01 * (1.0 - wx) + p11 * wx
    del p01, p11
    bilinear = top * (1.0 - wy) + bot * wy
    if interp == 'bilinear':
        return bilinear
    # Avoid materialising Mx4x3 corners. At multi-million-point scale that
    # temporary dominates edge-aware sampling memory and reduction time.
    local_range = (local_maximum - local_minimum).max(axis=1)
    ui = np.clip(np.round(uf).astype(np.int64), 0, width - 1)
    vi = np.clip(np.round(vf).astype(np.int64), 0, height - 1)
    use_nearest = local_range > edge_threshold
    bilinear[use_nearest] = im[vi[use_nearest], ui[use_nearest]]
    return bilinear


def _median_luminance(img: np.ndarray, exclusion_mask=None) -> float:
    """Median luminance of mono or RGB(A) image data."""
    arr = np.asarray(img, dtype=np.float32)
    if exclusion_mask is not None:
        excluded = np.asarray(exclusion_mask, dtype=bool)
        if excluded.shape != arr.shape[:2]:
            raise ValueError('exclusion mask must match image dimensions')
        if excluded.all():
            return 0.0  # No evidence for an exposure estimate.
        arr = arr[~excluded][:, None]
    if arr.ndim == 2:
        return float(np.median(arr))
    if arr.ndim != 3 or arr.shape[2] == 0:
        raise ValueError(f'image must be HxW or HxWxC, got {arr.shape}')
    if arr.shape[2] == 1:
        return float(np.median(arr[:, :, 0]))
    coeff = np.asarray([0.299, 0.587, 0.114], dtype=np.float32)
    return float(np.median(np.tensordot(arr[:, :, :3], coeff, axes=([-1], [0]))))


def estimate_radial_vignette_gains(images, width: int, height: int, *,
                                   cx: Optional[float] = None,
                                   cy: Optional[float] = None,
                                   bins: int = 32, sample_stride: int = 8,
                                   gain_limit: float = 2.5,
                                   exclusion_masks=None) -> np.ndarray:
    """Estimate one robust radial luminance correction shared by all views.

    Each image is normalised by its central luminance before annular profiles
    are combined, which separates per-view exposure from a lens-fixed radial
    falloff. The returned curve only brightens, is monotonically non-decreasing,
    and is clamped by ``gain_limit``. It therefore cannot amplify dark corners
    without a caller explicitly choosing a limit above one.
    """
    if bins < 4:
        raise ValueError('bins must be >= 4')
    if sample_stride < 1:
        raise ValueError('sample_stride must be >= 1')
    if gain_limit < 1.0:
        raise ValueError('gain_limit must be >= 1')
    if len(images) == 0:
        return np.ones(bins, dtype=np.float32)
    centre_x = (width - 1) * 0.5 if cx is None else float(cx)
    centre_y = (height - 1) * 0.5 if cy is None else float(cy)
    ys = np.arange(0, height, sample_stride, dtype=np.float32)
    xs = np.arange(0, width, sample_stride, dtype=np.float32)
    xx, yy = np.meshgrid(xs, ys)
    corner_radius = max(
        np.hypot(x - centre_x, y - centre_y)
        for x in (0.0, width - 1.0) for y in (0.0, height - 1.0))
    radius = np.hypot(xx - centre_x, yy - centre_y) / max(corner_radius, 1.0)
    bin_ids = np.minimum((radius * bins).astype(np.intp), bins - 1)
    central = radius <= 0.2
    profiles = []
    coeff = np.asarray([0.299, 0.587, 0.114], dtype=np.float32)
    for view_index, image in enumerate(images):
        arr = np.asarray(image, dtype=np.float32)
        if arr.shape[:2] != (height, width):
            raise ValueError('all images must match width and height')
        if arr.ndim == 2:
            lum = arr
        elif arr.ndim == 3 and arr.shape[2] == 1:
            lum = arr[:, :, 0]
        elif arr.ndim == 3 and arr.shape[2] >= 3:
            lum = np.tensordot(arr[:, :, :3], coeff, axes=([-1], [0]))
        else:
            raise ValueError(f'image must be HxW or HxWxC, got {arr.shape}')
        sampled = lum[::sample_stride, ::sample_stride]
        usable = np.ones(sampled.shape, dtype=bool)
        if exclusion_masks is not None and exclusion_masks[view_index] is not None:
            excluded = np.asarray(exclusion_masks[view_index], dtype=bool)
            if excluded.shape != (height, width):
                raise ValueError('exclusion mask must match image dimensions')
            usable &= ~excluded[::sample_stride, ::sample_stride]
        reference_pixels = sampled[central & usable]
        if not reference_pixels.size:
            continue
        reference = float(np.median(reference_pixels))
        if reference <= 1.0e-6:
            continue
        profile = np.asarray([
            np.median(sampled[(bin_ids == index) & usable]) / reference
            if np.any((bin_ids == index) & usable) else np.nan
            for index in range(bins)
        ], dtype=np.float32)
        valid_bins = np.flatnonzero(np.isfinite(profile))
        if valid_bins.size:
            profile = np.interp(
                np.arange(bins), valid_bins, profile[valid_bins]).astype(np.float32)
        profiles.append(profile)
    if not profiles:
        return np.ones(bins, dtype=np.float32)
    radial_profile = np.median(np.stack(profiles), axis=0)
    radial_profile = np.where(
        np.isfinite(radial_profile) & (radial_profile > 1.0e-6),
        radial_profile, 1.0)
    gains = 1.0 / radial_profile
    gains = np.convolve(np.pad(gains, (1, 1), mode='edge'),
                        np.asarray([0.25, 0.5, 0.25]), mode='valid')
    central_gain = float(np.median(gains[:max(1, bins // 5)]))
    gains /= max(central_gain, 1.0e-6)
    # Lens vignetting is an outer-field effect. Pin the inner 60% to identity
    # so scene-content outliers in a tiny central annulus cannot be propagated
    # across the whole curve by the monotonic constraint below.
    gains[:int(np.ceil(0.6 * bins))] = 1.0
    gains = np.maximum.accumulate(np.clip(gains, 1.0, gain_limit))
    return gains.astype(np.float32)


def estimate_voxel_normals(points: np.ndarray, *, voxel: float = 0.12,
                           min_points: int = 6) -> np.ndarray:
    """Estimate deterministic surface normals from per-voxel covariance.

    The dense coloured maps contain millions of points, making a per-point
    k-nearest-neighbour fit unnecessarily expensive.  Points are instead
    grouped into metric voxels and share the least-variance eigenvector of
    their voxel covariance.  Voxels without enough support receive a zero
    normal, which callers treat as "confidence unavailable" rather than
    inventing an orientation.
    """
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f'points must be Nx3, got {xyz.shape}')
    if voxel <= 0.0:
        raise ValueError('voxel must be > 0')
    if min_points < 3:
        raise ValueError('min_points must be >= 3')
    if len(xyz) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    keys = np.floor(xyz / float(voxel)).astype(np.int64)
    _, inverse, counts = np.unique(
        keys, axis=0, return_inverse=True, return_counts=True)
    groups = len(counts)
    sums = np.stack([
        np.bincount(inverse, weights=xyz[:, axis], minlength=groups)
        for axis in range(3)
    ], axis=1)
    means = sums / counts[:, None]
    centred = xyz - means[inverse]
    covariance = np.empty((groups, 3, 3), dtype=np.float64)
    for row in range(3):
        for col in range(row, 3):
            value = np.bincount(
                inverse, weights=centred[:, row] * centred[:, col],
                minlength=groups) / counts
            covariance[:, row, col] = value
            covariance[:, col, row] = value
    normals_by_group = np.zeros((groups, 3), dtype=np.float64)
    supported = counts >= int(min_points)
    if supported.any():
        _, eigenvectors = np.linalg.eigh(covariance[supported])
        normals_by_group[supported] = eigenvectors[:, :, 0]
    return normals_by_group[inverse].astype(np.float32)


def estimate_overlap_rgb_gains(points: np.ndarray, viewmats: np.ndarray,
                               K: np.ndarray, images, width: int, height: int,
                               *, gain_limit: float = 1.5,
                               sample_limit: int = 50000,
                               min_shared: int = 64,
                               neighbour_span: int = 8,
                               regularization: float = 256.0,
                               exclusion_masks=None) -> np.ndarray:
    """Solve per-view RGB gains from shared, visible 3D observations.

    For nearby image pairs, the same unoccluded LiDAR points provide direct
    colour correspondences.  Median log-RGB ratios form a small pose-order
    graph whose least-squares solution removes exposure and white-balance
    discontinuities without assuming that unrelated images depict equally
    bright scene content.  Gains are centred per channel and symmetrically
    clamped.  A disconnected or correspondence-free graph returns identity.
    """
    xyz = np.asarray(points, dtype=np.float64)
    views = np.asarray(viewmats, dtype=np.float64)
    if len(views) != len(images):
        raise ValueError('viewmats and images must have equal length')
    if gain_limit < 1.0:
        raise ValueError('gain_limit must be >= 1')
    if sample_limit < 1 or min_shared < 1 or neighbour_span < 1:
        raise ValueError('sample_limit, min_shared and neighbour_span must be >= 1')
    if regularization < 0.0:
        raise ValueError('regularization must be >= 0')
    if len(images) == 0 or len(xyz) == 0:
        return np.ones((len(images), 3), dtype=np.float32)
    if len(xyz) > sample_limit:
        choose = np.linspace(0, len(xyz) - 1, sample_limit).astype(np.intp)
        xyz = xyz[choose]
    observations = []
    for view_index, (vm, image) in enumerate(zip(views, images)):
        cam = xyz @ vm[:3, :3].T + vm[:3, 3]
        z = cam[:, 2]
        uf, vf = project_camera_pixels(cam, K)
        u = np.round(uf).astype(np.int64)
        v = np.round(vf).astype(np.int64)
        inside = ((z > 1.0e-6) & (u >= 0) & (u < width) &
                  (v >= 0) & (v < height))
        ids = np.flatnonzero(inside)
        if not len(ids):
            observations.append((ids, np.zeros((0, 3), dtype=np.float32)))
            continue
        pixel = v[ids] * int(width) + u[ids]
        depth = np.full(int(width) * int(height), np.inf, dtype=np.float32)
        np.minimum.at(depth, pixel, z[ids].astype(np.float32))
        visible = z[ids] <= depth[pixel] + 0.15 + 0.02 * z[ids]
        ids = ids[visible]
        if exclusion_masks is not None and exclusion_masks[view_index] is not None:
            excluded = np.asarray(exclusion_masks[view_index], dtype=bool)
            if excluded.shape != (height, width):
                raise ValueError('exclusion mask must match image dimensions')
            ids = ids[~excluded[v[ids], u[ids]]]
        colours = _sample_pixels(
            image, uf[ids], vf[ids], width, height, 'nearest', 48.0)
        if colours.shape[1] == 1:
            colours = np.repeat(colours, 3, axis=1)
        observations.append((ids, colours))

    rows = []
    targets = []
    for right in range(len(observations)):
        ids_r, colours_r = observations[right]
        for left in range(max(0, right - neighbour_span), right):
            ids_l, colours_l = observations[left]
            common, il, ir = np.intersect1d(
                ids_l, ids_r, assume_unique=True, return_indices=True)
            if len(common) < min_shared:
                continue
            a, b = colours_l[il], colours_r[ir]
            usable = np.all((a > 5.0) & (a < 250.0) &
                            (b > 5.0) & (b < 250.0), axis=1)
            if int(usable.sum()) < min_shared:
                continue
            row = np.zeros(len(observations), dtype=np.float64)
            row[left], row[right] = -1.0, 1.0
            rows.append(row)
            targets.append(np.median(
                np.log(a[usable]) - np.log(b[usable]), axis=0))
    if not rows:
        return np.ones((len(images), 3), dtype=np.float32)
    if regularization > 0.0:
        # Local pair ratios contain real changes of scene content.  Without an
        # absolute prior, tiny biases accumulate around a long walking loop and
        # drive almost every gain into its clamp.  The existing, conservative
        # luminance-only normalisation is a stable prior; overlap RGB ratios
        # refine white balance locally without being allowed to drift away.
        medians = np.asarray([
            _median_luminance(image, None if exclusion_masks is None else exclusion_masks[i])
            for i, image in enumerate(images)])
        valid = medians > 1.0e-6
        scalar = np.ones(len(images), dtype=np.float64)
        if valid.any():
            scalar[valid] = np.median(medians[valid]) / medians[valid]
        scalar = np.clip(scalar, 1.0 / gain_limit, gain_limit)
        weight = np.sqrt(float(regularization))
        for index in range(len(images)):
            row = np.zeros(len(images), dtype=np.float64)
            row[index] = weight
            rows.append(row)
            targets.append(np.full(3, weight * np.log(scalar[index])))
    else:
        # Gauge constraint used by exact synthetic tests and ablations.
        rows.append(np.ones(len(images), dtype=np.float64) / len(images))
        targets.append(np.zeros(3, dtype=np.float64))
    design = np.stack(rows)
    target = np.stack(targets)
    log_gain = np.stack([
        np.linalg.lstsq(design, target[:, channel], rcond=None)[0]
        for channel in range(3)
    ], axis=1)
    gains = np.exp(log_gain)
    gains = np.clip(gains, 1.0 / gain_limit, gain_limit)
    return gains.astype(np.float32)


def observed_color_medoids(samples: np.ndarray, chunk: int = 20000) -> np.ndarray:
    """Choose the observed RGB sample nearest all other samples per point.

    A channel-wise median can synthesize a colour that no camera observed. For
    example, red, green, and blue samples produce black. The L1 medoid retains
    median-like outlier resistance while guaranteeing that every output row is
    one of the input camera observations. Per-channel sorting and prefix sums
    compute the L1 scores with temporary storage linear in the sample count.
    """
    values = np.asarray(samples, dtype=np.uint8)
    if values.ndim != 3 or values.shape[2] != 3 or values.shape[1] < 1:
        raise ValueError(f'samples must be NxSx3 with S >= 1, got {values.shape}')
    if chunk < 1:
        raise ValueError('chunk must be >= 1')
    out = np.empty((values.shape[0], 3), dtype=np.uint8)
    count = values.shape[1]
    weights = 2 * np.arange(count, dtype=np.int32) - count + 2
    for start in range(0, len(values), chunk):
        block = values[start:start + chunk]
        rows = np.arange(len(block))[:, None]
        scores = np.zeros(block.shape[:2], dtype=np.int32)
        for channel in range(3):
            order = np.argsort(block[:, :, channel], axis=1)
            ordered = np.take_along_axis(
                block[:, :, channel], order, axis=1).astype(np.int32)
            prefix = np.cumsum(ordered, axis=1, dtype=np.int32)
            # Sum distances to values on either side in sorted order.
            costs = ordered * weights + prefix[:, -1:] - 2 * prefix
            scores[rows, order] += costs
        # Scatter back before argmin to preserve the first-observation tie rule.
        choice = np.argmin(scores, axis=1)
        out[start:start + len(block)] = block[np.arange(len(block)), choice]
    return out


def undistort_equidistant(pixels, K, distortion, iterations=50):
    """Normalized rays of equidistant (fisheye) pixels, principal branch.

    Newton iteration on theta_d = theta * (1 + k1 theta^2 + ... + k4 theta^8),
    started at theta_d, as cv2.fisheye.undistortPoints does. Written out
    because the OpenCV of ROS Humble (4.5) takes no termination criteria
    there, and the fold-back check needs a tight inverse.
    """
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    k1, k2, k3, k4 = np.resize(np.asarray(distortion, dtype=np.float64), 4)
    distorted = np.c_[(pixels[:, 0] - K[0, 2] - K[0, 1] *
                       (pixels[:, 1] - K[1, 2]) / K[1, 1]) / K[0, 0],
                      (pixels[:, 1] - K[1, 2]) / K[1, 1]]
    theta_d = np.hypot(distorted[:, 0], distorted[:, 1])
    theta = theta_d.copy()
    for _ in range(iterations):
        t2 = theta * theta
        poly = 1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4)))
        slope = 1 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * (7 * k3 + t2 * 9 * k4)))
        with np.errstate(divide='ignore', invalid='ignore'):
            step = (theta * poly - theta_d) / slope
        theta = theta - np.nan_to_num(step)
        if np.all(np.abs(step) < 1e-15):
            break
    with np.errstate(divide='ignore', invalid='ignore'):
        scale = np.where(theta_d > 1e-15, np.tan(theta) / theta_d, 1.0)
    return distorted * scale[:, None]


def project_camera_pixels(cam, K, *, distortion=None, distortion_model='plumb_bob'):
    """Project camera-frame points, optionally into an unrectified image.

    None means pinhole/rectified. Fisheye with zero coefficients still uses
    the equidistant projection. OpenCV is needed only for distorted images.
    """
    cam = np.asarray(cam, dtype=np.float64)
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    if distortion is not None:
        if distortion_model not in ('plumb_bob', 'rational_polynomial',
                                    'equidistant', 'fisheye'):
            raise ValueError(f'unsupported distortion model: {distortion_model}')
        fisheye = distortion_model in ('equidistant', 'fisheye')
        d = np.asarray(distortion, dtype=np.float64)
        if fisheye or np.any(d):
            import cv2
            if fisheye and d.size == 0:
                d = np.zeros(4)
            uv = np.full((len(cam), 2), -1.0)
            valid = np.isfinite(cam).all(axis=1) & (cam[:, 2] > 1e-6)
            if valid.any():
                project = cv2.fisheye.projectPoints if fisheye else cv2.projectPoints
                uv[valid] = project(
                    cam[valid, None, :], np.zeros(3), np.zeros(3), K, d)[0].reshape(-1, 2)
            # Polynomial extensions can fold rays far outside the lens field
            # back into the image. Only keep the branch recovered by the lens
            # inverse; otherwise a spurious foreground point can occlude a
            # valid point before any color is sampled.
            indices = np.flatnonzero(valid & np.isfinite(uv).all(axis=1))
            if indices.size:
                pixels = uv[indices, None, :]
                criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-12)
                if fisheye:
                    rays = undistort_equidistant(pixels.reshape(-1, 2), K, d)
                else:
                    rays = cv2.undistortPointsIter(pixels, K, d, None, None, criteria)
                rays = rays.reshape(-1, 2)
                expected = cam[indices, :2] / cam[indices, 2, None]
                # Numerical inverse tolerance in normalized ray coordinates,
                # independent of scene colors, depth gates and GT.
                consistent = (np.isfinite(rays).all(axis=1)
                              & np.isclose(rays, expected, rtol=1e-6, atol=1e-8).all(axis=1))
                uv[indices[~consistent]] = -1.0
            uv = np.nan_to_num(uv, nan=-1.0, posinf=-1.0, neginf=-1.0)
            return uv[:, 0], uv[:, 1]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = np.nan_to_num(K[0, 0] * cam[:, 0] / cam[:, 2] + K[0, 2],
                          nan=-1.0, posinf=-1.0, neginf=-1.0)
        v = np.nan_to_num(K[1, 1] * cam[:, 1] / cam[:, 2] + K[1, 2],
                          nan=-1.0, posinf=-1.0, neginf=-1.0)
    return u, v


def colorize_by_projection_robust(points: np.ndarray, viewmats: np.ndarray,
                                  K: np.ndarray, images, width: int, height: int,
                                  default_rgb=(128, 128, 128), *,
                                  zbuf_bin: int = 1, depth_tol: float = 0.15,
                                  max_samples: int = 12,
                                  normalize_exposure: bool = True,
                                  exposure_scale_limit: float = 1.5,
                                  interp: str = 'edge-aware',
                                  edge_threshold: float = 48.0,
                                  prefer_near: bool = True,
                                  observation_mask: Optional[np.ndarray] = None,
                                  image_margin: int = 0,
                                  vignette_gain_limit: float = 1.0,
                                  overlap_color_balance: bool = False,
                                  point_normals: Optional[np.ndarray] = None,
                                  min_view_cosine: float = 0.0,
                                  min_projected_scale: float = 0.0,
                                  view_score_power: float = 0.0,
                                  occlusion_margin_px: int = 0,
                                  depth_edge_margin_px: int = 0,
                                  depth_edge_tolerance: float = 0.15,
                                  depth_edge_relative_tolerance: float = 0.02,
                                  exclusion_masks: Optional[
                                      Sequence[Optional[np.ndarray]]] = None,
                                  dynamic_mask_margin_px: int = 0,
                                  calibration: Optional[dict] = None,
                                  view_timestamps: Optional[
                                      Sequence[float]] = None,
                                  calibration_sigma_multiplier: float = 0.0,
                                  maximum_uncertainty_margin_px: int = 12,
                                  distortion=None,
                                  distortion_model: str = 'plumb_bob',
                                  sky_up: Optional[np.ndarray] = None,
                                  sky_min_elevation_deg: float = 2.0,
                                  sky_fill_radius_m: float = 0.0,
                                  sky_fill_planar_voxel_m: float = 0.3,
                                  return_counts: bool = False,
                                  return_diagnostics: bool = False):
    """Occlusion-aware, exposure-normalised, median-robust point colorization.

    ``colorize_by_projection`` averages every view a point lands in, so points
    behind walls or machines pick up the colour of whatever occludes them and
    auto-exposure differences wash the average out. This variant first builds a
    per-view z-buffer from the point cloud itself (one pixel per bin by default)
    and only samples views where the point sits within ``depth_tol`` (plus
    2 % of range) of the nearest depth in its bin; each image is scaled toward
    the global median luminance (``normalize_exposure``), with the gain clamped
    symmetrically by ``exposure_scale_limit`` so genuine scene lighting is not
    flattened; and the
    final colour is the RGB medoid over up to ``max_samples`` valid samples,
    which rejects residual specular / motion-blur outliers without synthesizing
    a colour that no camera actually observed.

    A value above one for ``zbuf_bin`` trades memory for a coarse occlusion
    approximation. It can falsely hide a surface when an unrelated nearer point
    lands in a neighbouring pixel, so final map generation should keep the
    one-pixel default.

    Quality knobs: ``interp='edge-aware'`` blends sub-pixel samples on smooth
    patches and snaps to a real pixel across strong RGB edges, avoiding mixed
    foreground/background colours. ``edge_threshold`` controls that switch.
    ``prefer_near=True`` keeps,
    once a point has ``max_samples`` observations, the *nearest* ones (a new
    closer view evicts the farthest stored sample) so colour comes from the
    highest-resolution, least-foreshortened views rather than whichever happened
    to be visited first. ``image_margin`` ignores samples within that many
    pixels of the image border, where lens vignetting darkens the pixels that
    per-view global exposure gains cannot repair; points near one view's border
    stay colourable from the views that see them more centrally.
    ``vignette_gain_limit > 1`` estimates a shared radial luminance profile,
    pins the inner 60 % of the image radius to unity, and brightens only the
    outer field up to the requested clamp. The default 1 disables correction.

    Returns ``(rgb uint8 (N,3), seen bool (N,))``, or with ``return_counts`` the
    triple ``(rgb, seen, counts uint16 (N,))`` giving each point's surviving
    sample count (a colour-confidence signal). Unseen points get ``default_rgb``.

    ``distortion`` supplies raw-image coefficients shared by all views;
    ``None`` preserves pinhole projection. ``distortion_model`` follows
    CameraInfo (plumb_bob, rational_polynomial, equidistant/fisheye).
    Raw-image projection supports pixel sampling and z-buffer occlusion;
    pinhole overlap, scale and calibration-uncertainty weighting require
    rectified images instead.

    Geometry-aware fusion is opt-in. ``occlusion_margin_px`` tests nearby
    z-buffer cells so a foreground silhouette suppresses background colour
    samples in adjacent pixels. ``depth_edge_margin_px`` rejects both sides of
    a measured depth discontinuity. Per-frame ``exclusion_masks`` remove
    dynamic image regions. Accepted calibration uncertainty can expand all
    three guards per observation after propagation through range, focal length,
    and camera motion. ``return_diagnostics`` appends rejection counters.

    ``sky_up`` (a world up vector) enables sky-sample rejection. Thin branches
    and canopy edges often project onto the sky between them, and the median
    then paints them white or blue. A sample is sky-like when its viewing ray
    rises more than ``sky_min_elevation_deg`` above the horizon and its pixel
    is bright and unsaturated or blue (:func:`sky_like_colors`). A point keeps
    only its other samples when it has any, so a genuinely white facade seen
    against nothing else stays white. It changes colours only: which points
    are seen, and the returned counts, still include the sky-like samples.

    ``sky_fill_radius_m > 0`` (with ``sky_up``) also recolours points whose
    samples were all sky-like, such as bare branches that every camera saw
    only against the sky: unless their ``sky_fill_planar_voxel_m`` voxel is
    planar (a facade), they take the median colour of their eight nearest
    points within that radius that were coloured from other samples
    (:func:`fill_colors_from_neighbours`).
    """
    if distortion is not None and (
            overlap_color_balance or calibration_sigma_multiplier > 0.0
            or min_projected_scale > 0.0 or view_score_power > 0.0):
        raise ValueError('rectify images before using pinhole overlap, scale, '
                         'or calibration-uncertainty weighting')
    points = np.asarray(points, dtype=np.float64)
    n = points.shape[0]
    if observation_mask is not None:
        observation_mask = np.asarray(observation_mask, dtype=bool)
        if observation_mask.shape != (n, len(images)):
            raise ValueError('observation_mask must be points x images')
    if point_normals is not None:
        point_normals = np.asarray(point_normals, dtype=np.float64)
        if point_normals.shape != (n, 3):
            raise ValueError('point_normals must be Nx3')
    if exclusion_masks is not None:
        if len(exclusion_masks) != len(images):
            raise ValueError('exclusion_masks must match the image count')
        for mask in exclusion_masks:
            if mask is not None and np.asarray(mask).shape != (height, width):
                raise ValueError('each exclusion mask must be HxW')
    rgb = np.tile(np.asarray(default_rgb, dtype=np.uint8), (n, 1))
    counts = np.zeros(n, dtype=np.uint16)
    diagnostics = {
        'projected': 0, 'rejected_zbuffer': 0,
        'rejected_occlusion': 0, 'rejected_occlusion_margin': 0,
        'rejected_depth_edge': 0, 'rejected_dynamic_mask': 0,
        'accepted_samples': 0, 'rejected_sky': 0, 'sky_filled': 0,
    }
    if n == 0 or max_samples <= 0:
        seen = np.zeros(n, dtype=bool)
        result = (rgb, seen, counts) if return_counts else (rgb, seen)
        return result + (diagnostics,) if return_diagnostics else result
    if zbuf_bin < 1:
        raise ValueError('zbuf_bin must be >= 1')
    if exposure_scale_limit < 1.0:
        raise ValueError('exposure_scale_limit must be >= 1')
    if vignette_gain_limit < 1.0:
        raise ValueError('vignette_gain_limit must be >= 1')
    if image_margin < 0 or 2 * image_margin >= min(int(width), int(height)):
        raise ValueError('image_margin must be >= 0 and leave a usable '
                         'central image region')
    if not 0.0 <= min_view_cosine <= 1.0:
        raise ValueError('min_view_cosine must be in [0, 1]')
    if min_projected_scale < 0.0 or view_score_power < 0.0:
        raise ValueError('min_projected_scale and view_score_power must be >= 0')
    geometry_values = (
        occlusion_margin_px, depth_edge_margin_px, dynamic_mask_margin_px,
        maximum_uncertainty_margin_px)
    if any(value < 0 for value in geometry_values):
        raise ValueError('geometry-aware pixel margins must be non-negative')
    if (depth_edge_tolerance < 0.0 or
            depth_edge_relative_tolerance < 0.0 or
            calibration_sigma_multiplier < 0.0):
        raise ValueError('geometry tolerances and calibration sigma must be '
                         'non-negative')
    geometry_enabled = bool(
        occlusion_margin_px or depth_edge_margin_px or
        exclusion_masks is not None or calibration_sigma_multiplier)
    if geometry_enabled and zbuf_bin != 1:
        raise ValueError('geometry-aware fusion requires a one-pixel z-buffer')
    if calibration_sigma_multiplier > 0.0 and view_timestamps is None:
        raise ValueError('calibration uncertainty requires view timestamps')
    if sky_fill_radius_m < 0.0 or sky_fill_planar_voxel_m <= 0.0:
        raise ValueError('sky fill radius must be >= 0 and voxel > 0')
    if sky_fill_radius_m > 0.0 and sky_up is None:
        raise ValueError('sky fill requires sky_up')
    if sky_up is not None:
        sky_up = np.asarray(sky_up, dtype=np.float64).reshape(3)
        if not np.linalg.norm(sky_up) > 0.0:
            raise ValueError('sky_up must be a nonzero 3-vector')
        sky_up = sky_up / np.linalg.norm(sky_up)
        sky_min_sine = float(np.sin(np.deg2rad(sky_min_elevation_deg)))

    linear_speeds = np.zeros(len(images), dtype=np.float64)
    angular_speeds = np.zeros(len(images), dtype=np.float64)
    if calibration_sigma_multiplier > 0.0:
        linear_speeds, angular_speeds = gaf.camera_motion_rates(
            np.asarray(viewmats), view_timestamps)

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    zb_w = (int(width) + zbuf_bin - 1) // zbuf_bin
    zb_h = (int(height) + zbuf_bin - 1) // zbuf_bin
    # Each image contributes at most one observation per point.
    capacity = min(int(max_samples), len(images))
    samples = np.empty((n, capacity, 3), dtype=np.uint8)
    sky_flags = (np.zeros((n, capacity), dtype=bool)
                 if sky_up is not None else None)
    # Only one ranking criterion is used for all observations in this call.
    rank_by_quality = point_normals is not None or min_projected_scale > 0.0
    sample_rank = (np.full(
        (n, capacity), -np.inf if rank_by_quality else np.inf,
        dtype=np.float32) if prefer_near and capacity < len(images) else None)
    vignette_gains = None
    vignette_radius = 1.0
    if vignette_gain_limit > 1.0:
        vignette_gains = estimate_radial_vignette_gains(
            images, width, height, cx=cx, cy=cy,
            gain_limit=vignette_gain_limit, exclusion_masks=exclusion_masks)
        vignette_radius = max(
            np.hypot(x - cx, y - cy)
            for x in (0.0, width - 1.0) for y in (0.0, height - 1.0))

    scales = np.ones((len(images), 3), dtype=np.float32)
    if overlap_color_balance:
        scales = estimate_overlap_rgb_gains(
            points, viewmats, K, images, width, height,
            gain_limit=exposure_scale_limit, exclusion_masks=exclusion_masks)
    elif normalize_exposure:
        meds = np.asarray([
            _median_luminance(img, None if exclusion_masks is None else exclusion_masks[i])
            for i, img in enumerate(images)], dtype=np.float32)
        valid = meds > 1.0e-6
        if valid.any():
            scalar = float(np.median(meds[valid])) / meds[valid]
            scalar = np.clip(scalar, 1.0 / exposure_scale_limit,
                             exposure_scale_limit)
            scales[valid] = scalar[:, None]

    ids = np.arange(n)
    for vi, (vm, img) in enumerate(zip(viewmats, images)):
        vm = np.asarray(vm, dtype=np.float64)
        cam = points @ vm[:3, :3].T + vm[:3, 3]
        z = cam[:, 2]
        uf, vf = project_camera_pixels(
            cam, K, distortion=distortion, distortion_model=distortion_model)
        u = np.round(uf).astype(np.int64)
        v = np.round(vf).astype(np.int64)
        inb = (z > 1e-6) & (u >= 0) & (u < width) & (v >= 0) & (v < height)
        if not inb.any():
            continue
        diagnostics['projected'] += int(inb.sum())
        zbin = (v[inb] // zbuf_bin) * zb_w + (u[inb] // zbuf_bin)
        zbuf = np.full(zb_w * zb_h, np.inf, dtype=np.float32)
        projected_z = z[inb].astype(np.float32)
        np.minimum.at(zbuf, zbin, projected_z)
        uncertainty_margin = gaf.calibration_pixel_radii(
            projected_z, max(float(fx), float(fy)), calibration,
            linear_speed=linear_speeds[vi],
            angular_speed=angular_speeds[vi],
            sigma_multiplier=calibration_sigma_multiplier,
            maximum_radius=maximum_uncertainty_margin_px)
        zbuffer_image = zbuf.reshape(zb_h, zb_w)
        occlusion_radii = uncertainty_margin + int(occlusion_margin_px)
        zbuffer_visible = projected_z <= (
            zbuf[zbin] + depth_tol + 0.02 * projected_z)
        diagnostics['rejected_zbuffer'] += int((~zbuffer_visible).sum())
        if np.any(occlusion_radii > 0):
            local_minimum, _, _ = gaf.neighborhood_depth_statistics(
                zbuffer_image, u[inb], v[inb], occlusion_radii)
            visible = projected_z <= (
                local_minimum + depth_tol + 0.02 * projected_z)
            diagnostics['rejected_occlusion_margin'] += int(
                (zbuffer_visible & ~visible).sum())
        else:
            visible = zbuffer_visible
        diagnostics['rejected_occlusion'] += int((~visible).sum())

        edge_radii = uncertainty_margin + int(depth_edge_margin_px)
        if np.any(edge_radii > 0):
            local_minimum, local_maximum, support = \
                gaf.neighborhood_depth_statistics(
                    zbuffer_image, u[inb], v[inb], edge_radii)
            discontinuity = (
                (support >= 2) &
                ((local_maximum - local_minimum) >
                 depth_edge_tolerance +
                 depth_edge_relative_tolerance * local_minimum))
            diagnostics['rejected_depth_edge'] += int(
                (visible & discontinuity).sum())
            visible &= ~discontinuity

        if exclusion_masks is not None and exclusion_masks[vi] is not None:
            mask_radii = uncertainty_margin + int(dynamic_mask_margin_px)
            mask_u, mask_v = u[inb], v[inb]
            if interp != 'nearest':
                # Bilinear (including edge-aware fallback) may mix four pixels.
                # Check its support conservatively before inspecting RGB edges;
                # a masked neighbour must not leak into an unmasked sample.
                x0 = np.clip(np.floor(uf[inb]).astype(np.int64), 0, width - 1)
                x1 = np.clip(np.ceil(uf[inb]).astype(np.int64), 0, width - 1)
                y0 = np.clip(np.floor(vf[inb]).astype(np.int64), 0, height - 1)
                y1 = np.clip(np.ceil(vf[inb]).astype(np.int64), 0, height - 1)
                mask_u = np.stack((x0, x1, x0, x1), axis=1).ravel()
                mask_v = np.stack((y0, y0, y1, y1), axis=1).ravel()
                mask_radii = np.repeat(mask_radii, 4)
            dynamic = gaf.mask_neighborhood_any(
                exclusion_masks[vi], mask_u, mask_v, mask_radii)
            if interp != 'nearest':
                dynamic = dynamic.reshape(-1, 4).any(axis=1)
            diagnostics['rejected_dynamic_mask'] += int(
                (visible & dynamic).sum())
            visible &= ~dynamic
        if image_margin > 0:
            # The z-buffer above still uses the full frame (occlusion geometry
            # is valid to the border); only colour sampling skips the margin.
            visible &= ((u[inb] >= image_margin) &
                        (u[inb] < width - image_margin) &
                        (v[inb] >= image_margin) &
                        (v[inb] < height - image_margin))
        cand = ids[inb][visible]  # unique point ids seen (unoccluded) this view
        cand_z = projected_z[visible]
        if observation_mask is not None:
            keep = observation_mask[cand, vi]
            cand = cand[keep]
            cand_z = cand_z[keep]
        if rank_by_quality:
            quality = (float(fx) / np.maximum(cand_z, 1.0e-6)).astype(np.float32)
            if calibration_sigma_multiplier > 0.0 and cand.size:
                kept_uncertainty = uncertainty_margin[visible]
                if observation_mask is not None:
                    kept_uncertainty = kept_uncertainty[keep]
                quality /= 1.0 + kept_uncertainty.astype(np.float32)
        if point_normals is not None and cand.size:
            camera_centre = -vm[:3, :3].T @ vm[:3, 3]
            sight = camera_centre[None, :] - points[cand]
            sight /= np.maximum(np.linalg.norm(sight, axis=1, keepdims=True),
                                1.0e-9)
            normal_length = np.linalg.norm(point_normals[cand], axis=1)
            cosine = np.abs(np.sum(point_normals[cand] * sight, axis=1))
            cosine = np.where(normal_length > 0.5, cosine, 1.0)
            confidence = ((cosine >= min_view_cosine) &
                          (quality >= min_projected_scale))
            cand, cand_z, quality = (cand[confidence], cand_z[confidence],
                                     quality[confidence])
            cosine = cosine[confidence]
            if view_score_power > 0.0:
                # Incidence angle refines the existing projected-resolution
                # ranking but cannot make a much farther view beat a close one.
                # A hard cosine product caused neighbouring planar points to
                # select different distant frames and increased colour noise.
                angular = np.power(cosine, view_score_power).astype(np.float32)
                quality *= 0.75 + 0.25 * angular
        elif min_projected_scale > 0.0:
            confidence = quality >= min_projected_scale
            cand, cand_z, quality = (cand[confidence], cand_z[confidence],
                                     quality[confidence])
        if cand.size == 0:
            continue
        diagnostics['accepted_samples'] += int(cand.size)
        cols = _sample_pixels(
            img, uf[cand], vf[cand], width, height, interp, edge_threshold)
        if sky_flags is not None:
            camera_centre = -vm[:3, :3].T @ vm[:3, 3]
            ray = points[cand] - camera_centre[None, :]
            elevation = (ray @ sky_up) / np.maximum(
                np.linalg.norm(ray, axis=1), 1.0e-9)
            sky = (elevation > sky_min_sine) & sky_like_colors(cols)
        if vignette_gains is not None:
            radius = np.hypot(uf[cand] - cx, vf[cand] - cy) / vignette_radius
            gain = np.interp(
                radius, np.linspace(0.0, 1.0, len(vignette_gains)),
                vignette_gains).astype(np.float32)
            cols *= gain[:, None]
        cols = np.clip(cols * scales[vi][None, :], 0.0, 255.0).astype(np.uint8)

        rank = quality if rank_by_quality else cand_z
        # Points with room: append into the next free slot.
        room = counts[cand] < max_samples
        if room.any():
            rc = cand[room]
            slot = counts[rc].astype(np.intp)
            samples[rc, slot, :] = cols[room]
            if sky_flags is not None:
                sky_flags[rc, slot] = sky[room]
            if sample_rank is not None:
                sample_rank[rc, slot] = rank[room]
            counts[rc] += 1
        # Full points: if enabled, evict the farthest stored sample when nearer.
        if sample_rank is not None and (~room).any():
            fc = cand[~room]
            fcols = cols[~room]
            if rank_by_quality:
                replace_slot = np.argmin(sample_rank[fc], axis=1)
                better = rank[~room] > sample_rank[fc, replace_slot]
            else:
                replace_slot = np.argmax(sample_rank[fc], axis=1)
                better = rank[~room] < sample_rank[fc, replace_slot]
            if better.any():
                fb = fc[better]
                sb = replace_slot[better]
                samples[fb, sb, :] = fcols[better]
                sample_rank[fb, sb] = rank[~room][better]
                if sky_flags is not None:
                    sky_flags[fb, sb] = sky[~room][better]

    fused = counts
    clear = None
    if sky_flags is not None:
        filled = np.arange(capacity)[None, :] < counts[:, None]
        sky_flags &= filled
        clear = counts.astype(np.int64) - sky_flags.sum(axis=1)
        mixed = np.flatnonzero((clear > 0) & (clear < counts))
        if mixed.size:
            # Move the clear samples to the front; the medoid below reads the
            # first ``fused`` slots only.
            order = np.argsort(sky_flags[mixed], axis=1, kind='stable')
            samples[mixed] = np.take_along_axis(
                samples[mixed], order[:, :, None], axis=1)
            diagnostics['rejected_sky'] = int(
                (counts[mixed] - clear[mixed]).sum())
            fused = counts.copy()
            fused[mixed] = clear[mixed].astype(np.uint16)

    seen = counts > 0
    seen_idx = np.flatnonzero(seen)
    for c in np.unique(fused[seen_idx]):
        group = seen_idx[fused[seen_idx] == c]
        rgb[group] = observed_color_medoids(samples[group, :int(c), :])
    if clear is not None and sky_fill_radius_m > 0.0:
        sky_only = seen & (clear == 0)
        if sky_only.any():
            donors = seen & ~sky_only & ~sky_like_colors(rgb)
            targets = sky_only & ~voxel_planar_mask(
                points, sky_fill_planar_voxel_m)
            rgb, filled = fill_colors_from_neighbours(
                points, rgb, targets, donors, sky_fill_radius_m)
            diagnostics['sky_filled'] = int(filled)
    result = (rgb, seen, counts) if return_counts else (rgb, seen)
    return result + (diagnostics,) if return_diagnostics else result


def voxel_planar_mask(points: np.ndarray, voxel: float, *,
                      min_points: int = 10,
                      max_ratio: float = 0.02) -> np.ndarray:
    """Flag points whose ``voxel`` cell is a populated, flat PCA patch.

    A cell is planar when its smallest covariance eigenvalue is below
    ``max_ratio`` of the eigenvalue sum. Sparse cells are never planar.
    """
    xyz = np.asarray(points, dtype=np.float64)
    if voxel <= 0.0:
        raise ValueError('voxel must be > 0')
    if len(xyz) == 0:
        return np.zeros(0, dtype=bool)
    keys = np.ascontiguousarray(np.floor(xyz / voxel).astype(np.int64))
    _, inverse, count = np.unique(
        keys.view([('k', keys.dtype, 3)]).ravel(), return_inverse=True,
        return_counts=True)
    inverse = inverse.ravel()
    centred = xyz - xyz.mean(axis=0)
    first = np.zeros((len(count), 3))
    second = np.zeros((len(count), 3, 3))
    np.add.at(first, inverse, centred)
    np.add.at(second, inverse, centred[:, :, None] * centred[:, None, :])
    mean = first / count[:, None]
    cov = second / count[:, None, None] - mean[:, :, None] * mean[:, None, :]
    eigenvalues = np.linalg.eigvalsh(cov)
    flat = ((count >= min_points) &
            (eigenvalues[:, 0] < max_ratio * np.maximum(
                eigenvalues.sum(axis=1), 1.0e-12)))
    return flat[inverse]


def fill_colors_from_neighbours(points: np.ndarray, rgb: np.ndarray,
                                targets: np.ndarray, donors: np.ndarray,
                                radius: float, *, neighbours: int = 8):
    """Give ``targets`` the median RGB of their nearest ``donors``.

    Up to ``neighbours`` donors within ``radius`` count; a target without one
    keeps its colour. Returns ``(rgb, filled_count)`` with ``rgb`` a copy.
    """
    from scipy.spatial import cKDTree

    xyz = np.asarray(points, dtype=np.float64)
    out = np.array(rgb, dtype=np.uint8, copy=True)
    target_ids = np.flatnonzero(targets)
    donor_ids = np.flatnonzero(donors)
    if target_ids.size == 0 or donor_ids.size == 0:
        return out, 0
    k = min(int(neighbours), donor_ids.size)
    distance, index = cKDTree(xyz[donor_ids]).query(
        xyz[target_ids], k=k, distance_upper_bound=float(radius))
    distance = distance.reshape(len(target_ids), k)
    index = index.reshape(len(target_ids), k)
    found = np.isfinite(distance)
    has = found[:, 0]
    if not has.any():
        return out, 0
    colours = np.where(
        found[has][..., None],
        out[donor_ids[np.minimum(index[has], donor_ids.size - 1)]].astype(
            np.float32),
        np.nan)
    out[target_ids[has]] = np.round(np.nanmedian(colours, axis=1)).astype(
        np.uint8)
    return out, int(has.sum())


def swept_corridor_mask(points: np.ndarray, path: np.ndarray, up: np.ndarray,
                        *, radius: float = 0.5, below: float = 0.8,
                        above: float = 0.4, step: float = 0.05) -> np.ndarray:
    """Flag points inside the volume a carried sensor's bearer walked through.

    Nothing static can stand where the person carrying the camera walked, so
    points within ``radius`` (m, horizontal) of the sensor ``path`` and
    between ``below`` and ``above`` (m) around its height belong to something
    that moved away first, such as a companion who stood on the route. A
    forward-looking LiDAR never looks back at that space, so free-space
    voting cannot remove them. The ground stays when ``below`` is less than
    the sensor height above it; check the height histogram first.
    ``path`` is densified to ``step`` before the nearest-sample search.
    """
    from scipy.spatial import cKDTree

    xyz = np.asarray(points, dtype=np.float64)
    path = np.asarray(path, dtype=np.float64)
    up = np.asarray(up, dtype=np.float64).reshape(3)
    if path.ndim != 2 or path.shape[1] != 3 or len(path) == 0:
        raise ValueError('path must be a nonempty Nx3 array')
    if min(radius, below, above, step) < 0.0 or step == 0.0:
        raise ValueError('corridor sizes must be non-negative and step > 0')
    if not np.linalg.norm(up) > 0.0:
        raise ValueError('up must be a nonzero vector')
    up = up / np.linalg.norm(up)
    samples = [path[:1]]
    for start, end in zip(path[:-1], path[1:]):
        count = max(1, int(np.ceil(np.linalg.norm(end - start) / step)))
        fractions = np.arange(1, count + 1)[:, None] / count
        samples.append(start + (end - start) * fractions)
    dense = np.concatenate(samples)
    reach = float(np.hypot(radius, max(below, above)))
    distance, nearest = cKDTree(dense).query(xyz, distance_upper_bound=reach)
    near = np.isfinite(distance)
    mask = np.zeros(len(xyz), dtype=bool)
    if not near.any():
        return mask
    offset = xyz[near] - dense[nearest[near]]
    height = offset @ up
    horizontal = np.linalg.norm(offset - height[:, None] * up, axis=1)
    mask[near] = ((horizontal <= radius) & (height >= -below) &
                  (height <= above))
    return mask


def sky_like_colors(rgb: np.ndarray, *, min_value: float = 0.70,
                    max_saturation: float = 0.25,
                    blue_margin: float = 12.0,
                    min_blue_value: float = 0.45) -> np.ndarray:
    """Flag RGB samples that look like sky: bright and unsaturated, or blue.

    Value and saturation follow HSV. Blue means the blue channel exceeds red
    by ``blue_margin`` and is at least green, at value ``min_blue_value`` or
    more. Colour alone also matches white walls; callers add a geometric test.
    """
    rgb = np.asarray(rgb, dtype=np.float32)
    high = rgb.max(axis=-1)
    low = rgb.min(axis=-1)
    value = high / 255.0
    saturation = np.where(high > 0.0, (high - low) / np.maximum(high, 1.0e-6),
                          0.0)
    blue = ((rgb[..., 2] > rgb[..., 0] + blue_margin) &
            (rgb[..., 2] >= rgb[..., 1]) & (value >= min_blue_value))
    return ((value >= min_value) & (saturation <= max_saturation)) | blue


def estimate_world_up(viewmats) -> np.ndarray:
    """Average camera up direction in the world frame.

    The OpenCV camera y axis points down, so up is minus the second row of
    each world-to-camera rotation. Hand-held and vehicle cameras stay roughly
    level, so the mean is a usable gravity direction even when the map frame
    itself is tilted.
    """
    rotations = np.asarray(viewmats, dtype=np.float64)[:, :3, :3]
    up = -rotations[:, 1, :].mean(axis=0)
    norm = np.linalg.norm(up)
    if not norm > 1.0e-6:
        raise ValueError('camera orientations do not define an up direction')
    return up / norm


def project_depth_maps(points: np.ndarray, viewmats, K: np.ndarray,
                       width: int, height: int) -> list:
    """Project a world point cloud into each posed camera as a sparse depth map.

    For depth-supervised 3DGS training: the LiDAR cloud carries metric geometry,
    so projecting it into every view gives a sparse per-pixel ground-truth depth
    to regularise the rendered (expected) depth against. ``viewmats`` are OpenCV
    world->camera (as ``train_gsplat.load_transforms`` returns); a point's depth
    is its camera-frame ``z``. When several points land on the same pixel only
    the nearest is kept (a per-pixel z-buffer), so a wall in front naturally
    occludes the points behind it.

    Returns a list (one entry per view) of ``(pix_idx int64 (M,), depth float32
    (M,))`` where ``pix_idx = v * width + u`` indexes the flattened image and
    ``M`` is the number of occupied pixels in that view (sparse; usually far
    fewer than the point count after the z-buffer dedups colliding pixels).
    """
    pts = np.asarray(points, dtype=np.float64)
    npix = int(width) * int(height)
    out = []
    for vm in viewmats:
        vm = np.asarray(vm, dtype=np.float64)
        cam = pts @ vm[:3, :3].T + vm[:3, 3]
        z = cam[:, 2]
        u, v = project_camera_pixels(cam, K)
        ui = np.round(u).astype(np.int64)
        vi = np.round(v).astype(np.int64)
        inb = (z > 1e-6) & (ui >= 0) & (ui < width) & (vi >= 0) & (vi < height)
        if not inb.any():
            out.append((np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.float32)))
            continue
        pix = vi[inb] * int(width) + ui[inb]
        # Per-pixel nearest-depth z-buffer over the flattened image, then keep
        # only the pixels that actually received a point.
        buf = np.full(npix, np.inf, dtype=np.float64)
        np.minimum.at(buf, pix, z[inb])
        upix = np.unique(pix)
        out.append((upix.astype(np.int64), buf[upix].astype(np.float32)))
    return out


def drop_sparse_points(xyz: np.ndarray, min_neighbors: int = 3,
                       voxel: float = 0.1) -> np.ndarray:
    """Keep-mask of points whose 3x3x3 voxel neighbourhood holds enough points.

    The count includes the query point, so two means one supporting point and
    one is a no-op. Zero is handled by callers as an explicit disabled value.
    Isolated stray returns render as dust in the map flythrough; counting the
    points in each voxel plus its 26 neighbours (integer 3D keys, ``np.unique``
    histogram) and dropping points below ``min_neighbors`` removes them without
    touching dense surfaces.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    n = xyz.shape[0]
    if n == 0:
        return np.zeros(0, dtype=bool)
    if voxel <= 0.0:
        raise ValueError('voxel must be > 0')

    # Shift all voxel coords to >= 1 and pad the grid by one cell on each side
    # so every 26-neighbour key stays a distinct in-range cell (no wrap-around
    # false matches at the grid boundary).
    ijk = np.floor(xyz / voxel).astype(np.int64)
    ijk -= ijk.min(axis=0) - 1
    dims = ijk.max(axis=0) + 2
    keys = (ijk[:, 0] * dims[1] + ijk[:, 1]) * dims[2] + ijk[:, 2]
    uniq, counts = np.unique(keys, return_counts=True)

    total = np.zeros(n, dtype=np.int64)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                nk = keys + (dx * dims[1] + dy) * dims[2] + dz
                pos = np.searchsorted(uniq, nk)
                pos_c = np.minimum(pos, uniq.size - 1)
                hit = (pos < uniq.size) & (uniq[pos_c] == nk)
                total[hit] += counts[pos_c[hit]]
    return total >= int(min_neighbors)


def project_planar_voxels(points: np.ndarray, voxel_size: float, *,
                          min_points: int = 10,
                          max_planarity_ratio: float = 0.06,
                          min_second_to_first_ratio: float = 0.04,
                          max_projection_distance: float = 0.18
                          ) -> tuple[np.ndarray, np.ndarray]:
    """Project locally planar voxel points onto their PCA plane.

    Only sufficiently populated, two-dimensional voxels are modified. A
    distance cap prevents a second surface or boundary return from being
    collapsed onto the fitted plane. Returns coordinates and a projected mask.
    """
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f'points must be Nx3, got {xyz.shape}')
    if voxel_size <= 0.0:
        raise ValueError('voxel_size must be positive')
    if min_points < 3:
        raise ValueError('min_points must be >= 3')
    if not 0.0 <= max_planarity_ratio <= 1.0:
        raise ValueError('max_planarity_ratio must be in [0, 1]')
    if min_second_to_first_ratio < 0.0:
        raise ValueError('min_second_to_first_ratio must be >= 0')
    if max_projection_distance < 0.0:
        raise ValueError('max_projection_distance must be >= 0')
    refined = xyz.copy()
    projected = np.zeros(len(xyz), dtype=bool)
    if not len(xyz):
        return refined, projected

    keys = np.floor(xyz / voxel_size).astype(np.int64)
    order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
    ordered_keys = keys[order]
    changes = np.flatnonzero(np.any(
        ordered_keys[1:] != ordered_keys[:-1], axis=1)) + 1
    bounds = np.concatenate(([0], changes, [len(xyz)]))
    for begin, end in zip(bounds[:-1], bounds[1:]):
        ids = order[begin:end]
        if ids.size < min_points:
            continue
        block = xyz[ids]
        centre = block.mean(axis=0)
        centred = block - centre
        eigenvalues, eigenvectors = np.linalg.eigh(
            centred.T @ centred / ids.size)
        total = max(float(eigenvalues.sum()), 1.0e-12)
        if eigenvalues[0] / total > max_planarity_ratio:
            continue
        if eigenvalues[1] / max(float(eigenvalues[2]), 1.0e-12) < \
                min_second_to_first_ratio:
            continue
        normal = eigenvectors[:, 0]
        distances = centred @ normal
        use = np.abs(distances) <= max_projection_distance
        chosen = ids[use]
        refined[chosen] = block[use] - distances[use, None] * normal
        projected[chosen] = True
    return refined, projected


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
