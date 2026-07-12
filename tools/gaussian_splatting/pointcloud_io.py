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


def _sample_pixels(img: np.ndarray, uf: np.ndarray, vf: np.ndarray,
                   width: int, height: int, interp: str) -> np.ndarray:
    """Sample ``img`` at float pixel coords ``(uf, vf)`` -> float32 ``(M,3)``.

    ``interp='nearest'`` rounds to the pixel centre; ``'bilinear'`` blends the
    four surrounding pixels (edge coords clamp, so the border never wraps). A
    2-D (grayscale) image is broadcast to three channels.
    """
    im = np.asarray(img).astype(np.float32)
    if im.ndim == 2:
        im = np.repeat(im[:, :, None], 3, axis=2)
    im = im[:, :, :3]
    if interp == 'nearest':
        ui = np.clip(np.round(uf).astype(np.int64), 0, width - 1)
        vi = np.clip(np.round(vf).astype(np.int64), 0, height - 1)
        return im[vi, ui]
    if interp != 'bilinear':
        raise ValueError(f"interp must be 'nearest' or 'bilinear', got {interp!r}")
    x0 = np.clip(np.floor(uf).astype(np.int64), 0, width - 1)
    y0 = np.clip(np.floor(vf).astype(np.int64), 0, height - 1)
    x1 = np.minimum(x0 + 1, width - 1)
    y1 = np.minimum(y0 + 1, height - 1)
    wx = np.clip(uf - x0, 0.0, 1.0)[:, None].astype(np.float32)
    wy = np.clip(vf - y0, 0.0, 1.0)[:, None].astype(np.float32)
    top = im[y0, x0] * (1.0 - wx) + im[y0, x1] * wx
    bot = im[y1, x0] * (1.0 - wx) + im[y1, x1] * wx
    return top * (1.0 - wy) + bot * wy


def _median_luminance(img: np.ndarray) -> float:
    """Median luminance of mono or RGB(A) image data."""
    arr = np.asarray(img, dtype=np.float32)
    if arr.ndim == 2:
        return float(np.median(arr))
    if arr.ndim != 3 or arr.shape[2] == 0:
        raise ValueError(f'image must be HxW or HxWxC, got {arr.shape}')
    if arr.shape[2] == 1:
        return float(np.median(arr[:, :, 0]))
    coeff = np.asarray([0.299, 0.587, 0.114], dtype=np.float32)
    return float(np.median(np.tensordot(arr[:, :, :3], coeff, axes=([-1], [0]))))


def observed_color_medoids(samples: np.ndarray, chunk: int = 20000) -> np.ndarray:
    """Choose the observed RGB sample nearest all other samples per point.

    A channel-wise median can synthesize a colour that no camera observed. For
    example, red, green, and blue samples produce black. The L1 medoid retains
    median-like outlier resistance while guaranteeing that every output row is
    one of the input camera observations. Chunking bounds the temporary pairwise
    distance array for large maps.
    """
    values = np.asarray(samples, dtype=np.uint8)
    if values.ndim != 3 or values.shape[2] != 3 or values.shape[1] < 1:
        raise ValueError(f'samples must be NxSx3 with S >= 1, got {values.shape}')
    if chunk < 1:
        raise ValueError('chunk must be >= 1')
    out = np.empty((values.shape[0], 3), dtype=np.uint8)
    for start in range(0, len(values), chunk):
        block = values[start:start + chunk].astype(np.int16)
        pairwise = np.abs(block[:, :, None, :] - block[:, None, :, :])
        scores = pairwise.sum(axis=(2, 3), dtype=np.int32)
        choice = np.argmin(scores, axis=1)
        out[start:start + len(block)] = block[np.arange(len(block)), choice]
    return out


def colorize_by_projection_robust(points: np.ndarray, viewmats: np.ndarray,
                                  K: np.ndarray, images, width: int, height: int,
                                  default_rgb=(128, 128, 128), *,
                                  zbuf_bin: int = 1, depth_tol: float = 0.15,
                                  max_samples: int = 12,
                                  normalize_exposure: bool = True,
                                  interp: str = 'bilinear',
                                  prefer_near: bool = True,
                                  return_counts: bool = False):
    """Occlusion-aware, exposure-normalised, median-robust point colorization.

    ``colorize_by_projection`` averages every view a point lands in, so points
    behind walls or machines pick up the colour of whatever occludes them and
    auto-exposure differences wash the average out. This variant first builds a
    per-view z-buffer from the point cloud itself (one pixel per bin by default)
    and only samples views where the point sits within ``depth_tol`` (plus
    2 % of range) of the nearest depth in its bin; each image is scaled so its
    median luminance matches the global median (``normalize_exposure``); and the
    final colour is the RGB medoid over up to ``max_samples`` valid samples,
    which rejects residual specular / motion-blur outliers without synthesizing
    a colour that no camera actually observed.

    A value above one for ``zbuf_bin`` trades memory for a coarse occlusion
    approximation. It can falsely hide a surface when an unrelated nearer point
    lands in a neighbouring pixel, so final map generation should keep the
    one-pixel default.

    Quality knobs: ``interp='bilinear'`` samples sub-pixel (blends the four
    surrounding pixels) instead of snapping to the nearest, cutting the colour
    bleed nearest-pixel sampling leaves along edges. ``prefer_near=True`` keeps,
    once a point has ``max_samples`` observations, the *nearest* ones (a new
    closer view evicts the farthest stored sample) so colour comes from the
    highest-resolution, least-foreshortened views rather than whichever happened
    to be visited first.

    Returns ``(rgb uint8 (N,3), seen bool (N,))``, or with ``return_counts`` the
    triple ``(rgb, seen, counts uint16 (N,))`` giving each point's surviving
    sample count (a colour-confidence signal). Unseen points get ``default_rgb``.
    """
    points = np.asarray(points, dtype=np.float64)
    n = points.shape[0]
    rgb = np.tile(np.asarray(default_rgb, dtype=np.uint8), (n, 1))
    counts = np.zeros(n, dtype=np.uint16)
    if n == 0 or max_samples <= 0:
        seen = np.zeros(n, dtype=bool)
        return (rgb, seen, counts) if return_counts else (rgb, seen)
    if zbuf_bin < 1:
        raise ValueError('zbuf_bin must be >= 1')

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    zb_w = (int(width) + zbuf_bin - 1) // zbuf_bin
    zb_h = (int(height) + zbuf_bin - 1) // zbuf_bin
    samples = np.empty((n, int(max_samples), 3), dtype=np.uint8)
    # Depth stored alongside each sample so a nearer view can evict the farthest.
    sample_z = np.full((n, int(max_samples)), np.inf, dtype=np.float32)

    scales = np.ones(len(images), dtype=np.float32)
    if normalize_exposure:
        meds = np.asarray([_median_luminance(img) for img in images],
                          dtype=np.float32)
        valid = meds > 1.0e-6
        if valid.any():
            scales[valid] = float(np.median(meds[valid])) / meds[valid]

    ids = np.arange(n)
    for vi, (vm, img) in enumerate(zip(viewmats, images)):
        vm = np.asarray(vm, dtype=np.float64)
        cam = points @ vm[:3, :3].T + vm[:3, 3]
        z = cam[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            uf = np.nan_to_num(fx * cam[:, 0] / z + cx, nan=-1.0,
                               posinf=-1.0, neginf=-1.0)
            vf = np.nan_to_num(fy * cam[:, 1] / z + cy, nan=-1.0,
                               posinf=-1.0, neginf=-1.0)
        u = np.round(uf).astype(np.int64)
        v = np.round(vf).astype(np.int64)
        inb = (z > 1e-6) & (u >= 0) & (u < width) & (v >= 0) & (v < height)
        if not inb.any():
            continue
        zbin = (v[inb] // zbuf_bin) * zb_w + (u[inb] // zbuf_bin)
        zbuf = np.full(zb_w * zb_h, np.inf, dtype=np.float32)
        np.minimum.at(zbuf, zbin, z[inb].astype(np.float32))
        visible = z[inb] <= zbuf[zbin] + depth_tol + 0.02 * z[inb]
        cand = ids[inb][visible]  # unique point ids seen (unoccluded) this view
        if cand.size == 0:
            continue
        cand_z = z[inb][visible].astype(np.float32)
        cols = _sample_pixels(img, uf[cand], vf[cand], width, height, interp)
        cols = np.clip(cols * scales[vi], 0.0, 255.0).astype(np.uint8)

        # Points with room: append into the next free slot.
        room = counts[cand] < max_samples
        if room.any():
            rc = cand[room]
            slot = counts[rc].astype(np.intp)
            samples[rc, slot, :] = cols[room]
            sample_z[rc, slot] = cand_z[room]
            counts[rc] += 1
        # Full points: if enabled, evict the farthest stored sample when nearer.
        if prefer_near and (~room).any():
            fc = cand[~room]
            fcz = cand_z[~room]
            fcols = cols[~room]
            far_slot = np.argmax(sample_z[fc], axis=1)
            nearer = fcz < sample_z[fc, far_slot]
            if nearer.any():
                fb = fc[nearer]
                sb = far_slot[nearer]
                samples[fb, sb, :] = fcols[nearer]
                sample_z[fb, sb] = fcz[nearer]

    seen = counts > 0
    seen_idx = np.flatnonzero(seen)
    for c in np.unique(counts[seen_idx]):
        group = seen_idx[counts[seen_idx] == c]
        rgb[group] = observed_color_medoids(samples[group, :int(c), :])
    return (rgb, seen, counts) if return_counts else (rgb, seen)


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
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    npix = int(width) * int(height)
    out = []
    for vm in viewmats:
        vm = np.asarray(vm, dtype=np.float64)
        cam = pts @ vm[:3, :3].T + vm[:3, 3]
        z = cam[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = np.nan_to_num(fx * cam[:, 0] / z + cx, nan=-1.0,
                              posinf=-1.0, neginf=-1.0)
            v = np.nan_to_num(fy * cam[:, 1] / z + cy, nan=-1.0,
                              posinf=-1.0, neginf=-1.0)
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
