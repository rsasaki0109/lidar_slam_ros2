#!/usr/bin/env python3
"""Convert a trained 3DGS ``.ply`` (INRIA/gsplat layout) into a ``.splat`` blob.

``.splat`` is the compact, web-friendly format consumed by the MIT-licensed
WebGL viewer vendored under ``docs/assets/3dgs-viewer/`` (antimatter15/splat).
Each Gaussian is packed into 32 bytes:

    float32 x, y, z          (12)  position
    float32 sx, sy, sz       (12)  scale == exp(scale_log)
    uint8   r, g, b, a        (4)  band-0 colour + sigmoid(opacity)
    uint8   qx, qy, qz, qw     (4)  unit quaternion * 128 + 128

Splats are emitted in descending order of ``volume * opacity`` so the viewer's
progressive loader draws the most significant Gaussians first. ``--max-points``
keeps only the top-N of that order (a quality/size knob for web embedding) and
``--min-opacity`` drops near-transparent floaters before the cap.

``gaussians_to_splat_bytes`` is numpy-only and unit tested on CPU; no torch,
CUDA, or gsplat needed -- this is a pure reformatting of a trained ``.ply``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import numpy as np

# SH band-0 constant: rgb = 0.5 + C0 * f_dc (matches train_gsplat.export_ply).
SH_C0 = 0.28209479177387814
SPLAT_BYTES = 32


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def gaussians_to_splat_bytes(means: np.ndarray, scales_log: np.ndarray,
                             quats: np.ndarray, opacities_logit: np.ndarray,
                             colors_rgb: np.ndarray, *,
                             max_points: Optional[int] = None,
                             min_opacity: float = 0.0,
                             max_scale: Optional[float] = None) -> bytes:
    """Pack Gaussians into the 32-byte-per-splat ``.splat`` layout.

    ``colors_rgb`` is 0..1 (already band-0 decoded), ``opacities_logit`` and
    ``scales_log`` are the raw stored (logit/log) values. ``quats`` are
    ``(w, x, y, z)`` as written by the INRIA ``.ply``. ``max_scale`` (metres)
    drops oversized floater Gaussians whose largest axis exceeds it -- a handful
    of giant splats otherwise streak across arbitrary web-viewer angles.
    Returns the raw bytes.
    """
    means = np.asarray(means, dtype=np.float32)
    scales = np.exp(np.asarray(scales_log, dtype=np.float32))
    quats = np.asarray(quats, dtype=np.float64)
    opac = np.asarray(opacities_logit, dtype=np.float64)
    colors = np.asarray(colors_rgb, dtype=np.float64)
    n = means.shape[0]
    if not (scales.shape[0] == quats.shape[0] == opac.shape[0]
            == colors.shape[0] == n):
        raise ValueError('all Gaussian arrays must share the leading dimension')

    alpha = _sigmoid(opac)
    keep = alpha >= min_opacity
    if max_scale is not None:
        keep = keep & (scales.max(axis=1) <= max_scale)
    if not np.any(keep):
        raise ValueError(
            f'min_opacity={min_opacity} / max_scale={max_scale} pruned every '
            f'Gaussian (max sigmoid(opacity)={float(alpha.max()):.3f}, min '
            f'max-axis scale={float(scales.max(axis=1).min()):.3f} m)')

    # Most-significant first: large, opaque splats lead the progressive load.
    importance = scales.prod(axis=1).astype(np.float64) * alpha
    importance = np.where(keep, importance, -np.inf)
    order = np.argsort(-importance)
    if max_points is not None and max_points < int(keep.sum()):
        order = order[:max_points]
    else:
        order = order[:int(keep.sum())]

    means = means[order]
    scales = scales[order]
    quats = quats[order]
    rgba = np.empty((order.size, 4), dtype=np.uint8)
    rgba[:, :3] = np.clip(colors[order] * 255.0, 0, 255).astype(np.uint8)
    rgba[:, 3] = np.clip(alpha[order] * 255.0, 0, 255).astype(np.uint8)

    norm = np.linalg.norm(quats, axis=1, keepdims=True)
    norm[norm == 0.0] = 1.0
    rot = np.clip((quats / norm) * 128.0 + 128.0, 0, 255).astype(np.uint8)

    buf = np.empty((order.size, SPLAT_BYTES), dtype=np.uint8)
    buf[:, 0:12] = means.view(np.uint8).reshape(order.size, 12)
    buf[:, 12:24] = scales.view(np.uint8).reshape(order.size, 12)
    buf[:, 24:28] = rgba
    buf[:, 28:32] = rot
    return buf.tobytes()


def ply_to_splat_bytes(ply_path: str | Path, *, max_points: Optional[int] = None,
                       min_opacity: float = 0.0,
                       max_scale: Optional[float] = None) -> bytes:
    """Read an INRIA-layout ``.ply`` and return its ``.splat`` bytes."""
    import sys
    tool_dir = str(Path(__file__).resolve().parent)
    if tool_dir not in sys.path:
        sys.path.insert(0, tool_dir)
    from render_path import load_gaussian_ply

    g = load_gaussian_ply(ply_path)
    return gaussians_to_splat_bytes(
        g['means'], g['scales_log'], g['quats'], g['opacities_logit'],
        g['colors_rgb'], max_points=max_points, min_opacity=min_opacity,
        max_scale=max_scale)


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Convert a 3DGS .ply into a web-viewer .splat blob.')
    parser.add_argument('input', help='Input INRIA-layout 3DGS .ply.')
    parser.add_argument('--output', '-o', required=True, help='Output .splat.')
    parser.add_argument('--max-points', type=int, default=None,
                        help='Keep only the top-N most significant splats.')
    parser.add_argument('--min-opacity', type=float, default=0.0,
                        help='Drop splats with sigmoid(opacity) below this.')
    parser.add_argument('--max-scale', type=float, default=None,
                        help='Drop floater splats whose largest axis (m) '
                             'exceeds this.')
    args = parser.parse_args()

    data = ply_to_splat_bytes(args.input, max_points=args.max_points,
                              min_opacity=args.min_opacity,
                              max_scale=args.max_scale)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(data)
    print(f'wrote {out} ({len(data) // SPLAT_BYTES} splats, '
          f'{len(data) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
