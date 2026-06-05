#!/usr/bin/env python3
"""Train a 3D Gaussian Splatting model from posed images (gsplat, Apache-2.0).

Consumes a Nerfstudio ``transforms.json`` (as produced by
``extract_posed_images.py``) plus the referenced images, optimises a set of
3D Gaussians with the gsplat CUDA rasteriser, and exports a standard INRIA
3DGS ``.ply`` that SuperSplat / other viewers can open.

This is the GPU half of the pipeline in
``docs/research/3dgs-postprocess-map-design.md``. It is opt-in and requires a
CUDA device + torch + gsplat; importing the pure helpers
(``load_transforms``, ``looks_at_poses``) does not.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import posed_images as pi

# SH band-0 constant: f_dc = (rgb - 0.5) / C0 for the INRIA .ply layout.
SH_C0 = 0.28209479177387814


# --------------------------------------------------------------------------- #
# Pure helpers (no torch/CUDA)
# --------------------------------------------------------------------------- #
def load_transforms(path: str | Path) -> dict:
    """Load a transforms.json into intrinsics + per-frame OpenCV w2c poses.

    Returns a dict with ``K`` (3x3), ``width``, ``height``, ``image_paths``
    (resolved), and ``viewmats`` (list of 4x4 world->camera, OpenCV/gsplat
    convention). The stored ``transform_matrix`` is OpenGL c2w, so we undo the
    ``ROS_OPTICAL_TO_OPENGL`` flip and invert to get the OpenCV w2c gsplat wants.
    """
    path = Path(path)
    doc = json.loads(path.read_text())
    fx, fy = doc['fl_x'], doc['fl_y']
    cx, cy = doc['cx'], doc['cy']
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    image_paths: list[Path] = []
    viewmats: list[np.ndarray] = []
    for fr in doc['frames']:
        c2w_gl = np.asarray(fr['transform_matrix'], dtype=float)
        c2w_cv = c2w_gl @ pi.ROS_OPTICAL_TO_OPENGL  # OpenGL -> OpenCV camera
        viewmats.append(np.linalg.inv(c2w_cv))
        image_paths.append((path.parent / fr['file_path']).resolve())
    return {
        'K': K,
        'width': int(doc['w']),
        'height': int(doc['h']),
        'image_paths': image_paths,
        'viewmats': viewmats,
    }


def looks_at_poses(radius: float, count: int, *, height: float = 0.0) -> list[np.ndarray]:
    """Generate ``count`` OpenCV camera-to-world poses on a ring looking at origin.

    Used by the synthetic self-test; deterministic (no RNG) so it is unit
    testable. Cameras sit on a circle of ``radius`` at ``height`` and point at
    the origin with +z forward (OpenCV optical convention).
    """
    poses: list[np.ndarray] = []
    for i in range(count):
        ang = 2.0 * np.pi * i / count
        eye = np.array([radius * np.cos(ang), radius * np.sin(ang), height])
        forward = -eye / np.linalg.norm(eye)              # +z points at origin
        up_hint = np.array([0.0, 0.0, 1.0])
        right = np.cross(up_hint, forward)
        right /= np.linalg.norm(right)
        down = np.cross(forward, right)                   # +y is down in OpenCV
        c2w = np.eye(4)
        c2w[:3, 0] = right
        c2w[:3, 1] = down
        c2w[:3, 2] = forward
        c2w[:3, 3] = eye
        poses.append(c2w)
    return poses


# --------------------------------------------------------------------------- #
# Gaussian parameter container + INRIA .ply export (numpy only)
# --------------------------------------------------------------------------- #
def export_ply(path: str | Path, means: np.ndarray, scales_log: np.ndarray,
               quats: np.ndarray, opacities_logit: np.ndarray,
               colors_rgb: np.ndarray) -> Path:
    """Write a standard INRIA 3DGS binary ``.ply`` (SH degree 0).

    ``scales_log`` and ``opacities_logit`` are stored raw (log / logit), as the
    3DGS format expects; ``colors_rgb`` (0..1) become ``f_dc`` via SH band 0.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = means.shape[0]
    f_dc = (colors_rgb - 0.5) / SH_C0
    fields = [
        ('x', means[:, 0]), ('y', means[:, 1]), ('z', means[:, 2]),
        ('nx', np.zeros(n)), ('ny', np.zeros(n)), ('nz', np.zeros(n)),
        ('f_dc_0', f_dc[:, 0]), ('f_dc_1', f_dc[:, 1]), ('f_dc_2', f_dc[:, 2]),
        ('opacity', opacities_logit),
        ('scale_0', scales_log[:, 0]), ('scale_1', scales_log[:, 1]),
        ('scale_2', scales_log[:, 2]),
        ('rot_0', quats[:, 0]), ('rot_1', quats[:, 1]),
        ('rot_2', quats[:, 2]), ('rot_3', quats[:, 3]),
    ]
    header = 'ply\nformat binary_little_endian 1.0\n'
    header += f'element vertex {n}\n'
    header += ''.join(f'property float {name}\n' for name, _ in fields)
    header += 'end_header\n'
    arr = np.empty((n, len(fields)), dtype=np.float32)
    for i, (_, col) in enumerate(fields):
        arr[:, i] = col
    with open(path, 'wb') as fh:
        fh.write(header.encode('ascii'))
        fh.write(arr.tobytes())
    return path


# --------------------------------------------------------------------------- #
# Training (torch + gsplat; imported lazily)
# --------------------------------------------------------------------------- #
def train(dataset: dict, *, init_points: Optional[np.ndarray] = None,
          init_colors: Optional[np.ndarray] = None,
          num_init: int = 20000, iters: int = 2000, lr: float = 1e-2,
          device: str = 'cuda', log_every: int = 200) -> dict:
    """Optimise Gaussians to reconstruct the dataset images. Returns numpy params.

    ``init_points`` (N,3, e.g. a LiDAR map) seeds the means; otherwise points
    are sampled in the cameras' bounding sphere. ``init_colors`` (N,3 in 0..1)
    optionally seeds the per-Gaussian colour. The result dict holds ``means``,
    ``scales_log``, ``quats``, ``opacities_logit``, ``colors_rgb``, and the
    ``loss_history``.
    """
    import torch
    import torch.nn.functional as F
    import imageio.v3 as iio
    from gsplat import rasterization

    dev = torch.device(device)
    K = torch.tensor(dataset['K'], dtype=torch.float32, device=dev)[None]
    W, H = dataset['width'], dataset['height']
    viewmats = torch.tensor(np.stack(dataset['viewmats']), dtype=torch.float32, device=dev)
    gts = []
    for p in dataset['image_paths']:
        img = np.asarray(iio.imread(p), dtype=np.float32) / 255.0
        if img.ndim == 2:
            img = np.stack([img] * 3, axis=-1)
        gts.append(torch.tensor(img[..., :3], device=dev))
    gts = torch.stack(gts)  # (C, H, W, 3)

    # Seed means.
    cam_centers = np.stack([np.linalg.inv(v)[:3, 3] for v in dataset['viewmats']])
    center = cam_centers.mean(axis=0)
    extent = float(np.linalg.norm(cam_centers - center, axis=1).max()) + 1e-3
    if init_points is not None and len(init_points) > 0:
        means0 = np.asarray(init_points, dtype=np.float32)
    else:
        rng = np.random.default_rng(0)
        means0 = center + rng.normal(scale=extent * 0.5, size=(num_init, 3))
    means0 = means0.astype(np.float32)

    n = means0.shape[0]
    means = torch.nn.Parameter(torch.tensor(means0, device=dev))
    scales = torch.nn.Parameter(
        torch.full((n, 3), float(np.log(extent / max(n, 1) ** (1 / 3) * 0.5)), device=dev)
    )
    quats = torch.nn.Parameter(
        torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=dev).repeat(n, 1)
    )
    opacities = torch.nn.Parameter(torch.full((n,), 0.1, device=dev))
    if init_colors is not None and len(init_colors) == n:
        c0 = np.clip(np.asarray(init_colors, dtype=np.float32), 1e-4, 1 - 1e-4)
        colors = torch.nn.Parameter(torch.logit(torch.tensor(c0, device=dev)))
    else:
        colors = torch.nn.Parameter(torch.full((n, 3), 0.0, device=dev))

    opt = torch.optim.Adam([
        {'params': [means], 'lr': lr * extent},
        {'params': [scales], 'lr': lr},
        {'params': [quats], 'lr': lr},
        {'params': [opacities], 'lr': lr * 3},
        {'params': [colors], 'lr': lr * 3},
    ])

    loss_history: list[float] = []
    for it in range(iters):
        idx = it % viewmats.shape[0]
        renders, _, _ = rasterization(
            means, F.normalize(quats, dim=-1), torch.exp(scales),
            torch.sigmoid(opacities), torch.sigmoid(colors),
            viewmats[idx:idx + 1], K, W, H,
        )
        loss = F.mse_loss(renders[0], gts[idx])
        opt.zero_grad()
        loss.backward()
        opt.step()
        loss_history.append(float(loss.detach().cpu()))
        if log_every and (it % log_every == 0 or it == iters - 1):
            print(f'iter {it:5d}  mse {loss_history[-1]:.6f}', flush=True)

    return {
        'means': means.detach().cpu().numpy(),
        'scales_log': scales.detach().cpu().numpy(),
        'quats': F.normalize(quats, dim=-1).detach().cpu().numpy(),
        'opacities_logit': opacities.detach().cpu().numpy(),
        'colors_rgb': torch.sigmoid(colors).detach().cpu().numpy(),
        'loss_history': loss_history,
    }


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--transforms', required=True, help='transforms.json path')
    p.add_argument('--out', required=True, help='output .ply path')
    p.add_argument('--init-ply', default=None,
                   help='LiDAR-primed init cloud (xyz[+rgb]); random init if omitted')
    p.add_argument('--iters', type=int, default=2000)
    p.add_argument('--num-init', type=int, default=20000)
    p.add_argument('--lr', type=float, default=1e-2)
    p.add_argument('--device', default='cuda')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    dataset = load_transforms(args.transforms)
    print(f'loaded {len(dataset["image_paths"])} views @ {dataset["width"]}x{dataset["height"]}')
    init_points = None
    init_colors = None
    if args.init_ply:
        import pointcloud_io as pcio
        init_points, rgb = pcio.read_ply_xyz(args.init_ply)
        init_colors = None if rgb is None else rgb.astype(np.float32) / 255.0
        print(f'LiDAR-primed init: {len(init_points)} points from {args.init_ply}')
    params = train(dataset, init_points=init_points, init_colors=init_colors,
                   num_init=args.num_init, iters=args.iters,
                   lr=args.lr, device=args.device)
    out = export_ply(args.out, params['means'], params['scales_log'],
                     params['quats'], params['opacities_logit'], params['colors_rgb'])
    print(f'final mse {params["loss_history"][-1]:.6f} -> {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
