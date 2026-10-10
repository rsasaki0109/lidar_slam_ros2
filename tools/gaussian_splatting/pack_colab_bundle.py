#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Pack posed images and a coloured LiDAR map into one zip for 3DGS on Colab.

A posed-image directory of full-resolution PNGs is too large to upload
comfortably (640 Stadtgarten frames are 1.4 GB). The bundle re-encodes the
frames as JPEG, rewrites ``transforms.json`` to point at them, and adds a
voxel-thinned ``init.ply`` (xyz + rgb) for ``train_gsplat.py --init-ply``.
Points still carrying the colourizer's default grey are dropped, because
they would seed Gaussians with a colour no camera saw.

``--harmonize-exposure`` multiplies each frame by the per-view RGB gain that
``estimate_overlap_rgb_gains`` solves from LiDAR points seen in neighbouring
frames. Auto-exposure on a walking capture changes brightness by tens of
percent between views (Stadtgarten 2: 0.67 to 1.5), and ``train_gsplat.py``
only compensates exposure per capture session, so without this the Gaussians
absorb the flicker. The gains go into each frame as ``exposure_gain``.

Layout inside the zip::

    transforms.json
    images/00000.jpg ...
    init.ply
"""

from __future__ import annotations

import argparse
import io
import json
from pathlib import Path
import sys
import tempfile
from typing import Optional, Sequence
import zipfile

import numpy as np

_SOURCE_ROOT = Path(__file__).resolve().parents[2]
if not __package__ and (_SOURCE_ROOT / 'lidarslam_benchmark_tools').is_dir():
    if str(_SOURCE_ROOT) not in sys.path:
        sys.path.insert(0, str(_SOURCE_ROOT))

import lidarslam_benchmark_tools.gaussian_splatting.pointcloud_io as pcio  # noqa: E402, I100
import lidarslam_benchmark_tools.gaussian_splatting.train_gsplat as tg  # noqa: E402, I100


def thin_init_cloud(xyz: np.ndarray, rgb: Optional[np.ndarray], *,
                    voxel: float, max_points: int,
                    default_rgb=(128, 128, 128), seed: int = 0):
    """Drop uncoloured points, voxel-thin, and cap the count (seeded)."""
    xyz = np.asarray(xyz, dtype=np.float32)
    if rgb is not None:
        rgb = np.asarray(rgb, dtype=np.uint8)
        coloured = ~np.all(rgb == np.asarray(default_rgb, dtype=np.uint8), axis=1)
        xyz, rgb = xyz[coloured], rgb[coloured]
    xyz, rgb = pcio.voxel_downsample(xyz, voxel, rgb)
    if max_points > 0 and len(xyz) > max_points:
        keep = np.sort(np.random.default_rng(seed).choice(
            len(xyz), max_points, replace=False))
        xyz = xyz[keep]
        rgb = None if rgb is None else rgb[keep]
    return xyz, rgb


def pack_bundle(transforms: Path, out: Path, *, init_ply: Optional[Path] = None,
                jpeg_quality: int = 95, frame_stride: int = 1,
                init_voxel: float = 0.02, max_init_points: int = 3000000,
                harmonize_exposure: bool = False,
                exposure_gain_limit: float = 1.5) -> dict:
    """Write the zip and return a short summary."""
    from PIL import Image

    if not 1 <= jpeg_quality <= 100:
        raise ValueError('jpeg_quality must be in [1, 100]')
    if frame_stride < 1:
        raise ValueError('frame_stride must be >= 1')
    transforms = Path(transforms)
    doc = json.loads(transforms.read_text())
    selected = list(range(0, len(doc['frames']), frame_stride))
    frames = [doc['frames'][i] for i in selected]
    if not frames:
        raise ValueError('no frames selected')
    gains = None
    if harmonize_exposure:
        if init_ply is None:
            raise ValueError('exposure harmonization needs the map (--init-ply)')
        dataset = tg.load_transforms(transforms)
        map_xyz, _ = pcio.read_ply_xyz(init_ply)
        images = [np.asarray(Image.open(p).convert('RGB'))
                  for p in dataset['image_paths']]
        # Solve over every frame so the stride does not break the pose chain.
        gains = pcio.estimate_overlap_rgb_gains(
            map_xyz, dataset['viewmats'], dataset['K'], images,
            dataset['width'], dataset['height'], gain_limit=exposure_gain_limit)
        del images
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    summary = {'frames': len(frames), 'init_points': 0}
    with zipfile.ZipFile(out, 'w', compression=zipfile.ZIP_STORED) as bundle:
        new_frames = []
        for index, frame in zip(selected, frames):
            source = transforms.parent / frame['file_path']
            name = 'images/' + Path(frame['file_path']).stem + '.jpg'
            image = Image.open(source).convert('RGB')
            if gains is not None:
                scaled = np.asarray(image, dtype=np.float32) * gains[index]
                image = Image.fromarray(
                    np.clip(scaled + 0.5, 0.0, 255.0).astype(np.uint8))
            buffer = io.BytesIO()
            image.save(buffer, format='JPEG', quality=jpeg_quality, subsampling=0)
            bundle.writestr(name, buffer.getvalue())
            entry = {key: value for key, value in frame.items()
                     if key != 'dynamic_mask_path'}
            entry['file_path'] = name
            if gains is not None:
                entry['exposure_gain'] = [float(x) for x in gains[index]]
            new_frames.append(entry)
        packed = dict(doc)
        packed['frames'] = new_frames
        bundle.writestr('transforms.json', json.dumps(packed, indent=1))
        if init_ply is not None:
            xyz, rgb = pcio.read_ply_xyz(init_ply)
            xyz, rgb = thin_init_cloud(xyz, rgb, voxel=init_voxel,
                                       max_points=max_init_points)
            with tempfile.TemporaryDirectory() as tmp:
                path = pcio.write_ply(Path(tmp) / 'init.ply', xyz, rgb)
                bundle.write(path, 'init.ply')
            summary['init_points'] = int(len(xyz))
    if gains is not None:
        level = gains[selected].mean(axis=1)
        summary['exposure_gain_min'] = float(level.min())
        summary['exposure_gain_max'] = float(level.max())
    summary['bytes'] = out.stat().st_size
    return summary


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--transforms', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True, help='output .zip')
    parser.add_argument('--init-ply', type=Path, default=None,
                        help='coloured LiDAR map to thin into init.ply')
    parser.add_argument('--jpeg-quality', type=int, default=95)
    parser.add_argument('--frame-stride', type=int, default=1,
                        help='keep every Nth frame')
    parser.add_argument('--init-voxel', type=float, default=0.02,
                        help='voxel size (m) for thinning init.ply')
    parser.add_argument('--max-init-points', type=int, default=3000000,
                        help='cap on init.ply points (0 keeps all)')
    parser.add_argument('--harmonize-exposure', action='store_true',
                        help='scale each frame by its LiDAR-solved RGB gain')
    parser.add_argument('--exposure-gain-limit', type=float, default=1.5)
    args = parser.parse_args(argv)
    summary = pack_bundle(
        args.transforms, args.out, init_ply=args.init_ply,
        jpeg_quality=args.jpeg_quality, frame_stride=args.frame_stride,
        init_voxel=args.init_voxel, max_init_points=args.max_init_points,
        harmonize_exposure=args.harmonize_exposure,
        exposure_gain_limit=args.exposure_gain_limit)
    print(json.dumps(summary))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
