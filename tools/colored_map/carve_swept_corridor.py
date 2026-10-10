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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
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

"""Remove map points inside the corridor the camera bearer walked through.

Dynamic-object removal votes with LiDAR rays that pass through a voxel. A
person who stood on the route and left before the camera arrived is never
looked back at by a forward-facing sensor, so they stay in the map as a
ghost. The camera bearer later walked through exactly that space, which
nothing static can occupy, so points within ``--radius`` of the camera path
and between ``--below`` and ``--above`` around camera height are removed.
"Up" is the mean camera up vector of the posed images.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Optional, Sequence

import numpy as np

_HERE = Path(__file__).resolve().parent
_GS_DIR = _HERE.parent / 'gaussian_splatting'
for _path in (_HERE, _GS_DIR):
    if str(_path) not in sys.path:
        sys.path.append(str(_path))

import pointcloud_io as pcio  # noqa: E402, I100


def camera_path(transforms: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Return camera centres in frame order and the mean camera up vector."""
    import train_gsplat as tg

    dataset = tg.load_transforms(transforms)
    viewmats = np.asarray(dataset['viewmats'], dtype=np.float64)
    centres = np.einsum('nji,nj->ni', viewmats[:, :3, :3], -viewmats[:, :3, 3])
    return centres, pcio.estimate_world_up(viewmats)


def height_histogram(points, path, up, *, radius: float, bins=None) -> dict:
    """Count points by height around the path within ``radius`` (for tuning)."""
    bins = np.arange(-3.0, 1.01, 0.1) if bins is None else bins
    near = pcio.swept_corridor_mask(points, path, up, radius=radius,
                                    below=-bins[0], above=bins[-1])
    from scipy.spatial import cKDTree

    xyz = np.asarray(points, dtype=np.float64)[near]
    if len(xyz) == 0:
        return {'edges': bins.tolist(), 'counts': [0] * (len(bins) - 1)}
    _, nearest = cKDTree(path).query(xyz)
    height = (xyz - path[nearest]) @ (up / np.linalg.norm(up))
    counts, _ = np.histogram(height, bins=bins)
    return {'edges': [round(float(x), 2) for x in bins],
            'counts': counts.tolist()}


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--input', required=True, help='map PLY (xyz[+rgb])')
    parser.add_argument('--transforms', required=True,
                        help='posed-image transforms.json along the same path')
    parser.add_argument('--out', required=True)
    parser.add_argument('--radius', type=float, default=0.5,
                        help='horizontal corridor radius around the camera (m)')
    parser.add_argument('--below', type=float, default=0.8,
                        help='corridor depth below camera height (m); keep it '
                             'under the camera height above the ground')
    parser.add_argument('--above', type=float, default=0.4,
                        help='corridor height above the camera (m)')
    parser.add_argument('--report', default=None,
                        help='JSON with the removed count and a height '
                             'histogram near the path')
    args = parser.parse_args(argv)
    xyz, rgb = pcio.read_ply_xyz(args.input)
    path, up = camera_path(args.transforms)
    carve = pcio.swept_corridor_mask(xyz, path, up, radius=args.radius,
                                     below=args.below, above=args.above)
    out = pcio.write_ply(args.out, xyz[~carve],
                         None if rgb is None else rgb[~carve])
    report = {'input_points': int(len(xyz)), 'removed': int(carve.sum()),
              'radius': args.radius, 'below': args.below, 'above': args.above,
              'height_histogram': height_histogram(xyz, path, up,
                                                   radius=args.radius),
              'out': str(out)}
    if args.report:
        Path(args.report).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items()
                      if k != 'height_histogram'}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
