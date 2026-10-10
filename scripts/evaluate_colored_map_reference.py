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

"""Score map colours against a reference coloured point cloud (e.g. a TLS map).

Reprojection metrics compare the map with the same photographs it was
coloured from, so a colour error caused by a pose error also shifts the
"truth" pixel (sky painted onto branches scores as correct). A reference
cloud coloured independently, such as a terrestrial laser scan of the same
site, has no such blind spot. This report follows MPEG's colour PSNR:

- Each coloured map point is matched to its nearest reference point within
  ``--max-distance`` (map to reference), and each reference point near the
  map to its nearest coloured map point (reference to map).
- Colour error is reported as RGB L2 and as PSNR of BT.709 Y, Cb and Cr,
  combined as ``PSNR_YUV = (6 Y + Cb + Cr) / 8``; ``symmetric`` takes the
  worse direction, as MPEG's ``pc_error`` does.
- The reference was imaged by another camera with its own exposure and white
  balance, so a per-channel affine map (gain and bias, least squares on the
  map-to-reference pairs) is also fitted and the metrics are repeated after
  it (``affine``). Compare methods on the affine numbers; the raw ones show
  how far apart the two colour pipelines are.

Points still carrying the colouriser's default grey are left out and
counted, so coverage cannot be traded for accuracy silently.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_DIR = REPO_ROOT / 'tools' / 'colored_map'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import pointcloud_io as pcio  # noqa: E402

BT709 = np.array([[0.2126, 0.7152, 0.0722],
                  [-0.1146, -0.3854, 0.5],
                  [0.5, -0.4542, -0.0458]])


def colour_errors(test_rgb: np.ndarray, reference_rgb: np.ndarray) -> dict:
    """RGB L2 statistics and MPEG-style YCbCr PSNR between paired colours."""
    test = np.asarray(test_rgb, dtype=np.float64)
    reference = np.asarray(reference_rgb, dtype=np.float64)
    if test.shape != reference.shape or test.ndim != 2 or test.shape[1] != 3:
        raise ValueError('paired colours must both be Nx3')
    if len(test) == 0:
        raise ValueError('no colour pairs to score')
    l2 = np.linalg.norm(test - reference, axis=1)
    difference = (test - reference) @ BT709.T
    mse = np.mean(difference ** 2, axis=0)
    psnr = 10.0 * np.log10(255.0 ** 2 / np.maximum(mse, 1.0e-12))
    return {
        'pairs': int(len(test)),
        'rgb_l2_median': float(np.median(l2)),
        'rgb_l2_p90': float(np.percentile(l2, 90)),
        'psnr_y': float(psnr[0]), 'psnr_cb': float(psnr[1]),
        'psnr_cr': float(psnr[2]),
        'psnr_yuv': float((6.0 * psnr[0] + psnr[1] + psnr[2]) / 8.0),
    }


def fit_channel_affine(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Least-squares per-channel ``target ~ gain * source + bias`` (3x2)."""
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    params = np.zeros((3, 2))
    for channel in range(3):
        design = np.stack([source[:, channel], np.ones(len(source))], axis=1)
        params[channel] = np.linalg.lstsq(design, target[:, channel],
                                          rcond=None)[0]
    return params


def apply_channel_affine(rgb: np.ndarray, params: np.ndarray) -> np.ndarray:
    """Apply a ``fit_channel_affine`` result and clip to 0..255."""
    rgb = np.asarray(rgb, dtype=np.float64)
    return np.clip(rgb * params[:, 0] + params[:, 1], 0.0, 255.0)


def evaluate(test_xyz, test_rgb, reference_xyz, reference_rgb, *,
             max_distance: float = 0.05,
             default_rgb=(128, 128, 128)) -> dict:
    """Return the map-versus-reference colour report as a dict."""
    from scipy.spatial import cKDTree

    if max_distance <= 0.0:
        raise ValueError('max_distance must be > 0')
    test_xyz = np.asarray(test_xyz, dtype=np.float64)
    test_rgb = np.asarray(test_rgb, dtype=np.uint8)
    reference_xyz = np.asarray(reference_xyz, dtype=np.float64)
    reference_rgb = np.asarray(reference_rgb, dtype=np.uint8)
    coloured = ~np.all(test_rgb == np.asarray(default_rgb, dtype=np.uint8),
                       axis=1)
    report = {'map_points': int(len(test_xyz)),
              'map_uncoloured': int((~coloured).sum()),
              'reference_points': int(len(reference_xyz)),
              'max_distance': float(max_distance)}
    xyz, rgb = test_xyz[coloured], test_rgb[coloured]
    distance, nearest = cKDTree(reference_xyz).query(
        xyz, distance_upper_bound=max_distance)
    forward = np.isfinite(distance)
    report['map_matched_fraction'] = float(forward.mean()) if len(xyz) else 0.0
    if not forward.any():
        raise ValueError('no map point lies within max_distance of the '
                         'reference; check that both share a frame')
    distance_back, nearest_back = cKDTree(xyz).query(
        reference_xyz, distance_upper_bound=max_distance)
    backward = np.isfinite(distance_back)
    report['reference_matched_points'] = int(backward.sum())
    pairs = {
        'map_to_reference': (rgb[forward], reference_rgb[nearest[forward]]),
        'reference_to_map': (rgb[nearest_back[backward]],
                             reference_rgb[backward]),
    }
    params = fit_channel_affine(*pairs['map_to_reference'])
    report['affine_gain_bias'] = params.round(4).tolist()
    for name, transform in (('raw', None), ('affine', params)):
        block = {}
        for direction, (mapped, reference) in pairs.items():
            if transform is not None:
                mapped = apply_channel_affine(mapped, transform)
            block[direction] = colour_errors(mapped, reference)
        block['symmetric_psnr_yuv'] = min(
            block[d]['psnr_yuv'] for d in pairs)
        report[name] = block
    return report


def main(argv=None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--pointcloud', type=Path, required=True,
                        help='coloured map (PLY/PCD) in the reference frame')
    parser.add_argument('--reference', type=Path, required=True,
                        help='reference coloured cloud, e.g. a TLS map')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--max-distance', type=float, default=0.05,
                        help='nearest-neighbour match radius (m)')
    args = parser.parse_args(argv)
    test_xyz, test_rgb = pcio.read_point_cloud_xyz(args.pointcloud)
    reference_xyz, reference_rgb = pcio.read_point_cloud_xyz(args.reference)
    if test_rgb is None or reference_rgb is None:
        raise SystemExit('both clouds need RGB')
    report = evaluate(test_xyz, test_rgb, reference_xyz, reference_rgb,
                      max_distance=args.max_distance)
    report['pointcloud'] = str(args.pointcloud)
    report['reference'] = str(args.reference)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: report[key] for key in (
        'map_matched_fraction', 'reference_matched_points')}))
    print(json.dumps({'raw': report['raw']['symmetric_psnr_yuv'],
                      'affine': report['affine']['symmetric_psnr_yuv']}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
