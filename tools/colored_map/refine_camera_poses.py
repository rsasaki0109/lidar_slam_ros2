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

"""Refine each camera pose against the LiDAR map with image feature matches.

A coloured map blurs when the camera poses disagree by a fraction of a degree.
The disagreement changes from view to view (trajectory error, image timing,
fast turns), so one extrinsic correction cannot remove it. This tool corrects
every view on its own:

1. ORB features are matched between nearby views (one and three frames apart
   by default) and filtered with a fundamental-matrix RANSAC.
2. A match seen in view a is lifted onto the LiDAR surface through a's depth
   image, and its partner pixel in view b constrains b's pose (PnP, refined
   from the current pose). Every view is updated, the depth images are
   rendered again, and the loop repeats.
3. Pairs two frames apart are never fitted. Their epipolar (Sampson) error
   under the recorded and the refined poses is the check, and the refined
   poses are written only when it improves.

No colours are used, so the held-out colour error of the map stays an
independent check of the result.

Each view is pulled towards what its neighbours see, so disagreement between
nearby views (what blurs the colours) goes within a few rounds, while an error
shared by a run of neighbouring views fades slowly and one shared by every
view (a constant extrinsic error) is barely observed.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import os
from pathlib import Path
import sys
from typing import Optional, Sequence

import numpy as np

_HERE = Path(__file__).resolve().parent
_GS_DIR = _HERE.parent / 'gaussian_splatting'
for _path in (_HERE, _GS_DIR):
    if str(_path) not in sys.path:
        sys.path.append(str(_path))

import pointcloud_io as pcio  # noqa: E402
import posed_images as pi  # noqa: E402
from train_gsplat import load_transforms  # noqa: E402

EMPTY_DEPTH = np.float32(1e9)


def scale_camera_matrix(K: np.ndarray, scale: float) -> np.ndarray:
    """Intrinsics of the image resized by ``scale`` (pixel-centre convention)."""
    scaled = np.array(K, dtype=np.float64)
    scaled[0, 0] *= scale
    scaled[1, 1] *= scale
    scaled[0, 2] = (K[0, 2] + 0.5) * scale - 0.5
    scaled[1, 2] = (K[1, 2] + 0.5) * scale - 0.5
    return scaled


def rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = (np.trace(rotation) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def pose_change(w2c_a: np.ndarray, w2c_b: np.ndarray) -> tuple[float, float]:
    """Rotation (deg) and camera-centre shift (m) between two world->camera poses."""
    rotation = rotation_angle_deg(w2c_a[:3, :3].T @ w2c_b[:3, :3])
    centre_a = -w2c_a[:3, :3].T @ w2c_a[:3, 3]
    centre_b = -w2c_b[:3, :3].T @ w2c_b[:3, 3]
    return rotation, float(np.linalg.norm(centre_b - centre_a))


def sampson_errors(w2c_i: np.ndarray, w2c_j: np.ndarray, K: np.ndarray,
                   ui: np.ndarray, uj: np.ndarray,
                   min_baseline_m: float = 1e-3) -> Optional[np.ndarray]:
    """Sampson distance (px) of matches ui<->uj under the two poses.

    ``None`` when the cameras barely move apart, since the epipolar geometry
    is then undefined.
    """
    relative = w2c_j @ np.linalg.inv(w2c_i)
    rotation, translation = relative[:3, :3], relative[:3, 3]
    if np.linalg.norm(translation) < min_baseline_m:
        return None
    skew = np.array([[0.0, -translation[2], translation[1]],
                     [translation[2], 0.0, -translation[0]],
                     [-translation[1], translation[0], 0.0]])
    K_inv = np.linalg.inv(K)
    fundamental = K_inv.T @ skew @ rotation @ K_inv
    x_i = np.c_[ui, np.ones(len(ui))]
    x_j = np.c_[uj, np.ones(len(uj))]
    f_xi = x_i @ fundamental.T
    ft_xj = x_j @ fundamental
    numerator = np.sum(x_j * f_xi, axis=1) ** 2
    denominator = (f_xi[:, 0] ** 2 + f_xi[:, 1] ** 2 +
                   ft_xj[:, 0] ** 2 + ft_xj[:, 1] ** 2)
    return np.sqrt(numerator / np.maximum(denominator, 1e-18))


def depth_image(points: np.ndarray, w2c: np.ndarray, K: np.ndarray,
                width: int, height: int, min_depth: float = 0.3) -> np.ndarray:
    """Nearest map depth per pixel, min-filtered over 3x3 to close point gaps.

    Pixels without a map point hold ``EMPTY_DEPTH``.
    """
    import cv2

    camera = points @ w2c[:3, :3].T + w2c[:3, 3]
    camera = camera[camera[:, 2] > min_depth]
    pixels = camera[:, :2] / camera[:, 2:3] @ K[:2, :2].T + K[:2, 2]
    u = np.round(pixels[:, 0]).astype(np.int64)
    v = np.round(pixels[:, 1]).astype(np.int64)
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    depth = np.full(width * height, EMPTY_DEPTH, dtype=np.float32)
    np.minimum.at(depth, v[inside] * width + u[inside],
                  camera[inside, 2].astype(np.float32))
    return cv2.erode(depth.reshape(height, width), np.ones((3, 3), np.uint8))


def lift_to_world(depth: np.ndarray, w2c: np.ndarray, K: np.ndarray,
                  pixels: np.ndarray, max_depth: float
                  ) -> tuple[np.ndarray, np.ndarray]:
    """World points the depth image shows at ``pixels`` and which are valid."""
    height, width = depth.shape
    column = np.clip(np.round(pixels[:, 0]).astype(np.int64), 0, width - 1)
    row = np.clip(np.round(pixels[:, 1]).astype(np.int64), 0, height - 1)
    z = depth[row, column].astype(np.float64)
    valid = z < max_depth
    rays = np.c_[(pixels - K[:2, 2]) / np.diag(K)[:2], np.ones(len(pixels))]
    camera = rays * z[:, None]
    c2w = np.linalg.inv(w2c)
    return camera @ c2w[:3, :3].T + c2w[:3, 3], valid


def detect_features(image_path: Path, scale: float, max_features: int):
    """ORB keypoints (pixel coordinates at ``scale``) and descriptors."""
    import cv2

    gray = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(f'cannot read image {image_path}')
    if scale != 1.0:
        gray = cv2.resize(gray, None, fx=scale, fy=scale,
                          interpolation=cv2.INTER_AREA)
    keypoints, descriptors = cv2.ORB_create(max_features).detectAndCompute(
        gray, None)
    pixels = np.array([k.pt for k in keypoints], np.float64).reshape(-1, 2)
    return pixels, descriptors


def match_views(features_i, features_j, *, ratio: float = 0.75,
                ransac_px: float = 1.0, min_inliers: int = 25,
                min_motion_px: float = 2.0):
    """Mutual ratio-test ORB matches that pass a fundamental-matrix RANSAC.

    Returns ``(pixels_i, pixels_j)``, or ``None`` when the pair has too few
    matches or the image barely moves (no epipolar information).
    """
    import cv2

    (pixels_i, descriptors_i), (pixels_j, descriptors_j) = features_i, features_j
    if (descriptors_i is None or descriptors_j is None or
            len(descriptors_i) < 2 or len(descriptors_j) < 2):
        return None
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    backward = {m.queryIdx: m.trainIdx
                for m, *_ in matcher.knnMatch(descriptors_j, descriptors_i, k=1)}
    pairs = [(m.queryIdx, m.trainIdx)
             for candidates in matcher.knnMatch(descriptors_i, descriptors_j, k=2)
             if len(candidates) == 2
             for m, n in [candidates]
             if m.distance < ratio * n.distance
             and backward.get(m.trainIdx) == m.queryIdx]
    if len(pairs) < min_inliers:
        return None
    ui = pixels_i[[p[0] for p in pairs]]
    uj = pixels_j[[p[1] for p in pairs]]
    _, mask = cv2.findFundamentalMat(ui, uj, cv2.FM_RANSAC, ransac_px, 0.999)
    if mask is None:
        return None
    inlier = mask.ravel().astype(bool)
    if inlier.sum() < min_inliers:
        return None
    if np.median(np.linalg.norm(ui[inlier] - uj[inlier], axis=1)) < min_motion_px:
        return None
    return ui[inlier], uj[inlier]


def refine_view(w2c: np.ndarray, world: np.ndarray, pixels: np.ndarray,
                K: np.ndarray, *, min_links: int = 30,
                inlier_px: float = 6.0) -> Optional[np.ndarray]:
    """PnP refinement of one pose from 2D-3D links, started at ``w2c``."""
    import cv2

    if len(world) < min_links:
        return None
    rvec, _ = cv2.Rodrigues(w2c[:3, :3])
    tvec = w2c[:3, 3].reshape(3, 1).copy()
    projected, _ = cv2.projectPoints(world, rvec, tvec, K, None)
    error = np.linalg.norm(projected.reshape(-1, 2) - pixels, axis=1)
    # Links through a wrong surface (occlusion edge, missing geometry) are
    # far off; keep the bulk around the current median.
    inlier = error < max(inlier_px, 2.5 * float(np.median(error)))
    if inlier.sum() < min_links:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(
        world[inlier], pixels[inlier], K, None, rvec, tvec)
    refined = np.eye(4)
    refined[:3, :3] = cv2.Rodrigues(rvec)[0]
    refined[:3, 3] = tvec.ravel()
    return refined


_WORKER_POINTS: Optional[np.ndarray] = None


def _init_worker(points: np.ndarray) -> None:
    global _WORKER_POINTS
    _WORKER_POINTS = points


def _worker_depth(task):
    w2c, K, width, height = task
    return depth_image(_WORKER_POINTS, w2c, K, width, height)


def render_depths(points, poses, K, width, height, workers: int):
    tasks = [(pose, K, width, height) for pose in poses]
    if workers <= 1:
        return [depth_image(points, *task) for task in tasks]
    with ProcessPoolExecutor(max_workers=workers, initializer=_init_worker,
                             initargs=(points,)) as pool:
        return list(pool.map(_worker_depth, tasks))


def heldout_epipolar(poses, check_pairs, K) -> list[float]:
    """Median Sampson error (px) of each held-out pair under ``poses``."""
    medians = []
    for (i, j), (ui, uj) in check_pairs.items():
        errors = sampson_errors(poses[i], poses[j], K, ui, uj)
        if errors is not None:
            medians.append(float(np.median(errors)))
    return medians


def refine_poses(points, recorded, K, width, height, fit_pairs, *,
                 iterations: int, max_depth: float, max_rotation_deg: float,
                 max_translation_m: float, workers: int = 1,
                 log=print) -> tuple[list[np.ndarray], list[dict]]:
    """Alternate map lifting and per-view PnP; return poses and per-round stats.

    A view whose pose would move further from its recorded pose than the
    bounds keeps the recorded pose.
    """
    poses = [np.array(p, dtype=np.float64) for p in recorded]
    history = []
    for round_index in range(iterations):
        depths = render_depths(points, poses, K, width, height, workers)
        links = [[] for _ in poses]
        for (i, j), (ui, uj) in fit_pairs.items():
            for source, target, source_px, target_px in (
                    (i, j, ui, uj), (j, i, uj, ui)):
                world, valid = lift_to_world(
                    depths[source], poses[source], K, source_px, max_depth)
                links[target].append((world[valid], target_px[valid]))
        updated, refined, bounded = [], 0, 0
        for index, pose in enumerate(poses):
            candidate = None
            if links[index]:
                world = np.concatenate([w for w, _ in links[index]])
                pixels = np.concatenate([p for _, p in links[index]])
                candidate = refine_view(pose, world, pixels, K)
            if candidate is not None:
                rotation, translation = pose_change(recorded[index], candidate)
                if rotation > max_rotation_deg or translation > max_translation_m:
                    candidate = np.array(recorded[index], dtype=np.float64)
                    bounded += 1
                else:
                    refined += 1
            updated.append(pose if candidate is None else candidate)
        poses = updated
        history.append({'round': round_index, 'refined_views': refined,
                        'views_reset_to_recorded': bounded})
        log(f'round {round_index + 1}/{iterations}: refined {refined} views, '
            f'{bounded} reset to the recorded pose')
    return poses, history


def frame_pairs(groups: Sequence[int], gaps: Sequence[int]) -> list[tuple[int, int]]:
    """Index pairs ``gap`` frames apart within the same capture group."""
    return [(i, i + gap) for gap in gaps for i in range(len(groups) - gap)
            if groups[i] == groups[i + gap]]


def summarize(values: Sequence[float]) -> dict:
    if not values:
        return {'pairs': 0, 'median_px': None, 'p90_px': None}
    return {'pairs': len(values),
            'median_px': float(np.median(values)),
            'p90_px': float(np.percentile(values, 90))}


def write_refined_transforms(source: Path, output: Path, viewmats,
                             metadata: dict) -> Path:
    """Write the refined poses, keeping every other frame field."""
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output:
        raise ValueError('refined transforms output must differ from source')
    document = json.loads(source.read_text())
    dataset = load_transforms(source)
    for frame, viewmat, image_path in zip(
            document['frames'], viewmats, dataset['image_paths'], strict=True):
        frame['transform_matrix'] = (
            np.linalg.inv(viewmat) @ pi.ROS_OPTICAL_TO_OPENGL).tolist()
        frame['file_path'] = os.path.relpath(image_path, output.parent)
    document['camera_pose_refinement'] = metadata
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2) + '\n')
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--transforms', type=Path, required=True,
                        help='posed images (transforms.json) to refine')
    parser.add_argument('--pointcloud', type=Path, required=True,
                        help='LiDAR map in the same frame as the poses')
    parser.add_argument('--out', type=Path, required=True,
                        help='refined transforms.json')
    parser.add_argument('--report', type=Path, default=None,
                        help='JSON report (default: <out>.report.json)')
    parser.add_argument('--image-scale', type=float, default=0.5,
                        help='resize images by this factor for matching')
    parser.add_argument('--iterations', type=int, default=6)
    parser.add_argument('--fit-gaps', default='1,3',
                        help='frame gaps of the pairs that are fitted')
    parser.add_argument('--check-gap', type=int, default=2,
                        help='frame gap of the held-out pairs')
    parser.add_argument('--max-features', type=int, default=4000)
    parser.add_argument('--max-depth', type=float, default=40.0)
    parser.add_argument('--depth-max-points', type=int, default=3_000_000,
                        help='subsample the map (fixed stride) for depth images')
    parser.add_argument('--max-rotation-deg', type=float, default=3.0,
                        help='keep the recorded pose of a view that would '
                             'rotate further than this')
    parser.add_argument('--max-translation', type=float, default=0.3,
                        help='keep the recorded pose of a view whose centre '
                             'would move further than this (m)')
    parser.add_argument('--min-improvement', type=float, default=0.05,
                        help='required relative drop of the held-out epipolar '
                             'error before the refined poses are written')
    parser.add_argument('--workers', type=int, default=1)
    return parser


def run(args: argparse.Namespace, log=print) -> dict:
    """Refine, check on held-out pairs and write the adopted poses."""
    dataset = load_transforms(args.transforms)
    K = scale_camera_matrix(dataset['K'], args.image_scale)
    width = int(round(dataset['width'] * args.image_scale))
    height = int(round(dataset['height'] * args.image_scale))
    recorded = [np.asarray(v, dtype=np.float64) for v in dataset['viewmats']]
    points, _ = pcio.read_ply_xyz(args.pointcloud)
    stride = max(1, int(np.ceil(len(points) / args.depth_max_points)))
    points = np.ascontiguousarray(points[::stride], dtype=np.float64)
    log(f'{len(recorded)} views, {len(points)} map points for depth, '
        f'{width}x{height} images')

    features = [detect_features(path, args.image_scale, args.max_features)
                for path in dataset['image_paths']]
    fit_gaps = [int(v) for v in str(args.fit_gaps).split(',') if v.strip()]
    if args.check_gap in fit_gaps:
        raise ValueError('the held-out gap must differ from the fitted gaps')

    def matched(pairs):
        result = {}
        for i, j in pairs:
            match = match_views(features[i], features[j])
            if match is not None:
                result[(i, j)] = match
        return result

    fit_pairs = matched(frame_pairs(dataset['groups'], fit_gaps))
    check_pairs = matched(frame_pairs(dataset['groups'], [args.check_gap]))
    log(f'{len(fit_pairs)} fitted pairs, {len(check_pairs)} held-out pairs')

    before = summarize(heldout_epipolar(recorded, check_pairs, K))
    poses, history = refine_poses(
        points, recorded, K, width, height, fit_pairs,
        iterations=args.iterations, max_depth=args.max_depth,
        max_rotation_deg=args.max_rotation_deg,
        max_translation_m=args.max_translation, workers=args.workers, log=log)
    after = summarize(heldout_epipolar(poses, check_pairs, K))

    changes = np.array([pose_change(a, b) for a, b in zip(recorded, poses)])
    adopted = bool(
        before['median_px'] is not None and after['median_px'] is not None and
        after['median_px'] <= before['median_px'] * (1.0 - args.min_improvement))
    report = {
        'adopted': adopted,
        'views': len(recorded),
        'fitted_pairs': len(fit_pairs),
        'heldout_epipolar_recorded': before,
        'heldout_epipolar_refined': after,
        'image_scale': args.image_scale,
        'correction_rotation_deg': {
            'median': float(np.median(changes[:, 0])),
            'p90': float(np.percentile(changes[:, 0], 90))},
        'correction_translation_m': {
            'median': float(np.median(changes[:, 1])),
            'p90': float(np.percentile(changes[:, 1], 90))},
        'rounds': history,
    }
    write_refined_transforms(args.transforms, args.out,
                             poses if adopted else recorded, report)
    report_path = args.report or args.out.with_suffix('.report.json')
    report_path.write_text(json.dumps(report, indent=2) + '\n')
    log(f"held-out epipolar error {before['median_px']} -> {after['median_px']} px "
        f"({'adopted' if adopted else 'not adopted; recorded poses written'})")
    return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    run(build_parser().parse_args(argv))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
