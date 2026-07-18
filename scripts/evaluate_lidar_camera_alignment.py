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

"""Measure LiDAR depth-edge alignment against camera image edges."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import pointcloud_io as pcio  # noqa: E402
import posed_images as pi  # noqa: E402
import train_gsplat as tg  # noqa: E402


def image_edges(image: np.ndarray, percentile: float = 95.0) -> np.ndarray:
    """Return a strong grayscale-gradient edge mask without OpenCV/SciPy."""
    array = np.asarray(image, dtype=np.float32)
    if array.ndim == 3:
        array = array[:, :, :3] @ np.array([0.299, 0.587, 0.114])
    if array.ndim != 2:
        raise ValueError(f'image must be HxW or HxWxC, got {array.shape}')
    if not 0.0 <= percentile <= 100.0:
        raise ValueError('percentile must be between 0 and 100')
    gx = np.zeros_like(array)
    gy = np.zeros_like(array)
    gx[:, 1:-1] = np.abs(array[:, 2:] - array[:, :-2])
    gy[1:-1, :] = np.abs(array[2:, :] - array[:-2, :])
    magnitude = np.hypot(gx, gy)
    nonzero = magnitude[magnitude > 0.0]
    if nonzero.size == 0:
        return np.zeros(array.shape, dtype=bool)
    return magnitude >= np.percentile(nonzero, percentile)


def depth_edges(depth: np.ndarray, absolute: float = 0.25,
                relative: float = 0.02) -> np.ndarray:
    """Return pixels adjacent to a supported LiDAR depth discontinuity."""
    values = np.asarray(depth, dtype=np.float32)
    if values.ndim != 2:
        raise ValueError(f'depth must be HxW, got {values.shape}')
    if absolute < 0.0 or relative < 0.0:
        raise ValueError('depth thresholds must be non-negative')
    valid = np.isfinite(values) & (values > 0.0)
    edges = np.zeros(values.shape, dtype=bool)
    with np.errstate(invalid='ignore'):
        horizontal = (valid[:, :-1] & valid[:, 1:] &
                      (np.abs(values[:, :-1] - values[:, 1:]) >
                       absolute + relative * np.minimum(
                           values[:, :-1], values[:, 1:])))
        vertical = (valid[:-1, :] & valid[1:, :] &
                    (np.abs(values[:-1, :] - values[1:, :]) >
                     absolute + relative * np.minimum(
                         values[:-1, :], values[1:, :])))
    edges[:, :-1] |= horizontal
    edges[:, 1:] |= horizontal
    edges[:-1, :] |= vertical
    edges[1:, :] |= vertical
    return edges


def nearest_edge_distances(query: np.ndarray, target: np.ndarray,
                           max_distance: int = 12) -> np.ndarray:
    """Find target-edge distance for query pixels with a bounded local search."""
    query_y, query_x = np.nonzero(query)
    if query_y.size == 0:
        return np.zeros(0, dtype=np.float32)
    distances = np.full(query_y.size, float(max_distance + 1), dtype=np.float32)
    height, width = target.shape
    offsets = [(dy, dx) for dy in range(-max_distance, max_distance + 1)
               for dx in range(-max_distance, max_distance + 1)
               if dy * dy + dx * dx <= max_distance * max_distance]
    offsets.sort(key=lambda item: item[0] * item[0] + item[1] * item[1])
    for dy, dx in offsets:
        distance = float(np.hypot(dy, dx))
        pending = distances > distance
        y = query_y + dy
        x = query_x + dx
        inside = pending & (y >= 0) & (y < height) & (x >= 0) & (x < width)
        hit = np.zeros(query_y.size, dtype=bool)
        hit[inside] = target[y[inside], x[inside]]
        distances[hit] = distance
    return distances


def projected_depth(points: np.ndarray, viewmat: np.ndarray, K: np.ndarray,
                    width: int, height: int) -> np.ndarray:
    """Project one view into an image containing sparse nearest depths."""
    (pixels, depths), = pcio.project_depth_maps(
        points, np.asarray(viewmat)[None], K, width, height)
    image = np.full((height, width), np.inf, dtype=np.float32)
    image.reshape(-1)[pixels] = depths
    return image


def score_view(points: np.ndarray, viewmat: np.ndarray, K: np.ndarray,
               image: np.ndarray, *, edge_percentile: float = 95.0,
               max_distance: int = 12) -> dict:
    """Score one camera view and return pixel-distance alignment metrics."""
    height, width = image.shape[:2]
    lidar_edges = depth_edges(projected_depth(points, viewmat, K, width, height))
    distances = nearest_edge_distances(
        lidar_edges, image_edges(image, edge_percentile), max_distance)
    if distances.size == 0:
        return {'edge_points': 0, 'median_px': None, 'p90_px': None,
                'inlier_2px': None, 'out_of_range_fraction': None}
    return {'edge_points': int(distances.size),
            'median_px': float(np.median(distances)),
            'p90_px': float(np.percentile(distances, 90)),
            'inlier_2px': float(np.mean(distances <= 2.0)),
            'out_of_range_fraction': float(np.mean(distances > max_distance))}


def correction_matrix(parameters: np.ndarray) -> np.ndarray:
    """Build camera-frame SE(3) correction from xyz metres and xyz radians."""
    tx, ty, tz, rx, ry, rz = np.asarray(parameters, dtype=np.float64).reshape(6)
    sx, cx = np.sin(rx), np.cos(rx)
    sy, cy = np.sin(ry), np.cos(ry)
    sz, cz = np.sin(rz), np.cos(rz)
    rotation_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rotation_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rotation_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3] = rotation_z @ rotation_y @ rotation_x
    result[:3, 3] = [tx, ty, tz]
    return result


def alignment_objective(points: np.ndarray, viewmats: np.ndarray,
                        K: np.ndarray, images: list[np.ndarray],
                        parameters: np.ndarray, *, edge_percentile: float = 95.0,
                        max_distance: int = 12,
                        reference_edge_points: int | None = None,
                        image_edge_masks: list[np.ndarray] | None = None
                        ) -> tuple[float, dict]:
    """Return coverage-guarded mean edge distance for one SE(3) correction."""
    delta = correction_matrix(parameters)
    chunks = []
    targets = (image_edge_masks if image_edge_masks is not None else
               [image_edges(image, edge_percentile) for image in images])
    for viewmat, image, target in zip(viewmats, images, targets):
        height, width = image.shape[:2]
        lidar_edges = depth_edges(projected_depth(
            points, delta @ viewmat, K, width, height))
        chunks.append(nearest_edge_distances(lidar_edges, target, max_distance))
    values = np.concatenate([item for item in chunks if item.size]) \
        if any(item.size for item in chunks) else np.zeros(0, np.float32)
    if not values.size:
        return float('inf'), {'edge_points': 0, 'mean_px': None}
    edge_points = int(values.size)
    reference = edge_points if reference_edge_points is None else reference_edge_points
    coverage = edge_points / max(reference, 1)
    penalty = max(0.0, 0.9 - coverage) * max_distance * 4.0
    loss = float(np.mean(values) + penalty)
    return loss, {'edge_points': edge_points, 'mean_px': float(np.mean(values)),
                  'median_px': float(np.median(values)),
                  'out_of_range_fraction': float(np.mean(values > max_distance)),
                  'coverage': coverage}


def optimize_correction(points: np.ndarray, viewmats: np.ndarray, K: np.ndarray,
                        images: list[np.ndarray], *, rounds: int = 3,
                        translation_step: float = 0.02,
                        rotation_step_deg: float = 0.2,
                        edge_percentile: float = 95.0,
                        max_distance: int = 12) -> tuple[np.ndarray, dict, dict]:
    """Coordinate-search a camera-frame correction, coarse to fine."""
    parameters = np.zeros(6, dtype=np.float64)
    base_loss, before = alignment_objective(
        points, viewmats, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance)
    best_loss = base_loss
    reference = before['edge_points']
    edge_masks = [image_edges(image, edge_percentile) for image in images]
    steps = np.array([translation_step] * 3 +
                     [np.deg2rad(rotation_step_deg)] * 3)
    for _ in range(rounds):
        for axis in range(6):
            for direction in (-1.0, 1.0):
                candidate = parameters.copy()
                candidate[axis] += direction * steps[axis]
                loss, _ = alignment_objective(
                    points, viewmats, K, images, candidate,
                    edge_percentile=edge_percentile,
                    max_distance=max_distance,
                    reference_edge_points=reference,
                    image_edge_masks=edge_masks)
                if loss < best_loss:
                    parameters, best_loss = candidate, loss
        steps *= 0.5
    _, after = alignment_objective(
        points, viewmats, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance,
        reference_edge_points=reference, image_edge_masks=edge_masks)
    before['loss'] = base_loss
    after['loss'] = best_loss
    return parameters, before, after


def frame_stamps(transforms: Path) -> np.ndarray:
    """Read the camera timestamps retained by ``extract_posed_images``."""
    document = json.loads(Path(transforms).read_text())
    try:
        stamps = np.asarray([frame['stamp'] for frame in document['frames']],
                            dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f'{transforms}: every frame needs a numeric stamp for temporal '
            'calibration') from exc
    if stamps.size == 0 or not np.all(np.isfinite(stamps)):
        raise ValueError(f'{transforms}: frame stamps must be finite and non-empty')
    return stamps


def infer_body_T_camera(samples: list[pi.TrajectorySample], stamps: np.ndarray,
                        viewmats: np.ndarray) -> tuple[np.ndarray, dict]:
    """Recover the static body<-camera transform from extracted camera poses."""
    estimates = []
    for stamp, viewmat in zip(stamps, viewmats):
        world_T_body = pi.interpolate_pose(samples, float(stamp))
        world_T_camera = np.linalg.inv(viewmat)
        estimates.append(np.linalg.inv(world_T_body) @ world_T_camera)
    translations = np.asarray([item[:3, 3] for item in estimates])
    centre = np.median(translations, axis=0)
    representative = estimates[int(np.argmin(
        np.linalg.norm(translations - centre, axis=1)))]
    translation_spread = np.linalg.norm(
        translations - representative[:3, 3], axis=1)
    rotation_spread = []
    for estimate in estimates:
        relative = representative[:3, :3].T @ estimate[:3, :3]
        cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
        rotation_spread.append(np.rad2deg(np.arccos(cosine)))
    consistency = {
        'frames': len(estimates),
        'translation_spread_p95_m': float(np.percentile(translation_spread, 95)),
        'rotation_spread_p95_deg': float(np.percentile(rotation_spread, 95)),
    }
    return representative, consistency


def trajectory_excitation(samples: list[pi.TrajectorySample],
                          stamps: np.ndarray) -> dict:
    """Summarise motion that makes a camera/LiDAR time offset observable."""
    poses = [pi.interpolate_pose(samples, float(stamp)) for stamp in stamps]
    translations = np.asarray([pose[:3, 3] for pose in poses])
    translation_path = float(np.linalg.norm(np.diff(translations, axis=0), axis=1).sum())
    rotation_path = 0.0
    for first, second in zip(poses, poses[1:]):
        relative = first[:3, :3].T @ second[:3, :3]
        cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
        rotation_path += float(np.rad2deg(np.arccos(cosine)))
    return {'translation_path_m': translation_path,
            'rotation_path_deg': rotation_path,
            'time_offset_observable': bool(
                translation_path >= 0.05 or rotation_path >= 1.0)}


def recompose_viewmats(samples: list[pi.TrajectorySample], stamps: np.ndarray,
                       body_T_camera: np.ndarray,
                       parameters: np.ndarray) -> np.ndarray:
    """Compose world-to-camera poses for a time and local SE(3) correction."""
    values = np.asarray(parameters, dtype=np.float64).reshape(7)
    time_offset = float(values[0])
    corrected_extrinsic = body_T_camera @ correction_matrix(values[1:])
    return np.asarray([
        np.linalg.inv(pi.interpolate_pose(samples, float(stamp + time_offset)) @
                      corrected_extrinsic)
        for stamp in stamps
    ])


def spatiotemporal_objective(
        points: np.ndarray, samples: list[pi.TrajectorySample],
        stamps: np.ndarray, body_T_camera: np.ndarray, K: np.ndarray,
        images: list[np.ndarray], parameters: np.ndarray, *,
        edge_percentile: float = 95.0, max_distance: int = 12,
        reference_edge_points: int | None = None,
        image_edge_masks: list[np.ndarray] | None = None) -> tuple[float, dict]:
    """Evaluate a continuous-time pose recomposition against image edges."""
    try:
        viewmats = recompose_viewmats(
            samples, stamps, body_T_camera, parameters)
    except ValueError:
        return float('inf'), {'edge_points': 0, 'mean_px': None,
                              'median_px': None, 'coverage': 0.0}
    return alignment_objective(
        points, viewmats, K, images, np.zeros(6),
        edge_percentile=edge_percentile, max_distance=max_distance,
        reference_edge_points=reference_edge_points,
        image_edge_masks=image_edge_masks)


def optimize_spatiotemporal(
        points: np.ndarray, samples: list[pi.TrajectorySample],
        stamps: np.ndarray, body_T_camera: np.ndarray, K: np.ndarray,
        images: list[np.ndarray], *, rounds: int = 3,
        time_step: float = 0.02, translation_step: float = 0.02,
        rotation_step_deg: float = 0.2, max_time_offset: float = 0.1,
        max_translation: float = 0.1, max_rotation_deg: float = 2.0,
        edge_percentile: float = 95.0,
        max_distance: int = 12) -> tuple[np.ndarray, dict, dict]:
    """Bounded deterministic coordinate search over dt plus a local SE(3)."""
    parameters = np.zeros(7, dtype=np.float64)
    base_loss, before = spatiotemporal_objective(
        points, samples, stamps, body_T_camera, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance)
    reference = before['edge_points']
    edge_masks = [image_edges(image, edge_percentile) for image in images]
    best_loss = base_loss
    steps = np.array([time_step] + [translation_step] * 3 +
                     [np.deg2rad(rotation_step_deg)] * 3)
    bounds = np.array([max_time_offset] + [max_translation] * 3 +
                      [np.deg2rad(max_rotation_deg)] * 3)
    for _ in range(rounds):
        changed = True
        while changed:
            changed = False
            for axis in range(7):
                for direction in (-1.0, 1.0):
                    candidate = parameters.copy()
                    candidate[axis] += direction * steps[axis]
                    if abs(candidate[axis]) > bounds[axis] + 1e-12:
                        continue
                    loss, _ = spatiotemporal_objective(
                        points, samples, stamps, body_T_camera, K, images,
                        candidate, edge_percentile=edge_percentile,
                        max_distance=max_distance,
                        reference_edge_points=reference,
                        image_edge_masks=edge_masks)
                    if loss + 1e-12 < best_loss:
                        parameters, best_loss = candidate, loss
                        changed = True
        steps *= 0.5
    _, after = spatiotemporal_objective(
        points, samples, stamps, body_T_camera, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance,
        reference_edge_points=reference, image_edge_masks=edge_masks)
    before['loss'] = base_loss
    after['loss'] = best_loss
    return parameters, before, after


def calibration_acceptance(train_before: dict, train_after: dict,
                           heldout_before: dict, heldout_after: dict, *,
                           minimum_edge_points: int,
                           minimum_heldout_improvement: float) -> tuple[bool, str | None]:
    """Apply the independent validation gate used before exporting poses."""
    enough_edges = (
        train_before['edge_points'] >= minimum_edge_points and
        heldout_before['edge_points'] >= minimum_edge_points)
    if not enough_edges:
        return False, 'insufficient_edge_support'
    train_loss = train_after['loss']
    heldout_loss = heldout_after['loss']
    heldout_limit = heldout_before['loss'] * (
        1.0 - minimum_heldout_improvement)
    heldout_failed = (heldout_loss > heldout_limit or
                      (minimum_heldout_improvement == 0.0 and
                       heldout_loss >= heldout_limit))
    if (not np.isfinite(train_loss) or not np.isfinite(heldout_loss) or
            train_loss >= train_before['loss'] or heldout_failed):
        return False, 'heldout_or_training_loss_did_not_improve'
    return True, None


def write_recomposed_transforms(source: Path, output: Path,
                                viewmats: np.ndarray) -> Path:
    """Write continuous-time recomposed poses while preserving frame metadata."""
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output:
        raise ValueError('recomposed transforms output must differ from source')
    document = json.loads(source.read_text())
    dataset = tg.load_transforms(source)
    if len(document['frames']) != len(viewmats):
        raise ValueError('frame and recomposed viewmat counts differ')
    for frame, viewmat, image_path in zip(
            document['frames'], viewmats, dataset['image_paths']):
        frame['transform_matrix'] = (
            np.linalg.inv(viewmat) @ pi.ROS_OPTICAL_TO_OPENGL).tolist()
        frame['file_path'] = os.path.relpath(image_path, output.parent)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2) + '\n')
    return output


def write_corrected_transforms(source: Path, output: Path,
                               correction: np.ndarray) -> Path:
    """Write corrected camera poses without modifying the source dataset."""
    source = Path(source).resolve()
    output = Path(output).resolve()
    if source == output:
        raise ValueError('corrected transforms output must differ from source')
    document = json.loads(source.read_text())
    dataset = tg.load_transforms(source)
    if len(document['frames']) != len(dataset['viewmats']):
        raise ValueError('frame and viewmat counts differ')
    for frame, viewmat, image_path in zip(
            document['frames'], dataset['viewmats'], dataset['image_paths']):
        corrected_c2w_cv = np.linalg.inv(correction @ viewmat)
        frame['transform_matrix'] = (
            corrected_c2w_cv @ pi.ROS_OPTICAL_TO_OPENGL).tolist()
        frame['file_path'] = os.path.relpath(image_path, output.parent)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2) + '\n')
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pointcloud', type=Path, required=True)
    parser.add_argument('--transforms', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--view-stride', type=int, default=10)
    parser.add_argument('--max-points', type=int, default=0,
                        help='deterministically subsample this many geometry '
                             'points for evaluation (0 keeps all)')
    parser.add_argument('--edge-percentile', type=float, default=95.0,
                        help='retain this percentile of nonzero image gradients; '
                             '95 focuses the metric on structural edges')
    parser.add_argument('--max-distance', type=int, default=12)
    optimization_mode = parser.add_mutually_exclusive_group()
    optimization_mode.add_argument('--optimize-extrinsic', action='store_true')
    optimization_mode.add_argument('--optimize-spatiotemporal', action='store_true')
    parser.add_argument('--trajectory', type=Path,
                        help='dense TUM world<-body trajectory; required for '
                             'spatiotemporal optimization')
    parser.add_argument('--optimization-rounds', type=int, default=3)
    parser.add_argument('--time-step', type=float, default=0.02)
    parser.add_argument('--translation-step', type=float, default=0.02)
    parser.add_argument('--rotation-step-deg', type=float, default=0.2)
    parser.add_argument('--max-time-offset', type=float, default=0.1)
    parser.add_argument('--max-translation', type=float, default=0.1)
    parser.add_argument('--max-rotation-deg', type=float, default=2.0)
    parser.add_argument('--holdout-modulo', type=int, default=5,
                        help='every Nth selected view is validation-only')
    parser.add_argument('--minimum-edge-points', type=int, default=50)
    parser.add_argument('--minimum-heldout-improvement', type=float, default=0.0,
                        help='fractional held-out loss reduction required to '
                             'accept and export the correction')
    parser.add_argument('--corrected-transforms-out', type=Path)
    args = parser.parse_args()
    if (args.view_stride < 1 or args.max_distance < 1 or
            args.optimization_rounds < 1 or args.translation_step <= 0.0 or
            args.rotation_step_deg <= 0.0 or args.time_step <= 0.0 or
            args.max_time_offset < 0.0 or args.max_translation < 0.0 or
            args.max_rotation_deg < 0.0 or args.holdout_modulo < 2 or
            args.minimum_edge_points < 1 or args.max_points < 0 or
            not 0.0 <= args.minimum_heldout_improvement < 1.0):
        raise SystemExit('stride, distance, rounds, and search steps must be > 0')
    if args.optimize_spatiotemporal and args.trajectory is None:
        raise SystemExit('--optimize-spatiotemporal requires --trajectory')
    import imageio.v3 as iio
    points, _ = pcio.read_ply_xyz(args.pointcloud)
    if args.max_points and len(points) > args.max_points:
        indices = np.linspace(
            0, len(points) - 1, args.max_points, dtype=np.int64)
        points = points[indices]
    dataset = tg.load_transforms(args.transforms)
    viewmats = np.asarray(dataset['viewmats'], dtype=np.float64)
    selected = list(range(0, len(viewmats), args.view_stride))
    images = [np.asarray(iio.imread(dataset['image_paths'][index]))
              for index in selected]
    optimization = None
    delta = np.eye(4)
    effective_viewmats = viewmats.copy()
    if args.optimize_spatiotemporal:
        samples = pi.read_tum_trajectory(args.trajectory)
        stamps = frame_stamps(args.transforms)
        if len(stamps) != len(viewmats):
            raise SystemExit('frame stamp and camera pose counts differ')
        body_T_camera, consistency = infer_body_T_camera(
            samples, stamps, viewmats)
        if (consistency['translation_spread_p95_m'] > 0.001 or
                consistency['rotation_spread_p95_deg'] > 0.05):
            raise SystemExit(
                'camera poses are inconsistent with one static body-to-camera '
                'extrinsic (p95 spread exceeds 1 mm or 0.05 degree)')
        excitation = trajectory_excitation(samples, stamps[selected])
        if not excitation['time_offset_observable']:
            raise SystemExit(
                'time offset is unobservable: selected views contain less than '
                '0.05 m translation and 1 degree rotation')
        heldout = [index for ordinal, index in enumerate(selected)
                   if ordinal % args.holdout_modulo == 0]
        train = [index for index in selected if index not in heldout]
        if not train or not heldout:
            raise SystemExit('spatiotemporal calibration needs train and held-out views')
        image_by_index = dict(zip(selected, images))
        train_images = [image_by_index[index] for index in train]
        heldout_images = [image_by_index[index] for index in heldout]
        parameters, train_before, train_after = optimize_spatiotemporal(
            points, samples, stamps[train], body_T_camera, dataset['K'],
            train_images, rounds=args.optimization_rounds,
            time_step=args.time_step, translation_step=args.translation_step,
            rotation_step_deg=args.rotation_step_deg,
            max_time_offset=args.max_time_offset,
            max_translation=args.max_translation,
            max_rotation_deg=args.max_rotation_deg,
            edge_percentile=args.edge_percentile,
            max_distance=args.max_distance)
        zero = np.zeros(7)
        heldout_before_loss, heldout_before = spatiotemporal_objective(
            points, samples, stamps[heldout], body_T_camera, dataset['K'],
            heldout_images, zero, edge_percentile=args.edge_percentile,
            max_distance=args.max_distance)
        heldout_after_loss, heldout_after = spatiotemporal_objective(
            points, samples, stamps[heldout], body_T_camera, dataset['K'],
            heldout_images, parameters,
            edge_percentile=args.edge_percentile,
            max_distance=args.max_distance,
            reference_edge_points=heldout_before['edge_points'])
        heldout_before['loss'], heldout_after['loss'] = (
            heldout_before_loss, heldout_after_loss)
        accepted, rejection_reason = calibration_acceptance(
            train_before, train_after, heldout_before, heldout_after,
            minimum_edge_points=args.minimum_edge_points,
            minimum_heldout_improvement=args.minimum_heldout_improvement)
        if accepted:
            effective_viewmats = recompose_viewmats(
                samples, stamps, body_T_camera, parameters)
        parameter_bounds = np.array([
            args.max_time_offset, args.max_translation,
            args.max_translation, args.max_translation,
            args.max_rotation_deg, args.max_rotation_deg,
            args.max_rotation_deg])
        reported_parameters = np.array(
            [parameters[0], *parameters[1:4],
             *np.rad2deg(parameters[4:])])
        names = ['dt', 'tx', 'ty', 'tz', 'roll', 'pitch', 'yaw']
        boundary_axes = [
            name for name, value, bound in zip(
                names, reported_parameters, parameter_bounds)
            if bound > 0.0 and abs(value) >= bound - 1e-12]
        optimization = {
            'accepted': accepted,
            'parameters_dt_s_xyz_m_rpy_deg': [float(parameters[0])] +
            parameters[1:4].tolist() + np.rad2deg(parameters[4:]).tolist(),
            'body_T_camera_initial': body_T_camera.tolist(),
            'extrinsic_consistency': consistency,
            'trajectory_excitation': excitation,
            'train_view_indices': train,
            'heldout_view_indices': heldout,
            'minimum_edge_points': args.minimum_edge_points,
            'search_bounds_dt_s_xyz_m_rpy_deg': parameter_bounds.tolist(),
            'boundary_axes': boundary_axes,
            'train': {'before': train_before, 'after': train_after},
            'heldout': {'before': heldout_before, 'after': heldout_after},
            'rejection_reason': rejection_reason,
        }
    elif args.optimize_extrinsic:
        parameters, before, after = optimize_correction(
            points, viewmats[selected], dataset['K'], images,
            rounds=args.optimization_rounds,
            translation_step=args.translation_step,
            rotation_step_deg=args.rotation_step_deg,
            edge_percentile=args.edge_percentile,
            max_distance=args.max_distance)
        delta = correction_matrix(parameters)
        effective_viewmats = np.asarray(
            [delta @ viewmat for viewmat in viewmats])
        optimization = {
            'parameters_xyz_m_rpy_deg': parameters[:3].tolist() +
            np.rad2deg(parameters[3:]).tolist(),
            'camera_correction_matrix': delta.tolist(),
            'before': before, 'after': after,
        }
    per_view = []
    for index, image in zip(selected, images):
        score = score_view(
            points, effective_viewmats[index], dataset['K'], image,
            edge_percentile=args.edge_percentile,
            max_distance=args.max_distance)
        score['view_index'] = index
        per_view.append(score)
    valid = [item for item in per_view if item['edge_points'] > 0]
    if not valid:
        raise SystemExit('no LiDAR depth edges were measurable')
    weights = np.asarray([item['edge_points'] for item in valid], dtype=float)
    report = {'pointcloud': str(args.pointcloud.resolve()),
              'transforms': str(args.transforms.resolve()),
              'points_scored': len(points),
              'views_scored': len(valid), 'edge_points': int(weights.sum())}
    for name in ('median_px', 'p90_px', 'inlier_2px',
                 'out_of_range_fraction'):
        report[f'weighted_{name}'] = float(np.average(
            [item[name] for item in valid], weights=weights))
    if args.optimize_spatiotemporal:
        report['spatiotemporal_optimization'] = optimization
        if args.corrected_transforms_out is not None:
            corrected = write_recomposed_transforms(
                args.transforms, args.corrected_transforms_out,
                effective_viewmats)
            report['corrected_transforms'] = str(corrected)
    elif optimization is not None:
        report['extrinsic_optimization'] = optimization
        if args.corrected_transforms_out is not None:
            corrected = write_corrected_transforms(
                args.transforms, args.corrected_transforms_out, delta)
            report['corrected_transforms'] = str(corrected)
    elif args.corrected_transforms_out is not None:
        raise SystemExit('--corrected-transforms-out requires an optimization mode')
    report['per_view'] = per_view
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items()
                      if key != 'per_view'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
