#!/usr/bin/env python3
"""Extract deterministic high-confidence monocular relative-pose constraints."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


OPENGL_TO_OPENCV = np.diag([1.0, -1.0, -1.0, 1.0])


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def image_set_sha256(transforms_path: Path, frames: list[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    for frame in frames:
        path = (transforms_path.parent / frame['file_path']).resolve()
        digest.update(str(path).encode())
        digest.update(b'\0')
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def camera_pose_opencv(frame: dict[str, Any]) -> np.ndarray:
    """Return world<-camera in OpenCV axes from a NeRF/OpenGL frame."""
    pose = np.asarray(frame['transform_matrix'], dtype=np.float64)
    if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
        raise ValueError('frame transform_matrix must be a finite 4x4 matrix')
    return pose @ OPENGL_TO_OPENCV


def rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(float(cosine)))


def evaluate_gate(*, tracks: int, inliers: int, rotation_error_deg: float,
                  translation_cosine: float, predicted_translation_m: float,
                  min_tracks: int, min_inliers: int, min_inlier_ratio: float,
                  max_rotation_error_deg: float,
                  min_translation_cosine: float,
                  min_translation_m: float, max_translation_m: float
                  ) -> tuple[bool, list[str]]:
    """Apply the frozen visual-confidence gate and report rejection reasons."""
    reasons = []
    ratio = inliers / max(1, tracks)
    checks = [
        (tracks >= min_tracks, 'insufficient_tracks'),
        (inliers >= min_inliers, 'insufficient_inliers'),
        (ratio >= min_inlier_ratio, 'low_inlier_ratio'),
        (rotation_error_deg <= max_rotation_error_deg, 'rotation_disagreement'),
        (translation_cosine >= min_translation_cosine, 'translation_disagreement'),
        (predicted_translation_m >= min_translation_m, 'insufficient_baseline'),
        (predicted_translation_m <= max_translation_m, 'excessive_baseline'),
    ]
    reasons.extend(reason for passed, reason in checks if not passed)
    return not reasons, reasons


def _read_gray(path: Path):
    import cv2
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f'failed to read image: {path}')
    return image


def essential_candidates(essential: np.ndarray) -> list[np.ndarray]:
    """Split OpenCV's one-or-more essential matrices deterministically."""
    matrix = np.asarray(essential, dtype=np.float64)
    if matrix.ndim != 2:
        raise ValueError('essential matrix result must be two-dimensional')
    if matrix.shape == (3, 3):
        return [matrix]
    if matrix.shape[1] == 3 and matrix.shape[0] % 3 == 0:
        return [matrix[row:row + 3] for row in range(0, matrix.shape[0], 3)]
    if matrix.shape[0] == 3 and matrix.shape[1] % 3 == 0:
        return [matrix[:, column:column + 3]
                for column in range(0, matrix.shape[1], 3)]
    raise ValueError(f'unexpected essential matrix shape: {matrix.shape}')


def estimate_pair(previous, current, intrinsic: np.ndarray, args) -> dict[str, Any]:
    """Estimate current_camera<-previous_camera from tracked image corners."""
    import cv2
    corners = cv2.goodFeaturesToTrack(
        previous, args.max_features, args.feature_quality,
        args.feature_min_distance, blockSize=args.feature_block_size)
    if corners is None:
        return {'tracks': 0, 'inliers': 0, 'valid': False,
                'failure': 'no_features'}
    tracked, status, _ = cv2.calcOpticalFlowPyrLK(
        previous, current, corners, None,
        winSize=(args.lk_window, args.lk_window), maxLevel=args.lk_levels,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                  args.lk_iterations, args.lk_epsilon))
    backward, backward_status, _ = cv2.calcOpticalFlowPyrLK(
        current, previous, tracked, None,
        winSize=(args.lk_window, args.lk_window), maxLevel=args.lk_levels,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                  args.lk_iterations, args.lk_epsilon))
    forward_backward = np.linalg.norm(backward - corners, axis=2)[:, 0]
    keep = ((status[:, 0] > 0) & (backward_status[:, 0] > 0) &
            (forward_backward <= args.max_forward_backward_error))
    first = corners[keep, 0]
    second = tracked[keep, 0]
    tracks = int(len(first))
    if tracks < 5:
        return {'tracks': tracks, 'inliers': 0, 'valid': False,
                'failure': 'insufficient_tracks_for_essential'}
    essential, mask = cv2.findEssentialMat(
        first, second, intrinsic, cv2.RANSAC, args.ransac_probability,
        args.ransac_threshold_px)
    if essential is None:
        return {'tracks': tracks, 'inliers': 0, 'valid': False,
                'failure': 'essential_failed'}
    solutions = []
    for candidate_index, candidate in enumerate(essential_candidates(essential)):
        inliers, rotation, translation, pose_mask = cv2.recoverPose(
            candidate, first, second, intrinsic,
            mask=None if mask is None else mask.copy())
        solutions.append((int(inliers), -candidate_index, rotation,
                          translation, pose_mask))
    inliers, _, rotation, translation, pose_mask = max(
        solutions, key=lambda solution: solution[:2])
    return {
        'tracks': tracks, 'inliers': int(inliers), 'valid': True,
        'inlier_ratio': float(inliers / max(1, tracks)),
        'rotation': np.asarray(rotation).tolist(),
        'translation_direction':
            (np.asarray(translation)[:, 0] /
             np.linalg.norm(translation)).tolist(),
        'pose_inlier_mask_count': int(np.count_nonzero(pose_mask)),
    }


def extract(args) -> dict[str, Any]:
    import cv2
    cv2.setNumThreads(1)
    cv2.setRNGSeed(0)
    source = json.loads(args.transforms.read_text())
    frames = source['frames']
    intrinsic = np.array([
        [source['fl_x'], 0.0, source['cx']],
        [0.0, source['fl_y'], source['cy']],
        [0.0, 0.0, 1.0]], dtype=np.float64)
    constraints = []
    accepted = 0
    for first_index in range(0, len(frames) - args.stride, args.stride):
        second_index = first_index + args.stride
        first_frame, second_frame = frames[first_index], frames[second_index]
        first_path = (args.transforms.parent / first_frame['file_path']).resolve()
        second_path = (args.transforms.parent / second_frame['file_path']).resolve()
        estimate = estimate_pair(
            _read_gray(first_path), _read_gray(second_path), intrinsic, args)
        record: dict[str, Any] = {
            'first_index': first_index, 'second_index': second_index,
            'first_stamp': float(first_frame['stamp']),
            'second_stamp': float(second_frame['stamp']), **estimate}
        if estimate['valid']:
            first_pose = camera_pose_opencv(first_frame)
            second_pose = camera_pose_opencv(second_frame)
            predicted = np.linalg.inv(second_pose) @ first_pose
            measured_rotation = np.asarray(estimate['rotation'])
            rotation_error = rotation_angle_deg(
                measured_rotation @ predicted[:3, :3].T)
            predicted_translation = predicted[:3, 3]
            translation_norm = float(np.linalg.norm(predicted_translation))
            direction = np.asarray(estimate['translation_direction'])
            cosine = (float(direction @ predicted_translation / translation_norm)
                      if translation_norm > 1.0e-12 else -1.0)
            use, reasons = evaluate_gate(
                tracks=estimate['tracks'], inliers=estimate['inliers'],
                rotation_error_deg=rotation_error,
                translation_cosine=cosine,
                predicted_translation_m=translation_norm,
                min_tracks=args.min_tracks, min_inliers=args.min_inliers,
                min_inlier_ratio=args.min_inlier_ratio,
                max_rotation_error_deg=args.max_rotation_error_deg,
                min_translation_cosine=args.min_translation_cosine,
                min_translation_m=args.min_translation_m,
                max_translation_m=args.max_translation_m)
            record.update({
                'rotation_error_deg': rotation_error,
                'translation_cosine': cosine,
                'predicted_translation_m': translation_norm,
                'accepted': use, 'rejection_reasons': reasons})
            accepted += int(use)
        else:
            record.update({'accepted': False,
                           'rejection_reasons': [estimate['failure']]})
        constraints.append(record)
    return {
        'schema_version': 1,
        'method': 'lk_forward_backward_essential_ransac',
        'inputs': {
            'transforms': str(args.transforms.resolve()),
            'transforms_sha256': sha256_file(args.transforms),
            'image_set_sha256': image_set_sha256(args.transforms, frames),
            'frames': len(frames)},
        'config': {key: getattr(args, key) for key in (
            'stride', 'max_features', 'feature_quality',
            'feature_min_distance', 'feature_block_size', 'lk_window',
            'lk_levels', 'lk_iterations', 'lk_epsilon',
            'max_forward_backward_error', 'ransac_probability',
            'ransac_threshold_px', 'min_tracks', 'min_inliers',
            'min_inlier_ratio', 'max_rotation_error_deg',
            'min_translation_cosine', 'min_translation_m',
            'max_translation_m')},
        'summary': {'pairs': len(constraints), 'accepted': accepted,
                    'accepted_fraction': accepted / max(1, len(constraints))},
        'constraints': constraints,
    }


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--transforms', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stride', type=int, default=1)
    p.add_argument('--max-features', type=int, default=1200)
    p.add_argument('--feature-quality', type=float, default=0.01)
    p.add_argument('--feature-min-distance', type=float, default=8.0)
    p.add_argument('--feature-block-size', type=int, default=7)
    p.add_argument('--lk-window', type=int, default=31)
    p.add_argument('--lk-levels', type=int, default=4)
    p.add_argument('--lk-iterations', type=int, default=30)
    p.add_argument('--lk-epsilon', type=float, default=0.01)
    p.add_argument('--max-forward-backward-error', type=float, default=1.0)
    p.add_argument('--ransac-probability', type=float, default=0.999)
    p.add_argument('--ransac-threshold-px', type=float, default=1.5)
    p.add_argument('--min-tracks', type=int, default=80)
    p.add_argument('--min-inliers', type=int, default=50)
    p.add_argument('--min-inlier-ratio', type=float, default=0.2)
    p.add_argument('--max-rotation-error-deg', type=float, default=3.0)
    p.add_argument('--min-translation-cosine', type=float, default=0.5)
    p.add_argument('--min-translation-m', type=float, default=0.03)
    p.add_argument('--max-translation-m', type=float, default=2.0)
    return p


def main() -> int:
    args = parser().parse_args()
    if args.stride < 1:
        raise ValueError('stride must be positive')
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    report = extract(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    print(json.dumps(report['summary'], indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(2)
