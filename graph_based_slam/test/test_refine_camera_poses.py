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

"""Tests for per-view camera pose refinement against the LiDAR map."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'colored_map'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import pointcloud_io as pcio  # noqa: E402, I100
import refine_camera_poses as rcp  # noqa: E402

GL_FLIP = np.diag([1.0, -1.0, -1.0, 1.0])
WIDTH, HEIGHT, FOCAL = 320, 240, 220.0
K = np.array([[FOCAL, 0.0, WIDTH / 2], [0.0, FOCAL, HEIGHT / 2], [0.0, 0.0, 1.0]])
# A 8 x 8 x 3 m room: (axis, offset) of the floor and the four walls.
PLANES = [(2, 0.0), (0, -4.0), (0, 4.0), (1, -4.0), (1, 4.0)]


def _texture(points: np.ndarray, plane: int) -> np.ndarray:
    """Random grey level per 8 cm cell of the plane: many unique corners."""
    axis = PLANES[plane][0]
    uv = np.delete(points, axis, axis=1)
    cells = np.floor(uv / 0.08).astype(np.int64) + 1000
    key = cells[:, 0] * 73856093 ^ cells[:, 1] * 19349663 ^ (plane + 1) * 83492791
    return (key % 211 + 30).astype(np.uint8)


def _look(eye, target) -> np.ndarray:
    forward = np.asarray(target, float) - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    w2c = np.eye(4)
    w2c[:3, :3] = np.stack([right, down, forward])
    w2c[:3, 3] = -w2c[:3, :3] @ eye
    return w2c


def _render(w2c: np.ndarray) -> np.ndarray:
    """Ray-cast the textured room."""
    u, v = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    rays_camera = np.stack([(u - K[0, 2]) / FOCAL, (v - K[1, 2]) / FOCAL,
                            np.ones_like(u, float)], axis=-1).reshape(-1, 3)
    rotation = w2c[:3, :3].T
    origin = -rotation @ w2c[:3, 3]
    rays = rays_camera @ rotation.T
    best_t = np.full(len(rays), np.inf)
    best_plane = np.full(len(rays), -1)
    for index, (axis, offset) in enumerate(PLANES):
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (offset - origin[axis]) / rays[:, axis]
        closer = (t > 1e-6) & (t < best_t)
        best_t[closer] = t[closer]
        best_plane[closer] = index
    image = np.zeros(len(rays), np.uint8)
    for index in range(len(PLANES)):
        hit = best_plane == index
        image[hit] = _texture(origin + rays[hit] * best_t[hit, None], index)
    return image.reshape(HEIGHT, WIDTH)


def _room_points(spacing: float = 0.03) -> np.ndarray:
    grid = np.arange(-4.0, 4.0 + 1e-9, spacing)
    height = np.arange(0.0, 3.0 + 1e-9, spacing)
    a, b = np.meshgrid(grid, grid)
    floor = np.c_[a.ravel(), b.ravel(), np.zeros(a.size)]
    walls = []
    for axis, offset in PLANES[1:]:
        s, z = np.meshgrid(grid, height)
        wall = np.zeros((s.size, 3))
        wall[:, axis] = offset
        wall[:, 1 - axis] = s.ravel()
        wall[:, 2] = z.ravel()
        walls.append(wall)
    return np.vstack([floor, *walls])


def _rotation(axis_angle_deg: np.ndarray) -> np.ndarray:
    return cv2.Rodrigues(np.radians(axis_angle_deg).reshape(3, 1))[0]


def _write_scene(tmp_path: Path, true_poses, recorded_poses) -> Path:
    frames = []
    for index, (truth, recorded) in enumerate(zip(true_poses, recorded_poses)):
        name = f'images/{index:05d}.png'
        (tmp_path / 'images').mkdir(exist_ok=True)
        cv2.imwrite(str(tmp_path / name), _render(truth))
        frames.append({'file_path': name,
                       'transform_matrix': (np.linalg.inv(recorded) @ GL_FLIP).tolist()})
    document = {'camera_model': 'OPENCV', 'w': WIDTH, 'h': HEIGHT,
                'fl_x': FOCAL, 'fl_y': FOCAL, 'cx': K[0, 2], 'cy': K[1, 2],
                'frames': frames}
    path = tmp_path / 'transforms.json'
    path.write_text(json.dumps(document))
    pcio.write_ply(tmp_path / 'map.ply', _room_points(),
                   np.full((len(_room_points()), 3), 128, np.uint8))
    return path


def _walk(count: int = 14) -> list[np.ndarray]:
    """Walk a camera along the room, looking at the far wall and floor."""
    poses = []
    for index in range(count):
        x = -2.0 + 4.0 * index / (count - 1)
        eye = np.array([x, -2.5, 1.5])
        poses.append(_look(eye, [x + 0.8 * np.sin(index), 4.0, 0.6]))
    return poses


def _perturb(poses, seed: int = 3):
    rng = np.random.default_rng(seed)
    perturbed = []
    for pose in poses:
        delta = np.eye(4)
        delta[:3, :3] = _rotation(rng.normal(0.0, 0.8, 3))
        delta[:3, 3] = rng.normal(0.0, 0.03, 3)
        perturbed.append(delta @ pose)
    return perturbed


def _args(tmp_path: Path, transforms: Path, **overrides):
    args = rcp.build_parser().parse_args([
        '--transforms', str(transforms), '--pointcloud', str(tmp_path / 'map.ply'),
        '--out', str(tmp_path / 'refined.json'), '--image-scale', '1.0',
        '--iterations', '4', '--max-features', '2000'])
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def test_sampson_error_is_zero_for_true_poses_and_grows_with_rotation():
    truth = _walk(2)
    points = np.array([[0.5, 4.0, 1.0], [-1.0, 4.0, 2.0], [1.5, 4.0, 0.3]])
    pixels = []
    for pose in truth:
        camera = points @ pose[:3, :3].T + pose[:3, 3]
        pixels.append(camera[:, :2] / camera[:, 2:3] * FOCAL + K[:2, 2])
    exact = rcp.sampson_errors(truth[0], truth[1], K, *pixels)
    assert np.max(exact) < 1e-6
    # A pitch moves points across the (horizontal) epipolar lines.
    rotated = truth[1].copy()
    rotated[:3, :3] = _rotation(np.array([1.0, 0.0, 0.0])) @ rotated[:3, :3]
    assert np.median(rcp.sampson_errors(truth[0], rotated, K, *pixels)) > 1.0
    assert rcp.sampson_errors(truth[0], truth[0], K, *pixels) is None


def test_scaled_intrinsics_keep_pixel_centres():
    scaled = rcp.scale_camera_matrix(K, 0.5)
    assert scaled[0, 0] == pytest.approx(FOCAL / 2)
    assert scaled[0, 2] == pytest.approx((K[0, 2] + 0.5) / 2 - 0.5)


def test_frame_pairs_stay_inside_one_capture():
    assert rcp.frame_pairs([0, 0, 0, 1, 1], [1]) == [(0, 1), (1, 2), (3, 4)]


def test_perturbed_poses_are_pulled_back_to_the_map(tmp_path):
    truth = _walk()
    recorded = _perturb(truth)
    transforms = _write_scene(tmp_path, truth, recorded)
    report = rcp.run(_args(tmp_path, transforms), log=lambda *_: None)

    assert report['adopted']
    before = report['heldout_epipolar_recorded']['median_px']
    after = report['heldout_epipolar_refined']['median_px']
    assert after < 0.3 * before

    refined = rcp.load_transforms(tmp_path / 'refined.json')['viewmats']
    error_before = [rcp.pose_change(t, r)[0] for t, r in zip(truth, recorded)]
    error_after = [rcp.pose_change(t, r)[0] for t, r in zip(truth, refined)]
    # Disagreement between nearby views goes first; an error shared by
    # neighbouring views fades more slowly (about half after four rounds).
    assert np.median(error_after) < 0.6 * np.median(error_before)
    document = json.loads((tmp_path / 'refined.json').read_text())
    assert document['camera_pose_refinement']['adopted']
    assert document['frames'][0]['file_path'] == 'images/00000.png'


def test_poses_are_kept_when_the_check_does_not_improve(tmp_path):
    truth = _walk()
    transforms = _write_scene(tmp_path, truth, truth)
    report = rcp.run(_args(tmp_path, transforms, iterations=1),
                     log=lambda *_: None)

    assert not report['adopted']
    written = rcp.load_transforms(tmp_path / 'refined.json')['viewmats']
    assert all(np.allclose(w, t, atol=1e-9) for w, t in zip(written, truth))


def test_views_that_would_move_too_far_keep_the_recorded_pose(tmp_path):
    truth = _walk()
    recorded = _perturb(truth)
    transforms = _write_scene(tmp_path, truth, recorded)
    report = rcp.run(_args(tmp_path, transforms, iterations=1,
                           max_rotation_deg=0.01, min_improvement=-1.0),
                     log=lambda *_: None)

    assert report['rounds'][0]['views_reset_to_recorded'] > 0
    assert report['rounds'][0]['refined_views'] == 0
