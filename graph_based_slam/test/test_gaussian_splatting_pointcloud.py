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

"""Tests for PLY I/O, voxel downsampling, and LiDAR init transform (ROS-free)."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import build_lidar_init
    import pointcloud_io

    return pointcloud_io, build_lidar_init


pcio, bli = _load()


# --------------------------------------------------------------------------- #
# PLY round-trip
# --------------------------------------------------------------------------- #
def test_write_read_ply_xyz_only(tmp_path):
    xyz = np.array([[1.0, 2.0, 3.0], [-4.0, 5.0, 6.0]], dtype=np.float32)
    out = pcio.write_ply(tmp_path / 'p.ply', xyz)
    got, rgb = pcio.read_ply_xyz(out)
    np.testing.assert_allclose(got, xyz, atol=1e-6)
    assert rgb is None


def test_write_read_ply_with_rgb(tmp_path):
    xyz = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]], dtype=np.float32)
    rgb = np.array([[255, 0, 0], [0, 128, 64]], dtype=np.uint8)
    out = pcio.write_ply(tmp_path / 'p.ply', xyz, rgb)
    got, got_rgb = pcio.read_ply_xyz(out)
    np.testing.assert_allclose(got, xyz, atol=1e-6)
    np.testing.assert_array_equal(got_rgb, rgb)


def test_read_ascii_ply(tmp_path):
    text = ('ply\nformat ascii 1.0\nelement vertex 2\n'
            'property float x\nproperty float y\nproperty float z\n'
            'end_header\n1 2 3\n4 5 6\n')
    p = tmp_path / 'a.ply'
    p.write_text(text)
    got, rgb = pcio.read_ply_xyz(p)
    np.testing.assert_allclose(got, [[1, 2, 3], [4, 5, 6]], atol=1e-6)
    assert rgb is None


# --------------------------------------------------------------------------- #
# Voxel downsampling
# --------------------------------------------------------------------------- #
def test_voxel_downsample_collapses_close_points():
    xyz = np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [1.0, 0.0, 0.0]])
    out, _ = pcio.voxel_downsample(xyz, 0.1)
    assert out.shape[0] == 2  # first two share a voxel


def test_voxel_downsample_noop_when_zero():
    xyz = np.random.default_rng(0).normal(size=(10, 3))
    out, _ = pcio.voxel_downsample(xyz, 0.0)
    assert out.shape[0] == 10


def test_voxel_downsample_keeps_rgb_alignment():
    # Two points in distinct voxels plus a duplicate of the first; the kept rgb
    # must stay paired with its own xyz (first occurrence wins). Asserting the
    # actual values -- not just the row count -- is what pins the pairing.
    xyz = np.array([[0.0, 0.0, 0.0], [5.0, 5.0, 5.0], [0.02, 0.0, 0.0]])
    rgb = np.array([[10, 20, 30], [40, 50, 60], [70, 80, 90]], dtype=np.uint8)
    out, out_rgb = pcio.voxel_downsample(xyz, 0.1, rgb)
    assert out.shape[0] == 2
    order = np.lexsort(out.T[::-1])  # stable order for comparison
    np.testing.assert_allclose(out[order], [[0.0, 0.0, 0.0], [5.0, 5.0, 5.0]],
                               atol=1e-6)
    np.testing.assert_array_equal(out_rgb[order], [[10, 20, 30], [40, 50, 60]])


# --------------------------------------------------------------------------- #
# LiDAR init point transform
# --------------------------------------------------------------------------- #
def test_transform_points_translation():
    T = np.eye(4)
    T[:3, 3] = [1.0, 2.0, 3.0]
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    out = bli.transform_points(pts, T)
    np.testing.assert_allclose(out, [[1, 2, 3], [2, 2, 3]], atol=1e-9)


def test_transform_points_rotation_90z():
    T = np.eye(4)
    T[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]  # +90 deg about z
    out = bli.transform_points(np.array([[1.0, 0.0, 0.0]]), T)
    np.testing.assert_allclose(out, [[0, 1, 0]], atol=1e-9)


def test_compose_world_lidar_applies_rig_extrinsic_before_body_pose():
    world_T_body = np.eye(4)
    world_T_body[:3, 3] = [10.0, 0.0, 0.0]
    body_T_lidar = np.eye(4)
    body_T_lidar[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    body_T_lidar[:3, 3] = [0.0, 2.0, 0.0]
    world_T_lidar = bli.compose_world_lidar(world_T_body, body_T_lidar)
    out = bli.transform_points(np.array([[1.0, 0.0, 0.0]]), world_T_lidar)
    np.testing.assert_allclose(out, [[10.0, 3.0, 0.0]], atol=1e-9)


# --------------------------------------------------------------------------- #
# colorize_by_projection
# --------------------------------------------------------------------------- #
def _cam():
    # identity w2c (camera at origin, +z forward), 100x100, principal point centre
    K = np.array([[100.0, 0, 50.0], [0, 100.0, 50.0], [0, 0, 1.0]])
    return np.eye(4)[None], K, 100, 100


def test_colorize_samples_centre_pixel():
    vms, K, W, H = _cam()
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[50, 50] = [255, 0, 0]              # the pixel the on-axis point lands on
    pts = np.array([[0.0, 0.0, 5.0]])      # projects to (cx, cy) = (50, 50)
    rgb, seen = pcio.colorize_by_projection(pts, vms, K, [img], W, H)
    assert seen[0]
    np.testing.assert_array_equal(rgb[0], [255, 0, 0])


def test_colorize_behind_camera_is_unseen():
    vms, K, W, H = _cam()
    img = np.full((H, W, 3), 200, dtype=np.uint8)
    pts = np.array([[0.0, 0.0, -5.0]])     # behind the camera (z < 0)
    rgb, seen = pcio.colorize_by_projection(pts, vms, K, [img], W, H,
                                            default_rgb=(7, 7, 7))
    assert not seen[0]
    np.testing.assert_array_equal(rgb[0], [7, 7, 7])


def test_colorize_out_of_frame_is_unseen():
    vms, K, W, H = _cam()
    img = np.full((H, W, 3), 200, dtype=np.uint8)
    pts = np.array([[10.0, 0.0, 5.0]])     # u = 100*10/5 + 50 = 250 -> off image
    _, seen = pcio.colorize_by_projection(pts, vms, K, [img], W, H)
    assert not seen[0]


def test_colorize_averages_over_views():
    vms1, K, W, H = _cam()
    red = np.zeros((H, W, 3), dtype=np.uint8)
    red[50, 50] = [200, 0, 0]
    blue = np.zeros((H, W, 3), dtype=np.uint8)
    blue[50, 50] = [0, 0, 100]
    vms = np.concatenate([vms1, vms1], axis=0)  # same pose twice
    pts = np.array([[0.0, 0.0, 5.0]])
    rgb, seen = pcio.colorize_by_projection(pts, vms, K, [red, blue], W, H)
    assert seen[0]
    np.testing.assert_array_equal(rgb[0], [100, 0, 50])  # mean of the two


# --------------------------------------------------------------------------- #
# colorize_by_projection_robust
# --------------------------------------------------------------------------- #
def test_colorize_robust_occluded_point_is_unseen():
    vms, K, W, H = _cam()
    img = np.full((H, W, 3), 200, dtype=np.uint8)
    # Two points on the same camera ray: the far one is hidden by the near one.
    pts = np.array([[0.0, 0.0, 2.0], [0.0, 0.0, 8.0]])
    rgb, seen = pcio.colorize_by_projection_robust(
        pts, vms, K, [img], W, H, default_rgb=(7, 7, 7),
        normalize_exposure=False)
    assert seen[0] and not seen[1]
    np.testing.assert_array_equal(rgb[0], [200, 200, 200])
    np.testing.assert_array_equal(rgb[1], [7, 7, 7])


def test_colorize_robust_neighbouring_pixel_does_not_false_occlude():
    vms, K, W, H = _cam()
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[50, 50] = [200, 0, 0]
    img[50, 51] = [0, 200, 0]
    # These land in adjacent pixels but share a 4x4 coarse bin. The near red
    # point must not incorrectly hide the farther green surface.
    pts = np.array([[0.0, 0.0, 2.0], [0.08, 0.0, 8.0]])
    rgb, seen = pcio.colorize_by_projection_robust(
        pts, vms, K, [img], W, H, normalize_exposure=False,
        interp='nearest')
    assert seen.tolist() == [True, True]
    np.testing.assert_array_equal(rgb, [[200, 0, 0], [0, 200, 0]])

    _, coarse_seen = pcio.colorize_by_projection_robust(
        pts, vms, K, [img], W, H, normalize_exposure=False,
        interp='nearest', zbuf_bin=4)
    assert coarse_seen.tolist() == [True, False]


def test_colorize_robust_median_rejects_outlier_view():
    vms1, K, W, H = _cam()
    imgs = []
    for val in ((10, 10, 10), (10, 10, 10), (250, 0, 0)):  # one specular flash
        img = np.zeros((H, W, 3), dtype=np.uint8)
        img[50, 50] = val
        imgs.append(img)
    vms = np.concatenate([vms1] * 3, axis=0)
    pts = np.array([[0.0, 0.0, 5.0]])
    rgb, seen = pcio.colorize_by_projection_robust(
        pts, vms, K, imgs, W, H, normalize_exposure=False)
    assert seen[0]
    np.testing.assert_array_equal(rgb[0], [10, 10, 10])


def test_colorize_robust_exposure_normalization_rescales_bright_view():
    vms1, K, W, H = _cam()
    dark = np.full((H, W, 3), 60, dtype=np.uint8)
    bright = np.full((H, W, 3), 180, dtype=np.uint8)
    # The point is only visible in the bright view (the dark views look away).
    away = np.eye(4)
    away[:3, 3] = [1000.0, 0.0, 0.0]
    vms = np.stack([away, away, np.eye(4)])
    pts = np.array([[0.0, 0.0, 5.0]])
    rgb, seen = pcio.colorize_by_projection_robust(
        pts, vms, K, [dark, dark, bright], W, H, normalize_exposure=True)
    assert seen[0]
    # Global median luminance is the dark 60; the bright view is scaled by 1/3.
    assert abs(int(rgb[0][0]) - 60) <= 1


def test_colorize_robust_rejects_bad_zbuf_bin():
    vms, K, W, H = _cam()
    img = np.zeros((H, W, 3), dtype=np.uint8)
    with np.testing.assert_raises(ValueError):
        pcio.colorize_by_projection_robust(np.zeros((1, 3)), vms, K, [img], W, H,
                                           zbuf_bin=0)


def test_colorize_robust_bilinear_blends_neighbouring_pixels():
    vms, K, W, H = _cam()
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[50, 50] = [100, 100, 100]
    img[50, 51] = [200, 200, 200]
    # x = 0.02 -> u = 100*0.02/5 + 50 = 50.4 (40 % of the way to pixel 51).
    pts = np.array([[0.02, 0.0, 5.0]])
    rgb, seen = pcio.colorize_by_projection_robust(
        pts, vms, K, [img], W, H, normalize_exposure=False, interp='bilinear')
    assert seen[0]
    # 0.6*100 + 0.4*200 = 140 on every channel.
    np.testing.assert_array_equal(rgb[0], [140, 140, 140])
    # Nearest snaps to pixel 50 -> the un-blended 100.
    rgb_n, _ = pcio.colorize_by_projection_robust(
        pts, vms, K, [img], W, H, normalize_exposure=False, interp='nearest')
    np.testing.assert_array_equal(rgb_n[0], [100, 100, 100])


def test_colorize_robust_prefers_nearest_views_when_full():
    _, K, W, H = _cam()
    red = np.full((H, W, 3), [200, 0, 0], dtype=np.uint8)     # far / wrong colour
    green = np.full((H, W, 3), [0, 200, 0], dtype=np.uint8)   # near / true colour
    # Same on-axis point; a per-view +z shift changes only its camera depth.
    far, near0, near1 = np.eye(4), np.eye(4), np.eye(4)
    far[2, 3], near0[2, 3], near1[2, 3] = 20.0, 0.0, 1.0
    vms = np.stack([far, near0, near1])
    pts = np.array([[0.0, 0.0, 5.0]])
    # Budget of 2 samples, seen by 3 views: the far red view must be evicted.
    rgb, seen = pcio.colorize_by_projection_robust(
        pts, vms, K, [red, green, green], W, H, normalize_exposure=False,
        max_samples=2, prefer_near=True)
    assert seen[0]
    np.testing.assert_array_equal(rgb[0], [0, 200, 0])
    # Without the preference the first two (red + green) survive -> a blend.
    rgb_fifo, _ = pcio.colorize_by_projection_robust(
        pts, vms, K, [red, green, green], W, H, normalize_exposure=False,
        max_samples=2, prefer_near=False)
    assert not np.array_equal(rgb_fifo[0], [0, 200, 0])


def test_colorize_robust_return_counts_reports_confidence():
    vms1, K, W, H = _cam()
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[50, 50] = [10, 20, 30]
    vms = np.concatenate([vms1] * 3, axis=0)
    # One point seen by all three views, one far off-frame (never seen).
    pts = np.array([[0.0, 0.0, 5.0], [50.0, 0.0, 5.0]])
    out = pcio.colorize_by_projection_robust(
        pts, vms, K, [img, img, img], W, H, normalize_exposure=False,
        return_counts=True)
    assert len(out) == 3
    rgb, seen, counts = out
    assert counts[0] == 3 and counts[1] == 0
    assert seen[0] and not seen[1]


def test_colorize_robust_normalizes_mono_images_and_broadcasts_rgb():
    vms1, K, W, H = _cam()
    vms = np.concatenate([vms1, vms1], axis=0)
    dark = np.full((H, W), 50, dtype=np.uint8)
    bright = np.full((H, W), 100, dtype=np.uint8)
    rgb, seen = pcio.colorize_by_projection_robust(
        np.array([[0.0, 0.0, 5.0]]), vms, K, [dark, bright], W, H,
        normalize_exposure=True)
    assert seen[0]
    np.testing.assert_array_equal(rgb[0], [75, 75, 75])


# --------------------------------------------------------------------------- #
# project_depth_maps (LiDAR depth supervision GT)
# --------------------------------------------------------------------------- #
def test_project_depth_maps_centre_pixel_and_depth():
    vms, K, W, H = _cam()
    pts = np.array([[0.0, 0.0, 5.0]])      # projects to (50, 50) at depth 5
    (pix, depth), = pcio.project_depth_maps(pts, vms, K, W, H)
    assert pix.tolist() == [50 * W + 50]
    np.testing.assert_allclose(depth, [5.0], atol=1e-6)


def test_project_depth_maps_zbuffer_keeps_nearest():
    vms, K, W, H = _cam()
    # Two points on the same ray land on one pixel; only the near depth survives.
    pts = np.array([[0.0, 0.0, 8.0], [0.0, 0.0, 2.0]])
    (pix, depth), = pcio.project_depth_maps(pts, vms, K, W, H)
    assert pix.tolist() == [50 * W + 50]
    np.testing.assert_allclose(depth, [2.0], atol=1e-6)


def test_project_depth_maps_culls_behind_and_out_of_frame():
    vms, K, W, H = _cam()
    pts = np.array([[0.0, 0.0, -5.0],      # behind the camera
                    [10.0, 0.0, 5.0]])     # u = 250 -> off image
    (pix, depth), = pcio.project_depth_maps(pts, vms, K, W, H)
    assert pix.size == 0 and depth.size == 0


def test_project_depth_maps_one_entry_per_view():
    vms1, K, W, H = _cam()
    vms = np.concatenate([vms1, vms1], axis=0)
    pts = np.array([[0.0, 0.0, 5.0]])
    maps = pcio.project_depth_maps(pts, vms, K, W, H)
    assert len(maps) == 2
    for pix, depth in maps:
        assert pix.tolist() == [50 * W + 50]
        np.testing.assert_allclose(depth, [5.0], atol=1e-6)


# --------------------------------------------------------------------------- #
# drop_sparse_points
# --------------------------------------------------------------------------- #
def test_drop_sparse_points_keeps_cluster_drops_isolated():
    rng = np.random.default_rng(4)
    cluster = rng.uniform(0.0, 0.05, size=(8, 3))
    isolated = np.array([[5.0, 5.0, 5.0]])
    keep = pcio.drop_sparse_points(np.vstack([cluster, isolated]),
                                   min_neighbors=3, voxel=0.1)
    assert keep[:8].all()
    assert not keep[8]


def test_drop_sparse_points_grid_boundary_is_safe():
    # The max-corner point's +1 neighbour keys fall past the last occupied
    # voxel; searchsorted must not index out of bounds (regression).
    pts = np.array([[0.0, 0.0, 0.0], [9.0, 9.0, 9.0]])
    keep = pcio.drop_sparse_points(pts, min_neighbors=1, voxel=0.1)
    assert keep.tolist() == [True, True]


def test_drop_sparse_points_neighbouring_voxels_count_together():
    # Two points in adjacent voxels see each other through the 26-neighbourhood.
    pts = np.array([[0.0, 0.0, 0.0], [0.11, 0.0, 0.0]])
    keep = pcio.drop_sparse_points(pts, min_neighbors=2, voxel=0.1)
    assert keep.tolist() == [True, True]
