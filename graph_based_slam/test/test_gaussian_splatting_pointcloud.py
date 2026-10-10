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
import pytest

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


def test_read_binary_pcd_xyz_rgb(tmp_path):
    dtype = np.dtype([
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
        ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
    ])
    records = np.array([
        (1.0, 2.0, 3.0, 10, 20, 30),
        (-4.0, 5.0, 6.0, 40, 50, 60),
    ], dtype=dtype)
    header = (
        '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z red green blue\n'
        'SIZE 4 4 4 1 1 1\nTYPE F F F U U U\nCOUNT 1 1 1 1 1 1\n'
        'WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n')
    path = tmp_path / 'binary.pcd'
    path.write_bytes(header.encode('ascii') + records.tobytes())

    xyz, rgb = pcio.read_point_cloud_xyz(path)

    np.testing.assert_allclose(xyz, [[1, 2, 3], [-4, 5, 6]], atol=1e-6)
    np.testing.assert_array_equal(rgb, [[10, 20, 30], [40, 50, 60]])


def test_read_binary_pcd_packed_uint_rgb(tmp_path):
    dtype = np.dtype([
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4'),
    ])
    records = np.array([
        (1.0, 2.0, 3.0, 0x000A141E),
        (-4.0, 5.0, 6.0, 0x0028323C),
    ], dtype=dtype)
    header = (
        '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\n'
        'SIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n'
        'WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n')
    path = tmp_path / 'packed_uint.pcd'
    path.write_bytes(header.encode('ascii') + records.tobytes())

    xyz, rgb = pcio.read_point_cloud_xyz(path)

    np.testing.assert_allclose(xyz, [[1, 2, 3], [-4, 5, 6]], atol=1e-6)
    np.testing.assert_array_equal(rgb, [[10, 20, 30], [40, 50, 60]])


def test_read_ascii_pcd_packed_float_rgb(tmp_path):
    packed = np.array([0x000A141E, 0x0028323C], dtype='<u4').view('<f4')
    path = tmp_path / 'packed_float_ascii.pcd'
    path.write_text(
        '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\n'
        'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        'WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n'
        f'1 2 3 {packed[0]:.9g}\n4 5 6 {packed[1]:.9g}\n')

    xyz, rgb = pcio.read_point_cloud_xyz(path)

    np.testing.assert_allclose(xyz, [[1, 2, 3], [4, 5, 6]], atol=1e-6)
    np.testing.assert_array_equal(rgb, [[10, 20, 30], [40, 50, 60]])


def test_read_ascii_pcd_xyz(tmp_path):
    path = tmp_path / 'ascii.pcd'
    path.write_text(
        '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
        'TYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\n'
        'DATA ascii\n1 2 3\n4 5 6\n')

    xyz, rgb = pcio.read_point_cloud_xyz(path)

    np.testing.assert_allclose(xyz, [[1, 2, 3], [4, 5, 6]], atol=1e-6)
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


def test_project_planar_voxels_flattens_safe_plane_without_deleting_points():
    xy = np.array([(x, y) for x in np.linspace(0.1, 0.9, 4)
                   for y in np.linspace(0.1, 0.9, 4)])
    z = 0.5 + np.linspace(-0.03, 0.03, len(xy))
    points = np.column_stack((xy, z))
    refined, projected = pcio.project_planar_voxels(points, 1.0)
    assert projected.all()
    assert refined.shape == points.shape
    eigenvalues = np.linalg.eigvalsh(np.cov(refined.T, bias=True))
    assert eigenvalues[0] < 1.0e-12


def test_project_planar_voxels_keeps_sparse_groups():
    sparse = np.array([[0.1, 0.1, 0.1], [0.2, 0.2, 0.2]])
    refined, projected = pcio.project_planar_voxels(sparse, 1.0)
    np.testing.assert_array_equal(refined, sparse)
    assert not projected.any()


def test_project_planar_voxels_validates_options():
    with np.testing.assert_raises(ValueError):
        pcio.project_planar_voxels(np.zeros((1, 3)), 0.0)
    with np.testing.assert_raises(ValueError):
        pcio.project_planar_voxels(np.zeros((1, 3)), 1.0, min_points=2)


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


def test_deskew_points_compensates_body_motion_during_scan():
    import posed_images as pi
    samples = [
        pi.TrajectorySample(0.0, np.array([0.0, 0.0, 0.0]),
                            np.array([0.0, 0.0, 0.0, 1.0])),
        pi.TrajectorySample(0.1, np.array([1.0, 0.0, 0.0]),
                            np.array([0.0, 0.0, 0.0, 1.0])),
    ]
    # A static object at world x=10 is observed at LiDAR x=10 then x=9 as the
    # body translates by one metre during the scan.
    points = np.array([[10.0, 0.0, 0.0], [9.0, 0.0, 0.0]])
    out = bli.deskew_points(
        points, np.array([0.0, 0.1]), samples, np.eye(4),
        bin_seconds=0.001, max_extrapolation=0.0)
    np.testing.assert_allclose(out, [[10.0, 0.0, 0.0],
                                     [10.0, 0.0, 0.0]], atol=1e-6)


def test_deskew_points_applies_lidar_extrinsic_and_validates_inputs():
    import posed_images as pi
    samples = [
        pi.TrajectorySample(1.0, np.zeros(3),
                            np.array([0.0, 0.0, 0.0, 1.0])),
        pi.TrajectorySample(2.0, np.zeros(3),
                            np.array([0.0, 0.0, 0.0, 1.0])),
    ]
    extrinsic = np.eye(4)
    extrinsic[:3, 3] = [1.0, 2.0, 3.0]
    out = bli.deskew_points(
        np.array([[0.0, 0.0, 0.0]]), np.array([1.5]), samples, extrinsic)
    np.testing.assert_allclose(out, [[1.0, 2.0, 3.0]], atol=1e-6)
    with np.testing.assert_raises(ValueError):
        bli.deskew_points(np.zeros((2, 3)), np.zeros(1), samples, extrinsic)
    with np.testing.assert_raises(ValueError):
        bli.deskew_points(np.zeros((1, 3)), np.zeros(1), samples, extrinsic,
                          bin_seconds=0.0)


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
def test_observed_color_medoid_never_synthesizes_unseen_rgb():
    samples = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255]]],
                       dtype=np.uint8)
    out = pcio.observed_color_medoids(samples)
    # A channel median would invent black [0, 0, 0]. Tied medoids resolve to
    # the first real observation, deterministically.
    np.testing.assert_array_equal(out, [[255, 0, 0]])


def test_observed_color_medoid_rejects_single_outlier_and_chunks():
    samples = np.array([
        [[10, 20, 30], [11, 21, 31], [250, 0, 200]],
        [[100, 110, 120], [101, 111, 121], [0, 255, 0]],
    ], dtype=np.uint8)
    out = pcio.observed_color_medoids(samples, chunk=1)
    np.testing.assert_array_equal(out, [[11, 21, 31], [100, 110, 120]])


def test_observed_color_medoid_validates_shape_and_chunk():
    with np.testing.assert_raises(ValueError):
        pcio.observed_color_medoids(np.zeros((2, 0, 3), dtype=np.uint8))
    with np.testing.assert_raises(ValueError):
        pcio.observed_color_medoids(np.zeros((2, 1, 3), dtype=np.uint8), chunk=0)


def test_observed_color_medoid_matches_pairwise_distance_oracle():
    rng = np.random.default_rng(987)
    for count in (1, 2, 3, 12, 32, 64):
        for points in (0, 1, 37):
            for upper in (2, 256):
                samples = rng.integers(0, upper, (points, count, 3),
                                       dtype=np.uint8)
                # Independent quadratic oracle covers duplicate values and ties.
                wide = samples.astype(np.int16)
                distances = np.abs(wide[:, :, None] - wide[:, None, :])
                scores = distances.sum(axis=(2, 3), dtype=np.int32)
                expected = samples[np.arange(points), np.argmin(scores, axis=1)]
                for chunk in (1, 11, 20000):
                    np.testing.assert_array_equal(
                        pcio.observed_color_medoids(samples, chunk), expected)


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
        pts, vms, K, [dark, dark, bright], W, H, normalize_exposure=True,
        exposure_scale_limit=10.0)
    assert seen[0]
    # Global median luminance is the dark 60; the bright view is scaled by 1/3.
    assert abs(int(rgb[0][0]) - 60) <= 1


def test_colorize_robust_exposure_normalization_clamps_extreme_gain():
    _, K, W, H = _cam()
    dark = np.full((H, W, 3), 60, dtype=np.uint8)
    bright = np.full((H, W, 3), 180, dtype=np.uint8)
    away = np.eye(4)
    away[:3, 3] = [1000.0, 0.0, 0.0]
    vms = np.stack([away, away, np.eye(4)])
    rgb, seen = pcio.colorize_by_projection_robust(
        np.array([[0.0, 0.0, 5.0]]), vms, K,
        [dark, dark, bright], W, H, normalize_exposure=True,
        exposure_scale_limit=1.5)
    assert seen[0]
    # The requested 1/3 scale is capped at 1/1.5, preserving real brightness.
    np.testing.assert_array_equal(rgb[0], [120, 120, 120])


def test_colorize_robust_exposure_rejects_limit_below_one():
    vms, K, W, H = _cam()
    img = np.zeros((H, W, 3), dtype=np.uint8)
    with np.testing.assert_raises(ValueError):
        pcio.colorize_by_projection_robust(
            np.array([[0.0, 0.0, 5.0]]), vms, K, [img], W, H,
            exposure_scale_limit=0.9)


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


def test_colorize_robust_edge_aware_avoids_boundary_mix():
    vms, K, W, H = _cam()
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[50, 51] = [255, 255, 255]
    pts = np.array([[0.02, 0.0, 5.0]])  # u=50.4, nearest pixel is black at 50
    rgb, seen = pcio.colorize_by_projection_robust(
        pts, vms, K, [img], W, H, normalize_exposure=False,
        interp='edge-aware', edge_threshold=48.0)
    assert seen[0]
    np.testing.assert_array_equal(rgb[0], [0, 0, 0])

    mixed, _ = pcio.colorize_by_projection_robust(
        pts, vms, K, [img], W, H, normalize_exposure=False,
        interp='bilinear')
    np.testing.assert_array_equal(mixed[0], [102, 102, 102])


def test_colorize_robust_edge_aware_keeps_smooth_bilinear_sampling():
    vms, K, W, H = _cam()
    img = np.full((H, W, 3), 100, dtype=np.uint8)
    img[50, 51] = [110, 110, 110]
    pts = np.array([[0.02, 0.0, 5.0]])
    rgb, seen = pcio.colorize_by_projection_robust(
        pts, vms, K, [img], W, H, normalize_exposure=False,
        interp='edge-aware', edge_threshold=48.0)
    assert seen[0]
    np.testing.assert_array_equal(rgb[0], [104, 104, 104])


def test_edge_aware_sampling_matches_pairwise_corner_reference():
    rng = np.random.default_rng(19)
    image = rng.integers(0, 256, (23, 31, 3), dtype=np.uint8)
    u = rng.uniform(-0.25, 30.25, 1000)
    v = rng.uniform(-0.25, 22.25, 1000)
    actual = pcio._sample_pixels(
        image, u, v, 31, 23, 'edge-aware', 48.0)

    source = image.astype(np.float32)
    x0 = np.clip(np.floor(u).astype(np.int64), 0, 30)
    y0 = np.clip(np.floor(v).astype(np.int64), 0, 22)
    x1, y1 = np.minimum(x0 + 1, 30), np.minimum(y0 + 1, 22)
    wx = np.clip(u - x0, 0.0, 1.0)[:, None].astype(np.float32)
    wy = np.clip(v - y0, 0.0, 1.0)[:, None].astype(np.float32)
    top = source[y0, x0] * (1.0 - wx) + source[y0, x1] * wx
    bottom = source[y1, x0] * (1.0 - wx) + source[y1, x1] * wx
    expected = top * (1.0 - wy) + bottom * wy
    corners = np.stack([
        source[y0, x0], source[y0, x1],
        source[y1, x0], source[y1, x1],
    ], axis=1)
    use_nearest = np.ptp(corners, axis=1).max(axis=1) > 48.0
    nearest_u = np.clip(np.round(u).astype(np.int64), 0, 30)
    nearest_v = np.clip(np.round(v).astype(np.int64), 0, 22)
    expected[use_nearest] = source[
        nearest_v[use_nearest], nearest_u[use_nearest]]
    np.testing.assert_array_equal(actual, expected)


def test_sky_like_colors_flags_bright_grey_and_blue_only():
    rgb = np.array([
        [235, 238, 240],  # overcast sky
        [120, 160, 220],  # blue sky
        [60, 90, 40],     # foliage
        [230, 140, 40],   # autumn leaves
        [120, 120, 120],  # shaded concrete
    ], dtype=np.uint8)
    assert pcio.sky_like_colors(rgb).tolist() == [
        True, True, False, False, False]


def test_estimate_world_up_ignores_heading_and_map_tilt():
    tilt = np.array([[1.0, 0.0, 0.0],
                     [0.0, np.cos(0.3), -np.sin(0.3)],
                     [0.0, np.sin(0.3), np.cos(0.3)]])
    viewmats = []
    for yaw in (0.0, 1.0, 2.5):
        c, s = np.cos(yaw), np.sin(yaw)
        # Level OpenCV camera looking along world +x rotated by yaw (z up).
        world_from_camera = np.array([[s, 0.0, c], [-c, 0.0, s],
                                      [0.0, -1.0, 0.0]])
        vm = np.eye(4)
        vm[:3, :3] = (tilt @ world_from_camera).T
        viewmats.append(vm)
    np.testing.assert_allclose(
        pcio.estimate_world_up(viewmats), tilt @ [0.0, 0.0, 1.0], atol=1e-9)


def _sky_views(colours):
    vms1, K, W, H = _cam()
    images = [np.full((H, W, 3), colour, dtype=np.uint8) for colour in colours]
    return np.concatenate([vms1] * len(colours), axis=0), K, W, H, images


def test_colorize_robust_sky_rejection_keeps_branch_colour():
    # A branch above the horizon lands on bright sky in two of three views.
    vms, K, W, H, images = _sky_views(
        [(235, 238, 240), (235, 238, 240), (70, 50, 30)])
    branch = np.array([[0.0, -1.0, 5.0]])
    up = np.array([0.0, -1.0, 0.0])
    plain, _ = pcio.colorize_by_projection_robust(
        branch, vms, K, images, W, H, normalize_exposure=False)
    np.testing.assert_array_equal(plain, [[235, 238, 240]])
    rgb, seen, counts, diagnostics = pcio.colorize_by_projection_robust(
        branch, vms, K, images, W, H, normalize_exposure=False, sky_up=up,
        return_counts=True, return_diagnostics=True)
    # Counts keep every observation so min_samples still sees three.
    assert seen[0] and counts[0] == 3
    assert diagnostics['rejected_sky'] == 2
    np.testing.assert_array_equal(rgb, [[70, 50, 30]])


def test_colorize_robust_sky_rejection_keeps_white_and_low_surfaces():
    vms, K, W, H, images = _sky_views([(235, 238, 240)] * 3)
    up = np.array([0.0, -1.0, 0.0])
    # Only sky-like samples: a white facade keeps its colour.
    rgb, seen, counts = pcio.colorize_by_projection_robust(
        np.array([[0.0, -1.0, 5.0]]), vms, K, images, W, H,
        normalize_exposure=False, sky_up=up, return_counts=True)
    assert seen[0] and counts[0] == 3
    np.testing.assert_array_equal(rgb, [[235, 238, 240]])
    # Below the horizon a bright sample is never sky.
    vms, K, W, H, images = _sky_views(
        [(235, 238, 240), (235, 238, 240), (70, 50, 30)])
    rgb, _ = pcio.colorize_by_projection_robust(
        np.array([[0.0, 1.0, 5.0]]), vms, K, images, W, H,
        normalize_exposure=False, sky_up=up)
    np.testing.assert_array_equal(rgb, [[235, 238, 240]])
    with pytest.raises(ValueError):
        pcio.colorize_by_projection_robust(
            np.array([[0.0, 1.0, 5.0]]), vms, K, images, W, H,
            sky_up=np.zeros(3))


def test_voxel_planar_mask_separates_flat_patches_from_scatter():
    grid = np.array([[0.02 + 0.06 * i, 0.02 + 0.06 * j, 0.1]
                     for i in range(4) for j in range(4)])
    blob = np.random.default_rng(0).uniform(1.0, 1.29, size=(16, 3))
    sparse = np.array([[3.05, 3.05, 3.05], [3.1, 3.1, 3.05]])
    mask = pcio.voxel_planar_mask(np.vstack([grid, blob, sparse]), 0.3)
    assert mask[:16].all() and not mask[16:].any()


def test_fill_colors_from_neighbours_uses_median_within_radius():
    pytest.importorskip('scipy.spatial')
    pts = np.array([[0.0, 0, 0], [0.1, 0, 0], [0.2, 0, 0], [0.3, 0, 0],
                    [9.0, 0, 0]])
    rgb = np.array([[255, 255, 255], [10, 10, 10], [20, 20, 20],
                    [30, 30, 30], [255, 255, 255]], dtype=np.uint8)
    targets = np.array([True, False, False, False, True])
    donors = ~targets
    out, filled = pcio.fill_colors_from_neighbours(pts, rgb, targets, donors, 1.0)
    assert filled == 1
    np.testing.assert_array_equal(out[0], [20, 20, 20])
    np.testing.assert_array_equal(out[4], [255, 255, 255])
    np.testing.assert_array_equal(rgb[0], [255, 255, 255])


def test_colorize_robust_sky_fill_recolours_branches_not_facades():
    pytest.importorskip('scipy.spatial')
    vms1, K, W, H = _cam()
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:50] = (235, 238, 240)   # sky above the horizon row
    img[50:] = (40, 90, 30)      # foliage below it
    images = [img] * 3
    vms = np.concatenate([vms1] * 3, axis=0)
    up = np.array([0.0, -1.0, 0.0])
    branch = [[0.0, -1.0, 5.0]]
    foliage = [[0.0, 0.5, 5.0]]
    facade = [[1.53 + 0.03 * i, -1.17 + 0.06 * j, 6.05]
              for i in range(4) for j in range(4)]
    pts = np.array(branch + foliage + facade)
    plain, _ = pcio.colorize_by_projection_robust(
        pts, vms, K, images, W, H, normalize_exposure=False, sky_up=up)
    np.testing.assert_array_equal(plain[0], [235, 238, 240])
    rgb, seen, counts, diagnostics = pcio.colorize_by_projection_robust(
        pts, vms, K, images, W, H, normalize_exposure=False, sky_up=up,
        sky_fill_radius_m=2.0, return_counts=True, return_diagnostics=True)
    np.testing.assert_array_equal(rgb[0], [40, 90, 30])
    np.testing.assert_array_equal(rgb[2:], np.tile([235, 238, 240], (16, 1)))
    assert diagnostics['sky_filled'] == 1
    assert seen.all() and counts[0] == 3
    with pytest.raises(ValueError):
        pcio.colorize_by_projection_robust(
            pts, vms, K, images, W, H, sky_fill_radius_m=2.0)


def test_colorize_robust_edge_aware_validates_threshold():
    vms, K, W, H = _cam()
    img = np.zeros((H, W, 3), dtype=np.uint8)
    with np.testing.assert_raises(ValueError):
        pcio.colorize_by_projection_robust(
            np.array([[0.0, 0.0, 5.0]]), vms, K, [img], W, H,
            normalize_exposure=False, edge_threshold=-1.0)


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


def test_colorize_robust_observation_mask_rejects_bad_view():
    vms1, K, W, H = _cam()
    vms = np.concatenate([vms1] * 3, axis=0)
    images = [np.full((H, W, 3), value, np.uint8)
              for value in (50, 50, 240)]
    points = np.array([[0.0, 0.0, 5.0]])
    mask = np.array([[True, True, False]])
    rgb, seen, counts = pcio.colorize_by_projection_robust(
        points, vms, K, images, W, H, normalize_exposure=False,
        observation_mask=mask, return_counts=True)
    assert seen[0] and counts[0] == 2
    np.testing.assert_array_equal(rgb[0], [50, 50, 50])


def test_geometry_occlusion_margin_rejects_adjacent_background():
    vms, K, width, height = _cam()
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[50, 50], image[50, 51] = [200, 0, 0], [0, 200, 0]
    points = np.array([[0.0, 0.0, 5.0], [0.1, 0.0, 10.0]])
    rgb, seen, diagnostics = pcio.colorize_by_projection_robust(
        points, vms, K, [image], width, height,
        normalize_exposure=False, interp='nearest',
        occlusion_margin_px=1, return_diagnostics=True)
    assert seen.tolist() == [True, False]
    np.testing.assert_array_equal(rgb[0], [200, 0, 0])
    assert diagnostics['rejected_occlusion'] == 1
    assert diagnostics['rejected_zbuffer'] == 0
    assert diagnostics['rejected_occlusion_margin'] == 1


def test_geometry_depth_edge_rejects_both_sides_but_keeps_flat_surface():
    vms, K, width, height = _cam()
    image = np.full((height, width, 3), 100, dtype=np.uint8)
    discontinuity = np.array([[0.0, 0.0, 5.0], [0.1, 0.0, 10.0]])
    _, seen, diagnostics = pcio.colorize_by_projection_robust(
        discontinuity, vms, K, [image], width, height,
        normalize_exposure=False, depth_edge_margin_px=1,
        depth_edge_tolerance=0.2, return_diagnostics=True)
    assert seen.tolist() == [False, False]
    assert diagnostics['rejected_depth_edge'] == 2

    flat = np.array([[0.0, 0.0, 5.0], [0.05, 0.0, 5.0]])
    _, flat_seen = pcio.colorize_by_projection_robust(
        flat, vms, K, [image], width, height,
        normalize_exposure=False, depth_edge_margin_px=1,
        depth_edge_tolerance=0.2)
    assert flat_seen.all()


def test_geometry_dynamic_mask_and_margin_reject_image_regions():
    vms, K, width, height = _cam()
    image = np.full((height, width, 3), 100, dtype=np.uint8)
    mask = np.zeros((height, width), dtype=bool)
    mask[50, 50] = True
    points = np.array([[0.0, 0.0, 5.0], [0.05, 0.0, 5.0]])
    _, seen, diagnostics = pcio.colorize_by_projection_robust(
        points, vms, K, [image], width, height,
        normalize_exposure=False, exclusion_masks=[mask],
        dynamic_mask_margin_px=1, return_diagnostics=True)
    assert seen.tolist() == [False, False]
    assert diagnostics['rejected_dynamic_mask'] == 2


def test_calibration_uncertainty_expands_geometry_margin():
    vms, K, width, height = _cam()
    image = np.full((height, width, 3), 100, dtype=np.uint8)
    points = np.array([[0.0, 0.0, 5.0], [0.1, 0.0, 10.0]])
    calibration = {
        'accepted': True,
        'uncertainty_dt_s_xyz_m_rpy_rad':
            [0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
    _, seen = pcio.colorize_by_projection_robust(
        points, vms, K, [image], width, height,
        normalize_exposure=False, calibration=calibration,
        view_timestamps=[0.0], calibration_sigma_multiplier=1.0,
        maximum_uncertainty_margin_px=2, depth_edge_tolerance=100.0)
    assert seen.tolist() == [True, False]


def test_geometry_fusion_rejects_coarse_zbuffer_and_missing_timestamps():
    vms, K, width, height = _cam()
    image = np.zeros((height, width, 3), dtype=np.uint8)
    point = np.array([[0.0, 0.0, 5.0]])
    with np.testing.assert_raises(ValueError):
        pcio.colorize_by_projection_robust(
            point, vms, K, [image], width, height,
            occlusion_margin_px=1, zbuf_bin=2)
    with np.testing.assert_raises(ValueError):
        pcio.colorize_by_projection_robust(
            point, vms, K, [image], width, height,
            calibration={'accepted': True,
                         'uncertainty_dt_s_xyz_m_rpy_rad': [0.0] * 7},
            calibration_sigma_multiplier=1.0)


def test_builder_loads_manifest_dynamic_masks_for_geometry_fusion(tmp_path):
    import imageio as iio
    import json

    images = tmp_path / 'images'
    masks = tmp_path / 'masks'
    images.mkdir()
    masks.mkdir()
    image = np.full((100, 100, 3), 120, dtype=np.uint8)
    mask = np.zeros((100, 100), dtype=np.uint8)
    mask[50, 50] = 255
    iio.imwrite(images / '0.png', image)
    iio.imwrite(masks / '0.png', mask)
    document = {
        'w': 100, 'h': 100, 'fl_x': 100.0, 'fl_y': 100.0,
        'cx': 50.0, 'cy': 50.0,
        'frames': [{
            'file_path': 'images/0.png', 'dynamic_mask_path': 'masks/0.png',
            'timestamp': 0.0,
            'transform_matrix': np.diag([1.0, -1.0, -1.0, 1.0]).tolist(),
        }],
    }
    transforms = tmp_path / 'transforms.json'
    transforms.write_text(json.dumps(document))
    rgb, seen, diagnostics = bli._colorize(
        np.array([[0.0, 0.0, 5.0]]), str(transforms), robust=True,
        normalize_exposure=False, geometry_aware=True,
        occlusion_margin_px=0, depth_edge_margin_px=0,
        dynamic_exclusion=True, dynamic_mask_margin_px=0,
        return_diagnostics=True)
    assert not seen[0]
    np.testing.assert_array_equal(rgb[0], [128, 128, 128])
    assert diagnostics['rejected_dynamic_mask'] == 1


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


def test_drop_sparse_points_one_counts_the_query_point():
    pts = np.array([[0.0, 0.0, 0.0], [9.0, -4.0, 2.0]])
    keep = pcio.drop_sparse_points(pts, min_neighbors=1, voxel=0.1)
    assert keep.all()


def test_drop_sparse_points_neighbouring_voxels_count_together():
    # Two points in adjacent voxels see each other through the 26-neighbourhood.
    pts = np.array([[0.0, 0.0, 0.0], [0.11, 0.0, 0.0]])
    keep = pcio.drop_sparse_points(pts, min_neighbors=2, voxel=0.1)
    assert keep.tolist() == [True, True]


def test_colorize_robust_image_margin_skips_border_samples():
    vms, K, W, H = _cam()
    img = np.full((H, W, 3), 200, dtype=np.uint8)
    # Two points: one lands at the centre, one lands 4 px from the border.
    pts = np.array([[0.0, 0.0, 5.0], [2.3, 0.0, 5.0]])  # u = 50 and u = 96
    rgb, seen = pcio.colorize_by_projection_robust(
        pts, vms, K, [img], W, H, normalize_exposure=False, image_margin=10)
    assert seen[0] and not seen[1]
    np.testing.assert_array_equal(rgb[0], [200, 200, 200])
    # Margin 0 (default) keeps the border point colourable.
    _, seen_full = pcio.colorize_by_projection_robust(
        pts, vms, K, [img], W, H, normalize_exposure=False)
    assert seen_full.all()


def test_colorize_robust_image_margin_keeps_full_frame_occlusion():
    vms, K, W, H = _cam()
    img = np.full((H, W, 3), 200, dtype=np.uint8)
    # A near point inside the margin still occludes the far point behind it
    # even though the near point itself is never sampled for colour.
    near = [2.3, 0.0, 5.0]    # u = 96, inside the 10 px margin band
    far = [4.6, 0.0, 10.0]    # same pixel, twice the depth
    rgb, seen = pcio.colorize_by_projection_robust(
        np.array([near, far]), vms, K, [img], W, H,
        normalize_exposure=False, image_margin=10)
    assert not seen.any()


def test_colorize_robust_image_margin_validation():
    vms, K, W, H = _cam()
    img = np.zeros((H, W, 3), dtype=np.uint8)
    for margin in (-1, 50, 60):
        with np.testing.assert_raises(ValueError):
            pcio.colorize_by_projection_robust(
                np.zeros((1, 3)), vms, K, [img], W, H, image_margin=margin)


def test_radial_vignette_gain_recovers_dark_border_and_is_default_off():
    vms, K, W, H = _cam()
    yy, xx = np.mgrid[:H, :W]
    radius = np.hypot(xx - 50.0, yy - 50.0) / np.hypot(50.0, 50.0)
    image = np.clip(120.0 * (1.0 - 0.5 * radius ** 2), 0, 255)
    image = np.repeat(image[:, :, None], 3, axis=2).astype(np.uint8)
    points = np.array([[0.0, 0.0, 5.0], [2.25, 0.0, 5.0]])
    baseline, _ = pcio.colorize_by_projection_robust(
        points, vms, K, [image], W, H, normalize_exposure=False)
    disabled, _ = pcio.colorize_by_projection_robust(
        points, vms, K, [image], W, H, normalize_exposure=False,
        vignette_gain_limit=1.0)
    corrected, _ = pcio.colorize_by_projection_robust(
        points, vms, K, [image], W, H, normalize_exposure=False,
        vignette_gain_limit=2.5)
    np.testing.assert_array_equal(disabled, baseline)
    assert corrected[1, 0] > baseline[1, 0] + 15
    assert abs(int(corrected[1, 0]) - int(corrected[0, 0])) < 10


def test_radial_vignette_gain_validation():
    vms, K, W, H = _cam()
    with np.testing.assert_raises(ValueError):
        pcio.colorize_by_projection_robust(
            np.zeros((1, 3)), vms, K, [np.zeros((H, W, 3))], W, H,
            vignette_gain_limit=0.9)


def test_estimate_voxel_normals_finds_planar_axis_and_marks_sparse():
    yy, xx = np.mgrid[:4, :4]
    plane = np.column_stack([xx.ravel(), yy.ravel(), np.zeros(16)]) * 0.01
    sparse = np.array([[2.0, 2.0, 2.0]])
    normals = pcio.estimate_voxel_normals(
        np.vstack([plane, sparse]), voxel=0.1, min_points=6)
    assert np.all(np.abs(normals[:16, 2]) > 0.99)
    np.testing.assert_array_equal(normals[-1], [0.0, 0.0, 0.0])


def test_estimate_overlap_rgb_gains_matches_shared_scene_colours():
    _, K, W, H = _cam()
    xx, yy = np.meshgrid(np.linspace(-0.5, 0.5, 5),
                         np.linspace(-0.5, 0.5, 5))
    points = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, 5.0)])
    images = [
        np.full((H, W, 3), [50, 80, 120], dtype=np.uint8),
        np.full((H, W, 3), [100, 80, 60], dtype=np.uint8),
    ]
    gains = pcio.estimate_overlap_rgb_gains(
        points, np.stack([np.eye(4), np.eye(4)]), K, images, W, H,
        min_shared=16, neighbour_span=1, gain_limit=2.0,
        regularization=0.0)
    corrected0 = np.array([50, 80, 120]) * gains[0]
    corrected1 = np.array([100, 80, 60]) * gains[1]
    np.testing.assert_allclose(corrected0, corrected1, rtol=0.02)


def test_view_confidence_rejects_grazing_observation():
    _, K, W, H = _cam()
    red = np.full((H, W, 3), [200, 0, 0], dtype=np.uint8)
    green = np.full((H, W, 3), [0, 200, 0], dtype=np.uint8)
    centred = np.eye(4)
    side = np.eye(4)
    side[0, 3] = -1.0  # camera centre at world x=+1
    rgb, seen = pcio.colorize_by_projection_robust(
        np.array([[0.0, 0.0, 5.0]]), np.stack([centred, side]), K,
        [red, green], W, H, normalize_exposure=False, max_samples=1,
        point_normals=np.array([[1.0, 0.0, 0.0]]),
        min_view_cosine=0.1, view_score_power=1.0)
    assert seen[0]
    np.testing.assert_array_equal(rgb[0], [0, 200, 0])


def test_dynamic_map_cleaner_is_default_off_and_byte_compatible():
    points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    cleaned, report = bli.clean_dynamic_map(points, [], algorithm='none')
    assert cleaned is points
    assert report['enabled'] is False
    assert report['removed_points'] == 0


def test_dynamic_map_cleaner_forwards_fusion_evidence_and_reports_removal():
    class FakeCleaner:
        __version__ = 'test'
        received = None

        @classmethod
        def clean_map_by_fusion(cls, points, scans, **kwargs):
            cls.received = (scans, kwargs)
            keep = np.array([True, False, True])
            return points[keep], keep

    points = np.arange(9, dtype=np.float64).reshape(3, 3)
    scan = (points[:2], np.array([4.0, 5.0, 6.0]))
    cleaned, report = bli.clean_dynamic_map(
        points, [scan], algorithm='fusion', workers=3,
        evidence_stride=2,
        free_votes_fraction=0.7, free_votes_floor=4,
        void_min_scans=5, cleaner_module=FakeCleaner)
    np.testing.assert_array_equal(cleaned, points[[0, 2]])
    assert FakeCleaner.received[1] == {
        'workers': 3, 'free_votes_fraction': 0.7,
        'free_votes_floor': 4, 'void_min_scans': 5}
    assert report['implementation_version'] == 'test'
    assert report['scans'] == 1
    assert report['evidence_stride'] == 2
    assert report['removed_points'] == 1
    assert report['removed_ratio'] == 1 / 3


def test_builder_training_subset_excludes_heldout_images_and_masks(tmp_path, monkeypatch):
    import imageio as iio
    import json

    image = np.full((100, 100, 3), 73, dtype=np.uint8)
    mask = np.zeros((100, 100), dtype=np.uint8)
    iio.imwrite(tmp_path / 'train.png', image)
    iio.imwrite(tmp_path / 'train_mask.png', mask)
    pose = np.diag([1.0, -1.0, -1.0, 1.0]).tolist()
    frames = [
        {'file_path': 'missing_heldout.png', 'timestamp': 10.0,
         'dynamic_mask_path': 'missing_mask.png', 'transform_matrix': pose},
        {'file_path': 'train.png', 'timestamp': 20.0,
         'dynamic_mask_path': 'train_mask.png', 'transform_matrix': pose},
    ]
    document = {'w': 100, 'h': 100, 'fl_x': 100.0, 'fl_y': 100.0,
                'cx': 50.0, 'cy': 50.0, 'frames': frames}
    transforms = tmp_path / 'transforms.json'
    transforms.write_text(json.dumps(document))
    points = np.array([[0.0, 0.0, 5.0]])
    rgb, seen = bli._colorize(
        points, str(transforms), robust=True, frame_indices=[1],
        normalize_exposure=False, geometry_aware=True, dynamic_exclusion=True)
    assert seen.tolist() == [True]
    np.testing.assert_allclose(rgb[0], [73, 73, 73], atol=1e-5)
    with monkeypatch.context() as patch:
        def unexpected_read(path):
            raise AssertionError(f'image decoded again: {path}')
        patch.setattr(iio, 'imread', unexpected_read)
        loaded = [np.full_like(image, 255), image]
        reused_rgb, reused_seen = bli._colorize(
            points, str(transforms), robust=True, frame_indices=[1],
            normalize_exposure=False, loaded_images=loaded)
        np.testing.assert_array_equal(reused_rgb, rgb)
        np.testing.assert_array_equal(reused_seen, seen)
        _, limited_seen = bli._colorize(
            points, str(transforms), robust=True, frame_indices=[1],
            normalize_exposure=False, min_samples=2, loaded_images=loaded)
        assert limited_seen.tolist() == [False]
        with np.testing.assert_raises(ValueError):
            bli._colorize(points, str(transforms), loaded_images=[image])
    for indices in ([], [1, 1], [-1], [2], [0.5]):
        with np.testing.assert_raises(ValueError):
            bli._colorize(points, str(transforms), frame_indices=indices)


@pytest.mark.parametrize('interp', ['bilinear', 'edge-aware'])
@pytest.mark.parametrize('offset', [(0.25, 0.0), (0.0, 0.25), (0.25, 0.25)])
def test_dynamic_mask_excludes_interpolation_support(interp, offset):
    image = np.full((4, 4, 3), 100, dtype=np.uint8)
    mask = np.zeros((4, 4), dtype=bool)
    x, y = (2 if offset[0] else 1), (2 if offset[1] else 1)
    mask[y, x] = True
    image[y, x] = 140  # Below edge-aware threshold, but still excluded.
    points = np.array([[1 + offset[0], 1 + offset[1], 1.]])
    _, seen, counts, diagnostics = pcio.colorize_by_projection_robust(
        points, [np.eye(4)], np.eye(3), [image], 4, 4,
        interp=interp, normalize_exposure=False, exclusion_masks=[mask],
        return_counts=True, return_diagnostics=True)
    assert not seen[0] and counts[0] == 0
    assert diagnostics['rejected_dynamic_mask'] == 1
    assert diagnostics['accepted_samples'] == 0


@pytest.mark.parametrize('interp', ['nearest', 'bilinear', 'edge-aware'])
def test_dynamic_mask_preserves_unused_neighbours_and_clamped_borders(interp):
    image = np.full((4, 4, 3), 100, dtype=np.uint8)
    mask = np.zeros((4, 4), dtype=bool)
    mask[1, 2] = True
    image[1, 2] = 140
    points = np.array([[1., 1., 1.], [-0.25, 0., 1.], [3.25, 3., 1.]])
    rgb, seen = pcio.colorize_by_projection_robust(
        points, [np.eye(4)], np.eye(3), [image], 4, 4,
        interp=interp, normalize_exposure=False, exclusion_masks=[mask])
    assert seen.all()
    assert np.all(rgb == 100)
    if interp == 'nearest':
        rgb, seen = pcio.colorize_by_projection_robust(
            np.array([[1.25, 1., 1.]]), [np.eye(4)], np.eye(3), [image], 4, 4,
            interp=interp, normalize_exposure=False, exclusion_masks=[mask])
        assert seen[0] and np.all(rgb == 100)


@pytest.mark.parametrize('value', [20, 100, 200])
def test_fully_masked_image_cannot_change_other_exposure(value):
    clean = np.full((4, 4, 3), 100, dtype=np.uint8)
    excluded = np.full_like(clean, value)
    rgb, seen, counts = pcio.colorize_by_projection_robust(
        np.array([[1., 1., 1.]]), [np.eye(4)] * 2, np.eye(3),
        [clean, excluded], 4, 4,
        exclusion_masks=[None, np.ones((4, 4), dtype=bool)], return_counts=True)
    assert seen[0] and counts[0] == 1
    np.testing.assert_array_equal(rgb, [[100, 100, 100]])


@pytest.mark.parametrize('channels', [None, 1, 3, 4])
def test_masked_luminance_uses_only_static_pixels(channels):
    shape = (4, 4) if channels is None else (4, 4, channels)
    image = np.full(shape, 200, dtype=np.uint8)
    image[0, 0] = 100
    mask = np.ones((4, 4), dtype=bool)
    mask[0, 0] = False
    assert pcio._median_luminance(image, mask) == pytest.approx(100)
    assert pcio._median_luminance(image, np.ones_like(mask)) == 0


@pytest.mark.parametrize('regularization', [0., 256.])
def test_overlap_gain_ignores_masked_correspondences_and_prior(regularization):
    x, y = np.meshgrid(np.arange(1, 7), np.arange(1, 7))
    points = np.column_stack([x.ravel(), y.ravel(), np.ones(x.size)])
    mask = np.zeros((8, 8), dtype=bool)
    mask[:, 4:] = True
    gains = []
    for value in [20, 200]:
        clean = np.full((8, 8, 3), 100, dtype=np.uint8)
        changed = clean.copy()
        changed[mask] = value
        gains.append(pcio.estimate_overlap_rgb_gains(
            points, [np.eye(4)] * 2, np.eye(3), [clean, changed], 8, 8,
            exclusion_masks=[mask, mask], min_shared=8,
            regularization=regularization))
    np.testing.assert_array_equal(gains[0], gains[1])
    np.testing.assert_allclose(gains[0], 1.)


def test_radial_gain_ignores_masked_pixels_and_unsupported_centre():
    y, x = np.mgrid[:64, :64]
    mask = ((x - 32)**2 + (y - 32)**2) > 22**2
    curves = []
    for value in [20, 200]:
        image = np.full((64, 64, 3), 100, dtype=np.uint8)
        image[mask] = value
        curves.append(pcio.estimate_radial_vignette_gains(
            [image], 64, 64, sample_stride=1, exclusion_masks=[mask]))
    np.testing.assert_array_equal(curves[0], curves[1])
    np.testing.assert_array_equal(curves[0], np.ones(32))
    empty = pcio.estimate_radial_vignette_gains(
        [image], 64, 64, exclusion_masks=[np.ones_like(mask)])
    np.testing.assert_array_equal(empty, np.ones(32))


@pytest.mark.parametrize('size', [(80, 100), (100, 80), (120, 100), (100, 120)])
@pytest.mark.parametrize('channels', [None, 1, 3, 4])
@pytest.mark.parametrize('mode', ['average', 'nearest', 'bilinear', 'edge-aware', 'overlap'])
def test_colour_sampling_rejects_image_calibration_size_mismatch(size, channels, mode):
    vms, K, width, height = _cam()
    shape = size if channels is None else (*size, channels)
    image = np.full(shape, 100, dtype=np.uint8)
    # This pixel is inside both sizes, so a stale calibration used to silently
    # take a valid pixel from the wrong image coordinate system.
    points = np.array([[0., 0., 5.]])
    with pytest.raises(ValueError, match='image dimensions.*calibration'):
        if mode == 'average':
            pcio.colorize_by_projection(points, vms, K, [image], width, height)
        elif mode == 'overlap':
            pcio.estimate_overlap_rgb_gains(points, vms, K, [image], width, height)
        else:
            pcio.colorize_by_projection_robust(
                points, vms, K, [image], width, height, interp=mode,
                normalize_exposure=False)
