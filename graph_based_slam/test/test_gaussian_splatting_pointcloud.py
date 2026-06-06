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
