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
    xyz = np.array([[0.0, 0.0, 0.0], [5.0, 5.0, 5.0]])
    rgb = np.array([[10, 20, 30], [40, 50, 60]], dtype=np.uint8)
    out, out_rgb = pcio.voxel_downsample(xyz, 0.1, rgb)
    assert out.shape[0] == 2 and out_rgb.shape[0] == 2


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
