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

"""Tests for the side-by-side SLAM/3DGS renderer's pure helpers (CPU only)."""

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
    import render_slam_3dgs_sidebyside

    return render_slam_3dgs_sidebyside


sbs = _load()


# --------------------------------------------------------------------------- #
# height_colormap
# --------------------------------------------------------------------------- #
def test_height_colormap_shape_and_range():
    rgb = sbs.height_colormap(np.linspace(-2.0, 5.0, 100))
    assert rgb.shape == (100, 3)
    assert rgb.min() >= 0.0 and rgb.max() <= 1.0


def test_height_colormap_clamps_to_ramp_ends():
    rgb = sbs.height_colormap(np.array([-10.0, 0.5, 10.0]), lo=0.0, hi=1.0)
    np.testing.assert_allclose(rgb[0], sbs.HEIGHT_RAMP[0])
    np.testing.assert_allclose(rgb[2], sbs.HEIGHT_RAMP[-1])


def test_height_colormap_degenerate_range_is_finite():
    rgb = sbs.height_colormap(np.full(4, 2.0), lo=2.0, hi=2.0)
    assert np.isfinite(rgb).all()


# --------------------------------------------------------------------------- #
# resample_polyline
# --------------------------------------------------------------------------- #
def test_resample_polyline_even_spacing_keeps_endpoints():
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    out = sbs.resample_polyline(pts, 0.1)
    assert out.shape == (11, 3)
    np.testing.assert_allclose(out[0], pts[0])
    np.testing.assert_allclose(out[-1], pts[-1])
    steps = np.linalg.norm(np.diff(out, axis=0), axis=1)
    np.testing.assert_allclose(steps, 0.1, atol=1e-9)


def test_resample_polyline_single_point_passthrough():
    pts = np.array([[1.0, 2.0, 3.0]])
    np.testing.assert_allclose(sbs.resample_polyline(pts, 0.1), pts)


def test_resample_polyline_rejects_bad_spacing():
    with pytest.raises(ValueError):
        sbs.resample_polyline(np.arange(9.0).reshape(3, 3), 0.0)


# --------------------------------------------------------------------------- #
# points_to_gaussians / merge_gaussians / mask_far_from
# --------------------------------------------------------------------------- #
def test_points_to_gaussians_layout():
    xyz = np.arange(12.0).reshape(4, 3)
    rgb = np.full((4, 3), 0.5)
    g = sbs.points_to_gaussians(xyz, rgb, 0.01, opacity=0.9)
    assert g['means'].shape == (4, 3)
    np.testing.assert_allclose(g['scales_log'], np.log(0.01))
    np.testing.assert_allclose(g['quats'][:, 0], 1.0)  # wxyz identity
    np.testing.assert_allclose(g['quats'][:, 1:], 0.0)
    sig = 1.0 / (1.0 + np.exp(-g['opacities_logit']))
    np.testing.assert_allclose(sig, 0.9, atol=1e-6)
    assert g['sh_rest'] is None


def test_points_to_gaussians_validates_inputs():
    xyz = np.zeros((2, 3))
    rgb = np.zeros((2, 3))
    with pytest.raises(ValueError):
        sbs.points_to_gaussians(xyz, rgb, 0.0)
    with pytest.raises(ValueError):
        sbs.points_to_gaussians(xyz, rgb, 0.01, opacity=1.0)


def test_merge_gaussians_concatenates():
    a = sbs.points_to_gaussians(np.zeros((2, 3)), np.zeros((2, 3)), 0.01)
    b = sbs.points_to_gaussians(np.ones((3, 3)), np.ones((3, 3)), 0.02)
    m = sbs.merge_gaussians(a, b)
    assert m['means'].shape == (5, 3)
    assert m['opacities_logit'].shape == (5,)
    assert m['sh_rest'] is None


def test_merge_gaussians_rejects_sh_rest():
    a = sbs.points_to_gaussians(np.zeros((2, 3)), np.zeros((2, 3)), 0.01)
    b = dict(a, sh_rest=np.zeros((2, 3, 3)))
    with pytest.raises(ValueError):
        sbs.merge_gaussians(a, b)


def test_mask_far_from():
    pts = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 0.5, 0.0]])
    keep = sbs.mask_far_from(pts, np.zeros(3), 1.0)
    assert keep.tolist() == [False, True, False]


# --------------------------------------------------------------------------- #
# hstack_panes
# --------------------------------------------------------------------------- #
def test_hstack_panes_inserts_divider():
    left = np.zeros((2, 4, 6, 3), dtype=np.uint8)
    right = np.full((2, 4, 5, 3), 9, dtype=np.uint8)
    out = sbs.hstack_panes(left, right, divider=2, divider_rgb=(255, 0, 0))
    assert out.shape == (2, 4, 13, 3)
    np.testing.assert_array_equal(out[:, :, 6:8, 0], 255)
    np.testing.assert_array_equal(out[:, :, 6:8, 1], 0)
    np.testing.assert_array_equal(out[:, :, 8:, :], 9)


def test_hstack_panes_no_divider_and_mismatch():
    left = np.zeros((2, 4, 6, 3), dtype=np.uint8)
    right = np.zeros((2, 4, 5, 3), dtype=np.uint8)
    assert sbs.hstack_panes(left, right, divider=0).shape == (2, 4, 11, 3)
    with pytest.raises(ValueError):
        sbs.hstack_panes(left, np.zeros((2, 5, 5, 3), dtype=np.uint8))


# --------------------------------------------------------------------------- #
# CLI defaults
# --------------------------------------------------------------------------- #
def test_parser_defaults():
    args = sbs.build_parser().parse_args([
        '--ply', 'a.ply', '--pointcloud', 'b.ply', '--transforms', 'c.json'])
    assert args.point_size == pytest.approx(0.008)
    assert args.traj_radius == pytest.approx(0.04)
    assert args.traj_cull_radius == pytest.approx(1.0)
    assert args.traj_color == '255,40,220'
    assert args.traj_z_offset == pytest.approx(-0.4)
    assert args.label_left == 'LiDAR SLAM map + trajectory'
    assert args.label_right == '3DGS render'
