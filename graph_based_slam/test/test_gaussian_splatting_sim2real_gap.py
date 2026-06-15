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

"""Tests for the sim2real gap harness pure helpers (CPU, no torch/gsplat)."""

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
    import sim2real_gap

    return sim2real_gap


s2r = _load()


def _viewmat(center, rot=None):
    rot = np.eye(3) if rot is None else np.asarray(rot, dtype=float)
    vm = np.eye(4)
    vm[:3, :3] = rot
    vm[:3, 3] = -rot @ np.asarray(center, dtype=float)
    return vm


def _center_of(vm):
    return -vm[:3, :3].T @ vm[:3, 3]


def test_offset_viewmat_moves_camera_along_local_axis():
    vm = _viewmat([1.0, 2.0, 3.0])  # identity rotation: local == world axes
    moved = s2r.offset_viewmat(vm, 0.5, 'x')
    assert np.allclose(_center_of(moved), [1.5, 2.0, 3.0])
    assert np.allclose(moved[:3, :3], vm[:3, :3])  # rotation untouched


def test_offset_viewmat_respects_camera_rotation():
    # 90 deg yaw about world Z: camera local +x points along world -y here.
    rot = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    vm = _viewmat([0.0, 0.0, 0.0], rot)
    moved = s2r.offset_viewmat(vm, 1.0, 'x')
    center = _center_of(moved)
    assert np.isclose(np.linalg.norm(center), 1.0)
    assert not np.isclose(center[0], 1.0)  # not a naive world-x shift


def test_offset_viewmat_zero_is_identity():
    vm = _viewmat([0.3, -0.7, 2.0])
    assert np.allclose(s2r.offset_viewmat(vm, 0.0, 'y'), vm)


def test_offset_viewmat_rejects_bad_axis():
    with pytest.raises(ValueError):
        s2r.offset_viewmat(np.eye(4), 1.0, 'w')


def test_sweep_viewmats_shape_and_values():
    vms = [_viewmat([0.0, 0.0, 0.0]), _viewmat([1.0, 1.0, 1.0])]
    out = s2r.sweep_viewmats(vms, 0.25, 'x')
    assert out.shape == (2, 4, 4)
    assert np.allclose(_center_of(out[0]), [0.25, 0.0, 0.0])
    assert np.allclose(_center_of(out[1]), [1.25, 1.0, 1.0])


def test_psnr_identical_is_inf():
    a = np.full((8, 8, 3), 100, dtype=np.uint8)
    assert s2r.psnr(a, a) == float('inf')


def test_psnr_known_value():
    a = np.zeros((4, 4, 3), dtype=np.uint8)
    b = np.full((4, 4, 3), 1, dtype=np.uint8)  # mse = 1 -> 20*log10(255)
    assert s2r.psnr(a, b) == pytest.approx(20.0 * np.log10(255.0), rel=1e-6)


def test_ssim_identical_is_one():
    rng = np.random.default_rng(0)
    a = rng.integers(0, 256, (32, 32, 3), dtype=np.uint8)
    assert s2r.ssim(a, a) == pytest.approx(1.0, abs=1e-9)


def test_ssim_drops_for_different_images():
    rng = np.random.default_rng(1)
    a = rng.integers(0, 256, (32, 32, 3), dtype=np.uint8)
    b = rng.integers(0, 256, (32, 32, 3), dtype=np.uint8)
    assert s2r.ssim(a, b) < 0.3


def test_box_mean_constant_image():
    img = np.full((10, 10), 5.0)
    assert np.allclose(s2r._box_mean(img, 5), 5.0)


def test_floater_fraction_counts_bright_pixels():
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    img[:5] = 255  # half the pixels white
    assert s2r.floater_fraction(img) == pytest.approx(0.5)


def test_sharpness_flat_is_zero():
    assert s2r.sharpness(np.full((8, 8, 3), 128, dtype=np.uint8)) == 0.0


def test_box_iou_overlap_and_disjoint():
    assert s2r.box_iou([0, 0, 2, 2], [0, 0, 2, 2]) == pytest.approx(1.0)
    assert s2r.box_iou([0, 0, 2, 2], [10, 10, 12, 12]) == 0.0
    # half overlap: inter 2, union 6
    assert s2r.box_iou([0, 0, 2, 2], [1, 0, 3, 2]) == pytest.approx(2.0 / 6.0)


def test_match_detections_greedy_one_to_one():
    ref = [{'cls': 0, 'box': [0, 0, 2, 2], 'conf': 0.9},
           {'cls': 0, 'box': [10, 10, 12, 12], 'conf': 0.8}]
    qry = [{'cls': 0, 'box': [0, 0, 2, 2], 'conf': 0.7}]  # matches first only
    assert s2r.match_detections(ref, qry) == 1


def test_match_detections_class_must_agree():
    ref = [{'cls': 0, 'box': [0, 0, 2, 2], 'conf': 0.9}]
    qry = [{'cls': 1, 'box': [0, 0, 2, 2], 'conf': 0.9}]
    assert s2r.match_detections(ref, qry) == 0


def test_select_views_even_spacing():
    assert s2r.select_views(10, 0) == list(range(10))
    assert s2r.select_views(5, 10) == list(range(5))
    assert s2r.select_views(10, 3) == [0, 4, 9]


def test_parse_offsets_includes_zero_and_sorts():
    assert s2r.parse_offsets('0.5,-0.5,1.0') == [0.0, -0.5, 0.5, 1.0]


def test_montage_tiles_grid():
    a = np.full((4, 4, 3), 10, dtype=np.uint8)
    b = np.full((4, 4, 3), 20, dtype=np.uint8)
    sheet = s2r.montage([[a, b]], pad=1)
    assert sheet.shape == (4 + 2, 4 * 2 + 3, 3)
    assert (sheet == 10).any() and (sheet == 20).any()
