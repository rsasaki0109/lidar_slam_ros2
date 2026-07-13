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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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

"""Tests for FAST-LIVO2-style plane-prior patch warping."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'plane_patch_warp',
    REPO_ROOT / 'tools' / 'gaussian_splatting' / 'plane_patch_warp.py')
WARP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WARP)


def _camera():
    return np.array([[100.0, 0.0, 50.0],
                     [0.0, 100.0, 50.0],
                     [0.0, 0.0, 1.0]])


def test_identity_pose_produces_identity_homography():
    H = WARP.plane_homography(
        _camera(), np.eye(4), [0.0, 0.0, 1.0], [0.0, 0.0, 5.0])
    np.testing.assert_allclose(H, np.eye(3), atol=1.0e-12)


def test_fronto_parallel_translation_matches_direct_projection():
    transform = np.eye(4)
    transform[0, 3] = 0.5
    K = _camera()
    H = WARP.plane_homography(K, transform, [0, 0, 1], [0, 0, 5])
    warped, valid = WARP.warp_pixels(H, np.array([[50.0, 50.0]]))
    assert valid[0]
    # A point at z=5 translated +0.5 m in target camera projects +10 px.
    np.testing.assert_allclose(warped, [[60.0, 50.0]], atol=1.0e-9)


def test_plane_normal_sign_does_not_change_homography():
    transform = np.eye(4)
    transform[:3, 3] = [0.1, -0.2, 0.0]
    positive = WARP.plane_homography(
        _camera(), transform, [0, 0, 1], [0, 0, 4])
    negative = WARP.plane_homography(
        _camera(), transform, [0, 0, -1], [0, 0, 4])
    np.testing.assert_allclose(positive, negative)


def test_bilinear_sample_and_ncc_recover_shifted_patch():
    image = np.tile(np.arange(20, dtype=np.float32), (20, 1))
    pixels = np.array([[5.0, 5.0], [6.0, 5.0], [5.0, 6.0], [6.0, 6.0]])
    reference, valid = WARP.bilinear_sample(image, pixels)
    target, target_valid = WARP.bilinear_sample(image, pixels + [2.0, 0.0])
    assert valid.all() and target_valid.all()
    assert WARP.zero_mean_ncc(reference, target) == pytest.approx(1.0)


def test_degenerate_plane_and_textureless_ncc_are_rejected():
    with pytest.raises(ValueError, match='camera centre'):
        WARP.plane_homography(_camera(), np.eye(4), [1, 0, 0], [0, 0, 5])
    assert WARP.zero_mean_ncc(np.ones(9), np.ones(9)) is None


def test_reference_update_rejects_inconsistent_patch():
    y, x = np.mgrid[:100, :100]
    texture = (3.0 * x + 2.0 * y + 20.0 * np.sin(x / 4.0)).astype(np.float32)
    corrupted = np.flipud(texture).copy()
    views = np.repeat(np.eye(4)[None], 3, axis=0)
    views[1, 0, 3] = 0.1
    views[2, 0, 3] = -0.1
    selected, scores = WARP.select_reference_patch(
        [texture, texture, corrupted], _camera(), views,
        np.array([0.0, 0.0, 5.0]), np.array([0.0, 0.0, 1.0]))
    assert selected in (0, 1)
    assert scores[selected] > scores[2]


def test_reference_update_requires_two_usable_views():
    selected, scores = WARP.select_reference_patch(
        [np.zeros((100, 100), np.float32)], _camera(), np.eye(4)[None],
        np.array([0.0, 0.0, 5.0]), np.array([0.0, 0.0, 1.0]))
    assert selected is None
    assert np.isneginf(scores[0])


def test_planar_voxel_batch_selects_consistent_view_and_applies_colour():
    y, x = np.mgrid[:100, :100]
    texture = (3.0 * x + 2.0 * y + 20.0 * np.sin(x / 4.0)).astype(np.float32)
    corrupted = np.flipud(texture)
    views = np.repeat(np.eye(4)[None], 3, axis=0)
    views[1, 0, 3] = 0.1
    views[2, 0, 3] = -0.1
    xy = np.array([(a, b) for a in np.linspace(0.05, 0.35, 4)
                   for b in np.linspace(0.05, 0.35, 4)])
    points = np.column_stack((xy, np.full(len(xy), 5.0)))
    refs = WARP.select_planar_voxel_references(
        points, [texture, texture, corrupted], _camera(), views)
    assert np.all(refs == refs[0])
    assert refs[0] in (0, 1)
    rgb_images = [np.repeat(image[:, :, None], 3, axis=2)
                  for image in (texture, texture, corrupted)]
    colours, updated = WARP.apply_reference_colours(
        points, rgb_images, _camera(), views, refs,
        np.zeros((len(points), 3), dtype=np.uint8))
    assert updated.all()
    assert colours.any()
