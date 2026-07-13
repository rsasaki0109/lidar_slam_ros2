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

"""Tests for held-out camera-coloured point-map evaluation."""

import importlib.util
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'evaluate_heldout_point_colors',
    REPO_ROOT / 'scripts' / 'evaluate_heldout_point_colors.py')
hpc = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(hpc)


def _camera():
    return np.eye(4), np.array([
        [10.0, 0.0, 5.0], [0.0, 10.0, 5.0], [0.0, 0.0, 1.0]])


def test_visible_point_samples_keeps_nearest_per_pixel():
    vm, K = _camera()
    points = np.array([[0.0, 0.0, 5.0], [0.0, 0.0, 2.0],
                       [0.2, 0.0, 2.0]])
    ids, _, _ = hpc.visible_point_samples(points, vm, K, 10, 10)
    assert ids.tolist() == [1, 2]


def test_visible_point_samples_ignores_non_finite_projections():
    vm, K = _camera()
    points = np.array([[0.0, 0.0, 0.0], [np.nan, 0.0, 2.0],
                       [0.0, 0.0, 2.0]])
    ids, _, _ = hpc.visible_point_samples(points, vm, K, 10, 10)
    assert ids.tolist() == [2]


def test_score_heldout_view_zero_for_matching_color():
    vm, K = _camera()
    points = np.array([[0.0, 0.0, 2.0]])
    colors = np.array([[10, 20, 30]], dtype=np.uint8)
    image = np.zeros((10, 10, 3), dtype=np.uint8)
    image[5, 5] = colors[0]
    errors, visible = hpc.score_heldout_view(
        points, colors, np.array([True]), vm, K, image)
    assert visible == 1
    np.testing.assert_allclose(errors, [0.0])


def test_score_heldout_view_excludes_training_unseen_points():
    vm, K = _camera()
    points = np.array([[0.0, 0.0, 2.0]])
    errors, visible = hpc.score_heldout_view(
        points, np.zeros((1, 3), dtype=np.uint8), np.array([False]),
        vm, K, np.zeros((10, 10, 3), dtype=np.uint8))
    assert visible == 1
    assert errors.size == 0


def test_exposure_scales_are_clamped():
    images = [np.full((4, 4, 3), 10, dtype=np.uint8),
              np.full((4, 4, 3), 100, dtype=np.uint8),
              np.full((4, 4, 3), 200, dtype=np.uint8)]
    scales = hpc.exposure_scales(images, limit=1.5)
    np.testing.assert_allclose(scales, [1.5, 1.0, 2.0 / 3.0])


def test_score_heldout_view_can_compare_raw_exposure():
    vm, K = _camera()
    points = np.array([[0.0, 0.0, 2.0]])
    colors = np.array([[10, 20, 30]], dtype=np.uint8)
    image = np.zeros((10, 10, 3), dtype=np.uint8)
    image[5, 5] = colors[0]
    raw_errors, _ = hpc.score_heldout_view(
        points, colors, np.array([True]), vm, K, image,
        exposure_scale=1.0)
    scaled_errors, _ = hpc.score_heldout_view(
        points, colors, np.array([True]), vm, K, image,
        exposure_scale=1.5)
    np.testing.assert_allclose(raw_errors, [0.0])
    assert scaled_errors[0] > 0.0
