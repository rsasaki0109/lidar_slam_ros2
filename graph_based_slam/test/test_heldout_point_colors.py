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


def test_exposure_reference_excludes_heldout_brightness():
    images = [np.full((4, 4, 3), value, dtype=np.uint8)
              for value in (10, 20, 200)]
    scales = hpc.exposure_scales(images, limit=20, reference_indices=[0, 1])
    np.testing.assert_allclose(scales, [1.5, 0.75, 0.075], atol=1e-7)
    images[2][:] = 100
    changed = hpc.exposure_scales(images, limit=20, reference_indices=[0, 1])
    np.testing.assert_allclose(changed, [1.5, 0.75, 0.15], atol=1e-7)
    # A dark training set supplies no exposure reference, as in fusion.
    images[0][:] = images[1][:] = 0
    np.testing.assert_allclose(
        hpc.exposure_scales(images, reference_indices=[0, 1]), [1, 1, 1])


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


def test_shared_fusion_cli_preserves_split_and_observation_threshold(tmp_path, monkeypatch):
    import imageio as iio
    import json
    import sys

    frames = []
    for index, value in enumerate((20, 200, 20, 200)):
        name = f'{index}.png'
        iio.imwrite(tmp_path / name, np.full((10, 10, 3), value, dtype=np.uint8))
        frames.append({'file_path': name, 'timestamp': float(index),
                       'transform_matrix': np.diag([1., -1., -1., 1.]).tolist()})
    transforms = tmp_path / 'transforms.json'
    transforms.write_text(json.dumps({'w': 10, 'h': 10, 'fl_x': 5., 'fl_y': 5.,
                                     'cx': 5., 'cy': 5., 'frames': frames}))
    cloud = tmp_path / 'cloud.ply'
    hpc.pcio.write_ply(cloud, np.array([[0., 0., 2.]]))
    out = tmp_path / 'report.json'
    options = {'robust': True, 'normalize_exposure': False, 'max_samples': 1,
               'min_samples': 1, 'image_margin': 1}
    argv = ['evaluate', '--pointcloud', str(cloud), '--transforms', str(transforms),
            '--out', str(out), '--view-stride', '1', '--no-normalize-exposure',
            '--fusion-options', json.dumps(options)]
    monkeypatch.setattr(sys, 'argv', argv)
    assert hpc.main() == 0
    report = json.loads(out.read_text())
    assert report['train_view_indices'] == [0, 2]
    assert report['heldout_view_indices'] == [1, 3]
    assert report['fusion_options']['max_samples'] == 1
    np.testing.assert_allclose(report['rgb_l2_mean'], np.sqrt(3) * 180, rtol=1e-6)
    options['min_samples'] = 2
    argv[-1] = json.dumps(options)
    with np.testing.assert_raises(SystemExit):
        hpc.main()  # A single retained sample is below the configured minimum.


def test_fusion_options_reject_hidden_frame_override_and_wrong_types():
    for text in ('[]', '{"loaded_images": 1}', '{"frame_indices": [1]}', '{"max_samples": true}',
                 '{"normalize_exposure": "false"}', '{"normal_voxel": NaN}'):
        with np.testing.assert_raises(ValueError):
            hpc.parse_fusion_options(text)


def test_visible_point_samples_excludes_extreme_pixels_without_integer_overflow():
    vm, K = _camera()
    points = np.array([[1e30, 0., 2.], [-1e30, 0., 2.],
                       [0., 1e30, 2.], [0., -1e30, 2.], [0., 0., 2.]])
    with np.errstate(invalid='raise', over='raise'):
        ids, uf, vf = hpc.visible_point_samples(points, vm, K, 10, 10)
    assert ids.tolist() == [4]
    np.testing.assert_array_equal(uf, [5.])
    np.testing.assert_array_equal(vf, [5.])
