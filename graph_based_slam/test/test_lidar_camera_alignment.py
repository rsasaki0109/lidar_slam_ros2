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

"""Tests for LiDAR-camera depth/image edge alignment metrics."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'evaluate_lidar_camera_alignment',
    REPO_ROOT / 'scripts' / 'evaluate_lidar_camera_alignment.py')
lca = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(lca)


def test_image_edges_detects_vertical_step():
    image = np.zeros((20, 20, 3), dtype=np.uint8)
    image[:, 10:] = 255
    edges = lca.image_edges(image, percentile=50)
    assert edges[:, 9:11].any()
    assert not edges[:, :5].any()


def test_depth_edges_detects_supported_depth_step():
    depth = np.full((10, 10), np.inf, dtype=np.float32)
    depth[5, 3:5] = [2.0, 4.0]
    edges = lca.depth_edges(depth, absolute=0.25, relative=0.0)
    assert edges[5, 3] and edges[5, 4]
    assert edges.sum() == 2


def test_nearest_edge_distances_reports_alignment_and_shift():
    query = np.zeros((20, 20), dtype=bool)
    target = np.zeros_like(query)
    query[5:15, 8] = True
    target[5:15, 11] = True
    np.testing.assert_allclose(
        lca.nearest_edge_distances(query, target, max_distance=6), 3.0)
    np.testing.assert_allclose(
        lca.nearest_edge_distances(query, query, max_distance=6), 0.0)


def test_projected_depth_keeps_nearest_point():
    points = np.array([[0.0, 0.0, 2.0], [0.0, 0.0, 5.0]])
    K = np.array([[10.0, 0.0, 5.0], [0.0, 10.0, 5.0], [0.0, 0.0, 1.0]])
    depth = lca.projected_depth(points, np.eye(4), K, 10, 10)
    assert depth[5, 5] == 2.0


def test_score_view_handles_no_depth_edges():
    points = np.array([[0.0, 0.0, 2.0]])
    K = np.array([[10.0, 0.0, 5.0], [0.0, 10.0, 5.0], [0.0, 0.0, 1.0]])
    result = lca.score_view(
        points, np.eye(4), K, np.zeros((10, 10, 3), dtype=np.uint8))
    assert result['edge_points'] == 0
    assert result['median_px'] is None


def test_correction_matrix_applies_translation_and_rotation():
    matrix = lca.correction_matrix([1.0, 2.0, 3.0, 0.0, 0.0, np.pi / 2])
    np.testing.assert_allclose(matrix[:3, 3], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(
        matrix[:3, :3] @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)


def test_optimize_correction_descends_synthetic_objective(monkeypatch):
    target = np.array([0.02, -0.01, 0.0, np.deg2rad(0.2), 0.0, 0.0])

    def objective(*args, reference_edge_points=None, **kwargs):
        parameters = args[4]
        loss = float(np.sum((parameters - target) ** 2))
        return loss, {'edge_points': 100, 'mean_px': loss,
                      'median_px': loss, 'coverage': 1.0}

    monkeypatch.setattr(lca, 'alignment_objective', objective)
    parameters, before, after = lca.optimize_correction(
        np.zeros((0, 3)), np.zeros((0, 4, 4)), np.eye(3), [], rounds=2)
    assert after['loss'] < before['loss']
    np.testing.assert_allclose(parameters, target, atol=np.deg2rad(0.1))


def test_write_corrected_transforms_preserves_images_and_updates_pose(tmp_path):
    images = tmp_path / 'source' / 'images'
    images.mkdir(parents=True)
    (images / '000.png').write_bytes(b'pixel')
    source = tmp_path / 'source' / 'transforms.json'
    source.write_text(json.dumps({
        'fl_x': 10, 'fl_y': 10, 'cx': 5, 'cy': 5, 'w': 10, 'h': 10,
        'frames': [{'file_path': 'images/000.png',
                    'transform_matrix': np.eye(4).tolist()}],
    }))
    output = tmp_path / 'result' / 'corrected.json'
    correction = lca.correction_matrix([0.1, 0, 0, 0, 0, 0])
    original = lca.tg.load_transforms(source)['viewmats'][0]
    lca.write_corrected_transforms(source, output, correction)
    loaded = lca.tg.load_transforms(output)
    np.testing.assert_allclose(loaded['viewmats'][0], correction @ original)
    assert loaded['image_paths'][0] == (images / '000.png').resolve()
