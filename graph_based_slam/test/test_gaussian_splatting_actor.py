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

"""Tests for the Phase 3 actor-compositing pure helpers (CPU, no torch/gsplat)."""

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
    import actor_compositing

    return actor_compositing


ac = _load()


def test_make_box_actor_fills_and_rests_on_ground():
    box = ac.make_box_actor([0.6, 0.6, 1.7], spacing=0.2)
    means = box['means']
    assert box['sh_rest'] is None
    assert means.shape[0] == box['quats'].shape[0] > 0
    assert np.all(box['quats'][:, 0] == 1.0)  # wxyz identity
    assert means[:, 2].min() == pytest.approx(0.0)  # base on the ground
    assert means[:, 2].max() == pytest.approx(1.7, abs=0.2)
    assert abs(means[:, 0]).max() == pytest.approx(0.3, abs=1e-9)


def test_make_box_actor_rejects_bad_dims():
    with pytest.raises(ValueError):
        ac.make_box_actor([0.0, 1.0, 1.0])


def test_transform_gaussians_translates_means():
    box = ac.make_box_actor([0.4, 0.4, 1.0], spacing=0.5)
    t = ac.rigid_from_pos_yaw([10.0, -5.0, 2.0], 0.0)
    moved = ac.transform_gaussians(box, t)
    assert np.allclose(moved['means'].mean(axis=0),
                       box['means'].mean(axis=0) + [10.0, -5.0, 2.0])


def test_transform_gaussians_rotates_orientation_quats():
    box = ac.make_box_actor([0.4, 0.4, 0.4], spacing=0.4)
    t = ac.rigid_from_pos_yaw([0.0, 0.0, 0.0], np.pi / 2.0)
    moved = ac.transform_gaussians(box, t)
    assert not np.allclose(moved['quats'], box['quats'])  # yaw changed heading
    assert np.allclose(np.linalg.norm(moved['quats'], axis=1), 1.0)


def test_rigid_from_pos_yaw_is_rotation_about_up():
    t = ac.rigid_from_pos_yaw([0.0, 0.0, 0.0], np.pi / 2.0, up_axis=2)
    # +x (1,0,0) rotates to +y under a +90 deg yaw about z
    assert np.allclose(t[:3, :3] @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-9)
    assert np.allclose(t[:3, :3] @ [0.0, 0.0, 1.0], [0.0, 0.0, 1.0])  # up fixed


def test_actor_world_poses_places_actor_in_front():
    c2w = np.eye(4)  # camera at origin, +z forward (OpenCV)
    poses = ac.actor_world_poses(c2w, [-1.0, 0.0, 1.0], distance=5.0, drop=0.8)
    assert np.allclose(poses[1], [0.0, 0.8, 5.0])  # centred: ahead and below
    assert poses[0][0] == -1.0 and poses[2][0] == 1.0  # lateral sweep


def test_project_points_centre_maps_to_principal_point():
    K = np.array([[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]])
    vm = np.eye(4)
    uv, z = ac.project_points(np.array([[0.0, 0.0, 2.0]]), vm, K)
    assert np.allclose(uv[0], [50.0, 40.0])  # on-axis point -> principal point
    assert z[0] == pytest.approx(2.0)


def test_points_to_bbox_clips_to_image():
    uv = np.array([[10.0, 20.0], [60.0, 90.0]])
    z = np.array([1.0, 1.0])
    assert ac.points_to_bbox(uv, z, 50, 50) == [10, 20, 50, 50]


def test_points_to_bbox_none_when_behind_camera():
    uv = np.array([[10.0, 10.0]])
    assert ac.points_to_bbox(uv, np.array([-1.0]), 50, 50) is None


def test_composite_depth_actor_in_front_wins():
    scene = np.zeros((4, 4, 3), dtype=np.uint8)
    actor = np.full((4, 4, 3), 200, dtype=np.uint8)
    scene_d = np.full((4, 4), 5.0)
    actor_d = np.full((4, 4), 2.0)  # nearer everywhere
    alpha = np.ones((4, 4))
    out, mask = ac.composite_depth(scene, scene_d, actor, actor_d, alpha)
    assert mask.all() and (out == 200).all()


def test_composite_depth_actor_behind_is_occluded():
    scene = np.zeros((4, 4, 3), dtype=np.uint8)
    actor = np.full((4, 4, 3), 200, dtype=np.uint8)
    scene_d = np.full((4, 4), 1.0)
    actor_d = np.full((4, 4), 3.0)  # behind the scene
    alpha = np.ones((4, 4))
    out, mask = ac.composite_depth(scene, scene_d, actor, actor_d, alpha)
    assert not mask.any() and (out == 0).all()


def test_composite_depth_draws_over_empty_scene():
    scene = np.zeros((2, 2, 3), dtype=np.uint8)
    actor = np.full((2, 2, 3), 150, dtype=np.uint8)
    scene_d = np.zeros((2, 2))  # no scene return
    actor_d = np.full((2, 2), 4.0)
    out, mask = ac.composite_depth(scene, scene_d, actor, actor_d, np.ones((2, 2)))
    assert mask.all() and (out == 150).all()


def test_resize_nearest_changes_shape_preserves_values():
    img = np.array([[[0, 0, 0], [255, 255, 255]]], dtype=np.uint8)  # 1x2
    out = ac.resize_nearest(img, 4, 2)
    assert out.shape == (2, 4, 3)
    assert set(np.unique(out)) <= {0, 255}


def test_paste_sprite_blends_opaque_pixels_and_returns_bbox():
    scene = np.zeros((20, 20, 3), dtype=np.uint8)
    scene_d = np.full((20, 20), 10.0)
    sprite = np.zeros((4, 4, 4), dtype=np.uint8)
    sprite[..., :3] = 200
    sprite[..., 3] = 255  # fully opaque
    out, bbox = ac.paste_sprite(scene, scene_d, sprite, [10.0, 10.0], 8, 3.0)
    assert bbox is not None
    x1, y1, x2, y2 = bbox
    assert (out[y1:y2, x1:x2] == 200).any()
    assert 0 <= x1 < x2 <= 20 and 0 <= y1 < y2 <= 20


def test_paste_sprite_occluded_by_nearer_scene():
    scene = np.zeros((20, 20, 3), dtype=np.uint8)
    scene_d = np.full((20, 20), 1.0)  # scene nearer than the sprite
    sprite = np.zeros((4, 4, 4), dtype=np.uint8)
    sprite[..., 3] = 255
    out, bbox = ac.paste_sprite(scene, scene_d, sprite, [10.0, 10.0], 8, 5.0)
    assert bbox is None and (out == 0).all()


def test_paste_sprite_off_frame_returns_none():
    scene = np.zeros((10, 10, 3), dtype=np.uint8)
    scene_d = np.full((10, 10), 10.0)
    sprite = np.full((4, 4, 4), 255, dtype=np.uint8)
    out, bbox = ac.paste_sprite(scene, scene_d, sprite, [-50.0, -50.0], 4, 3.0)
    assert bbox is None


def test_linspace_sym_spans_range():
    assert ac.linspace_sym(2.0, 1) == [0.0]
    assert ac.linspace_sym(2.0, 3) == [-2.0, 0.0, 2.0]


def test_linspace_sym_rejects_nonpositive_count():
    with pytest.raises(ValueError):
        ac.linspace_sym(1.0, 0)


def test_logit_sigmoid_roundtrip():
    for p in (0.1, 0.5, 0.99):
        assert 1.0 / (1.0 + np.exp(-ac.logit(p))) == pytest.approx(p, abs=1e-6)


def _toy_scene(n=40):
    rng = np.random.default_rng(0)
    means = rng.uniform(-5.0, 5.0, (n, 3))
    return {
        'means': means,
        'scales_log': np.full((n, 3), -2.0),
        'quats': np.tile([1.0, 0.0, 0.0, 0.0], (n, 1)),
        'opacities_logit': np.zeros(n),
        'colors_rgb': rng.uniform(0.0, 1.0, (n, 3)),
        'sh_rest': None,
    }


def test_crop_gaussians_keeps_only_in_box():
    g = _toy_scene()
    g['means'][0] = [0.0, 0.0, 0.5]   # inside
    g['means'][1] = [9.0, 9.0, 0.5]   # outside in x/y
    out = ac.crop_gaussians(g, [0.0, 0.0], 1.0, [0.0, 2.0])
    m = out['means']
    assert np.all(np.abs(m[:, 0]) <= 1.0) and np.all(np.abs(m[:, 1]) <= 1.0)
    assert np.all((m[:, 2] >= 0.0) & (m[:, 2] <= 2.0))
    # every per-gaussian array is subset to the same length
    assert out['colors_rgb'].shape[0] == m.shape[0] == out['quats'].shape[0]


def test_crop_gaussians_subsets_sh_rest_when_present():
    g = _toy_scene(6)
    g['means'][:] = 0.0  # all inside
    g['sh_rest'] = np.arange(6 * 3 * 3, dtype=float).reshape(6, 3, 3)
    out = ac.crop_gaussians(g, [0.0, 0.0], 1.0, [-1.0, 1.0])
    assert out['sh_rest'].shape == (6, 3, 3)


def test_crop_gaussians_raises_on_empty_box():
    with pytest.raises(ValueError):
        ac.crop_gaussians(_toy_scene(), [100.0, 100.0], 0.1, [0.0, 1.0])


def test_recenter_gaussians_zeroes_xy_centroid_and_grounds_z():
    g = _toy_scene(2)
    g['means'] = np.array([[1.0, 2.0, 3.0], [3.0, 4.0, 5.0]], dtype=float)
    g['quats'] = np.tile([1.0, 0.0, 0.0, 0.0], (2, 1))
    out = ac.recenter_gaussians(g)
    assert np.allclose(out['means'][:, :2].mean(axis=0), [0.0, 0.0])
    assert out['means'][:, 2].min() == pytest.approx(0.0)
    assert np.allclose(out['quats'], g['quats'])  # pure translation
