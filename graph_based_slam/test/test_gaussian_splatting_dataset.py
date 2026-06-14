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

"""Tests for the Phase 1 dataset-generation pure helpers (CPU, no torch/gsplat)."""

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
    import generate_dataset

    return generate_dataset


gd = _load()


def _viewmat(center, rot=None):
    rot = np.eye(3) if rot is None else np.asarray(rot, dtype=float)
    vm = np.eye(4)
    vm[:3, :3] = rot
    vm[:3, 3] = -rot @ np.asarray(center, dtype=float)
    return vm


def _center_of(vm):
    return -vm[:3, :3].T @ vm[:3, 3]


def test_depth_quantisation_roundtrips_within_one_unit():
    depth = np.array([[0.0, 0.5, 1.234], [2.0, 10.0, 0.001]])
    u16 = gd.depth_to_uint16(depth, depth_scale=0.001)
    back = gd.uint16_to_depth(u16, depth_scale=0.001)
    assert u16.dtype == np.uint16
    assert np.all(np.abs(back - depth) <= 0.001)


def test_depth_quantisation_zeroes_invalid_samples():
    depth = np.array([[np.nan, -1.0, np.inf, 3.0]])
    u16 = gd.depth_to_uint16(depth, depth_scale=0.001)
    assert list(u16.ravel()) == [0, 0, 0, 3000]


def test_depth_quantisation_clamps_max_depth_and_ceiling():
    depth = np.array([[5.0, 1000.0]])
    u16 = gd.depth_to_uint16(depth, depth_scale=0.001, max_depth=2.0)
    assert u16[0, 0] == 2000 and u16[0, 1] == 2000  # both clipped to 2 m
    # without max_depth, 1000 m at mm scale saturates the uint16 ceiling
    u16b = gd.depth_to_uint16(depth, depth_scale=0.001)
    assert u16b[0, 1] == 65535


def test_depth_scale_must_be_positive():
    with pytest.raises(ValueError):
        gd.depth_to_uint16(np.zeros((2, 2)), depth_scale=0.0)


def test_plan_jitters_first_is_recorded_pose():
    js = gd.plan_jitters(4, max_lateral=0.5, max_vertical=0.1, seed=1)
    assert js[0] == (0.0, 0.0)
    assert len(js) == 5


def test_plan_jitters_respects_bounds():
    js = gd.plan_jitters(50, max_lateral=0.5, max_vertical=0.1, seed=2)
    for dx, dy in js:
        assert -0.5 <= dx <= 0.5
        assert -0.1 <= dy <= 0.1


def test_plan_jitters_is_deterministic():
    a = gd.plan_jitters(8, max_lateral=0.3, max_vertical=0.2, seed=7)
    b = gd.plan_jitters(8, max_lateral=0.3, max_vertical=0.2, seed=7)
    assert a == b


def test_plan_jitters_zero_aug_is_just_recorded():
    assert gd.plan_jitters(0, max_lateral=1.0, max_vertical=1.0) == [(0.0, 0.0)]


def test_plan_jitters_rejects_negative():
    with pytest.raises(ValueError):
        gd.plan_jitters(-1, max_lateral=0.5, max_vertical=0.1)


def test_jittered_viewmat_shifts_camera_local_right_and_down():
    vm = _viewmat([0.0, 0.0, 0.0])  # identity rotation -> local == world
    moved = gd.jittered_viewmat(vm, 0.5, 0.2)
    assert np.allclose(_center_of(moved), [0.5, 0.2, 0.0])
    assert np.allclose(moved[:3, :3], vm[:3, :3])  # heading preserved


def test_jittered_viewmat_zero_is_identity():
    vm = _viewmat([1.0, -2.0, 3.0])
    assert np.allclose(gd.jittered_viewmat(vm, 0.0, 0.0), vm)


def test_c2w_to_opengl_inverts_and_flips():
    center = [1.0, 2.0, 3.0]
    vm = _viewmat(center)
    c2w_gl = gd.c2w_to_opengl(vm)
    # translation (camera centre) is unchanged by the optical-axis flip
    assert np.allclose(c2w_gl[:3, 3], center)
    # the flip negates the y and z basis columns of the identity rotation
    assert np.allclose(np.diag(c2w_gl[:3, :3]), [1.0, -1.0, -1.0])


def test_c2w_to_opengl_roundtrips_through_load_transforms_convention():
    # load_transforms does: c2w_cv = c2w_gl @ ROS_OPTICAL_TO_OPENGL; vm = inv.
    import posed_images as pi

    vm = _viewmat([0.4, -0.6, 2.5],
                  rot=np.array([[0.0, 1.0, 0.0],
                                [-1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0]]))
    c2w_gl = gd.c2w_to_opengl(vm)
    c2w_cv = np.asarray(c2w_gl) @ pi.ROS_OPTICAL_TO_OPENGL
    assert np.allclose(np.linalg.inv(c2w_cv), vm)


def test_build_manifest_carries_intrinsics_and_depth_meta():
    K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    frames = [{'file_path': 'rgb/a.png'}]
    m = gd.build_manifest(K, 640, 480, frames, depth_scale=0.001)
    assert m['w'] == 640 and m['h'] == 480
    assert m['fl_x'] == 500.0 and m['cx'] == 320.0
    assert m['depth_scale'] == 0.001
    assert m['camera_model'] == 'OPENCV'
    assert m['frames'] == frames
