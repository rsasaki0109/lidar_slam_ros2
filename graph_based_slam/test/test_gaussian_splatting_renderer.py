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

"""Tests for the sensor-sim renderer's pure pose maths (CPU, no torch/gsplat)."""

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
    import gaussian_renderer
    import pose_player

    return gaussian_renderer, pose_player


gr, pp = _load()


def _cam_center(viewmat):
    return -viewmat[:3, :3].T @ viewmat[:3, 3]


def test_transform_identity_quat_is_pure_translation():
    t = gr.transform_from_pos_quat([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    assert np.allclose(t[:3, :3], np.eye(3))
    assert np.allclose(t[:3, 3], [1.0, 2.0, 3.0])
    assert np.allclose(t[3], [0.0, 0.0, 0.0, 1.0])


def test_transform_quat_rotation():
    # 90 deg about +z (xyzw): maps +x -> +y.
    t = gr.transform_from_pos_quat([0.0, 0.0, 0.0],
                                   [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    assert np.allclose(t[:3, :3] @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0],
                       atol=1e-9)


def test_pose_to_viewmat_is_inverse_of_camera_pose():
    t_wb = gr.transform_from_pos_quat([2.0, -1.0, 0.5],
                                      [0.0, 0.0, 0.0, 1.0])
    t_bc = gr.transform_from_pos_quat([0.1, 0.0, 0.2],
                                      [0.0, 0.0, 0.0, 1.0])
    vm = gr.pose_to_viewmat(t_wb, t_bc)
    assert np.allclose(vm @ (t_wb @ t_bc), np.eye(4), atol=1e-9)


def test_pose_to_viewmat_recovers_camera_centre():
    t_wb = gr.transform_from_pos_quat([5.0, 3.0, 1.0],
                                      [0.0, 0.0, np.sin(0.3), np.cos(0.3)])
    t_bc = gr.transform_from_pos_quat([0.0, 0.0, 0.0],
                                      [0.0, 0.0, 0.0, 1.0])
    vm = gr.pose_to_viewmat(t_wb, t_bc)
    assert np.allclose(_cam_center(vm), [5.0, 3.0, 1.0], atol=1e-9)


def test_pose_to_viewmat_applies_alignment():
    t_wb = gr.transform_from_pos_quat([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    t_bc = np.eye(4)
    t_align = gr.transform_from_pos_quat([10.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    vm = gr.pose_to_viewmat(t_wb, t_bc, t_align)
    # camera at world x=1, shifted by align +10 -> model-world x=11.
    assert np.allclose(_cam_center(vm), [11.0, 0.0, 0.0], atol=1e-9)


def test_pose_to_viewmat_default_align_is_identity():
    t_wb = gr.transform_from_pos_quat([0.0, 4.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    t_bc = np.eye(4)
    assert np.allclose(gr.pose_to_viewmat(t_wb, t_bc),
                       gr.pose_to_viewmat(t_wb, t_bc, np.eye(4)))


def test_pose_to_viewmat_extrinsic_offsets_camera():
    t_wb = np.eye(4)
    t_bc = gr.transform_from_pos_quat([0.0, 0.0, 1.5], [0.0, 0.0, 0.0, 1.0])
    vm = gr.pose_to_viewmat(t_wb, t_bc)
    assert np.allclose(_cam_center(vm), [0.0, 0.0, 1.5], atol=1e-9)


def _rand_viewmat(seed):
    rng = np.random.default_rng(seed)
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = rng.uniform(-np.pi, np.pi)
    half = angle / 2.0
    quat = np.array([*(axis * np.sin(half)), np.cos(half)])
    c2w = gr.transform_from_pos_quat(rng.normal(size=3) * 3.0, quat)
    return np.linalg.inv(c2w)


@pytest.mark.parametrize('seed', [0, 1, 7, 42])
def test_pose_player_roundtrips_through_node_math(seed):
    # pose_player publishes inv(viewmat) as a pose; the node rebuilds the
    # viewmat with identity extrinsic/align. The pair must be exact inverses.
    vm = _rand_viewmat(seed)
    pos, quat = pp.viewmat_to_pos_quat(vm)
    t_world_base = gr.transform_from_pos_quat(pos, quat)
    recovered = gr.pose_to_viewmat(t_world_base, np.eye(4), np.eye(4))
    assert np.allclose(recovered, vm, atol=1e-9)


def test_viewmat_to_pos_quat_recovers_centre():
    vm = _rand_viewmat(3)
    pos, _ = pp.viewmat_to_pos_quat(vm)
    assert np.allclose(pos, _cam_center(vm), atol=1e-9)
