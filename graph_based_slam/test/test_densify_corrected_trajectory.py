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

"""Tests for dense propagation of sparse pose-graph corrections."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import posed_images as pi  # noqa: E402

SPEC = importlib.util.spec_from_file_location(
    'densify_corrected_trajectory',
    REPO_ROOT / 'scripts' / 'densify_corrected_trajectory.py')
dct = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = dct
SPEC.loader.exec_module(dct)


def _sample(stamp, x, quat=(0.0, 0.0, 0.0, 1.0)):
    return pi.TrajectorySample(
        stamp=float(stamp),
        translation=np.array([x, 0.0, 0.0], dtype=float),
        quat_xyzw=np.asarray(quat, dtype=float),
    )


def test_dense_translation_correction_matches_anchors_and_interpolates():
    raw = [_sample(0, 0), _sample(1, 1), _sample(2, 2)]
    corrected = [_sample(0, 10), _sample(2, 14)]
    dense = dct.densify_trajectory(raw, corrected)
    np.testing.assert_allclose(
        [sample.translation[0] for sample in dense], [10, 12, 14],
        atol=1e-12)
    assert [sample.stamp for sample in dense] == [0, 1, 2]


def test_world_side_rotation_correction_uses_slerp():
    raw = [_sample(0, 0), _sample(1, 0), _sample(2, 0)]
    half_turn_z = (0.0, 0.0, 1.0, 0.0)
    corrected = [_sample(0, 0), _sample(2, 0, half_turn_z)]
    dense = dct.densify_trajectory(raw, corrected)
    rotated_x = pi.quat_to_matrix(dense[1].quat_xyzw) @ np.array([1, 0, 0])
    np.testing.assert_allclose(rotated_x, [0, 1, 0], atol=1e-9)


def test_correction_clamps_before_and_after_graph_anchor_range():
    raw = [_sample(0, 0), _sample(1, 1), _sample(2, 2), _sample(3, 3)]
    corrected = [_sample(1, 11), _sample(2, 12)]
    dense = dct.densify_trajectory(raw, corrected)
    np.testing.assert_allclose(
        [sample.translation[0] for sample in dense], [10, 11, 12, 13],
        atol=1e-12)


def test_write_and_read_tum_round_trip(tmp_path):
    samples = [_sample(1.25, 3.5), _sample(1.35, 4.5)]
    path = dct.write_tum(tmp_path / 'dense.tum', samples)
    loaded = pi.read_tum_trajectory(path)
    assert len(loaded) == 2
    np.testing.assert_allclose(loaded[1].translation, [4.5, 0, 0])


def test_matrix_to_quaternion_round_trip_for_half_turn():
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    recovered = dct.matrix_to_quat_xyzw(pi.quat_to_matrix(quaternion))
    np.testing.assert_allclose(
        pi.quat_to_matrix(recovered), pi.quat_to_matrix(quaternion), atol=1e-12)
