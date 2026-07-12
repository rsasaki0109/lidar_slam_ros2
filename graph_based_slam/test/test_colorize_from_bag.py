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

"""Tests for the numpy-only helpers in colorize_from_bag (no ROS needed)."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import colorize_from_bag

    return colorize_from_bag


cfb = _load()


class _Vec3:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Quat:
    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w


# --------------------------------------------------------------------------- #
# nearest_index
# --------------------------------------------------------------------------- #
def test_nearest_index_picks_closest():
    stamps = [100, 200, 300, 400]
    # 260-200=60, 300-260=40 -> index 2 wins.
    assert cfb.nearest_index(stamps, 260) == 2
    assert cfb.nearest_index(stamps, 240) == 1  # 40 vs 60 -> index 1


def test_nearest_index_exact_and_ends():
    stamps = [10, 20, 30]
    assert cfb.nearest_index(stamps, 20) == 1
    assert cfb.nearest_index(stamps, -5) == 0
    assert cfb.nearest_index(stamps, 999) == 2


def test_nearest_index_empty_raises():
    import pytest
    with pytest.raises(ValueError):
        cfb.nearest_index([], 5)


# --------------------------------------------------------------------------- #
# transform_msg_to_matrix
# --------------------------------------------------------------------------- #
def test_transform_identity():
    T = cfb.transform_msg_to_matrix(_Vec3(0, 0, 0), _Quat(0, 0, 0, 1))
    np.testing.assert_allclose(T, np.eye(4), atol=1e-9)


def test_transform_translation_only():
    T = cfb.transform_msg_to_matrix(_Vec3(1, 2, 3), _Quat(0, 0, 0, 1))
    np.testing.assert_allclose(T[:3, 3], [1, 2, 3], atol=1e-9)
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-9)


# --------------------------------------------------------------------------- #
# merge_colorings
# --------------------------------------------------------------------------- #
def test_merge_single_camera_passthrough():
    rgb = np.array([[10, 20, 30], [40, 50, 60]], dtype=np.uint8)
    seen = np.array([True, False])
    counts = np.array([1, 0], dtype=np.uint16)
    out, out_seen = cfb.merge_colorings([(rgb, seen, counts)], default_rgb=(7, 7, 7))
    np.testing.assert_array_equal(out[0], [10, 20, 30])
    np.testing.assert_array_equal(out[1], [7, 7, 7])  # unseen -> default
    assert out_seen.tolist() == [True, False]


def test_merge_count_weighted_blend():
    # Point 0: cam A (count 3) says red, cam B (count 1) says blue -> mostly red.
    a = (np.array([[200, 0, 0]], dtype=np.uint8), np.array([True]),
         np.array([3], dtype=np.uint16))
    b = (np.array([[0, 0, 200]], dtype=np.uint8), np.array([True]),
         np.array([1], dtype=np.uint16))
    out, seen = cfb.merge_colorings([a, b])
    assert seen[0]
    # (3*200 + 1*0)/4 = 150 red ; (3*0 + 1*200)/4 = 50 blue
    np.testing.assert_array_equal(out[0], [150, 0, 50])


def test_merge_union_of_coverage():
    # Cam A sees only point 0, cam B only point 1 -> both coloured after merge.
    a = (np.array([[100, 0, 0], [0, 0, 0]], dtype=np.uint8),
         np.array([True, False]), np.array([1, 0], dtype=np.uint16))
    b = (np.array([[0, 0, 0], [0, 100, 0]], dtype=np.uint8),
         np.array([False, True]), np.array([0, 1], dtype=np.uint16))
    out, seen = cfb.merge_colorings([a, b], default_rgb=(9, 9, 9))
    assert seen.tolist() == [True, True]
    np.testing.assert_array_equal(out[0], [100, 0, 0])
    np.testing.assert_array_equal(out[1], [0, 100, 0])


def test_merge_empty_raises():
    import pytest
    with pytest.raises(ValueError):
        cfb.merge_colorings([])


def test_transform_optical_rotation_is_a_valid_axis_permutation():
    # The standard camera_link<->optical quaternion (0.5,-0.5,0.5,-0.5) must
    # produce a proper rotation (orthonormal, det +1) that maps each unit axis
    # onto a signed unit axis. Lock in the concrete mapping as a regression.
    T = cfb.transform_msg_to_matrix(_Vec3(0, 0, 0), _Quat(0.5, -0.5, 0.5, -0.5))
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert abs(np.linalg.det(R) - 1.0) < 1e-9
    np.testing.assert_allclose(R @ np.array([1.0, 0.0, 0.0]), [0, -1, 0], atol=1e-9)
    np.testing.assert_allclose(R @ np.array([0.0, 0.0, 1.0]), [1, 0, 0], atol=1e-9)
