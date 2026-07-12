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


def test_select_synced_time_reselects_cloud_nearest_slower_image():
    clouds = [0, 100, 200, 300, 400]
    images = [40, 340]
    # frac=.4 initially picks cloud 200 -> image 340, then cloud 300 is closer.
    assert cfb.select_synced_time(clouds, images, 0.4) == (300, 340)


def test_select_synced_time_searches_neighbour_images_for_smallest_offset():
    clouds = [0, 100, 200, 300, 400]
    images = [40, 170, 305]
    # The closest image to target cloud 200 is 170 (30 ms), but neighbour 305
    # has a 5 ms pairing with cloud 300 and therefore gives sharper colour.
    assert cfb.select_synced_time(clouds, images, 0.4, search_radius=1) == (300, 305)


def test_select_synced_time_clamps_fraction_and_rejects_empty():
    import pytest
    assert cfb.select_synced_time([100, 200], [180], -1) == (200, 180)
    assert cfb.select_synced_time([100, 200], [180], 2) == (200, 180)
    with pytest.raises(ValueError, match='point-cloud'):
        cfb.select_synced_time([], [1], 0.5)
    with pytest.raises(ValueError, match='image'):
        cfb.select_synced_time([1], [], 0.5)


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
# manual extrinsic input (bags without TF)
# --------------------------------------------------------------------------- #
def test_extrinsic_matrix_from_cli_values_normalizes_quaternion():
    T = cfb.extrinsic_matrix([1, 2, 3, 0, 0, 0, 2])
    np.testing.assert_allclose(T, np.array([
        [1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1],
    ]), atol=1e-9)


def test_extrinsic_matrix_from_json_object(tmp_path):
    path = tmp_path / 'extrinsic.json'
    path.write_text(
        '{"translation": [4, 5, 6], "rotation_xyzw": [0, 0, 0, 1]}')
    T = cfb.extrinsic_matrix(path=path)
    np.testing.assert_allclose(T[:3, 3], [4, 5, 6], atol=1e-9)
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-9)


def test_extrinsic_matrix_from_json_list(tmp_path):
    path = tmp_path / 'extrinsic.json'
    path.write_text('[1, 2, 3, 0, 0, 0, 1]')
    np.testing.assert_allclose(
        cfb.extrinsic_matrix(path=path),
        cfb.extrinsic_matrix([1, 2, 3, 0, 0, 0, 1]))


def test_extrinsic_matrix_inverts_official_vlcal_result(tmp_path):
    path = tmp_path / 'calib.json'
    path.write_text(
        '{"results": {"T_lidar_camera": [1, 2, 3, 0, 0, 0, 1]}}')
    T = cfb.extrinsic_matrix(path=path)
    np.testing.assert_allclose(T[:3, 3], [-1, -2, -3], atol=1e-9)
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-9)


def test_extrinsic_matrix_none_keeps_tf_mode():
    assert cfb.extrinsic_matrix() is None


def test_extrinsic_matrix_rejects_bad_inputs(tmp_path):
    import pytest
    with pytest.raises(ValueError, match='7 values'):
        cfb.extrinsic_matrix([1, 2, 3])
    with pytest.raises(ValueError, match='non-zero'):
        cfb.extrinsic_matrix([0, 0, 0, 0, 0, 0, 0])
    path = tmp_path / 'extrinsic.json'
    path.write_text('{"translation": [1, 2, 3]}')
    with pytest.raises(ValueError, match='rotation_xyzw'):
        cfb.extrinsic_matrix(path=path)


# --------------------------------------------------------------------------- #
# diagnostic projection overlay geometry
# --------------------------------------------------------------------------- #
def test_projection_diagnostics_marks_occluded_point():
    K = np.array([[100, 0, 50], [0, 100, 50], [0, 0, 1]], dtype=float)
    points = np.array([
        [0, 0, 2],       # nearest surface at the principal point
        [0, 0, 8],       # same pixel, hidden behind the near surface
        [100, 0, 1],     # outside the image
    ], dtype=float)
    result = cfb.projection_diagnostics(
        points, np.eye(4), K, 100, 100, zbuf_bin=4, depth_tol=0.15)
    assert result['indices'].tolist() == [0, 1]
    np.testing.assert_allclose(result['u'], [50, 50])
    np.testing.assert_allclose(result['v'], [50, 50])
    assert result['visible'].tolist() == [True, False]


def test_projection_diagnostics_handles_no_in_frame_points():
    K = np.eye(3)
    result = cfb.projection_diagnostics(
        np.array([[0, 0, -1]], dtype=float), np.eye(4), K, 10, 10)
    assert result['indices'].size == 0
    assert result['visible'].size == 0


def test_projection_diagnostics_rejects_bad_zbuffer_bin():
    import pytest
    with pytest.raises(ValueError, match='zbuf_bin'):
        cfb.projection_diagnostics(
            np.zeros((1, 3)), np.eye(4), np.eye(3), 10, 10, zbuf_bin=0)


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
