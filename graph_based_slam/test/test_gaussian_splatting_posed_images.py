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

"""Tests for the 3DGS post-process posed-image core (GPU/ROS-free)."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import posed_images

    return posed_images


pi = _load()


# --------------------------------------------------------------------------- #
# Quaternion helpers
# --------------------------------------------------------------------------- #
def test_quat_to_matrix_identity():
    R = pi.quat_to_matrix([0.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(R, np.eye(3), atol=1e-12)


def test_quat_to_matrix_90deg_about_z():
    # 90 deg about +z: x-axis maps to +y.
    q = [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]
    R = pi.quat_to_matrix(q)
    np.testing.assert_allclose(R @ np.array([1.0, 0.0, 0.0]),
                               [0.0, 1.0, 0.0], atol=1e-9)


def test_quat_normalize_rejects_zero():
    with pytest.raises(ValueError):
        pi.quat_normalize([0.0, 0.0, 0.0, 0.0])


def test_slerp_endpoints():
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    q1 = pi.quat_normalize([0.0, 0.0, 1.0, 1.0])
    np.testing.assert_allclose(pi.quat_slerp(q0, q1, 0.0), q0, atol=1e-12)
    np.testing.assert_allclose(pi.quat_slerp(q0, q1, 1.0), q1, atol=1e-12)


def test_slerp_halfway_is_unit_and_between():
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    q1 = pi.quat_normalize([0.0, 0.0, 1.0, 1.0])  # 90 deg about z
    mid = pi.quat_slerp(q0, q1, 0.5)
    assert abs(np.linalg.norm(mid) - 1.0) < 1e-9
    # Halfway of a 90 deg rotation is 45 deg about z.
    expected = pi.quat_normalize([0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)])
    np.testing.assert_allclose(mid, expected, atol=1e-9)


def test_slerp_takes_shorter_arc_on_sign_flip():
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    q1 = -q0  # same orientation, opposite hemisphere
    mid = pi.quat_slerp(q0, q1, 0.5)
    # Should stay at the identity orientation, not rotate 360 deg.
    R = pi.quat_to_matrix(mid)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-9)


# --------------------------------------------------------------------------- #
# TUM parsing
# --------------------------------------------------------------------------- #
def _write(tmp_path: Path, text: str) -> Path:
    p = tmp_path / 'traj.tum'
    p.write_text(text)
    return p


def test_read_tum_skips_comments_and_sorts(tmp_path):
    p = _write(
        tmp_path,
        '# header\n'
        '2.0 1 0 0 0 0 0 1\n'
        '\n'
        '1.0 0 0 0 0 0 0 1\n',
    )
    samples = pi.read_tum_trajectory(p)
    assert [s.stamp for s in samples] == [1.0, 2.0]
    np.testing.assert_allclose(samples[1].translation, [1.0, 0.0, 0.0])


def test_read_tum_rejects_bad_columns(tmp_path):
    p = _write(tmp_path, '1.0 0 0 0 0 0 1\n')  # 7 cols
    with pytest.raises(ValueError):
        pi.read_tum_trajectory(p)


def test_read_tum_rejects_non_numeric(tmp_path):
    p = _write(tmp_path, '1.0 x 0 0 0 0 0 1\n')
    with pytest.raises(ValueError):
        pi.read_tum_trajectory(p)


# --------------------------------------------------------------------------- #
# Pose interpolation
# --------------------------------------------------------------------------- #
def _linear_traj():
    return [
        pi.TrajectorySample(0.0, np.array([0.0, 0.0, 0.0]),
                            np.array([0.0, 0.0, 0.0, 1.0])),
        pi.TrajectorySample(2.0, np.array([2.0, 0.0, 0.0]),
                            np.array([0.0, 0.0, 0.0, 1.0])),
    ]


def test_interpolate_exact_endpoint():
    T = pi.interpolate_pose(_linear_traj(), 0.0)
    np.testing.assert_allclose(T[:3, 3], [0.0, 0.0, 0.0], atol=1e-12)


def test_interpolate_midpoint_translation():
    T = pi.interpolate_pose(_linear_traj(), 1.0)
    np.testing.assert_allclose(T[:3, 3], [1.0, 0.0, 0.0], atol=1e-12)


def test_interpolate_clamps_within_tolerance():
    T = pi.interpolate_pose(_linear_traj(), 2.05, max_extrapolation=0.1)
    np.testing.assert_allclose(T[:3, 3], [2.0, 0.0, 0.0], atol=1e-12)


def test_interpolate_raises_beyond_tolerance():
    with pytest.raises(ValueError):
        pi.interpolate_pose(_linear_traj(), 5.0, max_extrapolation=0.1)


def test_interpolate_empty_raises():
    with pytest.raises(ValueError):
        pi.interpolate_pose([], 0.0)


# --------------------------------------------------------------------------- #
# Extrinsic composition + OpenGL conversion + transforms.json
# --------------------------------------------------------------------------- #
def test_compose_world_T_camera():
    world_T_body = pi.make_transform([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    body_T_cam = pi.make_transform([0.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    T = pi.compose_world_T_camera(world_T_body, body_T_cam)
    np.testing.assert_allclose(T[:3, 3], [1.5, 2.0, 3.0], atol=1e-12)


def test_opengl_conversion_flips_y_and_z():
    posed = pi.PosedImage('a.png', np.eye(4), 0.0)
    c2w = posed.opengl_c2w()
    np.testing.assert_allclose(np.diag(c2w), [1.0, -1.0, -1.0, 1.0], atol=1e-12)


def test_intrinsics_from_camera_info():
    k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
    intr = pi.CameraIntrinsics.from_camera_info(640, 480, k, [0.1, -0.2, 0, 0, 0])
    assert (intr.fx, intr.fy, intr.cx, intr.cy) == (500.0, 500.0, 320.0, 240.0)
    assert intr.width == 640 and intr.height == 480


def test_intrinsics_rejects_bad_k():
    with pytest.raises(ValueError):
        pi.CameraIntrinsics.from_camera_info(640, 480, [1, 2, 3])


def test_write_transforms_roundtrip(tmp_path):
    intr = pi.CameraIntrinsics(640, 480, 500.0, 500.0, 320.0, 240.0,
                               distortion=(0.1, -0.2, 0.0, 0.0, 0.0))
    frames = [pi.PosedImage('images/0.png', np.eye(4), 1.0)]
    out = pi.write_transforms(tmp_path / 'transforms.json', intr, frames)
    doc = json.loads(Path(out).read_text())
    assert doc['w'] == 640 and doc['h'] == 480
    assert doc['fl_x'] == 500.0 and doc['cx'] == 320.0
    assert doc['k1'] == 0.1 and doc['k2'] == -0.2
    assert len(doc['frames']) == 1
    assert doc['frames'][0]['file_path'] == 'images/0.png'
    mat = np.array(doc['frames'][0]['transform_matrix'])
    np.testing.assert_allclose(np.diag(mat), [1.0, -1.0, -1.0, 1.0], atol=1e-12)
