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

"""Tests for the ego-pose -> 3DGS camera bridge (pure geometry, CPU)."""

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
    import scene_camera

    return scene_camera


sc = _load()


def _synthetic_c2w(n=20, length=10.0):
    """Cameras marching along world +x at height 1.5 (world up = +z, OpenGL)."""
    # OpenGL camera looking along -z would not march in x; build a c2w whose
    # forward (-col2) is +x: right=+y? Use right=[0,-1,0], up=[0,0,1], fwd=[1,0,0].
    right = np.array([0.0, -1.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    fwd = np.array([1.0, 0.0, 0.0])
    base = np.eye(4)
    base[:3, 0] = right
    base[:3, 1] = up
    base[:3, 2] = -fwd  # OpenGL: camera looks down -z, so col2 = -forward
    mats = []
    for x in np.linspace(0.0, length, n):
        m = base.copy()
        m[:3, 3] = [x, 0.0, 1.5]
        mats.append(m)
    return np.array(mats)


def test_derive_ground_frame_recovers_up_and_height():
    frame = sc.derive_ground_frame(_synthetic_c2w())
    assert np.allclose(np.abs(frame['up']), [0.0, 0.0, 1.0], atol=1e-6)
    assert frame['height'] == pytest.approx(1.5)
    # e1, e2, up orthonormal right-handed
    assert np.allclose(np.cross(frame['e1'], frame['e2']), frame['up'], atol=1e-6)


def test_corridor_xy_runs_along_plus_x():
    c2w = _synthetic_c2w(length=10.0)
    frame = sc.derive_ground_frame(c2w)
    xy = sc.corridor_xy(c2w, frame)
    assert xy.shape == (20, 2)
    assert xy[-1, 0] > xy[0, 0]  # travel toward +x
    assert np.abs(xy[:, 1]).max() < 1e-6  # straight: no lateral spread


def test_eye_target_up_places_camera_at_height_and_heading():
    c2w = _synthetic_c2w()
    frame = sc.derive_ground_frame(c2w)
    eye, target, up = sc.eye_target_up(2.0, 0.0, 0.0, frame)
    assert eye[2] == pytest.approx(1.5)  # eye at recorded height
    fwd = target - eye
    fwd = fwd / np.linalg.norm(fwd)
    assert np.dot(fwd, frame['e1']) == pytest.approx(1.0)  # yaw=0 -> along e1


def test_look_at_viewmat_is_world_to_camera():
    eye = np.array([0.0, 0.0, 0.0])
    target = np.array([1.0, 0.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    vm = sc.look_at_viewmat(eye, target, up)
    # the target maps in front of the camera (+z in OpenCV optical frame)
    p = vm @ np.array([1.0, 0.0, 0.0, 1.0])
    assert p[2] > 0.0
    # eye maps to the origin
    e = vm @ np.array([0.0, 0.0, 0.0, 1.0])
    assert np.allclose(e[:3], 0.0, atol=1e-9)


def test_azimuth_basis_orthonormal_and_in_plane():
    e_app, e_lat, up = sc.azimuth_basis(125.0, up_axis='y')
    assert np.allclose(up, [0.0, 1.0, 0.0])
    # e_app, e_lat lie in the ground plane (no up component)
    assert e_app[1] == pytest.approx(0.0)
    assert e_lat[1] == pytest.approx(0.0)
    # right-handed orthonormal: up x e_app == e_lat
    assert np.allclose(np.cross(up, e_app), e_lat, atol=1e-9)
    assert np.linalg.norm(e_app) == pytest.approx(1.0)


def test_azimuth_basis_matches_dolly_convention():
    # azimuth 0 about y-up points along +x (dolly: eye[x] += d*cos(a))
    e_app, _, _ = sc.azimuth_basis(0.0, up_axis='y')
    assert np.allclose(e_app, [1.0, 0.0, 0.0], atol=1e-9)


def test_target_orbit_eye_distance_and_elevation():
    e_app, e_lat, up = sc.azimuth_basis(90.0, up_axis='y')  # +z
    target = np.array([1.0, 0.0, -2.0])
    eye = sc.target_orbit_eye(10.0, 0.0, target=target, e_app=e_app,
                              e_lat=e_lat, up=up, elevation=-2.0)
    ground = eye - np.array([0.0, eye[1], 0.0])  # drop up component
    tground = target - np.array([0.0, target[1], 0.0])
    assert np.linalg.norm(ground - tground) == pytest.approx(10.0)  # range
    assert eye[1] == pytest.approx(target[1] - 2.0)  # elevation along up
