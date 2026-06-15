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

"""Tests for the detect-in-scene pure helpers (CPU, no torch/gsplat/detector)."""

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
    import detect_in_scene

    return detect_in_scene


dis = _load()


def _center_of(vm):
    return -vm[:3, :3].T @ vm[:3, 3]


def test_look_at_viewmat_places_camera_and_faces_target():
    vm = dis.look_at_viewmat([0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    assert np.allclose(_center_of(vm), [0.0, 0.0, -5.0])
    # target maps in front of the camera (positive z in OpenCV camera frame)
    cam = vm @ [0.0, 0.0, 0.0, 1.0]
    assert cam[2] > 0.0


def test_orbit_viewmats_radius_and_count():
    target = np.array([1.0, 2.0, 3.0])
    vms = dis.orbit_viewmats(target, 10.0, 0.0, 8, up_axis='y')
    assert vms.shape == (8, 4, 4)
    for vm in vms:
        c = _center_of(vm)
        # horizontal (x,z) distance to target equals the radius; y matches target
        assert np.isclose(np.hypot(c[0] - 1.0, c[2] - 3.0), 10.0)
        assert np.isclose(c[1], 2.0)


def test_orbit_viewmats_full_circle_drops_duplicate_end():
    a = dis.orbit_viewmats([0, 0, 0], 5.0, 0.0, 4, up_axis='y', arc_deg=360.0)
    # 0 and 360 deg would coincide; endpoint dropped, so first != last
    assert not np.allclose(_center_of(a[0]), _center_of(a[-1]))


def test_orbit_viewmats_elevation_offsets_up_axis():
    vms = dis.orbit_viewmats([0, 0, 0], 5.0, 3.0, 4, up_axis='y')
    assert np.allclose([_center_of(vm)[1] for vm in vms], 3.0)


def test_orbit_viewmats_rejects_bad_inputs():
    with pytest.raises(ValueError):
        dis.orbit_viewmats([0, 0, 0], 5.0, 0.0, 0)
    with pytest.raises(ValueError):
        dis.orbit_viewmats([0, 0, 0], 5.0, 0.0, 4, up_axis='w')


def test_subsample_caps_length_and_keeps_endpoints():
    pts = np.arange(30).reshape(10, 3)
    out = dis.subsample(pts, 4)
    assert out.shape == (4, 3)
    assert np.array_equal(out[0], pts[0]) and np.array_equal(out[-1], pts[-1])


def test_subsample_returns_all_when_under_limit():
    pts = np.arange(9).reshape(3, 3)
    assert np.array_equal(dis.subsample(pts, 100), pts)


def test_score_detection_hit_when_same_class_overlaps():
    dets = [{'cls': 7, 'box': [10, 10, 50, 50], 'conf': 0.8}]
    best, hit = dis.score_detection(dets, [10, 10, 50, 50], 7)
    assert hit and best == pytest.approx(1.0)


def test_score_detection_class_must_match():
    dets = [{'cls': 2, 'box': [10, 10, 50, 50], 'conf': 0.8}]  # car, not truck
    best, hit = dis.score_detection(dets, [10, 10, 50, 50], 7)
    assert not hit and best == 0.0


def test_score_detection_no_gt_is_miss():
    dets = [{'cls': 7, 'box': [10, 10, 50, 50], 'conf': 0.8}]
    assert dis.score_detection(dets, None, 7) == (0.0, False)
