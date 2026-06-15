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

"""Tests for the actor-sprite extraction pure helpers (CPU, no detector)."""

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
    import make_actor_sprite

    return make_actor_sprite


ms = _load()


def test_compose_rgba_alpha_follows_mask():
    rgb = np.full((4, 4, 3), 100, dtype=np.uint8)
    mask = np.zeros((4, 4))
    mask[1:3, 1:3] = 1.0
    rgba = ms.compose_rgba(rgb, mask)
    assert rgba.shape == (4, 4, 4) and rgba.dtype == np.uint8
    assert rgba[1, 1, 3] == 255 and rgba[0, 0, 3] == 0
    assert np.all(rgba[..., :3] == 100)  # colour untouched


def test_compose_rgba_soft_mask_threshold():
    rgb = np.zeros((1, 3, 3), dtype=np.uint8)
    mask = np.array([[0.2, 0.5, 0.9]])
    rgba = ms.compose_rgba(rgb, mask, mask_thresh=0.5)
    assert list(rgba[0, :, 3]) == [0, 255, 255]


def test_compose_rgba_rejects_shape_mismatch():
    with pytest.raises(ValueError):
        ms.compose_rgba(np.zeros((4, 4, 3)), np.zeros((3, 3)))


def test_tight_crop_removes_transparent_margin():
    rgba = np.zeros((10, 10, 4), dtype=np.uint8)
    rgba[3:6, 4:8, 3] = 255  # opaque region 3 tall x 4 wide
    cropped = ms.tight_crop_rgba(rgba)
    assert cropped.shape == (3, 4, 4)


def test_tight_crop_raises_when_empty():
    with pytest.raises(ValueError):
        ms.tight_crop_rgba(np.zeros((5, 5, 4), dtype=np.uint8))


def test_alpha_coverage_counts_opaque_fraction():
    rgba = np.zeros((10, 10, 4), dtype=np.uint8)
    rgba[:5, :, 3] = 255  # half opaque
    assert ms.alpha_coverage(rgba) == pytest.approx(0.5)


def test_pick_portrait_instance_prefers_tallest_portrait():
    boxes = [
        [0, 0, 100, 50],     # person, landscape (wide) -> rejected
        [0, 0, 30, 60],      # person, portrait, height 60
        [0, 0, 40, 90],      # person, portrait, height 90 -> winner
        [0, 0, 20, 200],     # wrong class
    ]
    classes = [0, 0, 0, 2]
    assert ms.pick_portrait_instance(boxes, classes, 0) == 2


def test_pick_portrait_instance_none_when_no_match():
    boxes = [[0, 0, 100, 40]]  # only a landscape person
    assert ms.pick_portrait_instance(boxes, [0], 0) is None
    # wrong class only
    assert ms.pick_portrait_instance([[0, 0, 10, 50]], [5], 0) is None
