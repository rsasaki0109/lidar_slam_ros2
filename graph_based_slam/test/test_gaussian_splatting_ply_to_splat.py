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

"""Tests for the 3DGS .ply -> .splat web-viewer converter (ROS-free)."""

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
    import ply_to_splat
    import train_gsplat

    return ply_to_splat, train_gsplat


p2s, tg = _load()

SPLAT_BYTES = 32
SH_C0 = 0.28209479177387814


def _gaussian_set(n=6):
    means = np.arange(n * 3, dtype=float).reshape(n, 3) * 0.1
    scales_log = np.linspace(-3.0, -1.0, n * 3).reshape(n, 3)
    quats = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    opac = np.linspace(-2.0, 2.0, n)
    colors = np.linspace(0.05, 0.95, n * 3).reshape(n, 3)
    return means, scales_log, quats, opac, colors


def _decode(blob):
    n = len(blob) // SPLAT_BYTES
    assert len(blob) % SPLAT_BYTES == 0
    raw = np.frombuffer(blob, dtype=np.uint8).reshape(n, SPLAT_BYTES)
    pos = raw[:, 0:12].copy().view(np.float32).reshape(n, 3)
    scale = raw[:, 12:24].copy().view(np.float32).reshape(n, 3)
    rgba = raw[:, 24:28]
    rot = raw[:, 28:32]
    return pos, scale, rgba, rot


def test_blob_is_32_bytes_per_splat():
    means, scales_log, quats, opac, colors = _gaussian_set()
    blob = p2s.gaussians_to_splat_bytes(means, scales_log, quats, opac, colors)
    assert len(blob) == SPLAT_BYTES * means.shape[0]


def test_position_and_scale_roundtrip():
    means, scales_log, quats, opac, colors = _gaussian_set()
    blob = p2s.gaussians_to_splat_bytes(means, scales_log, quats, opac, colors)
    pos, scale, _, _ = _decode(blob)
    # Splats come back sorted by importance, so match as sets via positions.
    order = np.lexsort(pos.T)
    ref = np.lexsort(means.T.astype(np.float32))
    np.testing.assert_allclose(pos[order], means.astype(np.float32)[ref], atol=1e-5)
    np.testing.assert_allclose(
        scale[order], np.exp(scales_log).astype(np.float32)[ref], rtol=1e-5)


def test_color_is_band0_decoded_and_alpha_is_sigmoid():
    means, scales_log, quats, opac, colors = _gaussian_set(n=1)
    blob = p2s.gaussians_to_splat_bytes(means, scales_log, quats, opac, colors)
    _, _, rgba, _ = _decode(blob)
    np.testing.assert_allclose(
        rgba[0, :3] / 255.0, colors[0], atol=1.0 / 255 + 1e-6)
    expected_a = 1.0 / (1.0 + np.exp(-opac[0]))
    assert abs(rgba[0, 3] / 255.0 - expected_a) < 1.0 / 255 + 1e-6


def test_importance_order_is_descending():
    means, scales_log, quats, opac, colors = _gaussian_set()
    blob = p2s.gaussians_to_splat_bytes(means, scales_log, quats, opac, colors)
    _, scale, rgba, _ = _decode(blob)
    importance = scale.prod(axis=1) * (rgba[:, 3] / 255.0)
    assert np.all(np.diff(importance) <= 1e-6)


def test_max_points_keeps_top_n():
    means, scales_log, quats, opac, colors = _gaussian_set(n=10)
    full = p2s.gaussians_to_splat_bytes(means, scales_log, quats, opac, colors)
    capped = p2s.gaussians_to_splat_bytes(
        means, scales_log, quats, opac, colors, max_points=4)
    assert len(capped) == SPLAT_BYTES * 4
    # The capped blob is exactly the prefix of the full importance order.
    assert capped == full[:SPLAT_BYTES * 4]


def test_min_opacity_prunes_transparent():
    means, scales_log, quats, opac, colors = _gaussian_set(n=6)
    # opac spans -2..2 -> sigmoid spans ~0.12..0.88; cut below 0.5 drops half.
    blob = p2s.gaussians_to_splat_bytes(
        means, scales_log, quats, opac, colors, min_opacity=0.5)
    _, _, rgba, _ = _decode(blob)
    assert rgba.shape[0] == int(np.sum(1.0 / (1.0 + np.exp(-opac)) >= 0.5))
    assert np.all(rgba[:, 3] / 255.0 >= 0.5 - 1.0 / 255)


def test_max_scale_drops_giant_floaters():
    means, scales_log, quats, opac, colors = _gaussian_set(n=5)
    # Make the last Gaussian a giant floater (exp(scale) ~ e^2 = 7.4 m).
    scales_log = scales_log.copy()
    scales_log[-1] = 2.0
    blob = p2s.gaussians_to_splat_bytes(
        means, scales_log, quats, opac, colors, max_scale=1.0)
    _, scale, _, _ = _decode(blob)
    assert scale.shape[0] == 4
    assert np.all(scale.max(axis=1) <= 1.0 + 1e-5)


def test_min_opacity_pruning_everything_raises():
    means, scales_log, quats, opac, colors = _gaussian_set()
    with pytest.raises(ValueError):
        p2s.gaussians_to_splat_bytes(
            means, scales_log, quats, opac, colors, min_opacity=1.5)


def test_quaternion_is_normalised_and_packed():
    means, scales_log, _, opac, colors = _gaussian_set(n=1)
    quats = np.array([[2.0, 0.0, 0.0, 0.0]])  # unnormalised w-only
    blob = p2s.gaussians_to_splat_bytes(means, scales_log, quats, opac, colors)
    _, _, _, rot = _decode(blob)
    # (1,0,0,0) * 128 + 128 -> (255, 128, 128, 128) after clipping.
    np.testing.assert_array_equal(rot[0], np.array([255, 128, 128, 128]))


def test_ply_roundtrip_through_disk(tmp_path):
    means, scales_log, quats, opac, colors = _gaussian_set()
    ply = tg.export_ply(tmp_path / 'g.ply', means, scales_log, quats, opac, colors)
    blob = p2s.ply_to_splat_bytes(ply)
    assert len(blob) == SPLAT_BYTES * means.shape[0]
    direct = p2s.gaussians_to_splat_bytes(means, scales_log, quats, opac, colors)
    # f_dc round-trips through (rgb-0.5)/C0, so colours match to a quantum.
    _, _, rgba_disk, _ = _decode(blob)
    _, _, rgba_mem, _ = _decode(direct)
    assert np.max(np.abs(rgba_disk.astype(int) - rgba_mem.astype(int))) <= 1
