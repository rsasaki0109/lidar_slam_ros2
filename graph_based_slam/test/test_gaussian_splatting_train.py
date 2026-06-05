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

"""Tests for the 3DGS gsplat trainer pure helpers (no torch/CUDA needed)."""

from __future__ import annotations

from pathlib import Path
import struct
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import train_gsplat

    return train_gsplat


tg = _load()
pi = tg.pi


# --------------------------------------------------------------------------- #
# looks_at_poses
# --------------------------------------------------------------------------- #
def test_looks_at_poses_count_and_radius():
    poses = tg.looks_at_poses(radius=3.0, count=8)
    assert len(poses) == 8
    for c2w in poses:
        assert abs(np.linalg.norm(c2w[:3, 3]) - 3.0) < 1e-6


def test_looks_at_poses_forward_points_at_origin():
    for c2w in tg.looks_at_poses(radius=2.0, count=6, height=0.5):
        eye = c2w[:3, 3]
        forward = c2w[:3, 2]  # OpenCV +z is forward
        to_origin = -eye / np.linalg.norm(eye)
        assert float(np.dot(forward, to_origin)) > 0.9


def test_looks_at_poses_orthonormal_rotation():
    c2w = tg.looks_at_poses(radius=2.0, count=4)[0]
    R = c2w[:3, :3]
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-6)


# --------------------------------------------------------------------------- #
# load_transforms round-trips write_transforms and recovers OpenCV w2c
# --------------------------------------------------------------------------- #
def test_load_transforms_recovers_viewmat(tmp_path):
    intr = pi.CameraIntrinsics(64, 48, 50.0, 50.0, 32.0, 24.0)
    c2w_cv = pi.make_transform([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    (tmp_path / 'images').mkdir()
    (tmp_path / 'images' / '0.png').write_bytes(b'stub')
    frames = [pi.PosedImage('images/0.png', c2w_cv, 0.0)]
    pi.write_transforms(tmp_path / 'transforms.json', intr, frames)

    ds = tg.load_transforms(tmp_path / 'transforms.json')
    assert ds['width'] == 64 and ds['height'] == 48
    np.testing.assert_allclose(ds['K'], [[50, 0, 32], [0, 50, 24], [0, 0, 1]],
                               atol=1e-9)
    # viewmat should be the inverse of the original OpenCV c2w.
    np.testing.assert_allclose(ds['viewmats'][0], np.linalg.inv(c2w_cv), atol=1e-9)
    assert ds['image_paths'][0].name == '0.png'


# --------------------------------------------------------------------------- #
# export_ply
# --------------------------------------------------------------------------- #
def test_export_ply_header_and_roundtrip(tmp_path):
    means = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    scales_log = np.zeros((2, 3))
    quats = np.tile([1.0, 0.0, 0.0, 0.0], (2, 1))
    opac = np.array([0.0, 0.0])
    colors = np.array([[0.5, 0.5, 0.5], [1.0, 0.0, 0.0]])
    out = tg.export_ply(tmp_path / 'g.ply', means, scales_log, quats, opac, colors)

    raw = Path(out).read_bytes()
    header_end = raw.index(b'end_header\n') + len(b'end_header\n')
    header = raw[:header_end].decode('ascii')
    assert 'element vertex 2' in header
    assert 'property float f_dc_0' in header and 'property float rot_3' in header

    n_props = header.count('property float ')
    body = raw[header_end:]
    vals = struct.unpack('<' + 'f' * (2 * n_props), body)
    row0 = vals[:n_props]
    # x, y, z are the first three properties.
    np.testing.assert_allclose(row0[:3], [1.0, 2.0, 3.0], atol=1e-6)
    # grey (0.5) -> f_dc 0 for all three colour channels (indices 6,7,8).
    np.testing.assert_allclose(row0[6:9], [0.0, 0.0, 0.0], atol=1e-6)


def test_export_ply_vertex_count(tmp_path):
    n = 5
    out = tg.export_ply(
        tmp_path / 'g.ply', np.zeros((n, 3)), np.zeros((n, 3)),
        np.tile([1.0, 0, 0, 0], (n, 1)), np.zeros(n), np.full((n, 3), 0.5),
    )
    assert f'element vertex {n}'.encode() in Path(out).read_bytes()
