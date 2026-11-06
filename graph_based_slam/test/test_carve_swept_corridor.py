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

"""Tests for the swept-corridor carving of camera-bearer ghosts."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'colored_map'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import carve_swept_corridor
    import pointcloud_io
    import posed_images

    return carve_swept_corridor, pointcloud_io, posed_images


csc, pcio, pi = _load()


def _points():
    return np.array([
        [2.0, 0.2, 1.3],   # companion standing on the route: carved
        [2.0, 0.0, 0.0],   # ground 1.4 m below the camera: kept
        [2.0, 0.8, 1.3],   # hedge beside the route: kept
        [2.0, 0.0, 2.2],   # branch overhead: kept
        [9.0, 0.0, 1.4],   # beyond the end of the walk: kept
    ])


def test_swept_corridor_mask_keeps_ground_sides_overhead_and_ends():
    pytest.importorskip('scipy.spatial')
    path = np.array([[0.0, 0.0, 1.4], [5.0, 0.0, 1.4]])
    mask = pcio.swept_corridor_mask(_points(), path, np.array([0.0, 0.0, 1.0]))
    assert mask.tolist() == [True, False, False, False, False]


def test_swept_corridor_mask_follows_a_tilted_up_vector():
    pytest.importorskip('scipy.spatial')
    angle = np.deg2rad(20.0)
    tilt = np.array([[1.0, 0.0, 0.0],
                     [0.0, np.cos(angle), -np.sin(angle)],
                     [0.0, np.sin(angle), np.cos(angle)]])
    path = np.array([[0.0, 0.0, 1.4], [5.0, 0.0, 1.4]]) @ tilt.T
    mask = pcio.swept_corridor_mask(_points() @ tilt.T, path, tilt @ [0, 0, 1.0])
    assert mask.tolist() == [True, False, False, False, False]
    with pytest.raises(ValueError):
        pcio.swept_corridor_mask(_points(), path, np.zeros(3))
    with pytest.raises(ValueError):
        pcio.swept_corridor_mask(_points(), np.zeros((0, 3)), tilt @ [0, 0, 1.0])


def test_cli_removes_corridor_points_and_reports(tmp_path):
    pytest.importorskip('scipy.spatial')
    rotation = pytest.importorskip('scipy.spatial.transform').Rotation
    # OpenCV camera looking along +x with its down axis along -z.
    quat = rotation.from_matrix(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]).as_quat()
    intr = pi.CameraIntrinsics(32, 24, 30.0, 30.0, 16.0, 12.0)
    frames = [pi.PosedImage(f'images/{i}.png',
                            pi.make_transform([float(i), 0.0, 1.4], quat),
                            float(i)) for i in range(6)]
    pi.write_transforms(tmp_path / 'transforms.json', intr, frames)
    rgb = np.tile(np.array([[200, 10, 10]], dtype=np.uint8), (5, 1))
    pcio.write_ply(tmp_path / 'map.ply', _points(), rgb)
    assert csc.main(['--input', str(tmp_path / 'map.ply'),
                     '--transforms', str(tmp_path / 'transforms.json'),
                     '--out', str(tmp_path / 'carved.ply'),
                     '--report', str(tmp_path / 'report.json')]) == 0
    xyz, out_rgb = pcio.read_ply_xyz(tmp_path / 'carved.ply')
    assert len(xyz) == 4 and out_rgb is not None
    report = json.loads((tmp_path / 'report.json').read_text())
    assert report['removed'] == 1
    assert sum(report['height_histogram']['counts']) >= 2


def _ground(step=0.05, hole=None, z_fn=None):
    """Ground grid around a path along x (camera 1.4 m above it), optional square hole."""
    u, v = np.meshgrid(np.arange(-1.0, 6.0, step), np.arange(-3.0, 3.0, step), indexing='ij')
    u, v = u.ravel(), v.ravel()
    keep = np.ones(len(u), dtype=bool)
    if hole is not None:
        (u0, u1), (v0, v1) = hole
        keep = ~((u >= u0) & (u < u1) & (v >= v0) & (v < v1))
    z = np.zeros_like(u) if z_fn is None else z_fn(u, v)
    return np.stack([u, v, z], axis=1)[keep]


_PATH = np.array([[x, 0.0, 1.4] for x in np.arange(0.0, 5.01, 0.25)])
_UP = np.array([0.0, 0.0, 1.0])


def test_fill_ground_holes_fills_an_enclosed_hole_on_its_plane():
    pytest.importorskip('scipy.ndimage')
    slope = (lambda u, v: 0.05 * u + 0.02 * v)
    points = _ground(hole=((2.0, 2.6), (0.6, 1.2)), z_fn=slope)
    added, stats = csc.fill_ground_holes(points, _PATH, _UP)
    assert stats['filled'] == 1 and stats['points_added'] == len(added) > 0
    assert 0.3 <= stats['filled_area_m2'] <= 0.5
    assert np.all((added[:, 0] > 1.85) & (added[:, 0] < 2.75))
    assert np.all((added[:, 1] > 0.45) & (added[:, 1] < 1.35))
    assert np.max(np.abs(added[:, 2] - slope(added[:, 0], added[:, 1]))) < 0.01
    # deterministic
    again, _ = csc.fill_ground_holes(points, _PATH, _UP)
    assert np.array_equal(added, again)


def test_fill_ground_holes_leaves_steps_large_and_distant_holes():
    pytest.importorskip('scipy.ndimage')
    step = (lambda u, v: np.where(v > 0.9, 0.3, 0.0))
    _, stats = csc.fill_ground_holes(_ground(hole=((2.0, 2.6), (0.6, 1.2)), z_fn=step),
                                     _PATH, _UP)
    assert stats['filled'] == 0 and stats['not_flat'] == 1
    _, stats = csc.fill_ground_holes(_ground(hole=((1.0, 3.0), (0.2, 1.6))), _PATH, _UP,
                                     max_area=1.0, close_radius=1.0)
    assert stats['filled'] == 0 and stats['too_large'] == 1
    _, stats = csc.fill_ground_holes(_ground(hole=((2.0, 2.6), (2.3, 2.7))), _PATH, _UP,
                                     reach=1.0)
    assert stats['filled'] == 0


def test_cli_fill_ground_holes_reports(tmp_path):
    pytest.importorskip('scipy.ndimage')
    rotation = pytest.importorskip('scipy.spatial.transform').Rotation
    quat = rotation.from_matrix(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]).as_quat()
    intr = pi.CameraIntrinsics(32, 24, 30.0, 30.0, 16.0, 12.0)
    frames = [pi.PosedImage(f'images/{i}.png',
                            pi.make_transform([float(i), 0.0, 1.4], quat),
                            float(i)) for i in range(6)]
    pi.write_transforms(tmp_path / 'transforms.json', intr, frames)
    points = _ground(hole=((2.0, 2.6), (0.6, 1.2)))
    pcio.write_ply(tmp_path / 'map.ply', points, None)
    assert csc.main(['--input', str(tmp_path / 'map.ply'),
                     '--transforms', str(tmp_path / 'transforms.json'),
                     '--out', str(tmp_path / 'filled.ply'), '--fill-ground-holes',
                     '--report', str(tmp_path / 'report.json')]) == 0
    report = json.loads((tmp_path / 'report.json').read_text())
    assert report['ground_hole_fill']['filled'] == 1
    xyz, _ = pcio.read_ply_xyz(tmp_path / 'filled.ply')
    assert len(xyz) == len(points) - report['removed'] + report['ground_hole_fill']['points_added']
