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

"""Tests for colour scoring against a reference cloud and padded PCD input."""

from __future__ import annotations

import json
from pathlib import Path
import struct
import sys

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'


def _load():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    import evaluate_colored_map_reference

    return evaluate_colored_map_reference


ref = _load()
pcio = ref.pcio


def test_colour_errors_are_infinite_psnr_for_identical_colours():
    rgb = np.array([[10, 200, 30], [250, 0, 128]], dtype=np.uint8)
    result = ref.colour_errors(rgb, rgb)
    assert result['rgb_l2_median'] == 0.0 and result['psnr_yuv'] > 100.0


def test_colour_errors_weight_luma_six_to_one():
    grey = np.full((4, 3), 100.0)
    brighter = grey + 10.0     # pure luma offset: Cb and Cr unchanged
    result = ref.colour_errors(brighter, grey)
    assert result['psnr_y'] == pytest.approx(10 * np.log10(255 ** 2 / 100.0))
    assert result['psnr_cb'] > 100.0
    assert result['psnr_yuv'] == pytest.approx(
        (6 * result['psnr_y'] + result['psnr_cb'] + result['psnr_cr']) / 8)


def test_affine_fit_removes_gain_and_bias():
    rng = np.random.default_rng(0)
    source = rng.uniform(20, 200, size=(200, 3))
    target = source * [0.8, 1.1, 0.9] + [5.0, -3.0, 10.0]
    params = ref.fit_channel_affine(source, target)
    np.testing.assert_allclose(params, [[0.8, 5.0], [1.1, -3.0], [0.9, 10.0]],
                               atol=1e-9)
    np.testing.assert_allclose(ref.apply_channel_affine(source, params),
                               np.clip(target, 0, 255), atol=1e-9)


def test_evaluate_matches_both_directions_and_skips_uncoloured():
    pytest.importorskip('scipy.spatial')
    reference_xyz = np.array([[0.0, 0, 0], [1.0, 0, 0], [5.0, 0, 0]])
    reference_rgb = np.array([[100, 100, 100], [200, 50, 50], [0, 0, 255]],
                             dtype=np.uint8)
    test_xyz = np.array([[0.01, 0, 0], [1.02, 0, 0], [1.0, 0.01, 0],
                         [9.0, 0, 0]])
    test_rgb = np.array([[110, 110, 110], [200, 50, 50], [128, 128, 128],
                         [10, 10, 10]], dtype=np.uint8)
    report = ref.evaluate(test_xyz, test_rgb, reference_xyz, reference_rgb,
                          max_distance=0.05)
    assert report['map_uncoloured'] == 1
    assert report['map_matched_fraction'] == pytest.approx(2 / 3)
    assert report['reference_matched_points'] == 2
    assert report['raw']['map_to_reference']['pairs'] == 2
    assert report['affine']['symmetric_psnr_yuv'] >= \
        report['raw']['symmetric_psnr_yuv'] - 1e-9
    with pytest.raises(ValueError):
        ref.evaluate(test_xyz + 100.0, test_rgb, reference_xyz, reference_rgb)


def test_cli_writes_report(tmp_path):
    pytest.importorskip('scipy.spatial')
    xyz = np.array([[0.0, 0, 0], [1.0, 0, 0]])
    rgb = np.array([[10, 20, 30], [40, 50, 60]], dtype=np.uint8)
    pcio.write_ply(tmp_path / 'map.ply', xyz, rgb)
    pcio.write_ply(tmp_path / 'tls.ply', xyz, rgb)
    assert ref.main(['--pointcloud', str(tmp_path / 'map.ply'),
                     '--reference', str(tmp_path / 'tls.ply'),
                     '--out', str(tmp_path / 'report.json')]) == 0
    report = json.loads((tmp_path / 'report.json').read_text())
    assert report['raw']['map_to_reference']['rgb_l2_median'] == 0.0


def test_read_pcd_skips_padding_fields_with_count(tmp_path):
    header = ('# .PCD v0.7\nVERSION 0.7\n'
              'FIELDS C2C Intensity rgb x y z _\nSIZE 4 4 4 4 4 4 1\n'
              'TYPE F F F F F F U\nCOUNT 1 1 1 1 1 1 4\nWIDTH 2\nHEIGHT 1\n'
              'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\nDATA binary\n')
    packed = [(10 << 16) | (20 << 8) | 30, (200 << 16) | (100 << 8) | 50]
    body = b''
    for index, value in enumerate(packed):
        rgb = struct.unpack('<f', struct.pack('<I', value))[0]
        body += struct.pack('<6f', 0.5, 7.0, rgb, float(index), 2.0, 3.0)
        body += b'\x00\x01\x02\x03'
    path = tmp_path / 'tls.pcd'
    path.write_bytes(header.encode('ascii') + body)
    xyz, rgb = pcio.read_pcd_xyz(path)
    np.testing.assert_allclose(xyz, [[0, 2, 3], [1, 2, 3]])
    np.testing.assert_array_equal(rgb, [[10, 20, 30], [200, 100, 50]])
    ascii_path = tmp_path / 'tls_ascii.pcd'
    ascii_path.write_text(header.replace('binary', 'ascii') +
                          '0.5 7 0 4 5 6 0 0 0 0\n0.5 7 0 7 8 9 1 1 1 1\n')
    xyz, _ = pcio.read_pcd_xyz(ascii_path)
    np.testing.assert_allclose(xyz, [[4, 5, 6], [7, 8, 9]])


def _prepare():
    import prepare_oxford_spires_colour_eval

    return prepare_oxford_spires_colour_eval


def test_spires_stamps_parse_image_and_cloud_names():
    prep = _prepare()
    assert prep.stamp_from_name('cam0/1710338098.943278436.jpg') == \
        pytest.approx(1710338098.943278436)
    assert prep.stamp_from_name('cloud_1710338098_043290436.pcd') == \
        pytest.approx(1710338098.043290436)
    with pytest.raises(ValueError):
        prep.stamp_from_name('cloud.pcd')


def test_spires_camera_pose_composes_base_lidar_and_extrinsic():
    prep = _prepare()
    pi = prep.pi
    trajectory = [pi.TrajectorySample(0.0, np.array([1.0, 2.0, 3.0]),
                                      np.array([0.0, 0.0, 0.0, 1.0])),
                  pi.TrajectorySample(1.0, np.array([1.0, 2.0, 3.0]),
                                      np.array([0.0, 0.0, 0.0, 1.0]))]
    camera_T_lidar = np.eye(4)
    camera_T_lidar[:3, 3] = [0.0, -0.08, -0.05]
    pose = prep.world_T_camera(trajectory, 0.5, camera_T_lidar, 0.05)
    # The camera sits at lidar - (0, -0.08, -0.05); the lidar is 0.124 m
    # above base and turned 180 degrees about z.
    lidar_origin = np.array([1.0, 2.0, 3.124])
    expected = lidar_origin + np.diag([-1.0, -1.0, 1.0]) @ [0.0, 0.08, 0.05]
    np.testing.assert_allclose(pose[:3, 3], expected, atol=1e-12)
