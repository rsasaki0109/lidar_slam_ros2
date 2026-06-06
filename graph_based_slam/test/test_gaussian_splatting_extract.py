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

"""Tests for the 3DGS posed-image bag extractor pure helpers (ROS-free)."""

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
    import extract_posed_images

    return extract_posed_images


ex = _load()
pi = ex.pi


# --------------------------------------------------------------------------- #
# Stamp conversion
# --------------------------------------------------------------------------- #
def test_ros_stamp_to_seconds():
    assert ex.ros_stamp_to_seconds(5, 500_000_000) == pytest.approx(5.5)
    assert ex.ros_stamp_to_seconds(0, 0) == 0.0


def test_compute_clock_offset_aligns_sensor_clocks():
    # Camera and LiDAR on independent uptime clocks (~21.9s skew, koide case).
    off = ex.compute_clock_offset(545.614, 1678336967.407,
                                  566.800, 1678336966.714)
    assert off == pytest.approx(21.879, abs=1e-3)
    # Adding the offset maps the camera stamp onto the reference clock.
    assert 545.614 + off == pytest.approx(566.800 + (1678336967.407 - 1678336966.714),
                                          abs=1e-3)


def test_compute_clock_offset_zero_when_synced():
    assert ex.compute_clock_offset(10.0, 100.0, 10.0, 100.0) == pytest.approx(0.0)


# --------------------------------------------------------------------------- #
# Raw image decoding (cv_bridge-free)
# --------------------------------------------------------------------------- #
def test_decode_bgr8_to_rgb():
    # one pixel, BGR bytes (10, 20, 30) -> RGB (30, 20, 10)
    out = ex.decode_image('bgr8', 1, 1, 3, bytes([10, 20, 30]))
    assert out.shape == (1, 1, 3)
    np.testing.assert_array_equal(out[0, 0], [30, 20, 10])


def test_decode_rgb8_passthrough():
    out = ex.decode_image('rgb8', 1, 2, 6, bytes([1, 2, 3, 4, 5, 6]))
    assert out.shape == (1, 2, 3)
    np.testing.assert_array_equal(out[0, 0], [1, 2, 3])
    np.testing.assert_array_equal(out[0, 1], [4, 5, 6])


def test_decode_handles_row_padding_via_step():
    # 1x2 rgb8 with step 8 (2 padding bytes per row)
    data = bytes([1, 2, 3, 4, 5, 6, 0, 0])
    out = ex.decode_image('rgb8', 1, 2, 8, data)
    np.testing.assert_array_equal(out[0, 1], [4, 5, 6])


def test_decode_mono8_is_2d():
    out = ex.decode_image('mono8', 2, 2, 2, bytes([1, 2, 3, 4]))
    assert out.shape == (2, 2)
    np.testing.assert_array_equal(out, [[1, 2], [3, 4]])


def test_decode_rejects_unknown_encoding():
    with pytest.raises(ValueError):
        ex.decode_image('yuv422', 1, 1, 2, bytes([0, 0]))


def test_decode_rejects_short_payload():
    with pytest.raises(ValueError):
        ex.decode_image('rgb8', 2, 2, 6, bytes([1, 2, 3]))


# --------------------------------------------------------------------------- #
# Extrinsic parsing
# --------------------------------------------------------------------------- #
def test_parse_extrinsic_translation_rotation():
    T = ex.parse_extrinsic_dict(
        {'translation': [1.0, 2.0, 3.0], 'rotation_xyzw': [0.0, 0.0, 0.0, 1.0]}
    )
    np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0], atol=1e-12)
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-12)


def test_parse_extrinsic_matrix():
    m = np.eye(4)
    m[0, 3] = 7.0
    T = ex.parse_extrinsic_dict({'matrix': m.tolist()})
    np.testing.assert_allclose(T, m, atol=1e-12)


def test_parse_extrinsic_bad_matrix_shape():
    with pytest.raises(ValueError):
        ex.parse_extrinsic_dict({'matrix': [[1, 0], [0, 1]]})


def test_parse_extrinsic_missing_fields():
    with pytest.raises(ValueError):
        ex.parse_extrinsic_dict({'translation': [0, 0, 0]})


def test_load_extrinsic_identity_when_none():
    np.testing.assert_allclose(ex.load_extrinsic(None), np.eye(4), atol=1e-12)


# --------------------------------------------------------------------------- #
# Intrinsics YAML (NTU VIRAL / Kalibr style)
# --------------------------------------------------------------------------- #
def test_load_intrinsics_yaml_ntu_style(tmp_path):
    text = (
        '%YAML:1.0\n'
        'model_type:   PINHOLE\n'
        'image_width:  752\n'
        'image_height: 480\n'
        'distortion_parameters:\n'
        '   k1: -0.288105\n   k2: 0.074578\n   p1: 0.000778\n   p2: -0.000228\n'
        'projection_parameters:\n'
        '   fx: 425.0258\n   fy: 426.7976\n   cx: 386.0151\n   cy: 241.9130\n'
    )
    p = tmp_path / 'camera_left.yaml'
    p.write_text(text)
    intr = ex.load_intrinsics_yaml(p)
    assert intr.width == 752 and intr.height == 480
    assert intr.fx == pytest.approx(425.0258)
    assert intr.cy == pytest.approx(241.9130)
    assert intr.distortion[0] == pytest.approx(-0.288105)
    assert intr.distortion[2] == pytest.approx(0.000778)


def test_load_intrinsics_yaml_missing_field(tmp_path):
    p = tmp_path / 'bad.yaml'
    p.write_text('image_width: 100\nimage_height: 50\n')  # no projection params
    with pytest.raises(ValueError):
        ex.load_intrinsics_yaml(p)


# --------------------------------------------------------------------------- #
# Pose resolution with extrinsic + time offset + drop
# --------------------------------------------------------------------------- #
def _traj():
    return [
        pi.TrajectorySample(0.0, np.array([0.0, 0.0, 0.0]),
                            np.array([0.0, 0.0, 0.0, 1.0])),
        pi.TrajectorySample(2.0, np.array([2.0, 0.0, 0.0]),
                            np.array([0.0, 0.0, 0.0, 1.0])),
    ]


def test_resolve_in_range_applies_extrinsic():
    body_T_cam = pi.make_transform([0.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    T = ex.resolve_world_T_camera(1.0, _traj(), body_T_cam)
    assert T is not None
    # body at x=1.0, camera +0.5 ahead -> x=1.5
    np.testing.assert_allclose(T[:3, 3], [1.5, 0.0, 0.0], atol=1e-12)


def test_resolve_out_of_range_returns_none():
    assert ex.resolve_world_T_camera(10.0, _traj(), np.eye(4),
                                     max_extrapolation=0.05) is None


def test_resolve_time_offset_shifts_lookup():
    # stamp -0.5 alone is out of range, but +0.6 offset puts it at 0.1 (in range)
    T = ex.resolve_world_T_camera(-0.5, _traj(), np.eye(4),
                                  max_extrapolation=0.0, time_offset=0.6)
    assert T is not None
    np.testing.assert_allclose(T[:3, 3], [0.1, 0.0, 0.0], atol=1e-12)


# --------------------------------------------------------------------------- #
# Parser
# --------------------------------------------------------------------------- #
def test_parser_requires_bag_traj_out():
    parser = ex.build_parser()
    args = parser.parse_args(
        ['--bag', 'b', '--traj', 't', '--out', 'o', '--camera-topic', '/cam']
    )
    assert args.bag == 'b' and args.traj == 't' and args.out == 'o'
    assert args.camera_topic == '/cam'
    assert args.max_extrapolation == 0.05


def test_parser_missing_required_exits():
    with pytest.raises(SystemExit):
        ex.build_parser().parse_args(['--bag', 'b'])


# --------------------------------------------------------------------------- #
# FILE-compressed bag detection (metadata-only; ROS-free)
# --------------------------------------------------------------------------- #
def test_bag_file_compression_detected(tmp_path):
    (tmp_path / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  storage_identifier: sqlite3\n'
        '  compression_format: zstd\n'
        '  compression_mode: FILE\n'
    )
    assert ex._bag_is_file_compressed(tmp_path) is True


def test_bag_uncompressed_when_mode_none(tmp_path):
    (tmp_path / 'metadata.yaml').write_text(
        '  compression_format: ""\n  compression_mode: ""\n'
    )
    assert ex._bag_is_file_compressed(tmp_path) is False


def test_bag_uncompressed_when_no_metadata(tmp_path):
    assert ex._bag_is_file_compressed(tmp_path) is False
