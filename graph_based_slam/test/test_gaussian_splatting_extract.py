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
