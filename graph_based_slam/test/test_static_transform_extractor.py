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

"""Regression tests for extracting static transform chains from tf_static."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'extract_static_transform_from_bag.py'


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'extract_static_transform_from_bag',
        SCRIPT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_quaternion_matrix_round_trip_preserves_yaw_rotation():
    """Quaternion conversion should round-trip a simple yaw rotation."""
    module = _load_module()
    matrix = module.transform_matrix_from_xyz_xyzw(
        1.0,
        2.0,
        3.0,
        0.0,
        0.0,
        math.sqrt(0.5),
        math.sqrt(0.5),
    )

    x, y, z, qx, qy, qz, qw = module.xyz_xyzw_from_transform_matrix(matrix)
    assert (x, y, z) == pytest.approx((1.0, 2.0, 3.0))
    assert (qx, qy, qz, qw) == pytest.approx(
        (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
    )


def test_resolve_transform_chain_composes_parent_child_links():
    """Two static transforms should compose into one chain result."""
    module = _load_module()
    records = [
        module.StaticTransformRecord(
            parent_frame='base_link',
            child_frame='sensor_kit_base_link',
            matrix=module.transform_matrix_from_xyz_xyzw(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        ),
        module.StaticTransformRecord(
            parent_frame='sensor_kit_base_link',
            child_frame='velodyne_front',
            matrix=module.transform_matrix_from_xyz_xyzw(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0),
        ),
    ]

    matrix = module.resolve_transform_chain(records, 'base_link', 'velodyne_front')
    xyz_q = module.xyz_xyzw_from_transform_matrix(matrix)
    assert xyz_q == pytest.approx((1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0))


def test_resolve_transform_chain_inverts_edges_when_walking_backwards():
    """The resolver should invert transforms when going child -> parent."""
    module = _load_module()
    record = module.StaticTransformRecord(
        parent_frame='base_link',
        child_frame='velodyne_front',
        matrix=module.transform_matrix_from_xyz_xyzw(1.0, -2.0, 0.5, 0.0, 0.0, 0.0, 1.0),
    )

    inverse = module.resolve_transform_chain([record], 'velodyne_front', 'base_link')
    expected = np.eye(4, dtype=np.float64)
    expected[:3, 3] = np.array([-1.0, 2.0, -0.5], dtype=np.float64)
    assert inverse == pytest.approx(expected)
