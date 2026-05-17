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

"""Regression tests for triangle descriptor parameters in default YAML files."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]

EXPECTED_PARAM_KEYS = {
    'use_triangle_descriptor',
    'triangle_descriptor_grid_size_m',
    'triangle_descriptor_grid_cells',
    'triangle_descriptor_max_keypoints',
    'triangle_descriptor_min_salience_m',
    'triangle_descriptor_min_edge_m',
    'triangle_descriptor_max_edge_m',
    'triangle_descriptor_max_triangles',
    'triangle_descriptor_edge_bin_m',
    'triangle_descriptor_min_votes',
    'triangle_descriptor_min_inliers',
    'triangle_descriptor_inlier_translation_m',
    'triangle_descriptor_inlier_rotation_deg',
    'triangle_descriptor_exclude_recent',
}

PARAM_FILES = [
    REPO_ROOT / 'graph_based_slam' / 'param' / 'graphbasedslam.yaml',
    REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam_mid360_rko_graph.yaml',
]


def _load_graph_params(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert 'graph_based_slam' in data, f'{path} missing graph_based_slam node'
    params = data['graph_based_slam'].get('ros__parameters')
    assert params is not None, f'{path} missing ros__parameters'
    return params


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_yaml_contains_all_triangle_descriptor_keys(path):
    params = _load_graph_params(path)
    missing = EXPECTED_PARAM_KEYS - set(params.keys())
    assert not missing, (
        f'{path.name} missing keys: {sorted(missing)}'
    )


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_triangle_descriptor_defaults_off(path):
    params = _load_graph_params(path)
    assert params['use_triangle_descriptor'] is False, (
        f'{path.name}: triangle descriptor must default off so the existing '
        'workflow stays unchanged'
    )


@pytest.mark.parametrize('path', PARAM_FILES, ids=lambda p: p.name)
def test_triangle_descriptor_edge_bounds_sane(path):
    params = _load_graph_params(path)
    min_edge = params['triangle_descriptor_min_edge_m']
    max_edge = params['triangle_descriptor_max_edge_m']
    assert min_edge > 0.0, f'{path.name}: min_edge_m must be positive'
    assert max_edge > min_edge, (
        f'{path.name}: max_edge_m ({max_edge}) must exceed min_edge_m ({min_edge})'
    )


def test_mid360_preset_tightens_for_short_range_sensor():
    """MID-360 has shorter range than spinning 360° LiDAR; preset must reflect."""
    default = _load_graph_params(PARAM_FILES[0])
    mid360 = _load_graph_params(PARAM_FILES[1])
    assert mid360['triangle_descriptor_max_edge_m'] <= default['triangle_descriptor_max_edge_m'], (
        'MID-360 preset must not exceed the generic LiDAR max edge (shorter range sensor)'
    )
    assert mid360['triangle_descriptor_min_votes'] >= default['triangle_descriptor_min_votes'], (
        'MID-360 preset should be stricter on vote count to suppress FOV ambiguity'
    )
