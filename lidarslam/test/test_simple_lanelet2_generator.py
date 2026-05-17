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

"""Regression tests for the simple Lanelet2 generator helper script."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'simple_lanelet2_generator.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('simple_lanelet2_generator', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _straight_trajectory(n_pts: int = 80, dx: float = 1.0) -> np.ndarray:
    """Build a synthetic centreline along +x at 1 m spacing."""
    xs = np.arange(n_pts) * dx
    ys = np.zeros(n_pts)
    zs = np.zeros(n_pts)
    return np.column_stack([xs, ys, zs])


def _build_osm(module, n_pts: int = 80, segment_length: int = 25,
               add_local_coords: bool = True):
    centre = _straight_trajectory(n_pts)
    left, right = module.offset_boundaries(centre, half_width=1.75)
    return module.build_osm(
        left,
        right,
        origin_lat=35.6862,
        origin_lon=139.6891,
        speed_limit=20.0,
        segment_length=segment_length,
        add_local_coords=add_local_coords,
    )


def test_multi_segment_emits_multiple_lanelets():
    module = _load_module()
    osm, n_lanelets = _build_osm(module, n_pts=80, segment_length=25)
    assert n_lanelets >= 3, f'expected >=3 lanelets for 80pts/seg25, got {n_lanelets}'
    relations = [r for r in osm.findall('relation')
                 if any(t.get('k') == 'type' and t.get('v') == 'lanelet'
                        for t in r.findall('tag'))]
    assert len(relations) == n_lanelets


def test_adjacent_lanelets_share_boundary_node_ids():
    """The Lanelet2 routing graph relies on node identity, not coordinates."""
    module = _load_module()
    osm, _ = _build_osm(module, n_pts=80, segment_length=25)

    ways = {w.get('id'): [nd.get('ref') for nd in w.findall('nd')]
            for w in osm.findall('way')}
    lanelets = sorted(
        [r for r in osm.findall('relation')
         if any(t.get('k') == 'type' and t.get('v') == 'lanelet'
                for t in r.findall('tag'))],
        key=lambda r: int(r.get('id') or '0'),
    )
    for a, b in zip(lanelets, lanelets[1:]):
        a_left = next(m.get('ref') for m in a.findall('member') if m.get('role') == 'left')
        a_right = next(m.get('ref') for m in a.findall('member') if m.get('role') == 'right')
        b_left = next(m.get('ref') for m in b.findall('member') if m.get('role') == 'left')
        b_right = next(m.get('ref') for m in b.findall('member') if m.get('role') == 'right')
        assert ways[a_left][-1] == ways[b_left][0], (
            f'left boundary not shared between {a.get("id")} and {b.get("id")}'
        )
        assert ways[a_right][-1] == ways[b_right][0], (
            f'right boundary not shared between {a.get("id")} and {b.get("id")}'
        )


def test_every_node_has_ele_tag():
    module = _load_module()
    osm, _ = _build_osm(module)
    for node in osm.findall('node'):
        tags = {t.get('k') for t in node.findall('tag')}
        assert 'ele' in tags, f'node {node.get("id")} missing ele tag'


def test_lanelet_relations_carry_required_autoware_tags():
    module = _load_module()
    osm, _ = _build_osm(module)
    for rel in osm.findall('relation'):
        tag_kv = {t.get('k'): t.get('v') for t in rel.findall('tag')}
        if tag_kv.get('type') != 'lanelet':
            continue
        for required in module.REQUIRED_LANELET_TAGS:
            assert required in tag_kv, f'lanelet {rel.get("id")} missing tag {required}'
        assert tag_kv['subtype'] == 'road'
        assert tag_kv['one_way'] == 'yes'
        assert tag_kv['participant:vehicle'] == 'yes'


def test_local_coords_tags_emitted_by_default():
    module = _load_module()
    osm, _ = _build_osm(module, add_local_coords=True)
    sample = next(iter(osm.findall('node')))
    keys = {t.get('k') for t in sample.findall('tag')}
    assert 'local_x' in keys
    assert 'local_y' in keys


def test_local_coords_tags_omitted_when_disabled():
    module = _load_module()
    osm, _ = _build_osm(module, add_local_coords=False)
    sample = next(iter(osm.findall('node')))
    keys = {t.get('k') for t in sample.findall('tag')}
    assert 'local_x' not in keys
    assert 'local_y' not in keys


def test_validate_structure_passes_on_generated_map():
    module = _load_module()
    osm, _ = _build_osm(module, n_pts=80, segment_length=25)
    ok, msgs = module.validate_structure(osm)
    assert ok, '\n'.join(msgs)
    summary = '\n'.join(msgs)
    assert 'adjacent lanelets share boundary nodes' in summary


def test_validate_structure_catches_broken_node_sharing():
    """Mutate the OSM so adjacent lanelets no longer share a node."""
    module = _load_module()
    osm, _ = _build_osm(module, n_pts=80, segment_length=25)

    ways = {w.get('id'): w for w in osm.findall('way')}
    lanelets = sorted(
        [r for r in osm.findall('relation')
         if any(t.get('k') == 'type' and t.get('v') == 'lanelet'
                for t in r.findall('tag'))],
        key=lambda r: int(r.get('id') or '0'),
    )
    second_left_way_id = next(
        m.get('ref') for m in lanelets[1].findall('member') if m.get('role') == 'left'
    )
    way = ways[second_left_way_id]
    way.find('nd').set('ref', '999999')

    ok, msgs = module.validate_structure(osm)
    assert not ok
    assert any('does not share' in m for m in msgs)


def test_validate_structure_catches_missing_ele():
    module = _load_module()
    osm, _ = _build_osm(module)
    node = osm.find('node')
    for tag in list(node.findall('tag')):
        if tag.get('k') == 'ele':
            node.remove(tag)
    ok, msgs = module.validate_structure(osm)
    assert not ok
    assert any('ele' in m for m in msgs)


def test_validate_routing_skips_when_lanelet2_unavailable(tmp_path: Path,
                                                          monkeypatch: pytest.MonkeyPatch):
    """The routing check must degrade gracefully without ROS lanelet2 bindings."""
    module = _load_module()
    osm, _ = _build_osm(module)
    out = tmp_path / 'lanelet2_map.osm'
    module.write_osm(osm, out)

    import builtins
    real_import = builtins.__import__

    def _blocked(name, *args, **kwargs):
        if name == 'lanelet2' or name.startswith('lanelet2.'):
            raise ImportError('forced for test')
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', _blocked)
    result, msgs = module.validate_routing(out, origin_lat=35.6862, origin_lon=139.6891)
    assert result is None
    assert any('lanelet2 Python bindings' in m for m in msgs)
