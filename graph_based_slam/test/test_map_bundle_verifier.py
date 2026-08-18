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


"""Tests for the graph-SLAM map-save bundle verifier."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
VERIFY_SCRIPT = REPO_ROOT / 'scripts' / 'verify_map_bundle.py'


def _load_verifier():
    spec = importlib.util.spec_from_file_location('verify_map_bundle', VERIFY_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.BundleVerifier


BundleVerifier = _load_verifier()


def _create_bundle(root: Path, *, submaps: int = 2, loops: int = 1) -> None:
    pointcloud = root / 'pointcloud_map'
    pointcloud.mkdir()
    (root / 'map.pcd').write_bytes(b'pcd')
    (pointcloud / 'pointcloud_map_metadata.yaml').write_text(
        'x_resolution: 20\ny_resolution: 20\n', encoding='utf-8')
    (root / 'map_projector_info.yaml').write_text('projector_type: Local\n', encoding='utf-8')
    trajectory_rows = [f'{index}.0 0 0 0 0 0 0 1' for index in range(submaps)]
    (root / 'trajectory_optimized.tum').write_text(
        '\n'.join(trajectory_rows) + '\n', encoding='utf-8')
    graph_rows = [f'VERTEX_SE3:QUAT {index} 0 0 0 0 0 0 1' for index in range(submaps)]
    (root / 'pose_graph.g2o').write_text('\n'.join(graph_rows) + '\n', encoding='utf-8')
    header = 'from,to,fitness,tx,ty,tz,qx,qy,qz,qw\n'
    edge_rows = ''.join(f'{index},{index + 1},0.1,0,0,0,0,0,0,1\n' for index in range(loops))
    (root / 'loop_edges.csv').write_text(header + edge_rows, encoding='utf-8')
    manifest = {
        'format_version': 1,
        'frame_id': 'map',
        'submap_count': submaps,
        'loop_edge_count': loops,
        'artifacts': {
            'full_map': 'map.pcd',
            'pointcloud_map': 'pointcloud_map',
            'trajectory': 'trajectory_optimized.tum',
            'pose_graph': 'pose_graph.g2o',
            'loop_edges': 'loop_edges.csv',
            'projector_info': 'map_projector_info.yaml',
        },
    }
    (root / 'map_bundle.yaml').write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8')


def test_accepts_consistent_bundle(tmp_path):
    _create_bundle(tmp_path)
    verifier = BundleVerifier(tmp_path)
    assert verifier.run() is True
    assert verifier.failures == []


def test_rejects_cross_file_count_mismatch(tmp_path):
    _create_bundle(tmp_path, submaps=2)
    (tmp_path / 'trajectory_optimized.tum').write_text(
        '0.0 0 0 0 0 0 0 1\n', encoding='utf-8')
    verifier = BundleVerifier(tmp_path)
    assert verifier.run() is False
    assert any('trajectory rows 1 != submap_count 2' in item for item in verifier.failures)


def test_rejects_artifact_path_escape(tmp_path):
    _create_bundle(tmp_path)
    manifest_path = tmp_path / 'map_bundle.yaml'
    manifest = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
    manifest['artifacts']['full_map'] = '../map.pcd'
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8')
    verifier = BundleVerifier(tmp_path)
    assert verifier.run() is False
    assert any('escapes the bundle directory' in item for item in verifier.failures)


def test_rejects_loop_edge_vertex_outside_pose_graph(tmp_path):
    _create_bundle(tmp_path, submaps=2, loops=1)
    (tmp_path / 'loop_edges.csv').write_text(
        'from,to,fitness,tx,ty,tz,qx,qy,qz,qw\n'
        '0,2,0.1,0,0,0,0,0,0,1\n',
        encoding='utf-8',
    )
    verifier = BundleVerifier(tmp_path)
    assert verifier.run() is False
    assert any('invalid vertices 0->2' in item for item in verifier.failures)


def test_rejects_non_finite_loop_edge_value(tmp_path):
    _create_bundle(tmp_path, submaps=2, loops=1)
    (tmp_path / 'loop_edges.csv').write_text(
        'from,to,fitness,tx,ty,tz,qx,qy,qz,qw\n'
        '0,1,nan,0,0,0,0,0,0,1\n',
        encoding='utf-8',
    )
    verifier = BundleVerifier(tmp_path)
    assert verifier.run() is False
    assert any('non-finite value' in item for item in verifier.failures)
