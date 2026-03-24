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

"""Tests for Autoware map verification and staging helpers."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import struct
import subprocess

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
VERIFY_SCRIPT = REPO_ROOT / 'scripts' / 'verify_autoware_map.py'
PREPARE_SCRIPT = (
    REPO_ROOT
    / 'scripts'
    / 'prepare_autoware_map_from_graph_slam.sh'
)


def _load_verify_module():
    """Load the standalone verifier script as an importable module."""
    spec = importlib.util.spec_from_file_location(
        'verify_autoware_map',
        VERIFY_SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


VERIFY_MODULE = _load_verify_module()
MapVerifier = VERIFY_MODULE.MapVerifier


def _has_text(messages, needle):
    """Return True when any message contains the requested text."""
    return any(needle in message for message in messages)


def _write_binary_xyz_pcd(path, points):
    """Write a minimal binary XYZ PCD file."""
    header = '\n'.join([
        '# .PCD v0.7 - Point Cloud Data file format',
        'VERSION 0.7',
        'FIELDS x y z',
        'SIZE 4 4 4',
        'TYPE F F F',
        'COUNT 1 1 1',
        f'WIDTH {len(points)}',
        'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {len(points)}',
        'DATA binary',
        '',
    ])
    payload = b''.join(struct.pack('<fff', *point) for point in points)
    path.write_bytes(header.encode('ascii') + payload)


def _create_map_bundle(
    root,
    *,
    tile_coords=(0, 0),
    tile_name=None,
    points=None,
    projector_type='Local',
    map_origin=None,
):
    """Create a minimal Autoware-style pointcloud map bundle."""
    pointcloud_dir = root / 'pointcloud_map'
    pointcloud_dir.mkdir(parents=True, exist_ok=True)

    if points is None:
        points = [(1.0, 1.0, 0.0), (5.0, 5.0, 0.0)]
    if tile_name is None:
        tile_name = (
            f'{int(tile_coords[0])}_{int(tile_coords[1])}.pcd'
        )

    _write_binary_xyz_pcd(pointcloud_dir / tile_name, points)
    metadata = {
        'x_resolution': 20,
        'y_resolution': 20,
        tile_name: [tile_coords[0], tile_coords[1]],
    }
    metadata_path = pointcloud_dir / 'pointcloud_map_metadata.yaml'
    metadata_path.write_text(
        yaml.safe_dump(metadata, sort_keys=False),
        encoding='utf-8',
    )

    projector = {
        'projector_type': projector_type,
        'scale_factor': 1.0,
    }
    if map_origin is not None:
        projector['map_origin'] = map_origin
    projector_path = root / 'map_projector_info.yaml'
    projector_path.write_text(
        yaml.safe_dump(projector, sort_keys=False),
        encoding='utf-8',
    )

    return root, pointcloud_dir


def test_map_verifier_accepts_valid_pointcloud_map_dir(tmp_path):
    """A valid pointcloud_map directory should pass verification."""
    _, pointcloud_dir = _create_map_bundle(tmp_path)

    verifier = MapVerifier(str(pointcloud_dir))

    assert verifier.run() is True
    assert verifier.failures == []
    assert _has_text(
        verifier.passes,
        'map_projector_info.yaml: projector_type=Local',
    )


def test_map_verifier_accepts_bundle_root_and_subdirectory(tmp_path):
    """The verifier should locate metadata in a pointcloud_map subdir."""
    root, _ = _create_map_bundle(tmp_path)

    verifier = MapVerifier(str(root))

    assert verifier.run() is True
    assert _has_text(
        verifier.warnings,
        'metadata found in subdirectory',
    )


def test_map_verifier_rejects_float_tile_coordinates(tmp_path):
    """Float YAML coordinates should fail Autoware compatibility."""
    _, pointcloud_dir = _create_map_bundle(
        tmp_path,
        tile_coords=(0.0, 0.0),
    )

    verifier = MapVerifier(str(pointcloud_dir))

    assert verifier.run() is False
    assert _has_text(verifier.failures, 'YAML coordinates are floats')


def test_map_verifier_requires_origin_for_local_cartesian(tmp_path):
    """Require map_origin for LocalCartesian maps."""
    _, pointcloud_dir = _create_map_bundle(
        tmp_path,
        projector_type='LocalCartesian',
    )

    verifier = MapVerifier(str(pointcloud_dir))

    assert verifier.run() is False
    assert _has_text(verifier.failures, 'missing map_origin')


def test_map_verifier_accepts_local_cartesian_with_origin(tmp_path):
    """Accept LocalCartesian maps that provide map_origin."""
    _, pointcloud_dir = _create_map_bundle(
        tmp_path,
        projector_type='LocalCartesian',
        map_origin={
            'latitude': 25.0,
            'longitude': 121.0,
            'altitude': 10.0,
        },
    )

    verifier = MapVerifier(str(pointcloud_dir))

    assert verifier.run() is True
    assert _has_text(
        verifier.passes,
        'map_origin: lat=25.0, lon=121.0',
    )


def test_map_verifier_warns_on_out_of_bounds_points(tmp_path):
    """Bounds checking should warn on out-of-range tile points."""
    _, pointcloud_dir = _create_map_bundle(
        tmp_path,
        points=[(1.0, 1.0, 0.0), (25.0, 3.0, 0.0)],
    )

    verifier = MapVerifier(str(pointcloud_dir), check_bounds=True)

    assert verifier.run() is True
    assert _has_text(verifier.warnings, 'outside tile bounds')


def test_prepare_script_stages_bundle_and_optional_map_file(tmp_path):
    """The staging script should copy the bundle and optional map.pcd."""
    source_root, _ = _create_map_bundle(tmp_path / 'graph_output')
    _write_binary_xyz_pcd(source_root / 'map.pcd', [(0.0, 0.0, 0.0)])
    target_root = tmp_path / 'staged_map'

    result = subprocess.run(
        ['bash', str(PREPARE_SCRIPT), str(source_root), str(target_root)],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert (
        target_root
        / 'pointcloud_map'
        / 'pointcloud_map_metadata.yaml'
    ).is_file()
    assert (target_root / 'pointcloud_map' / '0_0.pcd').is_file()
    assert (target_root / 'map_projector_info.yaml').is_file()
    assert (target_root / 'map.pcd').is_file()
    assert 'RESULT: PASS -- map is Autoware-compatible' in result.stdout


def test_prepare_script_replaces_existing_pointcloud_map_contents(tmp_path):
    """The staging script should replace stale pointcloud_map contents."""
    source_root, _ = _create_map_bundle(tmp_path / 'graph_output')
    target_root = tmp_path / 'staged_map'
    stale_dir = target_root / 'pointcloud_map'
    stale_dir.mkdir(parents=True)
    (stale_dir / 'stale_tile.pcd').write_text(
        'stale',
        encoding='utf-8',
    )

    result = subprocess.run(
        ['bash', str(PREPARE_SCRIPT), str(source_root), str(target_root)],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert not (target_root / 'pointcloud_map' / 'stale_tile.pcd').exists()
    assert (target_root / 'pointcloud_map' / '0_0.pcd').is_file()


def test_prepare_script_fails_without_projector_file(tmp_path):
    """The staging script should reject source dirs without projector info."""
    source_root = tmp_path / 'graph_output'
    pointcloud_dir = source_root / 'pointcloud_map'
    pointcloud_dir.mkdir(parents=True)
    _write_binary_xyz_pcd(pointcloud_dir / '0_0.pcd', [(0.0, 0.0, 0.0)])
    metadata = {
        'x_resolution': 20,
        'y_resolution': 20,
        '0_0.pcd': [0, 0],
    }
    metadata_path = pointcloud_dir / 'pointcloud_map_metadata.yaml'
    metadata_path.write_text(
        yaml.safe_dump(metadata, sort_keys=False),
        encoding='utf-8',
    )
    target_root = tmp_path / 'staged_map'

    result = subprocess.run(
        ['bash', str(PREPARE_SCRIPT), str(source_root), str(target_root)],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert 'map_projector_info.yaml not found' in result.stderr
