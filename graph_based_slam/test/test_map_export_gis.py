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

"""Tests for the GIS export hub (coloured cloud -> QGIS delimited text)."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import map_export

    return map_export


mx = _load()


def _read_csv(path):
    lines = Path(path).read_text().strip().splitlines()
    header = lines[0].split(',')
    rows = [line.split(',') for line in lines[1:]]
    return header, rows


# --------------------------------------------------------------------------- #
# Local-frame export (no origin)
# --------------------------------------------------------------------------- #
def test_local_export_headers_and_values(tmp_path):
    xyz = np.array([[1.0, 2.0, 3.0], [-4.5, 5.25, 6.0]])
    out = mx.write_gis_csv(tmp_path / 'm.csv', xyz)
    header, rows = _read_csv(out)
    assert header == ['x', 'y', 'z']
    assert len(rows) == 2
    np.testing.assert_allclose([float(v) for v in rows[0]], [1.0, 2.0, 3.0])
    # No .prj sidecar without georeferencing; .csvt always present.
    assert not out.with_suffix('.prj').exists()
    assert out.with_suffix('.csvt').exists()


def test_local_export_carries_rgb(tmp_path):
    xyz = np.zeros((2, 3))
    rgb = np.array([[255, 0, 0], [10, 20, 30]], dtype=np.uint8)
    out = mx.write_gis_csv(tmp_path / 'm.csv', xyz, rgb)
    header, rows = _read_csv(out)
    assert header == ['x', 'y', 'z', 'red', 'green', 'blue']
    assert [int(v) for v in rows[1][3:]] == [10, 20, 30]
    # Integer columns must be typed as such for QGIS.
    assert '"Integer"' in out.with_suffix('.csvt').read_text()


# --------------------------------------------------------------------------- #
# Georeferenced export
# --------------------------------------------------------------------------- #
def test_geo_export_adds_lonlat_and_prj(tmp_path):
    xyz = np.zeros((1, 3))  # origin itself
    out = mx.write_gis_csv(tmp_path / 'g.csv', xyz,
                           origin_lat=35.0, origin_lon=139.0)
    header, rows = _read_csv(out)
    assert header == ['east', 'north', 'z', 'lon', 'lat', 'ele']
    lon, lat = float(rows[0][3]), float(rows[0][4])
    # A point at the local origin maps back to the WGS84 origin exactly.
    assert abs(lon - 139.0) < 1e-9
    assert abs(lat - 35.0) < 1e-9
    prj = out.with_suffix('.prj').read_text()
    assert 'EPSG' in prj and '4326' in prj


def test_geo_offset_moves_north_and_east(tmp_path):
    # 100 m east and 100 m north of the origin must increase lon and lat.
    xyz = np.array([[100.0, 100.0, 0.0]])
    out = mx.write_gis_csv(tmp_path / 'g.csv', xyz,
                           origin_lat=35.0, origin_lon=139.0)
    _, rows = _read_csv(out)
    lon, lat = float(rows[0][3]), float(rows[0][4])
    assert lon > 139.0
    assert lat > 35.0
    # ~100 m is ~0.0009 deg lat; sanity-bound it well away from a bug.
    assert 0.0005 < (lat - 35.0) < 0.0015


def test_enu_roundtrip_matches_lanelet_generator(tmp_path):
    # enu_to_wgs84 must agree with the geodesy used for lanelet2 maps so a
    # cloud and its map register. Compare against the same linearisation.
    x = np.array([250.0, -80.0])
    y = np.array([-40.0, 300.0])
    z = np.array([1.0, 2.0])
    lon, lat, ele = mx.enu_to_wgs84(x, y, z, 35.6812, 139.7671)
    assert lon.shape == (2,) and lat.shape == (2,)
    np.testing.assert_allclose(ele, z)
    # East offset only changes lon; north offset only changes lat (flat-Earth).
    lon2, lat2, _ = mx.enu_to_wgs84(np.array([250.0]), np.array([0.0]),
                                    np.array([0.0]), 35.6812, 139.7671)
    assert abs(lat2[0] - 35.6812) < 1e-9


# --------------------------------------------------------------------------- #
# Thinning shares one code path with the full export
# --------------------------------------------------------------------------- #
def test_thin_voxel_reduces_point_count(tmp_path):
    # Two tight clusters, each centred inside one 1 m voxel (away from the
    # integer grid boundaries), collapse to 2 points.
    a = np.random.default_rng(0).normal(0.5, 0.01, (50, 3))
    b = a + np.array([10.0, 0.0, 0.0])
    xyz = np.vstack([a, b])
    out = mx.write_gis_csv(tmp_path / 't.csv', xyz, thin_voxel=1.0)
    _, rows = _read_csv(out)
    assert len(rows) == 2


def test_thin_voxel_keeps_rgb_aligned(tmp_path):
    xyz = np.array([[0.0, 0, 0], [0.02, 0, 0], [10.0, 0, 0]])
    rgb = np.array([[255, 0, 0], [255, 0, 0], [0, 0, 255]], dtype=np.uint8)
    out = mx.write_gis_csv(tmp_path / 't.csv', xyz, rgb, thin_voxel=1.0)
    header, rows = _read_csv(out)
    assert 'red' in header
    assert len(rows) == 2  # first two collapse, blue survives separately


# --------------------------------------------------------------------------- #
# Input validation
# --------------------------------------------------------------------------- #
def test_bad_xyz_shape_raises(tmp_path):
    try:
        mx.write_gis_csv(tmp_path / 'x.csv', np.zeros((3, 2)))
    except ValueError:
        return
    raise AssertionError('expected ValueError for (N,2) input')


def test_rgb_shape_mismatch_raises(tmp_path):
    try:
        mx.write_gis_csv(tmp_path / 'x.csv', np.zeros((3, 3)),
                         np.zeros((2, 3), dtype=np.uint8))
    except ValueError:
        return
    raise AssertionError('expected ValueError for mismatched rgb')
