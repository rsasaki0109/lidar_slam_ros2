#!/usr/bin/env python3
"""Generate a minimal Autoware-compatible Lanelet2 map from a TUM trajectory.

Usage:
    python3 scripts/simple_lanelet2_generator.py \
        --input output/.../traj_corrected.tum \
        --output map/lanelet2_map.osm \
        --lane-width 3.5 \
        --origin-lat 1.345 --origin-lon 103.680

The script reads a TUM-format trajectory (timestamp x y z qx qy qz qw),
smooths it with spline interpolation, offsets left/right lane boundaries,
converts local coordinates to WGS84, and writes a Lanelet2 OSM XML file.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.interpolate import splev, splprep


# ---------------------------------------------------------------------------
# Coordinate conversion helpers
# ---------------------------------------------------------------------------

# WGS84 ellipsoid
_A = 6_378_137.0  # semi-major axis [m]
_F = 1.0 / 298.257223563
_B = _A * (1.0 - _F)  # semi-minor axis
_E2 = 1.0 - (_B * _B) / (_A * _A)


def _radius_of_curvature_n(lat_rad: float) -> float:
    sin_lat = math.sin(lat_rad)
    return _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)


def local_to_wgs84(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    origin_lat: float,
    origin_lon: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert local ENU (x=east, y=north) offsets to WGS84 lat/lon/ele."""
    lat0 = math.radians(origin_lat)
    lon0 = math.radians(origin_lon)
    n0 = _radius_of_curvature_n(lat0)
    m0 = _A * (1.0 - _E2) / (1.0 - _E2 * math.sin(lat0) ** 2) ** 1.5

    lat = np.degrees(lat0 + y / m0)
    lon = np.degrees(lon0 + x / (n0 * math.cos(lat0)))
    ele = z.copy()
    return lat, lon, ele


# ---------------------------------------------------------------------------
# TUM trajectory I/O
# ---------------------------------------------------------------------------


def read_tum(path: Path) -> np.ndarray:
    """Read TUM trajectory file. Returns Nx8 array (t x y z qx qy qz qw)."""
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) >= 8:
            rows.append([float(v) for v in parts[:8]])
    if not rows:
        raise ValueError(f'No valid TUM lines in {path}')
    return np.array(rows)


# ---------------------------------------------------------------------------
# Trajectory processing
# ---------------------------------------------------------------------------


def smooth_and_resample(
    xyz: np.ndarray, resolution: float, smooth_factor: float
) -> np.ndarray:
    """Smooth a 3D polyline with B-spline and resample at *resolution* metres."""
    dists = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    cum = np.concatenate(([0.0], np.cumsum(dists)))
    total = cum[-1]
    if total < resolution:
        return xyz

    # Remove duplicate points (zero-length segments break splprep)
    mask = np.concatenate(([True], dists > 1e-6))
    xyz_clean = xyz[mask]
    cum_clean = cum[mask]
    if len(xyz_clean) < 4:
        return xyz

    k = min(3, len(xyz_clean) - 1)
    tck, _ = splprep(
        [xyz_clean[:, 0], xyz_clean[:, 1], xyz_clean[:, 2]],
        u=cum_clean / cum_clean[-1],
        s=smooth_factor,
        k=k,
    )

    n_pts = max(int(total / resolution), 2)
    u_new = np.linspace(0, 1, n_pts)
    sx, sy, sz = splev(u_new, tck)
    return np.column_stack([sx, sy, sz])


def offset_boundaries(
    xyz: np.ndarray, half_width: float
) -> tuple[np.ndarray, np.ndarray]:
    """Compute left and right boundary polylines offset from the centreline.

    Left is to the +normal side (left when looking in the direction of travel),
    right is to the -normal side.
    """
    # Forward tangent vectors (central differences, forward/backward at ends)
    tangents = np.zeros_like(xyz)
    tangents[1:-1] = xyz[2:] - xyz[:-2]
    tangents[0] = xyz[1] - xyz[0]
    tangents[-1] = xyz[-1] - xyz[-2]

    # Normalise tangent in XY plane
    t_xy = tangents[:, :2]
    norms = np.linalg.norm(t_xy, axis=1, keepdims=True)
    norms = np.where(norms < 1e-12, 1.0, norms)
    t_xy = t_xy / norms

    # 2D left-normal: rotate tangent 90° CCW → (-ty, tx)
    normals = np.zeros_like(xyz)
    normals[:, 0] = -t_xy[:, 1]
    normals[:, 1] = t_xy[:, 0]

    left = xyz + normals * half_width
    right = xyz - normals * half_width
    return left, right


# ---------------------------------------------------------------------------
# Lanelet2 OSM writer
# ---------------------------------------------------------------------------


def build_osm(
    left: np.ndarray,
    right: np.ndarray,
    origin_lat: float,
    origin_lon: float,
    speed_limit: float,
) -> ET.Element:
    """Build an OSM ElementTree from left/right boundary arrays (local coords)."""
    osm = ET.Element('osm', version='0.6', generator='simple_lanelet2_generator')

    node_id = 0
    way_id_counter = 0
    rel_id_counter = 0

    def _next_node_id() -> int:
        nonlocal node_id
        node_id += 1
        return node_id

    def _next_way_id() -> int:
        nonlocal way_id_counter
        way_id_counter += 1
        return 10000 + way_id_counter

    def _next_rel_id() -> int:
        nonlocal rel_id_counter
        rel_id_counter += 1
        return 20000 + rel_id_counter

    def _add_tag(parent: ET.Element, k: str, v: str) -> None:
        ET.SubElement(parent, 'tag', k=k, v=v)

    # Convert local → WGS84
    left_lat, left_lon, left_ele = local_to_wgs84(
        left[:, 0], left[:, 1], left[:, 2], origin_lat, origin_lon
    )
    right_lat, right_lon, right_ele = local_to_wgs84(
        right[:, 0], right[:, 1], right[:, 2], origin_lat, origin_lon
    )

    # --- Create nodes ---
    left_node_ids = []
    for i in range(len(left)):
        nid = _next_node_id()
        left_node_ids.append(nid)
        node = ET.SubElement(
            osm, 'node',
            id=str(nid),
            visible='true',
            lat=f'{left_lat[i]:.10f}',
            lon=f'{left_lon[i]:.10f}',
        )
        _add_tag(node, 'ele', f'{left_ele[i]:.4f}')

    right_node_ids = []
    for i in range(len(right)):
        nid = _next_node_id()
        right_node_ids.append(nid)
        node = ET.SubElement(
            osm, 'node',
            id=str(nid),
            visible='true',
            lat=f'{right_lat[i]:.10f}',
            lon=f'{right_lon[i]:.10f}',
        )
        _add_tag(node, 'ele', f'{right_ele[i]:.4f}')

    # --- Create ways ---
    left_way_id = _next_way_id()
    left_way = ET.SubElement(osm, 'way', id=str(left_way_id), visible='true')
    for nid in left_node_ids:
        ET.SubElement(left_way, 'nd', ref=str(nid))
    _add_tag(left_way, 'type', 'line_thin')
    _add_tag(left_way, 'subtype', 'solid')

    right_way_id = _next_way_id()
    right_way = ET.SubElement(osm, 'way', id=str(right_way_id), visible='true')
    for nid in right_node_ids:
        ET.SubElement(right_way, 'nd', ref=str(nid))
    _add_tag(right_way, 'type', 'line_thin')
    _add_tag(right_way, 'subtype', 'solid')

    # --- Create lanelet relation ---
    rel_id = _next_rel_id()
    rel = ET.SubElement(osm, 'relation', id=str(rel_id), visible='true')
    ET.SubElement(rel, 'member', type='way', ref=str(left_way_id), role='left')
    ET.SubElement(rel, 'member', type='way', ref=str(right_way_id), role='right')
    _add_tag(rel, 'type', 'lanelet')
    _add_tag(rel, 'subtype', 'road')
    _add_tag(rel, 'speed_limit', str(speed_limit))
    _add_tag(rel, 'location', 'urban')
    _add_tag(rel, 'one_way', 'yes')
    _add_tag(rel, 'participant:vehicle', 'yes')

    return osm


def write_osm(osm: ET.Element, path: Path) -> None:
    """Write OSM element tree to file with XML declaration."""
    tree = ET.ElementTree(osm)
    ET.indent(tree, space='  ')
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(path), encoding='unicode', xml_declaration=True)
    # Append trailing newline
    with open(path, 'a') as f:
        f.write('\n')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description='Generate Lanelet2 map from TUM trajectory',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('--input', required=True, type=Path, help='TUM trajectory file')
    p.add_argument('--output', required=True, type=Path, help='Output OSM file path')
    p.add_argument('--lane-width', type=float, default=3.5, help='Lane width [m]')
    p.add_argument('--origin-lat', type=float, required=True, help='Origin latitude [deg]')
    p.add_argument('--origin-lon', type=float, required=True, help='Origin longitude [deg]')
    p.add_argument('--resolution', type=float, default=1.0, help='Resample spacing [m]')
    p.add_argument('--smooth-factor', type=float, default=0.0, help='Spline smoothing (0=interpolate)')
    p.add_argument('--speed-limit', type=float, default=10.0, help='Speed limit [km/h]')
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)

    tum = read_tum(args.input)
    xyz = tum[:, 1:4]  # x, y, z
    print(f'Read {len(xyz)} poses from {args.input}')

    xyz_smooth = smooth_and_resample(xyz, args.resolution, args.smooth_factor)
    print(f'Resampled to {len(xyz_smooth)} points (resolution={args.resolution}m)')

    left, right = offset_boundaries(xyz_smooth, args.lane_width / 2.0)

    osm = build_osm(left, right, args.origin_lat, args.origin_lon, args.speed_limit)
    write_osm(osm, args.output)

    n_nodes = len(left) + len(right)
    print(f'Wrote {args.output}  ({n_nodes} nodes, 2 ways, 1 lanelet)')


if __name__ == '__main__':
    main()
