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
    segment_length: int = 25,
    add_local_coords: bool = True,
) -> tuple[ET.Element, int]:
    """Build an OSM ElementTree from left/right boundary arrays (local coords).

    The trajectory is split into multiple lanelets of *segment_length* points
    each. Adjacent lanelets share boundary nodes so that Autoware's routing
    graph recognises them as connected.

    When *add_local_coords* is True, every node also carries ``local_x``/
    ``local_y`` tags holding the source frame x/y. Autoware's lanelet2
    extension uses these when ``projector_type: local`` is set in
    ``map_projector_info.yaml``.

    Returns (osm_element, number_of_lanelets).
    """
    n_pts = len(left)
    osm = ET.Element('osm', version='0.6', generator='simple_lanelet2_generator')
    ET.SubElement(osm, 'MetaInfo', format_version='1', map_version='1')

    def _add_tag(parent: ET.Element, k: str, v: str) -> None:
        ET.SubElement(parent, 'tag', k=k, v=v)

    # Convert local → WGS84
    left_lat, left_lon, left_ele = local_to_wgs84(
        left[:, 0], left[:, 1], left[:, 2], origin_lat, origin_lon
    )
    right_lat, right_lon, right_ele = local_to_wgs84(
        right[:, 0], right[:, 1], right[:, 2], origin_lat, origin_lon
    )

    # --- Create all nodes up-front (shared at segment boundaries) ---
    nid = 0
    left_node_ids: list[int] = []
    right_node_ids: list[int] = []

    for i in range(n_pts):
        for lat_arr, lon_arr, ele_arr, xy_src, node_list in [
            (left_lat, left_lon, left_ele, left, left_node_ids),
            (right_lat, right_lon, right_ele, right, right_node_ids),
        ]:
            nid += 1
            node = ET.SubElement(
                osm, 'node', id=str(nid), visible='true',
                lat=f'{lat_arr[i]:.10f}', lon=f'{lon_arr[i]:.10f}',
            )
            _add_tag(node, 'ele', f'{ele_arr[i]:.4f}')
            if add_local_coords:
                _add_tag(node, 'local_x', f'{xy_src[i, 0]:.4f}')
                _add_tag(node, 'local_y', f'{xy_src[i, 1]:.4f}')
            node_list.append(nid)

    # --- Split into lanelet segments ---
    n_segs = max(1, (n_pts - 1) // segment_length)
    way_id = 10000
    rel_id = 20000

    for seg in range(n_segs):
        start = seg * segment_length
        end = min(start + segment_length, n_pts - 1)
        if seg == n_segs - 1:
            end = n_pts - 1

        # Left boundary way
        way_id += 1
        left_way_id = way_id
        left_way = ET.SubElement(osm, 'way', id=str(left_way_id), visible='true')
        for i in range(start, end + 1):
            ET.SubElement(left_way, 'nd', ref=str(left_node_ids[i]))
        _add_tag(left_way, 'type', 'line_thin')
        _add_tag(left_way, 'subtype', 'solid')

        # Right boundary way
        way_id += 1
        right_way_id = way_id
        right_way = ET.SubElement(osm, 'way', id=str(right_way_id), visible='true')
        for i in range(start, end + 1):
            ET.SubElement(right_way, 'nd', ref=str(right_node_ids[i]))
        _add_tag(right_way, 'type', 'line_thin')
        _add_tag(right_way, 'subtype', 'solid')

        # Lanelet relation
        rel_id += 1
        rel = ET.SubElement(osm, 'relation', id=str(rel_id), visible='true')
        ET.SubElement(rel, 'member', type='way', ref=str(left_way_id), role='left')
        ET.SubElement(rel, 'member', type='way', ref=str(right_way_id), role='right')
        _add_tag(rel, 'type', 'lanelet')
        _add_tag(rel, 'subtype', 'road')
        _add_tag(rel, 'speed_limit', str(speed_limit))
        _add_tag(rel, 'location', 'urban')
        _add_tag(rel, 'one_way', 'yes')
        _add_tag(rel, 'participant:vehicle', 'yes')

    return osm, n_segs


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
# Validation helpers (pure Python structural + optional lanelet2 routing)
# ---------------------------------------------------------------------------


REQUIRED_LANELET_TAGS = (
    'subtype',
    'location',
    'one_way',
    'participant:vehicle',
    'speed_limit',
)


def validate_structure(osm: ET.Element) -> tuple[bool, list[str]]:
    """Pure-Python structural validation. Returns (ok, messages).

    Checks that:
      - every node carries an ``ele`` tag (Autoware lanelet2 extension requires)
      - every lanelet relation carries the tags Autoware needs to treat it as
        a drivable road for vehicles
      - every lanelet has both ``left`` and ``right`` boundary ways
      - adjacent lanelets share boundary node IDs (not just coordinates), which
        is what Lanelet2's default routing graph uses to infer succession.

    The check is intentionally cheap so it can run in CI without ROS.
    """
    msgs: list[str] = []
    ok = True

    nodes = {n.get('id'): n for n in osm.findall('node')}
    ways = {w.get('id'): w for w in osm.findall('way')}
    lanelets = [
        r
        for r in osm.findall('relation')
        if any(t.get('k') == 'type' and t.get('v') == 'lanelet' for t in r.findall('tag'))
    ]
    msgs.append(f'structure: {len(nodes)} nodes, {len(ways)} ways, {len(lanelets)} lanelets')

    if not lanelets:
        return False, msgs + ['FAIL: no lanelet relations found']

    no_ele = [
        nid for nid, n in nodes.items()
        if not any(t.get('k') == 'ele' for t in n.findall('tag'))
    ]
    if no_ele:
        ok = False
        msgs.append(f'FAIL: {len(no_ele)} nodes missing "ele" tag (first: {no_ele[:3]})')

    for rel in lanelets:
        rid = rel.get('id')
        tag_keys = {t.get('k') for t in rel.findall('tag')}
        missing = [k for k in REQUIRED_LANELET_TAGS if k not in tag_keys]
        if missing:
            ok = False
            msgs.append(f'FAIL: lanelet {rid} missing tags {missing}')
        roles = {m.get('role') for m in rel.findall('member')}
        if 'left' not in roles or 'right' not in roles:
            ok = False
            msgs.append(f'FAIL: lanelet {rid} missing left/right member')

    def way_node_refs(way_id: str | None) -> list[str]:
        w = ways.get(way_id) if way_id else None
        return [nd.get('ref') for nd in w.findall('nd')] if w is not None else []

    sorted_lanelets = sorted(lanelets, key=lambda r: int(r.get('id') or '0'))
    shared_pairs = 0
    total_pairs = max(0, len(sorted_lanelets) - 1)
    for i in range(total_pairs):
        a, b = sorted_lanelets[i], sorted_lanelets[i + 1]
        a_left = next((m.get('ref') for m in a.findall('member') if m.get('role') == 'left'), None)
        a_right = next((m.get('ref') for m in a.findall('member') if m.get('role') == 'right'), None)
        b_left = next((m.get('ref') for m in b.findall('member') if m.get('role') == 'left'), None)
        b_right = next((m.get('ref') for m in b.findall('member') if m.get('role') == 'right'), None)

        a_l, a_r = way_node_refs(a_left), way_node_refs(a_right)
        b_l, b_r = way_node_refs(b_left), way_node_refs(b_right)

        if a_l and b_l and a_r and b_r and a_l[-1] == b_l[0] and a_r[-1] == b_r[0]:
            shared_pairs += 1
        else:
            ok = False
            msgs.append(
                f'FAIL: lanelet pair ({a.get("id")}, {b.get("id")}) does not share '
                f'boundary node IDs (left last/first={a_l[-1] if a_l else None}/'
                f'{b_l[0] if b_l else None}, right={a_r[-1] if a_r else None}/'
                f'{b_r[0] if b_r else None})'
            )
    if total_pairs:
        msgs.append(f'structure: {shared_pairs}/{total_pairs} adjacent lanelets share boundary nodes')

    return ok, msgs


def validate_routing(
    osm_path: Path, origin_lat: float, origin_lon: float
) -> tuple[bool | None, list[str]]:
    """Optional routing-graph validation using lanelet2 Python bindings.

    Returns ``(True, msgs)`` on PASS, ``(False, msgs)`` on FAIL, or
    ``(None, msgs)`` when the lanelet2 bindings are unavailable.
    """
    try:
        import lanelet2  # type: ignore
        from lanelet2.projection import UtmProjector  # type: ignore
    except ImportError:
        return None, [
            'routing: lanelet2 Python bindings not importable; '
            'install via `apt install ros-${ROS_DISTRO}-lanelet2-python` and '
            'source the ROS overlay (skipping routing check)'
        ]

    msgs: list[str] = []
    projector = UtmProjector(lanelet2.io.Origin(origin_lat, origin_lon))
    try:
        laneletmap = lanelet2.io.load(str(osm_path), projector)
    except Exception as exc:  # pragma: no cover - depends on lanelet2 build
        return False, [f'routing: lanelet2.io.load failed: {exc}']

    lanelets = sorted(laneletmap.laneletLayer, key=lambda ll: ll.id)
    if not lanelets:
        return False, ['routing: no lanelets parsed from map']

    msgs.append(f'routing: loaded {len(lanelets)} lanelets')

    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(laneletmap, traffic_rules)

    connected = 0
    total = max(0, len(lanelets) - 1)
    for i in range(total):
        following_ids = {f.id for f in routing_graph.following(lanelets[i])}
        if lanelets[i + 1].id in following_ids:
            connected += 1
    msgs.append(f'routing: {connected}/{total} adjacent pairs connected in routing graph')

    path = routing_graph.shortestPath(lanelets[0], lanelets[-1])
    if path is None or len(path) == 0:
        msgs.append(
            f'routing: FAIL shortestPath({lanelets[0].id} → {lanelets[-1].id}) is empty'
        )
        return False, msgs

    msgs.append(
        f'routing: PASS shortestPath({lanelets[0].id} → {lanelets[-1].id}) '
        f'covers {len(path)} lanelets'
    )
    return True, msgs


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
    p.add_argument('--segment-length', type=int, default=25,
                   help='Points per lanelet segment (Autoware needs multiple connected lanelets)')
    p.add_argument('--no-local-coords', dest='add_local_coords', action='store_false',
                   help='Skip emitting local_x/local_y tags on every node '
                        '(default: emit, required by Autoware projector_type=local)')
    p.set_defaults(add_local_coords=True)
    p.add_argument('--no-validate-structure', dest='validate_structure',
                   action='store_false',
                   help='Skip the post-write structural validation')
    p.set_defaults(validate_structure=True)
    p.add_argument('--validate-routing', action='store_true',
                   help='Also build a Lanelet2 routing graph and verify '
                        'shortestPath(first → last). Requires lanelet2 Python bindings.')
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)

    tum = read_tum(args.input)
    xyz = tum[:, 1:4]  # x, y, z
    print(f'Read {len(xyz)} poses from {args.input}')

    xyz_smooth = smooth_and_resample(xyz, args.resolution, args.smooth_factor)
    print(f'Resampled to {len(xyz_smooth)} points (resolution={args.resolution}m)')

    left, right = offset_boundaries(xyz_smooth, args.lane_width / 2.0)

    osm, n_lanelets = build_osm(
        left, right, args.origin_lat, args.origin_lon,
        args.speed_limit, args.segment_length,
        add_local_coords=args.add_local_coords,
    )
    write_osm(osm, args.output)

    n_nodes = len(left) + len(right)
    print(f'Wrote {args.output}  ({n_nodes} nodes, {n_lanelets * 2} ways, {n_lanelets} lanelets)')

    exit_code = 0
    if args.validate_structure:
        ok, msgs = validate_structure(osm)
        for line in msgs:
            print(line)
        if not ok:
            print('Structural validation FAILED', file=sys.stderr)
            exit_code = 1

    if args.validate_routing:
        result, msgs = validate_routing(args.output, args.origin_lat, args.origin_lon)
        for line in msgs:
            print(line)
        if result is False:
            print('Routing validation FAILED', file=sys.stderr)
            exit_code = 1

    if exit_code:
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
