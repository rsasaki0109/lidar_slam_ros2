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

"""
Test the pure numpy plane-extraction helpers in bim_export.

The RANSAC/classification/rectangle maths is deps-free and tested here; the IFC
writer (ifcopenshell) is exercised separately and skipped when absent.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import bim_export

    return bim_export


be = _load()


def test_indoor_cli_preset_and_automatic_output():
    args = be._apply_cli_defaults(
        be._build_arg_parser().parse_args(['/tmp/scan.ply', '--indoor']))
    assert args.output == '/tmp/scan.ifc'
    assert args.building and args.regularize and args.openings and args.rooms
    assert args.max_planes == 20
    assert args.min_inliers == 600
    assert args.thin_voxel == 0.1
    assert args.denoise_voxel == 0.15
    assert args.opening_close == 1


def test_indoor_cli_allows_numeric_overrides():
    args = be._apply_cli_defaults(be._build_arg_parser().parse_args(
        ['scan.ply', 'model.ifc', '--indoor', '--max-planes', '30',
         '--thin-voxel', '0']))
    assert args.output == 'model.ifc'
    assert args.max_planes == 30
    assert args.thin_voxel == 0.0


def test_metrics_json_cli_accepts_automatic_and_explicit_paths():
    parser = be._build_arg_parser()
    assert parser.parse_args(['scan.ply', '--metrics-json']).metrics_json == 'auto'
    assert parser.parse_args(
        ['scan.ply', '--metrics-json', 'qa.json']).metrics_json == 'qa.json'


def test_bim_pipeline_resolves_hilti_name(tmp_path):
    import bim_pipeline
    bag = tmp_path / 'exp01_ros2'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}')
    kind, path = bim_pipeline.resolve_input('exp01', tmp_path)
    assert kind == 'bag'
    assert path == bag


def test_bim_pipeline_resolves_ply(tmp_path):
    import bim_pipeline
    ply = tmp_path / 'building.ply'
    ply.write_bytes(b'ply\n')
    kind, path = bim_pipeline.resolve_input(str(ply), tmp_path)
    assert kind == 'ply'
    assert path == ply


def _grid(nx=30, ny=30, span=5.0):
    xs = np.linspace(-span, span, nx)
    ys = np.linspace(-span, span, ny)
    gx, gy = np.meshgrid(xs, ys)
    return gx.ravel(), gy.ravel()


# --------------------------------------------------------------------------- #
# refit_plane / fit_plane_ransac
# --------------------------------------------------------------------------- #
def test_refit_plane_recovers_horizontal():
    gx, gy = _grid()
    pts = np.stack([gx, gy, np.full_like(gx, 2.0)], axis=1)
    normal, d = be.refit_plane(pts)
    assert abs(abs(normal[2]) - 1.0) < 1e-6      # normal is +-z
    # plane is z = 2  ->  normal.x + d = 0 gives z = -d/normal_z = 2
    np.testing.assert_allclose(-d / normal[2], 2.0, atol=1e-6)


def test_ransac_finds_plane_and_inliers():
    gx, gy = _grid()
    rng = np.random.default_rng(1)
    z = np.full_like(gx, 1.0) + rng.normal(0, 0.01, gx.shape)
    pts = np.stack([gx, gy, z], axis=1)
    res = be.fit_plane_ransac(pts, threshold=0.1, min_inliers=100, seed=3)
    assert res is not None
    normal, d, mask = res
    assert abs(abs(normal[2]) - 1.0) < 1e-3
    assert int(mask.sum()) == len(pts)           # all points are inliers


def test_ransac_returns_none_without_enough_inliers():
    rng = np.random.default_rng(0)
    pts = rng.normal(0, 5.0, size=(200, 3))       # pure noise volume
    res = be.fit_plane_ransac(pts, threshold=0.02, min_inliers=180, seed=0)
    assert res is None


# --------------------------------------------------------------------------- #
# classify_plane
# --------------------------------------------------------------------------- #
def test_classify_horizontal_vertical_other():
    assert be.classify_plane([0, 0, 1]) == 'horizontal'
    assert be.classify_plane([0, 0, -1]) == 'horizontal'
    assert be.classify_plane([1, 0, 0]) == 'vertical'
    assert be.classify_plane([0, 1, 0]) == 'vertical'
    # 45 degrees -> neither
    assert be.classify_plane([0, 1, 1]) == 'other'


# --------------------------------------------------------------------------- #
# plane_basis / oriented_rectangle / box_from_rectangle
# --------------------------------------------------------------------------- #
def test_plane_basis_is_orthonormal_and_in_plane():
    n = np.array([0.3, -0.4, 0.86602540])
    u, v = be.plane_basis(n)
    assert abs(np.linalg.norm(u) - 1) < 1e-9
    assert abs(np.linalg.norm(v) - 1) < 1e-9
    assert abs(u.dot(v)) < 1e-9
    assert abs(u.dot(n)) < 1e-9
    assert abs(v.dot(n)) < 1e-9


def test_oriented_rectangle_dimensions():
    # A flat 8 x 4 rectangle on the z=0 plane.
    xs = np.linspace(-4, 4, 20)   # width 8
    ys = np.linspace(-2, 2, 10)   # height 4
    gx, gy = np.meshgrid(xs, ys)
    pts = np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1)
    corners, size, thickness = be.oriented_rectangle(pts, [0, 0, 1])
    assert corners.shape == (4, 3)
    assert abs(max(size) - 8.0) < 1e-6
    assert abs(min(size) - 4.0) < 1e-6
    assert thickness < 1e-6
    assert np.allclose(corners[:, 2], 0.0)        # stays in plane


def test_box_from_rectangle_is_a_closed_solid():
    corners = np.array([[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]], float)
    verts, faces = be.box_from_rectangle(corners, [0, 0, 1], thickness=0.4)
    assert verts.shape == (8, 3)
    assert len(faces) == 6
    # bottom at z=-0.2, top at z=+0.2
    np.testing.assert_allclose(verts[:4, 2], -0.2)
    np.testing.assert_allclose(verts[4:, 2], 0.2)
    # every vertex index is used by the faces exactly (0..7 all appear)
    used = {i for f in faces for i in f}
    assert used == set(range(8))


def test_extrude_polygon_supports_non_rectangular_room():
    corners = np.array([[0., 0., 1.], [4., 0., 1.], [2., 3., 1.]])
    vertices, faces = be.extrude_polygon(corners, 2.5)
    assert vertices.shape == (6, 3)
    assert len(faces) == 5
    assert np.allclose(vertices[:3, 2], 1.0)
    assert np.allclose(vertices[3:, 2], 3.5)


def test_box_min_thickness_floor():
    corners = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], float)
    verts, _ = be.box_from_rectangle(corners, [0, 0, 1], thickness=0.0)
    # thickness clamped to 1e-3 -> half is 5e-4
    assert abs((verts[4, 2] - verts[0, 2]) - 1e-3) < 1e-9


# --------------------------------------------------------------------------- #
# largest_plane_patch (contiguity filter)
# --------------------------------------------------------------------------- #
def test_largest_patch_picks_bigger_contiguous_cluster():
    # Two disjoint squares on z=0, far apart: a 4x4 m block and a 1x1 m block.
    bx, by = np.meshgrid(np.linspace(0, 4, 20), np.linspace(0, 4, 20))
    big = np.stack([bx.ravel(), by.ravel(), np.zeros(bx.size)], axis=1)
    sx, sy = np.meshgrid(np.linspace(30, 31, 5), np.linspace(30, 31, 5))
    small = np.stack([sx.ravel(), sy.ravel(), np.zeros(sx.size)], axis=1)
    pts = np.vstack([big, small])
    mask = be.largest_plane_patch(pts, [0, 0, 1], cell=1.0)
    assert int(mask.sum()) == len(big)           # only the big block survives
    assert not mask[len(big):].any()             # small block excluded


def test_largest_patch_keeps_single_contiguous_surface():
    gx, gy = _grid(20, 20, 3.0)
    pts = np.stack([gx, gy, np.zeros_like(gx)], axis=1)
    mask = be.largest_plane_patch(pts, [0, 0, 1], cell=1.0)
    assert int(mask.sum()) == len(pts)           # all one cluster


# --------------------------------------------------------------------------- #
# extract_planes
# --------------------------------------------------------------------------- #
def _floor_and_walls(n_floor=60, n_wall=40, floor_span=8.0, wall_h=3.0,
                     floor_z_levels=(0.0,)):
    """
    Build stacked horizontal floors and one modest vertical wall.

    This reproduces the multi-storey "walls drowned by floors" setup.
    """
    parts = []
    fx, fy = _grid(n_floor, n_floor, floor_span)
    for z in floor_z_levels:
        parts.append(np.stack([fx, fy, np.full_like(fx, z)], axis=1))
    wy = np.linspace(-floor_span, floor_span, n_wall)
    wz = np.linspace(0, wall_h, n_wall // 2)
    wgy, wgz = np.meshgrid(wy, wz)
    parts.append(np.stack([np.zeros(wgy.size), wgy.ravel(), wgz.ravel()], axis=1))
    return np.vstack(parts)


def test_ransac_vertical_orient_finds_wall_over_bigger_floor():
    # Floor has ~5x the wall's points; unrestricted RANSAC returns the floor,
    # but orient='vertical' must return the wall.
    pts = _floor_and_walls()
    res = be.fit_plane_ransac(pts, threshold=0.05, min_inliers=100,
                              orient='vertical', iterations=300, seed=1)
    assert res is not None
    normal, _, _ = res
    assert abs(normal[2]) < 0.35            # a vertical plane (wall)


def test_extract_building_surfaces_walls_in_floor_heavy_scan():
    # Three stacked floors (multi-storey) + one wall: plain extract_planes with a
    # small budget finds only floors; extract_building must still return a wall.
    pts = _floor_and_walls(floor_z_levels=(0.0, 3.0, 6.0))
    plain = be.extract_planes(pts, threshold=0.05, min_inliers=200,
                              min_remaining=200, max_planes=3, patch_cell=0.5)
    assert all(p['kind'] == 'horizontal' for p in plain)      # walls hidden
    building = be.extract_building(pts, threshold=0.05, floor_planes=3,
                                   wall_planes=3, min_inliers=200,
                                   patch_cell=0.5)
    kinds = {p['kind'] for p in building}
    assert 'vertical' in kinds and 'horizontal' in kinds       # both recovered


def test_extract_planes_separates_floor_and_wall():
    # Floor on z=0 (10x10) and a wall on x=0 (10 wide x 3 tall).
    fx, fy = _grid(40, 40, 5.0)
    floor = np.stack([fx, fy, np.zeros_like(fx)], axis=1)
    wy = np.linspace(-5, 5, 40)
    wz = np.linspace(0, 3, 20)
    wgy, wgz = np.meshgrid(wy, wz)
    wall = np.stack([np.zeros(wgy.size), wgy.ravel(), wgz.ravel()], axis=1)
    pts = np.vstack([floor, wall])

    planes = be.extract_planes(pts, threshold=0.05, min_inliers=200,
                               min_remaining=200, max_planes=6)
    kinds = sorted(pl['kind'] for pl in planes)
    assert 'horizontal' in kinds
    assert 'vertical' in kinds
    # at least the two dominant surfaces are recovered
    assert len([k for k in kinds if k in ('horizontal', 'vertical')]) >= 2


def test_extract_planes_indices_are_disjoint():
    gx, gy = _grid(40, 40, 5.0)
    floor = np.stack([gx, gy, np.zeros_like(gx)], axis=1)
    planes = be.extract_planes(floor, threshold=0.05, min_inliers=200,
                               min_remaining=200, max_planes=4)
    seen = np.concatenate([pl['indices'] for pl in planes]) if planes else \
        np.array([], dtype=int)
    assert len(seen) == len(np.unique(seen))       # no point claimed twice


# --------------------------------------------------------------------------- #
# per-plane mean colour
# --------------------------------------------------------------------------- #
def test_extract_planes_mean_colour():
    gx, gy = _grid(40, 40, 5.0)
    floor = np.stack([gx, gy, np.zeros_like(gx)], axis=1)
    # mostly-red with mild variation -> mean stays near pure red
    rgb = np.tile(np.array([[220, 10, 10]], np.uint8), (len(floor), 1))
    planes = be.extract_planes(floor, colors=rgb, threshold=0.05,
                               min_inliers=200, min_remaining=200, max_planes=2)
    assert planes
    col = planes[0]['color']
    assert col.dtype == np.uint8
    assert col[0] > 200 and col[1] < 30 and col[2] < 30


def test_extract_planes_without_colours_has_no_color_key():
    gx, gy = _grid(30, 30, 5.0)
    floor = np.stack([gx, gy, np.zeros_like(gx)], axis=1)
    planes = be.extract_planes(floor, threshold=0.05, min_inliers=200,
                               min_remaining=200, max_planes=2)
    assert planes and 'color' not in planes[0]


# --------------------------------------------------------------------------- #
# Manhattan wall regularization
# --------------------------------------------------------------------------- #
def _tilted_wall(angle_deg, length=6.0, height=3.0, n=40):
    """Build a vertical wall with its normal rotated about z from +x."""
    th = np.radians(angle_deg)
    normal = np.array([np.cos(th), np.sin(th), 0.0])
    along = np.array([-np.sin(th), np.cos(th), 0.0])   # in-plane horizontal
    s = np.linspace(-length / 2, length / 2, n)
    z = np.linspace(0, height, n // 2)
    gs, gz = np.meshgrid(s, z)
    pts = (gs.ravel()[:, None] * along + gz.ravel()[:, None] * np.array([0, 0, 1.]))
    return pts, normal


def test_principal_axis_recovers_building_orientation():
    # Two walls a few degrees off a 10-degree building axis (0 and 90 to it).
    pa, na = _tilted_wall(12.0)
    pb, nb = _tilted_wall(102.0)
    walls = [{'normal': na, 'indices': np.arange(len(pa))},
             {'normal': nb, 'indices': np.arange(len(pb))}]
    theta = np.degrees(be.principal_axis(walls))
    assert min(abs(theta - 12.0), abs(theta - 12.0 + 90)) < 3.0


def test_regularize_walls_makes_normals_orthogonal():
    pa, na = _tilted_wall(8.0)          # ~ +x wall, 8 deg off
    pb, nb = _tilted_wall(85.0)         # ~ +y wall, 5 deg off perpendicular
    pts = np.vstack([pa, pb])
    planes = [
        {'kind': 'vertical', 'normal': na, 'd': 0.0,
         'indices': np.arange(len(pa)), 'centroid': pa.mean(0),
         'corners': np.zeros((4, 3)), 'size': (6, 3), 'thickness': 0.1},
        {'kind': 'vertical', 'normal': nb, 'd': 0.0,
         'indices': np.arange(len(pa), len(pa) + len(pb)),
         'centroid': pb.mean(0), 'corners': np.zeros((4, 3)),
         'size': (6, 3), 'thickness': 0.1},
    ]
    reg = be.regularize_walls(planes, pts)
    n0, n1 = reg[0]['normal'], reg[1]['normal']
    assert abs(n0[2]) < 1e-9 and abs(n1[2]) < 1e-9       # perfectly vertical
    assert abs(n0.dot(n1)) < 1e-6                         # orthogonal walls


def test_regularize_leaves_slabs_untouched():
    slab = {'kind': 'horizontal', 'normal': np.array([0, 0, 1.]), 'd': 0.0,
            'indices': np.arange(4), 'centroid': np.zeros(3),
            'corners': np.eye(4)[:, :3], 'size': (2, 2), 'thickness': 0.2}
    pts = np.zeros((4, 3))
    out = be.regularize_walls([slab], pts)
    assert out[0] is slab                                # slab passed through


def test_adaptive_regularize_keeps_outlier_wall_when_snap_is_bad():
    # A long dominant x wall establishes the grid; a 20-degree wall should not
    # be forced onto it when that would substantially increase residual error.
    pa, na = _tilted_wall(0.0, length=12, n=100)
    pb, nb = _tilted_wall(20.0, length=6, n=40)
    pts = np.vstack([pa, pb + np.array([0., 5., 0.])])
    planes = [
        {'kind': 'vertical', 'normal': na, 'd': 0.,
         'indices': np.arange(len(pa)), 'corners': np.zeros((4, 3)),
         'size': (12., 3.), 'thickness': .1},
        {'kind': 'vertical', 'normal': nb, 'd': float(-(pb + [0, 5, 0]).mean(0).dot(nb)),
         'indices': np.arange(len(pa), len(pts)), 'corners': np.zeros((4, 3)),
         'size': (6., 3.), 'thickness': .1},
    ]
    out = be.regularize_walls(planes, pts, adaptive=True)
    assert out[1]['regularization']['decision'] == 'Keep observed'
    assert out[1]['regularization']['angle_change_deg'] < 1e-6


# --------------------------------------------------------------------------- #
# room reconstruction (IfcSpace)
# --------------------------------------------------------------------------- #
def _wall_plane(x0, y0, x1, y1, h=3.0):
    return {'kind': 'vertical',
            'corners': np.array([[x0, y0, 0.0], [x1, y1, 0.0],
                                 [x1, y1, h], [x0, y0, h]], float)}


def _rect_room_walls(w=6.0, length=4.0):
    return [_wall_plane(0, 0, w, 0), _wall_plane(w, 0, w, length),
            _wall_plane(w, length, 0, length),
            _wall_plane(0, length, 0, 0)]


def test_wall_footprint_segment_endpoints():
    p0, p1 = be._wall_footprint_segment(_wall_plane(1, 2, 7, 2))
    ends = sorted([tuple(np.round(p0, 3)), tuple(np.round(p1, 3))])
    assert ends == [(1.0, 2.0), (7.0, 2.0)]


def test_wall_graph_extracts_closed_room_cycle():
    graph = be.build_wall_graph(_rect_room_walls(6, 4))
    cycles = be.extract_room_cycles(graph)
    assert len(cycles) == 1
    assert cycles[0]['area'] == pytest.approx(24.0)
    assert cycles[0]['observed_boundary_ratio'] == pytest.approx(1.0)
    assert cycles[0]['extension_length'] == pytest.approx(0.0)
    assert cycles[0]['height'] == pytest.approx(3.0)


def test_wall_graph_intersections_close_small_endpoint_gaps():
    walls = [_wall_plane(0.2, 0, 5.8, 0), _wall_plane(6, 0.2, 6, 3.8),
             _wall_plane(5.8, 4, 0.2, 4), _wall_plane(0, 3.8, 0, 0.2)]
    graph = be.build_wall_graph(walls, intersection_gap=0.3,
                                merge_distance=0.05)
    cycles = be.extract_room_cycles(graph)
    assert len(cycles) == 1
    assert cycles[0]['area'] == pytest.approx(24.0)
    assert 0.85 < cycles[0]['observed_boundary_ratio'] < 1.0
    assert cycles[0]['extension_length'] == pytest.approx(1.6)


def test_wall_graph_does_not_invent_cycle_for_open_room():
    graph = be.build_wall_graph(_rect_room_walls(6, 4)[:3])
    assert be.extract_room_cycles(graph) == []


def test_wall_graph_splits_shared_wall_into_two_rooms():
    walls = _rect_room_walls(8, 4) + [_wall_plane(4, 0, 4, 4)]
    graph = be.build_wall_graph(walls)
    cycles = be.extract_room_cycles(graph)
    assert len(cycles) == 2
    assert [cycle['area'] for cycle in cycles] == pytest.approx([16.0, 16.0])
    shared = [edge for edge in graph['edges']
              if edge['wall_index'] == len(walls) - 1]
    assert len(shared) == 1


def test_wall_graph_supports_non_manhattan_room():
    walls = [_wall_plane(0, 0, 4, 0), _wall_plane(4, 0, 2, 3),
             _wall_plane(2, 3, 0, 0)]
    cycles = be.extract_room_cycles(be.build_wall_graph(walls))
    assert len(cycles) == 1
    assert cycles[0]['area'] == pytest.approx(6.0)


def test_wall_cycles_become_polygonal_room_candidates():
    walls = [_wall_plane(0, 0, 4, 0), _wall_plane(4, 0, 2, 3),
             _wall_plane(2, 3, 0, 0)]
    rooms = be.derive_room_candidates(walls)
    assert len(rooms) == 1
    assert rooms[0]['corners'].shape == (3, 3)
    assert rooms[0]['area'] == pytest.approx(6.0)
    assert rooms[0]['generation_method'] == 'LiDAR wall topology cycle'
    assert rooms[0]['topology_metrics'][
        'observed_boundary_ratio'] == pytest.approx(1.0)


def test_topology_optimizer_extends_only_cycle_backed_corners():
    walls = [_wall_plane(0.2, 0, 5.8, 0), _wall_plane(6, 0.2, 6, 3.8),
             _wall_plane(5.8, 4, 0.2, 4), _wall_plane(0, 3.8, 0, 0.2)]
    optimized, cycles = be.optimize_wall_topology(
        walls, intersection_gap=0.3, repair_gap=0.0)
    assert len(cycles) == 1
    assert all(wall.get('topology_adjustment') for wall in optimized)
    graph = be.build_wall_graph(optimized, intersection_gap=0.0)
    assert len(be.extract_room_cycles(graph)) == 1

    open_walls = walls[:3]
    optimized, cycles = be.optimize_wall_topology(
        open_walls, intersection_gap=0.3, repair_gap=0.0)
    assert cycles == []
    assert all('topology_adjustment' not in wall for wall in optimized)


def test_topology_optimizer_accepts_short_repair_that_closes_room():
    walls = [_wall_plane(0, 0, 2.7, 0), _wall_plane(3.3, 0, 6, 0),
             _wall_plane(6, 0, 6, 4), _wall_plane(6, 4, 0, 4),
             _wall_plane(0, 4, 0, 0)]
    optimized, cycles = be.optimize_wall_topology(
        walls, intersection_gap=0.1, repair_gap=0.7)
    repairs = [wall for wall in optimized if wall.get('synthetic')]
    assert len(repairs) == 1
    assert repairs[0]['topology_validation']['room_cycles_before'] == 0
    assert repairs[0]['topology_validation']['room_cycles_after'] == 1
    assert len(cycles) == 1
    assert cycles[0]['observed_boundary_ratio'] > 0.95


def test_topology_optimizer_rejects_weak_small_room_repair():
    walls = [_wall_plane(0, 0, 0.65, 0), _wall_plane(1.35, 0, 2, 0),
             _wall_plane(2, 0, 2, 1), _wall_plane(2, 1, 0, 1),
             _wall_plane(0, 1, 0, 0)]
    optimized, cycles = be.optimize_wall_topology(
        walls, intersection_gap=0.1, repair_gap=0.75,
        min_room_area=0.5, min_observed_boundary=0.85)
    assert not any(wall.get('synthetic') for wall in optimized)
    assert cycles == []


def test_snap_wall_corners_closes_small_corner_gap():
    walls = [_wall_plane(0, 0, 3.8, 0), _wall_plane(4, 0.2, 4, 3)]
    snapped = be.snap_wall_corners(walls, max_gap=0.3)
    ends = [be._wall_footprint_segment(w) for w in snapped]
    assert any(np.allclose(p, [4, 0]) for p in ends[0])
    assert any(np.allclose(p, [4, 0]) for p in ends[1])


def test_snap_wall_corners_does_not_join_distant_walls():
    walls = [_wall_plane(0, 0, 3, 0), _wall_plane(4, 1, 4, 3)]
    snapped = be.snap_wall_corners(walls, max_gap=0.5)
    for before, after in zip(walls, snapped):
        assert np.allclose(before['corners'], after['corners'])


def test_corner_snap_turns_gapped_outline_into_room():
    walls = [_wall_plane(0.2, 0, 5.8, 0), _wall_plane(6, 0.2, 6, 3.8),
             _wall_plane(5.8, 4, 0.2, 4), _wall_plane(0, 3.8, 0, 0.2)]
    assert be.reconstruct_rooms(walls, cell=0.1, close_iter=0) == []
    snapped = be.snap_wall_corners(walls, max_gap=0.3)
    assert len(be.reconstruct_rooms(snapped, cell=0.1, close_iter=0)) == 1


def test_html_report_contains_plan_diagnosis_and_links(tmp_path):
    walls = _rect_room_walls(6, 4)[:3]
    report = be.write_html_report(walls, tmp_path / 'report.html',
                                  source='scan.ply', ifc_path='model.ifc',
                                  settings={'Corner snap': 0.5})
    text = report.read_text(encoding='utf-8')
    assert '<svg' in text and 'class="wall"' in text
    assert '未接続の壁端' in text
    assert '壁が閉ループを形成していない' in text
    assert 'model.ifc' in text and 'Corner snap' in text


def test_html_report_excludes_rejected_room_from_accepted_count(tmp_path,
                                                                monkeypatch):
    candidate = {
        'corners': np.array([[0., 0., 0.], [1., 0., 0.],
                             [1., 1., 0.], [0., 1., 0.]]),
        'height': 3.0, 'area': 1.0, 'name': 'Tiny pocket', 'number': '1',
    }
    monkeypatch.setattr(be, 'reconstruct_rooms',
                        lambda planes, **kwargs: [candidate])
    report = be.write_html_report(_rect_room_walls(4, 3)[:3],
                                  tmp_path / 'rejected.html')
    text = report.read_text(encoding='utf-8')
    assert '<b>部屋</b><span>0</span>' in text
    assert '<b>Rejected部屋候補</b><span>1</span>' in text
    assert '品質基準を満たさずRejected' in text


def test_html_report_shows_element_fit_columns(tmp_path):
    points = np.array([[0.5, 0.5, 0.01], [1.5, 0.5, 0.01],
                       [0.5, 1.5, 0.01], [1.5, 1.5, 0.01]])
    plane = _fit_plane(points)
    plane['corners'] = np.array([[0., 0., 0.], [2., 0., 0.],
                                 [2., 2., 0.], [0., 2., 0.]])
    plane = be.add_element_fit_metrics([plane], points, cell=1.0)[0]
    report = be.write_html_report([plane], tmp_path / 'fit.html')
    text = report.read_text(encoding='utf-8')
    assert '<th>Coverage</th>' in text
    assert '<th>Distribution</th>' in text
    assert '<th>Distance P95 [m]</th>' in text
    assert '<td>100%</td><td>100%</td>' in text


def test_bim_metrics_manifest_is_machine_readable_and_deterministic(tmp_path):
    planes = _rect_room_walls(6, 4)
    metrics = be.build_bim_metrics(planes, source='synthetic-room')
    assert metrics['schema_version'] == 1
    assert metrics['summary']['observed_walls'] == 4
    assert metrics['summary']['synthetic_walls'] == 0
    assert metrics['summary']['accepted_rooms'] == 1
    assert metrics['summary']['rejected_room_candidates'] == 0
    assert metrics['rooms'][0]['generation_method'] == 'LiDAR wall topology cycle'
    first = be.write_bim_metrics(planes, tmp_path / 'first.json',
                                 source='synthetic-room')
    second = be.write_bim_metrics(planes, tmp_path / 'second.json',
                                  source='synthetic-room')
    assert first.read_bytes() == second.read_bytes()
    assert json.loads(first.read_text())['summary']['accepted_rooms'] == 1


def test_wall_repair_closes_short_mutual_gap():
    walls = [_wall_plane(0, 0, 5.6, 0), _wall_plane(6, 0.4, 6, 4),
             _wall_plane(6, 4, 0, 4), _wall_plane(0, 4, 0, 0)]
    repairs = be.propose_wall_repairs(walls, min_gap=0.3, max_gap=0.7)
    assert len(repairs) == 1
    assert repairs[0]['synthetic']
    assert 0.5 < repairs[0]['size'][0] < 0.6


def test_wall_repair_does_not_bridge_door_sized_gap():
    walls = [_wall_plane(0, 0, 2.5, 0), _wall_plane(3.5, 0, 6, 0)]
    assert be.propose_wall_repairs(walls, max_gap=0.75) == []


def test_plane_confidence_rewards_clean_dense_observation():
    rng = np.random.default_rng(9)
    x, y = np.meshgrid(np.linspace(0, 4, 50), np.linspace(0, 3, 40))
    pts = np.c_[x.ravel(), y.ravel(), rng.normal(0, 0.003, x.size)]
    plane = {'kind': 'horizontal', 'normal': np.array([0., 0., 1.]), 'd': 0.,
             'indices': np.arange(len(pts)), 'size': (4., 3.)}
    scored = be.add_plane_confidence([plane], pts)[0]
    assert scored['confidence'] >= 75
    assert scored['confidence_level'] == 'High'
    assert scored['confidence_metrics']['plane_rmse_m'] < 0.01


def _fit_plane(points):
    return {
        'kind': 'horizontal', 'normal': np.array([0., 0., 1.]), 'd': 0.,
        'indices': np.arange(len(points)),
        'corners': np.array([[0., 0., 0.], [4., 0., 0.],
                             [4., 3., 0.], [0., 3., 0.]]),
        'centroid': np.array([2., 1.5, 0.]), 'size': (4., 3.),
        'thickness': 0.01,
    }


def test_element_fit_measures_full_even_surface():
    x, y = np.meshgrid(np.arange(0.5, 4., 1.), np.arange(0.5, 3., 1.))
    points = np.c_[x.ravel(), y.ravel(), np.full(x.size, 0.01)]
    fit = be.evaluate_element_fit(_fit_plane(points), points, cell=1.0,
                                  distance_threshold=0.1)
    assert fit['total_cells'] == 12
    assert fit['occupied_cells'] == 12
    assert fit['coverage_ratio'] == pytest.approx(1.0)
    assert fit['distribution_ratio'] == pytest.approx(1.0)
    assert fit['distance_rmse_m'] == pytest.approx(0.01)
    assert fit['distance_p95_m'] == pytest.approx(0.01)


def test_element_fit_separates_coverage_distribution_and_distance():
    x, y = np.meshgrid(np.arange(0.5, 4., 1.), np.arange(0.5, 3., 1.))
    base = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    clustered = np.repeat(base[:1], 120, axis=0)
    points = np.vstack([base, clustered])
    fit = be.evaluate_element_fit(_fit_plane(points), points, cell=1.0)
    assert fit['coverage_ratio'] == pytest.approx(1.0)
    assert fit['distribution_ratio'] < 0.4
    assert fit['distance_rmse_m'] == 0.0

    partial = base[:3].copy()
    partial[:, 2] = 0.2
    fit = be.evaluate_element_fit(_fit_plane(partial), partial, cell=1.0,
                                  distance_threshold=0.1)
    assert fit['coverage_ratio'] == 0.0
    assert fit['distance_rmse_m'] == pytest.approx(0.2)


def test_element_fit_marks_synthetic_element_as_unobserved():
    plane = _fit_plane(np.empty((0, 3)))
    plane['synthetic'] = True
    fit = be.evaluate_element_fit(plane, np.empty((0, 3)))
    assert fit['coverage_ratio'] == 0.0
    assert fit['distribution_ratio'] == 0.0
    assert fit['distance_rmse_m'] is None
    assert fit['matched_points'] == 0


def test_synthetic_plane_confidence_is_low_and_inferred():
    repair = {'kind': 'vertical', 'normal': np.array([1., 0., 0.]), 'd': 0.,
              'indices': np.array([], int), 'size': (0.5, 3.), 'synthetic': True}
    scored = be.add_plane_confidence([repair], np.empty((0, 3)))[0]
    assert scored['confidence'] == 25.0
    assert scored['confidence_level'] == 'Low'
    assert scored['provenance'] == 'inferred'


def test_wall_local_quality_finds_bad_half():
    rng = np.random.default_rng(12)
    x = np.linspace(0, 4, 800)
    pts = np.c_[x, rng.normal(0, 0.003, len(x)), np.linspace(0, 3, len(x))]
    pts[x >= 2, 1] += rng.normal(0, 0.35, (x >= 2).sum())
    wall = _wall_plane(0, 0, 4, 0)
    wall.update(normal=np.array([0., 1., 0.]), d=0., indices=np.arange(len(pts)),
                size=(4., 3.))
    scored = be.add_wall_quality_segments([wall], pts, segment_length=2.0)[0]
    assert len(scored['quality_segments']) == 2
    assert scored['quality_segments'][0]['score'] > scored['quality_segments'][1]['score']
    assert 'SLAMドリフト' in scored['quality_segments'][1]['recommendation']


def test_local_quality_separates_observation_from_regularized_model():
    x = np.linspace(0, 4, 800)
    pts = np.c_[x, np.zeros_like(x), np.linspace(0, 3, len(x))]
    wall = _wall_plane(0, 0, 4, 0)
    wall.update(normal=np.array([0., 1., 0.]), d=-0.3,
                observed_normal=np.array([0., 1., 0.]), observed_d=0.,
                indices=np.arange(len(pts)), size=(4., 3.))
    seg = be.add_wall_quality_segments([wall], pts, segment_length=4.0)[0][
        'quality_segments'][0]
    assert seg['observation_rmse_m'] < 1e-9
    assert abs(seg['model_rmse_m'] - 0.3) < 1e-9
    assert seg['level'] == 'High'
    assert '正規化を緩和' in seg['recommendation']


def test_reconstruct_single_closed_room():
    rooms = be.reconstruct_rooms(_rect_room_walls(6, 4), cell=0.2, close_iter=1,
                                 min_cells=25)
    assert len(rooms) == 1
    # area a bit under 6x4 (walls eat the border); comfortably within range
    assert 15.0 < rooms[0]['area'] < 24.0
    assert abs(rooms[0]['height'] - 3.0) < 1e-6
    assert rooms[0]['name'] == 'Room 1'
    assert rooms[0]['number'] == '1'


def test_reconstruct_two_rooms_sharing_a_wall():
    # Two 4x4 rooms side by side sharing the x=4 wall.
    walls = [
        _wall_plane(0, 0, 8, 0), _wall_plane(0, 4, 8, 4),       # long top/bottom
        _wall_plane(0, 0, 0, 4), _wall_plane(8, 0, 8, 4),       # far ends
        _wall_plane(4, 0, 4, 4),                                # shared middle
    ]
    rooms = be.reconstruct_rooms(walls, cell=0.2, close_iter=1, min_cells=25)
    assert len(rooms) == 2


def test_assess_room_confirms_observed_closed_room():
    walls = _rect_room_walls(6, 4)
    room = be.reconstruct_rooms(walls, cell=0.2, close_iter=1)[0]
    assessed = be.assess_rooms([room], walls)[0]
    assert assessed['room_status'] == 'Confirmed'
    assert assessed['room_confidence'] >= 75
    assert assessed['validation_metrics']['stable_without_repairs']


def test_assess_room_rejects_tiny_synthetic_pocket():
    walls = _rect_room_walls(1, 1)
    walls[0]['synthetic'] = True
    room = {'corners': np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0.]]),
            'height': 1.5, 'area': 1.0}
    assessed = be.assess_rooms([room], walls)[0]
    assert assessed['room_status'] == 'Rejected'
    assert '寸法または面積が非典型' in assessed['validation_reasons']


def test_open_room_leaks_and_is_not_counted():
    # Drop one wall entirely -> the interior connects to the exterior -> no room.
    walls = _rect_room_walls(6, 4)[:3]
    rooms = be.reconstruct_rooms(walls, cell=0.2, close_iter=1, min_cells=25)
    assert rooms == []


def test_reconstruct_bridges_a_doorway_gap():
    # A wall with a doorway: two segments leaving a 0.4 m gap; closing bridges it.
    walls = [_wall_plane(0, 0, 2.6, 0), _wall_plane(3.0, 0, 6, 0),   # gap 2.6..3.0
             _wall_plane(6, 0, 6, 4), _wall_plane(6, 4, 0, 4),
             _wall_plane(0, 4, 0, 0)]
    assert len(be.reconstruct_rooms(walls, cell=0.2, close_iter=2,
                                    min_cells=25)) == 1


# --------------------------------------------------------------------------- #
# detect_openings (doors / windows)
# --------------------------------------------------------------------------- #
def _wall_with_holes(holes, w=4.0, h=3.0, step=0.05):
    """
    Build dense x=0 wall points minus rectangular holes.

    Each hole is represented by ``(y0, y1, z0, z1)``.
    """
    ys = np.arange(0, w + 1e-9, step)
    zs = np.arange(0, h + 1e-9, step)
    gy, gz = np.meshgrid(ys, zs)
    gy = gy.ravel()
    gz = gz.ravel()
    keep = np.ones(gy.shape, bool)
    for y0, y1, z0, z1 in holes:
        keep &= ~((gy >= y0) & (gy <= y1) & (gz >= z0) & (gz <= z1))
    gy, gz = gy[keep], gz[keep]
    return np.stack([np.zeros_like(gy), gy, gz], axis=1)


def test_detect_window_opening():
    wall = _wall_with_holes([(1.0, 1.8, 1.0, 2.3)])       # 0.8 x 1.3 window
    ops = be.detect_openings(wall, [1, 0, 0], cell=0.15)
    assert len(ops) == 1
    assert ops[0]['kind'] == 'window'
    w, h = ops[0]['size']
    assert 0.6 < w < 1.1 and 1.1 < h < 1.6
    assert ops[0]['sill'] > 0.5                            # not at the floor


def test_detect_door_opening():
    wall = _wall_with_holes([(2.4, 3.3, 0.0, 2.1)])       # reaches the base
    ops = be.detect_openings(wall, [1, 0, 0], cell=0.15)
    assert len(ops) == 1
    assert ops[0]['kind'] == 'door'
    assert ops[0]['sill'] < 0.2                            # opening at the base


def test_detect_window_and_door_together():
    wall = _wall_with_holes([(0.8, 1.6, 1.0, 2.3), (2.6, 3.4, 0.0, 2.1)])
    ops = be.detect_openings(wall, [1, 0, 0], cell=0.15)
    kinds = sorted(o['kind'] for o in ops)
    assert kinds == ['door', 'window']


def test_solid_wall_has_no_openings():
    wall = _wall_with_holes([])
    assert be.detect_openings(wall, [1, 0, 0], cell=0.15) == []


def test_ragged_edge_is_not_an_opening():
    # Only the left half of the wall is scanned: the empty right half touches the
    # grid border, so it must not be reported as an opening.
    wall = _wall_with_holes([(2.2, 4.0, 0.0, 3.0)])
    assert be.detect_openings(wall, [1, 0, 0], cell=0.15) == []


# --------------------------------------------------------------------------- #
# denoise + morphological closing
# --------------------------------------------------------------------------- #
def test_voxel_density_filter_drops_isolated_noise():
    # A dense 2x2x2 m block plus a handful of far-away lone floaters.
    gx, gy, gz = np.meshgrid(np.linspace(0, 2, 20), np.linspace(0, 2, 20),
                             np.linspace(0, 2, 20))
    block = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    noise = np.array([[50, 50, 50], [-40, 10, 5], [100, 0, 0]], float)
    pts = np.vstack([block, noise])
    keep = be.voxel_density_filter(pts, voxel=0.3, min_count=4)
    assert keep[:len(block)].all()          # dense block kept
    assert not keep[len(block):].any()      # lone floaters dropped


def test_binary_close_bridges_small_gap():
    grid = np.ones((10, 10), bool)
    grid[4:6, 4:6] = False                  # a 2x2 hole (patchy scan)
    closed = be._binary_close(grid, iters=1)
    assert closed.all()                     # small gap filled


def test_binary_close_keeps_large_hole():
    grid = np.ones((14, 14), bool)
    grid[3:11, 3:11] = False                # a big 8x8 hole (real opening)
    closed = be._binary_close(grid, iters=1)
    assert not closed[5:9, 5:9].any()       # centre of the big hole survives


def test_openings_closing_bridges_patchy_wall():
    # A wall with a real window but also randomly-missing single cells (patchy).
    rng = np.random.default_rng(3)
    wall = _wall_with_holes([(1.0, 1.8, 1.0, 2.3)])
    drop = rng.random(len(wall)) < 0.35        # 35% random dropout -> speckle
    wall = wall[~drop]
    # Without closing the speckle creates spurious tiny holes; with closing the
    # single real window is recovered cleanly.
    ops = be.detect_openings(wall, [1, 0, 0], cell=0.15, close_iter=1)
    assert len(ops) == 1 and ops[0]['kind'] == 'window'


# --------------------------------------------------------------------------- #
# IFC writer + surface colour (needs ifcopenshell)
# --------------------------------------------------------------------------- #
def test_write_ifc_applies_surface_colour(tmp_path):
    ifcopenshell = pytest.importorskip('ifcopenshell')
    plane = {
        'normal': np.array([0.0, 0.0, 1.0]),
        'd': 0.0,
        'indices': np.arange(4),
        'centroid': np.zeros(3),
        'kind': 'horizontal',
        'corners': np.array([[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]], float),
        'size': (2.0, 2.0),
        'thickness': 0.2,
        'color': np.array([255, 128, 0], np.uint8),
    }
    out = be.write_ifc([plane], tmp_path / 'c.ifc')
    m = ifcopenshell.open(str(out))
    assert len(m.by_type('IfcSlab')) == 1
    styles = m.by_type('IfcSurfaceStyleShading')
    assert len(styles) == 1
    col = styles[0].SurfaceColour
    assert abs(col.Red - 1.0) < 1e-6
    assert abs(col.Green - 128 / 255) < 1e-6
    assert abs(col.Blue - 0.0) < 1e-6
    assert len(m.by_type('IfcStyledItem')) == 1


def test_write_ifc_emits_door_and_window(tmp_path):
    ifcopenshell = pytest.importorskip('ifcopenshell')
    wall = _wall_with_holes([(0.8, 1.6, 1.0, 2.3), (2.6, 3.4, 0.0, 2.1)])
    planes = be.extract_planes(wall, threshold=0.05, min_inliers=300,
                               min_remaining=300, max_planes=2, patch_cell=0.3,
                               find_openings=True)
    walls = [p for p in planes if p['kind'] == 'vertical']
    assert walls and walls[0].get('openings')
    out = be.write_ifc(planes, tmp_path / 'w.ifc')
    m = ifcopenshell.open(str(out))
    assert len(m.by_type('IfcDoor')) == 1
    assert len(m.by_type('IfcWindow')) == 1
    assert len(m.by_type('IfcOpeningElement')) == 2
    assert len(m.by_type('IfcRelVoidsElement')) == 2      # openings void the wall
    assert len(m.by_type('IfcRelFillsElement')) == 2      # door/window fill them


def test_write_ifc_space_has_room_properties(tmp_path):
    ifcopenshell = pytest.importorskip('ifcopenshell')
    room = {
        'corners': np.array([[0, 0, 0], [5, 0, 0],
                             [5, 4, 0], [0, 4, 0]], float),
        'height': 2.8,
        'area': 19.5,
        'name': 'Meeting Room',
        'number': 'A-101',
        'storey': 'Ground Floor',
        'generation_method': 'Unit test',
    }
    out = be.write_ifc([], tmp_path / 'space.ifc', rooms=[room])
    model = ifcopenshell.open(str(out))
    space = model.by_type('IfcSpace')[0]
    assert space.Name == 'Meeting Room'
    psets = ifcopenshell.util.element.get_psets(space)
    props = psets['Pset_LIDARSLAMSpace']
    assert props['RoomNumber'] == 'A-101'
    assert abs(props['NetFloorArea'] - 19.5) < 1e-6
    assert abs(props['CeilingHeight'] - 2.8) < 1e-6
    assert props['StoreyName'] == 'Ground Floor'
    assert props['GenerationMethod'] == 'Unit test'


def test_write_ifc_marks_synthetic_repair_wall(tmp_path):
    ifcopenshell = pytest.importorskip('ifcopenshell')
    wall = be.propose_wall_repairs([
        _wall_plane(0, 0, 2, 0), _wall_plane(2.4, 0, 4, 0),
    ], min_gap=0.3, max_gap=0.5)[0]
    out = be.write_ifc([wall], tmp_path / 'repair.ifc')
    model = ifcopenshell.open(str(out))
    props = ifcopenshell.util.element.get_psets(model.by_type('IfcWall')[0])
    repair = props['Pset_LIDARSLAMRepair']
    assert repair['IsSynthetic'] is True
    assert repair['Reason'] == 'mutually-nearest dangling wall ends'


def test_write_ifc_persists_element_fit_metrics(tmp_path):
    ifcopenshell = pytest.importorskip('ifcopenshell')
    points = np.array([[0.5, 0.5, 0.01], [1.5, 0.5, 0.01],
                       [0.5, 1.5, 0.01], [1.5, 1.5, 0.01]])
    plane = _fit_plane(points)
    plane['corners'] = np.array([[0., 0., 0.], [2., 0., 0.],
                                 [2., 2., 0.], [0., 2., 0.]])
    plane['size'] = (2., 2.)
    plane = be.add_element_fit_metrics([plane], points, cell=1.0)[0]
    out = be.write_ifc([plane], tmp_path / 'fit.ifc')
    model = ifcopenshell.open(str(out))
    props = ifcopenshell.util.element.get_psets(model.by_type('IfcSlab')[0])
    fit = props['Pset_LIDARSLAMElementFit']
    assert fit['CoverageRatio'] == pytest.approx(1.0)
    assert fit['DistributionRatio'] == pytest.approx(1.0)
    assert fit['DistanceRMSE'] == pytest.approx(0.01)
