#!/usr/bin/env python3
"""Measure accepted loop constraints against optimized g2o vertices."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def quaternion_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=float)
    norm = float(np.linalg.norm(q))
    if norm <= 0.0:
        raise ValueError('zero-norm quaternion')
    q /= norm
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pose_matrix(values: list[float]) -> np.ndarray:
    if len(values) < 7:
        raise ValueError('SE3 pose needs tx ty tz qx qy qz qw')
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = quaternion_matrix(*values[3:7])
    transform[:3, 3] = values[:3]
    return transform


def rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def parse_g2o(path: Path) -> tuple[dict[int, np.ndarray], list[dict[str, Any]]]:
    vertices: dict[int, np.ndarray] = {}
    edges: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
        parts = line.split()
        if not parts or parts[0].startswith('#'):
            continue
        try:
            if parts[0] == 'VERTEX_SE3:QUAT':
                vertices[int(parts[1])] = pose_matrix([float(v) for v in parts[2:9]])
            elif parts[0] == 'EDGE_SE3:QUAT':
                edges.append({
                    'from': int(parts[1]),
                    'to': int(parts[2]),
                    'measurement': pose_matrix([float(v) for v in parts[3:10]]),
                    'line_number': line_number,
                })
        except (IndexError, ValueError) as exc:
            raise ValueError(f'invalid g2o record at line {line_number}: {exc}') from exc
    return vertices, edges


def analyze(path: Path, adjacency_window: int = 20) -> dict[str, Any]:
    vertices, edges = parse_g2o(path)
    return analyze_edges(
        vertices, edges, adjacency_window=adjacency_window,
        source={'pose_graph_path': str(path.resolve())},
    )


def parse_tum_vertices(path: Path) -> dict[int, np.ndarray]:
    return {index: pose for index, (_, pose) in enumerate(parse_tum_poses(path))}


def parse_tum_poses(path: Path) -> list[tuple[float, np.ndarray]]:
    poses: list[tuple[float, np.ndarray]] = []
    for index, line in enumerate(path.read_text(encoding='utf-8').splitlines()):
        parts = line.split()
        if not parts or parts[0].startswith('#'):
            continue
        if len(parts) < 8:
            raise ValueError(f'invalid TUM record at line {index + 1}')
        poses.append((float(parts[0]), pose_matrix([float(v) for v in parts[1:8]])))
    return poses


def parse_loop_edges_csv(path: Path) -> list[dict[str, Any]]:
    edges: list[dict[str, Any]] = []
    with path.open(encoding='utf-8', newline='') as stream:
        for line_number, row in enumerate(csv.DictReader(stream), 2):
            try:
                edges.append({
                    'from': int(row['from']),
                    'to': int(row['to']),
                    'measurement': pose_matrix([
                        float(row[key]) for key in ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw')
                    ]),
                    'line_number': line_number,
                    'fitness': float(row['fitness']),
                })
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f'invalid loop CSV record at line {line_number}: {exc}') from exc
    return edges


def analyze_offline(trajectory: Path, loop_edges_csv: Path) -> dict[str, Any]:
    return analyze_edges(
        parse_tum_vertices(trajectory), parse_loop_edges_csv(loop_edges_csv),
        adjacency_window=-1,
        source={
            'trajectory_path': str(trajectory.resolve()),
            'loop_edges_csv_path': str(loop_edges_csv.resolve()),
        },
    )


def analyze_offline_reference(
    trajectory: Path,
    reference_trajectory: Path,
    loop_edges_csv: Path,
    max_time_diff: float = 0.05,
) -> dict[str, Any]:
    indexed_poses = parse_tum_poses(trajectory)
    reference_poses = parse_tum_poses(reference_trajectory)
    edges = parse_loop_edges_csv(loop_edges_csv)
    reference_vertices: dict[int, np.ndarray] = {}
    if reference_poses:
        reference_times = np.array([stamp for stamp, _ in reference_poses])
        for index in {edge[key] for edge in edges for key in ('from', 'to')}:
            if index < 0 or index >= len(indexed_poses):
                continue
            stamp = indexed_poses[index][0]
            insertion = int(np.searchsorted(reference_times, stamp))
            choices = [candidate for candidate in (insertion - 1, insertion)
                       if 0 <= candidate < len(reference_poses)]
            if not choices:
                continue
            nearest = min(choices, key=lambda candidate: abs(reference_times[candidate] - stamp))
            if abs(reference_times[nearest] - stamp) <= max_time_diff:
                reference_vertices[index] = reference_poses[nearest][1]
    return analyze_edges(
        reference_vertices, edges, adjacency_window=-1, pose_label='reference',
        source={
            'trajectory_path': str(trajectory.resolve()),
            'reference_trajectory_path': str(reference_trajectory.resolve()),
            'loop_edges_csv_path': str(loop_edges_csv.resolve()),
            'max_time_diff': max_time_diff,
        },
    )


def analyze_edges(
    vertices: dict[int, np.ndarray],
    edges: list[dict[str, Any]],
    *,
    adjacency_window: int,
    source: dict[str, Any],
    pose_label: str = 'optimized',
) -> dict[str, Any]:
    loop_rows: list[dict[str, Any]] = []
    missing_vertices = 0
    for edge in edges:
        index_gap = abs(edge['to'] - edge['from'])
        if adjacency_window >= 0 and index_gap <= adjacency_window:
            continue
        if edge['from'] not in vertices or edge['to'] not in vertices:
            missing_vertices += 1
            continue
        predicted = np.linalg.inv(vertices[edge['from']]) @ vertices[edge['to']]
        error = np.linalg.inv(edge['measurement']) @ predicted
        row = {
            'from_index': edge['from'],
            'to_index': edge['to'],
            'index_gap': index_gap,
            'translation_residual_m': float(np.linalg.norm(error[:3, 3])),
            'rotation_residual_deg': rotation_angle_deg(error[:3, :3]),
            f'{pose_label}_pair_distance_m': float(np.linalg.norm(predicted[:3, 3])),
            'measurement_translation_m': float(np.linalg.norm(edge['measurement'][:3, 3])),
            'line_number': edge['line_number'],
        }
        if 'fitness' in edge:
            row['fitness'] = edge['fitness']
        loop_rows.append(row)
    translation = [row['translation_residual_m'] for row in loop_rows]
    rotation = [row['rotation_residual_deg'] for row in loop_rows]
    return {
        'schema_version': 1,
        **source,
        'adjacency_window': adjacency_window,
        'vertex_count': len(vertices),
        'edge_count': len(edges),
        'loop_edge_count': len(loop_rows),
        'loop_edges_missing_vertices': missing_vertices,
        'loop_edges': loop_rows,
        'summary': {
            'translation_residual_mean_m': float(np.mean(translation)) if translation else None,
            'translation_residual_max_m': max(translation) if translation else None,
            'rotation_residual_mean_deg': float(np.mean(rotation)) if rotation else None,
            'rotation_residual_max_deg': max(rotation) if rotation else None,
        },
        'status': 'PASS' if loop_rows and not missing_vertices else 'FAIL',
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Measure optimized residuals of long-baseline EDGE_SE3 constraints.'
    )
    parser.add_argument('pose_graph', type=Path, nargs='?')
    parser.add_argument('--trajectory', type=Path)
    parser.add_argument('--loop-edges-csv', type=Path)
    parser.add_argument('--reference-trajectory', type=Path)
    parser.add_argument('--max-time-diff', type=float, default=0.05)
    parser.add_argument('--adjacency-window', type=int, default=20)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.adjacency_window < 0:
        parser.error('--adjacency-window must be >= 0')
    if args.max_time_diff < 0.0:
        parser.error('--max-time-diff must be >= 0')
    offline_mode = args.trajectory is not None or args.loop_edges_csv is not None
    if offline_mode and not (args.trajectory and args.loop_edges_csv):
        parser.error('--trajectory and --loop-edges-csv must be provided together')
    if offline_mode and args.pose_graph:
        parser.error('pose_graph cannot be combined with offline artifact arguments')
    if args.reference_trajectory and not offline_mode:
        parser.error('--reference-trajectory requires offline artifact arguments')
    if not offline_mode and not args.pose_graph:
        parser.error('pose_graph or offline artifact arguments are required')
    try:
        if offline_mode:
            if args.reference_trajectory:
                report = analyze_offline_reference(
                    args.trajectory.expanduser().resolve(),
                    args.reference_trajectory.expanduser().resolve(),
                    args.loop_edges_csv.expanduser().resolve(),
                    args.max_time_diff,
                )
            else:
                report = analyze_offline(
                    args.trajectory.expanduser().resolve(),
                    args.loop_edges_csv.expanduser().resolve(),
                )
        else:
            report = analyze(args.pose_graph.expanduser().resolve(), args.adjacency_window)
    except (OSError, ValueError) as exc:
        parser.exit(2, f'failed to analyze pose graph: {exc}\n')
    payload = json.dumps(report, indent=2, sort_keys=True) + '\n'
    if args.output:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(payload, encoding='utf-8')
    print(payload, end='')
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
