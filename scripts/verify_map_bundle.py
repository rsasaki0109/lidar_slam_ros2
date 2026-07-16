#!/usr/bin/env python3
"""Verify the self-describing output produced by graph_based_slam /map_save."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys

import yaml


REQUIRED_ARTIFACTS = {
    'full_map': 'file',
    'pointcloud_map': 'directory',
    'trajectory': 'file',
    'pose_graph': 'file',
    'loop_edges': 'file',
    'projector_info': 'file',
}


class BundleVerifier:
    """Validate artifact presence, safe paths, schemas, and cross-file counts."""

    def __init__(self, bundle_dir: str | Path):
        self.bundle_dir = Path(bundle_dir).resolve()
        self.passes: list[str] = []
        self.failures: list[str] = []

    def ok(self, message: str) -> None:
        self.passes.append(message)

    def fail(self, message: str) -> None:
        self.failures.append(message)

    def _artifact_path(self, value: object, name: str) -> Path | None:
        if not isinstance(value, str) or not value:
            self.fail(f"artifact '{name}' must be a non-empty relative path")
            return None
        relative = Path(value)
        if relative.is_absolute():
            self.fail(f"artifact '{name}' must not be an absolute path: {value}")
            return None
        candidate = (self.bundle_dir / relative).resolve()
        try:
            candidate.relative_to(self.bundle_dir)
        except ValueError:
            self.fail(f"artifact '{name}' escapes the bundle directory: {value}")
            return None
        return candidate

    def run(self) -> bool:
        manifest_path = self.bundle_dir / 'map_bundle.yaml'
        if not manifest_path.is_file():
            self.fail(f'manifest not found: {manifest_path}')
            return False
        try:
            manifest = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
        except (OSError, yaml.YAMLError) as error:
            self.fail(f'cannot parse map_bundle.yaml: {error}')
            return False
        if not isinstance(manifest, dict):
            self.fail('map_bundle.yaml must contain a mapping')
            return False
        if manifest.get('format_version') != 1:
            self.fail(f"unsupported format_version: {manifest.get('format_version')!r}")

        submap_count = manifest.get('submap_count')
        loop_edge_count = manifest.get('loop_edge_count')
        if not isinstance(submap_count, int) or isinstance(submap_count, bool) or submap_count <= 0:
            self.fail(f'submap_count must be a positive integer: {submap_count!r}')
        if not isinstance(loop_edge_count, int) or isinstance(loop_edge_count, bool) or loop_edge_count < 0:
            self.fail(f'loop_edge_count must be a non-negative integer: {loop_edge_count!r}')

        artifacts = manifest.get('artifacts')
        if not isinstance(artifacts, dict):
            self.fail('artifacts must contain a mapping')
            return False
        paths: dict[str, Path] = {}
        for name, expected_type in REQUIRED_ARTIFACTS.items():
            path = self._artifact_path(artifacts.get(name), name)
            if path is None:
                continue
            paths[name] = path
            exists = path.is_file() if expected_type == 'file' else path.is_dir()
            if not exists:
                self.fail(f"artifact '{name}' is not a {expected_type}: {path}")

        if 'full_map' in paths and paths['full_map'].is_file():
            if paths['full_map'].stat().st_size == 0:
                self.fail("artifact 'full_map' is empty")
        pointcloud_dir = paths.get('pointcloud_map')
        if pointcloud_dir is not None and pointcloud_dir.is_dir():
            if not (pointcloud_dir / 'pointcloud_map_metadata.yaml').is_file():
                self.fail('pointcloud_map_metadata.yaml is missing from pointcloud_map')

        trajectory = paths.get('trajectory')
        if trajectory is not None and trajectory.is_file() and isinstance(submap_count, int):
            rows = [line.split() for line in trajectory.read_text(encoding='utf-8').splitlines() if line.strip()]
            if len(rows) != submap_count:
                self.fail(f'trajectory rows {len(rows)} != submap_count {submap_count}')
            for index, row in enumerate(rows, start=1):
                try:
                    valid = len(row) == 8 and all(float(value) == float(value) for value in row)
                except ValueError:
                    valid = False
                if not valid:
                    self.fail(f'invalid TUM trajectory row {index}')
                    break

        pose_graph = paths.get('pose_graph')
        if pose_graph is not None and pose_graph.is_file() and isinstance(submap_count, int):
            vertices = sum(
                line.startswith('VERTEX_SE3:QUAT ')
                for line in pose_graph.read_text(encoding='utf-8').splitlines()
            )
            if vertices != submap_count:
                self.fail(f'pose graph vertices {vertices} != submap_count {submap_count}')

        loop_edges = paths.get('loop_edges')
        if loop_edges is not None and loop_edges.is_file() and isinstance(loop_edge_count, int):
            try:
                with loop_edges.open(newline='', encoding='utf-8') as stream:
                    reader = csv.DictReader(stream)
                    expected_fields = ['from', 'to', 'fitness', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
                    rows = list(reader)
                    if reader.fieldnames != expected_fields:
                        self.fail(f'invalid loop_edges.csv header: {reader.fieldnames!r}')
                    if len(rows) != loop_edge_count:
                        self.fail(f'loop edge rows {len(rows)} != loop_edge_count {loop_edge_count}')
                    seen_edges: set[tuple[int, int]] = set()
                    for index, row in enumerate(rows, start=2):
                        try:
                            source = int(row['from'])
                            target = int(row['to'])
                            values = [float(row[field]) for field in expected_fields[2:]]
                        except (KeyError, TypeError, ValueError):
                            self.fail(f'invalid loop edge row {index}')
                            break
                        if (
                            not isinstance(submap_count, int)
                            or source < 0
                            or target < 0
                            or source >= submap_count
                            or target >= submap_count
                            or source == target
                        ):
                            self.fail(
                                f'loop edge row {index} has invalid vertices '
                                f'{source}->{target} for {submap_count} submaps')
                            break
                        edge = (source, target)
                        if edge in seen_edges:
                            self.fail(f'duplicate loop edge row {index}: {source}->{target}')
                            break
                        seen_edges.add(edge)
                        if not all(math.isfinite(value) for value in values):
                            self.fail(f'loop edge row {index} contains a non-finite value')
                            break
            except (OSError, csv.Error) as error:
                self.fail(f'cannot parse loop_edges.csv: {error}')

        if self.failures:
            return False
        self.ok('required artifacts exist and remain inside the bundle')
        self.ok(f'cross-file counts agree: {submap_count} submaps, {loop_edge_count} loop edges')
        return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle_dir', help='directory containing map_bundle.yaml')
    args = parser.parse_args()
    verifier = BundleVerifier(args.bundle_dir)
    success = verifier.run()
    for message in verifier.passes:
        print(f'PASS: {message}')
    for message in verifier.failures:
        print(f'FAIL: {message}')
    print(f'MAP_BUNDLE_{"OK" if success else "FAILED"}')
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
