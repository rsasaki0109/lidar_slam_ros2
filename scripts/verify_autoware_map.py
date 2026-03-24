#!/usr/bin/env python3
"""Verify a pointcloud map directory for Autoware compatibility.

Mimics the parsing logic of Autoware's pointcloud_map_loader
(autoware_core/map/autoware_map_loader) to check that a generated map
directory is fully compatible.

Key checks performed:
  1. pointcloud_map_metadata.yaml exists and is parseable
  2. x_resolution / y_resolution are present and positive
  3. Each PCD entry has coordinates parseable as integers (Autoware uses
     node.second.as<std::vector<int>>())
  4. Coordinates are consistent with the filename (e.g. -40_-20.pcd -> [-40, -20])
  5. Every PCD file referenced in metadata exists and is loadable
  6. No orphan PCD files (files on disk but not in metadata)
  7. Each PCD file has a non-zero point count
  8. Bounding box: all points fall within the tile defined by
     [x, x+x_res) x [y, y+y_res)
  9. Optional: map_projector_info.yaml validation

Usage:
  python3 verify_autoware_map.py /path/to/map_dir
  python3 verify_autoware_map.py /path/to/map_dir --check-bounds
"""

from __future__ import annotations

import argparse
import os
import re
import struct
import sys
from pathlib import Path

import yaml


# ---------------------------------------------------------------------------
# Minimal PCD header parser (avoids dependency on open3d / pypcd at import)
# ---------------------------------------------------------------------------
def parse_pcd_header(filepath: str) -> dict:
    """Parse a PCD file header and return metadata + point count."""
    info: dict = {"fields": [], "size": [], "type": [], "count": [],
                  "width": 0, "height": 0, "points": 0, "data": "ascii",
                  "header_bytes": 0}
    with open(filepath, "rb") as f:
        while True:
            line_bytes = f.readline()
            if not line_bytes:
                break
            line = line_bytes.decode("ascii", errors="replace").strip()
            if line.startswith("#"):
                continue
            parts = line.split()
            if not parts:
                continue
            key = parts[0].upper()
            if key == "FIELDS":
                info["fields"] = [p.lower() for p in parts[1:]]
            elif key == "SIZE":
                info["size"] = [int(p) for p in parts[1:]]
            elif key == "TYPE":
                info["type"] = parts[1:]
            elif key == "COUNT":
                info["count"] = [int(p) for p in parts[1:]]
            elif key == "WIDTH":
                info["width"] = int(parts[1])
            elif key == "HEIGHT":
                info["height"] = int(parts[1])
            elif key == "POINTS":
                info["points"] = int(parts[1])
            elif key == "DATA":
                info["data"] = parts[1].lower()
                info["header_bytes"] = f.tell()
                break
    return info


def read_xyz_from_pcd(filepath: str, header: dict) -> list[tuple[float, float, float]]:
    """Read XYZ coordinates from a binary PCD file.  Returns list of (x,y,z)."""
    fields = header["fields"]
    sizes = header["size"]
    types = header["type"]
    counts = header["count"]

    if "x" not in fields or "y" not in fields:
        return []

    # Build struct format for one point row
    fmt_map = {("F", 4): "f", ("F", 8): "d",
               ("U", 1): "B", ("U", 2): "H", ("U", 4): "I",
               ("I", 1): "b", ("I", 2): "h", ("I", 4): "i"}
    row_fmt = "<"
    field_indices: dict[str, int] = {}
    idx = 0
    for i, (fld, sz, tp, cnt) in enumerate(zip(fields, sizes, types, counts)):
        code = fmt_map.get((tp, sz))
        if code is None:
            # Skip unknown type by padding
            row_fmt += f"{sz * cnt}x"
        else:
            for c in range(cnt):
                if c == 0:
                    field_indices[fld] = idx
                row_fmt += code
                idx += 1

    row_size = struct.calcsize(row_fmt)
    n_points = header["points"]

    points = []
    if header["data"] == "binary":
        with open(filepath, "rb") as f:
            f.seek(header["header_bytes"])
            data = f.read(row_size * n_points)
        xi = field_indices.get("x")
        yi = field_indices.get("y")
        zi = field_indices.get("z")
        for p in range(n_points):
            vals = struct.unpack_from(row_fmt, data, p * row_size)
            x = vals[xi] if xi is not None else 0.0
            y = vals[yi] if yi is not None else 0.0
            z = vals[zi] if zi is not None else 0.0
            points.append((x, y, z))
    # ascii and binary_compressed not implemented for bounds check
    return points


# ---------------------------------------------------------------------------
# Main verification
# ---------------------------------------------------------------------------
class MapVerifier:
    def __init__(self, map_dir: str, check_bounds: bool = False, verbose: bool = False):
        self.map_dir = Path(map_dir)
        self.check_bounds = check_bounds
        self.verbose = verbose
        self.passes: list[str] = []
        self.warnings: list[str] = []
        self.failures: list[str] = []

    def ok(self, msg: str):
        self.passes.append(msg)
        if self.verbose:
            print(f"  PASS  {msg}")

    def warn(self, msg: str):
        self.warnings.append(msg)
        print(f"  WARN  {msg}")

    def fail(self, msg: str):
        self.failures.append(msg)
        print(f"  FAIL  {msg}")

    def run(self) -> bool:
        print(f"Verifying Autoware pointcloud map: {self.map_dir}")
        print()

        # --- 1. Locate metadata ---
        meta_path = self.map_dir / "pointcloud_map_metadata.yaml"
        if not meta_path.exists():
            # Maybe the map dir contains a pointcloud_map/ subdirectory
            alt = self.map_dir / "pointcloud_map" / "pointcloud_map_metadata.yaml"
            if alt.exists():
                self.map_dir = self.map_dir / "pointcloud_map"
                meta_path = alt
                self.warn(f"metadata found in subdirectory: {alt}")
            else:
                self.fail(f"pointcloud_map_metadata.yaml not found in {self.map_dir}")
                self._print_summary()
                return False

        # --- 2. Parse YAML ---
        try:
            with open(meta_path) as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.fail(f"Failed to parse YAML: {e}")
            self._print_summary()
            return False
        self.ok(f"metadata YAML parsed: {meta_path.name}")

        # --- 3. Resolution ---
        x_res = config.get("x_resolution")
        y_res = config.get("y_resolution")
        if x_res is None or y_res is None:
            self.fail("x_resolution or y_resolution missing from metadata")
            self._print_summary()
            return False
        if x_res <= 0 or y_res <= 0:
            self.fail(f"resolution must be positive: x_resolution={x_res}, y_resolution={y_res}")
        else:
            self.ok(f"resolution: x={x_res}, y={y_res}")

        # --- 4. Parse tile entries ---
        tiles: dict[str, tuple[int, int]] = {}
        coord_type_warnings = []
        for key, val in config.items():
            if key in ("x_resolution", "y_resolution"):
                continue
            # Autoware parses as std::vector<int> -- values must be exact integers
            if not isinstance(val, (list, tuple)) or len(val) != 2:
                self.fail(f"entry '{key}' has invalid coordinate format: {val}")
                continue

            cx, cy = val[0], val[1]
            # Check if values are truly integers (Autoware uses as<int>)
            if isinstance(cx, float) or isinstance(cy, float):
                # Check if they are integer-valued floats (e.g. -40.0)
                if cx == int(cx) and cy == int(cy):
                    coord_type_warnings.append(
                        f"  {key}: [{cx}, {cy}] -- float representation of integer"
                    )
                else:
                    self.fail(
                        f"entry '{key}' has non-integer coordinates: [{cx}, {cy}]. "
                        f"Autoware parses as std::vector<int>."
                    )
                    continue

            ix, iy = int(cx), int(cy)
            tiles[key] = (ix, iy)

            # Verify filename matches coordinates
            expected_name = f"{ix}_{iy}.pcd"
            if key != expected_name:
                self.warn(f"filename '{key}' does not match expected '{expected_name}' "
                          f"for coordinates [{ix}, {iy}]")

        if coord_type_warnings:
            self.warn(
                f"{len(coord_type_warnings)} tile(s) use float representation of integer coords.\n"
                f"    Autoware uses as<std::vector<int>>() which WILL FAIL on 'x.0' YAML values.\n"
                f"    Coordinates must be bare integers (e.g., [-40, -20] not [-40.0, -20.0])."
            )
            if self.verbose:
                for w in coord_type_warnings:
                    print(f"    {w}")
            # This is actually a FAIL for Autoware compatibility
            self.fail(
                "YAML coordinates are floats -- Autoware's YAML::Node::as<std::vector<int>>() "
                "will throw yaml-cpp::BadConversion. Must be written as integers."
            )

        if not tiles:
            self.fail("No PCD tile entries found in metadata")
            self._print_summary()
            return False
        self.ok(f"{len(tiles)} tile entries parsed")

        # --- 5. Check PCD file existence ---
        missing = []
        loadable = []
        for fname in sorted(tiles):
            pcd_path = self.map_dir / fname
            if not pcd_path.exists():
                missing.append(fname)
                self.fail(f"PCD file missing: {fname}")
            else:
                loadable.append((fname, pcd_path))

        if not missing:
            self.ok(f"All {len(tiles)} PCD files exist on disk")

        # --- 6. Orphan PCD files ---
        all_pcds = set(p.name for p in self.map_dir.glob("*.pcd"))
        metadata_pcds = set(tiles.keys())
        orphans = all_pcds - metadata_pcds
        if orphans:
            self.warn(f"{len(orphans)} orphan PCD file(s) not in metadata: "
                      + ", ".join(sorted(orphans)))
        else:
            self.ok("No orphan PCD files")

        # --- 7. Load and validate each PCD ---
        total_points = 0
        for fname, pcd_path in loadable:
            try:
                header = parse_pcd_header(str(pcd_path))
            except Exception as e:
                self.fail(f"{fname}: failed to parse PCD header: {e}")
                continue

            n = header["points"]
            if n == 0:
                self.fail(f"{fname}: zero points")
            else:
                total_points += n
                if self.verbose:
                    print(f"    {fname}: {n:,} points, fields={header['fields']}")

            # Check required fields
            fields = set(header["fields"])
            if "x" not in fields or "y" not in fields or "z" not in fields:
                self.fail(f"{fname}: missing x/y/z fields (has {header['fields']})")

            # --- 8. Bounds check ---
            if self.check_bounds and header["data"] == "binary" and n > 0:
                ix, iy = tiles[fname]
                x_min, y_min = float(ix), float(iy)
                x_max = x_min + float(x_res)
                y_max = y_min + float(y_res)

                try:
                    pts = read_xyz_from_pcd(str(pcd_path), header)
                except Exception as e:
                    self.warn(f"{fname}: could not read points for bounds check: {e}")
                    continue

                oob = 0
                margin = 0.01  # 1cm tolerance for floating point
                for px, py, pz in pts:
                    if (px < x_min - margin or px >= x_max + margin or
                            py < y_min - margin or py >= y_max + margin):
                        oob += 1
                if oob > 0:
                    pct = 100.0 * oob / len(pts)
                    self.warn(f"{fname}: {oob}/{len(pts)} points ({pct:.1f}%) "
                              f"outside tile bounds [{x_min},{y_min}]-[{x_max},{y_max}]")
                else:
                    if self.verbose:
                        print(f"    {fname}: all points within tile bounds")

        self.ok(f"Total points across all tiles: {total_points:,}")

        # --- 9. map_projector_info.yaml (optional) ---
        projector_path = self.map_dir.parent / "map_projector_info.yaml"
        if not projector_path.exists():
            projector_path = self.map_dir / "map_projector_info.yaml"
        if projector_path.exists():
            try:
                with open(projector_path) as f:
                    proj = yaml.safe_load(f)
                ptype = proj.get("projector_type", "UNKNOWN")
                self.ok(f"map_projector_info.yaml: projector_type={ptype}")
                if ptype in ("local", "Local"):
                    self.ok("  local projector selected")
                if ptype in ("LocalCartesian", "LocalCartesianUTM", "TransverseMercator"):
                    # local maps should have map_origin
                    origin = proj.get("map_origin", {})
                    if origin:
                        self.ok(f"  map_origin: lat={origin.get('latitude')}, "
                                f"lon={origin.get('longitude')}")
                    else:
                        self.fail("map_projector_info.yaml is missing map_origin")
            except Exception as e:
                self.warn(f"map_projector_info.yaml parse error: {e}")
        else:
            self.warn("map_projector_info.yaml not found (optional for Autoware)")

        self._print_summary()
        return len(self.failures) == 0

    def _print_summary(self):
        print()
        print("=" * 60)
        print(f"  PASS: {len(self.passes)}  |  WARN: {len(self.warnings)}  |  "
              f"FAIL: {len(self.failures)}")
        print("=" * 60)
        if self.failures:
            print()
            print("FAILURES:")
            for f in self.failures:
                print(f"  - {f}")
        if self.warnings and not self.verbose:
            print()
            print("WARNINGS:")
            for w in self.warnings:
                print(f"  - {w}")
        print()
        if not self.failures:
            print("RESULT: PASS -- map is Autoware-compatible")
        else:
            print("RESULT: FAIL -- map has compatibility issues")


def main():
    parser = argparse.ArgumentParser(
        description="Verify a pointcloud map directory for Autoware compatibility")
    parser.add_argument("map_dir", help="Path to map directory (contains pointcloud_map_metadata.yaml or pointcloud_map/ subdir)")
    parser.add_argument("--check-bounds", action="store_true",
                        help="Check that all points fall within their tile bounds (slow)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Show per-file details")
    args = parser.parse_args()

    verifier = MapVerifier(args.map_dir, check_bounds=args.check_bounds, verbose=args.verbose)
    success = verifier.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
