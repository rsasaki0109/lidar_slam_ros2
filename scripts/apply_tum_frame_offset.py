#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.array([qw, qx, qy, qz], dtype=float)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Apply a fixed local-frame translation offset to a TUM trajectory. "
            "Orientation is preserved."
        ),
    )
    parser.add_argument("--in", dest="input_path", required=True, help="Input TUM trajectory")
    parser.add_argument("--out", dest="output_path", required=True, help="Output TUM trajectory")
    parser.add_argument("--tx", type=float, required=True, help="Offset x in the source local frame")
    parser.add_argument("--ty", type=float, required=True, help="Offset y in the source local frame")
    parser.add_argument("--tz", type=float, required=True, help="Offset z in the source local frame")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = Path(args.input_path)
    output_path = Path(args.output_path)
    offset = np.array([args.tx, args.ty, args.tz], dtype=float)

    lines_out = []
    with input_path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            stripped = line.strip()
            if not stripped:
                continue
            parts = stripped.split()
            if len(parts) != 8:
                raise SystemExit(f"invalid TUM line: {line.rstrip()}")

            stamp = parts[0]
            px, py, pz, qx, qy, qz, qw = map(float, parts[1:])
            rot = quat_to_rot(qx, qy, qz, qw)
            pos = np.array([px, py, pz], dtype=float) + rot @ offset
            lines_out.append(
                f"{stamp} {pos[0]:.9f} {pos[1]:.9f} {pos[2]:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n",
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        f.writelines(lines_out)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
