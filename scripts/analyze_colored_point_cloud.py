#!/usr/bin/env python3
"""Measure real-RGB coverage and chroma in a coloured PLY map."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import pointcloud_io as pcio  # noqa: E402


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def analyze(path: Path, default_rgb: tuple[int, int, int]) -> dict:
    xyz, rgb = pcio.read_ply_xyz(path)
    if rgb is None:
        raise ValueError('PLY has no red/green/blue properties')
    default = np.asarray(default_rgb, dtype=np.uint8)
    seen = np.any(rgb != default[None, :], axis=1)
    selected = rgb[seen]
    channel_range = (np.ptp(selected.astype(np.int16), axis=1)
                     if len(selected) else np.zeros(0, dtype=np.int16))
    bounds = {
        'min_xyz_m': xyz.min(axis=0).astype(float).tolist() if len(xyz) else None,
        'max_xyz_m': xyz.max(axis=0).astype(float).tolist() if len(xyz) else None,
    }
    return {
        'schema_version': 1,
        'input': str(path.resolve()),
        'input_sha256': file_sha256(path),
        'points': int(len(xyz)),
        'colored': int(seen.sum()),
        'colored_frac': float(seen.mean()) if len(seen) else 0.0,
        'default_rgb': list(default_rgb),
        'colour_statistics': {
            'mean_channel_range': float(channel_range.mean()) if len(channel_range) else 0.0,
            'chromatic_fraction_10': (
                float(np.mean(channel_range >= 10)) if len(channel_range) else 0.0),
            'unique_colours': int(len(np.unique(selected, axis=0))),
        },
        'bounds': bounds,
        'status': 'PASS' if len(xyz) and seen.any() else 'FAIL',
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--default-rgb', type=int, nargs=3, default=(128, 128, 128))
    args = parser.parse_args()
    if any(value < 0 or value > 255 for value in args.default_rgb):
        parser.error('--default-rgb values must be in [0, 255]')
    try:
        report = analyze(args.input.resolve(), tuple(args.default_rgb))
    except (OSError, ValueError) as exc:
        parser.exit(2, f'failed to analyze coloured point cloud: {exc}\n')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(report, indent=2, sort_keys=True) + '\n'
    args.output.write_text(payload)
    print(payload, end='')
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
