"""Tests for coloured-map coverage/chroma analysis."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / 'tools/gaussian_splatting'
sys.path.insert(0, str(TOOLS))
import pointcloud_io as pcio  # noqa: E402

SPEC = importlib.util.spec_from_file_location(
    'colored_report', ROOT / 'scripts/analyze_colored_point_cloud.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_report_excludes_default_fill_and_preserves_real_chroma(tmp_path: Path):
    path = tmp_path / 'map.ply'
    xyz = np.array([[0, 0, 0], [1, 2, 3], [2, 3, 4]], np.float32)
    rgb = np.array([[128, 128, 128], [10, 20, 30], [40, 40, 40]], np.uint8)
    pcio.write_ply(path, xyz, rgb)

    report = MODULE.analyze(path, (128, 128, 128))

    assert report['status'] == 'PASS'
    assert report['points'] == 3
    assert report['colored'] == 2
    assert report['colored_frac'] == 2 / 3
    assert report['colour_statistics']['mean_channel_range'] == 10.0
    assert report['colour_statistics']['chromatic_fraction_10'] == 0.5
    assert report['colour_statistics']['unique_colours'] == 2
    assert len(report['input_sha256']) == 64


def test_uncoloured_ply_is_rejected(tmp_path: Path):
    path = tmp_path / 'xyz.ply'
    pcio.write_ply(path, np.zeros((1, 3), np.float32))

    import pytest
    with pytest.raises(ValueError, match='no red/green/blue'):
        MODULE.analyze(path, (128, 128, 128))
