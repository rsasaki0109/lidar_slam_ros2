# Copyright 2026 Sasaki
# All rights reserved.

"""Tests for official NTU-VIRAL CSV reference conversion."""

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'ntu_reference', ROOT / 'scripts' / 'generate_ntu_viral_csv_reference.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_csv_conversion_deduplicates_stamps(tmp_path):
    source = tmp_path / 'gt.csv'
    header = ('field.header.stamp,field.pose.position.x,'
              'field.pose.position.y,field.pose.position.z,'
              'field.pose.orientation.x,field.pose.orientation.y,'
              'field.pose.orientation.z,field.pose.orientation.w\n')
    rows = [f'{stamp},{x},0,0,0,0,0,1\n'
            for stamp, x in ((1_000_000_000, 0), (1_000_000_000, 0),
                             (2_000_000_000, 1), (3_000_000_000, 2))]
    source.write_text(header + ''.join(rows))
    output = tmp_path / 'gt.tum'
    summary = MODULE.convert(source, output)
    assert summary['pose_count'] == 3
    assert output.read_text().splitlines() == [
        '1.000000000 0 0 0 0 0 0 1',
        '2.000000000 1 0 0 0 0 0 1',
        '3.000000000 2 0 0 0 0 0 1']


def test_lidar_to_body_translation_uses_frozen_rko_config(tmp_path):
    config = tmp_path / 'rko.yaml'
    config.write_text(
        'extrinsic_lidar2base_quat_xyzw_xyz: [0, 0, 0, 1, -0.05, 0, 0.055]\n')
    assert MODULE.lidar_to_body_translation(config) == (0.05, 0.0, -0.055)
