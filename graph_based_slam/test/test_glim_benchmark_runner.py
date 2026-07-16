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


"""Unit tests for the GLIM benchmark artifact parsers."""

import importlib.util
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
SPEC = importlib.util.spec_from_file_location(
    'run_glim_benchmark', ROOT / 'scripts/run_glim_benchmark.py')
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)
SCORE_SPEC = importlib.util.spec_from_file_location(
    'score_repeated_trajectory_benchmark',
    ROOT / 'scripts/score_repeated_trajectory_benchmark.py')
SCORER = importlib.util.module_from_spec(SCORE_SPEC)
SCORE_SPEC.loader.exec_module(SCORER)
EXPORT_SPEC = importlib.util.spec_from_file_location(
    'export_glim_dump_map', ROOT / 'scripts/export_glim_dump_map.py')
EXPORTER = importlib.util.module_from_spec(EXPORT_SPEC)
EXPORT_SPEC.loader.exec_module(EXPORTER)


def test_bag_bounds_reads_rosbag2_nanoseconds(tmp_path):
    metadata = tmp_path / 'metadata.yaml'
    metadata.write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
        'starting_time': {'nanoseconds_since_epoch': 2_000_000_000},
        'duration': {'nanoseconds': 500_000_000},
    }}))
    assert RUNNER.bag_bounds(metadata) == (2.0, 2.5)


def test_trajectory_info_reads_tum(tmp_path):
    trajectory = tmp_path / 'traj_lidar.txt'
    trajectory.write_text(
        '# TUM\n1.0 0 0 0 0 0 0 1\n2.5 1 0 0 0 0 0 1\n')
    assert RUNNER.trajectory_info(trajectory) == {
        'samples': 2, 'first_stamp': 1.0, 'last_stamp': 2.5}


def test_glim_lidar_pose_is_shifted_to_prism_in_local_frame(tmp_path):
    source = tmp_path / 'traj_lidar.txt'
    destination = tmp_path / 'trajectory_prism.tum'
    source.write_text('1 10 20 30 0 0 0 1\n')
    RUNNER.apply_tum_translation_offset(
        source, destination, (-0.243656, -0.012288, -0.328095))
    values = [float(value) for value in destination.read_text().split()]
    assert values[1:4] == [9.756344, 19.987712, 29.671905]


def test_common_reference_uses_intersection_of_trajectory_ranges(tmp_path):
    reference = tmp_path / 'gt.tum'
    reference.write_text('\n'.join(
        f'{stamp} 0 0 0 0 0 0 1' for stamp in range(5)) + '\n')
    first, second = tmp_path / 'first.tum', tmp_path / 'second.tum'
    first.write_text('1 0 0 0 0 0 0 1\n4 0 0 0 0 0 0 1\n')
    second.write_text('0 0 0 0 0 0 0 1\n3 0 0 0 0 0 0 1\n')
    selected, excluded = SCORER.select_common_reference(
        reference, [first, second])
    assert [float(line.split()[0]) for line in selected] == [1.0, 2.0, 3.0]
    assert excluded == [0.0, 4.0]


def test_common_scorer_normalizes_nested_fast_mapper_rss():
    assert SCORER.peak_rss_mb({'runtime': {
        'mapper': {'peak_rss_mb': 321.5}}}) == 321.5
    assert SCORER.peak_rss_mb({'runtime': {
        'peak_rss_mb': 123.0}}) == 123.0


def test_glim_compact_points_are_transformed_to_world(tmp_path):
    submap = tmp_path / '000000'
    submap.mkdir()
    import numpy as np
    np.asarray([[1.0, 2.0, 3.0]], dtype=np.float32).tofile(
        submap / 'points_compact.bin')
    (submap / 'data.txt').write_text(
        'T_world_origin: \n1 0 0 10\n0 1 0 20\n0 0 1 30\n0 0 0 1\n')
    points = EXPORTER.load_world_points(submap)
    assert np.allclose(points, [[11.0, 22.0, 33.0]])


def test_glim_runner_exposes_required_map_gate_option(monkeypatch):
    argv = [
        'runner', '--bag', '/tmp/bag',
        '--output', '/tmp/out',
        '--reference-meta', '/tmp/ref',
        '--save-maps']
    monkeypatch.setattr(sys, 'argv', argv)
    assert RUNNER.parse_args().save_maps is True
