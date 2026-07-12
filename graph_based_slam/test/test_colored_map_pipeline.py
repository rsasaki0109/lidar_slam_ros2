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

"""Tests for the user-facing coloured-map pipeline command composition."""

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import colored_map_pipeline as cmp  # noqa: E402


def _args(tmp_path, *extra):
    return cmp.build_parser().parse_args([
        str(tmp_path / 'bag'), str(tmp_path / 'traj.tum'), str(tmp_path / 'out'),
        '--extrinsic', str(tmp_path / 'calib.json'), *extra,
    ])


def test_build_commands_connects_extract_to_robust_map(tmp_path):
    commands = cmp.build_commands(_args(tmp_path))
    assert [name for name, _ in commands] == ['posed images', 'coloured map']
    extract, build = commands[0][1], commands[1][1]
    transforms = str(tmp_path / 'out' / 'posed_images' / 'transforms.json')
    assert '--undistort' in extract
    assert extract[extract.index('--time-offset') + 1] == 'auto'
    assert build[build.index('--color-transforms') + 1] == transforms
    assert '--color-robust' in build


def test_raw_trajectory_adds_densification_and_connects_dense_output(tmp_path):
    args = _args(tmp_path, '--raw-traj', str(tmp_path / 'raw.tum'))
    commands = cmp.build_commands(args)
    assert [name for name, _ in commands] == [
        'dense corrected trajectory', 'posed images', 'coloured map']
    densify, extract, build = [command for _, command in commands]
    dense = str(tmp_path / 'out' / 'dense_corrected_trajectory.tum')
    assert densify[densify.index('--raw') + 1] == str(tmp_path / 'raw.tum')
    assert densify[densify.index('--corrected') + 1] == str(
        tmp_path / 'traj.tum')
    assert densify[densify.index('--output') + 1] == dense
    assert extract[extract.index('--traj') + 1] == dense
    assert build[build.index('--traj') + 1] == dense


def test_existing_dense_trajectory_and_outputs_are_reused(tmp_path):
    out = tmp_path / 'out'
    (out / 'posed_images').mkdir(parents=True)
    (out / 'dense_corrected_trajectory.tum').write_text('dense\n')
    (out / 'posed_images' / 'transforms.json').write_text('{}')
    (out / 'colored_map.ply').write_text('ply\n')
    args = _args(tmp_path, '--raw-traj', str(tmp_path / 'raw.tum'))
    assert cmp.build_commands(args) == []


def test_force_trajectory_rebuilds_all_dependent_stages(tmp_path):
    out = tmp_path / 'out'
    (out / 'posed_images').mkdir(parents=True)
    (out / 'dense_corrected_trajectory.tum').write_text('dense\n')
    (out / 'posed_images' / 'transforms.json').write_text('{}')
    (out / 'colored_map.ply').write_text('ply\n')
    args = _args(
        tmp_path, '--raw-traj', str(tmp_path / 'raw.tum'),
        '--force-trajectory')
    assert [name for name, _ in cmp.build_commands(args)] == [
        'dense corrected trajectory', 'posed images', 'coloured map']


def test_build_commands_reuses_existing_outputs(tmp_path):
    out = tmp_path / 'out'
    (out / 'posed_images').mkdir(parents=True)
    (out / 'posed_images' / 'transforms.json').write_text('{}')
    (out / 'colored_map.ply').write_text('ply\n')
    assert cmp.build_commands(_args(tmp_path)) == []


def test_force_map_reuses_images_but_rebuilds_map(tmp_path):
    posed = tmp_path / 'out' / 'posed_images'
    posed.mkdir(parents=True)
    (posed / 'transforms.json').write_text('{}')
    commands = cmp.build_commands(_args(tmp_path, '--force-map'))
    assert [name for name, _ in commands] == ['coloured map']


def test_force_images_also_rebuilds_dependent_map(tmp_path):
    out = tmp_path / 'out'
    (out / 'posed_images').mkdir(parents=True)
    (out / 'posed_images' / 'transforms.json').write_text('{}')
    (out / 'colored_map.ply').write_text('ply\n')
    commands = cmp.build_commands(_args(tmp_path, '--force-images'))
    assert [name for name, _ in commands] == ['posed images', 'coloured map']


def test_no_undistort_and_custom_topics_are_forwarded(tmp_path):
    commands = cmp.build_commands(_args(
        tmp_path, '--no-undistort', '--points-topic', '/points',
        '--camera-topic', '/rgb', '--camera-info-topic', '/info'))
    extract, build = commands[0][1], commands[1][1]
    assert '--undistort' not in extract
    assert extract[extract.index('--camera-topic') + 1] == '/rgb'
    assert extract[extract.index('--camera-info-topic') + 1] == '/info'
    assert build[build.index('--points-topic') + 1] == '/points'


def test_intrinsics_yaml_is_forwarded_for_bags_without_camera_info(tmp_path):
    intrinsics = tmp_path / 'camchain.yaml'
    commands = cmp.build_commands(_args(
        tmp_path, '--intrinsics-yaml', str(intrinsics)))
    extract = commands[0][1]
    assert extract[extract.index('--intrinsics-yaml') + 1] == str(intrinsics)


def test_kalibr_pair_uses_generated_extrinsic(tmp_path):
    args = cmp.build_parser().parse_args([
        str(tmp_path / 'bag'), str(tmp_path / 'traj.tum'), str(tmp_path / 'out'),
        '--kalibr-camchain', str(tmp_path / 'camchain.yaml'),
        '--lidar-calibration', str(tmp_path / 'lidar.yaml'),
    ])
    extract, build = [command for _, command in cmp.build_commands(args)]
    generated = tmp_path / 'out' / 'generated_body_camera_extrinsic.json'
    assert extract[extract.index('--extrinsic') + 1] == str(generated)
    assert build[build.index('--lidar-calibration') + 1] == str(
        tmp_path / 'lidar.yaml')
    assert build[build.index('--lidar-key') + 1] == 'PandarXT-32'


def test_kalibr_camchain_requires_lidar_calibration(tmp_path):
    import pytest
    args = cmp.build_parser().parse_args([
        str(tmp_path / 'bag'), str(tmp_path / 'traj.tum'), str(tmp_path / 'out'),
        '--kalibr-camchain', str(tmp_path / 'camchain.yaml'), '--dry-run',
    ])
    with pytest.raises(ValueError, match='lidar-calibration'):
        cmp.run_pipeline(args)


def test_sparse_graph_keyframes_are_rejected(tmp_path):
    import pytest
    traj = tmp_path / 'traj.tum'
    traj.write_text(
        '1.0 0 0 0 0 0 0 1\n'
        '2.2 0 0 0 0 0 0 1\n')
    with pytest.raises(ValueError, match='dense SLAM trajectory'):
        cmp.validate_trajectory_density(traj, 0.5)


def test_dense_trajectory_passes_density_guard(tmp_path):
    traj = tmp_path / 'traj.tum'
    traj.write_text(
        '1.0 0 0 0 0 0 0 1\n'
        '1.1 0 0 0 0 0 0 1\n')
    cmp.validate_trajectory_density(traj, 0.5)
