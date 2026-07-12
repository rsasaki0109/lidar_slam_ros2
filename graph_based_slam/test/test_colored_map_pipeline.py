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
