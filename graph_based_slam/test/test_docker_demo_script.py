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

"""Regression tests for the one-command Docker demo."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
DEMO_SCRIPT = REPO_ROOT / 'scripts' / 'run_docker_demo.sh'
BAG_NAME = 'rosbag2_2024_04_16-14_17_01'


def test_demo_selects_nested_directory_that_contains_metadata(tmp_path: Path):
    data_dir = tmp_path / 'datasets'
    outer_dir = data_dir / 'driving_slam_mid360' / 'extracted' / BAG_NAME
    bag_dir = outer_dir / BAG_NAME
    bag_dir.mkdir(parents=True)
    (bag_dir / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: {}\n',
        encoding='utf-8',
    )

    stub_dir = tmp_path / 'bin'
    stub_dir.mkdir()
    capture_path = tmp_path / 'runner_args.txt'
    cli_stub = stub_dir / 'lidarslam-map'
    cli_stub.write_text(
        '#!/bin/sh\n'
        'printf "%s\\n" "$@" > "$DEMO_TEST_CAPTURE"\n',
        encoding='utf-8',
    )
    cli_stub.chmod(0o755)

    env = os.environ.copy()
    env.update(
        {
            'DEMO_DATA_DIR': str(data_dir),
            'DEMO_OUTPUT_DIR': str(tmp_path / 'output'),
            'DEMO_TEST_CAPTURE': str(capture_path),
            'PATH': f'{stub_dir}:{env["PATH"]}',
        }
    )
    result = subprocess.run(
        ['/bin/bash', str(DEMO_SCRIPT)],
        check=False,
        capture_output=True,
        env=env,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert f'bag: {bag_dir}' in result.stdout
    runner_args = capture_path.read_text(encoding='utf-8').splitlines()
    assert runner_args[:2] == ['run', str(bag_dir)]
    assert runner_args[runner_args.index('--profile') + 1] == (
        'rko_lio_graph_mid360_preset'
    )
    assert runner_args[runner_args.index('--output-dir') + 1] == str(
        tmp_path / 'output'
    )


def test_demo_delegates_to_versioned_product_contract():
    script = DEMO_SCRIPT.read_text(encoding='utf-8')

    assert 'lidarslam-map run "${bag_dir}"' in script
    assert '--profile rko_lio_graph_mid360_preset' in script
    assert 'run_manifest.json' in script
    assert 'verify_autoware_map.log' in script
    assert 'run_rko_lio_graph_autoware_dogfood.sh' not in script
