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

"""Regression tests for the Autoware map run diagnosis helper."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'diagnose_autoware_map_run.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('diagnose_autoware_map_run', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_summary_marks_success_when_map_and_verify_pass_exist(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'run'
    pointcloud_dir = run_dir / 'pointcloud_map'
    pointcloud_dir.mkdir(parents=True)
    meta = pointcloud_dir / 'pointcloud_map_metadata.yaml'
    meta.write_text('tile_size: 20\n', encoding='utf-8')
    (run_dir / 'map_projector_info.yaml').write_text(
        yaml.safe_dump({'projector_type': 'LocalCartesian'}),
        encoding='utf-8',
    )
    (run_dir / 'verify_autoware_map.log').write_text(
        'PASS: 8  |  WARN: 0  |  FAIL: 0\nRESULT: PASS -- map is Autoware-compatible\n',
        encoding='utf-8',
    )
    (run_dir / 'lidarslam.launch.log').write_text(
        'RKO LIO Node is up!\n[graph_based_slam]: initialization end\n',
        encoding='utf-8',
    )

    summary = module.summarize_run(run_dir)

    assert summary['status'] == 'success'
    assert summary['verify']['result'] == 'PASS'
    assert summary['projector_type'] == 'LocalCartesian'
    assert any(
        'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh' in step
        for step in summary['suggested_next_steps']
    )


def test_summary_reports_tf_issue_hints(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    (run_dir / 'lidarslam.launch.log').write_text(
        "Could not find a connection between 'odom' and 'velodyne_front'\n"
        'TF_NO_FRAME_ID\n'
        'process has died\n',
        encoding='utf-8',
    )
    (run_dir / 'map_save.log').write_text(
        'map_save service call failed\n',
        encoding='utf-8',
    )

    summary = module.summarize_run(run_dir)
    hints = '\n'.join(summary['problem_hints'])

    assert summary['status'] == 'runtime_failed'
    assert 'TF tree connectivity was missing' in hints
    assert 'The /map_save service call failed' in hints
    assert 'A ROS node died during the run' in hints
    assert any('tail -n 120' in step for step in summary['suggested_next_steps'])
