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

"""CLI contract tests for the deterministic offline backend runner wrapper."""

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_offline_determinism_check.sh'


def test_param_override_requires_ros_assignment_syntax():
    result = subprocess.run(
        ['bash', str(SCRIPT), '--param', 'refine_window_size=32'],
        cwd=ROOT, capture_output=True, text=True, check=False)

    assert result.returncode == 2
    assert '--param expects name:=value' in result.stderr


def test_param_override_is_forwarded_after_params_file():
    source = SCRIPT.read_text()

    assert 'PARAM_OVERRIDES+=("$2")' in source
    assert 'RUNNER_CMD+=(-p "${override}")' in source
    assert source.index('RUNNER_CMD+=(-p "${override}")') > source.index(
        '--params-file "${PARAMS}"')


def test_runs_are_dds_isolated_and_resume_requires_completion_marker():
    source = SCRIPT.read_text()

    assert '--disable-rosout-logs' in source
    assert 'ROS_DOMAIN_ID=' in source
    assert 'ROS_LOCALHOST_ONLY=1' in source
    assert '--ros-domain-base' in source
    assert '--resume' in source
    assert '${run_dir}/.complete' in source


def test_selected_setup_and_runner_binary_are_frozen_in_summary():
    source = SCRIPT.read_text()

    assert '--setup) SETUP_FILE="$2"' in source
    assert 'ros2 pkg prefix graph_based_slam' in source
    assert 'RUNNER_CMD=(' in source
    assert '"${RUNNER_EXECUTABLE}" --ros-args' in source
    assert 'runner_sha256:' in source
    assert 'params_sha256:' in source
    assert 'bag_metadata_sha256:' in source
    assert 'parameter_overrides:' in source


def test_fixed_loop_edge_replay_can_be_forwarded_as_one_knob_ablation():
    source = SCRIPT.read_text()
    runner_source = (
        ROOT / 'graph_based_slam/src/graph_slam_offline_runner.cpp').read_text()

    assert 'RUNNER_CMD+=(-p "${override}")' in source
    assert 'fixed_loop_edges_path' in runner_source
    assert 'descriptor search was skipped' in runner_source
    assert 'fixed_loop_edges_sha256:' in source
