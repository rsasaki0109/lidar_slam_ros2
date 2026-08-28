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

import json
import os
from pathlib import Path
import stat
import subprocess
import textwrap


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_offline_determinism_check.sh'


def _make_fake_offline_workspace(tmp_path):
    """Create a ROS-free fixture that exercises the shell parser/gates."""
    fake_bin = tmp_path / 'bin'
    prefix = tmp_path / 'prefix'
    runner = prefix / 'lib/graph_based_slam/graph_slam_offline_runner'
    fake_bin.mkdir(parents=True)
    runner.parent.mkdir(parents=True)

    ros2 = fake_bin / 'ros2'
    ros2.write_text(
        '#!/usr/bin/env bash\n'
        'if [[ "${1:-}" == pkg && "${2:-}" == prefix ]]; then\n'
        '  printf "%s\\n" "${FAKE_GRAPH_PREFIX}"\n'
        '  exit 0\n'
        'fi\n'
        'exit 1\n', encoding='utf-8')
    runner.write_text(textwrap.dedent(r"""
        #!/usr/bin/env bash
        set -euo pipefail
        output_dir=""
        for argument in "$@"; do
          if [[ "${argument}" == output_dir:=* ]]; then
            output_dir="${argument#output_dir:=}"
          fi
        done
        [[ -n "${output_dir}" ]]
        counter_file="${FAKE_GRAPH_PREFIX}/counter"
        count=0
        if [[ -f "${counter_file}" ]]; then
          count=$(<"${counter_file}")
        fi
        count=$((count + 1))
        printf '%s\n' "${count}" > "${counter_file}"
        sleep "${FAKE_RUNNER_SLEEP:-0.01}"
        mkdir -p "${output_dir}"
        printf 'source,target\n1,2\n' > "${output_dir}/loop_edges.csv"
        printf '0.0 0 0 0 0 0 0\n' > "${output_dir}/trajectory_optimized.tum"
        implementation_version="fixture"
        if [[ "${FAKE_RECEIPT_VARIANT:-}" == run-specific ]]; then
          implementation_version="fixture-${count}"
        fi
        cat > "${output_dir}/registration_plugin_receipt.yaml" <<EOF
        schema: 1
        role: "backend_loop"
        backend_kind: "host_builtin"
        requested_class: "lidarslam_builtin/NdtOmp"
        resolved_class: "lidarslam_builtin/NdtOmp"
        metadata_class_id: "lidarslam_builtin/NdtOmp"
        implementation_version: "${implementation_version}"
        license: "BSD-2-Clause"
        api_major: 1
        api_minor: 0
        capabilities_bits: 159
        target_policy: "raw_target"
        correspondence_metric: "mean_distance"
        thread_model: "serialized_owner"
        library_path: ""
        plugin_manifest_path: ""
        requirements:
          initial_guess: true
          aligned_source: true
          mean_correspondence_distance: true
        parameters:
          "maximum_iterations":
            type: "integer"
            value: "100"
          "neighborhood_search_method":
            type: "string"
            value: "DIRECT7"
          "num_threads":
            type: "integer"
            value: "1"
          "target_cell_cache_capacity":
            type: "integer"
            value: "3"
          "outlier_ratio":
            type: "double"
            value: "0.55"
          "resolution":
            type: "double"
            value: "5"
          "step_size":
            type: "double"
            value: "0.1"
          "transformation_epsilon":
            type: "double"
            value: "0.01"
        EOF
    """).lstrip(), encoding='utf-8')
    for path in (ros2, runner):
        path.chmod(path.stat().st_mode | stat.S_IXUSR)

    setup = tmp_path / 'setup.bash'
    setup.write_text(
        f'export FAKE_GRAPH_PREFIX="{prefix}"\n'
        f'export PATH="{fake_bin}:$PATH"\n'
        'export ROS_DISTRO=fixture\n', encoding='utf-8')
    params = tmp_path / 'params.yaml'
    params.write_text('/**:\n  ros__parameters: {}\n', encoding='utf-8')
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  duration:\n'
        '    nanoseconds: 1000000000\n', encoding='utf-8')
    (bag / 'input.db3').write_bytes(b'fixture-input')
    return setup, params, bag


def _run_fake_offline_check(tmp_path, *extra_args, receipt_variant=''):
    setup, params, bag = _make_fake_offline_workspace(tmp_path)
    output = tmp_path / 'output'
    environment = os.environ.copy()
    environment.update({
        'FAKE_GRAPH_PREFIX': str(setup.parent / 'prefix'),
        'FAKE_RUNNER_SLEEP': '0.02',
        'FAKE_RECEIPT_VARIANT': receipt_variant,
    })
    command = [
        'bash', str(SCRIPT), '--bag', str(bag), '--params', str(params),
        '--setup', str(setup), '--runs', '2', '--ros-domain-base', '10',
        '--output-dir', str(output), *extra_args,
    ]
    result = subprocess.run(
        command, cwd=ROOT, capture_output=True, text=True,
        check=False, env=environment)
    return result, output


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
    assert '--require-ape' in source
    assert 'execution_identity_sha256' in source
    assert 'git_worktree_fingerprint' in source


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


def test_fixture_writes_metrics_receipts_and_machine_summaries(tmp_path):
    result, output = _run_fake_offline_check(tmp_path)

    assert result.returncode == 0, result.stdout + result.stderr
    for run_index in (1, 2):
        run = output / f'run{run_index}'
        assert (run / '.complete').read_text().startswith('schema: 2')
        assert 'complete: true' in (run / '.complete').read_text()
        assert (run / 'run_time_v.txt').is_file()
        assert (run / 'run_metrics.yaml').is_file()
        assert (run / 'registration_plugin_receipt.yaml').is_file()
    assert (output / 'offline_determinism_receipt.yaml').is_file()
    assert (output / 'offline_determinism_summary.yaml').is_file()
    summary = json.loads(
        (output / 'offline_determinism_summary.json').read_text())
    assert summary['status'] == 'PASS'
    assert summary['inputs']['bag_tree_sha256']
    assert summary['determinism']['registration_receipts_identical'] is True
    assert all(run['rtf'] >= 0 for run in summary['runs_detail'])


def test_fixture_receipt_mismatch_is_fail_closed(tmp_path):
    result, output = _run_fake_offline_check(tmp_path, receipt_variant='run-specific')

    assert result.returncode != 0
    assert 'MISMATCH (registration receipt)' in result.stdout
    summary = json.loads(
        (output / 'offline_determinism_summary.json').read_text())
    assert summary['status'] == 'FAIL'
    assert summary['determinism']['registration_receipts_identical'] is False


def test_fixture_performance_threshold_is_fail_closed(tmp_path):
    result, output = _run_fake_offline_check(tmp_path, '--max-rtf', '0.0001')

    assert result.returncode != 0
    assert 'exceeds --max-rtf' in result.stderr
    summary = json.loads(
        (output / 'offline_determinism_summary.json').read_text())
    assert summary['status'] == 'FAIL'
    assert summary['determinism']['loop_edges_identical'] is True


def test_fixture_peak_rss_threshold_is_fail_closed(tmp_path):
    result, output = _run_fake_offline_check(tmp_path, '--max-peak-rss-mib', '0')

    assert result.returncode != 0
    assert 'exceeds --max-peak-rss-mib' in result.stderr
    summary = json.loads(
        (output / 'offline_determinism_summary.json').read_text())
    assert summary['status'] == 'FAIL'


def test_negative_wall_cv_threshold_is_rejected(tmp_path):
    result, _ = _run_fake_offline_check(tmp_path, '--max-wall-cv-percent', '-1')

    assert result.returncode == 2
    assert 'finite non-negative number' in result.stderr


def test_invalid_numeric_threshold_is_rejected_without_running_fixture(tmp_path):
    result, _ = _run_fake_offline_check(tmp_path, '--max-rtf', 'not-a-number')

    assert result.returncode == 2
    assert 'finite non-negative number' in result.stderr


def test_required_ape_failure_is_fail_closed(tmp_path):
    reference = tmp_path / 'reference.tum'
    reference.write_text('0.0 0 0 0 0 0 0 1\n', encoding='utf-8')
    result, output = _run_fake_offline_check(
        tmp_path, '--reference-tum', str(reference), '--require-ape')

    assert result.returncode != 0
    assert 'required APE post-processing failed' in result.stderr
    assert not (output / 'offline_determinism_receipt.yaml').exists()


def test_resume_refuses_complete_marker_without_metrics(tmp_path):
    first, output = _run_fake_offline_check(tmp_path)
    assert first.returncode == 0, first.stdout + first.stderr
    (output / 'run1' / 'run_metrics.yaml').unlink()
    setup, params, bag = _make_fake_offline_workspace(tmp_path / 'second')
    # Reuse the first workspace's setup/inputs and output, while making the
    # complete marker stale.  A valid marker must never hide missing metrics.
    environment = os.environ.copy()
    environment['FAKE_GRAPH_PREFIX'] = str(setup.parent / 'prefix')
    result = subprocess.run(
        [
            'bash', str(SCRIPT), '--bag', str(bag), '--params', str(params),
            '--setup', str(setup), '--runs', '2', '--resume',
            '--output-dir', str(output),
        ], cwd=ROOT, capture_output=True, text=True, check=False,
        env=environment)
    assert result.returncode != 0
    assert 'refusing to resume invalid, partial, or identity-mismatched run' in result.stderr


def test_resume_refuses_changed_bag_identity(tmp_path):
    first, output = _run_fake_offline_check(tmp_path)
    assert first.returncode == 0, first.stdout + first.stderr
    (tmp_path / 'bag' / 'input.db3').write_bytes(b'changed-input')
    setup = tmp_path / 'setup.bash'
    params = tmp_path / 'params.yaml'
    bag = tmp_path / 'bag'
    environment = os.environ.copy()
    environment['FAKE_GRAPH_PREFIX'] = str(tmp_path / 'prefix')
    result = subprocess.run(
        [
            'bash', str(SCRIPT), '--bag', str(bag), '--params', str(params),
            '--setup', str(setup), '--runs', '2', '--resume',
            '--output-dir', str(output),
        ], cwd=ROOT, capture_output=True, text=True, check=False,
        env=environment)
    assert result.returncode != 0
    assert 'identity-mismatched' in result.stderr


def test_nonempty_run_directory_is_not_overwritten_without_resume(tmp_path):
    first, output = _run_fake_offline_check(tmp_path)
    assert first.returncode == 0, first.stdout + first.stderr
    setup = tmp_path / 'setup.bash'
    params = tmp_path / 'params.yaml'
    bag = tmp_path / 'bag'
    environment = os.environ.copy()
    environment['FAKE_GRAPH_PREFIX'] = str(tmp_path / 'prefix')
    result = subprocess.run(
        [
            'bash', str(SCRIPT), '--bag', str(bag), '--params', str(params),
            '--setup', str(setup), '--runs', '2', '--output-dir', str(output),
        ], cwd=ROOT, capture_output=True, text=True, check=False,
        env=environment)
    assert result.returncode == 2
    assert 'run directory is non-empty' in result.stderr
