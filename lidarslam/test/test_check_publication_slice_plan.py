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
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for the fail-closed local publication slice plan."""

from __future__ import annotations

import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_publication_slice_plan.py'
PLAN_PATH = (
    ROOT
    / 'docs'
    / 'evidence'
    / 'growth'
    / 'g0-publication-slice-plan-2026-08-12.json'
)
SCHEMA_PATH = (
    ROOT / 'docs' / 'schemas' / 'publication-slice-plan-v1.schema.json'
)
SPEC = importlib.util.spec_from_file_location(
    'check_publication_slice_plan',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


def _load(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding='utf-8'))
    assert isinstance(payload, dict)
    return payload


def _plan() -> dict:
    return copy.deepcopy(_load(PLAN_PATH))


def _schema() -> dict:
    return copy.deepcopy(_load(SCHEMA_PATH))


def _planned_paths(plan: dict) -> list[str]:
    return sorted(
        path
        for review_slice in plan['review_slices']
        for path in review_slice['paths']
    )


def test_tracked_plan_covers_the_exact_candidate_once():
    plan = _plan()
    actual = CHECKER.candidate_paths(plan['candidate']['base_sha'])

    report = CHECKER.validate_plan(plan, _schema(), actual)

    assert report['status'] == 'PLAN_VALID_LOCAL_ONLY'
    assert report['base_sha'] == '3f4dd70cdc58ad421192559213cdee0bdc41eba8'
    assert report['public_baseline_sha'] == (
        '3ed632e6f6aa1e3ca7f32d893773de1079086ffb'
    )
    assert report['local_tip_sha'] == CHECKER._run_git(['rev-parse', 'HEAD'])[0]
    expected_follow_ups = int(CHECKER._run_git([
        'rev-list',
        '--count',
        f"{report['public_baseline_sha']}..HEAD",
    ])[0])
    assert report['follow_up_commit_count'] == expected_follow_ups
    status = CHECKER._run_git(['status', '--short'])
    assert report['worktree_clean'] is (not status)
    assert report['uncommitted_path_count'] == len(status)
    assert report['scope'] == 'worktree-delta-from-pr-base'
    assert report['path_count'] == 327
    assert report['slice_count'] == 7
    assert report['remote_mutations_performed'] is False
    assert _planned_paths(plan) == actual


def test_unassigned_candidate_path_is_rejected():
    plan = _plan()
    actual = _planned_paths(plan) + ['scripts/unplanned_follow_up.py']

    with pytest.raises(CHECKER.PlanError, match='missing_from_plan'):
        CHECKER.validate_plan(plan, _schema(), actual)


def test_stale_planned_path_is_rejected():
    plan = _plan()
    actual = _planned_paths(plan)
    actual.remove(plan['review_slices'][0]['paths'][0])

    with pytest.raises(CHECKER.PlanError, match='absent_from_candidate'):
        CHECKER.validate_plan(plan, _schema(), actual)


def test_duplicate_path_ownership_is_rejected():
    plan = _plan()
    duplicate = plan['review_slices'][0]['paths'][0]
    plan['review_slices'][1]['paths'].append(duplicate)
    plan['review_slices'][1]['paths'].sort()

    with pytest.raises(CHECKER.PlanError, match='multiple slices'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(_plan()))


def test_later_slice_dependency_is_rejected():
    plan = _plan()
    plan['review_slices'][0]['depends_on'] = [plan['review_slices'][1]['id']]

    with pytest.raises(CHECKER.PlanError, match='unknown or later'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_non_consecutive_order_is_rejected():
    plan = _plan()
    plan['review_slices'][-1]['order'] = 9

    with pytest.raises(CHECKER.PlanError, match='consecutive'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_stale_inventory_digest_is_rejected():
    plan = _plan()
    plan['candidate']['expected_paths_sha256'] = '0' * 64

    with pytest.raises(CHECKER.PlanError, match='sha256 is stale'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_unresolvable_public_head_is_rejected():
    plan = _plan()
    plan['candidate']['public_head_sha'] = '0' * 40

    with pytest.raises(CHECKER.PlanError, match='lineage cannot be verified'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_plan_cannot_authorize_github_writes():
    plan = _plan()
    plan['authority']['github_writes_authorized'] = True

    with pytest.raises(CHECKER.PlanError, match='schema validation'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_source_quickstart_verification_is_self_contained_and_read_only():
    plan = _plan()
    source_slice = next(
        review_slice
        for review_slice in plan['review_slices']
        if review_slice['id'] == 'S4-source-onboarding'
    )
    command = next(
        item
        for item in source_slice['verification']
        if 'source_quickstart.sh --dry-run' in item
    )

    result = subprocess.run(
        ['bash', '-c', command],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    workspace_line = next(
        line for line in result.stdout.splitlines() if line.startswith('  Workspace: ')
    )
    workspace = Path(workspace_line.removeprefix('  Workspace: '))
    assert not workspace.exists()
    assert 'Commands (--dry-run; nothing executed)' in result.stdout


def test_ros_bag_verification_sources_ros_from_an_unsourced_shell():
    """The S3 copy-ready command must restore its own ROS Python imports."""
    plan = _plan()
    lifecycle_slice = next(
        review_slice
        for review_slice in plan['review_slices']
        if review_slice['id'] == 'S3-map-lifecycle'
    )
    command = next(
        item
        for item in lifecycle_slice['verification']
        if 'test_sensor_setup_wizard.py' in item
    )
    environment = os.environ.copy()
    for name in (
        'AMENT_PREFIX_PATH',
        'CMAKE_PREFIX_PATH',
        'COLCON_PREFIX_PATH',
        'LD_LIBRARY_PATH',
        'PYTHONPATH',
        'ROS_PYTHON_VERSION',
        'ROS_VERSION',
    ):
        environment.pop(name, None)

    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', command],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
        timeout=90,
    )

    assert command.startswith(CHECKER.ROS_SOURCE_PREFIX)
    assert result.returncode == 0, result.stderr
    assert 'passed in' in result.stdout


def test_product_shell_verification_sources_ros_from_an_unsourced_shell():
    """The S6 copy-ready command must restore its ROS bag imports."""
    plan = _plan()
    product_slice = next(
        review_slice
        for review_slice in plan['review_slices']
        if review_slice['id'] == 'S6-product-shell-integration'
    )
    command = next(
        item
        for item in product_slice['verification']
        if 'test_lidarslam_product_cli.py' in item
    )
    environment = os.environ.copy()
    for name in (
        'AMENT_PREFIX_PATH',
        'CMAKE_PREFIX_PATH',
        'COLCON_PREFIX_PATH',
        'LD_LIBRARY_PATH',
        'PYTHONPATH',
        'ROS_PYTHON_VERSION',
        'ROS_VERSION',
    ):
        environment.pop(name, None)

    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', command],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
        timeout=90,
    )

    assert command.startswith(CHECKER.ROS_SOURCE_PREFIX)
    assert result.returncode == 0, result.stderr
    assert 'passed in' in result.stdout


def test_mixed_package_pytest_process_is_rejected():
    """Known duplicate module names require one pytest process per package."""
    plan = _plan()
    plan['review_slices'][4]['verification'].append(
        'python3 -m pytest -q -p no:cacheprovider '
        'lidarslam/test/test_benchmark_summary_profiles.py '
        'graph_based_slam/test/test_ntu_viral_download_script.py'
    )

    with pytest.raises(CHECKER.PlanError, match='separate processes'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_ros_dependent_command_without_source_is_rejected():
    """A valid plan cannot silently depend on the caller's shell state."""
    plan = _plan()
    command = plan['review_slices'][2]['verification'][0]
    plan['review_slices'][2]['verification'][0] = command.removeprefix(
        CHECKER.ROS_SOURCE_PREFIX
    )

    with pytest.raises(CHECKER.PlanError, match='source ROS first'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_product_shell_command_without_source_is_rejected():
    """S6 cannot silently depend on ROS state inherited by the caller."""
    plan = _plan()
    product_slice = next(
        review_slice
        for review_slice in plan['review_slices']
        if review_slice['id'] == 'S6-product-shell-integration'
    )
    index = next(
        index
        for index, item in enumerate(product_slice['verification'])
        if 'test_lidarslam_product_cli.py' in item
    )
    product_slice['verification'][index] = product_slice['verification'][
        index
    ].removeprefix(CHECKER.ROS_SOURCE_PREFIX)

    with pytest.raises(CHECKER.PlanError, match='source ROS first'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_pytest_cache_and_remote_write_commands_are_rejected():
    """Review commands remain local-only and avoid untracked cache writes."""
    plan = _plan()
    plan['review_slices'][0]['verification'][0] = (
        'python3 -m pytest -q graph_based_slam/test/'
        'test_mid360_robot_tools.py -k frame'
    )
    with pytest.raises(CHECKER.PlanError, match='disable its cache'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))

    plan = _plan()
    plan['review_slices'][-1]['verification'].append(
        'gh pr ready 427'
    )
    with pytest.raises(CHECKER.PlanError, match='remote write'):
        CHECKER.validate_plan(plan, _schema(), _planned_paths(plan))


def test_cli_emits_a_machine_readable_local_only_report():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report['status'] == 'PLAN_VALID_LOCAL_ONLY'
    assert report['public_baseline_sha'] == (
        '3ed632e6f6aa1e3ca7f32d893773de1079086ffb'
    )
    assert report['local_tip_sha'] == CHECKER._run_git(['rev-parse', 'HEAD'])[0]
    expected_follow_ups = int(CHECKER._run_git([
        'rev-list',
        '--count',
        f"{report['public_baseline_sha']}..HEAD",
    ])[0])
    assert report['follow_up_commit_count'] == expected_follow_ups
    status = CHECKER._run_git(['status', '--short'])
    assert report['worktree_clean'] is (not status)
    assert report['uncommitted_path_count'] == len(status)
    assert report['path_count'] == 327
    assert report['github_writes_authorized'] is False
    assert report['remote_mutations_performed'] is False


def test_slice_json_binds_exact_scope_without_executing_commands():
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--slice',
            'S1-runtime-safety',
            '--json',
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    review_slice = report['review_slice']
    assert report['status'] == 'SLICE_REVIEW_READY_LOCAL_ONLY'
    assert review_slice['id'] == 'S1-runtime-safety'
    assert review_slice['order'] == 1
    assert report['candidate']['slice_count'] == 7
    assert report['candidate']['uncommitted_path_count'] >= 0
    assert review_slice['path_count'] == 15
    assert review_slice['depends_on'] == []
    assert review_slice['publication_gate'] == 'PUBLIC_CI'
    assert report['commands_executed'] is False
    assert report['github_writes_authorized'] is False
    assert report['remote_mutations_performed'] is False


def test_slice_human_card_has_one_safe_next_action():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--slice', 'S7-publication-control'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert '# S7-publication-control:' in result.stdout
    assert '- Commands executed by this card: no' in result.stdout
    assert '- GitHub write authorized: no' in result.stdout
    assert '- Worktree clean:' in result.stdout
    assert '- Uncommitted paths:' in result.stdout
    assert result.stdout.count('Next action:') == 1


def test_unknown_slice_fails_closed_and_lists_valid_ids():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--slice', 'S8-not-real', '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    report = json.loads(result.stdout)
    assert report['status'] == 'PLAN_INVALID'
    assert 'unknown review slice' in report['error']
    assert 'S1-runtime-safety' in report['error']
    assert report['remote_mutations_performed'] is False
