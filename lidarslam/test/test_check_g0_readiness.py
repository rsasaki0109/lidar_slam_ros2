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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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

"""Tests for the local, read-only G0 readiness dashboard."""

from __future__ import annotations

import copy
import importlib.util
import pathlib
import re
import subprocess

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_g0_readiness.py'
SPEC = importlib.util.spec_from_file_location('check_g0_readiness', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
DASHBOARD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DASHBOARD)


def _absent_environment_handoff() -> dict:
    """Return the exact bounded admin handoff expected from the preflight."""
    return {
        'kind': 'CREATE_AND_REVIEW_ENVIRONMENT',
        'authority_required': 'repository-settings-admin',
        'external_write_required': True,
        'settings_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/'
            'settings/environments'
        ),
        'steps': [
            'Create an environment named candidate-images.',
            'Add 1–6 trusted User or Team required reviewers.',
            'Enable Prevent self-review.',
            (
                'Select custom deployment branches and allow exactly the '
                'develop branch.'
            ),
            'Have an independent maintainer review the saved settings.',
        ],
        'verification_command': (
            'GITHUB_TOKEN="$(gh auth token)" python3 '
            'scripts/check_candidate_environment.py --json --require-ready'
        ),
        'writes_performed': False,
    }


def _product_pull(
    head: str,
    *,
    draft: bool = True,
    state: str = 'open',
    merged: bool = False,
    mergeable: bool | None = True,
) -> dict:
    """Return the bounded PR identity consumed by the live audit."""
    return {
        'number': 427,
        'html_url': 'https://github.com/rsasaki0109/lidar_slam_ros2/pull/427',
        'state': state,
        'draft': draft,
        'merged': merged,
        'mergeable': mergeable,
        'head': {
            'sha': head,
            'ref': 'agent/product-g0-guided-ux',
            'repo': {'full_name': 'rsasaki0109/lidar_slam_ros2'},
        },
        'base': {
            'ref': 'develop',
            'repo': {'full_name': 'rsasaki0109/lidar_slam_ros2'},
        },
    }


def _product_checks(*, failed_name: str | None = None) -> dict:
    """Return the exact expected successful and non-publication check set."""
    runs = []
    run_id = 1
    for name in sorted(DASHBOARD.REQUIRED_SUCCESS_CHECKS):
        runs.append({
            'id': run_id,
            'name': name,
            'status': 'completed',
            'conclusion': 'failure' if name == failed_name else 'success',
        })
        run_id += 1
    for name in sorted(DASHBOARD.REQUIRED_SKIPPED_CHECKS):
        runs.append({
            'id': run_id,
            'name': name,
            'status': 'completed',
            'conclusion': 'skipped',
        })
        run_id += 1
    return {'total_count': len(runs), 'check_runs': runs}


def _audit_product(
    head: str,
    *,
    draft: bool = True,
    state: str = 'open',
    merged: bool = False,
    mergeable: bool | None = True,
    failed_name: str | None = None,
) -> dict:
    """Run the PR audit against deterministic GET fixtures."""
    def fetcher(path: str):
        if path.endswith('/pulls/427'):
            return 200, _product_pull(
                head,
                draft=draft,
                state=state,
                merged=merged,
                mergeable=mergeable,
            )
        assert path.endswith(f'/commits/{head}/check-runs?per_page=100')
        return 200, _product_checks(failed_name=failed_name)

    return DASHBOARD.audit_product_draft(
        fetcher=fetcher,
        local_head=head,
    )


def test_current_dashboard_preserves_the_tracked_hold_state():
    """The current local evidence remains an honest G0 HOLD."""
    reports = DASHBOARD.collect_checker_reports()
    report = DASHBOARD.build_report(reports)

    assert report['status'] == 'HOLD'
    assert report['authority'] == {
        'network_reads_performed': False,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }
    assert report['checks']['publication_plan']['status'] == (
        'PLAN_VALID_LOCAL_ONLY'
    )
    assert report['checks']['publication_plan']['path_count'] == 310
    assert report['checks']['onboarding_matrix']['comparable_rows'] == 0
    assert report['checks']['published_release']['status'] == 'NOT_CHECKED'
    assert report['checks']['product_draft'] == {
        'status': 'NOT_CHECKED',
        'pull_request': 427,
        'url': 'https://github.com/rsasaki0109/lidar_slam_ros2/pull/427',
        'state': 'NOT_CHECKED',
        'is_draft': None,
        'merged': None,
        'mergeable': None,
        'base_ref': 'develop',
        'head_ref': 'agent/product-g0-guided-ux',
        'local_head': None,
        'remote_head': None,
        'head_matches_local': None,
        'observed_check_count': 0,
        'passing_check_count': 0,
        'skipped_check_count': 0,
        'pending_check_count': 0,
        'failing_check_count': 0,
        'required_checks_complete': None,
        'blockers': [],
        'decision_state': 'NOT_CHECKED',
        'merge_authorized': False,
    }
    assert report['checks']['candidate_environment'] == {
        'status': 'NOT_CHECKED',
        'environment': 'candidate-images',
        'target_present': None,
        'required_reviewer_count': None,
        'prevent_self_review': None,
        'deployment_branch_policy': None,
        'decision_state': 'NOT_CHECKED',
        'blockers': [],
        'operator_handoff': None,
    }
    v1_details = report['checks']['v1_readiness']['incomplete_gate_details']
    assert [item['id'] for item in v1_details] == [
        'distribution',
        'external-adoption',
    ]
    assert any(
        'ndt_omp' in blocker
        for item in v1_details
        for blocker in item['blockers']
    )
    assert report['next_action']['id'] == 'align-public-product-version'
    assert '--include-published-release' in report['next_action']['command']
    assert 'mixed-version rows' in report['next_action']['reason']
    alternatives = report['next_action']['alternatives']
    assert [item['id'] for item in alternatives] == [
        'continue-current-candidate',
        'rebuild-against-published-version',
    ]
    assert alternatives[0]['status'] == 'REQUIRES_EXTERNAL_PUBLICATION'
    assert alternatives[1]['status'] == 'REQUIRES_EXPLICIT_REBASE'
    assert '--published-release-report - --render' in (
        alternatives[1]['command']
    )
    assert report['checks']['first_map_cohort']['pending_launch_gates'] == [
        'comparable_docker_row',
        'comparable_source_row',
        'canonical_documentation_path',
        'canonical_documentation_url',
        'canonical_documentation_provenance',
        'canonical_runtime_ref',
    ]

    card = DASHBOARD.render_card(report)
    assert card.count('Next action:') == 1
    assert 'GitHub/community writes: **no**' in card
    assert '| product Draft PR #427 | NOT_CHECKED |' in card
    assert 'Choices (no write):' in card
    assert 'never reuse mixed-version measurements' in card
    assert 'v1 blockers:' in card
    assert 'ndt_omp' in card
    assert 'first-map cohort blockers:' in card
    assert 'canonical_runtime_ref' in card
    assert 'check_public_docs_deployment.py' in card
    assert (
        'record one clean Docker PASS at that version with all seven '
        'measurements' in card
    )
    assert 'immutable GHCR digest' in card
    assert 'g0-current-action-packet-2026-08-14.md' in card

    packet = (
        ROOT / 'docs' / 'evidence' / 'growth'
        / 'g0-current-action-packet-2026-08-14.md'
    ).read_text(encoding='utf-8')
    match = re.search(
        r'Exact reviewed product-candidate tip: `([0-9a-f]{40})`',
        packet,
    )
    assert match is not None
    ancestor_check = subprocess.run(
        ['git', 'merge-base', '--is-ancestor', match.group(1), 'HEAD'],
        cwd=ROOT,
        check=False,
    )
    assert ancestor_check.returncode == 0

    public_baseline_match = re.search(
        r'Capture-time public Draft baseline: `([0-9a-f]{40})`',
        packet,
    )
    assert public_baseline_match is not None
    public_baseline = public_baseline_match.group(1)
    public_baseline_short = f'{public_baseline[:7]}…'
    assert (
        f'capture-time public baseline `{public_baseline_short}`'
        in packet
    )
    assert (
        f'**PASS** for `{public_baseline_short}`: 10 successful checks plus '
        '4 intentionally skipped non-publication jobs, 0 failures'
        in packet
    )
    assert (
        f'source route `READY` at exact public `{public_baseline_short}`'
        in packet
    )
    assert f'--source-commit {public_baseline}' in packet
    baseline_ancestor_check = subprocess.run(
        ['git', 'merge-base', '--is-ancestor', public_baseline, 'HEAD'],
        cwd=ROOT,
        check=False,
    )
    assert baseline_ancestor_check.returncode == 0
    assert 'e222bc490611e6d429f42a1b37778023d55faeb3' not in packet

    scorecard = (
        ROOT / 'docs' / 'growth-scorecard.md'
    ).read_text(encoding='utf-8')
    scorecard_match = re.search(
        r'It binds\nDraft PR #427, the reviewed product-candidate tip\n'
        r'`([0-9a-f]{40})`',
        scorecard,
    )
    assert scorecard_match is not None
    assert scorecard_match.group(1) == match.group(1)
    assert public_baseline in scorecard
    assert (
        '10 successful checks and 4\nintentional non-publication skips'
        in scorecard
    )
    assert 'current 310-path local plan' in scorecard


def test_dashboard_can_include_a_read_only_release_report_without_writes():
    """An optional release report is represented without adding authority."""
    reports = DASHBOARD.collect_checker_reports()
    reports['published_release'] = {
        'status': 'NOT_PUBLISHED',
        'expected_version': '0.9.1',
        'remote': {'tag_present': False},
        'images': [
            {'tag': 'ghcr.io/example:v0.9.1-humble', 'status': 'ABSENT'},
        ],
    }
    report = DASHBOARD.build_report(
        reports,
        published_release_version='0.9.1',
    )

    assert report['authority']['network_reads_performed'] is True
    assert report['checks']['published_release'] == {
        'status': 'NOT_PUBLISHED',
        'version': '0.9.1',
        'tag_present': False,
        'image_statuses': [
            {'tag': 'ghcr.io/example:v0.9.1-humble', 'status': 'ABSENT'},
        ],
    }
    assert report['authority']['remote_mutations_performed'] is False


def test_exact_green_draft_is_reviewed_before_candidate_environment():
    """Exact green Draft evidence takes priority over repository settings."""
    head = '1' * 40
    product = _audit_product(head)

    assert product['status'] == 'DRAFT_REVIEW_REQUIRED'
    assert product['local_head'] == product['remote_head'] == head
    assert product['head_matches_local'] is True
    assert product['mergeable'] is True
    assert product['passing_check_count'] == 10
    assert product['skipped_check_count'] == 4
    assert product['pending_check_count'] == 0
    assert product['failing_check_count'] == 0
    assert product['required_checks_complete'] is True
    assert product['authority']['merge_authorized'] is False

    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = product
    reports['candidate_environment'] = {
        'status': 'ABSENT',
        'environment': 'candidate-images',
        'observed': {'target_present': False, 'target': None},
        'findings': [{
            'id': 'candidate-environment-absent',
            'severity': 'BLOCKER',
            'detail': 'Configure and independently review the environment.',
        }],
        'decision': {
            'state': 'HOLD',
            'dispatch_authorized': False,
            'next_action': 'Configure the environment.',
        },
        'authority': {
            'network_reads_performed': True,
            'github_writes_authorized': False,
            'environment_writes_authorized': False,
            'artifact_publication_authorized': False,
            'remote_mutations_performed': False,
        },
        'operator_handoff': _absent_environment_handoff(),
    }
    report = DASHBOARD.build_report(reports)

    assert report['next_action']['id'] == 'review-product-draft'
    assert report['next_action']['command'] == (
        'python3 scripts/check_publication_slice_plan.py --json'
    )
    assert '10 passing checks and 4 intentional skips' in (
        report['next_action']['reason']
    )
    assert 'marking ready and merging remain separate' in (
        report['next_action']['write_boundary']
    )
    card = DASHBOARD.render_card(report)
    assert '| product Draft PR #427 | DRAFT_REVIEW_REQUIRED |' in card
    assert 'checks 10 pass / 4 skip / 0 fail' in card
    assert 'merge authorized: false' in card
    assert card.count('Next action:') == 1


def test_environment_audit_without_pr_audit_requests_the_missing_dependency():
    """A settings read cannot skip the earlier product merge dependency."""
    reports = DASHBOARD.collect_checker_reports()
    reports['candidate_environment'] = {
        'status': 'ABSENT',
        'environment': 'candidate-images',
        'observed': {'target_present': False, 'target': None},
        'findings': [{
            'id': 'candidate-environment-absent',
            'severity': 'BLOCKER',
            'detail': 'Configure and independently review the environment.',
        }],
        'decision': {
            'state': 'HOLD',
            'dispatch_authorized': False,
            'next_action': 'Configure the environment.',
        },
        'authority': {
            'network_reads_performed': True,
            'github_writes_authorized': False,
            'environment_writes_authorized': False,
            'artifact_publication_authorized': False,
            'remote_mutations_performed': False,
        },
        'operator_handoff': _absent_environment_handoff(),
    }

    report = DASHBOARD.build_report(reports)

    assert report['next_action']['id'] == 'inspect-product-draft'
    assert '--include-product-draft' in report['next_action']['command']
    assert 'settings changes' in report['next_action']['write_boundary']


def test_product_audit_fails_closed_on_drift_failed_ci_and_authority():
    """Head drift, failed required CI, and claimed merge authority all stop."""
    head = '2' * 40

    def drift_fetcher(path: str):
        assert path.endswith('/pulls/427')
        return 200, _product_pull('3' * 40)

    drift = DASHBOARD.audit_product_draft(
        fetcher=drift_fetcher,
        local_head=head,
    )
    assert drift['status'] == 'BLOCKED'
    assert drift['head_matches_local'] is False
    assert drift['required_checks_complete'] is False
    assert drift['blockers'] == [
        'Local and public product PR heads do not match.'
    ]

    failed = _audit_product(
        head,
        failed_name='docs and release metadata',
    )
    assert failed['status'] == 'BLOCKED'
    assert failed['failing_check_count'] == 1
    assert 'required successful checks are not successful' in (
        failed['blockers'][0]
    )

    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = _audit_product(head)
    reports['product_draft']['authority']['merge_authorized'] = True
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='unsafe or incomplete authority',
    ):
        DASHBOARD.build_report(reports)


def test_product_review_state_only_clears_after_exact_merge():
    """Ready-for-review stays ahead of settings; a merged PR may proceed."""
    head = '4' * 40
    ready = _audit_product(head, draft=False)
    assert ready['status'] == 'READY_FOR_SEPARATE_MERGE_REVIEW'
    assert ready['decision_state'] == 'READY_FOR_SEPARATE_MERGE_REVIEW'
    assert ready['authority']['merge_authorized'] is False

    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = ready
    report = DASHBOARD.build_report(reports)
    assert report['next_action']['id'] == 'review-product-merge'

    merged = _audit_product(
        head,
        draft=False,
        state='closed',
        merged=True,
        mergeable=None,
    )
    assert merged['status'] == 'MERGED'
    assert merged['decision_state'] == 'MERGED'


def test_dashboard_surfaces_absent_environment_before_candidate_alignment():
    """A requested E2 preflight becomes the one bounded next action."""
    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = _audit_product(
        '5' * 40,
        draft=False,
        state='closed',
        merged=True,
        mergeable=None,
    )
    reports['candidate_environment'] = {
        'status': 'ABSENT',
        'environment': 'candidate-images',
        'observed': {
            'target_present': False,
            'target': None,
        },
        'findings': [{
            'id': 'candidate-environment-absent',
            'severity': 'BLOCKER',
            'detail': 'Configure and independently review the environment.',
        }],
        'decision': {
            'state': 'HOLD',
            'dispatch_authorized': False,
            'next_action': 'Configure the environment.',
        },
        'authority': {
            'network_reads_performed': True,
            'github_writes_authorized': False,
            'environment_writes_authorized': False,
            'artifact_publication_authorized': False,
            'remote_mutations_performed': False,
        },
        'operator_handoff': _absent_environment_handoff(),
    }

    report = DASHBOARD.build_report(reports)

    assert report['authority']['network_reads_performed'] is True
    assert report['checks']['candidate_environment']['status'] == 'ABSENT'
    assert report['next_action']['id'] == 'review-candidate-environment'
    assert '--require-ready' in report['next_action']['command']
    handoff = report['checks']['candidate_environment']['operator_handoff']
    assert handoff == _absent_environment_handoff()
    assert report['next_action']['operator_handoff'] == handoff
    assert 'E2 dispatch remain separate' in (
        report['next_action']['write_boundary']
    )
    card = DASHBOARD.render_card(report)
    assert '| candidate environment | ABSENT |' in card
    assert 'dispatch authorized: false' in card
    assert 'Operator handoff (not executed):' in card
    assert 'Authority required: repository-settings-admin' in card
    assert '/settings/environments' in card
    assert '1. Create an environment named candidate-images.' in card
    assert 'Environment writes performed: no' in card

    unsafe = copy.deepcopy(reports)
    unsafe['candidate_environment']['decision'][
        'dispatch_authorized'
    ] = True
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='remote-write authority',
    ):
        DASHBOARD.build_report(unsafe)

    unsafe = copy.deepcopy(reports)
    unsafe['candidate_environment']['operator_handoff'][
        'writes_performed'
    ] = True
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='operator handoff is unsafe',
    ):
        DASHBOARD.build_report(unsafe)

    unsafe = copy.deepcopy(reports)
    unsafe['candidate_environment']['operator_handoff']['settings_url'] = (
        'https://example.com/settings'
    )
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='untrusted URL',
    ):
        DASHBOARD.build_report(unsafe)

    unsafe = copy.deepcopy(reports)
    unsafe['candidate_environment']['operator_handoff']['kind'] = (
        'REPAIR_AND_REVIEW_ENVIRONMENT'
    )
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='contradicts its status',
    ):
        DASHBOARD.build_report(unsafe)


def test_dashboard_selects_one_next_action_and_ready_state_is_explicit():
    """A fully ready synthetic report gets one explicit review action."""
    reports = DASHBOARD.collect_checker_reports()
    ready = copy.deepcopy(reports)
    ready['onboarding_matrix']['decision']['status'] = 'ACTIVATION_GATE_PASS'
    ready['onboarding_matrix']['summary'].update({
        'activation_gate': True,
        'comparable_rows': 2,
        'docker_comparable_rows': 1,
        'source_comparable_rows': 1,
        'product_version_aligned': True,
    })
    ready['first_map_cohort'].update({
        'status': 'READY_FOR_NEXT_ATTEMPT',
        'launch_status': 'READY_FOR_NEXT_ATTEMPT',
        'pending_launch_gates': [],
    })
    ready['v1_readiness'].update({
        'status': 'READY',
        'summary': {'complete': 10, 'incomplete': 0, 'total': 10},
        'gates': [],
    })
    ready['published_release'] = {
        'status': 'PUBLISHED',
        'expected_version': '0.9.1',
        'remote': {'tag_present': True},
        'images': [],
    }

    report = DASHBOARD.build_report(
        ready,
        published_release_version='0.9.1',
    )

    assert report['status'] == 'READY_FOR_REVIEW'
    assert report['next_action']['id'] == 'review-external-gates'
    assert report['next_action']['write_boundary'].startswith('read-only')


def test_checker_error_is_not_downgraded_to_a_hold():
    """A checker execution error remains an error, never a synthetic HOLD."""
    def failing_runner(*_args, **_kwargs):
        return DASHBOARD.subprocess.CompletedProcess(
            args=[],
            returncode=2,
            stdout='',
            stderr='synthetic checker failure',
        )

    with pytest.raises(DASHBOARD.G0ReadinessError, match='synthetic'):
        DASHBOARD.collect_checker_reports(runner=failing_runner)
