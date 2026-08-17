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
import hashlib
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
    body: str | None = '',
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
        'body': body,
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
    body: str | None = '',
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
                body=body,
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


def _desired_description(reports: dict, head: str) -> str:
    """Render the canonical body from deterministic local fixtures."""
    return DASHBOARD._product_draft_description_body(
        reports['publication_plan'],
        DASHBOARD._matrix_summary(reports['onboarding_matrix']),
        DASHBOARD._cohort_summary(reports['first_map_cohort']),
        DASHBOARD._v1_summary(reports['v1_readiness']),
        head,
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
    assert report['checks']['publication_plan']['path_count'] == 331
    assert report['checks']['publication_plan'][
        'whole_pr_commit_count'
    ] >= 315
    assert report['checks']['publication_plan'][
        'follow_up_review_commit_count'
    ] >= 271
    assert report['checks']['publication_plan']['whole_pr_path_count'] == 380
    assert report['checks']['publication_plan']['review_phase_count'] == 3
    assert report['checks']['publication_plan'][
        'review_coverage_complete'
    ] is True
    assert report['checks']['publication_plan']['bridge_path_count'] == 11
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
        'non_force_update_possible': None,
        'remote_description_sha256': None,
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
    assert 'current 331-path local plan' in scorecard
    assert (
        'complete 380-path / three-phase whole-PR review coverage'
        in scorecard
    )


def test_dashboard_rejects_incomplete_whole_pr_review_coverage():
    """A valid slice inventory cannot hide an uncovered historical gap."""
    reports = DASHBOARD.collect_checker_reports()
    reports['publication_plan']['review_coverage_complete'] = False

    report = DASHBOARD.build_report(reports)

    assert report['status'] == 'HOLD'
    assert report['next_action']['id'] == 'repair-publication-plan'
    assert 'whole-PR review coverage is not valid' in (
        report['next_action']['reason']
    )
    assert report['authority']['github_writes_authorized'] is False


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
    reports = DASHBOARD.collect_checker_reports()
    reports['publication_plan']['local_tip_sha'] = head
    reports['publication_plan']['worktree_clean'] = True
    reports['publication_plan']['uncommitted_path_count'] = 0
    body = _desired_description(reports, head)
    product = _audit_product(head, body=body)

    assert product['status'] == 'DRAFT_REVIEW_REQUIRED'
    assert product['local_head'] == product['remote_head'] == head
    assert product['head_matches_local'] is True
    assert product['mergeable'] is True
    assert product['passing_check_count'] == 10
    assert product['skipped_check_count'] == 4
    assert product['pending_check_count'] == 0
    assert product['failing_check_count'] == 0
    assert product['required_checks_complete'] is True
    assert product['remote_description_sha256'] == hashlib.sha256(
        body.encode('utf-8')
    ).hexdigest()
    assert product['authority']['merge_authorized'] is False

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
        'python3 scripts/check_publication_slice_plan.py --overview'
    )
    assert '10 passing checks and 4 intentional skips' in (
        report['next_action']['reason']
    )
    assert 'marking ready and merging remain separate' in (
        report['next_action']['write_boundary']
    )
    handoff = report['next_action']['product_draft_review_handoff']
    assert handoff == {
        'kind': 'EXACT_DRAFT_REVIEW_SEQUENCE',
        'external_write_required': False,
        'pull_request': 427,
        'url': 'https://github.com/rsasaki0109/lidar_slam_ros2/pull/427',
        'exact_head': head,
        'whole_pr_path_count': 380,
        'review_phase_count': 3,
        'slice_count': 7,
        'overview_command': (
            'python3 scripts/check_publication_slice_plan.py --overview'
        ),
        'slice_command_template': (
            'python3 scripts/check_publication_slice_plan.py --slice <ID>'
        ),
        'steps': [
            'Render the exact overview and confirm a clean matching tip.',
            'Review P0, P1, then P2 using their bounded hotspots.',
            (
                'Review S1 through S7 in dependency order and run each '
                'displayed verification group.'
            ),
            (
                'Record findings separately; review submission, mark-ready, '
                'and merge remain separate GitHub decisions.'
            ),
        ],
        'commands_executed': False,
        'github_review_submitted': False,
        'mark_ready_authorized': False,
        'merge_authorized': False,
        'writes_performed': False,
    }
    card = DASHBOARD.render_card(report)
    assert '| product Draft PR #427 | DRAFT_REVIEW_REQUIRED |' in card
    assert 'checks 10 pass / 4 skip / 0 fail' in card
    assert 'merge authorized: false' in card
    assert 'Draft review sequence (not executed):' in card
    assert f'- Exact head: `{head}`' in card
    assert '- Coverage: 380 paths / 3 phases / 7 slices' in card
    assert (
        'Slice template: `python3 scripts/check_publication_slice_plan.py '
        '--slice <ID>`'
    ) in card
    assert '- GitHub review submitted: no' in card
    assert card.count('Next action:') == 1


def test_exact_green_draft_refreshes_stale_description_before_review():
    """A green exact head cannot carry stale reviewer-facing scope facts."""
    head = '9' * 40
    reports = DASHBOARD.collect_checker_reports()
    reports['publication_plan']['local_tip_sha'] = head
    reports['publication_plan']['worktree_clean'] = True
    reports['publication_plan']['uncommitted_path_count'] = 0
    reports['product_draft'] = _audit_product(
        head,
        body='stale 321-path / 77-commit description',
    )

    report = DASHBOARD.build_report(reports)
    action = report['next_action']

    assert action['id'] == 'review-product-draft-description-refresh'
    handoff = action['product_draft_description_refresh_handoff']
    assert handoff['desired_head'] == head
    assert handoff['observed_public_head'] == head
    assert handoff['after_branch_update_required'] is False
    assert handoff['body_matches_observed'] is False
    assert handoff['body_sha256'] == hashlib.sha256(
        handoff['body'].encode('utf-8')
    ).hexdigest()
    assert f'Candidate head: `{head}`' in handoff['body']
    assert 'Whole PR review: **' in handoff['body']
    assert '380 paths / 3 phases**' in handoff['body']
    assert '331 paths / 7 slices**' in handoff['body']
    assert '0/4 comparable' in handoff['body']
    assert handoff['description_update_authorized'] is False
    assert handoff['review_submission_authorized'] is False
    assert handoff['mark_ready_authorized'] is False
    assert handoff['merge_authorized'] is False
    assert handoff['writes_performed'] is False
    assert 'gh pr edit' not in repr(action)

    card = DASHBOARD.render_card(report)
    assert 'Draft description refresh handoff (not executed):' in card
    assert 'Exact desired PR description:' in card
    assert handoff['body_sha256'] in card
    assert '- Description updates performed: no' in card


def test_exact_green_draft_refuses_review_handoff_from_dirty_worktree():
    """Uncommitted bytes cannot be mislabeled as the exact public review."""
    head = '2' * 40
    reports = DASHBOARD.collect_checker_reports()
    reports['publication_plan']['worktree_clean'] = False
    reports['publication_plan']['uncommitted_path_count'] = 2
    reports['product_draft'] = _audit_product(head)

    report = DASHBOARD.build_report(reports)

    assert report['next_action'] == {
        'id': 'restore-clean-draft-review-worktree',
        'title': 'Restore a clean worktree before exact Draft review',
        'reason': (
            'The local publication plan has 2 uncommitted paths, so its '
            'rendered review budget would not describe exact public head '
            f'{head}.'
        ),
        'command': 'git status --short',
        'write_boundary': (
            'read-only local inspection; no file cleanup, commit, push, '
            'review submission, mark-ready, or merge is authorized'
        ),
    }
    assert report['authority']['github_writes_authorized'] is False


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
        ancestor_checker=lambda _ancestor, _descendant: False,
    )
    assert drift['status'] == 'BLOCKED'
    assert drift['head_matches_local'] is False
    assert drift['non_force_update_possible'] is False
    assert drift['required_checks_complete'] is False
    assert drift['blockers'] == [
        'Local and public product PR heads do not match.'
    ]

    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = drift
    divergence = DASHBOARD.build_report(reports)['next_action']
    assert divergence['id'] == 'inspect-product-draft-divergence'
    assert divergence['command'] == f"git merge-base {'3' * 40} {head}"
    assert divergence['command'] != DASHBOARD.PRODUCT_PR_VERIFY_COMMAND

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

    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = _audit_product(head)
    reports['product_draft']['non_force_update_possible'] = False
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='non-force update claim contradicts its state',
    ):
        DASHBOARD.build_report(reports)


def test_product_audit_bounds_and_validates_description_digest():
    """Remote free text is hashed only when it satisfies the size contract."""
    head = 'a' * 40

    def malformed_fetcher(path: str):
        assert path.endswith('/pulls/427')
        pull = _product_pull(head)
        pull['body'] = {'unexpected': 'object'}
        return 200, pull

    malformed = DASHBOARD.audit_product_draft(
        fetcher=malformed_fetcher,
        local_head=head,
    )
    assert malformed['status'] == 'BLOCKED'
    assert malformed['blockers'] == [
        'Product PR description is not text or null.'
    ]
    assert malformed['remote_description_sha256'] is None

    def oversized_fetcher(path: str):
        assert path.endswith('/pulls/427')
        return 200, _product_pull(
            head,
            body='x' * (DASHBOARD.MAX_PRODUCT_DESCRIPTION_BYTES + 1),
        )

    oversized = DASHBOARD.audit_product_draft(
        fetcher=oversized_fetcher,
        local_head=head,
    )
    assert oversized['status'] == 'BLOCKED'
    assert oversized['blockers'] == [
        'Product PR description exceeds the audit size limit.'
    ]

    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = _audit_product(head)
    reports['product_draft']['remote_description_sha256'] = 'bad-digest'
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='description digest is invalid',
    ):
        DASHBOARD.build_report(reports)


def test_head_drift_emits_exact_non_force_handoff_without_push_command():
    """Fast-forward drift gets a bounded handoff, not a circular re-audit."""
    local_head = '6' * 40
    public_head = '5' * 40

    def drift_fetcher(path: str):
        assert path.endswith('/pulls/427')
        return 200, _product_pull(public_head)

    product = DASHBOARD.audit_product_draft(
        fetcher=drift_fetcher,
        local_head=local_head,
        ancestor_checker=lambda ancestor, descendant: (
            ancestor == public_head and descendant == local_head
        ),
    )
    assert product['non_force_update_possible'] is True

    reports = DASHBOARD.collect_checker_reports()
    reports['publication_plan']['local_tip_sha'] = local_head
    reports['publication_plan']['worktree_clean'] = True
    reports['publication_plan']['uncommitted_path_count'] = 0
    reports['product_draft'] = product
    report = DASHBOARD.build_report(reports)
    action = report['next_action']

    assert action['id'] == 'review-product-draft-branch-update'
    assert action['command'] == (
        f'git merge-base --is-ancestor {public_head} {local_head}'
    )
    assert action['command'] != DASHBOARD.PRODUCT_PR_VERIFY_COMMAND
    handoff = action['product_draft_update_handoff']
    assert handoff['repository_url'] == DASHBOARD.PRODUCT_REPOSITORY_URL
    assert handoff['public_head'] == public_head
    assert handoff['local_head'] == local_head
    assert handoff['fast_forward_verified'] is True
    assert handoff['non_force_only'] is True
    assert handoff['push_authorized'] is False
    assert handoff['force_push_authorized'] is False
    assert handoff['writes_performed'] is False
    assert handoff['verification_command'] == DASHBOARD.PRODUCT_PR_VERIFY_COMMAND
    description = action['product_draft_description_refresh_handoff']
    assert description['observed_public_head'] == public_head
    assert description['desired_head'] == local_head
    assert description['after_branch_update_required'] is True
    assert description['clean_tip_verified'] is True
    assert description['body_matches_observed'] is False
    assert description['body_sha256'] == hashlib.sha256(
        description['body'].encode('utf-8')
    ).hexdigest()
    assert f'Candidate head: `{local_head}`' in description['body']
    assert description['description_update_authorized'] is False
    assert description['mark_ready_authorized'] is False
    assert description['merge_authorized'] is False
    assert description['writes_performed'] is False
    assert 'git push' not in repr(action)

    card = DASHBOARD.render_card(report)
    assert 'Draft branch update handoff (not executed):' in card
    assert f'Public head: `{public_head}`' in card
    assert f'Local tip: `{local_head}`' in card
    assert f'Repository: `{DASHBOARD.PRODUCT_REPOSITORY_URL}`' in card
    assert 'Fast-forward verified: yes' in card
    assert 'Pushes performed: no' in card
    assert 'Draft description refresh handoff (not executed):' in card
    assert f'Desired head: `{local_head}`' in card
    assert 'Branch update required first: yes' in card
    assert '- Description updates performed: no' in card
    assert 'git push' not in card

    dirty_reports = copy.deepcopy(reports)
    dirty_reports['publication_plan']['worktree_clean'] = False
    dirty_reports['publication_plan']['uncommitted_path_count'] = 1
    dirty_action = DASHBOARD.build_report(dirty_reports)['next_action']
    assert dirty_action['id'] == 'restore-clean-draft-update-worktree'
    assert 'product_draft_update_handoff' not in dirty_action
    assert 'product_draft_description_refresh_handoff' not in dirty_action
    assert 'no cleanup, commit, push, PR edit' in (
        dirty_action['write_boundary']
    )


def test_missing_lineage_fetches_the_canonical_repository_not_origin():
    """Lineage recovery cannot trust a checkout-specific origin remote."""
    local_head = '8' * 40
    public_head = '7' * 40

    def drift_fetcher(path: str):
        assert path.endswith('/pulls/427')
        return 200, _product_pull(public_head)

    product = DASHBOARD.audit_product_draft(
        fetcher=drift_fetcher,
        local_head=local_head,
        ancestor_checker=lambda _ancestor, _descendant: None,
    )
    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = product
    action = DASHBOARD.build_report(reports)['next_action']

    assert action['id'] == 'restore-product-draft-lineage-evidence'
    assert DASHBOARD.PRODUCT_REPOSITORY_URL in action['command']
    assert ' origin ' not in action['command']
    assert action['command'].endswith(
        f'git merge-base --is-ancestor {public_head} {local_head}'
    )
    assert 'no remote write' in action['write_boundary']


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
