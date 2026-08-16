#!/usr/bin/env python3
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

"""Render one fail-closed, read-only G0 readiness dashboard.

The dashboard composes the existing local publication-plan, onboarding
matrix, validator-cohort, and v1-readiness checkers. It does not replace any
checker, execute a trial, or write remote state. Product-PR, protected-
environment, and published-release audits are opt-in because they perform
network reads; absence is reported as ``NOT_CHECKED`` rather than a pass.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Callable, Sequence
import urllib.error
import urllib.request

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = (
    REPO_ROOT / 'docs' / 'schemas' / 'g0-readiness-report-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/g0-readiness-report-v1.schema.json'
)
REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
CURRENT_PACKET = 'docs/evidence/growth/g0-current-action-packet-2026-08-14.md'
PRODUCT_PR_NUMBER = 427
PRODUCT_PR_URL = f'https://github.com/{REPOSITORY}/pull/{PRODUCT_PR_NUMBER}'
PRODUCT_PR_BASE = 'develop'
PRODUCT_PR_HEAD = 'agent/product-g0-guided-ux'
PRODUCT_PR_VERIFY_COMMAND = (
    'GITHUB_TOKEN="$(gh auth token)" python3 '
    'scripts/check_g0_readiness.py --include-product-draft --json'
)
MAX_GITHUB_JSON_BYTES = 2 * 1024 * 1024
MAX_CHECK_RUNS = 100
SHA_PATTERN = re.compile(r'^[0-9a-f]{40}$')
REQUIRED_SUCCESS_CHECKS = frozenset({
    'build (humble)',
    'build (jazzy)',
    'candidate gate contract',
    'docs and release metadata',
    'humble default workflow',
    'humble v0.6.0 to candidate',
    'jazzy default workflow',
    'jazzy v0.6.0 to candidate',
    'release readiness',
    'release readiness threshold guard',
})
REQUIRED_SKIPPED_CHECKS = frozenset({
    'authorize immutable candidate request',
    'build and push (${{ matrix.ros_distro }})',
    'publish immutable digest (${{ matrix.ros_distro }})',
    'verify immutable candidate pair',
})
ACCEPTED_EXTRA_CONCLUSIONS = frozenset({'success', 'neutral', 'skipped'})
DEFAULT_RELEASE_VERSION = (
    REPO_ROOT / 'VERSION'
).read_text(encoding='utf-8').strip()
CANDIDATE_ENVIRONMENT_SETTINGS_URL = (
    f'https://github.com/{REPOSITORY}/settings/environments'
)
CANDIDATE_ENVIRONMENT_VERIFY_COMMAND = (
    'GITHUB_TOKEN="$(gh auth token)" python3 '
    'scripts/check_candidate_environment.py --json --require-ready'
)
CANDIDATE_HANDOFF_KINDS = {
    'CREATE_AND_REVIEW_ENVIRONMENT',
    'REPAIR_AND_REVIEW_ENVIRONMENT',
    'RESTORE_READ_ACCESS',
    'REVIEW_E2_SEPARATELY',
}
CANDIDATE_HANDOFF_AUTHORITIES = {
    'repository-settings-admin',
    'read-access',
    'separate-e2-approval',
}
CANDIDATE_HANDOFF_BY_STATUS = {
    'READY': (
        'REVIEW_E2_SEPARATELY',
        'separate-e2-approval',
        False,
        None,
    ),
    'ABSENT': (
        'CREATE_AND_REVIEW_ENVIRONMENT',
        'repository-settings-admin',
        True,
        CANDIDATE_ENVIRONMENT_SETTINGS_URL,
    ),
    'MISCONFIGURED': (
        'REPAIR_AND_REVIEW_ENVIRONMENT',
        'repository-settings-admin',
        True,
        CANDIDATE_ENVIRONMENT_SETTINGS_URL,
    ),
    'BLOCKED': (
        'RESTORE_READ_ACCESS',
        'read-access',
        False,
        None,
    ),
}

COHORT_GATE_GUIDANCE = {
    'public_revision': (
        'name one exact public commit for the selected product version'
    ),
    'public_revision_resolvable': (
        'verify that the exact source commit resolves from the public '
        'repository'
    ),
    'comparable_docker_row': (
        'record one clean Docker PASS at that version with all seven '
        'measurements, including active time, command count, and isolated '
        'peak disk'
    ),
    'comparable_source_row': (
        'record one clean source PASS at that version with all seven '
        'measurements, including active time and command count'
    ),
    'canonical_documentation_path': (
        'select the public Docker First Map or source quickstart route used '
        'by the cohort'
    ),
    'canonical_documentation_url': (
        'bind that route to its canonical documentation URL and route fragment'
    ),
    'canonical_documentation_provenance': (
        'verify the deployed page manifest, exact source revision, and page '
        'SHA-256 with check_public_docs_deployment.py'
    ),
    'canonical_runtime_ref': (
        'bind Docker to an immutable GHCR digest or source to the exact '
        'public commit'
    ),
    'copy_ready_handoff_public': (
        'ensure the public revision contains the copy-ready first-map handoff'
    ),
}


class G0ReadinessError(ValueError):
    """The dashboard cannot safely summarize a checker result."""


Runner = Callable[..., subprocess.CompletedProcess[str]]
GithubFetcher = Callable[[str], tuple[int, dict[str, Any] | None]]
AncestorChecker = Callable[[str, str], bool | None]


def _cohort_gate_guidance(gate: str) -> str:
    """Return bounded human guidance while preserving the machine gate ID."""
    return COHORT_GATE_GUIDANCE.get(
        gate,
        'inspect the first-map cohort contract for the exact missing '
        'prerequisite',
    )


def _checker_command(script: str, *arguments: str) -> list[str]:
    return [sys.executable, str(REPO_ROOT / 'scripts' / script), *arguments]


def _run_json(
    script: str,
    arguments: Sequence[str] = (),
    *,
    runner: Runner = subprocess.run,
) -> dict[str, Any]:
    """Run one existing checker and require a JSON object on stdout.

    The checkers use exit code 1 for an unmet gate, which is still a valid
    report. Exit code 2 or malformed output is an audit error and never gets
    converted into a synthetic HOLD result.
    """
    result = runner(
        _checker_command(script, *arguments),
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode not in (0, 1):
        detail = result.stderr.strip() or 'checker returned no diagnostic'
        raise G0ReadinessError(
            f'{script} failed with exit {result.returncode}: {detail}'
        )
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise G0ReadinessError(
            f'{script} did not emit valid JSON: {exc}'
        ) from exc
    if not isinstance(payload, dict):
        raise G0ReadinessError(f'{script} JSON root is not an object')
    return payload


def _github_json(path: str) -> tuple[int, dict[str, Any] | None]:
    """Perform one bounded GitHub GET for the optional product-PR audit."""
    url = f'https://api.github.com/{path}'
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'lidarslam-g0-product-pr-audit/1',
        'X-GitHub-Api-Version': '2022-11-28',
    }
    token = os.environ.get('GITHUB_TOKEN')
    if token:
        headers['Authorization'] = f'Bearer {token}'
    request = urllib.request.Request(url, headers=headers, method='GET')
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            status = response.status
            payload = response.read(MAX_GITHUB_JSON_BYTES + 1)
    except urllib.error.HTTPError as exc:
        status = exc.code
        payload = exc.read(MAX_GITHUB_JSON_BYTES + 1)
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise G0ReadinessError(
            f'cannot read GitHub product PR API: {exc}'
        ) from exc
    if len(payload) > MAX_GITHUB_JSON_BYTES:
        raise G0ReadinessError(
            'GitHub product PR API response exceeds the byte limit'
        )
    if not payload:
        return status, None
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise G0ReadinessError(
            'GitHub product PR API returned invalid JSON'
        ) from exc
    if not isinstance(value, dict):
        raise G0ReadinessError(
            'GitHub product PR API JSON root is not an object'
        )
    return status, value


def _local_head() -> str:
    """Resolve the exact local review tip without consulting a remote."""
    result = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    head = result.stdout.strip()
    if result.returncode != 0 or SHA_PATTERN.fullmatch(head) is None:
        detail = result.stderr.strip() or 'HEAD is not a full commit ID'
        raise G0ReadinessError(f'cannot resolve local product tip: {detail}')
    return head


def _is_local_ancestor(ancestor: str, descendant: str) -> bool | None:
    """Check local commit ancestry without reading or updating a remote."""
    result = subprocess.run(
        ['git', 'merge-base', '--is-ancestor', ancestor, descendant],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode == 0:
        return True
    if result.returncode == 1:
        return False
    return None


def _product_draft_result(
    *,
    status: str,
    state: str,
    is_draft: bool | None,
    merged: bool | None,
    mergeable: bool | None,
    local_head: str,
    remote_head: str | None,
    head_matches_local: bool | None,
    observed_check_count: int,
    passing_check_count: int,
    skipped_check_count: int,
    pending_check_count: int,
    failing_check_count: int,
    required_checks_complete: bool | None,
    blockers: list[str],
    decision_state: str,
    non_force_update_possible: bool | None = None,
) -> dict[str, Any]:
    """Build one bounded PR result that can never authorize a write."""
    return {
        'status': status,
        'pull_request': PRODUCT_PR_NUMBER,
        'url': PRODUCT_PR_URL,
        'state': state,
        'is_draft': is_draft,
        'merged': merged,
        'mergeable': mergeable,
        'base_ref': PRODUCT_PR_BASE,
        'head_ref': PRODUCT_PR_HEAD,
        'local_head': local_head,
        'remote_head': remote_head,
        'head_matches_local': head_matches_local,
        'non_force_update_possible': non_force_update_possible,
        'observed_check_count': observed_check_count,
        'passing_check_count': passing_check_count,
        'skipped_check_count': skipped_check_count,
        'pending_check_count': pending_check_count,
        'failing_check_count': failing_check_count,
        'required_checks_complete': required_checks_complete,
        'blockers': blockers,
        'decision_state': decision_state,
        'authority': {
            'network_reads_performed': True,
            'github_writes_authorized': False,
            'merge_authorized': False,
            'remote_mutations_performed': False,
        },
    }


def _blocked_product_draft(
    *,
    local_head: str,
    detail: str,
    state: str = 'UNKNOWN',
    is_draft: bool | None = None,
    merged: bool | None = None,
    mergeable: bool | None = None,
    remote_head: str | None = None,
    head_matches_local: bool | None = None,
    observed_check_count: int = 0,
    passing_check_count: int = 0,
    skipped_check_count: int = 0,
    pending_check_count: int = 0,
    failing_check_count: int = 0,
    non_force_update_possible: bool | None = None,
) -> dict[str, Any]:
    """Return a fail-closed PR state without leaking remote free text."""
    return _product_draft_result(
        status='BLOCKED',
        state=state,
        is_draft=is_draft,
        merged=merged,
        mergeable=mergeable,
        local_head=local_head,
        remote_head=remote_head,
        head_matches_local=head_matches_local,
        observed_check_count=observed_check_count,
        passing_check_count=passing_check_count,
        skipped_check_count=skipped_check_count,
        pending_check_count=pending_check_count,
        failing_check_count=failing_check_count,
        required_checks_complete=False,
        blockers=[detail],
        decision_state='HOLD',
        non_force_update_possible=non_force_update_possible,
    )


def _object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise G0ReadinessError(f'{label} must be an object')
    return value


def audit_product_draft(
    *,
    fetcher: GithubFetcher = _github_json,
    local_head: str | None = None,
    ancestor_checker: AncestorChecker = _is_local_ancestor,
) -> dict[str, Any]:
    """Audit exact Draft PR identity and check runs through GitHub GETs."""
    exact_local_head = local_head if local_head is not None else _local_head()
    if SHA_PATTERN.fullmatch(exact_local_head) is None:
        raise G0ReadinessError('local product tip is not a full commit ID')

    try:
        pull_status, pull = fetcher(
            f'repos/{REPOSITORY}/pulls/{PRODUCT_PR_NUMBER}'
        )
    except G0ReadinessError as exc:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=str(exc),
        )
    if pull_status != 200 or pull is None:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=f'Product PR is not readable (HTTP {pull_status}).',
        )

    try:
        head = _object(pull.get('head'), 'product PR head')
        base = _object(pull.get('base'), 'product PR base')
        head_repo = _object(head.get('repo'), 'product PR head repository')
        base_repo = _object(base.get('repo'), 'product PR base repository')
    except G0ReadinessError as exc:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=str(exc),
        )

    remote_head = head.get('sha')
    raw_state = pull.get('state')
    state = raw_state.upper() if raw_state in ('open', 'closed') else 'UNKNOWN'
    is_draft = pull.get('draft')
    merged = pull.get('merged')
    mergeable = pull.get('mergeable')
    head_matches_local = (
        remote_head == exact_local_head
        if isinstance(remote_head, str)
        else None
    )
    identity_valid = (
        pull.get('number') == PRODUCT_PR_NUMBER
        and pull.get('html_url') == PRODUCT_PR_URL
        and raw_state in ('open', 'closed')
        and isinstance(is_draft, bool)
        and isinstance(merged, bool)
        and isinstance(remote_head, str)
        and SHA_PATTERN.fullmatch(remote_head) is not None
        and head.get('ref') == PRODUCT_PR_HEAD
        and base.get('ref') == PRODUCT_PR_BASE
        and head_repo.get('full_name') == REPOSITORY
        and base_repo.get('full_name') == REPOSITORY
    )
    if not identity_valid:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail='Product PR identity or branch contract is invalid.',
            state=state,
            is_draft=is_draft if isinstance(is_draft, bool) else None,
            merged=merged if isinstance(merged, bool) else None,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=(
                remote_head
                if isinstance(remote_head, str)
                and SHA_PATTERN.fullmatch(remote_head) is not None
                else None
            ),
            head_matches_local=head_matches_local,
        )
    if not head_matches_local:
        non_force_update_possible = ancestor_checker(
            remote_head,
            exact_local_head,
        )
        if (
            non_force_update_possible is not None
            and not isinstance(non_force_update_possible, bool)
        ):
            raise G0ReadinessError(
                'product Draft ancestor checker returned an invalid result'
            )
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail='Local and public product PR heads do not match.',
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=False,
            non_force_update_possible=non_force_update_possible,
        )
    if raw_state == 'closed' and not merged:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail='Product PR is closed without being merged.',
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
        )
    if raw_state == 'open' and mergeable is not True:
        detail = (
            'Product PR mergeability is still unknown.'
            if mergeable is None
            else 'Product PR is not mergeable.'
        )
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=detail,
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
        )

    try:
        checks_status, checks = fetcher(
            f'repos/{REPOSITORY}/commits/{remote_head}/check-runs?per_page=100'
        )
    except G0ReadinessError as exc:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=str(exc),
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
        )
    if checks_status != 200 or checks is None:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=(
                f'Exact-head checks are not readable '
                f'(HTTP {checks_status}).'
            ),
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
        )

    runs = checks.get('check_runs')
    total_count = checks.get('total_count')
    if (
        not isinstance(runs, list)
        or isinstance(total_count, bool)
        or not isinstance(total_count, int)
        or total_count != len(runs)
        or total_count > MAX_CHECK_RUNS
    ):
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail='Exact-head check-run inventory is invalid or truncated.',
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
        )

    latest_by_name: dict[str, dict[str, Any]] = {}
    for raw_run in runs:
        if not isinstance(raw_run, dict):
            return _blocked_product_draft(
                local_head=exact_local_head,
                detail='Exact-head check-run inventory contains a bad item.',
                state=state,
                is_draft=is_draft,
                merged=merged,
                mergeable=(
                    mergeable if isinstance(mergeable, bool) else None
                ),
                remote_head=remote_head,
                head_matches_local=True,
            )
        name = raw_run.get('name')
        run_id = raw_run.get('id')
        if (
            not isinstance(name, str)
            or not name
            or len(name) > 200
            or '\n' in name
            or '\r' in name
            or isinstance(run_id, bool)
            or not isinstance(run_id, int)
            or run_id <= 0
        ):
            return _blocked_product_draft(
                local_head=exact_local_head,
                detail='Exact-head check-run identity is invalid.',
                state=state,
                is_draft=is_draft,
                merged=merged,
                mergeable=(
                    mergeable if isinstance(mergeable, bool) else None
                ),
                remote_head=remote_head,
                head_matches_local=True,
            )
        current = latest_by_name.get(name)
        if current is None or run_id > current['id']:
            latest_by_name[name] = raw_run

    passing = 0
    skipped = 0
    pending = 0
    failing = 0
    for run in latest_by_name.values():
        status_value = run.get('status')
        conclusion = run.get('conclusion')
        if status_value != 'completed':
            pending += 1
        elif conclusion in ('success', 'neutral'):
            passing += 1
        elif conclusion == 'skipped':
            skipped += 1
        else:
            failing += 1

    check_blockers: list[str] = []
    missing_success = REQUIRED_SUCCESS_CHECKS - latest_by_name.keys()
    missing_skipped = REQUIRED_SKIPPED_CHECKS - latest_by_name.keys()
    if missing_success:
        check_blockers.append(
            f'{len(missing_success)} required successful checks are missing.'
        )
    if missing_skipped:
        check_blockers.append(
            f'{len(missing_skipped)} expected non-publication skips are '
            'missing.'
        )
    wrong_success = sum(
        1 for name in REQUIRED_SUCCESS_CHECKS
        if name in latest_by_name
        and (
            latest_by_name[name].get('status') != 'completed'
            or latest_by_name[name].get('conclusion') != 'success'
        )
    )
    wrong_skipped = sum(
        1 for name in REQUIRED_SKIPPED_CHECKS
        if name in latest_by_name
        and (
            latest_by_name[name].get('status') != 'completed'
            or latest_by_name[name].get('conclusion') != 'skipped'
        )
    )
    if wrong_success:
        check_blockers.append(
            f'{wrong_success} required successful checks are not successful.'
        )
    if wrong_skipped:
        check_blockers.append(
            f'{wrong_skipped} expected non-publication jobs are not skipped.'
        )
    extra_bad = sum(
        1 for name, run in latest_by_name.items()
        if name not in REQUIRED_SUCCESS_CHECKS
        and name not in REQUIRED_SKIPPED_CHECKS
        and (
            run.get('status') != 'completed'
            or run.get('conclusion') not in ACCEPTED_EXTRA_CONCLUSIONS
        )
    )
    if extra_bad:
        check_blockers.append(
            f'{extra_bad} additional exact-head checks are not complete.'
        )
    if check_blockers:
        return _blocked_product_draft(
            local_head=exact_local_head,
            detail=' '.join(check_blockers),
            state=state,
            is_draft=is_draft,
            merged=merged,
            mergeable=(
                mergeable if isinstance(mergeable, bool) else None
            ),
            remote_head=remote_head,
            head_matches_local=True,
            observed_check_count=len(latest_by_name),
            passing_check_count=passing,
            skipped_check_count=skipped,
            pending_check_count=pending,
            failing_check_count=failing,
        )

    if merged:
        result_status = 'MERGED'
        decision_state = 'MERGED'
    elif is_draft:
        result_status = 'DRAFT_REVIEW_REQUIRED'
        decision_state = 'REVIEW_DRAFT'
    else:
        result_status = 'READY_FOR_SEPARATE_MERGE_REVIEW'
        decision_state = 'READY_FOR_SEPARATE_MERGE_REVIEW'
    return _product_draft_result(
        status=result_status,
        state=state,
        is_draft=is_draft,
        merged=merged,
        mergeable=(mergeable if isinstance(mergeable, bool) else None),
        local_head=exact_local_head,
        remote_head=remote_head,
        head_matches_local=True,
        observed_check_count=len(latest_by_name),
        passing_check_count=passing,
        skipped_check_count=skipped,
        pending_check_count=pending,
        failing_check_count=failing,
        required_checks_complete=True,
        blockers=[],
        decision_state=decision_state,
    )


def collect_checker_reports(
    *,
    include_product_draft: bool = False,
    include_candidate_environment: bool = False,
    include_published_release: bool = False,
    published_release_version: str = DEFAULT_RELEASE_VERSION,
    runner: Runner = subprocess.run,
) -> dict[str, dict[str, Any] | None]:
    """Collect existing checker reports without changing their semantics."""
    reports: dict[str, dict[str, Any] | None] = {
        'publication_plan': _run_json(
            'check_publication_slice_plan.py',
            ('--json',),
            runner=runner,
        ),
        'onboarding_matrix': _run_json(
            'check_onboarding_trial_matrix.py',
            ('--json',),
            runner=runner,
        ),
        'first_map_cohort': _run_json(
            'first_map_validator_cohort.py',
            ('--json',),
            runner=runner,
        ),
        'v1_readiness': _run_json(
            'check_v1_readiness.py',
            ('--json',),
            runner=runner,
        ),
        'product_draft': None,
        'candidate_environment': None,
        'published_release': None,
    }
    if include_product_draft:
        reports['product_draft'] = audit_product_draft()
    if include_candidate_environment:
        reports['candidate_environment'] = _run_json(
            'check_candidate_environment.py',
            ('--json',),
            runner=runner,
        )
    if include_published_release:
        reports['published_release'] = _run_json(
            'check_published_release.py',
            ('--version', published_release_version, '--json'),
            runner=runner,
        )
    return reports


def _matrix_summary(report: dict[str, Any]) -> dict[str, Any]:
    summary = report.get('summary')
    decision = report.get('decision')
    if not isinstance(summary, dict) or not isinstance(decision, dict):
        raise G0ReadinessError('onboarding matrix report is incomplete')
    return {
        'status': decision.get('status'),
        'product_versions': report.get('product_versions', []),
        'present_rows': summary.get('present_rows'),
        'pass_rows': summary.get('pass_rows'),
        'comparable_rows': summary.get('comparable_rows'),
        'docker_comparable_rows': summary.get('docker_comparable_rows'),
        'source_comparable_rows': summary.get('source_comparable_rows'),
        'product_version_aligned': summary.get('product_version_aligned'),
        'activation_gate': summary.get('activation_gate'),
        'actions': decision.get('actions', []),
    }


def _cohort_summary(report: dict[str, Any]) -> dict[str, Any]:
    required = (
        'status',
        'launch_status',
        'attempt_count',
        'accepted_validations',
        'accepted_target',
        'pending_launch_gates',
    )
    if any(field not in report for field in required):
        raise G0ReadinessError('first-map cohort report is incomplete')
    pending_launch_gates = report['pending_launch_gates']
    if not isinstance(pending_launch_gates, list) or not all(
        isinstance(gate, str)
        and gate
        and '\n' not in gate
        and '\r' not in gate
        for gate in pending_launch_gates
    ):
        raise G0ReadinessError(
            'first-map cohort contains unsafe pending launch gate fields'
        )
    return {field: report[field] for field in required}


def _v1_summary(report: dict[str, Any]) -> dict[str, Any]:
    summary = report.get('summary')
    if not isinstance(summary, dict):
        raise G0ReadinessError('v1 readiness report is incomplete')
    gates = report.get('gates', [])
    incomplete_gate_details: list[dict[str, Any]] = []
    for gate in gates:
        if gate.get('status') != 'INCOMPLETE':
            continue
        gate_id = gate.get('id')
        title = gate.get('title')
        detail = gate.get('detail')
        blockers = gate.get('blockers', [])
        if not all(isinstance(value, str) and value for value in (
            gate_id, title, detail,
        )) or not isinstance(blockers, list) or not all(
            isinstance(item, str) and item for item in blockers
        ):
            raise G0ReadinessError(
                'v1 readiness contains an incomplete gate without safe '
                'display fields'
            )
        incomplete_gate_details.append({
            'id': gate_id,
            'title': title,
            'detail': detail,
            'blockers': blockers,
        })
    return {
        'status': report.get('status'),
        'complete': summary.get('complete'),
        'incomplete': summary.get('incomplete'),
        'total': summary.get('total'),
        'incomplete_gates': [
            gate.get('id')
            for gate in gates
            if gate.get('status') == 'INCOMPLETE'
        ],
        'incomplete_gate_details': incomplete_gate_details,
    }


def _published_summary(
    report: dict[str, Any] | None,
    version: str,
) -> dict[str, Any]:
    if report is None:
        return {
            'status': 'NOT_CHECKED',
            'version': version,
            'tag_present': None,
            'image_statuses': [],
        }
    return {
        'status': report.get('status'),
        'version': report.get('expected_version', version),
        'tag_present': report.get('remote', {}).get('tag_present'),
        'image_statuses': [
            {'tag': image.get('tag'), 'status': image.get('status')}
            for image in report.get('images', [])
        ],
    }


def _product_draft_summary(
    report: dict[str, Any] | None,
) -> dict[str, Any]:
    """Keep an optional exact-head PR audit distinct from a review pass."""
    if report is None:
        return {
            'status': 'NOT_CHECKED',
            'pull_request': PRODUCT_PR_NUMBER,
            'url': PRODUCT_PR_URL,
            'state': 'NOT_CHECKED',
            'is_draft': None,
            'merged': None,
            'mergeable': None,
            'base_ref': PRODUCT_PR_BASE,
            'head_ref': PRODUCT_PR_HEAD,
            'local_head': None,
            'remote_head': None,
            'head_matches_local': None,
            'non_force_update_possible': None,
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
    authority = report.get('authority')
    if not isinstance(authority, dict) or authority != {
        'network_reads_performed': True,
        'github_writes_authorized': False,
        'merge_authorized': False,
        'remote_mutations_performed': False,
    }:
        raise G0ReadinessError(
            'product Draft audit claims unsafe or incomplete authority'
        )
    if (
        report.get('pull_request') != PRODUCT_PR_NUMBER
        or report.get('url') != PRODUCT_PR_URL
        or report.get('base_ref') != PRODUCT_PR_BASE
        or report.get('head_ref') != PRODUCT_PR_HEAD
    ):
        raise G0ReadinessError('product Draft audit identity is invalid')
    blockers = report.get('blockers')
    if (
        not isinstance(blockers, list)
        or len(blockers) > 3
        or not all(
            isinstance(item, str)
            and item
            and len(item) <= 1000
            and '\n' not in item
            and '\r' not in item
            for item in blockers
        )
    ):
        raise G0ReadinessError('product Draft audit blockers are unsafe')
    non_force_update_possible = report.get('non_force_update_possible')
    if (
        non_force_update_possible is not None
        and not isinstance(non_force_update_possible, bool)
    ):
        raise G0ReadinessError(
            'product Draft non-force update state is invalid'
        )
    if isinstance(non_force_update_possible, bool) and (
        report.get('status') != 'BLOCKED'
        or report.get('head_matches_local') is not False
        or not isinstance(report.get('local_head'), str)
        or SHA_PATTERN.fullmatch(report['local_head']) is None
        or not isinstance(report.get('remote_head'), str)
        or SHA_PATTERN.fullmatch(report['remote_head']) is None
    ):
        raise G0ReadinessError(
            'product Draft non-force update claim contradicts its state'
        )
    fields = (
        'status',
        'pull_request',
        'url',
        'state',
        'is_draft',
        'merged',
        'mergeable',
        'base_ref',
        'head_ref',
        'local_head',
        'remote_head',
        'head_matches_local',
        'non_force_update_possible',
        'observed_check_count',
        'passing_check_count',
        'skipped_check_count',
        'pending_check_count',
        'failing_check_count',
        'required_checks_complete',
        'decision_state',
    )
    summary = {field: report.get(field) for field in fields}
    summary['blockers'] = list(blockers)
    summary['merge_authorized'] = False
    return summary


def _product_draft_update_handoff(
    product_draft: dict[str, Any],
) -> dict[str, Any]:
    """Build an exact, non-executing fast-forward branch handoff."""
    local_head = product_draft['local_head']
    remote_head = product_draft['remote_head']
    if (
        product_draft['status'] != 'BLOCKED'
        or product_draft['head_matches_local'] is not False
        or product_draft['non_force_update_possible'] is not True
        or not isinstance(local_head, str)
        or SHA_PATTERN.fullmatch(local_head) is None
        or not isinstance(remote_head, str)
        or SHA_PATTERN.fullmatch(remote_head) is None
    ):
        raise G0ReadinessError(
            'product Draft update handoff requires verified fast-forward tips'
        )
    return {
        'kind': 'NON_FORCE_PR_BRANCH_UPDATE',
        'authority_required': (
            'separate-exact-tip-non-force-push-approval'
        ),
        'external_write_required': True,
        'pull_request': PRODUCT_PR_NUMBER,
        'url': PRODUCT_PR_URL,
        'head_ref': PRODUCT_PR_HEAD,
        'public_head': remote_head,
        'local_head': local_head,
        'fast_forward_verified': True,
        'non_force_only': True,
        'push_authorized': False,
        'force_push_authorized': False,
        'steps': [
            (
                f'Confirm PR #{PRODUCT_PR_NUMBER} still has public head '
                f'{remote_head} and the reviewed local tip is {local_head}.'
            ),
            (
                f'Obtain separate exact-tip authority to update '
                f'{PRODUCT_PR_HEAD} from {remote_head} to {local_head} '
                'without force.'
            ),
            (
                'After an authorized external update, rerun the GET-only '
                'exact-head audit below.'
            ),
        ],
        'verification_command': PRODUCT_PR_VERIFY_COMMAND,
        'writes_performed': False,
    }


def _candidate_environment_summary(
    report: dict[str, Any] | None,
) -> dict[str, Any]:
    """Keep an optional live environment audit distinct from a pass."""
    if report is None:
        return {
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
    observed = report.get('observed')
    decision = report.get('decision')
    findings = report.get('findings')
    authority = report.get('authority')
    handoff = report.get('operator_handoff')
    if (
        not isinstance(observed, dict)
        or not isinstance(decision, dict)
        or not isinstance(findings, list)
        or not isinstance(authority, dict)
        or not isinstance(handoff, dict)
    ):
        raise G0ReadinessError(
            'candidate environment report is incomplete'
        )
    if decision.get('dispatch_authorized') is not False or any(
        authority.get(field) is not False
        for field in (
            'github_writes_authorized',
            'environment_writes_authorized',
            'artifact_publication_authorized',
            'remote_mutations_performed',
        )
    ):
        raise G0ReadinessError(
            'candidate environment report claims remote-write authority'
        )
    if (
        handoff.get('kind') not in CANDIDATE_HANDOFF_KINDS
        or handoff.get('authority_required')
        not in CANDIDATE_HANDOFF_AUTHORITIES
        or not isinstance(handoff.get('external_write_required'), bool)
        or handoff.get('writes_performed') is not False
        or handoff.get('verification_command')
        != CANDIDATE_ENVIRONMENT_VERIFY_COMMAND
    ):
        raise G0ReadinessError(
            'candidate environment operator handoff is unsafe'
        )
    settings_url = handoff.get('settings_url')
    if settings_url not in (None, CANDIDATE_ENVIRONMENT_SETTINGS_URL):
        raise G0ReadinessError(
            'candidate environment operator handoff has an untrusted URL'
        )
    expected_handoff = CANDIDATE_HANDOFF_BY_STATUS.get(report.get('status'))
    observed_handoff = (
        handoff.get('kind'),
        handoff.get('authority_required'),
        handoff.get('external_write_required'),
        settings_url,
    )
    if expected_handoff is None or observed_handoff != expected_handoff:
        raise G0ReadinessError(
            'candidate environment operator handoff contradicts its status'
        )
    steps = handoff.get('steps')
    if (
        not isinstance(steps, list)
        or not 2 <= len(steps) <= 5
        or len(steps) != len(set(steps))
        or not all(
            isinstance(step, str)
            and step
            and len(step) <= 300
            and '\n' not in step
            and '\r' not in step
            for step in steps
        )
    ):
        raise G0ReadinessError(
            'candidate environment operator handoff has unsafe steps'
        )
    target = observed.get('target')
    if target is not None and not isinstance(target, dict):
        raise G0ReadinessError(
            'candidate environment target must be an object or null'
        )
    blockers: list[str] = []
    for finding in findings:
        if not isinstance(finding, dict):
            raise G0ReadinessError(
                'candidate environment finding must be an object'
            )
        detail = finding.get('detail')
        if not isinstance(detail, str) or not detail:
            raise G0ReadinessError(
                'candidate environment finding has no safe detail'
            )
        blockers.append(detail)
    return {
        'status': report.get('status'),
        'environment': report.get('environment'),
        'target_present': observed.get('target_present'),
        'required_reviewer_count': (
            target.get('required_reviewer_count') if target else None
        ),
        'prevent_self_review': (
            target.get('prevent_self_review') if target else None
        ),
        'deployment_branch_policy': (
            target.get('deployment_branch_policy') if target else None
        ),
        'decision_state': decision.get('state'),
        'blockers': blockers,
        'operator_handoff': {
            'kind': handoff['kind'],
            'authority_required': handoff['authority_required'],
            'external_write_required': handoff['external_write_required'],
            'settings_url': settings_url,
            'steps': list(steps),
            'verification_command': handoff['verification_command'],
            'writes_performed': False,
        },
    }


def _identity_alternatives(published: dict[str, Any]) -> list[dict[str, str]]:
    """Describe safe identity choices without selecting or publishing one."""
    version = published['version']
    publication_status = (
        'AVAILABLE_FOR_READ_ONLY_PREFLIGHT'
        if published['status'] == 'PUBLISHED'
        else 'REQUIRES_EXTERNAL_PUBLICATION'
    )
    return [
        {
            'id': 'continue-current-candidate',
            'title': f'Continue the current candidate v{version}',
            'status': publication_status,
            'command': (
                'python3 scripts/check_published_release.py '
                f'--version {version} --json'
            ),
            'write_boundary': (
                'read-only preflight; release, tag, and image publication '
                'remain separate'
            ),
        },
        {
            'id': 'rebuild-against-published-version',
            'title': 'Rebuild all rows against one existing public version',
            'status': 'REQUIRES_EXPLICIT_REBASE',
            'command': (
                'python3 scripts/check_published_release.py '
                f'--version {version} --json --require-published | '
                'python3 scripts/prepare_onboarding_matrix_packet.py '
                '--published-release-report - --render'
            ),
            'write_boundary': (
                'local plan only; run a fresh source preflight and never '
                'reuse mixed-version measurements'
            ),
        },
    ]


def _next_action(
    plan: dict[str, Any],
    matrix: dict[str, Any],
    cohort: dict[str, Any],
    v1: dict[str, Any],
    product_draft: dict[str, Any],
    candidate_environment: dict[str, Any],
    published: dict[str, Any],
) -> dict[str, Any]:
    """Choose one safe next action in dependency order."""
    if plan['status'] != 'PLAN_VALID_LOCAL_ONLY':
        return {
            'id': 'repair-publication-plan',
            'title': 'Repair the local publication inventory',
            'reason': 'The exact candidate path plan is not valid.',
            'command': (
                'python3 scripts/check_publication_slice_plan.py --json'
            ),
            'write_boundary': 'read-only',
        }
    if (
        product_draft['status'] == 'NOT_CHECKED'
        and candidate_environment['status'] != 'NOT_CHECKED'
    ):
        return {
            'id': 'inspect-product-draft',
            'title': 'Inspect the product Draft before repository settings',
            'reason': (
                'The candidate environment was inspected before the product '
                'PR merge state was established.'
            ),
            'command': PRODUCT_PR_VERIFY_COMMAND,
            'write_boundary': (
                'GitHub GETs only; marking ready, merging, settings changes, '
                'and E2 dispatch remain separate decisions'
            ),
        }
    if product_draft['status'] == 'BLOCKED':
        blocker = '; '.join(product_draft['blockers']) or (
            'The exact product Draft state could not be established.'
        )
        if (
            product_draft['head_matches_local'] is False
            and product_draft['non_force_update_possible'] is True
        ):
            local_head = product_draft['local_head']
            remote_head = product_draft['remote_head']
            return {
                'id': 'review-product-draft-branch-update',
                'title': 'Review the exact non-force Draft branch update',
                'reason': (
                    f'Public head {remote_head} differs from local tip '
                    f'{local_head}; local ancestry proves that this exact '
                    'transition is a fast-forward.'
                ),
                'command': (
                    f'git merge-base --is-ancestor {remote_head} '
                    f'{local_head}'
                ),
                'write_boundary': (
                    'read-only ancestry check; push requires separate '
                    'exact-tip authority and force push is forbidden'
                ),
                'product_draft_update_handoff': (
                    _product_draft_update_handoff(product_draft)
                ),
            }
        if product_draft['head_matches_local'] is False:
            local_head = product_draft['local_head']
            remote_head = product_draft['remote_head']
            if product_draft['non_force_update_possible'] is False:
                return {
                    'id': 'inspect-product-draft-divergence',
                    'title': 'Inspect the divergent Draft branch history',
                    'reason': (
                        f'Public head {remote_head} is not an ancestor of '
                        f'local tip {local_head}; a non-force update is not '
                        'currently possible.'
                    ),
                    'command': f'git merge-base {remote_head} {local_head}',
                    'write_boundary': (
                        'read-only history inspection; no push, force push, '
                        'PR state change, or merge is authorized'
                    ),
                }
            return {
                'id': 'restore-product-draft-lineage-evidence',
                'title': 'Restore local evidence for Draft branch lineage',
                'reason': (
                    f'Public head {remote_head} and local tip {local_head} '
                    'differ, but the local object database cannot yet prove '
                    'whether a non-force update is possible.'
                ),
                'command': (
                    f'git fetch --no-tags origin {PRODUCT_PR_HEAD} && '
                    f'git merge-base --is-ancestor {remote_head} {local_head}'
                ),
                'write_boundary': (
                    'network read and local Git metadata only; no remote '
                    'write, PR state change, or merge is authorized'
                ),
            }
        return {
            'id': 'repair-product-draft-audit',
            'title': 'Restore an exact product Draft audit',
            'reason': blocker,
            'command': PRODUCT_PR_VERIFY_COMMAND,
            'write_boundary': (
                'GitHub GETs only; no PR state change or merge is authorized'
            ),
        }
    if product_draft['status'] == 'DRAFT_REVIEW_REQUIRED':
        return {
            'id': 'review-product-draft',
            'title': 'Review the exact Draft by publication slice',
            'reason': (
                f"PR #{product_draft['pull_request']} is an exact, mergeable "
                f"Draft at {product_draft['remote_head']} with "
                f"{product_draft['passing_check_count']} passing checks and "
                f"{product_draft['skipped_check_count']} intentional skips."
            ),
            'command': (
                'python3 scripts/check_publication_slice_plan.py --json'
            ),
            'write_boundary': (
                'read-only review; marking ready and merging remain separate '
                'GitHub decisions'
            ),
        }
    if (
        product_draft['status']
        == 'READY_FOR_SEPARATE_MERGE_REVIEW'
    ):
        return {
            'id': 'review-product-merge',
            'title': 'Review the exact PR merge separately',
            'reason': (
                f"PR #{product_draft['pull_request']} is no longer Draft, "
                'but the read-only audit cannot authorize or perform a merge.'
            ),
            'command': PRODUCT_PR_VERIFY_COMMAND,
            'write_boundary': (
                'GitHub GETs only; merge remains a separate maintainer '
                'decision'
            ),
        }
    if (
        not matrix['product_version_aligned']
        and published['status'] != 'PUBLISHED'
        and candidate_environment['status'] not in ('NOT_CHECKED', 'READY')
    ):
        blocker = '; '.join(candidate_environment['blockers']) or (
            'The protected candidate environment is not ready.'
        )
        return {
            'id': 'review-candidate-environment',
            'title': 'Review the protected candidate environment',
            'reason': (
                f"Environment state is {candidate_environment['status']}: "
                f'{blocker}'
            ),
            'command': (
                'GITHUB_TOKEN="$(gh auth token)" python3 '
                'scripts/check_candidate_environment.py '
                '--json --require-ready'
            ),
            'write_boundary': (
                'read-only audit; repository settings and E2 dispatch remain '
                'separate decisions'
            ),
            'operator_handoff': candidate_environment['operator_handoff'],
        }
    if not matrix['product_version_aligned']:
        versions = ', '.join(matrix['product_versions']) or 'multiple versions'
        return {
            'id': 'align-public-product-version',
            'title': 'Resolve one public product version before measuring',
            'reason': (
                f'The reviewed rows use {versions}; do not attach human '
                'measurements to mixed-version rows. The target publication '
                f'audit is currently {published["status"]}.'
            ),
            'command': (
                'python3 scripts/check_g0_readiness.py '
                '--include-published-release '
                f'--published-release-version {published["version"]}'
            ),
            'alternatives': _identity_alternatives(published),
            'write_boundary': (
                'read-only audit; release, tag, and image publication remain '
                'separate'
            ),
        }
    if not matrix['activation_gate']:
        reason = '; '.join(matrix['actions']) or (
            'The Docker/source onboarding matrix has not reached its '
            'activation gate.'
        )
        return {
            'id': 'complete-comparable-onboarding',
            'title': 'Complete same-version measured Docker/source rows',
            'reason': reason,
            'command': (
                'python3 scripts/check_onboarding_trial_matrix.py --json'
            ),
            'write_boundary': (
                'read-only audit; trial execution remains separate'
            ),
        }
    if cohort['launch_status'] != 'READY_FOR_NEXT_ATTEMPT':
        return {
            'id': 'review-cohort-launch-gates',
            'title': 'Review the independent first-map cohort launch gates',
            'reason': '; '.join(
                f'{gate}: {_cohort_gate_guidance(gate)}'
                for gate in cohort['pending_launch_gates']
            ) or (
                f"Cohort state is {cohort['status']}."
            ),
            'command': 'python3 scripts/first_map_validator_cohort.py --json',
            'write_boundary': (
                'read-only; community recruitment remains unauthorized'
            ),
        }
    if v1['status'] != 'READY':
        return {
            'id': 'close-v1-readiness',
            'title': 'Resolve the remaining v1 readiness gates',
            'reason': ', '.join(v1['incomplete_gates']) or (
                f"v1 readiness is {v1['status']}."
            ),
            'command': 'python3 scripts/check_v1_readiness.py --json',
            'write_boundary': (
                'read-only audit; release and adoption decisions remain '
                'separate'
            ),
        }
    if published['status'] != 'PUBLISHED':
        return {
            'id': 'inspect-release-publication',
            'title': 'Inspect or decide the stable release publication gate',
            'reason': (
                'The published-release state is '
                f"{published['status']}; no publication is implied."
            ),
            'command': (
                'python3 scripts/check_published_release.py '
                f"--version {published['version']} --json"
            ),
            'write_boundary': (
                'read-only audit; tag/release/image publication is separate'
            ),
        }
    return {
        'id': 'review-external-gates',
        'title': 'Review the external G0 transition packet',
        'reason': 'Local gates are ready for a separate maintainer decision.',
        'command': (
            'python3 scripts/check_g0_readiness.py '
            '--include-published-release'
        ),
        'write_boundary': 'read-only; no GitHub or community mutation',
    }


def build_report(
    reports: dict[str, dict[str, Any] | None],
    *,
    published_release_version: str = DEFAULT_RELEASE_VERSION,
) -> dict[str, Any]:
    """Build and schema-validate a stable, privacy-safe dashboard report."""
    plan_report = reports.get('publication_plan')
    matrix_report = reports.get('onboarding_matrix')
    cohort_report = reports.get('first_map_cohort')
    v1_report = reports.get('v1_readiness')
    if not all(
        isinstance(item, dict)
        for item in (plan_report, matrix_report, cohort_report, v1_report)
    ):
        raise G0ReadinessError(
            'the four local G0 checker reports are required'
        )

    plan = {
        'status': plan_report.get('status'),
        'path_count': plan_report.get('path_count'),
        'slice_count': plan_report.get('slice_count'),
        'worktree_clean': plan_report.get('worktree_clean'),
        'uncommitted_path_count': plan_report.get('uncommitted_path_count'),
    }
    matrix = _matrix_summary(matrix_report)
    cohort = _cohort_summary(cohort_report)
    v1 = _v1_summary(v1_report)
    published = _published_summary(
        reports.get('published_release'),
        published_release_version,
    )
    candidate_environment = _candidate_environment_summary(
        reports.get('candidate_environment')
    )
    product_draft = _product_draft_summary(reports.get('product_draft'))
    authority = {
        'network_reads_performed': (
            reports.get('published_release') is not None
            or reports.get('candidate_environment') is not None
            or reports.get('product_draft') is not None
        ),
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }
    local_error = plan['status'] != 'PLAN_VALID_LOCAL_ONLY'
    local_ready = (
        not local_error
        and bool(matrix['activation_gate'])
        and cohort['launch_status'] == 'READY_FOR_NEXT_ATTEMPT'
        and v1['status'] == 'READY'
        and published['status'] == 'PUBLISHED'
        and product_draft['status'] in ('NOT_CHECKED', 'MERGED')
    )
    status = 'READY_FOR_REVIEW' if local_ready else 'HOLD'
    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'repository': REPOSITORY,
        'scope': 'local-g0-readiness',
        'status': status,
        'authority': authority,
        'current_packet': {
            'path': CURRENT_PACKET,
            'supersedes_historical_snapshot': True,
        },
        'checks': {
            'publication_plan': plan,
            'onboarding_matrix': matrix,
            'first_map_cohort': cohort,
            'v1_readiness': v1,
            'product_draft': product_draft,
            'candidate_environment': candidate_environment,
            'published_release': published,
        },
        'next_action': _next_action(
            plan,
            matrix,
            cohort,
            v1,
            product_draft,
            candidate_environment,
            published,
        ),
    }
    try:
        schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(schema).validate(report)
    except (OSError, json.JSONDecodeError, jsonschema.SchemaError) as exc:
        raise G0ReadinessError(
            f'dashboard schema cannot be loaded: {exc}'
        ) from exc
    except jsonschema.ValidationError as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise G0ReadinessError(
            f'dashboard schema failed at {location or "<root>"}: {exc.message}'
        ) from exc
    return report


def render_card(report: dict[str, Any]) -> str:
    """Render one concise card with exactly one next action."""
    checks = report['checks']
    plan = checks['publication_plan']
    matrix = checks['onboarding_matrix']
    cohort = checks['first_map_cohort']
    v1 = checks['v1_readiness']
    product_draft = checks['product_draft']
    candidate_environment = checks['candidate_environment']
    published = checks['published_release']
    lines = [
        '# G0 readiness',
        '',
        f"- Overall: **{report['status']}**",
        '- Scope: local, read-only',
        (
            '- Network reads: **yes**'
            if report['authority']['network_reads_performed']
            else '- Network reads: **no**'
        ),
        '- GitHub/community writes: **no**',
        '',
        '| Gate | Status | Evidence summary |',
        '| --- | --- | --- |',
        (
            f"| publication plan | {plan['status']} | "
            f"{plan['path_count']} paths / {plan['slice_count']} slices; "
            f"worktree clean: {str(plan['worktree_clean']).lower()} |"
        ),
        (
            f"| onboarding matrix | {matrix['status']} | "
            f"{matrix['present_rows']}/4 present, "
            f"{matrix['comparable_rows']}/4 comparable, "
            f"activation: {str(matrix['activation_gate']).lower()} |"
        ),
        (
            f"| first-map cohort | {cohort['launch_status']} | "
            f"accepted {cohort['accepted_validations']}/"
            f"{cohort['accepted_target']}; "
            f"attempts {cohort['attempt_count']} |"
        ),
        (
            f"| v1 readiness | {v1['status']} | "
            f"{v1['complete']}/{v1['total']} complete |"
        ),
        (
            f"| product Draft PR #{product_draft['pull_request']} | "
            f"{product_draft['status']} | "
            f"head match: {str(product_draft['head_matches_local']).lower()}; "
            f"checks {product_draft['passing_check_count']} pass / "
            f"{product_draft['skipped_check_count']} skip / "
            f"{product_draft['failing_check_count']} fail; "
            'merge authorized: false |'
        ),
        (
            '| candidate environment | '
            f"{candidate_environment['status']} | "
            f"{candidate_environment['environment']}; "
            'dispatch authorized: false |'
        ),
        (
            f"| published release | {published['status']} | "
            f"v{published['version']} |"
        ),
    ]
    if v1['incomplete_gate_details']:
        lines.extend(['', 'v1 blockers:'])
        for gate in v1['incomplete_gate_details']:
            lines.append(
                f"- {gate['title']} (`{gate['id']}`): {gate['detail']}"
            )
            lines.extend(f'  - {blocker}' for blocker in gate['blockers'])
    if cohort['pending_launch_gates']:
        lines.extend(['', 'first-map cohort blockers:'])
        lines.extend(
            f'- `{gate}` — {_cohort_gate_guidance(gate)}'
            for gate in cohort['pending_launch_gates']
        )
    lines.extend([
        '',
        'Next action:',
        f"{report['next_action']['title']}",
        f"Reason: {report['next_action']['reason']}",
        f"Command: `{report['next_action']['command']}`",
        f"Boundary: {report['next_action']['write_boundary']}",
    ])
    alternatives = report['next_action'].get('alternatives', [])
    if alternatives:
        lines.extend(['', 'Choices (no write):'])
        for alternative in alternatives:
            lines.extend([
                f"- {alternative['title']} — **{alternative['status']}**",
                f"  Command: `{alternative['command']}`",
                f"  Boundary: {alternative['write_boundary']}",
            ])
    handoff = report['next_action'].get('operator_handoff')
    if handoff is not None:
        lines.extend([
            '',
            'Operator handoff (not executed):',
            f"- Authority required: {handoff['authority_required']}",
        ])
        if handoff['settings_url'] is not None:
            lines.append(f"- Settings: {handoff['settings_url']}")
        lines.extend(
            f'{index}. {step}'
            for index, step in enumerate(handoff['steps'], start=1)
        )
        lines.append(
            'Read-only verification: '
            f"`{handoff['verification_command']}`"
        )
        lines.append('- Environment writes performed: no')
    draft_handoff = report['next_action'].get(
        'product_draft_update_handoff'
    )
    if draft_handoff is not None:
        lines.extend([
            '',
            'Draft branch update handoff (not executed):',
            f"- Authority required: {draft_handoff['authority_required']}",
            f"- Head branch: `{draft_handoff['head_ref']}`",
            f"- Public head: `{draft_handoff['public_head']}`",
            f"- Local tip: `{draft_handoff['local_head']}`",
            '- Fast-forward verified: yes',
            '- Non-force only: yes',
        ])
        lines.extend(
            f'{index}. {step}'
            for index, step in enumerate(draft_handoff['steps'], start=1)
        )
        lines.append(
            'Post-update GET-only verification: '
            f"`{draft_handoff['verification_command']}`"
        )
        lines.append('- Pushes performed: no')
    lines.extend(['', f"Current packet: `{report['current_packet']['path']}`"])
    return '\n'.join(lines) + '\n'


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse dashboard CLI options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json', action='store_true')
    parser.add_argument(
        '--include-product-draft',
        action='store_true',
        help='Also run the read-only exact-head Draft PR audit.',
    )
    parser.add_argument(
        '--include-candidate-environment',
        action='store_true',
        help='Also run the read-only protected-environment audit.',
    )
    parser.add_argument(
        '--include-published-release',
        action='store_true',
        help='Also run the read-only remote v0.9.1 publication audit.',
    )
    parser.add_argument(
        '--published-release-version',
        default=DEFAULT_RELEASE_VERSION,
    )
    parser.add_argument(
        '--require-ready',
        action='store_true',
        help='Exit 1 unless all summarized gates are ready for review.',
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Run the dashboard without performing any writes."""
    args = parse_args(argv)
    try:
        reports = collect_checker_reports(
            include_product_draft=args.include_product_draft,
            include_candidate_environment=args.include_candidate_environment,
            include_published_release=args.include_published_release,
            published_release_version=args.published_release_version,
        )
        report = build_report(
            reports,
            published_release_version=args.published_release_version,
        )
    except (G0ReadinessError, OSError) as exc:
        print(f'G0 readiness audit error: {exc}', file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(render_card(report), end='')
    if args.require_ready and report['status'] != 'READY_FOR_REVIEW':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
