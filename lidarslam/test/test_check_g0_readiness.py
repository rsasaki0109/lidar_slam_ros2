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
    assert report['checks']['publication_plan']['path_count'] == 248
    assert report['checks']['onboarding_matrix']['comparable_rows'] == 0
    assert report['checks']['published_release']['status'] == 'NOT_CHECKED'
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
    assert report['checks']['first_map_cohort']['pending_launch_gates'] == [
        'comparable_docker_row',
        'comparable_source_row',
        'canonical_documentation_path',
        'canonical_documentation_url',
        'canonical_runtime_ref',
    ]

    card = DASHBOARD.render_card(report)
    assert card.count('Next action:') == 1
    assert 'GitHub/community writes: **no**' in card
    assert 'Choices (no write):' in card
    assert 'never reuse mixed-version measurements' in card
    assert 'v1 blockers:' in card
    assert 'ndt_omp' in card
    assert 'first-map cohort blockers:' in card
    assert 'canonical_runtime_ref' in card
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
    assert 'current 248-path local plan' in scorecard


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
