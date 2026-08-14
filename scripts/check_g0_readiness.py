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
checker, execute a trial, or write remote state. A published-release audit is
opt-in because it performs network reads; its absence is reported as
``NOT_CHECKED`` rather than being mistaken for a pass.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable, Sequence

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
DEFAULT_RELEASE_VERSION = (
    REPO_ROOT / 'VERSION'
).read_text(encoding='utf-8').strip()


class G0ReadinessError(ValueError):
    """The dashboard cannot safely summarize a checker result."""


Runner = Callable[..., subprocess.CompletedProcess[str]]


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


def collect_checker_reports(
    *,
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
        'published_release': None,
    }
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
        isinstance(gate, str) and gate and '\n' not in gate and '\r' not in gate
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
                'python3 scripts/prepare_onboarding_matrix_packet.py --help'
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
            'reason': ', '.join(cohort['pending_launch_gates']) or (
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
    authority = {
        'network_reads_performed': (
            reports.get('published_release') is not None
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
            'published_release': published,
        },
        'next_action': _next_action(
            plan, matrix, cohort, v1, published,
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
            lines.extend(f"  - {blocker}" for blocker in gate['blockers'])
    if cohort['pending_launch_gates']:
        lines.extend(['', 'first-map cohort blockers:'])
        lines.extend(
            f'- {gate}' for gate in cohort['pending_launch_gates']
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
    lines.extend(['', f"Current packet: `{report['current_packet']['path']}`"])
    return '\n'.join(lines) + '\n'


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse dashboard CLI options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json', action='store_true')
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
