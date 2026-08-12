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

"""Fail closed when a local PR follow-up is not completely review-sliced."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import subprocess
import sys
from typing import Any, Sequence

import jsonschema


sys.dont_write_bytecode = True

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLAN = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'growth'
    / 'g0-publication-slice-plan-2026-08-12.json'
)
DEFAULT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'publication-slice-plan-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/publication-slice-plan-v1.schema.json'
)


class PlanError(ValueError):
    """The publication plan cannot be trusted."""


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise PlanError(f'cannot read {label} {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise PlanError(f'{label} must be a JSON object')
    return payload


def _schema_error_path(error: jsonschema.ValidationError) -> str:
    path = '.'.join(str(item) for item in error.absolute_path)
    return path or '<root>'


def _run_git(arguments: Sequence[str]) -> list[str]:
    command = ['git', *arguments]
    try:
        result = subprocess.run(
            command,
            cwd=REPO_ROOT,
            check=False,
            capture_output=True,
            text=True,
            timeout=60,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise PlanError(f'cannot execute read-only Git inspection: {exc}') from exc
    if result.returncode != 0:
        detail = result.stderr.strip() or 'git returned no error text'
        raise PlanError(f'read-only Git inspection failed: {detail}')
    return [line for line in result.stdout.splitlines() if line]


def candidate_paths(base_sha: str) -> list[str]:
    """Return every tracked or untracked path in the follow-up candidate."""
    tracked = _run_git([
        'diff',
        '--name-only',
        '--diff-filter=ACDMRTUXB',
        base_sha,
        '--',
    ])
    untracked = _run_git(['ls-files', '--others', '--exclude-standard'])
    return sorted(set(tracked + untracked))


def path_inventory_sha256(paths: Sequence[str]) -> str:
    """Hash a canonical newline-terminated path inventory."""
    payload = ''.join(f'{path}\n' for path in paths).encode('utf-8')
    return hashlib.sha256(payload).hexdigest()


def _validate_repo_path(path: str) -> None:
    candidate = PurePosixPath(path)
    if not path or path.startswith('/') or '\\' in path:
        raise PlanError(f'invalid repository-relative path: {path!r}')
    if '\n' in path or '\r' in path or '..' in candidate.parts:
        raise PlanError(f'unsafe repository-relative path: {path!r}')
    if str(candidate) != path or path.startswith('./'):
        raise PlanError(f'non-canonical repository-relative path: {path!r}')


def _validate_candidate_lineage(
    base_sha: str,
    public_head_sha: str,
) -> tuple[str, int]:
    """Require the recorded public PR head between the PR base and local tip."""
    try:
        base_to_public = _run_git(['merge-base', base_sha, public_head_sha])
        public_to_local = _run_git(['merge-base', public_head_sha, 'HEAD'])
        local_tip = _run_git(['rev-parse', 'HEAD'])
        unpublished_count = _run_git([
            'rev-list', '--count', f'{public_head_sha}..HEAD',
        ])
    except PlanError as exc:
        raise PlanError('candidate lineage cannot be verified') from exc
    if base_to_public != [base_sha]:
        raise PlanError('public PR head does not descend from the PR base')
    if public_to_local != [public_head_sha]:
        raise PlanError('local candidate does not descend from public PR head')
    if len(local_tip) != 1 or len(local_tip[0]) != 40:
        raise PlanError('local candidate tip could not be resolved exactly')
    if len(unpublished_count) != 1 or not unpublished_count[0].isdigit():
        raise PlanError('unpublished commit count could not be resolved')
    return local_tip[0], int(unpublished_count[0])


def validate_plan(
    plan: dict[str, Any],
    schema: dict[str, Any],
    actual_paths: Sequence[str],
) -> dict[str, Any]:
    """Validate schema, dependency order, exact coverage, and authority."""
    validator = jsonschema.Draft7Validator(schema)
    errors = sorted(
        validator.iter_errors(plan),
        key=lambda item: [str(part) for part in item.absolute_path],
    )
    if errors:
        first = errors[0]
        raise PlanError(
            f'schema validation failed at {_schema_error_path(first)}: '
            f'{first.message}')
    if plan['schema_uri'] != SCHEMA_URI:
        raise PlanError('plan schema_uri is not the supported v1 URI')

    candidate = plan['candidate']
    local_tip_sha, unpublished_commit_count = _validate_candidate_lineage(
        candidate['base_sha'],
        candidate['public_head_sha'],
    )
    worktree_status = _run_git(['status', '--short'])
    slices = plan['review_slices']
    slice_ids = [item['id'] for item in slices]
    orders = [item['order'] for item in slices]
    if len(slice_ids) != len(set(slice_ids)):
        raise PlanError('review slice ids must not contain duplicates')
    if orders != list(range(1, len(slices) + 1)):
        raise PlanError('review slice orders must be consecutive from one')

    seen_ids: set[str] = set()
    planned_paths: list[str] = []
    for item in slices:
        dependencies = item['depends_on']
        if dependencies != sorted(dependencies):
            raise PlanError(f"{item['id']} dependencies must be sorted")
        unknown = set(dependencies) - seen_ids
        if unknown:
            raise PlanError(
                f"{item['id']} depends on unknown or later slices: "
                f'{sorted(unknown)}')
        paths = item['paths']
        if paths != sorted(paths):
            raise PlanError(f"{item['id']} paths must be sorted")
        if len(paths) != len(set(paths)):
            raise PlanError(f"{item['id']} contains duplicate paths")
        for path in paths:
            _validate_repo_path(path)
        planned_paths.extend(paths)
        seen_ids.add(item['id'])

    duplicates = sorted({
        path for path in planned_paths if planned_paths.count(path) > 1
    })
    if duplicates:
        raise PlanError(f'paths assigned to multiple slices: {duplicates}')

    canonical_planned = sorted(planned_paths)
    canonical_actual = sorted(set(actual_paths))
    missing = sorted(set(canonical_actual) - set(canonical_planned))
    stale = sorted(set(canonical_planned) - set(canonical_actual))
    if missing or stale:
        raise PlanError(
            'candidate path coverage mismatch: '
            f'missing_from_plan={missing}, absent_from_candidate={stale}')

    actual_digest = path_inventory_sha256(canonical_actual)
    if candidate['expected_path_count'] != len(canonical_actual):
        raise PlanError('candidate expected_path_count is stale')
    if candidate['expected_paths_sha256'] != actual_digest:
        raise PlanError('candidate expected_paths_sha256 is stale')

    authority = plan['authority']
    if authority['github_writes_authorized']:
        raise PlanError('a local review plan cannot authorize GitHub writes')
    if authority['remote_mutations_performed']:
        raise PlanError('a local review plan cannot claim remote mutations')

    return {
        'status': 'PLAN_VALID_LOCAL_ONLY',
        'base_sha': candidate['base_sha'],
        'public_head_sha': candidate['public_head_sha'],
        'local_tip_sha': local_tip_sha,
        'unpublished_commit_count': unpublished_commit_count,
        'worktree_clean': not worktree_status,
        'uncommitted_path_count': len(worktree_status),
        'scope': candidate['scope'],
        'pull_request': candidate['pull_request'],
        'path_count': len(canonical_actual),
        'paths_sha256': actual_digest,
        'slice_count': len(slices),
        'slice_ids': slice_ids,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def build_slice_review_report(
    plan: dict[str, Any],
    validation_report: dict[str, Any],
    slice_id: str,
) -> dict[str, Any]:
    """Return one exact, read-only review focus from a validated plan."""
    review_slice = next(
        (item for item in plan['review_slices'] if item['id'] == slice_id),
        None,
    )
    if review_slice is None:
        available = ', '.join(item['id'] for item in plan['review_slices'])
        raise PlanError(
            f'unknown review slice {slice_id!r}; available slices: {available}')

    return {
        'status': 'SLICE_REVIEW_READY_LOCAL_ONLY',
        'candidate': {
            'base_sha': validation_report['base_sha'],
            'public_head_sha': validation_report['public_head_sha'],
            'local_tip_sha': validation_report['local_tip_sha'],
            'unpublished_commit_count': validation_report[
                'unpublished_commit_count'
            ],
            'pull_request': validation_report['pull_request'],
            'slice_count': validation_report['slice_count'],
            'worktree_clean': validation_report['worktree_clean'],
            'uncommitted_path_count': validation_report[
                'uncommitted_path_count'
            ],
        },
        'review_slice': {
            'id': review_slice['id'],
            'order': review_slice['order'],
            'title': review_slice['title'],
            'review_outcome': review_slice['review_outcome'],
            'depends_on': list(review_slice['depends_on']),
            'path_count': len(review_slice['paths']),
            'paths': list(review_slice['paths']),
            'verification': list(review_slice['verification']),
            'publication_gate': review_slice['publication_gate'],
        },
        'commands_executed': False,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def render_slice_review_card(report: dict[str, Any]) -> str:
    """Render one copy-ready, human-facing publication review card."""
    candidate = report['candidate']
    review_slice = report['review_slice']
    dependencies = review_slice['depends_on']
    lines = [
        f"# {review_slice['id']}: {review_slice['title']}",
        '',
        f"- Review order: {review_slice['order']} of {candidate['slice_count']}",
        f"- Paths: {review_slice['path_count']}",
        f"- Depends on: {', '.join(dependencies) if dependencies else 'none'}",
        f"- Publication gate: {review_slice['publication_gate']}",
        f"- Public PR head: {candidate['public_head_sha']}",
        f"- Local HEAD: {candidate['local_tip_sha']}",
        (
            '- Unpublished commits after public head: '
            f"{candidate['unpublished_commit_count']}"
        ),
        f"- Worktree clean: {'yes' if candidate['worktree_clean'] else 'no'}",
        (
            '- Uncommitted paths: '
            f"{candidate['uncommitted_path_count']}"
        ),
        '- Commands executed by this card: no',
        '- GitHub write authorized: no',
        '',
        'Review outcome:',
        review_slice['review_outcome'],
        '',
        'Paths:',
        *(f'- {path}' for path in review_slice['paths']),
        '',
        'Verification commands:',
    ]
    for command in review_slice['verification']:
        lines.extend(['', '```bash', command, '```'])
    lines.extend([
        '',
        (
            'Next action: Review these paths in order, then run the listed '
            'commands and record their results without treating this card as '
            'publication approval.'
        ),
    ])
    return '\n'.join(lines)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--schema', type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument(
        '--slice',
        metavar='ID',
        help='Render one validated review slice without running its commands.',
    )
    parser.add_argument('--json', action='store_true')
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        plan = _load_json(args.plan, 'plan')
        schema = _load_json(args.schema, 'schema')
        paths = candidate_paths(plan.get('candidate', {}).get('base_sha', ''))
        report = validate_plan(plan, schema, paths)
        if args.slice:
            report = build_slice_review_report(plan, report, args.slice)
    except PlanError as exc:
        if args.json:
            print(json.dumps({
                'status': 'PLAN_INVALID',
                'error': str(exc),
                'remote_mutations_performed': False,
            }, sort_keys=True))
        else:
            print(f'ERROR: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    elif args.slice:
        print(render_slice_review_card(report))
    else:
        print(
            'PASS: publication slice plan covers '
            f"{report['path_count']} paths in {report['slice_count']} slices")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
