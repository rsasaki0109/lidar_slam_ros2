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

"""Inspect and verify the bounded local contributor starter queue."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path, PurePosixPath
from typing import Any, Sequence

import jsonschema


sys.dont_write_bytecode = True

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_QUEUE = (
    REPO_ROOT / 'docs' / 'contracts' / 'contributor-starter-queue-v1.json'
)
DEFAULT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'contributor-starter-queue-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/contributor-starter-queue-v1.schema.json'
)
EXPECTED_TASK_IDS = tuple(f'starter-C{index}' for index in range(5, 10))
COMMON_LABELS = {'good first issue', 'help wanted'}
DOMAIN_LABELS = {'bug', 'documentation'}

DOCS_STRICT_CHECKS = (
    ('python3', '-m', 'mkdocs', 'build', '--strict'),
)
MID360_EMPTY_FRAME_CHECKS = (
    (
        'python3',
        '-m',
        'pytest',
        '-q',
        'graph_based_slam/test/test_mid360_robot_tools.py',
        '-k',
        'frame',
    ),
    (
        'python3',
        '-m',
        'flake8',
        '--select=E9,F63,F7,F82',
        'scripts/lidarslam_tools/mid360_preflight.py',
        'graph_based_slam/test/test_mid360_robot_tools.py',
    ),
    ('git', 'diff', '--check'),
)
PROFILE_CHECKS = {
    'docs-strict': DOCS_STRICT_CHECKS,
    'mid360-empty-frame': MID360_EMPTY_FRAME_CHECKS,
}


class QueueError(ValueError):
    """The local starter queue cannot be trusted."""


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise QueueError(f'cannot read {label} {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise QueueError(f'{label} must be a JSON object')
    return payload


def _schema_error_path(error: jsonschema.ValidationError) -> str:
    path = '.'.join(str(item) for item in error.absolute_path)
    return path or '<root>'


def _validate_repo_path(path: str) -> None:
    candidate = PurePosixPath(path)
    if not path or path.startswith('/') or '\\' in path:
        raise QueueError(f'invalid repository-relative path: {path!r}')
    if '\n' in path or '\r' in path or '..' in candidate.parts:
        raise QueueError(f'unsafe repository-relative path: {path!r}')
    if str(candidate) != path or path.startswith('./'):
        raise QueueError(f'non-canonical repository-relative path: {path!r}')


def _declared_checks(task: dict[str, Any]) -> tuple[tuple[str, ...], ...]:
    return tuple(tuple(item['argv']) for item in task['focused_checks'])


def _validate_task(task: dict[str, Any], repo_root: Path) -> None:
    task_id = task['id']
    if task['labels'] != sorted(task['labels']):
        raise QueueError(f'{task_id} labels must be sorted')
    labels = set(task['labels'])
    if not COMMON_LABELS.issubset(labels):
        raise QueueError(f'{task_id} is missing required starter labels')
    domain = labels - COMMON_LABELS
    if len(domain) != 1 or not domain.issubset(DOMAIN_LABELS):
        raise QueueError(f'{task_id} must have exactly one domain label')

    if task['source_issue_numbers'] != sorted(task['source_issue_numbers']):
        raise QueueError(f'{task_id} source issue numbers must be sorted')
    if task['allowed_paths'] != sorted(task['allowed_paths']):
        raise QueueError(f'{task_id} allowed paths must be sorted')

    for path in task['allowed_paths']:
        _validate_repo_path(path)
        candidate = repo_root / path
        if not candidate.is_file() or candidate.is_symlink():
            raise QueueError(
                f'{task_id} allowed path is not a regular file: {path}')

    profile = task['check_profile']
    expected_checks = PROFILE_CHECKS.get(profile)
    if expected_checks is None:
        raise QueueError(f'{task_id} uses unsupported check profile {profile}')
    if _declared_checks(task) != expected_checks:
        raise QueueError(
            f'{task_id} focused checks do not match profile {profile}')

    check_ids = [item['id'] for item in task['focused_checks']]
    if len(check_ids) != len(set(check_ids)):
        raise QueueError(f'{task_id} focused check ids contain duplicates')
    for command in expected_checks:
        if any('\x00' in argument or '\n' in argument for argument in command):
            raise QueueError(f'{task_id} focused check contains unsafe text')

    allowed = set(task['allowed_paths'])
    for probe in task['gap_probes']:
        _validate_repo_path(probe['path'])
        if probe['path'] not in allowed:
            raise QueueError(
                f'{task_id} gap probe is outside its path scope: '
                f"{probe['path']}")


def _task_readiness(task: dict[str, Any], repo_root: Path) -> dict[str, Any]:
    failures = []
    contents: dict[str, str] = {}
    for probe in task['gap_probes']:
        path = probe['path']
        if path not in contents:
            contents[path] = (repo_root / path).read_text(encoding='utf-8')
        present = probe['value'] in contents[path]
        passed = present if probe['mode'] == 'contains' else not present
        if not passed:
            failures.append(probe['failure_detail'])
    return {
        'id': task['id'],
        'status': 'READY' if not failures else 'STALE',
        'reasons': failures,
    }


def validate_queue(
    queue: dict[str, Any],
    schema: dict[str, Any],
    repo_root: Path = REPO_ROOT,
) -> dict[str, Any]:
    """Validate schema, scope, authority, duplicate audit, and local drift."""
    validator = jsonschema.Draft7Validator(schema)
    errors = sorted(
        validator.iter_errors(queue),
        key=lambda item: [str(part) for part in item.absolute_path],
    )
    if errors:
        first = errors[0]
        raise QueueError(
            f'schema validation failed at {_schema_error_path(first)}: '
            f'{first.message}')
    if queue['schema_uri'] != SCHEMA_URI:
        raise QueueError('queue schema_uri is not the supported v1 URI')

    tasks = queue['tasks']
    task_ids = tuple(task['id'] for task in tasks)
    if task_ids != EXPECTED_TASK_IDS:
        raise QueueError(
            'task ids must be the ordered bounded set '
            f'{list(EXPECTED_TASK_IDS)}')
    for task in tasks:
        _validate_task(task, repo_root)

    audit = queue['remote_duplicate_audit']
    if audit['open_pull_request_count'] != len(
        audit['open_pull_request_numbers']
    ):
        raise QueueError(
            'open pull request count does not match its inventory')
    audited_ids = tuple(item['task_id'] for item in audit['task_matches'])
    if audited_ids != EXPECTED_TASK_IDS:
        raise QueueError('duplicate audit must cover the ordered task set')
    duplicate_matches = {
        item['task_id']: item['matching_open_pull_requests']
        for item in audit['task_matches']
        if item['matching_open_pull_requests']
    }
    if duplicate_matches:
        raise QueueError(
            'open pull request duplicates require review: '
            f'{duplicate_matches}')

    authority = queue['authority']
    if any((
        authority['github_writes_authorized'],
        authority['issues_published'],
        authority['remote_mutations_performed'],
    )):
        raise QueueError(
            'a local starter queue cannot authorize GitHub writes')

    readiness = [_task_readiness(task, repo_root) for task in tasks]
    stale = [item for item in readiness if item['status'] == 'STALE']
    return {
        'status': (
            'QUEUE_READY_LOCAL_ONLY'
            if not stale else 'QUEUE_STALE_LOCAL_ONLY'
        ),
        'queue_id': queue['queue_id'],
        'publication_status': queue['status'],
        'task_count': len(tasks),
        'ready_task_ids': [
            item['id'] for item in readiness if item['status'] == 'READY'
        ],
        'stale_tasks': stale,
        'remote_duplicate_audit': {
            'checked_at': audit['checked_at'],
            'open_pull_request_count': audit['open_pull_request_count'],
            'matching_task_pull_requests': 0,
            'recheck_before_publication': True,
        },
        'authority': {
            'github_writes_authorized': False,
            'issues_published': False,
            'remote_mutations_performed': False,
        },
    }


def evaluate(
    queue_path: Path = DEFAULT_QUEUE,
    schema_path: Path = DEFAULT_SCHEMA,
    repo_root: Path = REPO_ROOT,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Load and validate the queue, returning the source and report."""
    queue = _load_json(queue_path, 'queue')
    schema = _load_json(schema_path, 'schema')
    return queue, validate_queue(queue, schema, repo_root)


def _find_task(queue: dict[str, Any], task_id: str) -> dict[str, Any]:
    for task in queue['tasks']:
        if task['id'] == task_id:
            return task
    raise QueueError(f'unknown starter task: {task_id}')


def _require_render_ready(
    report: dict[str, Any],
    task_id: str,
) -> None:
    stale = {
        item['id']: item['reasons'] for item in report['stale_tasks']
    }
    if task_id in stale:
        raise QueueError(
            f'{task_id} is stale and must be reviewed before rendering: '
            f"{'; '.join(stale[task_id])}")


def _bullet_lines(items: Sequence[str]) -> list[str]:
    return [f'- {item}' for item in items]


def render_task_markdown(task: dict[str, Any]) -> str:
    """Render one copy-ready issue body without publishing it."""
    issue_links = ', '.join(
        f'#{number}' for number in task['source_issue_numbers'])
    lines = [
        f"# {task['title']}",
        '',
        '> Coordination gate: this task is prepared locally but is not a '
        'published GitHub issue. Start only after a maintainer publishes the '
        'issue or confirms that it is still unclaimed.',
        '',
        '## Outcome',
        '',
        task['outcome'],
        '',
        f"Estimate in a prepared environment: **{task['estimate_minutes']} "
        'minutes**.',
        '',
        f'Related support issues: {issue_links}.',
        '',
        '## Allowed files',
        '',
        *_bullet_lines([f'`{path}`' for path in task['allowed_paths']]),
        '',
        '## Acceptance',
        '',
        *_bullet_lines(task['acceptance']),
        '',
        '## Non-goals',
        '',
        *_bullet_lines(task['non_goals']),
        '',
        '## Focused checks',
        '',
        '```bash',
        *(shlex.join(check['argv']) for check in task['focused_checks']),
        '```',
        '',
        'No private data or hardware is required.',
        '',
        'Labels: ' + ', '.join(f'`{label}`' for label in task['labels']),
    ]
    return '\n'.join(lines) + '\n'


def _verification_commands(
    task: dict[str, Any],
    temporary_root: Path,
) -> list[tuple[str, list[str]]]:
    profile = task['check_profile']
    if profile == 'docs-strict':
        return [(
            task['focused_checks'][0]['id'],
            [
                sys.executable,
                '-m',
                'mkdocs',
                'build',
                '--strict',
                '--site-dir',
                str(temporary_root / 'site'),
            ],
        )]
    if profile == 'mid360-empty-frame':
        return [
            (item['id'], list(command))
            for item, command in zip(
                task['focused_checks'], MID360_EMPTY_FRAME_CHECKS)
        ]
    raise QueueError(f'unsupported verification profile: {profile}')


def verify_task(
    task: dict[str, Any],
    repo_root: Path = REPO_ROOT,
) -> dict[str, Any]:
    """Run only the built-in command profile selected by a validated task."""
    environment = os.environ.copy()
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    environment['PYTEST_ADDOPTS'] = '-p no:cacheprovider'
    checks = []
    with tempfile.TemporaryDirectory(prefix='lidarslam-starter-') as temp_dir:
        commands = _verification_commands(task, Path(temp_dir))
        for (check_id, command), declared in zip(
            commands, task['focused_checks']
        ):
            try:
                result = subprocess.run(
                    command,
                    cwd=repo_root,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=300,
                    env=environment,
                )
                returncode = result.returncode
            except (OSError, subprocess.TimeoutExpired):
                returncode = 124
            checks.append({
                'id': check_id,
                'argv': declared['argv'],
                'status': 'PASS' if returncode == 0 else 'FAIL',
                'returncode': returncode,
            })
            if returncode != 0:
                break
    return {
        'status': (
            'FOCUSED_CHECKS_PASSED'
            if all(item['status'] == 'PASS' for item in checks)
            else 'FOCUSED_CHECKS_FAILED'
        ),
        'task_id': task['id'],
        'check_profile': task['check_profile'],
        'checks': checks,
        'acceptance_review_still_required': True,
        'workspace_artifacts_written': False,
        'remote_mutations_performed': False,
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--queue', type=Path, default=DEFAULT_QUEUE)
    parser.add_argument('--schema', type=Path, default=DEFAULT_SCHEMA)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--list', action='store_true')
    action.add_argument('--task', choices=EXPECTED_TASK_IDS)
    action.add_argument('--verify', choices=EXPECTED_TASK_IDS)
    parser.add_argument('--json', action='store_true')
    return parser.parse_args(argv)


def _print_list(
    queue: dict[str, Any],
    report: dict[str, Any],
) -> None:
    stale_ids = {item['id'] for item in report['stale_tasks']}
    print('Local starter queue — PREPARED_NOT_PUBLISHED')
    print('Wait for a published issue or explicit maintainer confirmation.')
    for task in queue['tasks']:
        readiness = 'STALE' if task['id'] in stale_ids else 'READY'
        print(
            f"{task['id']} [{readiness}] {task['estimate_minutes']} min — "
            f"{task['title']}")


def main(argv: Sequence[str] | None = None) -> int:
    """Run the selected read-only inspection or fixed verification action."""
    args = parse_args(argv)
    try:
        queue, report = evaluate(args.queue, args.schema)
        if args.verify:
            task = _find_task(queue, args.verify)
            verification = verify_task(task)
            if args.json:
                print(json.dumps(verification, indent=2, sort_keys=True))
            else:
                for check in verification['checks']:
                    print(f"{check['status']}: {shlex.join(check['argv'])}")
                print(verification['status'])
            return (
                0
                if verification['status'] == 'FOCUSED_CHECKS_PASSED'
                else 1
            )
        if args.task:
            task = _find_task(queue, args.task)
            _require_render_ready(report, args.task)
            if args.json:
                print(json.dumps({
                    'queue_status': report['status'],
                    'task': task,
                    'authority': report['authority'],
                }, indent=2, sort_keys=True))
            else:
                print(render_task_markdown(task), end='')
            return 0
        if args.list:
            if args.json:
                print(json.dumps({
                    'queue_status': report['status'],
                    'publication_status': report['publication_status'],
                    'tasks': [
                        {
                            'id': task['id'],
                            'title': task['title'],
                            'estimate_minutes': task['estimate_minutes'],
                        }
                        for task in queue['tasks']
                    ],
                    'authority': report['authority'],
                }, indent=2, sort_keys=True))
            else:
                _print_list(queue, report)
            return 0
        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            print(
                f"PASS: {report['task_count']} local starter tasks are "
                'bounded, unclaimed, and ready for maintainer review')
            print('No GitHub issue was created or changed.')
        return 0 if report['status'] == 'QUEUE_READY_LOCAL_ONLY' else 1
    except QueueError as exc:
        if args.json:
            print(json.dumps({
                'status': 'QUEUE_INVALID',
                'error': str(exc),
                'remote_mutations_performed': False,
            }, sort_keys=True))
        else:
            print(f'ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
