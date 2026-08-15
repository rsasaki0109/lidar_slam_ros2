#!/usr/bin/env python3
"""Read-only, fail-closed preflight for the first ndt_omp_ros2 release."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
SUBMODULE = Path('Thirdparty/ndt_omp_ros2')
EXPECTED_COMMIT = '8b77fa5a6cdcad45bf35918361c892b6d94a287e'
EXPECTED_VERSION = '0.1.0'
SCHEMA_PATH = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'ndt-omp-release-readiness-v2.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/ndt-omp-release-readiness-v2.schema.json'
)
SOURCE_REPOSITORY = 'rsasaki0109/ndt_omp_ros2'
RELEASE_REPOSITORY = 'rsasaki0109/ndt_omp_ros2-release'
ROSDISTRO_REPOSITORY = 'ros/rosdistro'
DISTROS = ('humble', 'jazzy')
ROSDISTRO_PULL_REQUESTS = {
    'humble': 52949,
    'jazzy': 52950,
}
ACTIONABLE_REVIEW = re.compile(
    r'(?:\?|\b(?:please|could you|would you|how does|how do|why|what is|'
    r'what are|same question|needs? to|must)\b)',
    re.IGNORECASE,
)


class PreflightError(ValueError):
    """The release candidate or remote inspection could not be trusted."""


def _run(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ['git', *args],
        cwd=repo,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise PreflightError(
            f"git {' '.join(args)} failed in {repo}: "
            f'{result.stderr.strip() or result.stdout.strip()}'
        )
    return result.stdout.strip()


def _check(check_id: str, passed: bool, detail: str) -> dict[str, str]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'detail': detail,
    }


def inspect_local(repo_root: Path = REPO_ROOT) -> dict[str, Any]:
    """Inspect the pinned submodule without changing either repository."""
    repo_root = repo_root.resolve()
    submodule = repo_root / SUBMODULE
    checks: list[dict[str, str]] = []

    try:
        stage = _run(repo_root, 'ls-files', '--stage', '--', SUBMODULE.as_posix())
        fields = stage.split()
        gitlink = fields[1] if len(fields) >= 2 and fields[0] == '160000' else ''
        checks.append(_check(
            'parent-gitlink',
            gitlink == EXPECTED_COMMIT,
            f'expected {EXPECTED_COMMIT}; found {gitlink or "no gitlink"}',
        ))
    except PreflightError as exc:
        gitlink = ''
        checks.append(_check('parent-gitlink', False, str(exc)))

    if not submodule.is_dir():
        checks.append(_check(
            'submodule-present',
            False,
            f'missing directory {SUBMODULE.as_posix()}',
        ))
        return {
            'ready': False,
            'gitlink_commit': gitlink,
            'head_commit': '',
            'package_version': '',
            'checks': checks,
        }

    checks.append(_check(
        'submodule-present',
        True,
        f'{SUBMODULE.as_posix()} is initialized',
    ))
    try:
        head = _run(submodule, 'rev-parse', 'HEAD')
        checks.append(_check(
            'candidate-commit',
            head == EXPECTED_COMMIT,
            f'expected {EXPECTED_COMMIT}; found {head}',
        ))
        dirty = _run(submodule, 'status', '--porcelain')
        checks.append(_check(
            'clean-worktree',
            not dirty,
            'submodule worktree is clean' if not dirty else
            'submodule has uncommitted changes',
        ))
    except PreflightError as exc:
        head = ''
        checks.append(_check('submodule-git-state', False, str(exc)))

    package_name = ''
    package_version = ''
    package_license = ''
    try:
        package = ET.parse(submodule / 'package.xml').getroot()
        package_name = (package.findtext('name') or '').strip()
        package_version = (package.findtext('version') or '').strip()
        licenses = [
            (node.text or '').strip() for node in package.findall('license')
        ]
        package_license = ','.join(licenses)
        checks.extend([
            _check(
                'package-name',
                package_name == 'ndt_omp_ros2',
                f'expected ndt_omp_ros2; found {package_name or "empty"}',
            ),
            _check(
                'package-version',
                package_version == EXPECTED_VERSION,
                f'expected {EXPECTED_VERSION}; '
                f'found {package_version or "empty"}',
            ),
            _check(
                'package-license',
                licenses == ['BSD-2-Clause'],
                f'expected BSD-2-Clause; found {package_license or "empty"}',
            ),
        ])
    except (OSError, ET.ParseError) as exc:
        checks.append(_check('package-metadata', False, str(exc)))

    required_text = {
        'changelog-entry': (
            'CHANGELOG.rst',
            f'{EXPECTED_VERSION} (',
        ),
        'cmake-library-install': (
            'CMakeLists.txt',
            '  TARGETS\n    ndt_omp\n  EXPORT export_ndt_omp',
        ),
        'cmake-target-export': (
            'CMakeLists.txt',
            'ament_export_targets(export_ndt_omp HAS_LIBRARY_TARGET)',
        ),
        'bloom-preflight': (
            'scripts/check_bloom_release.py',
            "'bloom-generate'",
        ),
        'bloom-schema': (
            'schemas/bloom-release-v1.schema.json',
            '"$schema"',
        ),
        'release-ci': (
            '.github/workflows/ci.yml',
            'check_bloom_release.py',
        ),
    }
    for check_id, (relative, marker) in required_text.items():
        path = submodule / relative
        try:
            content = path.read_text(encoding='utf-8')
            checks.append(_check(
                check_id,
                marker in content,
                f'{relative} contains required marker' if marker in content
                else f'{relative} lacks required marker {marker!r}',
            ))
        except OSError as exc:
            checks.append(_check(check_id, False, f'{relative}: {exc}'))

    return {
        'ready': all(item['status'] == 'PASS' for item in checks),
        'gitlink_commit': gitlink,
        'head_commit': head,
        'package_version': package_version,
        'checks': checks,
    }


def _request_text(url: str) -> tuple[int, str]:
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'lidarslam-ndt-release-preflight/1',
    }
    token = os.environ.get('GITHUB_TOKEN')
    if token and url.startswith('https://api.github.com/'):
        headers['Authorization'] = f'Bearer {token}'
    request = urllib.request.Request(
        url,
        headers=headers,
    )
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            return response.status, response.read().decode('utf-8')
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            return 404, ''
        raise PreflightError(f'HTTP {exc.code} while reading {url}') from exc
    except (urllib.error.URLError, TimeoutError, UnicodeDecodeError) as exc:
        raise PreflightError(f'cannot read {url}: {exc}') from exc


def _empty_pull_requests() -> dict[str, dict[str, Any]]:
    return {
        distro: {
            'number': ROSDISTRO_PULL_REQUESTS[distro],
            'url': (
                'https://github.com/ros/rosdistro/pull/'
                f'{ROSDISTRO_PULL_REQUESTS[distro]}'
            ),
            'state': None,
            'merged': None,
            'mergeable': None,
            'head_sha': None,
            'updated_at': None,
            'latest_actionable_review': {
                'url': None,
                'author': None,
                'created_at': None,
            },
            'response_pending': None,
        }
        for distro in DISTROS
    }


def _request_json(url: str) -> Any:
    status, body = _request_text(url)
    if status != 200:
        raise PreflightError(f'unexpected HTTP {status} while reading {url}')
    try:
        return json.loads(body)
    except json.JSONDecodeError as exc:
        raise PreflightError(f'invalid JSON while reading {url}: {exc}') from exc


def _activity_user(item: dict[str, Any]) -> tuple[str, bool]:
    user = item.get('user')
    if not isinstance(user, dict):
        return '', False
    login = user.get('login')
    user_type = user.get('type')
    if not isinstance(login, str):
        return '', False
    is_bot = user_type == 'Bot' or login.endswith('[bot]')
    return login, is_bot


def _inspect_pull_request(distro: str) -> dict[str, Any]:
    number = ROSDISTRO_PULL_REQUESTS[distro]
    api_root = f'https://api.github.com/repos/{ROSDISTRO_REPOSITORY}'
    payload = _request_json(f'{api_root}/pulls/{number}')
    if not isinstance(payload, dict):
        raise PreflightError(f'pull request #{number} response is not an object')
    author, _ = _activity_user(payload)
    if not author:
        raise PreflightError(f'pull request #{number} has no author identity')

    activity_specs = (
        (f'{api_root}/pulls/{number}/reviews?per_page=100', 'submitted_at'),
        (f'{api_root}/issues/{number}/comments?per_page=100', 'created_at'),
        (f'{api_root}/pulls/{number}/comments?per_page=100', 'created_at'),
    )
    actionable: list[dict[str, str]] = []
    author_responses: list[str] = []
    for url, timestamp_field in activity_specs:
        items = _request_json(url)
        if not isinstance(items, list):
            raise PreflightError(
                f'pull request #{number} activity response is not a list')
        for item in items:
            if not isinstance(item, dict):
                continue
            body = item.get('body')
            timestamp = item.get(timestamp_field)
            login, is_bot = _activity_user(item)
            if (
                not isinstance(body, str)
                or not body.strip()
                or not isinstance(timestamp, str)
                or not login
            ):
                continue
            if login == author:
                author_responses.append(timestamp)
                continue
            review_state = item.get('state')
            is_actionable = (
                review_state == 'CHANGES_REQUESTED'
                or ACTIONABLE_REVIEW.search(body) is not None
            )
            if is_bot or not is_actionable:
                continue
            html_url = item.get('html_url')
            if not isinstance(html_url, str) or not html_url.startswith(
                    'https://github.com/'):
                continue
            actionable.append({
                'url': html_url,
                'author': login,
                'created_at': timestamp,
            })

    latest = max(actionable, key=lambda item: item['created_at']) \
        if actionable else None
    response_pending = bool(
        latest is not None
        and not any(
            timestamp > latest['created_at']
            for timestamp in author_responses
        )
        and payload.get('merged') is not True
    )
    state = payload.get('state')
    merged = payload.get('merged')
    mergeable = payload.get('mergeable')
    head = payload.get('head')
    head_sha = head.get('sha') if isinstance(head, dict) else None
    updated_at = payload.get('updated_at')
    html_url = payload.get('html_url')
    if state not in {'open', 'closed'} or not isinstance(merged, bool):
        raise PreflightError(f'pull request #{number} has invalid state fields')
    if mergeable is not None and not isinstance(mergeable, bool):
        raise PreflightError(f'pull request #{number} has invalid mergeable field')
    if not isinstance(head_sha, str) or not isinstance(updated_at, str):
        raise PreflightError(f'pull request #{number} has incomplete identity')
    if not isinstance(html_url, str):
        raise PreflightError(f'pull request #{number} has no public URL')
    return {
        'number': number,
        'url': html_url,
        'state': state,
        'merged': merged,
        'mergeable': mergeable,
        'head_sha': head_sha,
        'updated_at': updated_at,
        'latest_actionable_review': latest or {
            'url': None,
            'author': None,
            'created_at': None,
        },
        'response_pending': response_pending,
    }


def inspect_remote() -> dict[str, Any]:
    """Read public GitHub and rosdistro state without authenticating or writing."""
    errors: list[str] = []
    origin_commit: str | None = None
    source_tag_present: bool | None = None
    release_repository_present: bool | None = None
    rosdistro: dict[str, bool | None] = {distro: None for distro in DISTROS}
    pull_requests = _empty_pull_requests()

    try:
        status, body = _request_text(
            f'https://api.github.com/repos/{SOURCE_REPOSITORY}/'
            'git/ref/heads/humble'
        )
        if status != 200:
            raise PreflightError('source branch humble unexpectedly returned 404')
        payload = json.loads(body)
        origin_commit = payload['object']['sha']
    except (PreflightError, json.JSONDecodeError, KeyError, TypeError) as exc:
        errors.append(f'origin branch: {exc}')

    try:
        status, _ = _request_text(
            f'https://api.github.com/repos/{SOURCE_REPOSITORY}/'
            f'git/ref/tags/{EXPECTED_VERSION}'
        )
        source_tag_present = status == 200
    except PreflightError as exc:
        errors.append(f'source tag: {exc}')

    try:
        status, _ = _request_text(
            f'https://api.github.com/repos/{RELEASE_REPOSITORY}'
        )
        release_repository_present = status == 200
    except PreflightError as exc:
        errors.append(f'release repository: {exc}')

    for distro in DISTROS:
        try:
            status, body = _request_text(
                'https://raw.githubusercontent.com/ros/rosdistro/'
                f'master/{distro}/distribution.yaml'
            )
            if status != 200:
                raise PreflightError(
                    f'{distro} distribution unexpectedly returned 404')
            rosdistro[distro] = any(
                line == '  ndt_omp_ros2:'
                for line in body.splitlines()
            )
        except PreflightError as exc:
            errors.append(f'rosdistro {distro}: {exc}')

    if source_tag_present is True and release_repository_present is True:
        for distro in DISTROS:
            try:
                pull_requests[distro] = _inspect_pull_request(distro)
            except PreflightError as exc:
                errors.append(f'rosdistro PR {distro}: {exc}')

    return {
        'errors': errors,
        'origin_branch_commit': origin_commit,
        'source_tag_present': source_tag_present,
        'release_repository_present': release_repository_present,
        'rosdistro': rosdistro,
        'pull_requests': pull_requests,
    }


def _unanswered_review_action(
    distro: str,
    pull_request: dict[str, Any],
) -> str:
    review = pull_request['latest_actionable_review']
    return (
        f'Respond to the unanswered human review for ros/rosdistro PR '
        f'#{pull_request["number"]} ({distro}) at {review["url"]}. Explain '
        'the upstream lineage, required API delta, and collision-free '
        'convergence plan; do not describe this as wait-only or rerun Bloom.'
    )


def evaluate_readiness(
    *,
    repo_root: Path = REPO_ROOT,
    offline: bool = False,
    local: dict[str, Any] | None = None,
    remote: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a deterministic report from local and optional remote state."""
    local = inspect_local(repo_root) if local is None else local
    actions: list[str] = []

    if not local['ready']:
        status = 'BLOCKED'
        actions.append('Repair every failed local candidate check before tagging.')
        remote_report = {
            'inspected': False,
            'errors': [],
            'origin_branch_commit': None,
            'source_tag_present': None,
            'release_repository_present': None,
            'rosdistro': {distro: None for distro in DISTROS},
            'pull_requests': _empty_pull_requests(),
        }
    elif offline:
        status = 'LOCAL_READY'
        actions.append(
            'Run again without --offline before creating the source tag.')
        remote_report = {
            'inspected': False,
            'errors': [],
            'origin_branch_commit': None,
            'source_tag_present': None,
            'release_repository_present': None,
            'rosdistro': {distro: None for distro in DISTROS},
            'pull_requests': _empty_pull_requests(),
        }
    else:
        remote_report = inspect_remote() if remote is None else dict(remote)
        remote_report.setdefault('pull_requests', _empty_pull_requests())
        remote_report['inspected'] = True
        artifacts = [
            remote_report.get('source_tag_present'),
            remote_report.get('release_repository_present'),
            remote_report.get('rosdistro', {}).get('humble'),
            remote_report.get('rosdistro', {}).get('jazzy'),
        ]
        remote_failed = (
            bool(remote_report.get('errors'))
            or remote_report.get('origin_branch_commit') != EXPECTED_COMMIT
            or any(value is None for value in artifacts)
        )
        generated_prs_required = (
            remote_report.get('source_tag_present') is True
            and remote_report.get('release_repository_present') is True
            and not all(remote_report.get('rosdistro', {}).values())
        )
        if generated_prs_required:
            for distro in DISTROS:
                if remote_report['rosdistro'][distro]:
                    continue
                pull_request = remote_report['pull_requests'].get(distro, {})
                if (
                    pull_request.get('state') is None
                    or pull_request.get('merged') is None
                    or pull_request.get('response_pending') is None
                ):
                    remote_failed = True
        closed_unmerged = any(
            not remote_report['rosdistro'][distro]
            and remote_report['pull_requests'][distro]['state'] == 'closed'
            and remote_report['pull_requests'][distro]['merged'] is False
            for distro in DISTROS
        ) if not remote_failed else False
        unmergeable = any(
            not remote_report['rosdistro'][distro]
            and remote_report['pull_requests'][distro]['state'] == 'open'
            and remote_report['pull_requests'][distro]['mergeable'] is False
            for distro in DISTROS
        ) if not remote_failed else False
        mergeability_pending = any(
            not remote_report['rosdistro'][distro]
            and remote_report['pull_requests'][distro]['state'] == 'open'
            and remote_report['pull_requests'][distro]['mergeable'] is None
            for distro in DISTROS
        ) if not remote_failed else False
        review_pending = any(
            not remote_report['rosdistro'][distro]
            and remote_report['pull_requests'][distro]['response_pending']
            is True
            for distro in DISTROS
        ) if not remote_failed else False
        if remote_failed:
            status = 'BLOCKED'
            actions.append(
                'Resolve remote inspection failures or candidate drift; '
                'do not publish from this report.')
        elif all(value is False for value in artifacts):
            status = 'READY_TO_TAG'
            actions.append(
                'Create and push source tag 0.1.0, then follow the Bloom runbook.')
        elif all(value is True for value in artifacts):
            status = 'RELEASED'
            actions.append(
                'Proceed with the lidarslam_ros2 distribution gate.')
        elif closed_unmerged:
            status = 'BLOCKED'
            for distro in DISTROS:
                pull_request = remote_report['pull_requests'][distro]
                if (
                    not remote_report['rosdistro'][distro]
                    and pull_request['state'] == 'closed'
                    and pull_request['merged'] is False
                ):
                    actions.append(
                        f'ros/rosdistro PR #{pull_request["number"]} '
                        f'({distro}) closed without merge. Resolve the '
                        'review outcome before creating replacement release '
                        'state.')
        elif unmergeable:
            status = 'BLOCKED'
            for distro in DISTROS:
                pull_request = remote_report['pull_requests'][distro]
                if (
                    not remote_report['rosdistro'][distro]
                    and pull_request['state'] == 'open'
                    and pull_request['mergeable'] is False
                ):
                    actions.append(
                        f'ros/rosdistro PR #{pull_request["number"]} '
                        f'({distro}) is not mergeable at exact head '
                        f'{pull_request["head_sha"]}. Resolve the base '
                        'conflict and rerun this audit; do not merge or '
                        'rerun Bloom from this state.')
            for distro in DISTROS:
                pull_request = remote_report['pull_requests'][distro]
                if (
                    not remote_report['rosdistro'][distro]
                    and pull_request['response_pending'] is True
                ):
                    actions.append(_unanswered_review_action(
                        distro, pull_request))
        elif review_pending:
            status = 'REVIEW_REQUIRED'
            for distro in DISTROS:
                pull_request = remote_report['pull_requests'][distro]
                if (
                    not remote_report['rosdistro'][distro]
                    and pull_request['response_pending'] is True
                ):
                    actions.append(_unanswered_review_action(
                        distro, pull_request))
        elif mergeability_pending:
            status = 'BLOCKED'
            for distro in DISTROS:
                pull_request = remote_report['pull_requests'][distro]
                if (
                    not remote_report['rosdistro'][distro]
                    and pull_request['state'] == 'open'
                    and pull_request['mergeable'] is None
                ):
                    actions.append(
                        f'GitHub has not resolved mergeability for '
                        f'ros/rosdistro PR #{pull_request["number"]} '
                        f'({distro}) at exact head '
                        f'{pull_request["head_sha"]}. Rerun the read-only '
                        'audit before treating the PR as wait-only or '
                        'mergeable.')
        else:
            status = 'IN_PROGRESS'
            if not remote_report['source_tag_present']:
                actions.append('Create source tag 0.1.0.')
            if not remote_report['release_repository_present']:
                actions.append('Create and initialize ndt_omp_ros2-release.')
            for distro in DISTROS:
                if not remote_report['rosdistro'][distro]:
                    if (
                        remote_report['source_tag_present']
                        and remote_report['release_repository_present']
                    ):
                        pr_number = ROSDISTRO_PULL_REQUESTS[distro]
                        actions.append(
                            f'Wait for ros/rosdistro PR #{pr_number} '
                            f'({distro}) to merge; do not recreate the source '
                            'tag or rerun Bloom while this generated PR is '
                            'current.'
                        )
                    else:
                        actions.append(
                            f'After the source tag and release repository are '
                            f'ready, Bloom-release ndt_omp_ros2 for {distro} '
                            'and merge the generated rosdistro entry.'
                        )

    report = {
        'schema_version': 2,
        'schema_uri': SCHEMA_URI,
        'package': 'ndt_omp_ros2',
        'candidate': {
            'version': EXPECTED_VERSION,
            'commit': EXPECTED_COMMIT,
        },
        'mode': 'offline' if offline else 'online',
        'status': status,
        'local': local,
        'remote': remote_report,
        'actions': actions,
    }
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.Draft7Validator(schema).validate(report)
    return report


def _summary(report: dict[str, Any]) -> str:
    lines = [
        f"NDT release preflight: {report['status']}",
        f"Candidate: {report['candidate']['version']} "
        f"({report['candidate']['commit']})",
    ]
    for item in report['local']['checks']:
        lines.append(f"  [{item['status']}] {item['id']}: {item['detail']}")
    if report['remote']['inspected']:
        remote = report['remote']
        lines.extend([
            f"Remote branch: {remote['origin_branch_commit']}",
            f"Source tag present: {remote['source_tag_present']}",
            f'Release repository present: '
            f"{remote['release_repository_present']}",
            f"rosdistro: {remote['rosdistro']}",
        ])
        for distro in DISTROS:
            pull_request = remote['pull_requests'][distro]
            lines.append(
                f"rosdistro PR {distro}: #{pull_request['number']} "
                f"state={pull_request['state']} merged="
                f"{pull_request['merged']} mergeable="
                f"{pull_request['mergeable']} response_pending="
                f"{pull_request['response_pending']}")
        lines.extend(f'  [ERROR] {error}' for error in remote['errors'])
    lines.extend(f'Next: {action}' for action in report['actions'])
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--offline',
        action='store_true',
        help='validate only the pinned local release candidate',
    )
    parser.add_argument('--json', action='store_true', help='print JSON')
    parser.add_argument('--output-json', type=Path)
    strict = parser.add_mutually_exclusive_group()
    strict.add_argument('--require-ready-to-tag', action='store_true')
    strict.add_argument('--require-released', action='store_true')
    args = parser.parse_args(argv)
    try:
        report = evaluate_readiness(offline=args.offline)
        rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
        if args.output_json:
            args.output_json.write_text(rendered, encoding='utf-8')
        print(rendered if args.json else _summary(report))
    except (
        OSError,
        PreflightError,
        json.JSONDecodeError,
        jsonschema.SchemaError,
        jsonschema.ValidationError,
    ) as exc:
        print(f'ndt release preflight error: {exc}', file=sys.stderr)
        return 2

    if args.require_ready_to_tag and report['status'] != 'READY_TO_TAG':
        return 1
    if args.require_released and report['status'] != 'RELEASED':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
