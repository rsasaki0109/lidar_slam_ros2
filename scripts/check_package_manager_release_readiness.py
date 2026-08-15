#!/usr/bin/env python3
"""Audit the public Humble/Jazzy main-channel package-manager release gate."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import sys
from typing import Any
import urllib.error
import urllib.parse
import urllib.request

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
WORKFLOW_PATH = '.github/workflows/package-manager-install-upgrade.yml'
WORKFLOW_NAME = 'package-manager-install-upgrade.yml'
DISTROS = ('humble', 'jazzy')
SCHEMA_PATH = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'package-manager-release-readiness-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/package-manager-release-readiness-v1.schema.json'
)
SEMVER = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+$')
SHA = re.compile(r'^[a-f0-9]{40}$')
MAX_RESPONSE_BYTES = 2 * 1024 * 1024


class PackageManagerReleaseError(ValueError):
    """The public package-manager release state could not be trusted."""


def expected_run_name(version: str) -> str:
    """Return the immutable identity encoded by the workflow run name."""
    if SEMVER.fullmatch(version) is None:
        raise PackageManagerReleaseError(
            f'expected MAJOR.MINOR.PATCH version, found {version!r}')
    return (
        f'package-manager / v{version} / {version} / main / clean-install'
    )


def _request_json(url: str) -> dict[str, Any]:
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'lidarslam-package-manager-release-audit/1',
        'X-GitHub-Api-Version': '2022-11-28',
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
            length_text = response.headers.get('Content-Length')
            if length_text is not None and int(length_text) > MAX_RESPONSE_BYTES:
                raise PackageManagerReleaseError(
                    f'remote response is too large: {length_text} bytes')
            payload = response.read(MAX_RESPONSE_BYTES + 1)
            if len(payload) > MAX_RESPONSE_BYTES:
                raise PackageManagerReleaseError(
                    'remote response exceeds the download limit')
    except (
        OSError,
        ValueError,
        urllib.error.HTTPError,
        urllib.error.URLError,
    ) as exc:
        raise PackageManagerReleaseError(
            f'GitHub API request failed for {url}: {exc}') from exc
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PackageManagerReleaseError(
            f'GitHub API returned invalid JSON for {url}: {exc}') from exc
    if not isinstance(value, dict):
        raise PackageManagerReleaseError(
            f'GitHub API root is not an object for {url}')
    return value


def inspect_remote(version: str) -> dict[str, Any]:
    """Inspect successful workflow dispatches without mutating GitHub."""
    run_name = expected_run_name(version)
    query = urllib.parse.urlencode({
        'event': 'workflow_dispatch',
        'status': 'success',
        'per_page': 100,
    })
    runs_url = (
        f'https://api.github.com/repos/{REPOSITORY}/actions/workflows/'
        f'{WORKFLOW_NAME}/runs?{query}'
    )
    try:
        payload = _request_json(runs_url)
        runs = payload.get('workflow_runs')
        if not isinstance(runs, list):
            raise PackageManagerReleaseError(
                'GitHub workflow-runs response has no workflow_runs array')
        candidates = [
            run for run in runs
            if (
                isinstance(run, dict)
                and run.get('display_title') == run_name
            )
        ]
        inspected: list[dict[str, Any]] = []
        for run in candidates:
            jobs_url = run.get('jobs_url')
            if not isinstance(jobs_url, str):
                raise PackageManagerReleaseError(
                    'matching workflow run has no jobs_url')
            jobs_payload = _request_json(jobs_url)
            jobs = jobs_payload.get('jobs')
            if not isinstance(jobs, list):
                raise PackageManagerReleaseError(
                    'GitHub jobs response has no jobs array')
            inspected.append({
                'id': run.get('id'),
                'html_url': run.get('html_url'),
                'display_title': run.get('display_title'),
                'event': run.get('event'),
                'status': run.get('status'),
                'conclusion': run.get('conclusion'),
                'workflow_path': run.get('path'),
                'head_sha': run.get('head_sha'),
                'jobs': [
                    {
                        'name': job.get('name'),
                        'status': job.get('status'),
                        'conclusion': job.get('conclusion'),
                    }
                    for job in jobs
                    if isinstance(job, dict)
                ],
            })
        return {
            'inspected': True,
            'errors': [],
            'runs': inspected,
        }
    except PackageManagerReleaseError as exc:
        return {
            'inspected': False,
            'errors': [str(exc)],
            'runs': [],
        }


def _check(check_id: str, passed: bool, detail: str) -> dict[str, str]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'detail': detail,
    }


def evaluate_readiness(
    *,
    version: str,
    snapshot: dict[str, Any],
) -> dict[str, Any]:
    """Evaluate an injected or live workflow snapshot fail-closed."""
    run_name = expected_run_name(version)
    errors = snapshot.get('errors', [])
    runs = snapshot.get('runs', [])
    if not isinstance(errors, list) or not all(
        isinstance(error, str) and error for error in errors
    ):
        raise PackageManagerReleaseError(
            'snapshot errors must be an array of non-empty strings')
    if not isinstance(runs, list) or not all(
        isinstance(run, dict) for run in runs
    ):
        raise PackageManagerReleaseError(
            'snapshot runs must be an array of objects')

    selected: dict[str, Any] | None = None
    checks: list[dict[str, str]] = []
    for run in runs:
        jobs = run.get('jobs', [])
        if not isinstance(jobs, list):
            continue
        job_state = {
            job.get('name'): (
                job.get('status'),
                job.get('conclusion'),
            )
            for job in jobs
            if isinstance(job, dict)
        }
        candidate_checks = [
            _check(
                'immutable-run-name',
                run.get('display_title') == run_name,
                f"expected {run_name!r}; found {run.get('display_title')!r}",
            ),
            _check(
                'workflow-dispatch',
                run.get('event') == 'workflow_dispatch',
                f"event={run.get('event')!r}",
            ),
            _check(
                'trusted-workflow-path',
                run.get('workflow_path') == WORKFLOW_PATH,
                f"workflow_path={run.get('workflow_path')!r}",
            ),
            _check(
                'completed-successfully',
                (
                    run.get('status') == 'completed'
                    and run.get('conclusion') == 'success'
                ),
                (
                    f"status={run.get('status')!r}, "
                    f"conclusion={run.get('conclusion')!r}"
                ),
            ),
            _check(
                'workflow-head-identity',
                (
                    isinstance(run.get('head_sha'), str)
                    and SHA.fullmatch(run['head_sha']) is not None
                ),
                f"head_sha={run.get('head_sha')!r}",
            ),
        ]
        for distro in DISTROS:
            name = f'{distro} clean-install'
            state = job_state.get(name)
            candidate_checks.append(_check(
                f'{distro}-main-clean-install',
                state == ('completed', 'success'),
                f'job {name!r}: {state!r}',
            ))
        if all(check['status'] == 'PASS' for check in candidate_checks):
            selected = run
            checks = candidate_checks
            break
        if not checks:
            checks = candidate_checks

    if errors:
        status = 'BLOCKED'
        actions = [
            'Restore trusted read-only access to the public GitHub Actions API.'
        ]
    elif selected is None:
        status = 'NOT_RUN'
        actions = [
            (
                f'Run {WORKFLOW_NAME} from source_ref=v{version} with '
                f'target_version={version}, target_channel=main, and '
                'mode=clean-install; require both matrix jobs to pass.'
            )
        ]
    else:
        status = 'READY'
        actions = []

    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': status,
        'candidate': {
            'product_version': version,
            'source_ref': f'v{version}',
            'apt_channel': 'main',
            'mode': 'clean-install',
            'run_name': run_name,
        },
        'remote': {
            'inspected': snapshot.get('inspected') is True,
            'errors': errors,
            'matching_runs': len(runs),
        },
        'selected_run': selected,
        'checks': checks,
        'actions': actions,
    }
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.Draft7Validator(schema).validate(report)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version', required=True)
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--require-ready', action='store_true')
    args = parser.parse_args(argv)
    try:
        report = evaluate_readiness(
            version=args.version,
            snapshot=inspect_remote(args.version),
        )
    except (
        OSError,
        PackageManagerReleaseError,
        json.JSONDecodeError,
        jsonschema.SchemaError,
        jsonschema.ValidationError,
    ) as exc:
        print(
            f'package-manager release readiness error: {exc}',
            file=sys.stderr,
        )
        return 2
    rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered, encoding='utf-8')
    if args.json:
        print(rendered, end='')
    else:
        print(
            f"Package-manager release gate: {report['status']}\n"
            f"Expected run: {report['candidate']['run_name']}"
        )
        for action in report['actions']:
            print(f'  - {action}')
    if args.require_ready and report['status'] != 'READY':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
