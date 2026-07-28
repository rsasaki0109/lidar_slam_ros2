#!/usr/bin/env python3
"""Collect a privacy-bounded independent first-map validation report."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any
from urllib.parse import urlparse


REPO_ROOT = Path(__file__).resolve().parents[1]
FIRST_MAP_CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'first-map-v1.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/independent-first-map-validation-v1.schema.json'
)
REPORT_NAME = 'independent_first_map_validation.json'
MARKDOWN_NAME = 'independent_first_map_validation.md'
IDENTITY_PATTERNS = (
    re.compile(r'^[0-9a-f]{40}$'),
    re.compile(r'^.+@sha256:[0-9a-f]{64}$'),
    re.compile(r'^v[0-9]+\.[0-9]+\.[0-9]+(?:[-+][0-9A-Za-z.-]+)?$'),
)
VERIFY_COUNTS = re.compile(
    r'PASS:\s*(?P<pass>[0-9]+)\s*\|\s*'
    r'WARN:\s*(?P<warn>[0-9]+)\s*\|\s*'
    r'FAIL:\s*(?P<fail>[0-9]+)'
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding='utf-8'))


def _atomic_write(path: Path, text: str) -> None:
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_text(text, encoding='utf-8')
    temporary.replace(path)


def _validate_answers(value: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(value, dict):
        return ['answers root must be an object']
    if value.get('schema_version') != 1:
        errors.append('schema_version must be 1')
    if value.get('entrypoint_id') not in {
        'docker-demo',
        'own-bag',
        'source-demo',
    }:
        errors.append('entrypoint_id is not an official first-map entrypoint')
    for key in ('tested_identity', 'starting_document'):
        if not isinstance(value.get(key), str) or not value[key].strip():
            errors.append(f'{key} must be a non-empty string')

    environment = value.get('environment')
    if not isinstance(environment, dict):
        errors.append('environment must be an object')
    else:
        expected_keys = {
            'ros_distro',
            'os',
            'architecture',
            'cpu_label',
            'ram_gib',
            'docker_version',
        }
        if set(environment) != expected_keys:
            errors.append(
                f'environment fields must be {sorted(expected_keys)!r}'
            )
        if environment.get('ros_distro') not in {'humble', 'jazzy'}:
            errors.append('environment.ros_distro must be humble or jazzy')
        for key in ('os', 'architecture', 'cpu_label'):
            if (
                not isinstance(environment.get(key), str)
                or not environment[key].strip()
            ):
                errors.append(f'environment.{key} must be a non-empty string')
        ram_gib = environment.get('ram_gib')
        if (
            not isinstance(ram_gib, (int, float))
            or isinstance(ram_gib, bool)
            or ram_gib <= 0
        ):
            errors.append('environment.ram_gib must be greater than zero')
        docker_version = environment.get('docker_version')
        if docker_version is not None and not isinstance(docker_version, str):
            errors.append('environment.docker_version must be a string or null')

    commands = value.get('commands')
    if (
        not isinstance(commands, list)
        or not commands
        or not all(isinstance(item, str) and item.strip() for item in commands)
    ):
        errors.append('commands must be a non-empty string array')

    first_attempt = value.get('first_attempt')
    if not isinstance(first_attempt, dict):
        errors.append('first_attempt must be an object')
    else:
        if set(first_attempt) != {'result', 'elapsed', 'findings'}:
            errors.append(
                'first_attempt fields must be result, elapsed and findings'
            )
        if first_attempt.get('result') not in {
            'verified_map',
            'map_unverified',
            'workflow_incomplete',
        }:
            errors.append('first_attempt.result is invalid')
        if (
            not isinstance(first_attempt.get('elapsed'), str)
            or not first_attempt['elapsed'].strip()
        ):
            errors.append('first_attempt.elapsed must be a non-empty string')
        findings = first_attempt.get('findings')
        if (
            not isinstance(findings, list)
            or not all(
                isinstance(item, str) and item.strip() for item in findings
            )
        ):
            errors.append('first_attempt.findings must be a string array')

    attestations = value.get('attestations')
    expected_attestations = {
        'independent_tester',
        'docs_only_start',
        'first_attempt_preserved',
        'commands_redacted',
    }
    if not isinstance(attestations, dict):
        errors.append('attestations must be an object')
    elif set(attestations) != expected_attestations:
        errors.append(
            f'attestation fields must be {sorted(expected_attestations)!r}'
        )
    elif not all(
        isinstance(attestations[key], bool) for key in expected_attestations
    ):
        errors.append('all attestations must be boolean')
    if not isinstance(value.get('public_consent'), bool):
        errors.append('public_consent must be boolean')
    return errors


def _artifact_evidence(
    run_dir: Path | None,
    artifact_paths: list[str],
) -> list[dict[str, Any]]:
    evidence: list[dict[str, Any]] = []
    for relative in artifact_paths:
        is_directory = relative.endswith('/')
        clean_relative = relative.rstrip('/')
        path = run_dir / clean_relative if run_dir else None
        present = bool(
            path
            and (
                path.is_dir()
                if is_directory
                else path.is_file()
            )
        )
        evidence.append({
            'path': relative,
            'kind': 'directory' if is_directory else 'file',
            'present': present,
            'size_bytes': (
                path.stat().st_size
                if present and not is_directory and path is not None
                else None
            ),
            'sha256': (
                _sha256(path)
                if present and not is_directory and path is not None
                else None
            ),
        })
    return evidence


def _manifest_evidence(
    run_dir: Path | None,
) -> tuple[str | None, dict[str, Any] | None]:
    path = run_dir / 'run_manifest.json' if run_dir else None
    if path is None or not path.is_file():
        return None, None
    try:
        manifest = _load_json(path)
    except (OSError, json.JSONDecodeError):
        return _sha256(path), None
    lifecycle = manifest.get('lifecycle') or {}
    software = manifest.get('software') or {}
    profile = manifest.get('profile') or {}
    output = manifest.get('output') or {}
    return _sha256(path), {
        'schema_version': manifest.get('schema_version'),
        'status': manifest.get('status'),
        'lifecycle_stage': lifecycle.get('stage'),
        'runner_exit_code': lifecycle.get('runner_exit_code'),
        'product_version': software.get('product_version'),
        'git_commit': software.get('git_commit'),
        'git_dirty': software.get('git_dirty'),
        'ros_distro': software.get('ros_distro'),
        'profile_id': profile.get('id'),
        'finalized': output.get('finalized'),
        'diagnosis_status': output.get('diagnosis_status'),
    }


def _verification_evidence(run_dir: Path | None) -> dict[str, Any]:
    path = run_dir / 'verify_autoware_map.log' if run_dir else None
    if path is None or not path.is_file():
        return {
            'present': False,
            'passed': False,
            'sha256': None,
            'summary': None,
        }
    text = path.read_text(encoding='utf-8', errors='replace')
    result_line = next(
        (line.strip() for line in text.splitlines() if 'RESULT:' in line),
        None,
    )
    counts = VERIFY_COUNTS.search(text)
    count_summary = (
        'PASS: {pass} | WARN: {warn} | FAIL: {fail}'.format(
            **counts.groupdict()
        )
        if counts
        else None
    )
    summary = ' — '.join(
        item for item in (result_line, count_summary) if item
    ) or None
    return {
        'present': True,
        'passed': bool(
            result_line
            and result_line.startswith('RESULT: PASS')
            and (counts is None or int(counts.group('fail')) == 0)
        ),
        'sha256': _sha256(path),
        'summary': summary,
    }


def _diagnosis_evidence(run_dir: Path | None) -> dict[str, Any]:
    path = run_dir / 'autoware_map_diagnosis.json' if run_dir else None
    if path is None or not path.is_file():
        return {'present': False, 'status': None, 'sha256': None}
    try:
        status = _load_json(path).get('status')
    except (OSError, json.JSONDecodeError, AttributeError):
        status = None
    return {
        'present': True,
        'status': status if isinstance(status, str) else None,
        'sha256': _sha256(path),
    }


def _check(check_id: str, passed: bool, observed: str) -> dict[str, Any]:
    return {'id': check_id, 'passed': passed, 'observed': observed}


def build_report(
    answers: dict[str, Any],
    run_dir: Path | None,
    artifact_paths: list[str],
) -> dict[str, Any]:
    """Build a redacted evidence report without copying raw run artifacts."""
    artifacts = _artifact_evidence(run_dir, artifact_paths)
    manifest_sha256, manifest = _manifest_evidence(run_dir)
    verification = _verification_evidence(run_dir)
    diagnosis = _diagnosis_evidence(run_dir)
    attestations = answers['attestations']
    immutable_identity = any(
        pattern.fullmatch(answers['tested_identity'])
        for pattern in IDENTITY_PATTERNS
    )
    tested_identity = answers['tested_identity']
    if IDENTITY_PATTERNS[0].fullmatch(tested_identity):
        identity_binding = bool(
            manifest and manifest['git_commit'] == tested_identity
        )
        identity_binding_observed = (
            f"reported={tested_identity}, "
            f"manifest_git_commit="
            f"{manifest.get('git_commit') if manifest else None}"
        )
    elif IDENTITY_PATTERNS[2].fullmatch(tested_identity):
        expected_version = tested_identity.removeprefix('v').split('+', 1)[0]
        identity_binding = bool(
            manifest and manifest['product_version'] == expected_version
        )
        identity_binding_observed = (
            f"reported={tested_identity}, "
            f"manifest_product_version="
            f"{manifest.get('product_version') if manifest else None}"
        )
    else:
        identity_binding = (
            answers['entrypoint_id'] == 'docker-demo'
            and bool(IDENTITY_PATTERNS[1].fullmatch(tested_identity))
        )
        identity_binding_observed = (
            f"entrypoint={answers['entrypoint_id']}, "
            'identity_kind=image_digest'
        )
    starting_url = urlparse(answers['starting_document'])
    manifest_success = bool(
        manifest
        and manifest['schema_version'] == 2
        and manifest['status'] == 'succeeded'
        and manifest['lifecycle_stage'] == 'complete'
        and manifest['runner_exit_code'] == 0
        and manifest['finalized'] is True
    )
    ros_matches = bool(
        manifest
        and manifest['ros_distro'] == answers['environment']['ros_distro']
    )
    checks = [
        _check(
            'immutable_tested_identity',
            immutable_identity,
            answers['tested_identity'],
        ),
        _check(
            'tested_identity_bound_to_entrypoint',
            identity_binding,
            identity_binding_observed,
        ),
        _check(
            'public_starting_document',
            starting_url.scheme == 'https' and bool(starting_url.netloc),
            answers['starting_document'],
        ),
        _check(
            'independent_tester_attested',
            attestations['independent_tester'],
            str(attestations['independent_tester']),
        ),
        _check(
            'docs_only_start_attested',
            attestations['docs_only_start'],
            str(attestations['docs_only_start']),
        ),
        _check(
            'first_attempt_preserved',
            attestations['first_attempt_preserved'],
            str(attestations['first_attempt_preserved']),
        ),
        _check(
            'commands_redacted',
            attestations['commands_redacted'],
            str(attestations['commands_redacted']),
        ),
        _check(
            'public_consent',
            answers['public_consent'],
            str(answers['public_consent']),
        ),
        _check(
            'first_attempt_verified_map',
            answers['first_attempt']['result'] == 'verified_map',
            answers['first_attempt']['result'],
        ),
        _check(
            'required_artifacts_present',
            all(item['present'] for item in artifacts),
            (
                f"{sum(item['present'] for item in artifacts)}"
                f'/{len(artifacts)} present'
            ),
        ),
        _check(
            'terminal_success_manifest',
            manifest_success,
            (
                f"status={manifest.get('status') if manifest else None}, "
                'stage='
                f"{manifest.get('lifecycle_stage') if manifest else None}, "
                'runner_exit_code='
                f"{manifest.get('runner_exit_code') if manifest else None}"
            ),
        ),
        _check(
            'verification_passed',
            verification['passed'],
            str(verification['summary']),
        ),
        _check(
            'diagnosis_success',
            diagnosis['status'] == 'success',
            f"status={diagnosis['status']}",
        ),
        _check(
            'reported_ros_distro_matches_manifest',
            ros_matches,
            (
                f"reported={answers['environment']['ros_distro']}, "
                f"manifest={manifest.get('ros_distro') if manifest else None}"
            ),
        ),
    ]
    eligible = all(item['passed'] for item in checks)
    return {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'acceptance_status': 'eligible' if eligible else 'not_eligible',
        'collected_at': _utc_now(),
        'submission': {
            key: answers[key]
            for key in (
                'entrypoint_id',
                'tested_identity',
                'starting_document',
                'environment',
                'commands',
                'first_attempt',
                'attestations',
                'public_consent',
            )
        },
        'run_evidence': {
            'run_directory_supplied': run_dir is not None,
            'manifest_sha256': manifest_sha256,
            'manifest_summary': manifest,
            'verification': verification,
            'diagnosis': diagnosis,
            'artifacts': artifacts,
        },
        'checks': checks,
        'privacy': {
            'geometry_included': False,
            'absolute_local_paths_included': False,
            'raw_logs_included': False,
        },
    }


def _markdown(report: dict[str, Any]) -> str:
    def safe(value: Any) -> str:
        return str(value).replace('|', '\\|').replace('\n', ' ')

    submission = report['submission']
    evidence = report['run_evidence']
    lines = [
        '# Independent first-map validation report',
        '',
        f"Acceptance status: **{report['acceptance_status']}**.",
        '',
        'This report contains hashes and bounded summaries only. It does not '
        'contain pointcloud geometry, raw logs or absolute local paths.',
        '',
        '## Submission',
        '',
        f"- Entrypoint: `{safe(submission['entrypoint_id'])}`",
        f"- Tested identity: `{safe(submission['tested_identity'])}`",
        f"- Starting document: {safe(submission['starting_document'])}",
        f"- First attempt: `{safe(submission['first_attempt']['result'])}`",
        f"- Elapsed: {safe(submission['first_attempt']['elapsed'])}",
        f"- ROS distro: `{safe(submission['environment']['ros_distro'])}`",
        f"- OS / architecture: {safe(submission['environment']['os'])} / "
        f"{safe(submission['environment']['architecture'])}",
        '',
        '## Checks',
        '',
        '| Check | Result | Observed |',
        '| --- | --- | --- |',
    ]
    for check in report['checks']:
        lines.append(
            f"| `{safe(check['id'])}` | "
            f"{'PASS' if check['passed'] else 'FAIL'} | "
            f"{safe(check['observed'])} |"
        )
    lines.extend([
        '',
        '## Evidence identity',
        '',
        '- Run directory supplied: '
        f"`{safe(evidence['run_directory_supplied'])}`",
        f"- Manifest SHA-256: `{safe(evidence['manifest_sha256'])}`",
        f"- Verification: {safe(evidence['verification']['summary'])}",
        f"- Diagnosis: `{safe(evidence['diagnosis']['status'])}`",
        '',
        '## First-attempt findings',
        '',
    ])
    findings = submission['first_attempt']['findings']
    lines.extend(
        [f'- {safe(finding)}' for finding in findings]
        or ['- No findings supplied.']
    )
    lines.extend([
        '',
        '## Redacted commands',
        '',
        '```bash',
        *submission['commands'],
        '```',
        '',
    ])
    return '\n'.join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Collect a schema-versioned, privacy-bounded first-map validation '
            'report for the independent v1.0 gate.'
        )
    )
    parser.add_argument(
        '--answers',
        type=Path,
        required=True,
        help='Completed copy of configs/first_map_validation_answers.example.json.',
    )
    parser.add_argument(
        '--run-dir',
        type=Path,
        help='Completed or partial map-run directory. Omit when none was created.',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        required=True,
        help='New or empty directory for the JSON and Markdown reports.',
    )
    parser.add_argument(
        '--require-eligible',
        action='store_true',
        help='Exit 1 when the generated report is not acceptance-eligible.',
    )
    args = parser.parse_args()

    try:
        answers = _load_json(args.answers.expanduser().resolve())
    except (OSError, json.JSONDecodeError) as exc:
        print(f'error: cannot read answers: {exc}', file=sys.stderr)
        return 2
    answer_errors = _validate_answers(answers)
    if answer_errors:
        print('error: invalid validation answers:', file=sys.stderr)
        for error in answer_errors:
            print(f'- {error}', file=sys.stderr)
        return 2

    run_dir = args.run_dir.expanduser().resolve() if args.run_dir else None
    if run_dir is not None and not run_dir.is_dir():
        print(f'error: run directory does not exist: {run_dir}', file=sys.stderr)
        return 2
    output_dir = args.output_dir.expanduser().resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        print(f'error: output directory is not empty: {output_dir}', file=sys.stderr)
        return 2
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        first_map_contract = _load_json(FIRST_MAP_CONTRACT)
        artifact_paths = first_map_contract['successful_run_artifacts']
        report = build_report(answers, run_dir, artifact_paths)
        _atomic_write(
            output_dir / REPORT_NAME,
            json.dumps(report, indent=2, sort_keys=True) + '\n',
        )
        _atomic_write(output_dir / MARKDOWN_NAME, _markdown(report))
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f'error: cannot collect validation evidence: {exc}', file=sys.stderr)
        return 2

    print(f"first-map validation report: {report['acceptance_status']}")
    print(f'- JSON: {output_dir / REPORT_NAME}')
    print(f'- Markdown: {output_dir / MARKDOWN_NAME}')
    if args.require_eligible and report['acceptance_status'] != 'eligible':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
