#!/usr/bin/env python3
"""Audit one immutable candidate-image set without mutating remote state."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable, Sequence

from product_schema import load_json_object, validate_contract


CANDIDATE_SET_SCHEMA = 'candidate-image-set-v1.schema.json'
AUDIT_SCHEMA = 'candidate-image-set-audit-v1.schema.json'
AUDIT_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-image-set-audit-v1.schema.json'
)
EXPECTED_REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
EXPECTED_WORKFLOW_PATH = '.github/workflows/candidate-image.yml'
EXPECTED_ARTIFACTS = (
    'candidate-image-request',
    'candidate-image-record-humble',
    'candidate-image-record-jazzy',
    'candidate-image-set',
)
RUN_URL_RE = re.compile(
    r'^https://github\.com/rsasaki0109/lidar_slam_ros2/actions/'
    r'runs/(?P<run_id>[1-9][0-9]*)$'
)
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
Runner = Callable[..., subprocess.CompletedProcess[str]]


class CandidateSetAuditError(ValueError):
    """Candidate evidence cannot be interpreted without guessing."""


def sha256_file(path: Path) -> str:
    """Hash one retained evidence file without loading arbitrary-size input."""
    digest = hashlib.sha256()
    try:
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    except OSError as exc:
        raise CandidateSetAuditError(
            f'candidate set cannot be hashed: {exc}'
        ) from exc
    return digest.hexdigest()


def validate_candidate_set(
    candidate_set: dict[str, Any],
) -> list[dict[str, str]]:
    """Require one self-consistent tag-free image for each distro."""
    validate_contract(candidate_set, CANDIDATE_SET_SCHEMA)
    if candidate_set['repository'] != EXPECTED_REPOSITORY:
        raise CandidateSetAuditError('candidate repository is unexpected')
    images = candidate_set['images']
    by_distro = {image['ros_distro']: image for image in images}
    if len(by_distro) != 2 or set(by_distro) != {'humble', 'jazzy'}:
        raise CandidateSetAuditError(
            'candidate set requires Humble and Jazzy exactly once'
        )
    ordered = [by_distro['humble'], by_distro['jazzy']]
    if len({image['digest'] for image in ordered}) != 2:
        raise CandidateSetAuditError(
            'Humble and Jazzy candidate digests must differ'
        )
    repository_ref = f"ghcr.io/{candidate_set['repository']}"
    for image in ordered:
        expected_ref = f"{repository_ref}@{image['digest']}"
        if image['immutable_ref'] != expected_ref:
            raise CandidateSetAuditError(
                f"{image['ros_distro']} immutable ref does not match digest"
            )
    run_match = RUN_URL_RE.fullmatch(candidate_set['workflow_run_url'])
    if run_match is None:
        raise CandidateSetAuditError('candidate workflow run URL is malformed')
    return ordered


def _run(
    command: list[str],
    *,
    runner: Runner,
) -> subprocess.CompletedProcess[str]:
    try:
        return runner(
            command,
            capture_output=True,
            text=True,
            check=False,
            timeout=60,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return subprocess.CompletedProcess(command, 125, '', str(exc))


def _json_object(
    result: subprocess.CompletedProcess[str],
) -> dict[str, Any] | None:
    if result.returncode != 0:
        return None
    try:
        value = json.loads(result.stdout)
    except json.JSONDecodeError:
        return None
    return value if isinstance(value, dict) else None


def _base_report(
    candidate_set: dict[str, Any],
    *,
    candidate_set_sha256: str,
    images: Sequence[dict[str, str]],
) -> dict[str, Any]:
    run_match = RUN_URL_RE.fullmatch(candidate_set['workflow_run_url'])
    if run_match is None:  # already rejected by validate_candidate_set
        raise CandidateSetAuditError('candidate workflow run URL is malformed')
    return {
        'schema_version': 1,
        'schema_uri': AUDIT_SCHEMA_URI,
        'status': 'LOCAL_CONTRACT_PASS',
        'candidate_set_sha256': candidate_set_sha256,
        'repository': candidate_set['repository'],
        'source_pr': candidate_set['source_pr'],
        'source_commit': candidate_set['source_commit'],
        'product_version': candidate_set['product_version'],
        'workflow_run_url': candidate_set['workflow_run_url'],
        'workflow_branch_ref': candidate_set['workflow_branch_ref'],
        'requested_by': candidate_set['requested_by'],
        'workflow': {
            'status': 'NOT_CHECKED',
            'run_id': int(run_match.group('run_id')),
            'event': None,
            'conclusion': None,
            'head_branch': None,
            'path': None,
        },
        'artifacts': {
            'status': 'NOT_CHECKED',
            'required_names': list(EXPECTED_ARTIFACTS),
            'expires_at': None,
        },
        'images': [
            {
                'ros_distro': image['ros_distro'],
                'digest': image['digest'],
                'immutable_ref': image['immutable_ref'],
                'registry_status': 'NOT_CHECKED',
                'attestation_status': 'NOT_CHECKED',
            }
            for image in images
        ],
        'findings': [],
        'authority': {
            'network_reads_performed': False,
            'github_writes_authorized': False,
            'registry_writes_authorized': False,
            'remote_mutations_performed': False,
        },
    }


def audit_candidate_set(
    candidate_set: dict[str, Any],
    *,
    candidate_set_sha256: str,
    remote: bool = False,
    runner: Runner = subprocess.run,
) -> dict[str, Any]:
    """Validate local identity and optionally perform bounded remote reads."""
    if SHA256_RE.fullmatch(candidate_set_sha256) is None:
        raise CandidateSetAuditError('candidate set SHA-256 is malformed')
    images = validate_candidate_set(candidate_set)
    report = _base_report(
        candidate_set,
        candidate_set_sha256=candidate_set_sha256,
        images=images,
    )
    if not remote:
        validate_contract(report, AUDIT_SCHEMA)
        return report

    report['authority']['network_reads_performed'] = True
    findings: list[str] = []
    repository = candidate_set['repository']
    run_id = report['workflow']['run_id']

    run_result = _run(
        ['gh', 'api', f'repos/{repository}/actions/runs/{run_id}'],
        runner=runner,
    )
    run = _json_object(run_result)
    if run is None:
        findings.append('workflow-run-unavailable')
        report['workflow']['status'] = 'FAIL'
    else:
        report['workflow'].update({
            'event': run.get('event'),
            'conclusion': run.get('conclusion'),
            'head_branch': run.get('head_branch'),
            'path': run.get('path'),
        })
        run_ok = (
            run.get('id') == run_id
            and run.get('html_url') == candidate_set['workflow_run_url']
            and run.get('event') == 'repository_dispatch'
            and run.get('status') == 'completed'
            and run.get('conclusion') == 'success'
            and run.get('head_branch') == 'develop'
            and run.get('path') == EXPECTED_WORKFLOW_PATH
        )
        report['workflow']['status'] = 'PASS' if run_ok else 'FAIL'
        if not run_ok:
            findings.append('workflow-run-identity-mismatch')

    artifacts_result = _run(
        [
            'gh', 'api',
            f'repos/{repository}/actions/runs/{run_id}/artifacts?per_page=100',
        ],
        runner=runner,
    )
    artifacts_payload = _json_object(artifacts_result)
    if artifacts_payload is None:
        report['artifacts']['status'] = 'FAIL'
        findings.append('workflow-artifacts-unavailable')
    else:
        artifacts = artifacts_payload.get('artifacts')
        if not isinstance(artifacts, list):
            artifacts = []
        active_by_name: dict[str, list[dict[str, Any]]] = {
            name: [] for name in EXPECTED_ARTIFACTS
        }
        for artifact in artifacts:
            if not isinstance(artifact, dict):
                continue
            name = artifact.get('name')
            if name in active_by_name and artifact.get('expired') is False:
                active_by_name[name].append(artifact)
        artifacts_ok = all(
            len(active_by_name[name]) == 1 for name in EXPECTED_ARTIFACTS
        )
        expires = [
            active_by_name[name][0].get('expires_at')
            for name in EXPECTED_ARTIFACTS
            if len(active_by_name[name]) == 1
        ]
        if artifacts_ok and all(
            isinstance(value, str) and value for value in expires
        ):
            report['artifacts']['status'] = 'PASS'
            report['artifacts']['expires_at'] = min(expires)
        else:
            report['artifacts']['status'] = 'FAIL'
            findings.append('candidate-artifact-set-incomplete-or-expired')

    for image_report in report['images']:
        immutable_ref = image_report['immutable_ref']
        manifest_result = _run(
            [
                'docker', 'buildx', 'imagetools', 'inspect', immutable_ref,
                '--format', '{{json .Manifest}}',
            ],
            runner=runner,
        )
        manifest = _json_object(manifest_result)
        registry_ok = (
            manifest is not None
            and manifest.get('digest') == image_report['digest']
        )
        image_report['registry_status'] = 'PASS' if registry_ok else 'FAIL'
        if not registry_ok:
            findings.append(
                f"{image_report['ros_distro']}-digest-unavailable-"
                'or-mismatched'
            )

        attestation_result = _run(
            [
                'gh', 'attestation', 'verify', f'oci://{immutable_ref}',
                '-R', repository,
            ],
            runner=runner,
        )
        attestation_ok = attestation_result.returncode == 0
        image_report['attestation_status'] = (
            'PASS' if attestation_ok else 'FAIL'
        )
        if not attestation_ok:
            findings.append(
                f"{image_report['ros_distro']}-attestation-unverified"
            )

    report['findings'] = sorted(set(findings))
    report['status'] = (
        'REMOTE_AUDIT_PASS' if not report['findings']
        else 'REMOTE_AUDIT_FAIL'
    )
    validate_contract(report, AUDIT_SCHEMA)
    return report


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate-image-set', required=True, type=Path)
    parser.add_argument(
        '--remote',
        action='store_true',
        help=(
            'read the exact workflow, retained artifacts, GHCR digests, and '
            'GitHub attestations; never write remote state'
        ),
    )
    parser.add_argument('--json', action='store_true')
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Render one fail-closed local or remote candidate audit."""
    args = _parse_args(argv)
    try:
        candidate_set = load_json_object(
            args.candidate_image_set,
            'candidate image set',
        )
        report = audit_candidate_set(
            candidate_set,
            candidate_set_sha256=sha256_file(args.candidate_image_set),
            remote=args.remote,
        )
    except (OSError, ValueError) as exc:
        print(f'candidate image set audit error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(report['status'])
        for finding in report['findings']:
            print(f'- {finding}')
    return 1 if report['status'] == 'REMOTE_AUDIT_FAIL' else 0


if __name__ == '__main__':
    raise SystemExit(main())
