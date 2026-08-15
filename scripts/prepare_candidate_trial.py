#!/usr/bin/env python3
"""Download, audit, and package one exact candidate trial handoff.

The command performs bounded GitHub and registry reads, writes one new local
directory atomically, and never runs a trial or mutates remote state.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Callable, Sequence

from audit_candidate_image_set import (
    audit_candidate_bundle,
    EVIDENCE_FILES,
    EXPECTED_REPOSITORY,
    load_candidate_evidence_bundle,
    RUN_URL_RE,
)

from prepare_onboarding_matrix_packet import (
    build_candidate_packet,
    render_packet,
)

from product_schema import validate_contract


PREPARATION_SCHEMA = 'candidate-trial-preparation-v1.schema.json'
PREPARATION_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-trial-preparation-v1.schema.json'
)
ARTIFACTS_DIRECTORY = 'artifacts'
AUDIT_FILENAME = 'candidate-audit.json'
PACKET_JSON_FILENAME = 'observer-packet.json'
PACKET_MARKDOWN_FILENAME = 'observer-packet.md'
PREPARATION_FILENAME = 'preparation.json'
EXPECTED_ARTIFACTS = tuple(item[0] for item in EVIDENCE_FILES)
EXPECTED_FILENAMES = tuple(item[1] for item in EVIDENCE_FILES)
Runner = Callable[..., subprocess.CompletedProcess[str]]


class CandidateTrialPreparationError(ValueError):
    """The requested handoff cannot be prepared without guessing."""

    exit_code = 2


class CandidateTrialCheckError(CandidateTrialPreparationError):
    """A required remote or evidence check did not pass."""

    exit_code = 1


def _utc_now() -> str:
    """Return one second-resolution UTC contract timestamp."""
    return (
        datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace('+00:00', 'Z')
    )


def _validate_request(
    workflow_run_url: str,
    output_dir: Path,
) -> tuple[int, Path]:
    """Validate identity and destination before performing a network read."""
    match = RUN_URL_RE.fullmatch(workflow_run_url)
    if match is None:
        raise CandidateTrialPreparationError(
            'workflow run URL must be the exact rsasaki0109/lidar_slam_ros2 '
            'Actions run URL'
        )
    if os.path.lexists(output_dir):
        raise CandidateTrialPreparationError(
            f'refusing to overwrite candidate handoff: {output_dir}'
        )
    parent = output_dir.parent
    try:
        parent_is_valid = parent.is_dir() and not parent.is_symlink()
    except OSError as exc:
        raise CandidateTrialPreparationError(
            f'output parent cannot be inspected: {exc}'
        ) from exc
    if not parent_is_valid:
        raise CandidateTrialPreparationError(
            'output parent must be one existing, non-symlink directory: '
            f'{parent}'
        )
    return int(match.group('run_id')), parent


def _run_download(
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
            timeout=120,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return subprocess.CompletedProcess(command, 125, '', str(exc))


def _failure_detail(result: subprocess.CompletedProcess[str]) -> str:
    detail = result.stderr.strip() or result.stdout.strip()
    if not detail:
        return f'exit {result.returncode}'
    return detail.splitlines()[0][:300]


def _download_candidate_artifacts(
    run_id: int,
    destination: Path,
    *,
    runner: Runner,
) -> None:
    """Download each named single-file artifact into one canonical directory."""
    destination.mkdir(mode=0o700)
    for artifact_name, _filename, _key in EVIDENCE_FILES:
        result = _run_download(
            [
                'gh', 'run', 'download', str(run_id),
                '--repo', EXPECTED_REPOSITORY,
                '--name', artifact_name,
                '--dir', str(destination),
            ],
            runner=runner,
        )
        if result.returncode != 0:
            raise CandidateTrialCheckError(
                f'failed to download {artifact_name}: '
                f'{_failure_detail(result)}'
            )


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    with path.open('x', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write('\n')


def _build_receipt(
    *,
    evidence_bundle: dict[str, Any],
    audit: dict[str, Any],
    packet: dict[str, Any],
    prepared_at: str,
) -> dict[str, Any]:
    candidate_set = evidence_bundle['candidate_set']
    receipt = {
        'schema_version': 1,
        'schema_uri': PREPARATION_SCHEMA_URI,
        'status': 'READY_FOR_OBSERVER',
        'prepared_at': prepared_at,
        'repository': candidate_set['repository'],
        'workflow_run_url': candidate_set['workflow_run_url'],
        'run_id': audit['workflow']['run_id'],
        'source_pr': candidate_set['source_pr'],
        'source_commit': candidate_set['source_commit'],
        'product_version': candidate_set['product_version'],
        'candidate_bundle_sha256': evidence_bundle['bundle_sha256'],
        'candidate_set_sha256': evidence_bundle['file_hashes'][
            'candidate-image-set.json'
        ],
        'artifact_expires_at': audit['artifacts']['expires_at'],
        'audit_status': audit['status'],
        'observer_packet_status': packet['status'],
        'artifacts': audit['retained_evidence']['files'],
        'outputs': {
            'artifacts_directory': ARTIFACTS_DIRECTORY,
            'audit_json': AUDIT_FILENAME,
            'observer_packet_json': PACKET_JSON_FILENAME,
            'observer_packet_markdown': PACKET_MARKDOWN_FILENAME,
        },
        'authority': {
            'network_reads_performed': True,
            'local_files_written': True,
            'atomic_output_publication': True,
            'trial_executed': False,
            'github_writes_authorized': False,
            'registry_writes_authorized': False,
            'remote_mutations_performed': False,
        },
    }
    validate_contract(receipt, PREPARATION_SCHEMA)
    return receipt


def prepare_candidate_trial(
    workflow_run_url: str,
    output_dir: Path,
    *,
    runner: Runner = subprocess.run,
    prepared_at: str | None = None,
) -> dict[str, Any]:
    """Publish one fully authenticated local candidate-trial handoff."""
    run_id, parent = _validate_request(workflow_run_url, output_dir)
    staging = Path(tempfile.mkdtemp(
        prefix=f'.{output_dir.name}.preparing-',
        dir=parent,
    ))
    try:
        artifacts = staging / ARTIFACTS_DIRECTORY
        _download_candidate_artifacts(run_id, artifacts, runner=runner)
        try:
            evidence_bundle = load_candidate_evidence_bundle(artifacts)
        except ValueError as exc:
            raise CandidateTrialCheckError(
                f'downloaded candidate evidence is invalid: {exc}'
            ) from exc
        candidate_run_url = evidence_bundle['candidate_set'][
            'workflow_run_url'
        ]
        if candidate_run_url != workflow_run_url:
            raise CandidateTrialCheckError(
                'downloaded evidence names a different workflow run URL'
            )

        audit = audit_candidate_bundle(
            evidence_bundle,
            remote=True,
            runner=runner,
        )
        if audit['status'] != 'REMOTE_AUDIT_PASS':
            findings = ', '.join(audit['findings']) or 'unknown finding'
            raise CandidateTrialCheckError(
                f'remote candidate audit did not pass: {findings}'
            )
        packet = build_candidate_packet(evidence_bundle)
        receipt = _build_receipt(
            evidence_bundle=evidence_bundle,
            audit=audit,
            packet=packet,
            prepared_at=prepared_at or _utc_now(),
        )

        _write_json(staging / AUDIT_FILENAME, audit)
        _write_json(staging / PACKET_JSON_FILENAME, packet)
        with (staging / PACKET_MARKDOWN_FILENAME).open(
            'x', encoding='utf-8'
        ) as stream:
            stream.write(render_packet(packet))
        _write_json(staging / PREPARATION_FILENAME, receipt)
        os.replace(staging, output_dir)
        return receipt
    except CandidateTrialPreparationError:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    except (OSError, ValueError) as exc:
        shutil.rmtree(staging, ignore_errors=True)
        raise CandidateTrialPreparationError(str(exc)) from exc
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--workflow-run-url',
        required=True,
        help=(
            'exact successful candidate-image Actions run URL; the command '
            'performs read-only GitHub, GHCR, and attestation checks'
        ),
    )
    parser.add_argument(
        '--output-dir',
        required=True,
        type=Path,
        help=(
            'new handoff directory to publish atomically; its parent must '
            'already exist and the directory is never overwritten'
        ),
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='print the preparation receipt after writing the handoff',
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Prepare one candidate handoff or leave the requested output absent."""
    args = _parse_args(argv)
    try:
        receipt = prepare_candidate_trial(
            args.workflow_run_url,
            args.output_dir,
        )
    except CandidateTrialPreparationError as exc:
        print(f'candidate trial preparation error: {exc}', file=sys.stderr)
        return exc.exit_code

    if args.json:
        print(json.dumps(receipt, indent=2, sort_keys=True))
    else:
        print(f'Candidate trial handoff ready: {args.output_dir}')
        print('Remote audit: REMOTE_AUDIT_PASS')
        print(
            'Candidate bundle SHA-256: '
            f"{receipt['candidate_bundle_sha256']}"
        )
        print(
            f'Observer packet: {args.output_dir / PACKET_MARKDOWN_FILENAME}'
        )
        print('No trial or remote mutation was performed.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
