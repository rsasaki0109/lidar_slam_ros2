#!/usr/bin/env python3
"""Prepare and run one exact candidate trial as one atomic local session.

The command starts from one candidate-image Actions run URL, authenticates its
four-file handoff, runs one structured Docker/source row, and publishes the
handoff plus bounded execution evidence under one new session directory.  It
performs no GitHub, registry, release, issue, or community write.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Sequence

from prepare_candidate_trial import (
    CandidateTrialPreparationError,
    EXPECTED_HANDOFF_ENTRIES,
    PREPARATION_FILENAME,
    prepare_candidate_trial,
)

from product_schema import validate_contract

from run_candidate_trial import (
    CandidateTrialExecutionError,
    EXECUTION_FILENAME,
    INTERFACE_RE,
    REPOSITORY,
    REPO_ROOT,
    ROW_IDS,
    TRIAL_ID_RE,
    run_candidate_trial,
)


SESSION_SCHEMA = 'candidate-trial-session-v1.schema.json'
SESSION_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-trial-session-v1.schema.json'
)
SESSION_FILENAME = 'session.json'
HANDOFF_DIRECTORY = 'handoff'
EXECUTION_DIRECTORY = 'execution'
MAX_CHILD_RECEIPT_BYTES = 1024 * 1024
Runner = Callable[..., subprocess.CompletedProcess[str]]


class CandidateTrialSessionError(ValueError):
    """A complete candidate trial session cannot be created safely."""

    def __init__(self, message: str, *, exit_code: int = 2) -> None:
        """Keep the stage-specific process exit code with the error."""
        super().__init__(message)
        self.exit_code = exit_code


def _utc_now() -> str:
    return (
        datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace('+00:00', 'Z')
    )


def _load_child_receipt(
    path: Path,
    label: str,
) -> tuple[dict[str, Any], str]:
    """Load and hash the exact bounded child receipt retained on disk."""
    try:
        if path.is_symlink() or not path.is_file():
            raise CandidateTrialSessionError(
                f'{label} is not one regular non-symlink file'
            )
        size = path.stat().st_size
        if size < 1 or size > MAX_CHILD_RECEIPT_BYTES:
            raise CandidateTrialSessionError(
                f'{label} must be 1..{MAX_CHILD_RECEIPT_BYTES} bytes'
            )
        payload = path.read_bytes()
        receipt = json.loads(payload.decode('utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CandidateTrialSessionError(
            f'cannot read {label}: {exc}'
        ) from exc
    if not isinstance(receipt, dict):
        raise CandidateTrialSessionError(f'{label} JSON root is not an object')
    return receipt, hashlib.sha256(payload).hexdigest()


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    with path.open('x', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write('\n')


def _contains(parent: Path, child: Path) -> bool:
    try:
        child.relative_to(parent)
    except ValueError:
        return False
    return True


def _validate_request(
    output_dir: Path,
    row_id: str,
    *,
    acknowledge_dedicated_trial_host: bool,
    trial_id: str | None,
    human_measurements: str,
    disk_scope: Path,
    network_interface: str | None,
    interactive: bool,
    timeout_sec: float,
) -> tuple[Path, Path]:
    """Validate every local mutation boundary before staging any output."""
    if not acknowledge_dedicated_trial_host:
        raise CandidateTrialSessionError(
            '--acknowledge-dedicated-trial-host is required; source rows '
            'modify the disposable host and Docker rows build/use a reviewed '
            'observer image plus a privileged isolated container'
        )
    if row_id not in ROW_IDS:
        raise CandidateTrialSessionError(
            'row must be one of: ' + ', '.join(ROW_IDS)
        )
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise CandidateTrialSessionError(
            'timeout must be finite and greater than zero'
        )
    if trial_id is not None and TRIAL_ID_RE.fullmatch(trial_id) is None:
        raise CandidateTrialSessionError(
            'trial ID must be a privacy-bounded 3..80 character lower-case '
            'slug'
        )
    if human_measurements not in {'auto', 'prompt', 'unknown'}:
        raise CandidateTrialSessionError(
            'human measurement mode must be auto, prompt, or unknown'
        )
    if human_measurements == 'prompt' and not interactive:
        raise CandidateTrialSessionError(
            '--human-measurements prompt requires an interactive terminal'
        )
    route = row_id.split('-', 1)[0]
    if network_interface is not None:
        if route != 'source':
            raise CandidateTrialSessionError(
                '--network-interface is available only for a source row'
            )
        if INTERFACE_RE.fullmatch(network_interface) is None:
            raise CandidateTrialSessionError(
                'network interface name is unsafe'
            )
    try:
        if disk_scope.is_symlink() or not disk_scope.is_dir():
            raise CandidateTrialSessionError(
                'disk scope must be one existing real directory'
            )
        disk_scope.resolve(strict=True)
    except OSError as exc:
        raise CandidateTrialSessionError(
            f'disk scope cannot be resolved: {exc}'
        ) from exc
    if os.path.lexists(output_dir):
        raise CandidateTrialSessionError(
            f'refusing to overwrite candidate trial session: {output_dir}'
        )
    parent = output_dir.parent
    try:
        if parent.is_symlink() or not parent.is_dir():
            raise CandidateTrialSessionError(
                'session output parent must be one existing non-symlink '
                'directory'
            )
        resolved_parent = parent.resolve(strict=True)
    except OSError as exc:
        raise CandidateTrialSessionError(
            f'session output parent cannot be resolved: {exc}'
        ) from exc
    output = resolved_parent / output_dir.name
    if _contains(REPO_ROOT.resolve(), output):
        raise CandidateTrialSessionError(
            'candidate trial session must be outside the product checkout'
        )
    return output, resolved_parent


def _build_session(
    *,
    started_at: str,
    completed_at: str,
    workflow_run_url: str,
    preparation: dict[str, Any],
    preparation_sha256: str,
    execution: dict[str, Any],
    execution_sha256: str,
) -> dict[str, Any]:
    handoff_entries = [
        f'{HANDOFF_DIRECTORY}/{name}'
        for name in sorted(EXPECTED_HANDOFF_ENTRIES)
    ]
    execution_entries = [
        f'{EXECUTION_DIRECTORY}/{name}'
        for name in execution['sharing']['bounded_outputs']
    ]
    private = execution['outputs']['private_evidence_directory']
    receipt = {
        'schema_version': 1,
        'schema_uri': SESSION_SCHEMA_URI,
        'status': execution['status'],
        'started_at': started_at,
        'completed_at': completed_at,
        'repository': REPOSITORY,
        'workflow_run_url': workflow_run_url,
        'row': execution['row'],
        'handoff': {
            'directory': HANDOFF_DIRECTORY,
            'preparation_json': (
                f'{HANDOFF_DIRECTORY}/{PREPARATION_FILENAME}'
            ),
            'preparation_sha256': preparation_sha256,
            'status': preparation['status'],
            'run_id': preparation['run_id'],
            'source_pr': preparation['source_pr'],
            'source_commit': preparation['source_commit'],
            'product_version': preparation['product_version'],
            'candidate_bundle_sha256': (
                preparation['candidate_bundle_sha256']
            ),
            'candidate_set_sha256': preparation['candidate_set_sha256'],
            'artifact_expires_at': preparation['artifact_expires_at'],
        },
        'execution': {
            'directory': EXECUTION_DIRECTORY,
            'execution_json': (
                f'{EXECUTION_DIRECTORY}/{EXECUTION_FILENAME}'
            ),
            'execution_sha256': execution_sha256,
            'status': execution['status'],
            'trial_id': execution['trial_id'],
            'outcome_status': execution['trial']['outcome_status'],
            'measurement_status': execution['trial']['measurement_status'],
            'comparable': execution['trial']['comparable'],
        },
        'outputs': {
            'handoff_directory': HANDOFF_DIRECTORY,
            'execution_directory': EXECUTION_DIRECTORY,
            'session_json': SESSION_FILENAME,
        },
        'sharing': {
            'bounded_outputs': [
                SESSION_FILENAME,
                *handoff_entries,
                *execution_entries,
            ],
            'private_evidence_directory': (
                None if private is None
                else f'{EXECUTION_DIRECTORY}/{private}'
            ),
            'private_evidence_shareable': False,
            'review_before_sharing': True,
        },
        'authority': {
            'network_reads_performed': True,
            'local_files_written': True,
            'atomic_output_publication': True,
            'candidate_handoff_prepared': True,
            'trial_executed': execution['authority']['trial_executed'],
            'docker_observer_bootstrap_requested': (
                execution['row']['route'] == 'docker'
            ),
            'github_writes_authorized': False,
            'registry_writes_authorized': False,
            'release_writes_authorized': False,
            'community_writes_authorized': False,
            'remote_mutations_performed': False,
        },
    }
    validate_contract(receipt, SESSION_SCHEMA)
    return receipt


def start_candidate_trial(
    workflow_run_url: str,
    row_id: str,
    output_dir: Path,
    *,
    acknowledge_dedicated_trial_host: bool,
    trial_id: str | None = None,
    human_measurements: str = 'auto',
    disk_scope: Path = Path('/'),
    timeout_sec: float = 7200.0,
    network_interface: str | None = None,
    preparation_runner: Runner = subprocess.run,
    read_runner: Runner = subprocess.run,
    probe_runner: Runner = subprocess.run,
    interactive: bool | None = None,
    started_at: str | None = None,
) -> tuple[dict[str, Any], int]:
    """Prepare, execute, and publish one complete candidate trial session."""
    resolved_interactive = (
        sys.stdin.isatty() if interactive is None else interactive
    )
    output, parent = _validate_request(
        output_dir,
        row_id,
        acknowledge_dedicated_trial_host=(
            acknowledge_dedicated_trial_host
        ),
        trial_id=trial_id,
        human_measurements=human_measurements,
        disk_scope=disk_scope,
        network_interface=network_interface,
        interactive=resolved_interactive,
        timeout_sec=timeout_sec,
    )
    start = started_at or _utc_now()
    staging = Path(tempfile.mkdtemp(
        prefix=f'.{output.name}.session-',
        dir=parent,
    ))
    try:
        handoff_dir = staging / HANDOFF_DIRECTORY
        execution_dir = staging / EXECUTION_DIRECTORY
        preparation = prepare_candidate_trial(
            workflow_run_url,
            handoff_dir,
            runner=preparation_runner,
            prepared_at=start,
        )
        execution, exit_code = run_candidate_trial(
            handoff_dir,
            row_id,
            execution_dir,
            acknowledge_dedicated_trial_host=True,
            trial_id=trial_id,
            human_measurements=human_measurements,
            disk_scope=disk_scope,
            timeout_sec=timeout_sec,
            network_interface=network_interface,
            read_runner=read_runner,
            probe_runner=probe_runner,
            interactive=resolved_interactive,
            started_at=start,
        )
        retained_preparation, preparation_sha256 = _load_child_receipt(
            handoff_dir / PREPARATION_FILENAME,
            'retained preparation receipt',
        )
        if retained_preparation != preparation:
            raise CandidateTrialSessionError(
                'retained preparation receipt differs from the delegated '
                'result'
            )
        retained_execution, execution_sha256 = _load_child_receipt(
            execution_dir / EXECUTION_FILENAME,
            'retained execution receipt',
        )
        if retained_execution != execution:
            raise CandidateTrialSessionError(
                'retained execution receipt differs from the delegated '
                'result'
            )
        receipt = _build_session(
            started_at=start,
            completed_at=_utc_now(),
            workflow_run_url=workflow_run_url,
            preparation=retained_preparation,
            preparation_sha256=preparation_sha256,
            execution=retained_execution,
            execution_sha256=execution_sha256,
        )
        _write_json(staging / SESSION_FILENAME, receipt)
        os.replace(staging, output)
        return receipt, exit_code
    except CandidateTrialPreparationError as exc:
        shutil.rmtree(staging, ignore_errors=True)
        raise CandidateTrialSessionError(
            f'candidate handoff preparation failed: {exc}',
            exit_code=exc.exit_code,
        ) from exc
    except CandidateTrialExecutionError as exc:
        shutil.rmtree(staging, ignore_errors=True)
        raise CandidateTrialSessionError(
            f'candidate row execution failed before evidence publication: '
            f'{exc}',
            exit_code=exc.exit_code,
        ) from exc
    except CandidateTrialSessionError:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    except (OSError, ValueError) as exc:
        shutil.rmtree(staging, ignore_errors=True)
        raise CandidateTrialSessionError(str(exc)) from exc
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--workflow-run-url',
        required=True,
        help='exact successful candidate-image Actions run URL',
    )
    parser.add_argument('--row', required=True, choices=ROW_IDS)
    parser.add_argument(
        '--output-dir',
        required=True,
        type=Path,
        help=(
            'new atomic session directory outside the checkout; contains '
            'handoff/, execution/, and session.json'
        ),
    )
    parser.add_argument(
        '--acknowledge-dedicated-trial-host',
        action='store_true',
        help=(
            'confirm an isolated disposable trial host and dedicated '
            'measured filesystem; permit source changes or reviewed Docker '
            'bootstrap and the privileged nested host for the selected row'
        ),
    )
    parser.add_argument('--trial-id')
    parser.add_argument(
        '--human-measurements',
        choices=('auto', 'prompt', 'unknown'),
        default='auto',
    )
    parser.add_argument('--disk-scope', type=Path, default=Path('/'))
    parser.add_argument('--timeout-sec', type=float, default=7200.0)
    parser.add_argument(
        '--network-interface',
        help='optional isolated interface override for a source row',
    )
    parser.add_argument('--json', action='store_true')
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Run one complete session and retain valid blocked/FAIL evidence."""
    args = _parse_args(argv)
    try:
        receipt, exit_code = start_candidate_trial(
            args.workflow_run_url,
            args.row,
            args.output_dir,
            acknowledge_dedicated_trial_host=(
                args.acknowledge_dedicated_trial_host
            ),
            trial_id=args.trial_id,
            human_measurements=args.human_measurements,
            disk_scope=args.disk_scope,
            timeout_sec=args.timeout_sec,
            network_interface=args.network_interface,
        )
    except CandidateTrialSessionError as exc:
        print(f'candidate trial session error: {exc}', file=sys.stderr)
        return exc.exit_code

    if args.json:
        print(json.dumps(receipt, indent=2, sort_keys=True))
    else:
        print(f"Candidate trial session: {receipt['status']}")
        print(f"Row: {receipt['row']['row_id']}")
        print(f'Session evidence: {args.output_dir}')
        print(f'Handoff: {args.output_dir / HANDOFF_DIRECTORY}')
        print(f'Execution: {args.output_dir / EXECUTION_DIRECTORY}')
        if receipt['execution']['outcome_status'] is not None:
            print(
                'Outcome: '
                f"{receipt['execution']['outcome_status']} / "
                f"measurements {receipt['execution']['measurement_status']}"
            )
            print(
                'Comparable: '
                + ('YES' if receipt['execution']['comparable'] else 'NO')
            )
        print('No GitHub, registry, release, issue, or community write ran.')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
