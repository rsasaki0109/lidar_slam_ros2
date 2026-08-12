#!/usr/bin/env python3
"""Validate or render the bounded independent first-map cohort packet."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys
from typing import Any, Sequence

import jsonschema


sys.dont_write_bytecode = True

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = (
    REPO_ROOT / 'docs' / 'contracts' / 'first-map-validator-cohort-v1.json'
)
DEFAULT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'first-map-validator-cohort-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
    'first-map-validator-cohort-v1.schema.json'
)
ISSUE_FORM_URL = (
    'https://github.com/rsasaki0109/lidar_slam_ros2/issues/new'
    '?template=first-map-validation.yml'
)


class CohortError(ValueError):
    """The cohort contract is invalid or not copy-ready."""


def _load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise CohortError(f'cannot read {label} {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise CohortError(f'{label} root must be an object')
    return payload


def _schema_path(error: jsonschema.ValidationError) -> str:
    return '.'.join(str(item) for item in error.absolute_path) or '<root>'


def _ready_gates(gates: dict[str, Any]) -> bool:
    path = gates['canonical_documentation_path']
    runtime_ref = gates['canonical_runtime_ref']
    identity_matches_path = (
        path == 'docker-first-map'
        and isinstance(runtime_ref, str)
        and runtime_ref.startswith('ghcr.io/')
    ) or (
        path == 'source-quickstart'
        and runtime_ref == gates['public_revision']
    )
    return (
        gates['public_revision'] is not None
        and gates['public_revision_resolvable']
        and gates['comparable_docker_row']
        and gates['comparable_source_row']
        and gates['canonical_documentation_path'] is not None
        and gates['canonical_documentation_url'] is not None
        and gates['canonical_runtime_ref'] is not None
        and gates['copy_ready_handoff_public']
        and identity_matches_path
    )


def validate_contract(
    contract: dict[str, Any],
    schema: dict[str, Any],
) -> dict[str, Any]:
    """Validate schema, launch state, capacity, and no-write boundaries."""
    validator = jsonschema.Draft7Validator(
        schema,
        format_checker=jsonschema.FormatChecker(),
    )
    errors = sorted(
        validator.iter_errors(contract),
        key=lambda item: [str(part) for part in item.absolute_path],
    )
    if errors:
        first = errors[0]
        raise CohortError(
            f'schema validation failed at {_schema_path(first)}: '
            f'{first.message}'
        )
    if contract['schema_uri'] != SCHEMA_URI:
        raise CohortError('contract schema_uri is not the supported v1 URI')

    capacity = contract['capacity']
    if not (
        capacity['accepted_target']
        <= capacity['initial_attempt_cap']
        <= capacity['hard_attempt_cap']
    ):
        raise CohortError('cohort attempt capacity is inconsistent')
    if capacity['max_concurrent_attempts'] > 2:
        raise CohortError('cohort exceeds the two-attempt review WIP limit')

    gates = contract['launch_gates']
    documentation_path = gates['canonical_documentation_path']
    documentation_url = gates['canonical_documentation_url']
    runtime_ref = gates['canonical_runtime_ref']
    if (documentation_path is None) != (documentation_url is None):
        raise CohortError(
            'canonical documentation path and URL must be set together'
        )
    expected_fragments = {
        'docker-first-map': '#docker-first-map-no-ros-2-workspace',
        'source-quickstart': '#1-install-and-build-from-source',
    }
    if documentation_path is not None and expected_fragments[
        documentation_path
    ] not in documentation_url:
        raise CohortError(
            'canonical documentation URL does not match the selected path'
        )
    if runtime_ref is not None and documentation_path is None:
        raise CohortError(
            'a canonical runtime identity requires a selected public path'
        )
    if (
        documentation_path == 'docker-first-map'
        and runtime_ref is not None
        and not runtime_ref.startswith('ghcr.io/')
    ):
        raise CohortError(
            'the Docker path requires an immutable GHCR digest identity'
        )
    if (
        documentation_path == 'source-quickstart'
        and runtime_ref is not None
        and runtime_ref != gates['public_revision']
    ):
        raise CohortError(
            'the source path runtime identity must equal the public revision'
        )
    if gates['public_revision_resolvable'] and (
        gates['public_revision'] is None
    ):
        raise CohortError(
            'a resolvable public revision requires an exact commit'
        )

    ready = _ready_gates(gates)
    expected_status = (
        'COPY_READY_NOT_AUTHORIZED'
        if ready else 'WAITING_FOR_PUBLIC_GATES'
    )
    if contract['status'] != expected_status:
        raise CohortError(
            f'status must be {expected_status} for the current launch gates'
        )

    authority = contract['authority']
    if any(authority.values()):
        raise CohortError('the local cohort packet cannot authorize a write')

    return {
        'schema_version': 1,
        'cohort_id': contract['cohort_id'],
        'status': contract['status'],
        'copy_ready': ready,
        'accepted_target': capacity['accepted_target'],
        'initial_attempt_cap': capacity['initial_attempt_cap'],
        'hard_attempt_cap': capacity['hard_attempt_cap'],
        'max_concurrent_attempts': capacity['max_concurrent_attempts'],
        'community_posts_authorized': False,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def render_recruitment(contract: dict[str, Any]) -> str:
    """Render copy-ready public text only after every product gate passes."""
    if not _ready_gates(contract['launch_gates']):
        raise CohortError(
            'recruitment text is blocked until the public revision, '
            'comparable Docker/source rows, canonical path/runtime identity, '
            'and public copy-ready handoff all pass'
        )
    gates = contract['launch_gates']
    capacity = contract['capacity']
    service = contract['service_levels']
    return '\n'.join([
        'Help validate the public lidarslam_ros2 first-map path',
        '',
        'We are inviting a small first cohort of independent ROS 2 users to '
        'run one public first-map path without private maintainer guidance.',
        '',
        f"Public path: {gates['canonical_documentation_url']}",
        f"Exact source revision: {gates['public_revision']}",
        f"Exact product identity: {gates['canonical_runtime_ref']}",
        f"Tracking issue: {gates['tracking_issue_url']}",
        '',
        'You are eligible if you are not a lidarslam_ros2 maintainer and can '
        'follow only the public documentation. Please report both PASS and '
        'FAIL outcomes; a failure is useful product evidence.',
        '',
        'After the attempt, run `lidarslam-map support <session> --first-map` '
        'and use its copy-ready fields. Attach only the reviewed '
        '`first_map_validation_receipt.json` to this form:',
        ISSUE_FORM_URL,
        '',
        'Do not upload a bag, map, trajectory, raw log, parameters, precise '
        'location, or private-place screenshot. No product telemetry is used.',
        '',
        f"We will begin with at most {capacity['initial_attempt_cap']} attempts "
        f"and no more than {capacity['max_concurrent_attempts']} at once. "
        'We aim to acknowledge a report within '
        f"{service['acknowledgement_business_days']} business days and finish "
        'evidence review within '
        f"{service['evidence_review_business_days']} business days.",
        '',
        'Posting this text still requires explicit community-write approval.',
    ])


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract', type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument('--schema', type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument('--json', action='store_true')
    parser.add_argument(
        '--render',
        action='store_true',
        help='Render recruitment text only when every public launch gate passes.',
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        contract = _load_object(args.contract, 'contract')
        schema = _load_object(args.schema, 'schema')
        report = validate_contract(contract, schema)
        if args.render:
            print(render_recruitment(copy.deepcopy(contract)))
            return 0
    except CohortError as exc:
        if args.json:
            print(json.dumps({
                'status': 'COHORT_INVALID_OR_BLOCKED',
                'error': str(exc),
                'remote_mutations_performed': False,
            }, sort_keys=True))
        else:
            print(f'first-map cohort blocked: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(
            f"{report['status']}: target {report['accepted_target']} accepted "
            f"from at most {report['hard_attempt_cap']} assessed attempts"
        )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
