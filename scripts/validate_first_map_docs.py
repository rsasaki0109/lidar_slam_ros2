#!/usr/bin/env python3
"""Validate the beginner first-map documentation against executable help."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'first-map-v1.json'
EXPECTED_ENTRYPOINT_IDS = {'docker-demo', 'own-bag', 'source-demo'}
EXPECTED_PLATFORMS = {('humble', '22.04'), ('jazzy', '24.04')}
EXPECTED_ARTIFACTS = {
    'pointcloud_map/',
    'pointcloud_map/pointcloud_map_metadata.yaml',
    'map_projector_info.yaml',
    'traj_corrected.tum',
    'verify_autoware_map.log',
    'autoware_map_diagnosis.md',
    'autoware_map_diagnosis.json',
    'run_manifest.json',
}


def _load_contract(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'cannot load contract {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise ValueError(f'contract root must be an object: {path}')
    return value


def _document_text(relative_path: str, errors: list[str]) -> str:
    path = REPO_ROOT / relative_path
    try:
        resolved = path.resolve(strict=True)
        resolved.relative_to(REPO_ROOT)
        return resolved.read_text(encoding='utf-8')
    except (OSError, UnicodeError, ValueError) as exc:
        errors.append(f'document is unavailable: {relative_path}: {exc}')
        return ''


def _validate_probe(probe: dict[str, Any], label: str, errors: list[str]) -> None:
    argv = probe.get('argv')
    expected_output = probe.get('expected_output')
    if (
        not isinstance(argv, list)
        or not argv
        or not all(isinstance(item, str) and item for item in argv)
    ):
        errors.append(f'{label}: argv must be a non-empty string array')
        return
    if (
        not isinstance(expected_output, list)
        or not expected_output
        or not all(isinstance(item, str) and item for item in expected_output)
    ):
        errors.append(f'{label}: expected_output must be a non-empty string array')
        return

    try:
        completed = subprocess.run(
            argv,
            cwd=REPO_ROOT,
            check=False,
            capture_output=True,
            text=True,
            timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        errors.append(f'{label}: probe could not run: {exc}')
        return

    output = completed.stdout + completed.stderr
    if completed.returncode != 0:
        errors.append(
            f'{label}: probe exited {completed.returncode}: {" ".join(argv)}'
        )
    for snippet in expected_output:
        if snippet not in output:
            errors.append(f'{label}: help output is missing: {snippet!r}')


def validate(contract: dict[str, Any]) -> list[str]:
    """Return contract violations without mutating the repository."""
    errors: list[str] = []
    if contract.get('schema_version') != 1:
        errors.append('schema_version must be 1')
    if contract.get('contract_id') != 'lidarslam-first-map-v1':
        errors.append('contract_id must be lidarslam-first-map-v1')

    platforms = contract.get('supported_source_platforms')
    actual_platforms: set[tuple[str, str]] = set()
    if isinstance(platforms, list):
        for index, item in enumerate(platforms):
            if not isinstance(item, dict):
                errors.append(
                    f'supported_source_platforms item {index} must be an object'
                )
                continue
            ros_distro = item.get('ros_distro')
            ubuntu = item.get('ubuntu')
            if not isinstance(ros_distro, str) or not isinstance(ubuntu, str):
                errors.append(
                    f'supported_source_platforms item {index} values must be strings'
                )
                continue
            actual_platforms.add((ros_distro, ubuntu))
    else:
        errors.append('supported_source_platforms must be an array')
    if actual_platforms != EXPECTED_PLATFORMS:
        errors.append(
            f'supported_source_platforms must be {sorted(EXPECTED_PLATFORMS)!r}'
        )

    artifacts = contract.get('successful_run_artifacts')
    actual_artifacts = (
        set(artifacts)
        if isinstance(artifacts, list)
        and all(isinstance(item, str) for item in artifacts)
        else set()
    )
    if actual_artifacts != EXPECTED_ARTIFACTS:
        errors.append(
            f'successful_run_artifacts must be {sorted(EXPECTED_ARTIFACTS)!r}'
        )

    entrypoints = contract.get('official_entrypoints')
    if not isinstance(entrypoints, list):
        errors.append('official_entrypoints must be an array')
        entrypoints = []
    ids = {
        item.get('id')
        for item in entrypoints
        if isinstance(item, dict) and isinstance(item.get('id'), str)
    }
    if len(entrypoints) != 3 or ids != EXPECTED_ENTRYPOINT_IDS:
        errors.append(
            'official_entrypoints must contain exactly docker-demo, own-bag, '
            'and source-demo'
        )

    document_cache: dict[str, str] = {}
    for entrypoint in entrypoints:
        if not isinstance(entrypoint, dict):
            errors.append('official_entrypoints items must be objects')
            continue
        entrypoint_id = entrypoint.get('id', '<missing-id>')
        command = entrypoint.get('command')
        documents = entrypoint.get('documentation')
        if not isinstance(command, str) or not command:
            errors.append(f'{entrypoint_id}: command must be a non-empty string')
            continue
        if (
            not isinstance(documents, list)
            or not documents
            or not all(isinstance(item, str) and item for item in documents)
        ):
            errors.append(f'{entrypoint_id}: documentation must be a string array')
            continue

        for relative_path in documents:
            text = document_cache.setdefault(
                relative_path, _document_text(relative_path, errors)
            )
            if command not in text:
                errors.append(
                    f'{entrypoint_id}: {relative_path} is missing exact command: '
                    f'{command}'
                )

        required_snippets = entrypoint.get('required_documentation_snippets', {})
        if not isinstance(required_snippets, dict):
            errors.append(
                f'{entrypoint_id}: required_documentation_snippets must be an object'
            )
            required_snippets = {}
        for relative_path, snippets in required_snippets.items():
            if not isinstance(relative_path, str) or not relative_path:
                errors.append(
                    f'{entrypoint_id}: snippet document paths must be strings'
                )
                continue
            text = document_cache.setdefault(
                relative_path, _document_text(relative_path, errors)
            )
            if not isinstance(snippets, list):
                errors.append(
                    f'{entrypoint_id}: snippets for {relative_path} must be an array'
                )
                continue
            for snippet in snippets:
                if not isinstance(snippet, str) or not snippet:
                    errors.append(
                        f'{entrypoint_id}: snippets for {relative_path} must be strings'
                    )
                elif snippet not in text:
                    errors.append(
                        f'{entrypoint_id}: {relative_path} is missing: {snippet!r}'
                    )

        probes = entrypoint.get('probes', [])
        if not isinstance(probes, list):
            errors.append(f'{entrypoint_id}: probes must be an array')
            continue
        for index, probe in enumerate(probes):
            if not isinstance(probe, dict):
                errors.append(f'{entrypoint_id}: probe {index} must be an object')
                continue
            _validate_probe(probe, f'{entrypoint_id} probe {index}', errors)

    verification_probe = contract.get('verification_probe')
    if isinstance(verification_probe, dict):
        _validate_probe(verification_probe, 'verification probe', errors)
    else:
        errors.append('verification_probe must be an object')

    product_contract = _document_text('docs/product-contract.md', errors)
    getting_started = _document_text('docs/getting-started.md', errors)
    for artifact in sorted(EXPECTED_ARTIFACTS):
        if artifact not in product_contract:
            errors.append(f'docs/product-contract.md is missing artifact: {artifact}')
        if artifact not in getting_started:
            errors.append(f'docs/getting-started.md is missing artifact: {artifact}')

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Validate first-map docs, commands, and success artifacts.'
    )
    parser.add_argument(
        '--contract',
        type=Path,
        default=DEFAULT_CONTRACT,
        help='Contract JSON path (default: docs/contracts/first-map-v1.json).',
    )
    args = parser.parse_args()

    try:
        contract = _load_contract(args.contract)
    except ValueError as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        return 1
    errors = validate(contract)
    if errors:
        print(f'FAIL: first-map documentation contract ({len(errors)} violations)')
        for error in errors:
            print(f'- {error}')
        return 1

    print('PASS: first-map documentation contract')
    print('- official entrypoints: 3')
    print('- supported source platforms: Humble/Jammy, Jazzy/Noble')
    print(f'- successful-run artifacts: {len(EXPECTED_ARTIFACTS)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
