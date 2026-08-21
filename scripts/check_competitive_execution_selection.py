#!/usr/bin/env python3
"""Fail-closed preflight for the competitive benchmark execution identity.

This checker only validates the preregistration receipt.  It does not build
containers, download datasets, inspect ground truth, or run a benchmark.
Missing values are reported as ``INCOMPLETE``; malformed or mismatched values
are ``INVALID``.  A pending receipt can therefore be committed before the
environment is built without being mistaken for a runnable comparison.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any

import yaml

try:
    from scripts.competitive_identity_hash import (
        PROFILE_CANONICAL_HASH_KIND, canonical_profile_sha256)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from competitive_identity_hash import (  # type: ignore[no-redef]
        PROFILE_CANONICAL_HASH_KIND, canonical_profile_sha256)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
DEFAULT_RECEIPT = ROOT / (
    'configs/slam_benchmark_profiles/'
    'competitive_execution_selection_2026-08.yaml')
SHA256_RE = re.compile(r'^[0-9a-fA-F]{64}$')
COMMIT_RE = re.compile(r'^[0-9a-fA-F]{40}$')
CONTAINER_DIGEST_RE = re.compile(r'^sha256:[0-9a-fA-F]{64}$')
THREAD_KEYS = (
    'cpu_affinity', 'max_threads', 'omp_num_threads',
    'openblas_num_threads', 'mkl_num_threads', 'tbb_num_threads',
    'accelerator_policy')
SYSTEMS = ('ours', 'glim', 'fast_livo2')


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path: Path) -> str:
    """Hash relative file names and file bytes using the repository contract."""
    digest = hashlib.sha256()
    for candidate in sorted(item for item in path.rglob('*') if item.is_file()):
        digest.update(candidate.relative_to(path).as_posix().encode())
        digest.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def _resolve(root: Path, value: Any) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    path = Path(value)
    return path if path.is_absolute() else root / path


def _hash_path(root: Path, value: Any) -> str | None:
    path = _resolve(root, value)
    if path is None or not path.exists():
        return None
    return sha256_tree(path) if path.is_dir() else sha256_file(path)


def _is_sha(value: Any) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def _nonempty(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _canonical_hash(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(',', ':'),
                         ensure_ascii=True).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _add_missing(incomplete: list[str], path: str) -> None:
    incomplete.append(path)


def _check_hash_file(root: Path, value: Any, label: str,
                     errors: list[str], incomplete: list[str]) -> bool:
    path = _resolve(root, value.get('path') if isinstance(value, dict) else None)
    expected = value.get('sha256') if isinstance(value, dict) else None
    if path is None or expected is None:
        _add_missing(incomplete, f'{label}.path_or_sha256')
        return False
    if not _is_sha(expected):
        errors.append(f'{label}.sha256 must be a 64-hex SHA-256')
        return False
    if not path.is_file():
        errors.append(f'{label}.path is not an existing file: {path}')
        return False
    actual = sha256_file(path)
    if actual.lower() != expected.lower():
        errors.append(f'{label}.sha256 does not match {path}')
        return False
    return True


def _check_config(root: Path, config: Any, label: str,
                  errors: list[str], incomplete: list[str]) -> bool:
    if not isinstance(config, dict):
        errors.append(f'{label} must be a mapping')
        return False
    path = _resolve(root, config.get('path'))
    expected = config.get('sha256')
    if path is None or expected is None:
        _add_missing(incomplete, f'{label}.path_or_sha256')
        return False
    if not _is_sha(expected):
        errors.append(f'{label}.sha256 must be a 64-hex SHA-256')
        return False
    if not path.exists():
        errors.append(f'{label}.path does not exist: {path}')
        return False
    actual = sha256_tree(path) if path.is_dir() else sha256_file(path)
    if actual.lower() != expected.lower():
        errors.append(f'{label}.sha256 does not match {path}')
        return False
    return True


def _check_thread_policy(value: Any, label: str, errors: list[str],
                         incomplete: list[str]) -> bool:
    if not isinstance(value, dict):
        _add_missing(incomplete, f'{label} mapping')
        return False
    valid = True
    for key in THREAD_KEYS:
        if key not in value or value[key] is None:
            _add_missing(incomplete, f'{label}.{key}')
            valid = False
            continue
        item = value[key]
        if key == 'cpu_affinity':
            if (not isinstance(item, list) or not item or
                    any(isinstance(cpu, bool) or not isinstance(cpu, int)
                        or cpu < 0 for cpu in item)):
                errors.append(f'{label}.{key} must be a non-empty integer list')
                valid = False
        elif key == 'accelerator_policy':
            if not _nonempty(item):
                errors.append(f'{label}.{key} must be a non-empty string')
                valid = False
        elif isinstance(item, bool) or not isinstance(item, int) or item <= 0:
            errors.append(f'{label}.{key} must be a positive integer')
            valid = False
    return valid


def evaluate(receipt: dict[str, Any], profile: dict[str, Any],
             root: Path = ROOT) -> dict[str, Any]:
    """Return a JSON/YAML-safe preflight result without mutating inputs."""
    errors: list[str] = []
    incomplete: list[str] = []
    checks: dict[str, dict[str, Any]] = {}

    def check(name: str, passed: bool, evidence: Any) -> None:
        checks[name] = {'pass': bool(passed), 'evidence': evidence}

    contract = profile.get('competitive_slam_profile', profile)
    policy = contract.get('evidence_gate_v2', {})
    receipt_path_value = policy.get('execution_selection_receipt_path')
    receipt_expected_sha = policy.get('execution_selection_receipt_sha256')
    receipt_path = _resolve(root, receipt_path_value)
    receipt_path_ok = receipt_path is not None and receipt_path.is_file()
    if receipt_expected_sha is None:
        _add_missing(incomplete, 'profile.execution_selection_receipt_sha256')
    elif not _is_sha(receipt_expected_sha):
        errors.append('profile.execution_selection_receipt_sha256 must be 64-hex')
    if not receipt_path_ok:
        _add_missing(incomplete, 'profile.execution_selection_receipt_path')
    actual_receipt_sha = sha256_file(receipt_path) if receipt_path_ok else None
    if (_is_sha(receipt_expected_sha) and actual_receipt_sha is not None and
            actual_receipt_sha.lower() != receipt_expected_sha.lower()):
        errors.append('execution selection receipt SHA does not match profile')
    receipt_path_check = (receipt_path_ok and _is_sha(receipt_expected_sha) and
                          actual_receipt_sha is not None and
                          actual_receipt_sha.lower() == receipt_expected_sha.lower())
    check('receipt_path_and_sha256', receipt_path_check, {
        'path': receipt_path_value,
        'expected_sha256': receipt_expected_sha,
        'actual_sha256': actual_receipt_sha,
    })

    if not isinstance(receipt, dict):
        errors.append('execution selection receipt must be a mapping')
        receipt = {}
    if receipt.get('schema_version') != 1:
        errors.append('execution selection receipt schema_version must be 1')
    if receipt.get('receipt_kind') != 'competitive_execution_selection':
        errors.append('execution selection receipt_kind is not competitive_execution_selection')
    status = receipt.get('status')
    if status not in {'ready', 'frozen'}:
        if status is None or 'pending' in str(status):
            _add_missing(incomplete, f'receipt.status is not ready: {status!r}')
        else:
            errors.append(f'receipt.status must be ready/frozen: {status!r}')
    check('receipt_status_ready', status in {'ready', 'frozen'}, {
        'status': status,
        'ready_statuses': ['frozen', 'ready'],
    })

    release = receipt.get('release')
    if release != 'Release':
        if release is None:
            _add_missing(incomplete, 'receipt.release')
        else:
            errors.append('receipt.release must be exactly Release')
    check('release_exact', release == 'Release', {'release': release})

    common = receipt.get('common_identity')
    if not isinstance(common, dict):
        errors.append('receipt.common_identity must be a mapping')
        common = {}
    try:
        computed_profile_sha = canonical_profile_sha256(profile)
    except ValueError as exc:
        errors.append(str(exc))
        computed_profile_sha = None
    declared_profile_sha = common.get('profile_sha256')
    declared_profile_kind = common.get('profile_sha256_kind')
    profile_hash_ok = True
    if declared_profile_sha is None:
        _add_missing(incomplete, 'common_identity.profile_sha256')
        profile_hash_ok = False
    elif not _is_sha(declared_profile_sha):
        errors.append('common_identity.profile_sha256 must be 64-hex')
        profile_hash_ok = False
    if declared_profile_kind is None:
        _add_missing(incomplete, 'common_identity.profile_sha256_kind')
        profile_hash_ok = False
    elif declared_profile_kind != PROFILE_CANONICAL_HASH_KIND:
        errors.append(
            'common_identity.profile_sha256_kind must be '
            f'{PROFILE_CANONICAL_HASH_KIND}')
        profile_hash_ok = False
    if (profile_hash_ok and computed_profile_sha is not None and
            declared_profile_sha.lower() != computed_profile_sha):
        errors.append('common_identity.profile_sha256 does not match canonical profile')
        profile_hash_ok = False
    check('profile_canonical_hash', profile_hash_ok and computed_profile_sha is not None, {
        'hash_kind': declared_profile_kind,
        'expected_hash_kind': PROFILE_CANONICAL_HASH_KIND,
        'declared_sha256': declared_profile_sha,
        'computed_sha256': computed_profile_sha,
        'excluded_path': '.'.join(
            ('competitive_slam_profile', 'evidence_gate_v2',
             'execution_selection_receipt_sha256')),
    })
    scorer = common.get('scorer')
    scorer_ok = isinstance(scorer, dict)
    if not scorer_ok:
        errors.append('receipt.common_identity.scorer must be a mapping')
        scorer = {}
    scorer_files = scorer.get('files') or {}
    scorer_files_ok = isinstance(scorer_files, dict)
    if not scorer_files_ok:
        errors.append('common_identity.scorer.files must be a mapping')
        scorer_files = {}
    scorer_payload: list[dict[str, Any]] = []
    for name, item in sorted(scorer_files.items()):
        file_ok = _check_hash_file(root, item, f'scorer.{name}', errors,
                                   incomplete)
        scorer_files_ok = file_ok and scorer_files_ok
        if file_ok and isinstance(item, dict):
            actual = _hash_path(root, item.get('path'))
            scorer_payload.append({
                'name': name,
                'path': item.get('path'),
                'sha256': actual,
                'policy': item.get('policy'),
            })
    computed_scorer_fingerprint = (
        _canonical_hash(scorer_payload) if scorer_files_ok else None)
    scorer_fingerprint = scorer.get('canonical_fingerprint')
    if scorer_fingerprint is None:
        _add_missing(incomplete, 'common_identity.scorer.canonical_fingerprint')
    elif not _is_sha(scorer_fingerprint):
        errors.append('common_identity.scorer.canonical_fingerprint must be 64-hex')
    if (computed_scorer_fingerprint is not None and
            isinstance(scorer_fingerprint, str) and
            computed_scorer_fingerprint != scorer_fingerprint.lower()):
        errors.append('common_identity.scorer.canonical_fingerprint does not match '
                      'the canonical scorer file payload')
    check('scorer_files_and_fingerprint', scorer_ok and scorer_files_ok and
          _is_sha(scorer_fingerprint) and
          computed_scorer_fingerprint == scorer_fingerprint.lower(), {
              'canonical_fingerprint': scorer_fingerprint,
              'computed_fingerprint': computed_scorer_fingerprint,
              'canonical_payload': scorer_payload,
              'file_count': len(scorer_files),
          })

    machine = common.get('machine_fingerprint')
    machine_ok = _check_hash_file(root, machine, 'machine_fingerprint', errors,
                                  incomplete)
    machine_status = machine.get('status') if isinstance(machine, dict) else None
    if machine_status not in {'ready', 'frozen'}:
        if machine_status is None or str(machine_status).startswith('pending') or \
                machine_status == 'available_but_refresh_required_before_run':
            _add_missing(incomplete, 'machine_fingerprint.status is not refreshed')
        else:
            errors.append('machine_fingerprint.status must be ready or frozen')
        machine_ok = False
    check('machine_fingerprint', machine_ok, {
        'path': machine.get('path') if isinstance(machine, dict) else None,
        'status': machine_status,
    })

    thread = common.get('thread_policy')
    common_thread_ok = _check_thread_policy(thread, 'common_identity.thread_policy',
                                            errors, incomplete)
    thread_status = thread.get('status') if isinstance(thread, dict) else None
    if thread_status not in {'ready', 'frozen'}:
        if thread_status is None or str(thread_status).startswith('pending'):
            _add_missing(incomplete, 'common_identity.thread_policy.status is not ready')
        else:
            errors.append('common_identity.thread_policy.status must be ready or frozen')
        common_thread_ok = False
    thread_policy_canonical_hash = (
        _canonical_hash({key: thread.get(key) for key in THREAD_KEYS})
        if common_thread_ok and isinstance(thread, dict) else None)
    check('thread_policy_complete', common_thread_ok, {
        'required_keys': list(THREAD_KEYS),
        'status': thread_status,
        'canonical_sha256': thread_policy_canonical_hash,
        'policy': thread,
    })

    systems = receipt.get('systems')
    if not isinstance(systems, dict):
        errors.append('receipt.systems must be a mapping')
        systems = {}
    systems_ok = True
    per_system_results: dict[str, bool] = {}
    for system in SYSTEMS:
        per_system_ok = True
        item = systems.get(system)
        if not isinstance(item, dict):
            _add_missing(incomplete, f'systems.{system}')
            per_system_ok = False
            per_system_results[system] = per_system_ok
            continue
        repository = item.get('repository') or {}
        revision = repository.get('revision')
        if not _nonempty(repository.get('url')):
            _add_missing(incomplete, f'{system}.repository.url')
            per_system_ok = False
        if revision is None:
            _add_missing(incomplete, f'{system}.repository.revision')
            per_system_ok = False
        elif not isinstance(revision, str) or not COMMIT_RE.fullmatch(revision):
            errors.append(f'{system}.repository.revision must be a 40-hex commit')
            per_system_ok = False
        revision_status = repository.get('revision_status')
        if revision_status not in {'ready', 'frozen', 'pinned'}:
            if revision_status is None or 'pending' in str(revision_status):
                _add_missing(incomplete, f'{system}.repository.revision_status is not ready')
            else:
                errors.append(f'{system}.repository.revision_status is not ready/frozen/pinned')
            per_system_ok = False
        worktree_dirty = repository.get('worktree_dirty')
        if worktree_dirty is None:
            _add_missing(incomplete, f'{system}.repository.worktree_dirty')
            per_system_ok = False
        elif worktree_dirty is not False:
            _add_missing(incomplete, f'{system}.repository.worktree must be clean')
            per_system_ok = False
        for clean_key in ('tracked_diff_sha256', 'untracked_content_sha256',
                          'clean_provenance_sha256'):
            clean_value = repository.get(clean_key)
            if worktree_dirty is False and not _is_sha(clean_value):
                _add_missing(incomplete, f'{system}.repository.{clean_key}')
                per_system_ok = False
        for index, config in enumerate(item.get('configs', [])):
            per_system_ok = (_check_config(root, config, f'{system}.configs[{index}]',
                                           errors, incomplete) and per_system_ok)
        runner = item.get('runner')
        per_system_ok = (_check_hash_file(root, runner, f'{system}.runner', errors,
                                          incomplete) and per_system_ok)
        container = item.get('container') or {}
        container_status = container.get('status')
        if container_status not in {'ready', 'frozen'}:
            if container_status is None or 'pending' in str(container_status):
                _add_missing(incomplete, f'{system}.container.status is not ready')
            else:
                errors.append(f'{system}.container.status must be ready or frozen')
            per_system_ok = False
        tag = container.get('image_tag')
        digest = container.get('image_digest')
        if tag is None or digest is None:
            _add_missing(incomplete, f'{system}.container.image_tag_or_digest')
            per_system_ok = False
        else:
            if not _nonempty(tag):
                errors.append(f'{system}.container.image_tag must be non-empty')
                per_system_ok = False
            if not isinstance(digest, str) or not CONTAINER_DIGEST_RE.fullmatch(digest):
                errors.append(f'{system}.container.image_digest must be sha256:<64hex>')
                per_system_ok = False
        toolchain = item.get('toolchain') or {}
        toolchain_status = toolchain.get('status')
        if toolchain_status not in {'ready', 'frozen'}:
            if toolchain_status is None or 'pending' in str(toolchain_status):
                _add_missing(incomplete, f'{system}.toolchain.status is not ready')
            else:
                errors.append(f'{system}.toolchain.status must be ready or frozen')
            per_system_ok = False
        fingerprint = toolchain.get('fingerprint')
        if fingerprint is None:
            _add_missing(incomplete, f'{system}.toolchain.fingerprint')
            per_system_ok = False
        elif not _is_sha(fingerprint):
            errors.append(f'{system}.toolchain.fingerprint must be 64-hex')
            per_system_ok = False
        if item.get('release') != 'Release':
            if item.get('release') is None:
                _add_missing(incomplete, f'{system}.release')
            else:
                errors.append(f'{system}.release must be exactly Release')
            per_system_ok = False
        ref = item.get('thread_policy_ref')
        if not _nonempty(ref):
            _add_missing(incomplete, f'{system}.thread_policy_ref')
            per_system_ok = False
        per_system_results[system] = per_system_ok
        systems_ok = systems_ok and per_system_ok
        check(f'system_{system}', per_system_ok, {
            'revision': revision,
            'revision_status': revision_status,
            'worktree_dirty': worktree_dirty,
            'container_digest': digest,
            'container_status': container_status,
            'toolchain_fingerprint': fingerprint,
            'toolchain_status': toolchain_status,
        })
    check('all_systems_pinned_and_resolved', systems_ok, {
        'systems': list(SYSTEMS),
        'per_system': per_system_results,
        'expected_thread_policy_keys': list(THREAD_KEYS),
    })

    status = 'INVALID' if errors else ('INCOMPLETE' if incomplete else 'PASS')
    return {
        'schema_version': 1,
        'receipt_kind': 'competitive_execution_preflight_result',
        'status': status,
        'pass': status == 'PASS',
        'errors': errors + incomplete,
        'checks': checks,
        'receipt_status': receipt.get('status'),
        'canonical_scorer_fingerprint': computed_scorer_fingerprint,
        'thread_policy_canonical_sha256': thread_policy_canonical_hash,
        'profile_execution_selection_receipt': {
            'path': receipt_path_value,
            'expected_sha256': receipt_expected_sha,
            'actual_sha256': actual_receipt_sha,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--receipt', type=Path, default=DEFAULT_RECEIPT)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--yaml-output', type=Path)
    args = parser.parse_args()
    receipt = yaml.safe_load(args.receipt.read_text(encoding='utf-8'))
    profile = yaml.safe_load(args.profile.read_text(encoding='utf-8'))
    result = evaluate(receipt, profile)
    profile_file_sha256 = sha256_file(args.profile)
    result['identity'] = {
        'profile_sha256': canonical_profile_sha256(profile),
        'profile_sha256_kind': PROFILE_CANONICAL_HASH_KIND,
        'profile_file_sha256': profile_file_sha256,
        'execution_receipt_sha256': result['profile_execution_selection_receipt'].get(
            'actual_sha256'),
        'canonical_scorer_fingerprint': result.get(
            'canonical_scorer_fingerprint'),
        'thread_policy_canonical_sha256': result.get(
            'thread_policy_canonical_sha256'),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n',
                           encoding='utf-8')
    yaml_output = args.yaml_output
    if yaml_output is not None:
        yaml_output.parent.mkdir(parents=True, exist_ok=True)
        yaml_output.write_text(yaml.safe_dump(result, sort_keys=True),
                               encoding='utf-8')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError,
            json.JSONDecodeError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
