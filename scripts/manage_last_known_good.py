#!/usr/bin/env python3
"""Validate release-image evidence and produce digest-pinned rollback plans."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shlex
import sys
import tempfile
from typing import Any


REPOSITORY = 'ghcr.io/rsasaki0109/lidar_slam_ros2'
DISTROS = ('humble', 'jazzy')
DIGEST_RE = re.compile(r'^sha256:[0-9a-f]{64}$')
COMMIT_RE = re.compile(r'^[0-9a-f]{40}$')
VERSION_RE = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+$')


def _load(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'cannot load {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise ValueError(f'JSON root must be an object: {path}')
    return value


def _write_atomic(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{path.name}.',
        suffix='.tmp',
        dir=path.parent,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            json.dump(value, stream, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def validate_evidence(
    value: dict[str, Any],
    expected_distro: str | None = None,
) -> list[str]:
    """Validate one release-image evidence record."""
    errors: list[str] = []
    distro = value.get('ros_distro')
    if value.get('schema_version') != 1:
        errors.append('schema_version must be 1')
    if value.get('status') != 'PASS':
        errors.append('status must be PASS')
    if distro not in DISTROS:
        errors.append('ros_distro must be humble or jazzy')
    if expected_distro is not None and distro != expected_distro:
        errors.append(f'ros_distro must be {expected_distro}')
    if value.get('platform') != 'linux/amd64':
        errors.append('platform must be linux/amd64')

    version = value.get('product_version')
    commit = value.get('git_commit')
    digest = value.get('digest')
    tag = value.get('tag')
    cli_version = value.get('cli_version')
    if not isinstance(version, str) or VERSION_RE.fullmatch(version) is None:
        errors.append('product_version must be numeric SemVer')
    if not isinstance(commit, str) or COMMIT_RE.fullmatch(commit) is None:
        errors.append('git_commit must be a full lowercase SHA-1')
    if not isinstance(digest, str) or DIGEST_RE.fullmatch(digest) is None:
        errors.append('digest must be sha256 followed by 64 lowercase hex digits')

    expected_tag = (
        f'{REPOSITORY}:v{version}-{distro}'
        if isinstance(version, str) and isinstance(distro, str)
        else None
    )
    if tag != expected_tag:
        errors.append(f'tag must be {expected_tag}')
    if cli_version != f'lidarslam_ros2 {version}':
        errors.append(f'cli_version must be lidarslam_ros2 {version}')
    return errors


def validate_ledger(
    value: dict[str, Any],
    require_assigned: bool = False,
) -> list[str]:
    """Validate cross-distro last-known-good identity."""
    errors: list[str] = []
    if value.get('schema_version') != 1:
        errors.append('schema_version must be 1')
    if value.get('repository') != REPOSITORY:
        errors.append(f'repository must be {REPOSITORY}')
    if not isinstance(value.get('reason'), str) or not value['reason'].strip():
        errors.append('reason must be a non-empty string')
    images = value.get('images')
    if not isinstance(images, dict) or set(images) != set(DISTROS):
        errors.append('images must contain exactly humble and jazzy')
        images = {distro: None for distro in DISTROS}

    status = value.get('status')
    if status == 'unassigned':
        if any(images.get(distro) is not None for distro in DISTROS):
            errors.append('unassigned ledger images must both be null')
        if require_assigned:
            errors.append('last-known-good is unassigned')
        return errors
    if status != 'assigned':
        errors.append('status must be unassigned or assigned')
        return errors

    version = value.get('product_version')
    commit = value.get('git_commit')
    release_tag = value.get('release_tag')
    promoted_at = value.get('promoted_at')
    if not isinstance(version, str) or VERSION_RE.fullmatch(version) is None:
        errors.append('assigned product_version must be numeric SemVer')
    if release_tag != f'v{version}':
        errors.append(f'release_tag must be v{version}')
    if not isinstance(commit, str) or COMMIT_RE.fullmatch(commit) is None:
        errors.append('assigned git_commit must be a full lowercase SHA-1')
    try:
        if not isinstance(promoted_at, str):
            raise ValueError
        parsed_time = datetime.fromisoformat(promoted_at.replace('Z', '+00:00'))
        if parsed_time.tzinfo is None:
            raise ValueError
    except ValueError:
        errors.append('promoted_at must be an ISO-8601 timestamp with timezone')

    for distro in DISTROS:
        evidence = images.get(distro)
        if not isinstance(evidence, dict):
            errors.append(f'images.{distro} must be an evidence object')
            continue
        errors.extend(
            f'images.{distro}: {error}'
            for error in validate_evidence(evidence, distro)
        )
        if evidence.get('product_version') != version:
            errors.append(f'images.{distro}: product_version differs from ledger')
        if evidence.get('git_commit') != commit:
            errors.append(f'images.{distro}: git_commit differs from ledger')
    return errors


def _print_errors(errors: list[str]) -> int:
    for error in errors:
        print(f'error: {error}', file=sys.stderr)
    return 1


def _promote(args: argparse.Namespace) -> int:
    evidence = {
        distro: _load(getattr(args, distro))
        for distro in DISTROS
    }
    errors = [
        f'{distro}: {error}'
        for distro in DISTROS
        for error in validate_evidence(evidence[distro], distro)
    ]
    if errors:
        return _print_errors(errors)

    versions = {item.get('product_version') for item in evidence.values()}
    commits = {item.get('git_commit') for item in evidence.values()}
    if len(versions) != 1:
        errors.append('Humble and Jazzy product versions differ')
    if len(commits) != 1:
        errors.append('Humble and Jazzy git commits differ')
    if errors:
        return _print_errors(errors)

    if args.output.exists():
        current = _load(args.output)
        if current.get('status') == 'assigned':
            return _print_errors([
                f'refusing to replace assigned ledger: {args.output}',
            ])

    version = next(iter(versions))
    commit = next(iter(commits))
    promoted_at = args.promoted_at or (
        datetime.now(timezone.utc).isoformat(timespec='seconds')
        .replace('+00:00', 'Z')
    )
    ledger = {
        'schema_version': 1,
        'status': 'assigned',
        'repository': REPOSITORY,
        'release_tag': f'v{version}',
        'product_version': version,
        'git_commit': commit,
        'promoted_at': promoted_at,
        'reason': args.reason,
        'images': evidence,
    }
    errors = validate_ledger(ledger, require_assigned=True)
    if errors:
        return _print_errors(errors)
    _write_atomic(args.output, ledger)
    print(f'PASS: promoted v{version} as last-known-good')
    print(f'- output: {args.output}')
    print(f'- git commit: {commit}')
    return 0


def _verify(args: argparse.Namespace) -> int:
    ledger = _load(args.ledger)
    errors = validate_ledger(ledger, require_assigned=args.require_assigned)
    if errors:
        return _print_errors(errors)
    print(f'PASS: last-known-good ledger is {ledger["status"]}')
    if ledger['status'] == 'assigned':
        print(f'- release: {ledger["release_tag"]}')
        print(f'- git commit: {ledger["git_commit"]}')
    else:
        print(f'- reason: {ledger["reason"]}')
    return 0


def _validate_evidence_command(args: argparse.Namespace) -> int:
    evidence = _load(args.evidence)
    errors = validate_evidence(evidence, args.ros_distro)
    if errors:
        return _print_errors(errors)
    print(f'PASS: release-image evidence is valid for {args.ros_distro}')
    print(f'- tag: {evidence["tag"]}')
    print(f'- digest: {evidence["digest"]}')
    return 0


def _plan(args: argparse.Namespace) -> int:
    ledger = _load(args.ledger)
    errors = validate_ledger(ledger, require_assigned=True)
    if errors:
        return _print_errors(errors)
    evidence = ledger['images'][args.ros_distro]
    image_ref = f'{ledger["repository"]}@{evidence["digest"]}'
    quoted_ref = shlex.quote(image_ref)
    quoted_repository = shlex.quote(args.github_repository)
    print(f'# last-known-good {ledger["release_tag"]} / {args.ros_distro}')
    print(f'export LIDARSLAM_ROLLBACK_IMAGE={quoted_ref}')
    print('docker pull "$LIDARSLAM_ROLLBACK_IMAGE"')
    print(
        'gh attestation verify '
        '"oci://$LIDARSLAM_ROLLBACK_IMAGE" '
        f'-R {quoted_repository}'
    )
    print(
        'docker run --rm "$LIDARSLAM_ROLLBACK_IMAGE" '
        'lidarslam-map --version'
    )
    print('# Deploy only the digest above; do not retag a moving convenience tag.')
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Manage digest-pinned lidarslam release rollback identity.'
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    evidence = subparsers.add_parser(
        'validate-evidence',
        help='Validate one release-image evidence file.',
    )
    evidence.add_argument('evidence', type=Path)
    evidence.add_argument('--ros-distro', choices=DISTROS, required=True)
    evidence.set_defaults(handler=_validate_evidence_command)

    verify = subparsers.add_parser('verify', help='Validate an LKG ledger.')
    verify.add_argument('ledger', type=Path)
    verify.add_argument('--require-assigned', action='store_true')
    verify.set_defaults(handler=_verify)

    promote = subparsers.add_parser(
        'promote',
        help='Create an assigned ledger from two accepted image records.',
    )
    promote.add_argument('--humble', type=Path, required=True)
    promote.add_argument('--jazzy', type=Path, required=True)
    promote.add_argument('--output', type=Path, required=True)
    promote.add_argument('--reason', required=True)
    promote.add_argument('--promoted-at')
    promote.set_defaults(handler=_promote)

    plan = subparsers.add_parser(
        'plan',
        help='Print a digest-pinned rollback verification plan.',
    )
    plan.add_argument('ledger', type=Path)
    plan.add_argument('--ros-distro', choices=DISTROS, required=True)
    plan.add_argument(
        '--github-repository',
        default='rsasaki0109/lidar_slam_ros2',
    )
    plan.set_defaults(handler=_plan)
    return parser


def main() -> int:
    args = _parser().parse_args()
    try:
        return args.handler(args)
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1
    except OSError as exc:
        print(f'error: filesystem operation failed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
