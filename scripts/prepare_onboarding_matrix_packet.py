#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Prepare a path-free, exact-identity G0 observer packet.

The packet is a local plan, not onboarding evidence. It validates that the
source revision and both Docker image digests use one product version. Docker
identity is either a published release or one retained candidate-image set;
the two modes never invent or reuse each other's tags and preflights. The tool
performs no network request, trial, cleanup, or GitHub/community mutation.
"""

from __future__ import annotations

import argparse
import json
import re
import shlex
import sys
from pathlib import Path
from typing import Any

from audit_candidate_image_set import (
    audit_candidate_set,
    sha256_file,
)

import jsonschema

from product_schema import load_json_object


REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
DATASET_ID = 'mid360-public-zenodo-14841855'
DISTROS = ('humble', 'jazzy')
OS_FAMILY = {'humble': 'ubuntu-22.04', 'jazzy': 'ubuntu-24.04'}
VERSION_RE = re.compile(
    r'^[0-9]+\.[0-9]+\.[0-9]+(?:[-+][A-Za-z0-9.-]+)?$'
)
COMMIT_RE = re.compile(r'^[0-9a-f]{40}$')
DIGEST_RE = re.compile(r'^sha256:[0-9a-f]{64}$')
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/onboarding-matrix-observer-packet-v2.schema.json'
)
CANDIDATE_SET_PLACEHOLDER = '<CANDIDATE_IMAGE_SET_JSON>'


class PacketError(ValueError):
    """The supplied identity inputs cannot produce a safe packet."""


def _validate_identity(
    product_version: str,
    source_commit: str,
    docker_digests: dict[str, str],
) -> None:
    if not VERSION_RE.fullmatch(product_version):
        raise PacketError(
            'product_version must be a semantic version, optionally with a '
            'pre-release or build suffix'
        )
    if not COMMIT_RE.fullmatch(source_commit):
        raise PacketError(
            'source_commit must be one lowercase 40-character hexadecimal SHA'
        )
    if set(docker_digests) != set(DISTROS):
        raise PacketError('one Docker digest is required for Humble and Jazzy')
    for distro, digest in docker_digests.items():
        if not DIGEST_RE.fullmatch(digest):
            raise PacketError(
                f'{distro} Docker digest must be sha256 plus 64 lowercase '
                'hexadecimal characters'
            )
    if docker_digests['humble'] == docker_digests['jazzy']:
        raise PacketError(
            'Humble and Jazzy image digests must differ; a copied digest '
            'would make the row identity ambiguous'
        )


def _command(parts: list[str]) -> str:
    return shlex.join(parts)


def _required_measurements() -> list[str]:
    return [
        'input.download_bytes',
        'measurements.workflow_download_bytes',
        'measurements.wall_time_sec',
        'measurements.active_operator_time_sec',
        'measurements.command_count',
        'measurements.peak_disk_bytes',
        'measurements.output_bytes',
    ]


def _docker_row(
    product_version: str,
    distro: str,
    digest: str,
    *,
    candidate_set: dict[str, Any] | None = None,
    candidate_set_sha256: str | None = None,
) -> dict[str, Any]:
    image_tag: str | None
    immutable_ref: str
    if candidate_set is None:
        image_tag = f'ghcr.io/{REPOSITORY}:v{product_version}-{distro}'
        immutable_ref = f'{image_tag}@{digest}'
        preflight_command = _command([
            'python3',
            'scripts/check_published_release.py',
            '--version',
            product_version,
            '--json',
        ])
        image_arguments = ['--image-tag', image_tag]
        candidate_arguments: list[str] = []
    else:
        if candidate_set_sha256 is None:
            raise PacketError('candidate packet requires the exact set hash')
        image_tag = None
        by_distro = {
            image['ros_distro']: image
            for image in candidate_set['images']
        }
        immutable_ref = by_distro[distro]['immutable_ref']
        preflight_command = _command([
            'python3',
            'scripts/audit_candidate_image_set.py',
            '--candidate-image-set',
            CANDIDATE_SET_PLACEHOLDER,
            '--remote',
            '--json',
        ])
        image_arguments = ['--candidate-image-ref', immutable_ref]
        candidate_arguments = [
            '--candidate-image-set-sha256',
            candidate_set_sha256,
            '--candidate-source-pr',
            str(candidate_set['source_pr']),
            '--candidate-source-commit',
            candidate_set['source_commit'],
            '--candidate-workflow-run-url',
            candidate_set['workflow_run_url'],
        ]
    return {
        'row_id': f'docker-{distro}',
        'route': 'docker',
        'ros_distro': distro,
        'os_family': OS_FAMILY[distro],
        'product_version': product_version,
        'identity': {
            'kind': 'image-digest',
            'value': digest,
            'tag': image_tag,
            'immutable_ref': immutable_ref,
        },
        'preflight_command': preflight_command,
        'observer_command': _command([
            'python3',
            'scripts/run_docker_onboarding_probe.py',
            '--trial-id',
            f'g0-docker-{distro}-<UTC_DATE>-a',
            '--ros-distro',
            distro,
            *image_arguments,
            '--image-digest',
            digest,
            '--product-version',
            product_version,
            *candidate_arguments,
            '--record',
            '<TRIAL_RECORD_OUTSIDE_CHECKOUT>',
            '--disk-scope',
            '/',
            '--acknowledge-dedicated-filesystem',
            '--prompt-human-measurements',
            '--allow-privileged-container-host',
        ]),
        'required_measurements': _required_measurements(),
    }


def _source_row(
    product_version: str,
    distro: str,
    commit: str,
) -> dict[str, Any]:
    return {
        'row_id': f'source-{distro}',
        'route': 'source',
        'ros_distro': distro,
        'os_family': OS_FAMILY[distro],
        'product_version': product_version,
        'identity': {
            'kind': 'git-commit',
            'value': commit,
            'tag': None,
            'immutable_ref': None,
        },
        'preflight_command': _command([
            'python3',
            'scripts/run_source_onboarding_probe.py',
            '--public-preflight',
            '--source-commit',
            commit,
            '--product-version',
            product_version,
        ]),
        'observer_command': _command([
            'python3',
            'scripts/run_source_onboarding_probe.py',
            '--trial-id',
            f'g0-source-{distro}-<UTC_DATE>-a',
            '--ros-distro',
            distro,
            '--source-commit',
            commit,
            '--product-version',
            product_version,
            '--trial-root',
            '<TRIAL_ROOT_OUTSIDE_CHECKOUT>',
            '--observer-parent',
            '<OBSERVER_PARENT_OUTSIDE_CHECKOUT>',
            '--disk-scope',
            '/',
            '--record',
            '<TRIAL_RECORD_OUTSIDE_CHECKOUT>',
            '--prompt-human-measurements',
            '--acknowledge-disposable-host',
            '--acknowledge-isolated-network',
        ]),
        'required_measurements': _required_measurements(),
    }


def _validate_packet(packet: dict[str, Any]) -> None:
    schema_path = (
        Path(__file__).resolve().parents[1]
        / 'docs'
        / 'schemas'
        / 'onboarding-matrix-observer-packet-v2.schema.json'
    )
    try:
        schema = json.loads(schema_path.read_text(encoding='utf-8'))
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(
            schema,
            format_checker=jsonschema.FormatChecker(),
        ).validate(packet)
    except (OSError, json.JSONDecodeError, jsonschema.SchemaError) as exc:
        raise PacketError(f'packet schema cannot be loaded: {exc}') from exc
    except jsonschema.ValidationError as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise PacketError(
            f'packet schema failed at {location or "<root>"}: {exc.message}'
        ) from exc


def _build_packet(
    product_version: str,
    source_commit: str,
    docker_humble_digest: str,
    docker_jazzy_digest: str,
    *,
    candidate_set: dict[str, Any] | None = None,
    candidate_set_sha256: str | None = None,
) -> dict[str, Any]:
    """Build one schema-validated release or candidate observer packet."""
    docker_digests = {
        'humble': docker_humble_digest,
        'jazzy': docker_jazzy_digest,
    }
    _validate_identity(product_version, source_commit, docker_digests)
    rows = [
        _docker_row(
            product_version,
            distro,
            docker_digests[distro],
            candidate_set=candidate_set,
            candidate_set_sha256=candidate_set_sha256,
        )
        for distro in DISTROS
    ]
    rows.extend(
        _source_row(product_version, distro, source_commit)
        for distro in DISTROS
    )
    if candidate_set is None:
        docker_mode = 'release'
        packet_suffix = 'release'
        docker_command = _command([
            'python3',
            'scripts/check_published_release.py',
            '--version',
            product_version,
            '--json',
        ])
        docker_required_status = 'PUBLISHED'
        docker_evidence = {
            'mode': docker_mode,
            'candidate_set_sha256': None,
            'source_pr': None,
            'source_commit': None,
            'workflow_run_url': None,
            'requested_by': None,
            'registry_retention_status': 'NOT_APPLICABLE',
            'evidence_retention_days': None,
        }
        identity_action = (
            'Run the read-only published-release check and require PUBLISHED '
            'before provisioning a trial host.'
        )
    else:
        if candidate_set_sha256 is None:
            raise PacketError('candidate packet requires the exact set hash')
        docker_mode = 'candidate-image-set'
        run_id = candidate_set['workflow_run_url'].rsplit('/', 1)[1]
        packet_suffix = f'candidate-{run_id}'
        docker_command = _command([
            'python3',
            'scripts/audit_candidate_image_set.py',
            '--candidate-image-set',
            CANDIDATE_SET_PLACEHOLDER,
            '--remote',
            '--json',
        ])
        docker_required_status = 'REMOTE_AUDIT_PASS'
        docker_evidence = {
            'mode': docker_mode,
            'candidate_set_sha256': candidate_set_sha256,
            'source_pr': candidate_set['source_pr'],
            'source_commit': candidate_set['source_commit'],
            'workflow_run_url': candidate_set['workflow_run_url'],
            'requested_by': candidate_set['requested_by'],
            'registry_retention_status': (
                candidate_set['registry_retention_status']
            ),
            'evidence_retention_days': (
                candidate_set['evidence_retention_days']
            ),
        }
        identity_action = (
            'Retain the exact candidate-image-set bytes, run the read-only '
            'remote audit, and require REMOTE_AUDIT_PASS before '
            'provisioning a trial host.'
        )
    source_command = _command([
        'python3',
        'scripts/run_source_onboarding_probe.py',
        '--public-preflight',
        '--source-commit',
        source_commit,
        '--product-version',
        product_version,
    ])
    packet = {
        'schema_version': 2,
        'schema_uri': SCHEMA_URI,
        'packet_id': f'g0-onboarding-{product_version}-{packet_suffix}',
        'repository': REPOSITORY,
        'product_version': product_version,
        'dataset_id': DATASET_ID,
        'status': 'READY_FOR_READ_ONLY_PREFLIGHT',
        'docker_identity_mode': docker_mode,
        'docker_evidence': docker_evidence,
        'public_checks': {
            'docker': {
                'command': docker_command,
                'required_status': docker_required_status,
                'read_only': True,
            },
            'source': {
                'command': source_command,
                'required_status': 'READY',
                'read_only': True,
            },
        },
        'rows': rows,
        'next_actions': [
            identity_action,
            'Run the source read-only public check and require READY before '
            'provisioning a source-row trial host.',
            'Use one disposable host and one independent observer per row; '
            'do not use a moving tag or a shared filesystem.',
            'Record all seven required measurements, including human active '
            'time and submitted command count; blank means non-comparable.',
            'Replace a checked-in matrix row only with the immutable record '
            'and an exclusive measurement supplement bound to its bytes.',
            'This packet is not evidence and authorizes no release, image, '
            'issue, community, or telemetry write.',
        ],
        'authority': {
            'network_reads_performed': False,
            'trial_executed': False,
            'github_writes_authorized': False,
            'community_posts_authorized': False,
            'remote_mutations_performed': False,
        },
    }
    _validate_packet(packet)
    return packet


def build_packet(
    product_version: str,
    source_commit: str,
    docker_humble_digest: str,
    docker_jazzy_digest: str,
) -> dict[str, Any]:
    """Build a published-release observer packet (v1 API compatibility)."""
    return _build_packet(
        product_version,
        source_commit,
        docker_humble_digest,
        docker_jazzy_digest,
    )


def build_candidate_packet(
    candidate_set: dict[str, Any],
    candidate_set_sha256: str,
) -> dict[str, Any]:
    """Build a tag-free packet derived only from one candidate set."""
    try:
        audit_candidate_set(
            candidate_set,
            candidate_set_sha256=candidate_set_sha256,
            remote=False,
        )
    except ValueError as exc:
        raise PacketError(f'candidate image set is invalid: {exc}') from exc
    by_distro = {
        image['ros_distro']: image for image in candidate_set['images']
    }
    return _build_packet(
        candidate_set['product_version'],
        candidate_set['source_commit'],
        by_distro['humble']['digest'],
        by_distro['jazzy']['digest'],
        candidate_set=candidate_set,
        candidate_set_sha256=candidate_set_sha256,
    )


def render_packet(packet: dict[str, Any]) -> str:
    """Render a concise operator card without private paths or evidence."""
    lines = [
        '# G0 onboarding observer packet',
        '',
        f"- Status: **{packet['status']}**",
        f"- Product version: `{packet['product_version']}`",
        f"- Dataset: `{packet['dataset_id']}`",
        f"- Docker identity mode: `{packet['docker_identity_mode']}`",
        '- This is a plan only; it is not a trial record or a release claim.',
        '',
    ]
    if packet['docker_evidence']['candidate_set_sha256'] is not None:
        lines.extend([
            '- Candidate set SHA-256: '
            f"`{packet['docker_evidence']['candidate_set_sha256']}`",
            '- Candidate workflow: '
            f"`{packet['docker_evidence']['workflow_run_url']}`",
            '',
        ])
    lines.extend(['## Public checks', ''])
    for name, check in packet['public_checks'].items():
        lines.extend([
            f'### {name}',
            f"- Required status: `{check['required_status']}`",
            f"- Read-only command: `{check['command']}`",
            '',
        ])
    lines.extend([
        '## Fixed rows',
        '',
        '| Row | Identity | Required human observations |',
        '| --- | --- | --- |',
    ])
    for row in packet['rows']:
        identity = row['identity']['immutable_ref'] or row['identity']['value']
        lines.append(
            f"| `{row['row_id']}` | `{identity}` | "
            '`active_operator_time_sec`, `command_count` |'
        )
    lines.extend(['', '## Observer commands', ''])
    for row in packet['rows']:
        lines.extend([
            f"### `{row['row_id']}`",
            '```bash',
            row['observer_command'],
            '```',
            '',
        ])
    lines.extend(['## Required next actions', ''])
    lines.extend(f'- {action}' for action in packet['next_actions'])
    return '\n'.join(lines) + '\n'


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--product-version')
    parser.add_argument('--source-commit')
    parser.add_argument('--docker-humble-digest')
    parser.add_argument('--docker-jazzy-digest')
    parser.add_argument(
        '--candidate-image-set',
        type=Path,
        help=(
            'derive all Docker and source identities from one retained '
            'candidate-image-set JSON; cannot be mixed with release inputs'
        ),
    )
    output = parser.add_mutually_exclusive_group()
    output.add_argument('--json', action='store_true')
    output.add_argument('--render', action='store_true')
    parser.add_argument(
        '--output',
        type=Path,
        help=(
            'write the selected JSON or Markdown packet once; refuse an '
            'existing path'
        ),
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Build and optionally write one local-only observer packet."""
    args = _parse_args(argv)
    try:
        release_inputs = {
            '--product-version': args.product_version,
            '--source-commit': args.source_commit,
            '--docker-humble-digest': args.docker_humble_digest,
            '--docker-jazzy-digest': args.docker_jazzy_digest,
        }
        supplied_release_inputs = [
            name for name, value in release_inputs.items()
            if value is not None
        ]
        if args.candidate_image_set is not None:
            if supplied_release_inputs:
                raise PacketError(
                    '--candidate-image-set cannot be mixed with manual '
                    'release inputs: ' + ', '.join(supplied_release_inputs)
                )
            candidate_set = load_json_object(
                args.candidate_image_set,
                'candidate image set',
            )
            packet = build_candidate_packet(
                candidate_set,
                sha256_file(args.candidate_image_set),
            )
        else:
            missing = [
                name for name, value in release_inputs.items()
                if value is None
            ]
            if missing:
                raise PacketError(
                    'release mode requires: ' + ', '.join(missing)
                )
            packet = build_packet(
                args.product_version,
                args.source_commit,
                args.docker_humble_digest,
                args.docker_jazzy_digest,
            )
    except (OSError, ValueError) as exc:
        print(f'onboarding observer packet error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        payload = json.dumps(packet, indent=2, sort_keys=True) + '\n'
    else:
        payload = render_packet(packet)
    try:
        if args.output is None:
            sys.stdout.write(payload)
        else:
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(payload)
            print(
                f'Wrote local-only observer packet: {args.output}',
                file=sys.stderr,
            )
            print(
                'The packet is a plan, not trial evidence or publication '
                'authority.',
                file=sys.stderr,
            )
    except OSError as exc:
        print(f'onboarding observer packet error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
