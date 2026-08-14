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
source revision and both Docker image digests use one product version, then
renders read-only preflight and human-observer commands. It performs no
network request, trial, filesystem cleanup, or GitHub/community mutation.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shlex
import sys
from typing import Any

import jsonschema


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
    'schemas/onboarding-matrix-observer-packet-v1.schema.json'
)


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
) -> dict[str, Any]:
    image_tag = f'ghcr.io/{REPOSITORY}:v{product_version}-{distro}'
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
        },
        'preflight_command': _command([
            'python3',
            'scripts/check_published_release.py',
            '--version',
            product_version,
            '--json',
        ]),
        'observer_command': _command([
            'python3',
            'scripts/run_docker_onboarding_probe.py',
            '--trial-id',
            f'g0-docker-{distro}-<UTC_DATE>-a',
            '--ros-distro',
            distro,
            '--image-tag',
            image_tag,
            '--image-digest',
            digest,
            '--product-version',
            product_version,
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
        / 'onboarding-matrix-observer-packet-v1.schema.json'
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


def build_packet(
    product_version: str,
    source_commit: str,
    docker_humble_digest: str,
    docker_jazzy_digest: str,
) -> dict[str, Any]:
    """Build and schema-validate one immutable-identity observer packet."""
    docker_digests = {
        'humble': docker_humble_digest,
        'jazzy': docker_jazzy_digest,
    }
    _validate_identity(product_version, source_commit, docker_digests)
    rows = [
        _docker_row(product_version, distro, docker_digests[distro])
        for distro in DISTROS
    ]
    rows.extend(
        _source_row(product_version, distro, source_commit)
        for distro in DISTROS
    )
    release_command = _command([
        'python3',
        'scripts/check_published_release.py',
        '--version',
        product_version,
        '--json',
    ])
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
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'packet_id': f'g0-onboarding-{product_version}',
        'repository': REPOSITORY,
        'product_version': product_version,
        'dataset_id': DATASET_ID,
        'status': 'READY_FOR_READ_ONLY_PREFLIGHT',
        'public_checks': {
            'release': {
                'command': release_command,
                'required_status': 'PUBLISHED',
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
            'Run both read-only public checks and require the exact statuses '
            'before provisioning a trial host.',
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


def render_packet(packet: dict[str, Any]) -> str:
    """Render a concise operator card without private paths or evidence."""
    lines = [
        '# G0 onboarding observer packet',
        '',
        f"- Status: **{packet['status']}**",
        f"- Product version: `{packet['product_version']}`",
        f"- Dataset: `{packet['dataset_id']}`",
        '- This is a plan only; it is not a trial record or a release claim.',
        '',
        '## Public checks',
        '',
    ]
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
        identity = row['identity']['value']
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
    parser.add_argument('--product-version', required=True)
    parser.add_argument('--source-commit', required=True)
    parser.add_argument('--docker-humble-digest', required=True)
    parser.add_argument('--docker-jazzy-digest', required=True)
    output = parser.add_mutually_exclusive_group()
    output.add_argument('--json', action='store_true')
    output.add_argument('--render', action='store_true')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        packet = build_packet(
            args.product_version,
            args.source_commit,
            args.docker_humble_digest,
            args.docker_jazzy_digest,
        )
    except PacketError as exc:
        print(f'onboarding observer packet error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(packet, indent=2, sort_keys=True))
    else:
        print(render_packet(packet), end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
