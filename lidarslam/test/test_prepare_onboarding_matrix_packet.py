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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for the exact-identity G0 observer packet."""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'prepare_onboarding_matrix_packet.py'


def _module():
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        spec = importlib.util.spec_from_file_location(
            'prepare_onboarding_matrix_packet', SCRIPT
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPT.parent))


def _packet(module):
    return module.build_packet(
        '0.9.1',
        'a' * 40,
        'sha256:' + 'b' * 64,
        'sha256:' + 'c' * 64,
    )


def _candidate_set():
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/candidate-image-set-v1.schema.json'
        ),
        'status': 'PASS',
        'publication_mode': 'digest_only',
        'repository': 'rsasaki0109/lidar_slam_ros2',
        'source_pr': 427,
        'source_commit': 'a' * 40,
        'product_version': '0.9.1',
        'platform': 'linux/amd64',
        'workflow_run_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/'
            'actions/runs/12345'
        ),
        'workflow_branch_ref': 'refs/heads/develop',
        'requested_by': 'maintainer',
        'images': [
            {
                'ros_distro': 'humble',
                'digest': 'sha256:' + 'b' * 64,
                'immutable_ref': (
                    'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:'
                    + 'b' * 64
                ),
            },
            {
                'ros_distro': 'jazzy',
                'digest': 'sha256:' + 'c' * 64,
                'immutable_ref': (
                    'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:'
                    + 'c' * 64
                ),
            },
        ],
        'tags_created': [],
        'moving_tags_mutated': False,
        'release_mutated': False,
        'registry_retention_status': 'REQUIRES_REMOTE_AUDIT',
        'evidence_retention_days': 30,
    }


def test_packet_aligns_all_rows_to_one_version_and_exact_identities():
    """Every row shares the requested version and immutable identity."""
    module = _module()
    packet = _packet(module)

    assert packet['status'] == 'READY_FOR_READ_ONLY_PREFLIGHT'
    assert packet['schema_version'] == 2
    assert packet['packet_id'] == 'g0-onboarding-0.9.1-release'
    assert packet['docker_identity_mode'] == 'release'
    assert [row['row_id'] for row in packet['rows']] == [
        'docker-humble',
        'docker-jazzy',
        'source-humble',
        'source-jazzy',
    ]
    assert {row['product_version'] for row in packet['rows']} == {'0.9.1'}
    assert packet['rows'][0]['identity']['value'] == 'sha256:' + 'b' * 64
    assert packet['rows'][1]['identity']['value'] == 'sha256:' + 'c' * 64
    assert packet['rows'][2]['identity']['value'] == 'a' * 40
    assert packet['rows'][3]['identity']['value'] == 'a' * 40
    assert all(
        len(row['required_measurements']) == 7 for row in packet['rows']
    )
    assert packet['authority'] == {
        'network_reads_performed': False,
        'trial_executed': False,
        'github_writes_authorized': False,
        'community_posts_authorized': False,
        'remote_mutations_performed': False,
    }


def test_packet_commands_pin_identity_and_keep_paths_as_placeholders():
    """Rendered commands pin identities without leaking local paths."""
    module = _module()
    packet = _packet(module)
    commands = '\n'.join(
        [
            packet['public_checks']['docker']['command'],
            packet['public_checks']['source']['command'],
            *(row['observer_command'] for row in packet['rows']),
        ]
    )

    assert '--version 0.9.1' in commands
    assert 'a' * 40 in commands
    assert 'sha256:' + 'b' * 64 in commands
    assert 'sha256:' + 'c' * 64 in commands
    assert '--prompt-human-measurements' in commands
    assert '<TRIAL_RECORD_OUTSIDE_CHECKOUT>' in commands
    assert '<TRIAL_ROOT_OUTSIDE_CHECKOUT>' in commands
    assert '<OBSERVER_PARENT_OUTSIDE_CHECKOUT>' in commands
    assert '/home/' not in commands
    assert '$HOME' not in commands
    assert packet['public_checks']['docker']['read_only'] is True
    assert packet['public_checks']['source']['read_only'] is True
    assert packet['public_checks']['docker']['command'] == (
        'python3 scripts/check_published_release.py '
        '--version 0.9.1 --json'
    )
    assert packet['public_checks']['docker']['command'].count(
        'check_published_release.py'
    ) == 1


def test_candidate_packet_uses_retained_set_without_inventing_release_tags():
    """Candidate rows bind one set and remain tag-free end to end."""
    module = _module()
    packet = module.build_candidate_packet(_candidate_set(), 'd' * 64)
    payload = json.dumps(packet, sort_keys=True)
    docker_rows = [
        row for row in packet['rows'] if row['route'] == 'docker'
    ]

    assert packet['packet_id'] == 'g0-onboarding-0.9.1-candidate-12345'
    assert packet['docker_identity_mode'] == 'candidate-image-set'
    assert packet['docker_evidence'] == {
        'mode': 'candidate-image-set',
        'candidate_set_sha256': 'd' * 64,
        'source_pr': 427,
        'source_commit': 'a' * 40,
        'workflow_run_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/'
            'actions/runs/12345'
        ),
        'requested_by': 'maintainer',
        'registry_retention_status': 'REQUIRES_REMOTE_AUDIT',
        'evidence_retention_days': 30,
    }
    assert packet['public_checks']['docker']['required_status'] == (
        'REMOTE_AUDIT_PASS'
    )
    assert packet['public_checks']['docker']['command'] == (
        'python3 scripts/audit_candidate_image_set.py '
        "--candidate-image-set '<CANDIDATE_IMAGE_SET_JSON>' --remote --json"
    )
    assert all(row['identity']['tag'] is None for row in docker_rows)
    assert all(
        row['identity']['immutable_ref'].startswith(
            'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:'
        )
        for row in docker_rows
    )
    assert '--candidate-image-ref' in payload
    assert '--candidate-image-set-sha256 ' + 'd' * 64 in payload
    assert '--candidate-source-pr 427' in payload
    assert 'v0.9.1-humble' not in payload
    assert 'v0.9.1-jazzy' not in payload
    assert 'check_published_release.py' not in payload


def test_candidate_cli_derives_identity_and_rejects_manual_overrides(
    tmp_path,
    capsys,
):
    """Candidate CLI accepts one retained set and rejects mixed identity."""
    module = _module()
    candidate_path = tmp_path / 'candidate-image-set.json'
    candidate_path.write_text(
        json.dumps(_candidate_set(), sort_keys=True) + '\n',
        encoding='utf-8',
    )

    assert module.main([
        '--candidate-image-set', str(candidate_path), '--json',
    ]) == 0
    packet = json.loads(capsys.readouterr().out)
    assert packet['docker_evidence']['candidate_set_sha256'] == (
        module.sha256_file(candidate_path)
    )

    assert module.main([
        '--candidate-image-set', str(candidate_path),
        '--product-version', '0.9.1',
        '--json',
    ]) == 2
    assert 'cannot be mixed' in capsys.readouterr().err


def test_candidate_packet_rejects_a_malformed_retained_set():
    """A schema-shaped but duplicate distro pair fails before rendering."""
    module = _module()
    candidate_set = _candidate_set()
    candidate_set['images'][1]['ros_distro'] = 'humble'

    with pytest.raises(module.PacketError, match='candidate image set'):
        module.build_candidate_packet(candidate_set, 'd' * 64)


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('product_version', 'development'),
        ('source_commit', 'A' * 40),
        ('docker_humble_digest', 'sha256:' + 'b' * 63),
        ('docker_jazzy_digest', 'sha256:' + 'b' * 64),
    ],
)
def test_packet_rejects_ambiguous_or_malformed_identity(field, value):
    """Malformed or copied identities fail before a packet is emitted."""
    module = _module()
    values = {
        'product_version': '0.9.1',
        'source_commit': 'a' * 40,
        'docker_humble_digest': 'sha256:' + 'b' * 64,
        'docker_jazzy_digest': 'sha256:' + 'c' * 64,
    }
    values[field] = value

    with pytest.raises(module.PacketError):
        module.build_packet(**values)


def test_render_is_explicitly_a_plan_not_a_measurement_record():
    """The human card remains a plan and never claims observed evidence."""
    module = _module()
    rendered = module.render_packet(_packet(module))

    assert 'READY_FOR_READ_ONLY_PREFLIGHT' in rendered
    assert 'not a trial record or a release claim' in rendered
    assert 'active_operator_time_sec' in rendered
    assert 'command_count' in rendered
    assert 'network_reads_performed' not in rendered


def test_output_is_exclusive_and_does_not_overwrite(tmp_path, capsys):
    """The packet writer refuses a second write to the same path."""
    module = _module()
    output = tmp_path / 'observer-packet.json'
    args = [
        '--product-version',
        '0.9.1',
        '--source-commit',
        'a' * 40,
        '--docker-humble-digest',
        'sha256:' + 'b' * 64,
        '--docker-jazzy-digest',
        'sha256:' + 'c' * 64,
        '--json',
        '--output',
        str(output),
    ]

    assert module.main(args) == 0
    original = output.read_bytes()
    assert module.main(args) == 2
    assert output.read_bytes() == original
    assert 'File exists' in capsys.readouterr().err
