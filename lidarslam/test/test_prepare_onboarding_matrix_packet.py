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


def test_packet_aligns_all_rows_to_one_version_and_exact_identities():
    """Every row shares the requested version and immutable identity."""
    module = _module()
    packet = _packet(module)

    assert packet['status'] == 'READY_FOR_READ_ONLY_PREFLIGHT'
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
            packet['public_checks']['release']['command'],
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
    assert packet['public_checks']['release']['read_only'] is True
    assert packet['public_checks']['source']['read_only'] is True
    assert packet['public_checks']['release']['command'] == (
        'python3 scripts/check_published_release.py '
        '--version 0.9.1 --json'
    )
    assert packet['public_checks']['release']['command'].count(
        'check_published_release.py'
    ) == 1


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
