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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for the read-only immutable candidate-image set audit."""

from __future__ import annotations

import importlib.util
import json
import pathlib
import subprocess
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'audit_candidate_image_set.py'


def _module():
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        spec = importlib.util.spec_from_file_location(
            'audit_candidate_image_set', SCRIPT
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPT.parent))


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


def test_local_audit_binds_the_set_without_network_or_write_authority():
    """Offline validation binds exact bytes and grants no write authority."""
    module = _module()
    report = module.audit_candidate_set(
        _candidate_set(),
        candidate_set_sha256='d' * 64,
    )

    assert report['status'] == 'LOCAL_CONTRACT_PASS'
    assert report['candidate_set_sha256'] == 'd' * 64
    assert [image['ros_distro'] for image in report['images']] == [
        'humble', 'jazzy',
    ]
    assert all(
        image['registry_status'] == 'NOT_CHECKED'
        for image in report['images']
    )
    assert report['authority'] == {
        'network_reads_performed': False,
        'github_writes_authorized': False,
        'registry_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def test_semantically_duplicate_distro_fails_even_when_schema_shape_passes():
    """Schema-shaped duplicate distro entries cannot masquerade as a pair."""
    module = _module()
    candidate_set = _candidate_set()
    candidate_set['images'][1]['ros_distro'] = 'humble'

    with pytest.raises(
        module.CandidateSetAuditError,
        match='Humble and Jazzy exactly once',
    ):
        module.audit_candidate_set(
            candidate_set,
            candidate_set_sha256='d' * 64,
        )


def _remote_runner(module, *, expire_artifact=False, fail_attestation=False):
    calls = []

    def runner(command, **_kwargs):
        calls.append(command)
        if command[:2] == ['gh', 'api'] and command[2].endswith(
            '/actions/runs/12345'
        ):
            payload = {
                'id': 12345,
                'html_url': (
                    'https://github.com/rsasaki0109/lidar_slam_ros2/'
                    'actions/runs/12345'
                ),
                'event': 'repository_dispatch',
                'status': 'completed',
                'conclusion': 'success',
                'head_branch': 'develop',
                'path': '.github/workflows/candidate-image.yml',
            }
            return subprocess.CompletedProcess(
                command, 0, json.dumps(payload), ''
            )
        if command[:2] == ['gh', 'api'] and 'artifacts?' in command[2]:
            payload = {
                'artifacts': [
                    {
                        'name': name,
                        'expired': expire_artifact and index == 0,
                        'expires_at': '2026-09-14T00:00:00Z',
                    }
                    for index, name in enumerate(module.EXPECTED_ARTIFACTS)
                ],
            }
            return subprocess.CompletedProcess(
                command, 0, json.dumps(payload), ''
            )
        if command[:4] == ['docker', 'buildx', 'imagetools', 'inspect']:
            digest = command[4].split('@', 1)[1]
            return subprocess.CompletedProcess(
                command, 0, json.dumps({'digest': digest}), ''
            )
        if command[:3] == ['gh', 'attestation', 'verify']:
            return subprocess.CompletedProcess(
                command,
                1 if fail_attestation else 0,
                '',
                'unverified' if fail_attestation else '',
            )
        raise AssertionError(f'unexpected command: {command}')

    return runner, calls


def test_remote_audit_requires_run_artifacts_digests_and_attestations():
    """Remote PASS requires every retained and registry-backed proof."""
    module = _module()
    runner, calls = _remote_runner(module)

    report = module.audit_candidate_set(
        _candidate_set(),
        candidate_set_sha256='d' * 64,
        remote=True,
        runner=runner,
    )

    assert report['status'] == 'REMOTE_AUDIT_PASS'
    assert report['workflow']['status'] == 'PASS'
    assert report['artifacts'] == {
        'status': 'PASS',
        'required_names': list(module.EXPECTED_ARTIFACTS),
        'expires_at': '2026-09-14T00:00:00Z',
    }
    assert report['findings'] == []
    assert all(
        image['registry_status'] == 'PASS'
        and image['attestation_status'] == 'PASS'
        for image in report['images']
    )
    assert sum(
        call[:3] == ['gh', 'attestation', 'verify'] for call in calls
    ) == 2
    assert report['authority']['network_reads_performed'] is True
    assert report['authority']['remote_mutations_performed'] is False


def test_remote_audit_fails_closed_on_expiry_or_attestation_failure():
    """Expired evidence or missing attestations keep the trial blocked."""
    module = _module()
    runner, _calls = _remote_runner(
        module,
        expire_artifact=True,
        fail_attestation=True,
    )

    report = module.audit_candidate_set(
        _candidate_set(),
        candidate_set_sha256='d' * 64,
        remote=True,
        runner=runner,
    )

    assert report['status'] == 'REMOTE_AUDIT_FAIL'
    assert report['artifacts']['status'] == 'FAIL'
    assert 'candidate-artifact-set-incomplete-or-expired' in report['findings']
    assert 'humble-attestation-unverified' in report['findings']
    assert 'jazzy-attestation-unverified' in report['findings']


def test_cli_hashes_the_exact_retained_set_without_remote_reads(
    tmp_path,
    capsys,
):
    """The CLI hashes one retained set and stays offline by default."""
    module = _module()
    candidate_path = tmp_path / 'candidate-image-set.json'
    candidate_path.write_text(
        json.dumps(_candidate_set(), sort_keys=True) + '\n',
        encoding='utf-8',
    )

    assert module.main([
        '--candidate-image-set', str(candidate_path), '--json',
    ]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['candidate_set_sha256'] == module.sha256_file(candidate_path)
    assert report['status'] == 'LOCAL_CONTRACT_PASS'
