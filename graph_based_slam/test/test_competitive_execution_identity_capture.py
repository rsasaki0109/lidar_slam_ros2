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
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for read-only competitive execution identity capture/finalization."""

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
RECEIPT_PATH = ROOT / 'configs' / 'slam_benchmark_profiles' / (
    'competitive_execution_selection_2026-08.yaml')
PROFILE_PATH = ROOT / 'configs' / 'slam_benchmark_profiles' / (
    'competitive_slam_v1.yaml')
SCRIPT_PATH = ROOT / 'scripts' / 'capture_competitive_execution_identity.py'
SPEC = importlib.util.spec_from_file_location('execution_identity_capture', SCRIPT_PATH)
assert SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _documents():
    return (
        yaml.safe_load(RECEIPT_PATH.read_text(encoding='utf-8')),
        yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8')),
    )


def test_capture_is_observation_only_and_preserves_pending_receipt(monkeypatch):
    receipt, profile = _documents()
    before = RECEIPT_PATH.read_bytes()
    monkeypatch.setattr(
        MODULE, 'probe_container',
        lambda tag: {'status': 'pending_build', 'image_tag': tag,
                     'image_digest': None})
    captured = MODULE.capture_identity(receipt, profile)
    assert RECEIPT_PATH.read_bytes() == before
    assert captured['status'] == 'INCOMPLETE'
    assert captured['pass'] is False
    assert captured['receipt_mutation']['performed'] is False
    assert captured['fresh_data_access']['raw_bag_opened'] is False
    assert captured['fresh_data_access']['ground_truth_opened'] is False
    assert captured['source_receipt']['sha256'] == hashlib.sha256(before).hexdigest()
    assert captured['machine_fingerprint']['machine_id']
    assert set(MODULE.THREAD_KEYS).issubset(captured['thread_policy'])
    assert all(
        slot['status'] == 'selected_unopened'
        for slot in profile['competitive_slam_profile']['datasets'][
            'fresh_holdout_slots'].values())


def test_finalize_requires_complete_capture_and_never_promotes_receipt(monkeypatch):
    receipt, profile = _documents()
    monkeypatch.setattr(
        MODULE, 'probe_container',
        lambda tag: {'status': 'pending_build', 'image_tag': tag,
                     'image_digest': None})
    captured = MODULE.capture_identity(receipt, profile)
    finalized = MODULE.finalize_identity(receipt, profile, captured)
    assert finalized['status'] == 'INCOMPLETE'
    assert finalized['pass'] is False
    assert finalized['receipt_mutation']['performed'] is False
    assert finalized['receipt_mutation']['automatic_promotion'] is False
    assert finalized['source_receipt_sha256'] == captured['source_receipt']['sha256']
    assert any('source receipt status' in item for item in finalized['blockers'])


def test_finalize_rejects_capture_from_different_receipt(monkeypatch):
    receipt, profile = _documents()
    monkeypatch.setattr(
        MODULE, 'probe_container',
        lambda tag: {'status': 'pending_build', 'image_tag': tag,
                     'image_digest': None})
    captured = MODULE.capture_identity(receipt, profile)
    mutated = copy.deepcopy(captured)
    mutated['source_receipt']['sha256'] = '0' * 64
    finalized = MODULE.finalize_identity(receipt, profile, mutated)
    assert finalized['status'] == 'INVALID'
    assert finalized['pass'] is False
    assert any('different execution-selection receipt' in item
               for item in finalized['errors'])


def _complete_fixture(tmp_path):
    """Create a fully measured, synthetic ready/frozen identity contract."""
    receipt_path = tmp_path / 'custom-receipt.yaml'
    profile_path = tmp_path / 'custom-profile.yaml'
    machine_path = tmp_path / 'machine-fingerprint.json'
    machine_path.write_text('{"machine": "fixture"}\n', encoding='utf-8')
    machine_sha = hashlib.sha256(machine_path.read_bytes()).hexdigest()
    thread = {
        'status': 'ready',
        'cpu_affinity': [0, 1],
        'max_threads': 2,
        'omp_num_threads': 1,
        'openblas_num_threads': 1,
        'mkl_num_threads': 1,
        'tbb_num_threads': 1,
        'accelerator_policy': 'cpu',
    }
    toolchain_fields = {
        'compiler': 'gcc-fixture',
        'linker': 'ld-fixture',
        'ros_distro': 'jazzy-fixture',
        'pcl': 'pcl-fixture',
        'eigen': 'eigen-fixture',
        'openmp': 'openmp-fixture',
    }
    repository = {
        'revision': 'a' * 40,
        'revision_status': 'ready',
        'worktree_dirty': False,
        'tracked_diff_sha256': 'b' * 64,
        'untracked_content_sha256': 'c' * 64,
        'clean_provenance_sha256': 'd' * 64,
    }
    systems = {}
    for system in MODULE.SYSTEMS:
        systems[system] = {
            'repository': dict(repository),
            'container': {
                'image_tag': f'{system}:fixture',
                'image_digest': 'sha256:' + 'e' * 64,
                'status': 'ready',
            },
            'toolchain': {
                'fingerprint': 'f' * 64,
                'status': 'ready',
                'scope': 'system_container',
                'image_digest': 'sha256:' + 'e' * 64,
                'observed': dict(toolchain_fields),
            },
        }
    receipt = {
        'schema_version': 1,
        'receipt_kind': 'competitive_execution_selection',
        'status': 'ready',
        'common_identity': {
            'machine_fingerprint': {
                'path': str(machine_path),
                'sha256': machine_sha,
                'machine_id': 'fixture-machine',
                'status': 'ready',
            },
            'thread_policy': thread,
        },
        'systems': systems,
    }
    profile = {
        'competitive_slam_profile': {
            'evidence_gate_v2': {
                'execution_selection_receipt_path': str(receipt_path),
                'execution_selection_receipt_sha256': None,
            },
            'datasets': {'fresh_holdout_slots': {}},
        },
    }
    receipt['common_identity']['profile_sha256'] = (
        MODULE.canonical_profile_sha256(profile))
    receipt['common_identity']['profile_sha256_kind'] = (
        MODULE.PROFILE_CANONICAL_HASH_KIND)
    receipt_path.write_text(yaml.safe_dump(receipt, sort_keys=True), encoding='utf-8')
    profile['competitive_slam_profile']['evidence_gate_v2'][
        'execution_selection_receipt_sha256'] = hashlib.sha256(
            receipt_path.read_bytes()).hexdigest()
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=True), encoding='utf-8')
    return (
        receipt, profile, receipt_path, profile_path,
        {system: tmp_path / f'{system}-checkout' for system in MODULE.SYSTEMS},
        toolchain_fields)


def _patch_complete_probes(monkeypatch, toolchain_fields):
    repository = {
        'revision': 'a' * 40,
        'worktree_dirty': False,
        'tracked_diff_sha256': 'b' * 64,
        'untracked_content_sha256': 'c' * 64,
        'clean_provenance_sha256': 'd' * 64,
        'status': 'observed',
    }
    monkeypatch.setattr(
        MODULE, 'capture_machine',
        lambda: {'machine_id': 'fixture-machine'})
    monkeypatch.setattr(
        MODULE, 'capture_thread_policy',
        lambda: {
            'status': 'observed',
            'cpu_affinity': [0, 1],
            'max_threads': 2,
            'omp_num_threads': 1,
            'openblas_num_threads': 1,
            'mkl_num_threads': 1,
            'tbb_num_threads': 1,
            'accelerator_policy': 'cpu',
        })
    monkeypatch.setattr(
        MODULE, 'capture_git_provenance',
        lambda _path: dict(repository))
    monkeypatch.setattr(
        MODULE, 'probe_container',
        lambda tag: {
            'status': 'observed',
            'image_tag': tag,
            'image_digest': 'sha256:' + 'e' * 64,
        })
    monkeypatch.setattr(
        MODULE, 'capture_container_toolchain',
        lambda tag, digest: {
            'status': 'observed',
            'fingerprint': 'f' * 64,
            'scope': 'system_container',
            'image_tag': tag,
            'image_digest': digest,
            'observed': dict(toolchain_fields),
        })


def test_custom_paths_and_complete_capture_can_finalize_pass(monkeypatch, tmp_path):
    (receipt, profile, receipt_path, profile_path, sources,
     toolchain_fields) = _complete_fixture(tmp_path)
    for source in sources.values():
        source.mkdir()
    _patch_complete_probes(monkeypatch, toolchain_fields)
    captured = MODULE.capture_identity(
        receipt, profile, root=tmp_path, receipt_path=receipt_path,
        profile_path=profile_path, system_sources=sources)
    assert captured['status'] == 'PASS'
    assert captured['pass'] is True
    assert captured['source_receipt']['path'] == str(receipt_path.resolve())
    assert captured['source_profile']['path'] == str(profile_path.resolve())
    finalized = MODULE.finalize_identity(
        receipt, profile, captured, root=tmp_path,
        receipt_path=receipt_path, profile_path=profile_path)
    assert finalized['status'] == 'PASS'
    assert finalized['pass'] is True
    assert finalized['receipt_mutation']['performed'] is False


def test_finalize_detects_measured_repository_tampering(monkeypatch, tmp_path):
    (receipt, profile, receipt_path, profile_path, sources,
     toolchain_fields) = _complete_fixture(tmp_path)
    for source in sources.values():
        source.mkdir()
    _patch_complete_probes(monkeypatch, toolchain_fields)
    captured = MODULE.capture_identity(
        receipt, profile, root=tmp_path, receipt_path=receipt_path,
        profile_path=profile_path, system_sources=sources)
    tampered = copy.deepcopy(captured)
    tampered['systems']['ours']['repository']['revision'] = '0' * 40
    finalized = MODULE.finalize_identity(
        receipt, profile, tampered, root=tmp_path,
        receipt_path=receipt_path, profile_path=profile_path)
    assert finalized['status'] == 'INVALID'
    assert finalized['pass'] is False
    assert any('ours repository revision mismatch' in item
               for item in finalized['errors'])


def test_finalize_detects_measured_thread_and_machine_tampering(monkeypatch, tmp_path):
    (receipt, profile, receipt_path, profile_path, sources,
     toolchain_fields) = _complete_fixture(tmp_path)
    for source in sources.values():
        source.mkdir()
    _patch_complete_probes(monkeypatch, toolchain_fields)
    captured = MODULE.capture_identity(
        receipt, profile, root=tmp_path, receipt_path=receipt_path,
        profile_path=profile_path, system_sources=sources)
    tampered = copy.deepcopy(captured)
    tampered['thread_policy']['omp_num_threads'] = 2
    tampered['machine_fingerprint']['machine_id'] = 'tampered-machine'
    finalized = MODULE.finalize_identity(
        receipt, profile, tampered, root=tmp_path,
        receipt_path=receipt_path, profile_path=profile_path)
    assert finalized['status'] == 'INVALID'
    assert any('machine fingerprint machine_id mismatch' in item
               for item in finalized['errors'])
    assert any('canonical thread policy mismatch' in item
               for item in finalized['errors'])


def test_finalize_rejects_mismatched_toolchain_image_digest(monkeypatch, tmp_path):
    (receipt, profile, receipt_path, profile_path, sources,
     toolchain_fields) = _complete_fixture(tmp_path)
    for source in sources.values():
        source.mkdir()
    _patch_complete_probes(monkeypatch, toolchain_fields)
    captured = MODULE.capture_identity(
        receipt, profile, root=tmp_path, receipt_path=receipt_path,
        profile_path=profile_path, system_sources=sources)
    tampered = copy.deepcopy(captured)
    tampered['systems']['ours']['toolchain']['image_digest'] = 'sha256:' + '0' * 64
    finalized = MODULE.finalize_identity(
        receipt, profile, tampered, root=tmp_path,
        receipt_path=receipt_path, profile_path=profile_path)
    assert finalized['status'] == 'INVALID'
    assert any('observed toolchain image digest' in item
               for item in finalized['errors'])


def test_container_toolchain_probe_is_bounded_pull_free_and_digest_bound(monkeypatch):
    commands = []

    def fake_run(command, **_kwargs):
        commands.append(command)
        return 0, 'fixture-toolchain\n', ''

    monkeypatch.setattr(MODULE, 'run_read_only', fake_run)
    digest = 'sha256:' + '1' * 64
    result = MODULE.capture_container_toolchain('fixture:local', digest)
    assert result['status'] == 'observed'
    assert result['image_digest'] == digest
    assert result['fingerprint']
    assert len(commands) == 6
    assert all('--pull=never' in command for command in commands)
    assert all('--network' in command and 'none' in command for command in commands)
    assert all('--read-only' in command for command in commands)
    assert all('docker' in command and 'pull' not in command for command in commands)
    assert all(digest in command for command in commands)
    assert all('fixture:local' not in command for command in commands)
    openmp_command = commands[-1][-1]
    assert 'libomp-dev' in openmp_command and 'libgomp1' in openmp_command


def test_cli_explicit_custom_paths_are_owned_by_capture(tmp_path):
    custom_receipt = tmp_path / 'receipt.yaml'
    custom_profile = tmp_path / 'profile.yaml'
    custom_receipt.write_bytes(RECEIPT_PATH.read_bytes())
    custom_profile.write_bytes(PROFILE_PATH.read_bytes())
    output = tmp_path / 'capture.json'
    completed = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), 'capture',
         '--receipt', str(custom_receipt), '--profile', str(custom_profile),
         '--source', f'ours={ROOT}', '--image', 'ours=fixture:local',
         '--output', str(output)],
        cwd=str(ROOT), check=False, capture_output=True, text=True)
    assert completed.returncode == 1
    result = json.loads(output.read_text(encoding='utf-8'))
    assert result['source_receipt']['path'] == str(custom_receipt.resolve())
    assert result['source_profile']['path'] == str(custom_profile.resolve())
    assert result['systems']['ours']['repository']['source_path'] == str(ROOT.resolve())
    assert result['systems']['ours']['container']['image_tag'] == 'fixture:local'
