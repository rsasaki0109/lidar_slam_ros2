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

"""Regression tests for source-safe RTK-SLAM dataset acquisition."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'download_rtk_slam_dataset.py'
SPEC = importlib.util.spec_from_file_location('rtk_slam_download', SCRIPT)
DOWNLOAD = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(DOWNLOAD)


def _tiny_manifest() -> dict:
    db3 = b'db3-data'
    metadata = b'metadata'
    return {
        'tiny': {
            'db3': {
                'relative_path': 'ros2/tiny/tiny.db3',
                'bytes': len(db3),
                'sha256': hashlib.sha256(db3).hexdigest(),
            },
            'metadata': {
                'relative_path': 'ros2/tiny/metadata.yaml',
                'bytes': len(metadata),
                'sha256': hashlib.sha256(metadata).hexdigest(),
            },
        },
    }


def _write_tiny_inputs(dest: Path) -> None:
    sequence_dir = dest / 'ros2' / 'tiny'
    sequence_dir.mkdir(parents=True)
    (sequence_dir / 'tiny.db3').write_bytes(b'db3-data')
    (sequence_dir / 'metadata.yaml').write_bytes(b'metadata')


def test_manifest_pins_official_exact_identities():
    """Every upstream source is immutable and byte-exact."""
    assert DOWNLOAD.HF_REVISION == (
        '87619d2da3f345109b9a2b0d3a192a8596b4d2d3'
    )
    assert DOWNLOAD.EVAL_REPO_COMMIT == (
        'f2921a58caf5a87c1f4f73b48c6f2a5e35f92924'
    )
    assert list(DOWNLOAD.SEQUENCES) == [
        'construction_seq2',
        'construction_seq1',
        'stadtgarten_seq2',
        'stadtgarten_seq1',
    ]
    assert DOWNLOAD.SEQUENCES['construction_seq2']['db3'] == {
        'relative_path': 'ros2/construction_seq2/construction_seq2.db3',
        'bytes': 10_656_112_640,
        'sha256': (
            '9e808703a57d7be6afa6a37abb8f5d65'
            'c6566f71f4864cd4c24cb01f6ab82af5'
        ),
    }
    assert DOWNLOAD.SEQUENCES['construction_seq1']['db3']['bytes'] == (
        13_180_936_192
    )
    assert DOWNLOAD.SEQUENCES['stadtgarten_seq2']['db3']['bytes'] == (
        16_793_665_536
    )
    assert DOWNLOAD.SEQUENCES['stadtgarten_seq1']['db3']['bytes'] == (
        30_263_574_528
    )


def test_dry_run_json_is_write_and_network_free(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    """JSON planning does not create its absent destination or acquire data."""
    dest = tmp_path / 'missing-destination'
    monkeypatch.setattr(DOWNLOAD, '_available_bytes', lambda unused: 1)

    def unexpected_side_effect(*unused_args, **unused_kwargs):
        raise AssertionError('dry-run attempted an acquisition side effect')

    monkeypatch.setattr(DOWNLOAD, '_download', unexpected_side_effect)
    monkeypatch.setattr(DOWNLOAD, '_run_command', unexpected_side_effect)

    result = DOWNLOAD.main([
        '--sequence', 'construction_seq2', '--dest', str(dest),
        '--dry-run', '--json',
    ])
    output = capsys.readouterr()
    plan = json.loads(output.out)

    assert result == 0
    assert plan['status'] == 'BLOCKED_INSUFFICIENT_SPACE'
    assert plan['side_effects_started'] is False
    assert plan['dataset']['revision'] == DOWNLOAD.HF_REVISION
    assert 'no files, network requests' in output.err
    assert not dest.exists()


def test_capacity_plan_reports_exact_shortfall(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """Capacity reporting uses exact bytes and a deterministic reserve."""
    monkeypatch.setattr(DOWNLOAD, '_available_bytes', lambda unused: 1_000)

    plan = DOWNLOAD.build_plan(
        tmp_path / 'dataset', ['construction_seq2'], False,
    )
    storage = plan['storage']
    payload = 10_656_112_640 + 1_930
    reserve = (payload + 9) // 10

    assert storage['payload_remaining_bytes'] == payload
    assert storage['reserve_bytes'] == reserve
    assert storage['required_peak_bytes'] == payload + reserve
    assert storage['additional_bytes_required'] == payload + reserve - 1_000
    assert plan['status'] == 'BLOCKED_INSUFFICIENT_SPACE'


def test_partial_file_reduces_remaining_capacity_and_plans_resume(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """A regular partial file contributes to the resumable plan."""
    dest = tmp_path / 'dataset'
    sequence_dir = dest / 'ros2' / 'construction_seq2'
    sequence_dir.mkdir(parents=True)
    (sequence_dir / 'construction_seq2.db3').write_bytes(b'partial')
    monkeypatch.setattr(
        DOWNLOAD, '_available_bytes', lambda unused: 100_000_000_000,
    )

    plan = DOWNLOAD.build_plan(dest, ['construction_seq2'], False)
    db3, metadata = plan['files']

    assert db3['action'] == 'resume'
    assert db3['existing_bytes'] == len(b'partial')
    assert db3['transfer_bytes'] == db3['expected_bytes'] - len(b'partial')
    assert metadata['action'] == 'download'
    assert plan['storage']['payload_remaining_bytes'] == (
        db3['transfer_bytes'] + metadata['expected_bytes']
    )


def test_live_capacity_failure_stops_before_network_and_writes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    """A live request fails closed before acquisition when space is short."""
    dest = tmp_path / 'dataset'
    called = False

    def unexpected_download(unused_item):
        nonlocal called
        called = True

    monkeypatch.setattr(DOWNLOAD, '_available_bytes', lambda unused: 0)
    monkeypatch.setattr(DOWNLOAD, '_download', unexpected_download)

    result = DOWNLOAD.main(['--dest', str(dest)])
    output = capsys.readouterr()

    assert result == 2
    assert 'BLOCKED_INSUFFICIENT_SPACE' in output.out
    assert 'no files, network requests' in output.err
    assert called is False
    assert not dest.exists()


def test_exact_complete_files_are_verified_and_reused(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    """Exact local files are hash-verified and never downloaded again."""
    dest = tmp_path / 'dataset'
    _write_tiny_inputs(dest)
    monkeypatch.setattr(DOWNLOAD, 'SEQUENCES', _tiny_manifest())
    monkeypatch.setattr(DOWNLOAD, '_available_bytes', lambda unused: 0)

    def unexpected_download(unused_item):
        raise AssertionError('verified input was downloaded again')

    monkeypatch.setattr(DOWNLOAD, '_download', unexpected_download)

    result = DOWNLOAD.main(['--sequence', 'tiny', '--dest', str(dest)])
    output = capsys.readouterr()

    assert result == 0, output.err
    assert output.out.count('identity: PASS') == 2
    assert 'downloading:' not in output.out


def test_same_size_hash_mismatch_fails_before_download(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """Matching size cannot hide a wrong local identity."""
    dest = tmp_path / 'dataset'
    _write_tiny_inputs(dest)
    manifest = _tiny_manifest()
    manifest['tiny']['db3']['sha256'] = hashlib.sha256(b'not-data').hexdigest()
    monkeypatch.setattr(DOWNLOAD, 'SEQUENCES', manifest)

    with pytest.raises(DOWNLOAD.AcquisitionError, match='SHA-256 mismatch'):
        DOWNLOAD.build_plan(dest, ['tiny'], False)


def test_symlinked_sequence_destination_is_rejected(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """A sequence directory cannot redirect writes through a symlink."""
    dest = tmp_path / 'dataset'
    (dest / 'ros2').mkdir(parents=True)
    outside = tmp_path / 'outside'
    outside.mkdir()
    (dest / 'ros2' / 'tiny').symlink_to(outside, target_is_directory=True)
    monkeypatch.setattr(DOWNLOAD, 'SEQUENCES', _tiny_manifest())

    with pytest.raises(
        DOWNLOAD.AcquisitionError,
        match='must not be a symlink',
    ):
        DOWNLOAD.build_plan(dest, ['tiny'], False)


def test_eval_fetch_uses_only_the_pinned_commit(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """Eval acquisition fetches and checks out one immutable commit."""
    commands: list[list[str]] = []
    target = tmp_path / 'dataset' / 'rtk_slam_eval'
    item = {
        'path': str(target),
        'action': 'fetch-pinned',
    }
    monkeypatch.setattr(
        DOWNLOAD,
        '_run_command',
        lambda command: commands.append(list(command)),
    )
    monkeypatch.setattr(DOWNLOAD, '_verify_eval_assets', lambda unused: None)

    DOWNLOAD.fetch_eval_assets(item)

    assert commands[0][:3] == ['git', 'init', '--quiet']
    assert commands[1][-2:] == ['origin', DOWNLOAD.EVAL_REPO_URL]
    assert commands[2][-2:] == ['origin', DOWNLOAD.EVAL_REPO_COMMIT]
    assert commands[3][-1] == DOWNLOAD.EVAL_REPO_COMMIT
    assert '--detach' in commands[3]


def test_existing_eval_checkout_at_wrong_commit_is_rejected(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """A moving or stale eval checkout is not silently reused."""
    target = tmp_path / 'rtk_slam_eval'
    (target / '.git').mkdir(parents=True)
    monkeypatch.setattr(
        DOWNLOAD,
        '_git_output',
        lambda *unused, **kwargs: 'bad',
    )

    with pytest.raises(DOWNLOAD.AcquisitionError, match='expected'):
        DOWNLOAD._eval_plan(tmp_path, True)
