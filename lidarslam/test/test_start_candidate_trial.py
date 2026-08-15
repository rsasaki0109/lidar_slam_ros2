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

"""Tests for one-command candidate preparation plus row execution."""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / 'scripts'
SCRIPT = SCRIPT_DIR / 'start_candidate_trial.py'
sys.path.insert(0, str(SCRIPT_DIR))
try:
    SPEC = importlib.util.spec_from_file_location(
        'start_candidate_trial_test', SCRIPT
    )
    assert SPEC is not None and SPEC.loader is not None
    SESSION = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(SESSION)
finally:
    sys.path.remove(str(SCRIPT_DIR))


RUN_URL = (
    'https://github.com/rsasaki0109/lidar_slam_ros2/'
    'actions/runs/12345'
)
SOURCE_COMMIT = 'a' * 40
BUNDLE_SHA256 = 'b' * 64
SET_SHA256 = 'c' * 64


def _preparation() -> dict[str, object]:
    return {
        'status': 'READY_FOR_OBSERVER',
        'run_id': 12345,
        'source_pr': 427,
        'source_commit': SOURCE_COMMIT,
        'product_version': '0.9.1',
        'candidate_bundle_sha256': BUNDLE_SHA256,
        'candidate_set_sha256': SET_SHA256,
        'artifact_expires_at': '2026-09-14T00:00:00Z',
    }


def _row(route: str = 'docker', distro: str = 'jazzy') -> dict[str, object]:
    digest = 'sha256:' + ('d' * 64)
    docker = route == 'docker'
    return {
        'row_id': f'{route}-{distro}',
        'route': route,
        'ros_distro': distro,
        'os_family': 'ubuntu-22.04' if distro == 'humble' else 'ubuntu-24.04',
        'product_version': '0.9.1',
        'identity': {
            'kind': 'image-digest' if docker else 'git-commit',
            'value': digest if docker else SOURCE_COMMIT,
            'immutable_ref': (
                f'ghcr.io/rsasaki0109/lidar_slam_ros2@{digest}'
                if docker else
                'https://github.com/rsasaki0109/lidar_slam_ros2/tree/'
                f'{SOURCE_COMMIT}'
            ),
        },
    }


def _execution(
    *,
    status: str = 'TRIAL_RECORDED',
    route: str = 'docker',
    distro: str = 'jazzy',
    outcome: str | None = 'PASS',
    complete: bool = True,
) -> dict[str, object]:
    attempted = status != 'PREFLIGHT_BLOCKED'
    return {
        'status': status,
        'trial_id': f'g0-{route}-{distro}-20260815-a',
        'row': _row(route, distro),
        'trial': {
            'outcome_status': outcome,
            'measurement_status': (
                'COMPLETE' if complete and outcome is not None
                else 'INCOMPLETE' if outcome is not None else None
            ),
            'comparable': (
                complete and outcome == 'PASS'
                if outcome is not None else None
            ),
        },
        'outputs': {
            'private_evidence_directory': 'private' if attempted else None,
        },
        'sharing': {
            'bounded_outputs': [
                'row-preflight.json',
                'execution.json',
            ],
        },
        'authority': {'trial_executed': attempted},
    }


def _write_fake_handoff(directory: pathlib.Path) -> dict[str, object]:
    directory.mkdir()
    for name in SESSION.EXPECTED_HANDOFF_ENTRIES:
        path = directory / name
        if name == 'artifacts':
            path.mkdir()
        else:
            path.write_text('{}\n', encoding='utf-8')
    preparation = _preparation()
    (directory / 'preparation.json').write_text(
        json.dumps(preparation) + '\n', encoding='utf-8'
    )
    return preparation


def _write_fake_execution(
    directory: pathlib.Path,
    receipt: dict[str, object],
) -> None:
    directory.mkdir()
    (directory / 'row-preflight.json').write_text('{}\n', encoding='utf-8')
    if receipt['authority']['trial_executed']:
        (directory / 'private').mkdir()
    (directory / 'execution.json').write_text(
        json.dumps(receipt) + '\n', encoding='utf-8'
    )


def _install_success(
    monkeypatch,
    *,
    execution: dict[str, object] | None = None,
    exit_code: int = 0,
):
    calls = []
    selected = execution or _execution()

    def prepare(url, directory, **kwargs):
        calls.append(('prepare', url, directory, kwargs))
        return _write_fake_handoff(directory)

    def run(handoff, row_id, directory, **kwargs):
        calls.append(('run', handoff, row_id, directory, kwargs))
        _write_fake_execution(directory, selected)
        return selected, exit_code

    monkeypatch.setattr(SESSION, 'prepare_candidate_trial', prepare)
    monkeypatch.setattr(SESSION, 'run_candidate_trial', run)
    return calls


def test_one_command_publishes_handoff_execution_and_bound_session(
    monkeypatch,
    tmp_path,
):
    """The public directory appears only after both delegated stages pass."""
    calls = _install_success(monkeypatch)
    output = tmp_path / 'candidate-session'

    receipt, exit_code = SESSION.start_candidate_trial(
        RUN_URL,
        'docker-jazzy',
        output,
        acknowledge_dedicated_trial_host=True,
        interactive=False,
        started_at='2026-08-15T12:00:00Z',
    )

    assert exit_code == 0
    assert receipt['status'] == 'TRIAL_RECORDED'
    assert receipt['execution']['outcome_status'] == 'PASS'
    assert receipt['execution']['comparable'] is True
    assert receipt['authority']['docker_observer_bootstrap_requested'] is True
    assert receipt['authority']['remote_mutations_performed'] is False
    assert sorted(item.name for item in output.iterdir()) == [
        'execution', 'handoff', 'session.json',
    ]
    assert json.loads((output / 'session.json').read_text()) == receipt
    assert len(receipt['handoff']['preparation_sha256']) == 64
    assert len(receipt['execution']['execution_sha256']) == 64
    assert calls[0][0] == 'prepare'
    assert calls[1][0] == 'run'
    assert calls[1][4]['interactive'] is False
    assert not list(tmp_path.glob('.candidate-session.session-*'))


def test_valid_product_fail_is_preserved_as_a_complete_session(
    monkeypatch,
    tmp_path,
):
    """A real product failure remains evidence instead of disappearing."""
    observed = _execution(outcome='FAIL', complete=False)
    _install_success(monkeypatch, execution=observed, exit_code=1)
    output = tmp_path / 'candidate-session'

    receipt, exit_code = SESSION.start_candidate_trial(
        RUN_URL,
        'docker-jazzy',
        output,
        acknowledge_dedicated_trial_host=True,
        interactive=False,
    )

    assert exit_code == 1
    assert receipt['status'] == 'TRIAL_RECORDED'
    assert receipt['execution']['outcome_status'] == 'FAIL'
    assert receipt['execution']['comparable'] is False
    assert output.is_dir()


def test_preflight_block_is_published_without_claiming_a_trial(
    monkeypatch,
    tmp_path,
):
    """A blocked live identity check produces a bounded terminal session."""
    blocked = _execution(
        status='PREFLIGHT_BLOCKED', outcome=None, complete=False
    )
    _install_success(monkeypatch, execution=blocked, exit_code=1)

    receipt, exit_code = SESSION.start_candidate_trial(
        RUN_URL,
        'docker-jazzy',
        tmp_path / 'candidate-session',
        acknowledge_dedicated_trial_host=True,
        interactive=False,
    )

    assert exit_code == 1
    assert receipt['status'] == 'PREFLIGHT_BLOCKED'
    assert receipt['authority']['trial_executed'] is False
    assert receipt['sharing']['private_evidence_directory'] is None


def test_source_session_never_claims_docker_bootstrap(
    monkeypatch,
    tmp_path,
):
    """The umbrella receipt preserves route-specific authority."""
    observed = _execution(route='source', distro='humble')
    _install_success(monkeypatch, execution=observed)

    receipt, _ = SESSION.start_candidate_trial(
        RUN_URL,
        'source-humble',
        tmp_path / 'candidate-session',
        acknowledge_dedicated_trial_host=True,
        interactive=False,
    )

    assert receipt['row']['route'] == 'source'
    assert receipt['authority']['docker_observer_bootstrap_requested'] is False


def test_preparation_failure_removes_outer_staging(monkeypatch, tmp_path):
    """An unauthenticated handoff never leaves a session-shaped output."""
    def fail(*_args, **_kwargs):
        raise SESSION.CandidateTrialPreparationError('artifact mismatch')

    monkeypatch.setattr(SESSION, 'prepare_candidate_trial', fail)
    output = tmp_path / 'candidate-session'

    with pytest.raises(
        SESSION.CandidateTrialSessionError,
        match='preparation failed',
    ):
        SESSION.start_candidate_trial(
            RUN_URL,
            'docker-jazzy',
            output,
            acknowledge_dedicated_trial_host=True,
        )

    assert not output.exists()
    assert not list(tmp_path.glob('.candidate-session.session-*'))


def test_execution_contract_failure_removes_prepared_staging(
    monkeypatch,
    tmp_path,
):
    """Pre-evidence argument/identity errors do not publish half a session."""
    def prepare(_url, directory, **_kwargs):
        return _write_fake_handoff(directory)

    def fail(*_args, **_kwargs):
        raise SESSION.CandidateTrialExecutionError('unsafe interface')

    monkeypatch.setattr(SESSION, 'prepare_candidate_trial', prepare)
    monkeypatch.setattr(SESSION, 'run_candidate_trial', fail)
    output = tmp_path / 'candidate-session'

    with pytest.raises(
        SESSION.CandidateTrialSessionError,
        match='before evidence publication',
    ):
        SESSION.start_candidate_trial(
            RUN_URL,
            'source-jazzy',
            output,
            acknowledge_dedicated_trial_host=True,
        )

    assert not output.exists()
    assert not list(tmp_path.glob('.candidate-session.session-*'))


def test_retained_child_receipt_mismatch_removes_session(
    monkeypatch, tmp_path
):
    """The umbrella receipt derives from retained bytes, not return values."""
    selected = _execution()

    def prepare(_url, directory, **_kwargs):
        return _write_fake_handoff(directory)

    def run(_handoff, _row_id, directory, **_kwargs):
        _write_fake_execution(directory, selected)
        changed = dict(selected)
        changed['status'] = 'HARNESS_ERROR'
        return changed, 2

    monkeypatch.setattr(SESSION, 'prepare_candidate_trial', prepare)
    monkeypatch.setattr(SESSION, 'run_candidate_trial', run)
    output = tmp_path / 'candidate-session'

    with pytest.raises(
        SESSION.CandidateTrialSessionError,
        match='retained execution receipt differs',
    ):
        SESSION.start_candidate_trial(
            RUN_URL,
            'docker-jazzy',
            output,
            acknowledge_dedicated_trial_host=True,
        )

    assert not output.exists()
    assert not list(tmp_path.glob('.candidate-session.session-*'))


def test_acknowledgement_and_existing_output_fail_before_delegation(
    monkeypatch,
    tmp_path,
):
    """Dangerous or destructive requests stop before network reads."""
    def unexpected(*_args, **_kwargs):
        raise AssertionError('request validation delegated work')

    monkeypatch.setattr(SESSION, 'prepare_candidate_trial', unexpected)
    output = tmp_path / 'candidate-session'
    with pytest.raises(
        SESSION.CandidateTrialSessionError,
        match='acknowledge',
    ):
        SESSION.start_candidate_trial(
            RUN_URL,
            'docker-jazzy',
            output,
            acknowledge_dedicated_trial_host=False,
        )

    output.mkdir()
    with pytest.raises(
        SESSION.CandidateTrialSessionError,
        match='overwrite',
    ):
        SESSION.start_candidate_trial(
            RUN_URL,
            'docker-jazzy',
            output,
            acknowledge_dedicated_trial_host=True,
        )


def test_invalid_row_options_fail_before_handoff_download(
    monkeypatch, tmp_path
):
    """A local typo cannot trigger candidate artifact network reads."""
    def unexpected(*_args, **_kwargs):
        raise AssertionError('invalid row options delegated preparation')

    monkeypatch.setattr(SESSION, 'prepare_candidate_trial', unexpected)

    with pytest.raises(
        SESSION.CandidateTrialSessionError,
        match='only for a source row',
    ):
        SESSION.start_candidate_trial(
            RUN_URL,
            'docker-jazzy',
            tmp_path / 'candidate-session-a',
            acknowledge_dedicated_trial_host=True,
            network_interface='eth0',
        )
    with pytest.raises(
        SESSION.CandidateTrialSessionError,
        match='interactive terminal',
    ):
        SESSION.start_candidate_trial(
            RUN_URL,
            'source-jazzy',
            tmp_path / 'candidate-session-b',
            acknowledge_dedicated_trial_host=True,
            human_measurements='prompt',
            interactive=False,
        )


def test_json_cli_stdout_is_one_parseable_session(
    monkeypatch, tmp_path, capsys
):
    """Automation receives one JSON document and no child receipt noise."""
    receipt = {'status': 'TRIAL_RECORDED', 'row': {'row_id': 'docker-jazzy'}}
    monkeypatch.setattr(
        SESSION,
        'start_candidate_trial',
        lambda *_args, **_kwargs: (receipt, 0),
    )

    result = SESSION.main([
        '--workflow-run-url', RUN_URL,
        '--row', 'docker-jazzy',
        '--output-dir', str(tmp_path / 'candidate-session'),
        '--acknowledge-dedicated-trial-host',
        '--json',
    ])

    captured = capsys.readouterr()
    assert result == 0
    assert json.loads(captured.out) == receipt
    assert captured.err == ''
