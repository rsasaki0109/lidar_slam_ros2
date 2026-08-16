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

"""Verify that the TTY home reduces choice without changing automation."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
CLI_MODULE_PATH = REPO_ROOT / 'scripts' / 'lidarslam_cli.py'


@pytest.fixture
def cli_module():
    spec = importlib.util.spec_from_file_location(
        'lidarslam_cli_interactive_test', CLI_MODULE_PATH
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _set_answers(monkeypatch, answers):
    responses = iter(answers)
    monkeypatch.setattr('builtins.input', lambda _prompt='': next(responses))


def _capture_delegate(monkeypatch, cli_module, returncode=0):
    captured = {}

    def fake_run(command, **kwargs):
        captured['command'] = command
        captured['kwargs'] = kwargs
        return SimpleNamespace(returncode=returncode)

    monkeypatch.setattr(cli_module.subprocess, 'run', fake_run)
    return captured


def test_noninteractive_no_argument_behavior_remains_usage_error(
    monkeypatch, capsys, cli_module
):
    monkeypatch.setattr(cli_module, '_interactive_terminal', lambda: False)

    assert cli_module.main([]) == 2

    captured = capsys.readouterr()
    assert captured.out == ''
    assert 'Usage:' in captured.err
    assert '<command>' in captured.err


def test_demo_requires_explicit_write_confirmation(
    monkeypatch, capsys, cli_module
):
    monkeypatch.setattr(cli_module, '_interactive_terminal', lambda: True)
    _set_answers(monkeypatch, ['1', '', ''])

    def unexpected_run(*_args, **_kwargs):
        raise AssertionError('cancelled home must not delegate')

    monkeypatch.setattr(cli_module.subprocess, 'run', unexpected_run)

    assert cli_module.main([]) == 0

    output = capsys.readouterr().out
    assert '517 MB' in output
    assert '8 GiB' in output
    assert 'Next command: ./scripts/lidarslam demo' in output
    assert 'No changes made' in output


def test_confirmed_demo_delegates_to_the_existing_command(
    monkeypatch, cli_module
):
    monkeypatch.setattr(cli_module, '_interactive_terminal', lambda: True)
    _set_answers(monkeypatch, ['1', '/tmp/map work', 'yes'])
    captured = _capture_delegate(monkeypatch, cli_module, returncode=19)

    assert cli_module.main([]) == 19

    command = captured['command']
    assert Path(command[1]).name == 'first_map_demo.py'
    assert command[2:] == ['/tmp/map work']
    assert captured['kwargs']['env']['LIDARSLAM_CLI_COMMAND'] == (
        './scripts/lidarslam demo'
    )


def test_own_bag_home_reuses_start_and_its_sensor_review(
    monkeypatch, capsys, cli_module
):
    monkeypatch.setattr(cli_module, '_interactive_terminal', lambda: True)
    _set_answers(monkeypatch, ['2', '/tmp/my bag', ''])
    captured = _capture_delegate(monkeypatch, cli_module)

    assert cli_module.main([]) == 0

    command = captured['command']
    assert Path(command[1]).name == 'sensor_setup_wizard.py'
    assert command[2:] == ['--run', '/tmp/my bag']
    output = capsys.readouterr().out
    assert "'/tmp/my bag'" in output
    assert 'No launch-file or YAML edits are needed' in output
    assert 'profile, and calibration are reviewed before mapping' in output


def test_sessions_home_reuses_the_local_catalog(
    monkeypatch, cli_module
):
    monkeypatch.setattr(cli_module, '_interactive_terminal', lambda: True)
    _set_answers(monkeypatch, ['3', '', 'yes'])
    captured = _capture_delegate(monkeypatch, cli_module)

    assert cli_module.main([]) == 0

    command = captured['command']
    assert Path(command[1]).name == 'session_history.py'
    assert command[2:] == []


def test_installation_check_runs_without_confirmation_or_writes(
    monkeypatch, capsys, cli_module
):
    monkeypatch.setattr(cli_module, '_interactive_terminal', lambda: True)
    _set_answers(monkeypatch, ['4'])
    captured = _capture_delegate(monkeypatch, cli_module)

    assert cli_module.main([]) == 0

    command = captured['command']
    assert Path(command[1]).name == 'lidarslam_doctor.py'
    assert command[2:] == []
    output = capsys.readouterr().out
    assert 'Next command: ./scripts/lidarslam doctor' in output
    assert 'uses no network and writes no files' in output


def test_home_help_and_quit_never_delegate(
    monkeypatch, capsys, cli_module
):
    monkeypatch.setattr(cli_module, '_interactive_terminal', lambda: True)

    def unexpected_run(*_args, **_kwargs):
        raise AssertionError('help and quit must not delegate')

    monkeypatch.setattr(cli_module.subprocess, 'run', unexpected_run)

    _set_answers(monkeypatch, ['5'])
    assert cli_module.main([]) == 0
    assert 'Fastest verified first map:' in capsys.readouterr().out

    _set_answers(monkeypatch, ['q'])
    assert cli_module.main([]) == 0
    assert 'No changes made.' in capsys.readouterr().out


def test_home_bounds_invalid_answers_and_handles_interrupt(
    monkeypatch, capsys, cli_module
):
    monkeypatch.setattr(cli_module, '_interactive_terminal', lambda: True)
    _set_answers(monkeypatch, ['bad', 'still-bad', '?'])

    assert cli_module.main([]) == 2
    assert 'No workflow selected.' in capsys.readouterr().out

    def interrupt(_prompt=''):
        raise KeyboardInterrupt

    monkeypatch.setattr('builtins.input', interrupt)
    assert cli_module.main([]) == 130
    assert 'Cancelled; no changes made.' in capsys.readouterr().out
