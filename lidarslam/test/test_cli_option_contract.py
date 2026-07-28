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

"""Keep the published CLI option inventory synchronized with command help."""

from __future__ import annotations

import json
import re
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
CLI = REPO_ROOT / 'scripts' / 'lidarslam'
CONTRACT_PATH = REPO_ROOT / 'docs' / 'contracts' / 'cli-v1.json'
OPTION_PATTERN = re.compile(r'(?<![A-Za-z0-9-])(--[a-z][a-z0-9-]*|-h)(?![A-Za-z0-9-])')


def _contract() -> dict[str, object]:
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


def _option_names(entries: list[dict[str, object]]) -> set[str]:
    return {
        name
        for entry in entries
        for name in entry['names']
    }


def _help(*args: str) -> str:
    completed = subprocess.run(
        [str(CLI), *args, '--help'],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    return completed.stdout


def test_contract_identifies_the_complete_product_surface():
    """The manifest should enumerate every product subcommand and global flag."""
    contract = _contract()

    assert contract['schema_version'] == 1
    assert contract['contract_id'] == 'lidarslam-map-v1'
    assert contract['product_command'] == 'lidarslam-map'
    assert contract['exit_codes'] == {
        '0': 'command completed successfully',
        '2': 'invalid usage, input, profile, or output path',
        '70': 'internal or tooling error prevented command startup',
        'other_nonzero': 'delegated workflow or viewer failure',
    }
    assert set(contract['commands']) == {'doctor', 'run', 'inspect', 'view'}
    assert _option_names(contract['global_options']) == {
        '-h',
        '--help',
        '--version',
    }


def test_each_subcommand_help_matches_its_option_inventory():
    """No public flag may appear without an inventory entry."""
    contract = _contract()

    for command, command_contract in contract['commands'].items():
        rendered = _help(command)
        option_lines = '\n'.join(
            line
            for line in rendered.splitlines()
            if line.lstrip().startswith('-')
        )
        rendered_options = set(OPTION_PATTERN.findall(option_lines))
        contract_options = _option_names(command_contract['options'])

        assert rendered_options == contract_options, command
        assert command_contract['positional']['name'] in rendered


def test_stability_and_tier_values_are_explicit():
    """Every flag should carry a bounded stability state and operator tier."""
    contract = _contract()
    entries = list(contract['global_options'])
    for command in contract['commands'].values():
        entries.extend(command['options'])
        assert command['positional']['stability'] == 'stable'

    assert {entry['stability'] for entry in entries} <= {
        'stable',
        'provisional',
        'deprecated',
    }
    assert all(entry['tier'] for entry in entries)
    deprecated = [
        entry for entry in entries if entry['stability'] == 'deprecated'
    ]
    assert deprecated
    assert all(entry['replacement'] for entry in deprecated)
