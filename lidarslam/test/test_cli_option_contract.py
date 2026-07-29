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
import runpy
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
CLI = REPO_ROOT / 'scripts' / 'lidarslam'
CONTRACT_PATH = REPO_ROOT / 'docs' / 'contracts' / 'cli-v1.json'
PROFILE_REGISTRY_PATH = REPO_ROOT / 'scripts' / 'product_profiles.py'
OPTION_PATTERN = re.compile(r'(?<![A-Za-z0-9-])(--[a-z][a-z0-9-]*|-h)(?![A-Za-z0-9-])')
VALUE_OPTION_PATTERN = re.compile(
    r'(?P<option>--[a-z][a-z0-9-]*)[ =]'
    r'(?P<value>\{[^}\n]+\}|<[^>\n]+>|[A-Z][A-Z0-9_]*)'
)


def _contract() -> dict[str, object]:
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


def _option_names(entries: list[dict[str, object]]) -> set[str]:
    return {
        name
        for entry in entries
        for name in entry['names']
    }


def _help(command: str, *, all_options: bool = False) -> str:
    help_option = '--help-all' if all_options else '--help'
    completed = subprocess.run(
        [str(CLI), command, help_option],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    return completed.stdout


def _rendered_value_options(rendered_help: str) -> dict[str, str]:
    option_lines = '\n'.join(
        line
        for line in rendered_help.splitlines()
        if line.lstrip().startswith('-')
    )
    return {
        match.group('option'): match.group('value')
        for match in VALUE_OPTION_PATTERN.finditer(option_lines)
    }


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
    assert set(contract['commands']) == {
        'doctor',
        'run',
        'inspect',
        'view',
        'migrate-manifest',
        'rollback-plan',
    }
    assert _option_names(contract['global_options']) == {
        '-h',
        '--help',
        '--help-all',
        '--version',
    }


def test_recovery_commands_are_kept_out_of_beginner_help():
    """Recovery tools should be explicit without enlarging the golden path."""
    normal = subprocess.run(
        [str(CLI), '--help'],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    complete = subprocess.run(
        [str(CLI), '--help-all'],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert normal.returncode == 0
    assert complete.returncode == 0
    for command in ('migrate-manifest', 'rollback-plan'):
        assert command not in normal.stdout
        assert command in complete.stdout


def test_each_subcommand_help_matches_its_option_inventory():
    """Both help levels should match the machine-readable visibility rules."""
    contract = _contract()
    exclusions = contract['help_modes']['normal']['excludes']

    for command, command_contract in contract['commands'].items():
        rendered_all = _help(command, all_options=True)
        all_option_lines = '\n'.join(
            line
            for line in rendered_all.splitlines()
            if line.lstrip().startswith('-')
        )
        rendered_all_options = set(OPTION_PATTERN.findall(all_option_lines))
        contract_options = _option_names(command_contract['options'])

        assert rendered_all_options == contract_options, command
        assert command_contract['positional']['name'] in rendered_all

        rendered_normal = _help(command)
        normal_option_lines = '\n'.join(
            line
            for line in rendered_normal.splitlines()
            if line.lstrip().startswith('-')
        )
        rendered_normal_options = set(
            OPTION_PATTERN.findall(normal_option_lines)
        )
        expected_normal_options = _option_names([
            entry
            for entry in command_contract['options']
            if entry['stability'] not in exclusions['stability']
            and entry['tier'] not in exclusions['tiers']
        ])
        assert rendered_normal_options == expected_normal_options, command


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


def test_value_contract_matches_full_help_and_bounded_choices():
    """Value-taking options must publish their type, default and metavar."""
    contract = _contract()

    assert set(contract['value_options']) == set(contract['commands'])
    for command, command_contract in contract['commands'].items():
        value_contract = contract['value_options'][command]
        rendered_values = _rendered_value_options(
            _help(command, all_options=True)
        )
        assert set(value_contract) == set(rendered_values), command

        declared_options = _option_names(command_contract['options'])
        for option, value in value_contract.items():
            assert option in declared_options
            assert value['kind'] in {
                'directory',
                'enum',
                'integer',
                'number',
                'file',
            }
            assert 'default' in value
            assert value['value_name'] == rendered_values[option]
            if value['kind'] == 'enum':
                assert value['choices']
                if value['value_name'].startswith('{'):
                    assert (
                        '{' + ','.join(value['choices']) + '}'
                        == value['value_name']
                    )
            else:
                assert 'choices' not in value


def test_profile_option_and_help_use_the_maintained_registry():
    """Keep profile choices and descriptions in one authoritative source."""
    contract = _contract()
    registry = runpy.run_path(str(PROFILE_REGISTRY_PATH))
    profile_ids = tuple(registry['PROFILE_IDS'])
    profile_help = tuple(registry['PROFILE_HELP'])

    assert profile_ids
    assert len(profile_ids) == len(set(profile_ids))
    assert tuple(
        contract['value_options']['run']['--profile']['choices']
    ) == profile_ids
    for command in ('doctor', 'run'):
        rendered = _help(command)
        for profile_id, description in profile_help:
            assert f'{profile_id}: {description}' in rendered


def test_positionals_and_deprecation_lifecycle_are_machine_readable():
    """Directory requirements and the warning/removal policy are explicit."""
    contract = _contract()
    policy = contract['deprecation_policy']

    assert policy == {
        'minimum_compatibility_window': 'one minor release',
        'removal_status': 'not_scheduled',
        'warning_channel': 'stderr',
        'warning_count_per_invocation': 1,
    }
    for command in contract['commands'].values():
        positional = command['positional']
        assert positional['kind'] in {'directory', 'file'}
        assert positional['stability'] == 'stable'
        if 'must_contain' in positional:
            assert positional['must_contain']
            assert all(positional['must_contain'])


@pytest.mark.parametrize(
    ('arguments', 'message'),
    [
        (['unknown'], 'unknown command'),
        (['doctor', '/tmp', '--unknown'], 'unrecognized arguments'),
        (
            ['run', '/tmp', '--min-free-space-gib', '0'],
            'finite number greater than zero',
        ),
        (
            ['run', '/tmp', '--auto-exit-secs', '0'],
            'positive integer',
        ),
        (
            ['run', '/tmp', '--verification', 'maybe'],
            'invalid choice',
        ),
        (
            [
                'run',
                '/tmp',
                '--verification',
                'off',
                '--no-verify-map',
            ],
            'cannot be combined',
        ),
        (
            ['run', '/tmp', '--resume', '--dry-run'],
            'cannot be combined',
        ),
        (
            ['run', '--help-all', '--dry-run'],
            'cannot be combined',
        ),
    ],
)
def test_invalid_options_have_stable_usage_exit(
    arguments: list[str],
    message: str,
):
    """Invalid names, values, and combinations should consistently exit 2."""
    completed = subprocess.run(
        [str(CLI), *arguments],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert message in completed.stderr
