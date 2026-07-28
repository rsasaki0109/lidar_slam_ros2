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

"""Tests for the installed Bash completion contract."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
COMPLETION = (
    REPO_ROOT / 'scripts' / 'completions' / 'lidarslam-map.bash'
)
CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'cli-v1.json'


def _complete(*words: str) -> set[str]:
    word_array = ' '.join(f'"{word}"' for word in words)
    script = f"""
source "$1"
COMP_WORDS=({word_array})
COMP_CWORD=$((${{#COMP_WORDS[@]}} - 1))
_lidarslam_map_complete
printf '%s\\n' "${{COMPREPLY[@]}}"
"""
    completed = subprocess.run(
        ['bash', '-c', script, 'bash', str(COMPLETION)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    return set(completed.stdout.splitlines())


def test_completion_is_valid_bash_and_covers_the_option_manifest():
    """Every public long option should remain discoverable in completion."""
    syntax = subprocess.run(
        ['bash', '-n', str(COMPLETION)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr

    contract = json.loads(CONTRACT.read_text(encoding='utf-8'))
    completion_text = COMPLETION.read_text(encoding='utf-8')
    for command, command_contract in contract['commands'].items():
        assert command in completion_text
        for option in command_contract['options']:
            for name in option['names']:
                if name.startswith('--'):
                    assert name in completion_text


def test_completion_suggests_commands_options_and_bounded_choices():
    """Completion should understand command context and finite option values."""
    assert {'doctor', 'run', 'inspect', 'view'} <= _complete(
        'lidarslam-map',
        '',
    )
    assert _complete('lidarslam-map', 'run', '--ver') == {'--verification'}
    assert _complete(
        'lidarslam-map',
        'run',
        '--verification',
        '',
    ) == {'required', 'off'}
    assert _complete(
        'lidarslam-map',
        'run',
        '--viewer',
        '',
    ) == {'none', 'autoware', 'foxglove'}
    assert _complete(
        'lidarslam-map',
        'view',
        '--viewer',
        '',
    ) == {'autoware', 'foxglove'}
