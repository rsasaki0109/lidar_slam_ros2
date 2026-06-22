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
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
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

"""CLI regression tests for the public Autoware quickstart wrapper."""

from __future__ import annotations

from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
QUICKSTART_SCRIPT = REPO_ROOT / 'scripts' / 'run_autoware_quickstart.sh'


def _run_quickstart(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['bash', str(QUICKSTART_SCRIPT), *args],
        check=False,
        capture_output=True,
        text=True,
    )


def test_quickstart_help_exits_successfully():
    result = _run_quickstart('--help')

    assert result.returncode == 0
    assert 'run_autoware_quickstart.sh' in result.stderr
    assert 'dogfood' in result.stderr
    assert 'existing <graph_slam_output_dir>' in result.stderr


def test_quickstart_existing_requires_source_directory():
    result = _run_quickstart('existing')

    assert result.returncode == 2
    assert 'error: existing requires <graph_slam_output_dir>.' in result.stderr


def test_quickstart_existing_rejects_missing_source_directory(tmp_path: Path):
    result = _run_quickstart('existing', str(tmp_path / 'missing_output'))

    assert result.returncode == 2
    assert 'error: graph_slam_output_dir does not exist' in result.stderr
    assert 'run_rko_lio_graph_autoware_dogfood.sh' not in result.stderr


def test_quickstart_missing_path_is_not_treated_as_dogfood_option(tmp_path: Path):
    result = _run_quickstart(str(tmp_path / 'missing_output'), '--dry-run')

    assert result.returncode == 2
    assert 'error: graph_slam_output_dir does not exist' in result.stderr
    assert 'Unknown option:' not in result.stderr
