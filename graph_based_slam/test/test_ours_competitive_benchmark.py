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


"""Unit tests for the in-workspace competitive benchmark determinism gate."""

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
SPEC = importlib.util.spec_from_file_location(
    'run_ours_competitive_benchmark',
    ROOT / 'scripts/run_ours_competitive_benchmark.py')
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


def report(trajectory_hash):
    """Build the minimal report shape consumed by the gate helper."""
    return {'trajectory': {'sha256': trajectory_hash}}


def test_trajectory_determinism_accepts_identical_runs():
    """Three identical hashes satisfy the byte-identity contract."""
    result = RUNNER.trajectory_determinism([
        report('a' * 64), report('a' * 64), report('a' * 64)])
    assert result == {
        'comparable_runs': 3,
        'trajectory_byte_identical': True,
        'trajectory_sha256_by_run': ['a' * 64, 'a' * 64, 'a' * 64],
    }


def test_trajectory_determinism_rejects_mismatch_or_missing_artifact():
    """A mismatch or absent artifact must fail the byte-identity contract."""
    mismatch = RUNNER.trajectory_determinism([
        report('a' * 64), report('b' * 64)])
    assert mismatch['trajectory_byte_identical'] is False
    assert mismatch['comparable_runs'] == 2

    missing = RUNNER.trajectory_determinism([report('a' * 64), report(None)])
    assert missing['trajectory_byte_identical'] is False
    assert missing['comparable_runs'] == 0
