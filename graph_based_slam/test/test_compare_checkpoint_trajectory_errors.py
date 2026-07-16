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


"""Tests for per-checkpoint trajectory gap reports."""

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
SCRIPT = ROOT / 'scripts' / 'compare_checkpoint_trajectory_errors.py'
SPEC = importlib.util.spec_from_file_location('checkpoint_errors', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _write(path: Path, rows):
    path.write_text(''.join(
        f'{t} {x} {y} {z} 0 0 0 1\n' for t, x, y, z in rows))


def test_reports_each_common_checkpoint_and_largest_deficit(tmp_path):
    ref = tmp_path / 'ref.tum'
    ours = tmp_path / 'ours.tum'
    rival = tmp_path / 'rival.tum'
    rows = [(0, 0, 0, 0), (1, 1, 0, 0), (2, 1, 1, 0), (3, 2, 1, 1)]
    _write(ref, rows)
    _write(rival, rows)
    _write(ours, [(0, 0, 0, 0), (1, 1, 0.2, 0),
                  (2, 1, 1.3, 0), (3, 2, 1, 1)])
    result = MODULE.evaluate(ref, ours, rival, 0.1)
    assert result['checkpoint_count'] == 4
    assert result['aggregate']['rival_wins'] > 0
    assert result['aggregate']['largest_ours_deficit_m'] > 0
    assert len(result['checkpoints']) == 4


def test_rejects_trajectory_that_does_not_cover_reference(tmp_path):
    ref = tmp_path / 'ref.tum'
    ours = tmp_path / 'ours.tum'
    rival = tmp_path / 'rival.tum'
    _write(ref, [(0, 0, 0, 0), (10, 1, 0, 0), (20, 2, 0, 0)])
    _write(ours, [(0, 0, 0, 0), (1, 1, 0, 0)])
    _write(rival, [(0, 0, 0, 0), (10, 1, 0, 0), (20, 2, 0, 0)])
    try:
        MODULE.evaluate(ref, ours, rival, 0.1)
    except ValueError as error:
        assert 'does not cover' in str(error)
    else:
        raise AssertionError('uncovered checkpoint was accepted')
