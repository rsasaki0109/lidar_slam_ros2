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

"""Tests for scripts/summarize_degeneracy_csv.py (v0.8 Phase 1 report-only stage)."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'summarize_degeneracy_csv.py'

HEADER = (
    'stamp_sec,diagnostics_available,'
    'eigenvalue_0,eigenvalue_1,eigenvalue_2,eigenvalue_3,eigenvalue_4,eigenvalue_5,'
    'category_0,category_1,category_2,category_3,category_4,category_5,'
    'well_conditioned_count,degenerate_count,non_observable_count,condition_number,'
    + ','.join(
        f'eigenvector_{i}_{axis}'
        for i in range(6)
        for axis in ('tx', 'ty', 'tz', 'rx', 'ry', 'rz'))
)


def _row(stamp: float, available: bool, well: int, degen: int, non_obs: int) -> str:
    """Build one synthetic CSV data row matching degeneracy_diagnostics_csv.hpp."""
    if not available:
        return f'{stamp},0' + ',' * 52
    eigenvalues = ','.join(str(10.0 * (i + 1)) for i in range(6))
    categories = []
    categories += ['NON_OBSERVABLE'] * non_obs
    categories += ['DEGENERATE'] * degen
    categories += ['WELL_CONDITIONED'] * well
    eigenvector = ','.join(('1,0,0,0,0,0'.split(',') * 6))
    return (
        f'{stamp},1,{eigenvalues},' + ','.join(categories) +
        f',{well},{degen},{non_obs},123.4,{eigenvector}'
    )


def _write_csv(path: Path, rows: list[str]) -> None:
    """Write a synthetic diagnostics CSV."""
    path.write_text('\n'.join([HEADER] + rows) + '\n', encoding='utf-8')


def _run(args: list[str]) -> subprocess.CompletedProcess:
    """Run the summarizer CLI."""
    return subprocess.run(
        [sys.executable, str(SCRIPT)] + args,
        capture_output=True, text=True, check=False,
    )


def test_summary_counts_rates_and_worst_interval(tmp_path: Path) -> None:
    """Counts, availability, rates and the longest interval match the fixture."""
    csv_path = tmp_path / 'diag.csv'
    _write_csv(csv_path, [
        _row(0.0, False, 0, 0, 0),
        _row(1.0, True, 6, 0, 0),
        _row(2.0, True, 5, 1, 0),
        _row(3.0, True, 5, 1, 0),
        _row(4.0, True, 3, 0, 3),
        _row(5.0, True, 6, 0, 0),
        _row(6.0, True, 5, 1, 0),
    ])
    json_path = tmp_path / 'summary.json'
    result = _run(['--csv', str(csv_path), '--write-json', str(json_path)])
    assert result.returncode == 0, result.stderr

    data = json.loads(json_path.read_text(encoding='utf-8'))
    assert data['total_scans'] == 7
    assert data['diagnostics_available_scans'] == 6
    assert data['well_conditioned_scans'] == 2
    assert data['degenerate_scans'] == 3
    assert data['non_observable_scans'] == 1
    assert abs(data['degenerate_ratio'] - 0.5) < 1e-9
    interval = data['worst_interval']
    assert interval['valid'] is True
    assert interval['length_scans'] == 3
    assert interval['start_stamp_sec'] == 2.0
    assert interval['end_stamp_sec'] == 4.0
    assert interval['category'] == 'NON_OBSERVABLE'


def test_markdown_output_written(tmp_path: Path) -> None:
    """--write-md produces a Markdown report with the category table."""
    csv_path = tmp_path / 'diag.csv'
    _write_csv(csv_path, [_row(1.0, True, 6, 0, 0)])
    md_path = tmp_path / 'summary.md'
    result = _run(['--csv', str(csv_path), '--write-md', str(md_path)])
    assert result.returncode == 0, result.stderr
    text = md_path.read_text(encoding='utf-8')
    assert 'Degeneracy diagnostics summary' in text
    assert '| WELL_CONDITIONED | 1 | 100.0% |' in text
    assert 'worst interval: none' in text


def test_missing_csv_fails_with_error(tmp_path: Path) -> None:
    """A missing CSV is a usability error (the readiness stage stays report-only)."""
    result = _run(['--csv', str(tmp_path / 'nope.csv')])
    assert result.returncode == 1
    assert 'not found' in result.stderr


def test_empty_csv_fails_with_error(tmp_path: Path) -> None:
    """A header-only CSV has no data rows and is rejected."""
    csv_path = tmp_path / 'diag.csv'
    _write_csv(csv_path, [])
    result = _run(['--csv', str(csv_path)])
    assert result.returncode == 1
    assert 'no data rows' in result.stderr


def test_wrong_header_fails_with_error(tmp_path: Path) -> None:
    """A CSV with an unexpected column count is rejected up front."""
    csv_path = tmp_path / 'diag.csv'
    csv_path.write_text('a,b,c\n1,2,3\n', encoding='utf-8')
    result = _run(['--csv', str(csv_path)])
    assert result.returncode == 1
    assert 'unexpected CSV header' in result.stderr
