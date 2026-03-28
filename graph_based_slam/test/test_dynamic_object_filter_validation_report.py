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

"""Regression tests for the dynamic-object-filter validation report generator."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'generate_dynamic_object_filter_validation_report.py'


def _write_benchmark(
    root: Path,
    *,
    baseline: int,
    filtered: int,
    reduction: float,
    kept: float,
    removed: float,
    cells: tuple[int, int],
) -> None:
    root.mkdir(parents=True, exist_ok=True)
    payload = {
        'baseline': {
            'total_pcd_points': baseline,
            'saved_cell_count': cells[0],
            'verify_result': 'PASS',
            'projector_type': 'LocalCartesian',
        },
        'filtered': {
            'total_pcd_points': filtered,
            'saved_cell_count': cells[1],
            'verify_result': 'PASS',
            'projector_type': 'LocalCartesian',
        },
        'point_reduction_ratio': reduction,
        'kept_candidate_voxel_ratio': kept,
        'removed_candidate_voxel_ratio': removed,
        'shared_metadata_tiles': 120,
        'tile_jaccard': 0.95,
        'filtered_tile_overlap_ratio': 0.98,
        'baseline_tile_overlap_ratio': 0.94,
    }
    (root / 'dynamic_object_filter_report.json').write_text(
        json.dumps(payload, indent=2) + '\n',
        encoding='utf-8',
    )


def test_dynamic_object_filter_validation_report_summarizes_multiple_benchmarks(tmp_path):
    """The validation report should compare multiple dynamic-filter runs."""
    bag1 = tmp_path / 'bag1'
    bag6 = tmp_path / 'bag6'
    out_md = tmp_path / 'validation.md'
    out_json = tmp_path / 'validation.json'
    out_svg = tmp_path / 'validation.svg'

    _write_benchmark(
        bag1,
        baseline=542947,
        filtered=268237,
        reduction=0.505961,
        kept=0.573084,
        removed=0.426916,
        cells=(176, 176),
    )
    _write_benchmark(
        bag6,
        baseline=344922,
        filtered=172332,
        reduction=0.500374,
        kept=0.448413,
        removed=0.551587,
        cells=(161, 169),
    )

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--benchmark',
            f'Leo Drive bag1={bag1}',
            '--benchmark',
            f'Leo Drive bag6={bag6}',
            '--out',
            str(out_md),
            '--write-json',
            str(out_json),
            '--write-svg',
            str(out_svg),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    report = out_md.read_text(encoding='utf-8')
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert 'Leo Drive bag1' in report
    assert 'Leo Drive bag6' in report
    assert 'Best point reduction' in report
    assert 'Tile jaccard' in report
    assert payload['best_point_reduction_label'] == 'Leo Drive bag1'
    assert payload['most_conservative_removed_ratio_label'] == 'Leo Drive bag1'
    assert payload['benchmarks'][0]['tile_jaccard'] == 0.95
    assert out_svg.is_file()
    assert 'Dynamic-filter point reduction ratio' in out_svg.read_text(encoding='utf-8')
