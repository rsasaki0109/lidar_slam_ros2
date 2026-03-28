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

"""Regression tests for the dynamic-object-filter report generator."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'generate_dynamic_object_filter_report.py'


def _write_minimal_pcd(path: Path, points: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        '\n'.join(
            [
                '# .PCD v0.7 - Point Cloud Data file format',
                'VERSION 0.7',
                'FIELDS x y z intensity',
                'SIZE 4 4 4 4',
                'TYPE F F F F',
                'COUNT 1 1 1 1',
                f'WIDTH {points}',
                'HEIGHT 1',
                'VIEWPOINT 0 0 0 1 0 0 0',
                f'POINTS {points}',
                'DATA ascii',
            ]
        )
        + '\n',
        encoding='utf-8',
    )


def _write_run_dir(
    root: Path,
    *,
    total_points: int,
    tiles: int,
    verify: str,
    projector_type: str,
    filter_log: str | None = None,
) -> None:
    pointcloud_map = root / 'pointcloud_map'
    per_tile = max(1, total_points // max(1, tiles))
    for idx in range(tiles):
        _write_minimal_pcd(pointcloud_map / f'{idx}_0.pcd', per_tile)
    metadata_lines = ['x_resolution: 20.0', 'y_resolution: 20.0']
    for idx in range(tiles):
        metadata_lines.append(f'{idx}_0.pcd: [{idx * 20}, 0]')
    (pointcloud_map / 'pointcloud_map_metadata.yaml').write_text(
        '\n'.join(metadata_lines) + '\n',
        encoding='utf-8',
    )
    (root / 'map_projector_info.yaml').write_text(
        f'projector_type: {projector_type}\n',
        encoding='utf-8',
    )
    (root / 'verify_autoware_map.log').write_text(
        f'RESULT: {verify} -- map is Autoware-compatible\n',
        encoding='utf-8',
    )
    launch_lines = ['Saved grid-divided map: 12 cells (20x20m) to /tmp/pointcloud_map']
    if filter_log is not None:
        launch_lines.insert(0, filter_log)
    (root / 'lidarslam.launch.log').write_text(
        '\n'.join(launch_lines) + '\n',
        encoding='utf-8',
    )


def test_dynamic_object_filter_report_summarizes_point_reduction(tmp_path):
    """The report should summarize point-count reduction and filter stats."""
    baseline = tmp_path / 'baseline'
    filtered = tmp_path / 'filtered'
    out_md = tmp_path / 'dynamic_filter.md'
    out_json = tmp_path / 'dynamic_filter.json'
    out_svg = tmp_path / 'dynamic_filter.svg'

    _write_run_dir(
        baseline,
        total_points=1200,
        tiles=3,
        verify='PASS',
        projector_type='LocalCartesian',
    )
    _write_run_dir(
        filtered,
        total_points=600,
        tiles=3,
        verify='PASS',
        projector_type='LocalCartesian',
        filter_log=(
            'Dynamic object filter: input_points 1000, kept 20/40 candidate voxels, '
            'removed 20, always_keep 10, output_points 600'
        ),
    )

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--baseline-dir',
            str(baseline),
            '--filtered-dir',
            str(filtered),
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
    assert 'point reduction ratio' in report
    assert '`0.500`' in report
    assert 'removed candidate voxel ratio' in report
    assert 'tile jaccard' in report
    assert 'kept candidate voxels' in report
    assert payload['filtered']['dynamic_filter_stats']['removed_candidate_voxels'] == 20
    assert payload['shared_metadata_tiles'] == 3
    assert payload['tile_jaccard'] == 1.0
    assert payload['filtered_tile_overlap_ratio'] == 1.0
    assert out_svg.is_file()
    assert 'Saved point count comparison' in out_svg.read_text(encoding='utf-8')
