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

"""Tests for MID-360 public loop-candidate analysis."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import zipfile


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
SCRIPT_PATH = SCRIPT_DIR / 'analyze_mid360_robot_public_loop_candidates.py'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_loop_candidates import (  # noqa: E402
    PUBLIC_LOOP_CANDIDATES_JSON,
    PUBLIC_LOOP_CANDIDATES_MARKDOWN,
    PublicLoopCandidateAnalyzer,
    PublicLoopCandidateOptions,
    write_public_loop_candidate_report,
)


def _loop_trajectory() -> str:
    lines = []
    for index in range(60):
        lines.append(f'{index} {index * 0.1:.3f} 0.0 0.0 0 0 0 1')
    for index in range(60, 120):
        x = (119 - index) * 0.1
        lines.append(f'{index} {x:.3f} 0.1 0.0 0 0 0 1')
    return '\n'.join(lines) + '\n'


def _non_loop_trajectory() -> str:
    lines = [
        f'{index} {index * 0.5:.3f} 20.0 0.0 0 0 0 1'
        for index in range(120)
    ]
    return '\n'.join(lines) + '\n'


def _write_gt_zip(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, 'w') as archive:
        archive.writestr('gt/traj_lidar_outdoor_kidnap.txt', _loop_trajectory())
        archive.writestr('gt/traj_lidar_outdoor_hard_01.txt', _non_loop_trajectory())
        archive.writestr('gt/traj_lidar_indoor_easy_01.txt', _loop_trajectory())


def test_public_loop_candidate_analyzer_selects_outdoor_mid360_loop(tmp_path: Path):
    gt_zip = tmp_path / 'gt.zip'
    _write_gt_zip(gt_zip)

    report = PublicLoopCandidateAnalyzer().analyze(
        PublicLoopCandidateOptions(
            dataset_root=tmp_path / 'datasets',
            output_dir=tmp_path / 'out',
            gt_zip=gt_zip,
            verify_md5=False,
        )
    )

    assert report['status'] == 'PASS'
    assert report['counts']['target_mid360'] == 2
    assert report['recommended_sequences'][0]['sequence_id'] == 'outdoor_kidnap'
    assert report['recommended_sequences'][0]['bag_file_ids'] == [
        'outdoor_kidnap_a',
        'outdoor_kidnap_b',
    ]
    assert all(sequence['sequence_id'] != 'indoor_easy_01' for sequence in report['sequences'])

    paths = write_public_loop_candidate_report(report, tmp_path / 'out')
    assert paths['json'].name == PUBLIC_LOOP_CANDIDATES_JSON
    assert paths['markdown'].name == PUBLIC_LOOP_CANDIDATES_MARKDOWN
    assert 'outdoor_kidnap' in paths['markdown'].read_text(encoding='utf-8')


def test_public_loop_candidate_cli_writes_report(tmp_path: Path):
    gt_zip = tmp_path / 'gt.zip'
    output_dir = tmp_path / 'out'
    _write_gt_zip(gt_zip)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--gt-zip',
            str(gt_zip),
            '--skip-md5',
            '--output-dir',
            str(output_dir),
            '--write',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)

    assert report['status'] == 'PASS'
    assert report['recommended_sequences'][0]['sequence_id'] == 'outdoor_kidnap'
    assert (output_dir / PUBLIC_LOOP_CANDIDATES_JSON).is_file()
    assert (output_dir / PUBLIC_LOOP_CANDIDATES_MARKDOWN).is_file()
