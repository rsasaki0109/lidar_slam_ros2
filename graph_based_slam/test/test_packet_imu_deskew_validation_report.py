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

"""Regression tests for packet IMU deskew validation reporting."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'generate_packet_imu_deskew_validation_report.py'


def _write_metrics(
    run_dir: Path,
    *,
    rmse: float,
    matched: int,
    estimated_path_length_m: float,
    reference_path_length_m: float,
) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        'evo': {
            'ape': {
                'rmse': rmse,
            }
        },
        'cross_validation': {
            'matched_poses': matched,
            'estimated_path_length_m': estimated_path_length_m,
            'reference_path_length_m': reference_path_length_m,
        },
    }
    (run_dir / 'metrics.json').write_text(
        json.dumps(payload, indent=2),
        encoding='utf-8',
    )


def test_packet_imu_deskew_validation_report_passes_for_good_cases(tmp_path):
    root = tmp_path / 'matrix'
    _write_metrics(
        root / 'bag1_front' / 'no_imu',
        rmse=0.248,
        matched=531,
        estimated_path_length_m=94.5,
        reference_path_length_m=96.8,
    )
    _write_metrics(
        root / 'bag1_front' / 'imu',
        rmse=0.251,
        matched=571,
        estimated_path_length_m=95.0,
        reference_path_length_m=96.8,
    )
    _write_metrics(
        root / 'bag6_front' / 'no_imu',
        rmse=0.422,
        matched=381,
        estimated_path_length_m=119.0,
        reference_path_length_m=120.5,
    )
    _write_metrics(
        root / 'bag6_front' / 'imu',
        rmse=0.365,
        matched=711,
        estimated_path_length_m=119.1,
        reference_path_length_m=120.5,
    )

    out_md = tmp_path / 'packet_imu.md'
    out_json = tmp_path / 'packet_imu.json'
    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--root',
            str(root),
            '--write-md',
            str(out_md),
            '--write-json',
            str(out_json),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert 'cases: 2' in result.stdout
    assert out_md.is_file()
    assert out_json.is_file()
    assert 'Overall: PASS' in out_md.read_text(encoding='utf-8')


def test_packet_imu_deskew_validation_report_fails_on_regression(tmp_path):
    root = tmp_path / 'matrix'
    _write_metrics(
        root / 'bad_case' / 'no_imu',
        rmse=0.4,
        matched=400,
        estimated_path_length_m=119.0,
        reference_path_length_m=120.0,
    )
    _write_metrics(
        root / 'bad_case' / 'imu',
        rmse=2.0,
        matched=100,
        estimated_path_length_m=20.0,
        reference_path_length_m=120.0,
    )

    out_md = tmp_path / 'packet_imu.md'
    out_json = tmp_path / 'packet_imu.json'
    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--root',
            str(root),
            '--write-md',
            str(out_md),
            '--write-json',
            str(out_json),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 2
    assert 'bad_case' in result.stdout
    assert 'Overall: FAIL' in out_md.read_text(encoding='utf-8')
