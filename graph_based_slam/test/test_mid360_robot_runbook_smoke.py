# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Smoke tests for the MID-360 robot runbook script."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'smoke_mid360_robot_runbook.sh'


def test_mid360_robot_runbook_smoke_script(tmp_path: Path):
    work_dir = tmp_path / 'work'
    output_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            'bash',
            str(SCRIPT_PATH),
            '--work-dir',
            str(work_dir),
            '--output-dir',
            str(output_dir),
            '--keep-work-dir',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    readiness = json.loads((output_dir / 'mid360_robot_readiness.json').read_text())

    assert 'MID-360 robot runbook smoke: PASS' in result.stdout
    assert readiness['status'] == 'PASS'
    assert readiness['bag_diagnostics']['topics']['pointcloud']['metadata_rate_hz'] == 10.0
    assert readiness['bag_diagnostics']['topics']['imu']['metadata_rate_hz'] == 100.0
    assert (output_dir / 'mid360_robot_session_dashboard.html').is_file()
    assert (output_dir / 'mid360_robot_field_session.json').is_file()
    assert (output_dir / 'mid360_robot_run_plan.json').is_file()
    assert (output_dir / 'mid360_robot_recording_check.json').is_file()
    assert (work_dir / 'recordings' / 'smoke_field_record_plan.json').is_file()
    assert (work_dir / 'recordings' / 'smoke_record_record_plan.json').is_file()
    assert (work_dir / 'recordings' / 'smoke_record_profile.yaml').is_file()
    assert (work_dir / 'recordings' / 'smoke_record' / 'metadata.yaml').is_file()
