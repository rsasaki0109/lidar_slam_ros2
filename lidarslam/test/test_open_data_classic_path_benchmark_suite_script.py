# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Regression tests for the classic-path benchmark suite wrapper."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_open_data_classic_path_benchmark_suite.sh'


def test_classic_path_suite_wrapper_runs_four_cases_and_renders_report():
    """The wrapper should exercise no-GNSS, GNSS-only, GNSS+odom, and GNSS+IMU cases."""
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'run_open_data_applanix_velodyne_gnss_benchmark.sh' in script
    assert '--use-gnss false' in script
    assert '--use-odom-prior true' in script
    assert '--odom-frame-id odom' in script
    assert '--odom-prior-planar true' in script
    assert '--robot-frame-id velodyne_front' in script
    assert '--use-imu true' in script
    assert '--robot-frame-id base_link' in script
    assert '--imu-frame-id base_link' in script
    assert 'generate_classic_path_report.py' in script
    assert 'classic_path_report.md' in script
    assert 'classic_path_report.json' in script
    assert 'classic_path_report.svg' in script
