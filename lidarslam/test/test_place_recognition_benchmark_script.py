# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Regression tests for the place-recognition benchmark wrapper."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_place_recognition_benchmark.sh'


def test_place_recognition_wrapper_runs_baseline_candidate_and_report():
    """The wrapper should compare distance-only and Scan Context candidate runs."""
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'run_rko_lio_mid360_crossval_benchmark.sh' in script
    assert '--use-scan-context false' in script
    assert '--use-scan-context true' in script
    assert '--scan-context-threshold' in script
    assert 'generate_place_recognition_report.py' in script
    assert 'place_recognition_report.md' in script
    assert 'place_recognition_report.json' in script
