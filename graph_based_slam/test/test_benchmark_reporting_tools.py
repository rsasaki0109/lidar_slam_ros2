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

"""Regression tests for benchmark summary and release-readiness scripts."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
BENCHMARK_SUMMARY_SCRIPT = REPO_ROOT / 'scripts' / 'benchmark_summary.py'
FIXTURE_SCRIPT = REPO_ROOT / 'scripts' / 'generate_sample_benchmark_metrics.py'
RELEASE_READINESS_SCRIPT = (
    REPO_ROOT / 'scripts' / 'run_release_readiness_checks.sh'
)


def _write_metrics(
    run_dir: Path,
    *,
    bag_name: str,
    ape_rmse: float | None,
    lid_ok: bool = True,
    lid_rtf: float = 0.5,
    lid_wall: float = 12.0,
    glim_ok: bool = True,
    glim_rtf: float = 0.8,
    glim_wall: float = 20.0,
) -> None:
    """Write a minimal metrics.json fixture for report scripts."""
    run_dir.mkdir(parents=True, exist_ok=True)
    metrics = {
        'out_dir': str(run_dir),
        'bag_path': f'/bags/{bag_name}',
        'bag_duration_sec': 30.0,
        'points_topic': '/points',
        'frames': {'points_frame_id': 'os_sensor'},
        'reference': {
            'source': 'leica_prism_gt',
            'kind': 'ground_truth',
        },
        'lidarslam': {
            'success': lid_ok,
            'rtf': lid_rtf,
            'wall_sec': lid_wall,
            'param_path': str(REPO_ROOT / 'lidarslam/param/lidarslam.yaml'),
        },
        'glim': {
            'available': True,
            'success': glim_ok,
            'reference_source': 'cache',
            'rtf': glim_rtf,
            'wall_sec': glim_wall,
        },
    }
    if ape_rmse is not None:
        metrics['evo'] = {
            'ape': {
                'rmse': ape_rmse,
                'median': ape_rmse / 2.0,
                'max': ape_rmse * 2.0,
            }
        }

    (run_dir / 'metrics.json').write_text(
        json.dumps(metrics, indent=2),
        encoding='utf-8',
    )


def test_benchmark_summary_ranks_runs_and_writes_artifacts(tmp_path):
    """Benchmark summary should rank by APE and emit CSV/Markdown."""
    root = tmp_path / 'benchmarks'
    _write_metrics(
        root / 'suite_a' / 'run_slower',
        bag_name='run_slower',
        ape_rmse=0.25,
        lid_rtf=0.62,
    )
    _write_metrics(
        root / 'suite_a' / 'run_best',
        bag_name='run_best',
        ape_rmse=0.09,
        lid_rtf=0.57,
    )
    _write_metrics(
        root / 'suite_b' / 'run_no_ape',
        bag_name='run_no_ape',
        ape_rmse=None,
        lid_rtf=0.41,
    )

    markdown_path = tmp_path / 'summary.md'
    csv_path = tmp_path / 'summary.csv'
    result = subprocess.run(
        [
            'python3',
            str(BENCHMARK_SUMMARY_SCRIPT),
            '--root',
            str(root),
            '--write-md',
            str(markdown_path),
            '--write-csv',
            str(csv_path),
            '--ape-threshold',
            '0.10',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert '- runs: 3' in result.stdout
    assert 'APE-primary best: run_best (0.090m)' in result.stdout
    assert markdown_path.is_file()
    assert csv_path.is_file()

    with csv_path.open('r', encoding='utf-8', newline='') as handle:
        rows = list(csv.DictReader(handle))

    assert [row['run'] for row in rows] == [
        'run_best',
        'run_slower',
        'run_no_ape',
    ]
    assert rows[0]['primary_rank'] == '1'
    assert rows[0]['ape_ok'] == 'true'
    assert rows[0]['ape_ref_kind'] == 'ground_truth'
    assert rows[0]['ape_ref_src'] == 'leica_prism_gt'
    assert rows[1]['ape_ok'] == 'false'
    assert rows[2]['ape_rmse_m'] == ''


def test_benchmark_summary_fails_when_metrics_are_missing(tmp_path):
    """Empty benchmark roots should fail with a clear message."""
    result = subprocess.run(
        [
            'python3',
            str(BENCHMARK_SUMMARY_SCRIPT),
            '--root',
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 1
    assert 'no metrics.json found under' in result.stdout


def test_benchmark_summary_can_fail_on_threshold(tmp_path):
    """Threshold gate should fail when a run exceeds the requested APE."""
    root = tmp_path / 'benchmarks'
    _write_metrics(
        root / 'suite_a' / 'run_good',
        bag_name='run_good',
        ape_rmse=0.08,
    )
    _write_metrics(
        root / 'suite_a' / 'run_bad',
        bag_name='run_bad',
        ape_rmse=0.22,
    )

    result = subprocess.run(
        [
            'python3',
            str(BENCHMARK_SUMMARY_SCRIPT),
            '--root',
            str(root),
            '--ape-threshold',
            '0.10',
            '--fail-on-ape-threshold',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 2
    assert 'error: APE threshold failed for runs: run_bad' in result.stdout


def test_benchmark_summary_threshold_can_filter_reference_kind(tmp_path):
    """Cross-validation runs should not fail a ground-truth-only threshold gate."""
    root = tmp_path / 'benchmarks'
    _write_metrics(
        root / 'suite_a' / 'run_gt',
        bag_name='run_gt',
        ape_rmse=0.08,
    )
    crossval_dir = root / 'suite_a' / 'run_crossval'
    _write_metrics(
        crossval_dir,
        bag_name='run_crossval',
        ape_rmse=0.22,
    )
    metrics = json.loads((crossval_dir / 'metrics.json').read_text(encoding='utf-8'))
    metrics['reference'] = {
        'source': 'glim_mid360_reference',
        'kind': 'cross_validation',
    }
    (crossval_dir / 'metrics.json').write_text(
        json.dumps(metrics, indent=2),
        encoding='utf-8',
    )

    result = subprocess.run(
        [
            'python3',
            str(BENCHMARK_SUMMARY_SCRIPT),
            '--root',
            str(root),
            '--ape-threshold',
            '0.10',
            '--ape-threshold-reference-kind',
            'ground_truth',
            '--fail-on-ape-threshold',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert 'reference_kind=ground_truth' in result.stdout


def test_synthetic_fixture_generator_drives_release_gate(tmp_path):
    """The synthetic fixture generator should produce gate-ready artifacts."""
    benchmark_root = tmp_path / 'fixture'
    out_dir = tmp_path / 'release_ready'

    fixture = subprocess.run(
        [
            'python3',
            str(FIXTURE_SCRIPT),
            '--root',
            str(benchmark_root),
            '--profile',
            'passing',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert fixture.returncode == 0, fixture.stderr
    assert (benchmark_root / 'ci_fixture' / 'run_best' / 'metrics.json').is_file()
    assert (benchmark_root / 'references' / 'synthetic_reference.tum').is_file()

    result = subprocess.run(
        [
            'bash',
            str(RELEASE_READINESS_SCRIPT),
            '--skip-default-ci',
            '--benchmark-root',
            str(benchmark_root),
            '--out-dir',
            str(out_dir),
            '--ape-threshold',
            '0.10',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert (out_dir / 'benchmark_summary.md').is_file()
    assert (out_dir / 'benchmark_report.html').is_file()
    assert 'run_best' in (out_dir / 'benchmark_summary.md').read_text(
        encoding='utf-8',
    )


def test_synthetic_fixture_generator_failing_profile_trips_release_gate(tmp_path):
    """The failing fixture profile should trip the release gate."""
    benchmark_root = tmp_path / 'fixture_fail'
    out_dir = tmp_path / 'release_fail'

    fixture = subprocess.run(
        [
            'python3',
            str(FIXTURE_SCRIPT),
            '--root',
            str(benchmark_root),
            '--profile',
            'failing',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert fixture.returncode == 0, fixture.stderr
    assert (benchmark_root / 'ci_fixture' / 'run_bad' / 'metrics.json').is_file()

    result = subprocess.run(
        [
            'bash',
            str(RELEASE_READINESS_SCRIPT),
            '--skip-default-ci',
            '--benchmark-root',
            str(benchmark_root),
            '--out-dir',
            str(out_dir),
            '--ape-threshold',
            '0.10',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 2
    assert (out_dir / 'benchmark_summary.md').is_file()
    assert not (out_dir / 'benchmark_report.html').exists()
    assert 'run_bad' in (out_dir / 'benchmark_summary.md').read_text(
        encoding='utf-8',
    )


def test_release_readiness_generates_summary_and_html_report(tmp_path):
    """Release-readiness wrapper should emit benchmark artifacts."""
    benchmark_root = tmp_path / 'benchmarks'
    _write_metrics(
        benchmark_root / 'newer_college' / 'run_good',
        bag_name='math_hard',
        ape_rmse=0.08,
    )
    _write_metrics(
        benchmark_root / 'newer_college' / 'run_ok_too',
        bag_name='math_hard',
        ape_rmse=0.09,
    )
    out_dir = tmp_path / 'release_readiness'

    result = subprocess.run(
        [
            'bash',
            str(RELEASE_READINESS_SCRIPT),
            '--skip-default-ci',
            '--benchmark-root',
            str(benchmark_root),
            '--out-dir',
            str(out_dir),
            '--ape-threshold',
            '0.10',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert (out_dir / 'benchmark_summary.md').is_file()
    assert (out_dir / 'benchmark_summary.csv').is_file()
    assert (out_dir / 'benchmark_report.html').is_file()
    assert 'benchmark_summary_md:' in result.stdout
    assert 'benchmark_report_html:' in result.stdout
    html = (out_dir / 'benchmark_report.html').read_text(
        encoding='utf-8',
    )
    assert 'run_good' in html
    assert 'newer_college' in html


def test_release_readiness_fails_on_threshold_violation(tmp_path):
    """Release-readiness should stop when the APE gate is violated."""
    benchmark_root = tmp_path / 'benchmarks'
    _write_metrics(
        benchmark_root / 'newer_college' / 'run_good',
        bag_name='math_hard',
        ape_rmse=0.08,
    )
    _write_metrics(
        benchmark_root / 'newer_college' / 'run_bad',
        bag_name='math_hard',
        ape_rmse=0.22,
    )
    out_dir = tmp_path / 'release_readiness_fail'

    result = subprocess.run(
        [
            'bash',
            str(RELEASE_READINESS_SCRIPT),
            '--skip-default-ci',
            '--benchmark-root',
            str(benchmark_root),
            '--out-dir',
            str(out_dir),
            '--ape-threshold',
            '0.10',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 2
    assert (out_dir / 'benchmark_summary.md').is_file()
    assert not (out_dir / 'benchmark_report.html').exists()
    assert 'error: APE threshold failed for runs: run_bad' in result.stdout
