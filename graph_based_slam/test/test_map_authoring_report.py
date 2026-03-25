# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Regression tests for the public map-authoring report generator."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'generate_map_authoring_report.py'


def test_map_authoring_report_summarizes_benchmark_gnss_and_filter(tmp_path):
    """The report should summarize the repo's map-authoring evidence."""
    benchmark_metrics = tmp_path / 'bench' / 'metrics.json'
    projector = tmp_path / 'gnss' / 'map_projector_info.yaml'
    dynamic_filter = tmp_path / 'dynamic' / 'report.json'
    classic_path = tmp_path / 'classic' / 'report.json'
    out_md = tmp_path / 'map_authoring.md'
    out_json = tmp_path / 'map_authoring.json'

    benchmark_metrics.parent.mkdir(parents=True, exist_ok=True)
    benchmark_metrics.write_text(
        json.dumps({'evo': {'ape': {'rmse': 0.95, 'pairs': 198}}}, indent=2) + '\n',
        encoding='utf-8',
    )
    projector.parent.mkdir(parents=True, exist_ok=True)
    projector.write_text(
        'projector_type: LocalCartesian\nmap_origin:\n  latitude: 1.0\n  longitude: 2.0\n',
        encoding='utf-8',
    )
    dynamic_filter.parent.mkdir(parents=True, exist_ok=True)
    dynamic_filter.write_text(
        json.dumps({'point_reduction_ratio': 0.5}, indent=2) + '\n',
        encoding='utf-8',
    )
    classic_path.parent.mkdir(parents=True, exist_ok=True)
    classic_path.write_text(
        json.dumps({'gnss_gain_m': 118.4}, indent=2) + '\n',
        encoding='utf-8',
    )

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT),
            '--benchmark-metrics',
            str(benchmark_metrics),
            '--gnss-projector',
            str(projector),
            '--dynamic-filter-report',
            str(dynamic_filter),
            '--classic-path-report',
            str(classic_path),
            '--out',
            str(out_md),
            '--write-json',
            str(out_json),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    report = out_md.read_text(encoding='utf-8')
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert 'Map Authoring Report' in report
    assert 'pointcloud-map authoring stack' in report
    assert 'saved-point reduction ratio' in report
    assert payload['gnss_georeference']['has_map_origin'] is True
    assert payload['classic_path']['gnss_gain_m'] == 118.4
