# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)

"""Tests for submission-bundle staging helpers."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess

from test_autoware_map_tools import _create_map_bundle, _write_binary_xyz_pcd


REPO_ROOT = Path(__file__).resolve().parents[2]
SUBMISSION_BUNDLE_SCRIPT = (
    REPO_ROOT / 'scripts' / 'create_map_authoring_submission_bundle.sh'
)


def test_submission_bundle_script_collects_map_metrics_reports_and_manifest(tmp_path):
    """The bundle script should stage a reusable map-authoring submission bundle."""
    source_root, _ = _create_map_bundle(tmp_path / 'graph_output')
    _write_binary_xyz_pcd(source_root / 'map.pcd', [(0.0, 0.0, 0.0)])
    (source_root / 'metrics.json').write_text('{"ok": true}\n', encoding='utf-8')
    (source_root / 'traj_raw.tum').write_text('0 0 0 0 0 0 0 1\n', encoding='utf-8')
    (source_root / 'traj_corrected.tum').write_text(
        '0 0 0 0 0 0 0 1\n',
        encoding='utf-8',
    )
    report_path = tmp_path / 'focused_report.md'
    report_path.write_text('# Report\n', encoding='utf-8')
    report_path.with_suffix('.json').write_text('{"ok": true}\n', encoding='utf-8')
    report_path.with_suffix('.svg').write_text('<svg/>\n', encoding='utf-8')
    target_root = tmp_path / 'submission_bundle'

    result = subprocess.run(
        [
            'bash',
            str(SUBMISSION_BUNDLE_SCRIPT),
            str(source_root),
            str(target_root),
            '--report',
            str(report_path),
            '--label',
            'example_bundle',
            '--verify-map',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert (target_root / 'pointcloud_map' / 'pointcloud_map_metadata.yaml').is_file()
    assert (target_root / 'map_projector_info.yaml').is_file()
    assert (target_root / 'metrics.json').is_file()
    assert (target_root / 'map.pcd').is_file()
    assert (target_root / 'traj_raw.tum').is_file()
    assert (target_root / 'traj_corrected.tum').is_file()
    assert (target_root / 'reports' / 'focused_report.md').is_file()
    assert (target_root / 'reports' / 'focused_report.json').is_file()
    assert (target_root / 'reports' / 'focused_report.svg').is_file()
    assert (target_root / 'map_qa_summary.md').is_file()
    assert (target_root / 'verify_autoware_map.log').is_file()

    manifest = json.loads((target_root / 'manifest.json').read_text(encoding='utf-8'))
    assert manifest['bundle_label'] == 'example_bundle'
    assert manifest['verify_map_ran'] is True
    assert 'metrics.json' in manifest['files']
    assert 'reports/focused_report.md' in manifest['files']
    assert 'reports/focused_report.json' in manifest['files']
    assert 'reports/focused_report.svg' in manifest['files']

    qa_summary = (target_root / 'map_qa_summary.md').read_text(encoding='utf-8')
    assert 'Included QA Reports' in qa_summary
    assert 'reports/focused_report.svg' in qa_summary
