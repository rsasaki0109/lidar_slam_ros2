#!/usr/bin/env python3
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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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

"""Run Localization Zoo on raw/corrected TUM and freeze cross-repo evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = REPO_ROOT / 'configs/slam_benchmark_profiles/public_suite_v1.yaml'


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def git_revision(path: Path) -> str | None:
    result = subprocess.run(
        ['git', '-C', str(path), 'rev-parse', 'HEAD'], capture_output=True,
        text=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else None


def load_profile(path: Path, dataset: str) -> tuple[dict[str, Any], dict[str, Any]]:
    document = yaml.safe_load(Path(path).read_text())
    profile = document.get('slam_benchmark_profile') if isinstance(document, dict) else None
    if not isinstance(profile, dict) or not isinstance(profile.get('datasets'), dict):
        raise ValueError('profile needs slam_benchmark_profile.datasets')
    if dataset not in profile['datasets']:
        raise ValueError(f'dataset {dataset!r} is not defined by the profile')
    return profile, profile['datasets'][dataset]


def zoo_command(zoo: Path, dataset_config: dict[str, Any], gt: Path,
                raw: Path, corrected: Path, output: Path) -> list[str]:
    command = [
        sys.executable,
        str(zoo / 'evaluation/scripts/evaluate_external_tum.py'),
        '--gt-tum', str(gt),
        '--est', f'frontend_raw:{raw}',
        '--est', f'graph_corrected:{corrected}',
        '--summary-json', str(output),
        '--alignment', str(dataset_config['alignment']),
        '--segment-length', str(dataset_config['segment_length_m']),
        '--max-time-difference', str(dataset_config['max_time_difference_s']),
    ]
    if dataset_config.get('position_only_reference'):
        command.append('--position-only-reference')
    return command


def densify_command(raw: Path, corrected: Path, output: Path) -> list[str]:
    """Build the canonical dense graph trajectory before metric association."""
    return [sys.executable, str(REPO_ROOT / 'scripts/densify_corrected_trajectory.py'),
            '--raw', str(raw), '--corrected', str(corrected),
            '--output', str(output), '--max-anchor-offset', '0.2']


def metric_delta(summary: dict[str, Any], metric: str) -> dict[str, Any]:
    methods = {row['name']: row for row in summary.get('methods', [])}
    raw = methods.get('frontend_raw', {}).get(metric)
    corrected = methods.get('graph_corrected', {}).get(metric)
    if not isinstance(raw, (int, float)) or not isinstance(corrected, (int, float)):
        return {'metric': metric, 'raw': raw, 'corrected': corrected,
                'change_percent': None, 'improved': None}
    change = ((corrected - raw) / abs(raw) * 100.0) if raw != 0.0 else None
    improved = None if change is None or abs(change) <= 1.0e-6 else corrected < raw
    return {'metric': metric, 'raw': float(raw), 'corrected': float(corrected),
            'change_percent': change, 'improved': improved}


def load_optional_reports(paths: dict[str, Path | None]) -> dict[str, Any]:
    reports = {}
    for name, path in paths.items():
        if path is not None:
            document = yaml.safe_load(Path(path).read_text())
            if not isinstance(document, dict):
                raise ValueError(f'{path}: report must contain a mapping')
            reports[name] = document
    return reports


def build_manifest(profile: dict[str, Any], dataset_name: str,
                   dataset_config: dict[str, Any], zoo: Path,
                   inputs: dict[str, Path], trajectory_summary: dict[str, Any],
                   reports: dict[str, Any]) -> dict[str, Any]:
    methods = {row.get('name'): row for row in trajectory_summary.get('methods', [])}
    normalized = {'trajectory': methods, **reports}
    available = {'trajectory', *reports.keys()}
    missing = sorted(set(dataset_config.get('required_reports', [])) - available)
    missing_metrics = []
    resolved_metrics = {}
    for metric_path in dataset_config.get('required_metrics', []):
        value: Any = normalized
        for key in metric_path.split('.'):
            if not isinstance(value, dict) or key not in value:
                value = None
                break
            value = value[key]
        if (not isinstance(value, (int, float)) or isinstance(value, bool) or
                value != value or abs(float(value)) == float('inf')):
            missing_metrics.append(metric_path)
        else:
            resolved_metrics[metric_path] = value
    failed_success_metrics = []
    for metric_path, expected in profile.get('required_success_metrics', {}).items():
        value: Any = normalized
        for key in metric_path.split('.'):
            if not isinstance(value, dict) or key not in value:
                value = None
                break
            value = value[key]
        if value != expected:
            failed_success_metrics.append(
                {'metric': metric_path, 'value': value, 'expected': expected})
    delta = metric_delta(trajectory_summary, dataset_config['primary_metric'])
    return {
        'schema_version': 1,
        'profile': profile['name'],
        'enforcement': profile.get('enforcement', 'report_only'),
        'dataset': dataset_name,
        'dataset_contract': dataset_config,
        'revisions': {
            'lidar_slam_ros2': git_revision(REPO_ROOT),
            'localization_zoo': git_revision(zoo),
        },
        'inputs': {name: {'path': str(path.resolve()), 'sha256': sha256(path)}
                   for name, path in inputs.items()},
        'trajectory': trajectory_summary,
        'primary_delta': delta,
        'reports': reports,
        'evidence': {
            'required_reports': dataset_config.get('required_reports', []),
            'missing_reports': missing,
            'required_metrics': dataset_config.get('required_metrics', []),
            'missing_metrics': missing_metrics,
            'resolved_metrics': resolved_metrics,
            'failed_success_metrics': failed_success_metrics,
            'complete': not missing and not missing_metrics and not failed_success_metrics,
        },
        'adoption_policy': profile.get('adoption_policy', {}),
        'verdict': ('INCOMPLETE' if missing or missing_metrics or failed_success_metrics else
                    'IMPROVED' if delta['improved'] is True else
                    'REGRESSED' if delta['improved'] is False else 'RECORDED'),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--localization-zoo', type=Path, required=True)
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--gt-tum', type=Path, required=True)
    parser.add_argument('--raw-tum', type=Path, required=True)
    parser.add_argument('--corrected-tum', type=Path, required=True)
    parser.add_argument('--geometry-report', type=Path)
    parser.add_argument('--alignment-report', type=Path)
    parser.add_argument('--colour-report', type=Path)
    parser.add_argument('--runtime-report', type=Path)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    try:
        profile, dataset_config = load_profile(args.profile, args.dataset)
    except (OSError, yaml.YAMLError, ValueError) as error:
        parser.error(str(error))
    args.out_dir.mkdir(parents=True, exist_ok=True)
    dense_corrected = args.out_dir / 'traj_corrected_dense.tum'
    subprocess.run(
        densify_command(args.raw_tum, args.corrected_tum, dense_corrected),
        check=True)
    trajectory_path = args.out_dir / 'trajectory_summary.json'
    command = zoo_command(
        args.localization_zoo, dataset_config, args.gt_tum, args.raw_tum,
        dense_corrected, trajectory_path)
    subprocess.run(command, check=True)
    reports = load_optional_reports({
        'geometry': args.geometry_report,
        'alignment': args.alignment_report,
        'colour': args.colour_report,
        'runtime': args.runtime_report,
    })
    inputs = {'profile': args.profile, 'gt_tum': args.gt_tum,
              'raw_tum': args.raw_tum, 'corrected_tum': args.corrected_tum,
              'corrected_dense_tum': dense_corrected}
    for name, path in (('geometry_report', args.geometry_report),
                       ('alignment_report', args.alignment_report),
                       ('colour_report', args.colour_report),
                       ('runtime_report', args.runtime_report)):
        if path is not None:
            inputs[name] = path
    manifest = build_manifest(
        profile, args.dataset, dataset_config, args.localization_zoo, inputs,
        json.loads(trajectory_path.read_text()), reports)
    output = args.out_dir / 'cross_repo_benchmark.json'
    output.write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'output': str(output), 'dataset': args.dataset,
                      'verdict': manifest['verdict'],
                      'primary_delta': manifest['primary_delta'],
                      'missing_reports': manifest['evidence']['missing_reports'],
                      'missing_metrics': manifest['evidence']['missing_metrics'],
                      'failed_success_metrics': manifest['evidence'][
                          'failed_success_metrics']},
                     indent=2))
    return 0 if manifest['verdict'] != 'INCOMPLETE' else 2


if __name__ == '__main__':
    raise SystemExit(main())
