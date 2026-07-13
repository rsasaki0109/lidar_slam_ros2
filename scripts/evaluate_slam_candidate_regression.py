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
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Compare baseline/candidate public-suite manifests as a regression gate."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = (
    REPO_ROOT / 'configs/slam_benchmark_profiles/phase7_regression_v1.yaml')


def _finite(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)))


def _normalized_manifest(document: dict[str, Any]) -> dict[str, Any]:
    """Expose named trajectory methods beside the already named reports."""
    methods = {}
    for row in document.get('trajectory', {}).get('methods', []):
        if isinstance(row, dict) and isinstance(row.get('name'), str):
            methods[row['name']] = row
    return {'trajectory': methods, **document.get('reports', {})}


def _resolve(document: dict[str, Any], path: str) -> Any:
    value: Any = document
    for key in path.split('.'):
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def _change_percent(baseline: float, candidate: float) -> float:
    if baseline == 0.0:
        return 0.0 if candidate == 0.0 else math.inf
    return 100.0 * (candidate - baseline) / abs(baseline)


def evaluate_metric(baseline: dict[str, Any], candidate: dict[str, Any],
                    config: dict[str, Any]) -> dict[str, Any]:
    """Evaluate one lower/higher-is-better metric with a frozen tolerance."""
    path = config['path']
    before = _resolve(baseline, path)
    after = _resolve(candidate, path)
    result = {
        'path': path,
        'label': config.get('label', path),
        'direction': config['direction'],
        'baseline': before,
        'candidate': after,
        'required': config.get('required', True),
    }
    if not _finite(before) or not _finite(after):
        result.update({'change_percent': None, 'improved': None,
                       'passed': not result['required'],
                       'reason': 'required numeric metric is missing'})
        return result

    before_float = float(before)
    after_float = float(after)
    change = _change_percent(before_float, after_float)
    lower_is_better = config['direction'] == 'lower'
    if config['direction'] not in {'lower', 'higher'}:
        raise ValueError(f'{path}: direction must be lower or higher')
    improvement = before_float - after_float if lower_is_better else after_float - before_float
    tolerance_percent = float(config.get('max_regression_percent', 0.0))
    tolerance_absolute = float(config.get('max_regression_absolute', 0.0))
    scale_tolerance = abs(before_float) * tolerance_percent / 100.0
    allowed_regression = max(scale_tolerance, tolerance_absolute)
    passed = improvement >= -allowed_regression
    result.update({
        'change_percent': change,
        'improved': improvement > 0.0,
        'allowed_regression_absolute': allowed_regression,
        'passed': passed,
        'reason': 'within regression budget' if passed else 'regression budget exceeded',
    })
    return result


def _load_profile(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text())
    profile = document.get('slam_candidate_regression_profile') \
        if isinstance(document, dict) else None
    if not isinstance(profile, dict) or not isinstance(profile.get('datasets'), dict):
        raise ValueError('profile needs slam_candidate_regression_profile.datasets')
    return profile


def _load_manifests(paths: list[Path], label: str) -> dict[str, tuple[Path, dict]]:
    manifests = {}
    for path in paths:
        document = json.loads(path.read_text())
        dataset = document.get('dataset')
        if not isinstance(dataset, str) or not dataset:
            raise ValueError(f'{path}: dataset is missing')
        if dataset in manifests:
            raise ValueError(f'duplicate {label} manifest for {dataset}')
        manifests[dataset] = (path, document)
    return manifests


def _comparable_input_issues(baseline: dict[str, Any],
                             candidate: dict[str, Any]) -> list[str]:
    """Reject comparisons whose frozen contract or source capture differs."""
    issues = []
    if baseline.get('profile') != candidate.get('profile'):
        issues.append('benchmark profile differs')
    if baseline.get('dataset_contract') != candidate.get('dataset_contract'):
        issues.append('dataset contract differs')
    before_inputs = baseline.get('inputs', {})
    after_inputs = candidate.get('inputs', {})
    frozen_names = {
        name for name in set(before_inputs) | set(after_inputs)
        if name == 'gt_tum' or name.startswith('raw_artifact.')}
    if not any(name.startswith('raw_artifact.') for name in frozen_names):
        issues.append('no raw source artifact is frozen')
    for name in sorted(frozen_names):
        before_hash = before_inputs.get(name, {}).get('sha256')
        after_hash = after_inputs.get(name, {}).get('sha256')
        if not before_hash or not after_hash:
            issues.append(f'{name}: hash is missing from one arm')
        elif before_hash != after_hash:
            issues.append(f'{name}: hash differs')
    return issues


def evaluate(profile: dict[str, Any],
             baselines: dict[str, tuple[Path, dict]],
             candidates: dict[str, tuple[Path, dict]]) -> dict[str, Any]:
    """Evaluate all frozen datasets and return a machine-readable verdict."""
    configured = set(profile['datasets'])
    available = set(baselines) & set(candidates)
    missing = sorted(configured - available)
    unexpected = sorted((set(baselines) | set(candidates)) - configured)
    datasets = []
    for name, dataset_config in profile['datasets'].items():
        if name not in available:
            continue
        baseline_path, baseline_document = baselines[name]
        candidate_path, candidate_document = candidates[name]
        baseline = _normalized_manifest(baseline_document)
        candidate = _normalized_manifest(candidate_document)
        metrics = [evaluate_metric(baseline, candidate, metric)
                   for metric in dataset_config.get('metrics', [])]
        primary_path = dataset_config.get('primary_metric')
        primary = next((row for row in metrics if row['path'] == primary_path), None)
        complete = (
            baseline_document.get('evidence', {}).get('complete') is True
            and candidate_document.get('evidence', {}).get('complete') is True)
        primary_improved = (
            not dataset_config.get('require_primary_improvement', False)
            or (primary is not None and primary['improved'] is True))
        comparison_issues = _comparable_input_issues(
            baseline_document, candidate_document)
        checks = {
            'manifests_complete': complete,
            'inputs_comparable': not comparison_issues,
            'all_metrics_within_budget': bool(metrics)
            and all(row['passed'] for row in metrics),
            'primary_improved': primary_improved,
        }
        datasets.append({
            'dataset': name,
            'purpose': dataset_config.get('purpose'),
            'baseline_manifest': str(baseline_path.resolve()),
            'candidate_manifest': str(candidate_path.resolve()),
            'metrics': metrics,
            'comparison_issues': comparison_issues,
            'checks': checks,
            'passed': all(checks.values()),
        })
    checks = {
        'all_required_datasets_present': not missing,
        'no_unconfigured_datasets': not unexpected,
        'all_datasets_passed': bool(datasets) and all(row['passed'] for row in datasets),
    }
    return {
        'schema_version': 1,
        'profile': profile.get('name'),
        'missing_datasets': missing,
        'unexpected_datasets': unexpected,
        'datasets': datasets,
        'checks': checks,
        'passed': all(checks.values()),
        'verdict': 'ADOPT_CANDIDATE' if all(checks.values()) else 'REJECT_CANDIDATE',
    }


def markdown(report: dict[str, Any]) -> str:
    """Render a compact review artifact beside the JSON contract."""
    lines = [
        f"# SLAM candidate regression: {report['verdict']}", '',
        '| dataset | metric | baseline | candidate | change | pass |',
        '| --- | --- | ---: | ---: | ---: | :---: |',
    ]
    for dataset in report['datasets']:
        for metric in dataset['metrics']:
            change = metric['change_percent']
            change_text = 'n/a' if change is None else f'{change:+.3f}%'
            lines.append(
                f"| {dataset['dataset']} | {metric['label']} | "
                f"{metric['baseline']} | {metric['candidate']} | {change_text} | "
                f"{'yes' if metric['passed'] else 'no'} |")
    if report['missing_datasets']:
        lines.extend(['', 'Missing datasets: ' + ', '.join(report['missing_datasets'])])
    return '\n'.join(lines) + '\n'


def main() -> int:
    """Run the regression gate CLI and optionally enforce its verdict."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--baseline', type=Path, action='append', required=True)
    parser.add_argument('--candidate', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-pass', action='store_true')
    args = parser.parse_args()
    try:
        profile = _load_profile(args.profile)
        report = evaluate(
            profile, _load_manifests(args.baseline, 'baseline'),
            _load_manifests(args.candidate, 'candidate'))
    except (OSError, ValueError, json.JSONDecodeError, yaml.YAMLError) as error:
        parser.error(str(error))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    args.output.with_suffix('.md').write_text(markdown(report))
    print(json.dumps({'output': str(args.output), 'verdict': report['verdict'],
                      'checks': report['checks']}, indent=2))
    return 2 if args.require_pass and not report['passed'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
