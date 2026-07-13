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

"""Aggregate cross-repository manifests into a conservative adoption gate."""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import defaultdict
from pathlib import Path
from typing import Any


def sha256(path: Path) -> str:
    """Hash a file or directory tree with the benchmark manifest algorithm."""
    digest = hashlib.sha256()
    files = [path] if path.is_file() else sorted(
        candidate for candidate in path.rglob('*') if candidate.is_file())
    if not files:
        raise ValueError(f'{path}: input is not a file or non-empty directory')
    for candidate in files:
        if path.is_dir():
            digest.update(candidate.relative_to(path).as_posix().encode())
            digest.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def finite_number(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool) and
            value == value and abs(float(value)) != float('inf'))


def input_integrity(manifest: dict[str, Any]) -> tuple[bool, list[str]]:
    issues = []
    for name, record in manifest.get('inputs', {}).items():
        if not isinstance(record, dict):
            issues.append(f'{name}: invalid input record')
            continue
        path_value = record.get('path')
        expected = record.get('sha256')
        if not isinstance(path_value, str) or not isinstance(expected, str):
            issues.append(f'{name}: missing path or sha256')
            continue
        path = Path(path_value)
        if not path.exists():
            issues.append(f'{name}: file not found')
        else:
            try:
                if sha256(path) != expected:
                    issues.append(f'{name}: sha256 mismatch')
            except ValueError as error:
                issues.append(f'{name}: {error}')
    if not manifest.get('inputs'):
        issues.append('no hashed inputs')
    raw_names = [name for name in manifest.get('inputs', {})
                 if name.startswith('raw_') or name.startswith('raw_artifact.')]
    if not raw_names:
        issues.append('no raw source artifact')
    return not issues, issues


def runtime_memory_complete(manifest: dict[str, Any]) -> bool:
    runtime = manifest.get('reports', {}).get('runtime', {})
    return (finite_number(runtime.get('realtime_factor')) and
            finite_number(runtime.get('peak_rss_mb')) and
            runtime.get('process_exit_status') == 0)


def aggregate(manifests: list[tuple[Path, dict[str, Any]]],
              minimum_complete_datasets: int = 2) -> dict[str, Any]:
    if not manifests:
        raise ValueError('at least one manifest is required')
    profiles = {document.get('profile') for _, document in manifests}
    if len(profiles) != 1 or None in profiles:
        raise ValueError('all manifests must use one named profile')
    policies = [document.get('adoption_policy', {}) for _, document in manifests]
    if any(policy != policies[0] for policy in policies[1:]):
        raise ValueError('all manifests must use the same adoption policy')
    policy = policies[0]
    minimum_improved = int(policy.get('minimum_improved_datasets', 2))
    maximum_regression = float(
        policy.get('maximum_primary_metric_regression_percent', 0.0))

    entries = []
    by_dataset: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for path, document in manifests:
        dataset = document.get('dataset')
        if not isinstance(dataset, str) or not dataset:
            raise ValueError(f'{path}: missing dataset name')
        integrity_ok, integrity_issues = input_integrity(document)
        primary = document.get('primary_delta', {})
        entry = {
            'manifest': str(path.resolve()),
            'manifest_sha256': sha256(path),
            'dataset': dataset,
            'complete': document.get('evidence', {}).get('complete') is True,
            'runtime_memory_complete': runtime_memory_complete(document),
            'input_integrity': integrity_ok,
            'input_integrity_issues': integrity_issues,
            'primary': primary,
        }
        entries.append(entry)
        by_dataset[dataset].append(entry)

    datasets = []
    for name, records in sorted(by_dataset.items()):
        comparable = [row['primary'].get('improved') for row in records
                      if isinstance(row['primary'].get('improved'), bool)]
        changes = [float(row['primary']['change_percent']) for row in records
                   if finite_number(row['primary'].get('change_percent'))]
        datasets.append({
            'dataset': name,
            'captures': len(records),
            'complete': all(row['complete'] for row in records),
            'improved': bool(comparable) and all(comparable),
            'regressed': any(value is False for value in comparable),
            'maximum_regression_percent': max([0.0, *changes]),
        })

    complete_count = sum(row['complete'] for row in datasets)
    improved_count = sum(row['improved'] for row in datasets)
    worst_regression = max(row['maximum_regression_percent'] for row in datasets)
    gates = {
        'all_manifests_complete': all(row['complete'] for row in entries),
        'multiple_complete_datasets': complete_count >= minimum_complete_datasets,
        'minimum_improved_datasets': improved_count >= minimum_improved,
        'maximum_primary_regression': worst_regression <= maximum_regression,
        'runtime_and_memory': (
            all(row['runtime_memory_complete'] for row in entries)
            if policy.get('require_runtime_and_memory', False) else True),
        'raw_artifact_integrity': (
            all(row['input_integrity'] for row in entries)
            if policy.get('require_raw_artifacts', False) else True),
    }
    return {
        'schema_version': 1,
        'profile': next(iter(profiles)),
        'adoption_policy': policy,
        'minimum_complete_datasets': minimum_complete_datasets,
        'summary': {
            'unique_datasets': len(datasets),
            'complete_datasets': complete_count,
            'improved_datasets': improved_count,
            'worst_primary_regression_percent': worst_regression,
        },
        'datasets': datasets,
        'manifests': entries,
        'gates': gates,
        'verdict': 'ADOPT' if all(gates.values()) else 'DO_NOT_ADOPT',
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, action='append', required=True)
    parser.add_argument('--minimum-complete-datasets', type=int, default=2)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument(
        '--require-adopt', action='store_true',
        help='return 2 when the suite gate rejects the candidate')
    args = parser.parse_args()
    if args.minimum_complete_datasets < 2:
        parser.error('--minimum-complete-datasets must be at least 2')
    try:
        inputs = [(path, json.loads(path.read_text())) for path in args.manifest]
        report = aggregate(inputs, args.minimum_complete_datasets)
    except (OSError, json.JSONDecodeError, ValueError) as error:
        parser.error(str(error))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({
        'output': str(args.out), 'verdict': report['verdict'],
        'summary': report['summary'], 'gates': report['gates']}, indent=2))
    return 2 if args.require_adopt and report['verdict'] != 'ADOPT' else 0


if __name__ == '__main__':
    raise SystemExit(main())
