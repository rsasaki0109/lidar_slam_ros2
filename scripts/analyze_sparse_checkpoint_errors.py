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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the distribution.
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
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
# OF SUCH DAMAGE.

"""Attribute SE(3)-aligned sparse-checkpoint errors to trajectory candidates."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load_tum(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load sorted TUM timestamps and translations."""
    rows = []
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        fields = line.split()
        if not fields or fields[0].startswith('#'):
            continue
        if len(fields) != 8:
            raise ValueError(f'{path}:{line_number}: expected 8 TUM fields')
        rows.append([float(value) for value in fields[:4]])
    if not rows:
        raise ValueError(f'{path}: no TUM poses')
    data = np.asarray(rows, dtype=float)
    data = data[np.argsort(data[:, 0], kind='stable')]
    if not np.isfinite(data).all() or np.any(np.diff(data[:, 0]) <= 0.0):
        raise ValueError(f'{path}: timestamps and positions must be finite and unique')
    return data[:, 0], data[:, 1:4]


def associate(reference_stamps: np.ndarray, estimate_stamps: np.ndarray,
              max_difference: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Nearest unique timestamp association, matching the public-suite evaluator."""
    candidates = []
    for estimate_index, stamp in enumerate(estimate_stamps):
        insertion = int(np.searchsorted(reference_stamps, stamp))
        for reference_index in (insertion - 1, insertion):
            if 0 <= reference_index < len(reference_stamps):
                difference = abs(float(reference_stamps[reference_index] - stamp))
                if difference <= max_difference:
                    candidates.append((difference, estimate_index, reference_index))
    used_estimate: set[int] = set()
    used_reference: set[int] = set()
    matches = []
    for difference, estimate_index, reference_index in sorted(candidates):
        if estimate_index in used_estimate or reference_index in used_reference:
            continue
        used_estimate.add(estimate_index)
        used_reference.add(reference_index)
        matches.append((reference_index, estimate_index, difference))
    matches.sort()
    return (
        np.asarray([row[0] for row in matches], dtype=np.int64),
        np.asarray([row[1] for row in matches], dtype=np.int64),
        np.asarray([row[2] for row in matches], dtype=float),
    )


def rigid_align(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Apply scale-free Kabsch/Umeyama alignment to translations."""
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - source_mean).T @ (target - target_mean))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
    return (rotation @ source.T).T + target_mean - rotation @ source_mean


def load_labels(path: Path | None, reference_stamps: np.ndarray) -> list[dict]:
    """Load optional RTK-SLAM point_id/env labels and verify row timestamps."""
    labels = [{'point_id': f'checkpoint_{index:03d}', 'env': None}
              for index in range(len(reference_stamps))]
    if path is None:
        return labels
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != len(reference_stamps):
        raise ValueError('reference CSV row count differs from reference TUM')
    for index, row in enumerate(rows):
        if abs(float(row['timestamp']) - reference_stamps[index]) > 1.0e-3:
            raise ValueError(f'reference CSV timestamp differs at row {index + 2}')
        labels[index] = {'point_id': row.get('point_id') or labels[index]['point_id'],
                         'env': row.get('env') or None}
    return labels


def analyze(reference_tum: Path, estimates: list[tuple[str, Path]], baseline: str,
            max_difference: float, reference_csv: Path | None = None) -> dict:
    """Return per-checkpoint errors and method summaries."""
    reference_stamps, reference_positions = load_tum(reference_tum)
    labels = load_labels(reference_csv, reference_stamps)
    method_errors: dict[str, dict[int, float]] = {}
    methods = []
    for label, path in estimates:
        estimate_stamps, estimate_positions = load_tum(path)
        reference_ids, estimate_ids, differences = associate(
            reference_stamps, estimate_stamps, max_difference)
        if len(reference_ids) < 3:
            raise ValueError(f'{label}: fewer than three associated checkpoints')
        aligned = rigid_align(
            estimate_positions[estimate_ids], reference_positions[reference_ids])
        errors = np.linalg.norm(aligned - reference_positions[reference_ids], axis=1)
        method_errors[label] = {
            int(reference_id): float(error)
            for reference_id, error in zip(reference_ids, errors)
        }
        methods.append({
            'label': label, 'trajectory': str(path.resolve()),
            'associated_checkpoints': int(len(errors)),
            'rmse_m': float(np.sqrt(np.mean(errors ** 2))),
            'mean_m': float(np.mean(errors)), 'max_m': float(np.max(errors)),
            'max_time_difference_s': float(np.max(differences)),
        })
    if baseline not in method_errors:
        raise ValueError(f'baseline label not found: {baseline}')
    checkpoints = []
    for index, stamp in enumerate(reference_stamps):
        errors = {label: values.get(index) for label, values in method_errors.items()}
        baseline_error = errors[baseline]
        deltas = {label: (None if error is None or baseline_error is None
                          else error - baseline_error)
                  for label, error in errors.items() if label != baseline}
        checkpoints.append({
            'index': index, 'timestamp': float(stamp), **labels[index],
            'errors_m': errors, 'delta_from_baseline_m': deltas,
        })
    return {
        'schema_version': 1, 'alignment': 'se3_kabsch_no_scale',
        'association': 'nearest_unique_with_tolerance',
        'max_time_difference_s': max_difference,
        'reference_tum': str(reference_tum.resolve()),
        'reference_csv': str(reference_csv.resolve()) if reference_csv else None,
        'baseline_label': baseline, 'methods': methods, 'checkpoints': checkpoints,
    }


def markdown(report: dict) -> str:
    """Render a compact human-reviewable checkpoint table."""
    labels = [method['label'] for method in report['methods']]
    lines = ['# Sparse checkpoint error attribution', '',
             f"Alignment: `{report['alignment']}`  ",
             f"Baseline: `{report['baseline_label']}`", '',
             '| method | checkpoints | RMSE (m) | mean (m) | max (m) |',
             '| --- | ---: | ---: | ---: | ---: |']
    for method in report['methods']:
        lines.append(
            f"| {method['label']} | {method['associated_checkpoints']} | "
            f"{method['rmse_m']:.6f} | {method['mean_m']:.6f} | {method['max_m']:.6f} |")
    lines += ['', '| point | env | ' + ' | '.join(labels) + ' |',
              '| --- | --- | ' + ' | '.join('---:' for _ in labels) + ' |']
    for checkpoint in report['checkpoints']:
        values = ['n/a' if checkpoint['errors_m'][label] is None
                  else f"{checkpoint['errors_m'][label]:.6f}"
                  for label in labels]
        lines.append(f"| {checkpoint['point_id']} | {checkpoint['env'] or '-'} | "
                     + ' | '.join(values) + ' |')
    return '\n'.join(lines) + '\n'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-tum', type=Path, required=True)
    parser.add_argument('--reference-csv', type=Path)
    parser.add_argument('--estimate', action='append', required=True,
                        help='LABEL=trajectory.tum; repeat for each method')
    parser.add_argument('--baseline-label', required=True)
    parser.add_argument('--max-time-difference', type=float, default=2.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.max_time_difference < 0.0:
        parser.error('--max-time-difference must be non-negative')
    estimates = []
    for specification in args.estimate:
        if '=' not in specification:
            parser.error('--estimate must use LABEL=PATH')
        label, path = specification.split('=', 1)
        estimates.append((label, Path(path)))
    if len({label for label, _ in estimates}) != len(estimates):
        parser.error('estimate labels must be unique')
    try:
        report = analyze(args.reference_tum, estimates, args.baseline_label,
                         args.max_time_difference, args.reference_csv)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    args.output.with_suffix('.md').write_text(markdown(report))
    print(json.dumps({'output': str(args.output), 'methods': report['methods']}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
