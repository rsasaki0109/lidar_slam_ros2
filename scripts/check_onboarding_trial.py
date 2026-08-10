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
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Validate one privacy-bounded first-map onboarding trial record."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'onboarding-trial-v1.schema.json'
)
MEASUREMENT_PATHS = (
    'input.download_bytes',
    'measurements.wall_time_sec',
    'measurements.active_operator_time_sec',
    'measurements.command_count',
    'measurements.peak_disk_bytes',
    'measurements.output_bytes',
)
IMMUTABLE_REVISION_KINDS = {'git-commit', 'image-digest'}


class TrialError(ValueError):
    """The onboarding trial record is structurally or semantically invalid."""


def _load_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise TrialError(f'cannot read JSON object {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise TrialError(f'JSON root must be an object: {path}')
    return value


def _validate_schema(
    record: dict[str, Any],
    schema: dict[str, Any],
) -> None:
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(
            schema,
            format_checker=jsonschema.FormatChecker(),
        ).validate(record)
    except (jsonschema.SchemaError, jsonschema.ValidationError) as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise TrialError(
            f'onboarding trial schema failed at '
            f'{location or "<root>"}: {exc.message}'
        ) from exc


def _path_value(record: dict[str, Any], dotted_path: str) -> Any:
    value: Any = record
    for item in dotted_path.split('.'):
        value = value[item]
    return value


def _validate_measurements(record: dict[str, Any]) -> None:
    measurements = record['measurements']
    wall_time = measurements['wall_time_sec']
    active_time = measurements['active_operator_time_sec']
    peak_disk = measurements['peak_disk_bytes']
    output_bytes = measurements['output_bytes']
    if wall_time is not None and wall_time <= 0:
        raise TrialError('wall_time_sec must be greater than zero when known')
    if (
        wall_time is not None
        and active_time is not None
        and active_time > wall_time
    ):
        raise TrialError('active_operator_time_sec cannot exceed wall_time_sec')
    if (
        peak_disk is not None
        and output_bytes is not None
        and output_bytes > peak_disk
    ):
        raise TrialError('output_bytes cannot exceed peak_disk_bytes')


def _validate_evidence(record: dict[str, Any]) -> None:
    outcome = record['outcome']
    evidence = record['evidence']
    manifest_available = outcome['manifest_status'] != 'missing'
    receipt_available = outcome['receipt_status'] != 'NOT_CREATED'
    if manifest_available != (evidence['manifest_sha256'] is not None):
        raise TrialError(
            'manifest status and manifest_sha256 availability disagree')
    if receipt_available != (evidence['receipt_sha256'] is not None):
        raise TrialError(
            'receipt status and receipt_sha256 availability disagree')


def _validate_outcome(record: dict[str, Any]) -> None:
    outcome = record['outcome']
    if outcome['status'] == 'PASS':
        expected = {
            'runner_exit_code': 0,
            'manifest_status': 'succeeded',
            'diagnosis_status': 'success',
            'verifier_status': 'PASS',
            'receipt_status': 'PASS',
            'undocumented_manual_steps': 0,
            'failure_stage': 'none',
        }
        inconsistent = [
            key for key, value in expected.items()
            if outcome[key] != value
        ]
        if inconsistent:
            raise TrialError(
                'PASS trial has inconsistent outcome fields: '
                + ', '.join(inconsistent)
            )
    else:
        if outcome['failure_stage'] == 'none':
            raise TrialError('FAIL trial must identify a failure_stage')
        if not outcome['finding_codes']:
            raise TrialError('FAIL trial must include at least one finding code')


def evaluate_trial(
    record: dict[str, Any],
    schema: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Validate one record and return a deterministic comparability report."""
    contract = schema or _load_object(DEFAULT_SCHEMA)
    _validate_schema(record, contract)
    _validate_measurements(record)
    _validate_evidence(record)
    _validate_outcome(record)

    missing = [
        path for path in MEASUREMENT_PATHS
        if _path_value(record, path) is None
    ]
    blockers: list[str] = []
    if missing:
        blockers.append('measurements_incomplete')
    if not record['environment']['clean_start']:
        blockers.append('environment_not_clean')
    if record['environment']['revision']['kind'] not in IMMUTABLE_REVISION_KINDS:
        blockers.append('revision_not_immutable')
    if record['outcome']['status'] != 'PASS':
        blockers.append('outcome_failed')
    if record['outcome']['undocumented_manual_steps']:
        blockers.append('undocumented_manual_steps')

    return {
        'schema_version': 1,
        'trial_id': record['trial_id'],
        'outcome_status': record['outcome']['status'],
        'measurement_status': 'INCOMPLETE' if missing else 'COMPLETE',
        'comparable': not blockers,
        'missing_measurements': missing,
        'comparability_blockers': blockers,
    }


def render_markdown(report: dict[str, Any]) -> str:
    """Render a compact, reviewer-facing trial audit."""
    lines = [
        '# Onboarding trial audit',
        '',
        f"- Trial: `{report['trial_id']}`",
        f"- Outcome: **{report['outcome_status']}**",
        f"- Measurements: **{report['measurement_status']}**",
        (
            '- Comparable onboarding baseline: **YES**'
            if report['comparable']
            else '- Comparable onboarding baseline: **NO**'
        ),
        '',
        '## Missing measurements',
        '',
    ]
    if report['missing_measurements']:
        lines.extend(
            f'- `{item}`' for item in report['missing_measurements'])
    else:
        lines.append('- None.')
    lines.extend(['', '## Comparability blockers', ''])
    if report['comparability_blockers']:
        lines.extend(
            f'- `{item}`' for item in report['comparability_blockers'])
    else:
        lines.append('- None.')
    return '\n'.join(lines) + '\n'


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Validate a first-map onboarding trial and report whether it is '
            'complete enough for cross-path and cross-release comparison.'
        ),
    )
    parser.add_argument('record', type=Path)
    parser.add_argument('--schema', type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print the comparability report as JSON.',
    )
    parser.add_argument(
        '--require-comparable',
        action='store_true',
        help='Exit 1 when the valid trial is not a comparable PASS baseline.',
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """CLI entrypoint; invalid records exit 2 and unmet gates exit 1."""
    args = _parse_args(argv)
    try:
        report = evaluate_trial(
            _load_object(args.record),
            _load_object(args.schema),
        )
    except TrialError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(render_markdown(report), end='')
    if args.require_comparable and not report['comparable']:
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
