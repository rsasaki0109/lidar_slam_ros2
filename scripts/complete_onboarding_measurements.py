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

"""Attach observed missing onboarding measurements without rerunning a trial.

The supplement is bound to the exact base-record bytes and never overwrites
the original evidence file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shlex
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from check_onboarding_trial import (
    SUPPLEMENT_FIELDS,
    SUPPLEMENT_SOURCE_BY_FIELD as SOURCE_BY_FIELD,
    TrialError,
    apply_measurement_supplement,
    evaluate_trial,
)


SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/onboarding-measurement-supplement-v1.schema.json'
)


def _read_object(path: Path) -> tuple[dict[str, Any], bytes]:
    try:
        raw = path.read_bytes()
        value = json.loads(raw.decode('utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise TrialError(f'cannot read JSON object {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise TrialError(f'JSON root must be an object: {path}')
    return value, raw


def _prompt_active_time(wall_time: float | None) -> float | None:
    while True:
        try:
            sys.stderr.write(
                'Observed active operator seconds '
                '(paused stopwatch; blank leaves it unknown): '
            )
            sys.stderr.flush()
            value = input().strip()
        except EOFError:
            return None
        if not value:
            return None
        try:
            parsed = float(value)
        except ValueError:
            print('Enter a finite number or leave blank.', file=sys.stderr)
            continue
        if not math.isfinite(parsed) or parsed < 0:
            print('Enter a non-negative finite number.', file=sys.stderr)
            continue
        if wall_time is not None and parsed > wall_time:
            print(
                f'Enter a value from 0 through {wall_time:.3f}.',
                file=sys.stderr,
            )
            continue
        return round(parsed, 3)


def _prompt_command_count() -> int | None:
    while True:
        try:
            sys.stderr.write(
                'Observed human-submitted command count '
                '(blank leaves it unknown): '
            )
            sys.stderr.flush()
            value = input().strip()
        except EOFError:
            return None
        if not value:
            return None
        try:
            parsed = int(value)
        except ValueError:
            print('Enter a positive integer or leave blank.', file=sys.stderr)
            continue
        if parsed < 1:
            print('Enter a positive integer or leave blank.', file=sys.stderr)
            continue
        return parsed


def _validate_explicit_values(values: dict[str, Any]) -> None:
    for field, value in values.items():
        if value is None:
            continue
        if field in {'wall_time_sec', 'active_operator_time_sec'}:
            if not math.isfinite(value) or value < 0:
                raise TrialError(f'{field} must be a finite non-negative number')
        elif field == 'command_count':
            if value < 1:
                raise TrialError('command_count must be a positive integer')
        elif value < 0:
            raise TrialError(f'{field} must be a non-negative integer')


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record', type=Path)
    parser.add_argument(
        '--output',
        type=Path,
        help=(
            'Write the supplement here; defaults to '
            '<record>.measurements.json.'
        ),
    )
    parser.add_argument(
        '--prompt-human-measurements',
        action='store_true',
        help='Prompt for active operator time and human command count.',
    )
    parser.add_argument('--input-download-bytes', type=int)
    parser.add_argument('--workflow-download-bytes', type=int)
    parser.add_argument('--wall-time-sec', type=float)
    parser.add_argument('--active-operator-time-sec', type=float)
    parser.add_argument('--command-count', type=int)
    parser.add_argument('--peak-disk-bytes', type=int)
    parser.add_argument('--output-bytes', type=int)
    parser.add_argument('--json', action='store_true')
    parser.add_argument(
        '--require-comparable',
        action='store_true',
        help='Exit 1 when the supplemented record is still not comparable.',
    )
    args = parser.parse_args(argv)
    if args.output is None:
        args.output = Path(f'{args.record}.measurements.json')
    return args


def _validation_command(record: Path, supplement: Path) -> str:
    """Return the safe next command without embedding shell syntax."""
    return shlex.join([
        'python3',
        'scripts/check_onboarding_trial.py',
        str(record),
        '--supplement',
        str(supplement),
        '--json',
        '--require-comparable',
    ])


def _supplement_values(
    args: argparse.Namespace,
    record: dict[str, Any],
) -> dict[str, Any]:
    values = {
        field: getattr(args, field)
        for field, _ in SUPPLEMENT_FIELDS
    }
    measurements = record['measurements']
    if args.prompt_human_measurements:
        print(
            'Enter only values observed during this exact trial; '
            'blank leaves a field unknown.',
            file=sys.stderr,
        )
        if (
            values['active_operator_time_sec'] is None
            and measurements['active_operator_time_sec'] is None
        ):
            values['active_operator_time_sec'] = _prompt_active_time(
                values['wall_time_sec']
                if values['wall_time_sec'] is not None
                else measurements['wall_time_sec']
            )
        if (
            values['command_count'] is None
            and measurements['command_count'] is None
        ):
            values['command_count'] = _prompt_command_count()
    _validate_explicit_values(values)
    if not any(value is not None for value in values.values()):
        raise TrialError(
            'provide at least one measurement value or use '
            '--prompt-human-measurements'
        )
    return values


def _build_supplement(
    record: dict[str, Any],
    record_bytes: bytes,
    values: dict[str, Any],
) -> dict[str, Any]:
    captured_at = datetime.now(timezone.utc).replace(microsecond=0)
    captured_text = captured_at.isoformat().replace('+00:00', 'Z')
    base_sha256 = hashlib.sha256(record_bytes).hexdigest()
    supplement_id = (
        f'measurement-{base_sha256[:16]}-'
        f'{captured_at:%Y%m%d%H%M%S}'
    )
    sources = {
        field: (
            SOURCE_BY_FIELD[field]
            if values[field] is not None
            else 'not-supplemented'
        )
        for field, _ in SUPPLEMENT_FIELDS
    }
    return {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'supplement_id': supplement_id,
        'captured_at': captured_text,
        'trial_id': record['trial_id'],
        'base_record_sha256': base_sha256,
        'measurements': values,
        'measurement_sources': sources,
        'privacy': {
            'contains_private_paths': False,
            'contains_exact_command': False,
            'contains_operator_identity': False,
            'review_before_sharing': True,
        },
    }


def _write_exclusive(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, indent=2, sort_keys=True) + '\n'
    try:
        with path.open('x', encoding='utf-8') as stream:
            stream.write(payload)
    except FileExistsError as exc:
        raise TrialError(
            f'refusing to overwrite existing supplement: {path}'
        ) from exc


def main(argv: list[str] | None = None) -> int:
    """Create one supplement and report the resulting comparability state."""
    args = _parse_args(argv)
    try:
        record, record_bytes = _read_object(args.record)
        # Validate the base before accepting a supplement; an attachment must
        # never hide a malformed product result.
        evaluate_trial(record)
        values = _supplement_values(args, record)
        supplement = _build_supplement(record, record_bytes, values)
        effective = apply_measurement_supplement(
            record,
            supplement,
            record_bytes=record_bytes,
        )
        report = evaluate_trial(effective)
        _write_exclusive(args.output, supplement)
    except (OSError, TrialError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    result = {
        'supplement_id': supplement['supplement_id'],
        'trial_id': supplement['trial_id'],
        'supplement_path': str(args.output),
        'validation_command': _validation_command(
            args.record,
            args.output,
        ),
        **report,
    }
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(
            f"Wrote measurement supplement `{supplement['supplement_id']}` "
            f"to `{args.output}`."
        )
        print(
            'Supplemented comparability: '
            f"**{'YES' if report['comparable'] else 'NO'}**"
        )
        if report['missing_measurements']:
            print('Remaining missing measurements:')
            for field in report['missing_measurements']:
                print(f'- `{field}`')
        print('')
        print('Validate this supplement with:')
        print(f"  {result['validation_command']}")
    if args.require_comparable and not report['comparable']:
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
