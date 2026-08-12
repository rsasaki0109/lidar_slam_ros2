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

"""Audit a neutral six-task lidar_slam_ros2 and GLIM usability scorecard."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
TRIAL_SCHEMA_PATH = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'usability-scorecard-trial-v1.schema.json'
)
INDEX_SCHEMA_PATH = (
    REPO_ROOT / 'docs' / 'schemas'
    / 'usability-scorecard-evidence-index-v1.schema.json'
)
EVIDENCE_INDEX_PATH = (
    REPO_ROOT / 'docs' / 'contracts'
    / 'glim-usability-scorecard-evidence-v1.json'
)
TRIAL_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/usability-scorecard-trial-v1.schema.json'
)
INDEX_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/usability-scorecard-evidence-index-v1.schema.json'
)
SCORECARD_ID = 'stable-release-lidarslam-vs-glim-v1'
PRODUCT_IDS = ('lidarslam_ros2', 'glim')
TASK_CONTRACTS = (
    {
        'task_id': 'discover-supported-path',
        'required_metrics': (
            'wall_time_sec',
            'active_operator_time_sec',
            'command_count',
            'failure_count',
        ),
        'checks': ('supported-command-identified',),
    },
    {
        'task_id': 'run-fixed-demo',
        'required_metrics': (
            'wall_time_sec',
            'active_operator_time_sec',
            'command_count',
            'workflow_download_bytes',
            'peak_disk_bytes',
            'failure_count',
            'output_bytes',
        ),
        'checks': ('fixed-demo-completed', 'result-verifiable'),
    },
    {
        'task_id': 'inspect-own-bag',
        'required_metrics': (
            'wall_time_sec',
            'active_operator_time_sec',
            'command_count',
            'failure_count',
        ),
        'checks': (
            'topics-explained',
            'frames-explained',
            'timestamps-explained',
            'profile-choice-explained',
        ),
    },
    {
        'task_id': 'produce-downstream-artifact',
        'required_metrics': (
            'wall_time_sec',
            'active_operator_time_sec',
            'command_count',
            'failure_count',
            'output_bytes',
        ),
        'checks': (
            'downstream-artifact-produced',
            'downstream-artifact-verified',
        ),
    },
    {
        'task_id': 'understand-failure',
        'required_metrics': (
            'wall_time_sec',
            'active_operator_time_sec',
            'command_count',
            'failure_count',
        ),
        'checks': (
            'public-error-has-stable-code',
            'safe-recovery-action-linked',
        ),
    },
    {
        'task_id': 'repeat-or-upgrade',
        'required_metrics': (
            'wall_time_sec',
            'active_operator_time_sec',
            'command_count',
            'workflow_download_bytes',
            'failure_count',
        ),
        'checks': ('same-command-preserved', 'output-contract-preserved'),
    },
)
TASK_IDS = tuple(item['task_id'] for item in TASK_CONTRACTS)
PAIR_FIELDS = (
    'comparison_pair_id',
    'ros_distro',
    'os_family',
    'architecture',
    'hardware_class',
)
PRIVATE_PATH = re.compile(r'(?:/home/|/Users/|[A-Za-z]:\\)[^\s]+')


class ScorecardError(ValueError):
    """The scorecard cannot be evaluated without inventing evidence."""


def _load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ScorecardError(f'cannot read {label} {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise ScorecardError(f'{label} root is not an object: {path}')
    return value


def _schema_validate(
    value: dict[str, Any],
    schema_path: Path,
    label: str,
) -> None:
    schema = _load_object(schema_path, f'{label} schema')
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        validator = jsonschema.Draft7Validator(
            schema,
            format_checker=jsonschema.FormatChecker(),
        )
        errors = sorted(
            validator.iter_errors(value),
            key=lambda item: [str(part) for part in item.absolute_path],
        )
    except jsonschema.SchemaError as exc:
        raise ScorecardError(
            f'{label} schema is invalid: {exc.message}') from exc
    if errors:
        error = errors[0]
        path = '.'.join(str(part) for part in error.absolute_path) or '<root>'
        raise ScorecardError(
            f'{label} schema failed at {path}: {error.message}')


def validate_trial(record: dict[str, Any]) -> None:
    """Validate schema and task-specific evidence invariants."""
    _schema_validate(record, TRIAL_SCHEMA_PATH, 'trial')
    if record['schema_uri'] != TRIAL_SCHEMA_URI:
        raise ScorecardError('trial uses an unsupported schema URI')
    tasks = record['tasks']
    task_ids = tuple(task['task_id'] for task in tasks)
    if task_ids != TASK_IDS:
        raise ScorecardError(
            'trial tasks must use the fixed scorecard order: '
            + ', '.join(TASK_IDS)
        )

    for contract, task in zip(TASK_CONTRACTS, tasks):
        check_ids = tuple(check['id'] for check in task['checks'])
        if check_ids != contract['checks']:
            raise ScorecardError(
                f"{task['task_id']} checks must be: "
                + ', '.join(contract['checks'])
            )
        commands = task['exact_commands']
        measurements = task['measurements']
        command_count = measurements['command_count']
        if command_count is not None and command_count != len(commands):
            raise ScorecardError(
                f"{task['task_id']} command_count does not match the exact "
                'command sequence')
        for command in commands:
            if '\n' in command or '\r' in command:
                raise ScorecardError(
                    f"{task['task_id']} command must be one line")
            if PRIVATE_PATH.search(command):
                raise ScorecardError(
                    f"{task['task_id']} command contains a private path")
        for name, value in measurements.items():
            if isinstance(value, float) and not math.isfinite(value):
                raise ScorecardError(
                    f"{task['task_id']} {name} must be finite")
        wall_time = measurements['wall_time_sec']
        active_time = measurements['active_operator_time_sec']
        if (
            wall_time is not None
            and active_time is not None
            and active_time > wall_time
        ):
            raise ScorecardError(
                f"{task['task_id']} active time exceeds wall time")
        passed = [item['passed'] for item in task['checks']]
        outcome = task['outcome']['status']
        if outcome == 'PASS' and not all(passed):
            raise ScorecardError(
                f"{task['task_id']} PASS has a failed criterion")
        if outcome == 'FAIL' and all(passed):
            raise ScorecardError(
                f"{task['task_id']} FAIL has no failed criterion")


def load_evidence_index(
    index_path: Path = EVIDENCE_INDEX_PATH,
    repo_root: Path = REPO_ROOT,
) -> list[dict[str, Any]]:
    """Load reviewed product records from the checked-in evidence index."""
    index = _load_object(index_path, 'evidence index')
    _schema_validate(index, INDEX_SCHEMA_PATH, 'evidence index')
    if index['schema_uri'] != INDEX_SCHEMA_URI:
        raise ScorecardError('evidence index uses an unsupported schema URI')
    product_ids = tuple(row['product_id'] for row in index['rows'])
    if product_ids != PRODUCT_IDS:
        raise ScorecardError(
            'evidence index rows must use the fixed order: '
            + ', '.join(PRODUCT_IDS))

    records = []
    for row in index['rows']:
        relative = row['record_path']
        if relative is None:
            continue
        path = PurePosixPath(relative)
        if (
            str(path) != relative
            or relative.startswith('/')
            or '..' in path.parts
            or '\\' in relative
        ):
            raise ScorecardError(
                f'evidence index contains an unsafe path: {relative!r}')
        candidate = repo_root / relative
        if candidate.is_symlink() or not candidate.is_file():
            raise ScorecardError(
                f'evidence record is not a regular file: {relative}')
        record = _load_object(candidate, 'trial')
        validate_trial(record)
        if record['product']['id'] != row['product_id']:
            raise ScorecardError(
                f"evidence row {row['product_id']} points to another product")
        records.append(record)
    return records


def _metric_missing(
    task: Mapping[str, Any],
    names: Sequence[str],
) -> list[str]:
    missing = []
    for name in names:
        value = task['measurements'][name]
        if value is None or isinstance(value, bool):
            missing.append(name)
        elif isinstance(value, float) and not math.isfinite(value):
            missing.append(name)
    return missing


def _product_summary(record: dict[str, Any] | None, product_id: str
                     ) -> dict[str, Any]:
    if record is None:
        return {
            'product_id': product_id,
            'present': False,
            'trial_id': None,
            'version': None,
            'revision': None,
            'operator_class': None,
            'first_attempt': None,
        }
    return {
        'product_id': product_id,
        'present': True,
        'trial_id': record['trial_id'],
        'version': record['product']['version'],
        'revision': record['product']['revision'],
        'operator_class': record['operator']['class'],
        'first_attempt': record['operator']['first_attempt'],
    }


def _task_result(task: Mapping[str, Any] | None) -> dict[str, Any] | None:
    if task is None:
        return None
    checks = task['checks']
    return {
        'outcome_status': task['outcome']['status'],
        'measurements': task['measurements'],
        'checks_passed': sum(1 for item in checks if item['passed']),
        'checks_total': len(checks),
        'undocumented_manual_steps': (
            task['outcome']['undocumented_manual_steps']
        ),
    }


def _task_comparison(
    contract: Mapping[str, Any],
    selected: Mapping[str, dict[str, Any]],
) -> dict[str, Any]:
    task_id = contract['task_id']
    tasks: dict[str, Mapping[str, Any] | None] = {}
    blockers: list[str] = []
    for product_id in PRODUCT_IDS:
        record = selected.get(product_id)
        if record is None:
            tasks[product_id] = None
            blockers.append(f'{product_id}-record-missing')
            continue
        task = next(
            item for item in record['tasks']
            if item['task_id'] == task_id
        )
        tasks[product_id] = task
        if not record['product']['publicly_resolvable']:
            blockers.append(f'{product_id}-identity-not-public')
        if not record['environment']['clean_start']:
            blockers.append(f'{product_id}-host-not-clean')
        missing = _metric_missing(task, contract['required_metrics'])
        if missing:
            blockers.append(
                f'{product_id}-measurements-missing:' + ','.join(missing))
        if task['evidence']['transcript_sha256'] is None:
            blockers.append(f'{product_id}-transcript-missing')
        if task['outcome']['undocumented_manual_steps']:
            blockers.append(f'{product_id}-undocumented-manual-steps')

    left_record = selected.get(PRODUCT_IDS[0])
    right_record = selected.get(PRODUCT_IDS[1])
    if left_record is not None and right_record is not None:
        for field in PAIR_FIELDS:
            if (
                left_record['environment'][field]
                != right_record['environment'][field]
            ):
                blockers.append(f'environment-{field}-mismatch')
        for field in ('class', 'cohort_id', 'first_attempt'):
            if (
                left_record['operator'][field]
                != right_record['operator'][field]
            ):
                blockers.append(f'operator-{field}-mismatch')
        product_orders = {
            left_record['operator']['product_order'],
            right_record['operator']['product_order'],
        }
        if product_orders != {'first', 'second'}:
            blockers.append('operator-product-order-invalid')
        left_task = tasks[PRODUCT_IDS[0]]
        right_task = tasks[PRODUCT_IDS[1]]
        if left_task is not None and right_task is not None:
            if left_task['input_id'] != right_task['input_id']:
                blockers.append('input-id-mismatch')

    blockers = sorted(set(blockers))
    return {
        'task_id': task_id,
        'comparable': not blockers,
        'comparability_blockers': blockers,
        'results': {
            product_id: _task_result(tasks[product_id])
            for product_id in PRODUCT_IDS
        },
    }


def evaluate_scorecard(records: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Return a no-winner, task-independent usability comparison report."""
    selected: dict[str, dict[str, Any]] = {}
    trial_ids: set[str] = set()
    for record in records:
        validate_trial(record)
        trial_id = record['trial_id']
        if trial_id in trial_ids:
            raise ScorecardError(f'duplicate trial_id: {trial_id}')
        trial_ids.add(trial_id)
        product_id = record['product']['id']
        if product_id in selected:
            raise ScorecardError(f'duplicate product row: {product_id}')
        selected[product_id] = record

    tasks = [
        _task_comparison(contract, selected)
        for contract in TASK_CONTRACTS
    ]
    comparable = sum(1 for task in tasks if task['comparable'])
    records_present = len(selected)
    external_first_attempt = (
        records_present == len(PRODUCT_IDS)
        and all(
            record['operator']['class'] == 'external'
            and record['operator']['first_attempt']
            for record in selected.values()
        )
    )
    public_scorecard_ready = (
        comparable == len(TASK_CONTRACTS)
        and external_first_attempt
    )
    if public_scorecard_ready:
        status = 'READY'
    elif comparable:
        status = 'PARTIAL'
    else:
        status = 'NOT_READY'

    actions = []
    missing_products = [
        product_id for product_id in PRODUCT_IDS if product_id not in selected
    ]
    if missing_products:
        actions.append(
            'Capture missing product records: ' + ', '.join(missing_products))
    for task in tasks:
        if task['comparable']:
            continue
        actions.append(
            f"Resolve {task['task_id']}: "
            + ', '.join(task['comparability_blockers']))
    if comparable == len(TASK_CONTRACTS) and not external_first_attempt:
        actions.append(
            'Repeat the complete paired scorecard with an external first-time '
            'operator before public parity claims.')

    return {
        'schema_version': 1,
        'scorecard_id': SCORECARD_ID,
        'status': status,
        'products': [
            _product_summary(selected.get(product_id), product_id)
            for product_id in PRODUCT_IDS
        ],
        'tasks': tasks,
        'summary': {
            'required_product_records': len(PRODUCT_IDS),
            'records_present': records_present,
            'required_tasks': len(TASK_CONTRACTS),
            'comparable_tasks': comparable,
            'external_first_attempt_pair': external_first_attempt,
            'public_scorecard_ready': public_scorecard_ready,
        },
        'decision': {'actions': actions},
        'comparison_policy': {
            'task_independent': True,
            'overall_winner_inferred': False,
            'missing_evidence_counts_as_success': False,
        },
        'remote_mutations_performed': False,
    }


def render_report(report: Mapping[str, Any]) -> str:
    """Render the bounded status without inventing an overall winner."""
    summary = report['summary']
    lines = [
        'lidar_slam_ros2 / GLIM usability scorecard',
        f"Status: {report['status']}",
        (
            'Records: '
            f"{summary['records_present']}/"
            f"{summary['required_product_records']}"
        ),
        (
            'Comparable tasks: '
            f"{summary['comparable_tasks']}/{summary['required_tasks']}"
        ),
        'Overall winner: not inferred; inspect each overlapping task.',
    ]
    if report['decision']['actions']:
        lines.append('Next actions:')
        lines.extend(
            f'  - {action}' for action in report['decision']['actions']
        )
    return '\n'.join(lines)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--record',
        action='append',
        default=[],
        type=Path,
        help='Explicit trial record; repeat for each product.',
    )
    parser.add_argument(
        '--evidence-index',
        type=Path,
        default=EVIDENCE_INDEX_PATH,
        help='Checked-in evidence index used when --record is omitted.',
    )
    parser.add_argument('--json', action='store_true')
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Validate evidence and print the current no-winner decision."""
    args = _parser().parse_args(argv)
    try:
        if args.record:
            if args.evidence_index != EVIDENCE_INDEX_PATH:
                raise ScorecardError(
                    '--record and an explicit --evidence-index are mutually '
                    'exclusive')
            records = [_load_object(path, 'trial') for path in args.record]
        else:
            records = load_evidence_index(args.evidence_index)
        report = evaluate_scorecard(records)
    except ScorecardError as exc:
        print(f'usability scorecard error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(render_report(report))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
