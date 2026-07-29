#!/usr/bin/env python3
"""Fail-closed audit of the tracked lidarslam_ros2 v1.0 readiness gates."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'v1-readiness.json'
DEFAULT_CONTRACT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'v1-readiness-contract-v1.schema.json'
)
DEFAULT_REPORT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'v1-readiness-report-v1.schema.json'
)
DEFAULT_LEDGER = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'external-first-map-validations.json'
)
DEFAULT_LEDGER_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'external-first-map-validations-v1.schema.json'
)
REPORT_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/v1-readiness-report-v1.schema.json'
)
EXPECTED_GATE_IDS = {
    'first-success',
    'official-surface',
    'diagnosability',
    'reproducibility',
    'distribution',
    'reliability',
    'compatibility',
    'oss-operations',
    'external-adoption',
    'release-publication',
}
SEMVER_PATTERN = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+$')


class ReadinessError(ValueError):
    """The readiness contract or its evidence is structurally invalid."""


def _load_object(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ReadinessError(f'cannot read JSON object {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise ReadinessError(f'JSON root must be an object: {path}')
    return payload


def _validate_schema(payload: dict[str, Any], schema_path: Path) -> None:
    schema = _load_object(schema_path)
    try:
        jsonschema.Draft202012Validator.check_schema(schema)
        jsonschema.Draft202012Validator(schema).validate(payload)
    except (jsonschema.SchemaError, jsonschema.ValidationError) as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise ReadinessError(
            f'{schema_path.name} validation failed at '
            f'{location or "<root>"}: {exc.message}'
        ) from exc


def _version_tuple(value: str) -> tuple[int, int, int]:
    if not SEMVER_PATTERN.fullmatch(value):
        raise ReadinessError(
            f'product version must be MAJOR.MINOR.PATCH: {value!r}')
    return tuple(int(part) for part in value.split('.'))  # type: ignore[return-value]


def _external_checker() -> Any:
    path = REPO_ROOT / 'scripts' / 'check_external_first_map_readiness.py'
    spec = importlib.util.spec_from_file_location(
        'external_first_map_readiness_for_v1',
        path,
    )
    if spec is None or spec.loader is None:
        raise ReadinessError(f'cannot load external readiness checker: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _git_tags(repo_root: Path) -> set[str]:
    result = subprocess.run(
        ['git', 'tag', '--list'],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise ReadinessError(
            f'cannot list git tags: {result.stderr.strip()}')
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def _missing_evidence(
    repo_root: Path,
    evidence: list[str],
) -> list[str]:
    root = repo_root.resolve()
    missing: list[str] = []
    for relative_text in evidence:
        relative = Path(relative_text)
        if relative.is_absolute():
            raise ReadinessError(
                f'evidence path must be repository-relative: {relative_text}')
        resolved = (root / relative).resolve()
        try:
            resolved.relative_to(root)
        except ValueError as exc:
            raise ReadinessError(
                f'evidence path escapes repository: {relative_text}') from exc
        if not resolved.is_file():
            missing.append(relative_text)
    return missing


def evaluate_readiness(
    *,
    repo_root: Path = REPO_ROOT,
    contract_path: Path = DEFAULT_CONTRACT,
    contract_schema_path: Path = DEFAULT_CONTRACT_SCHEMA,
    report_schema_path: Path = DEFAULT_REPORT_SCHEMA,
    ledger_path: Path = DEFAULT_LEDGER,
    ledger_schema_path: Path = DEFAULT_LEDGER_SCHEMA,
    tags: set[str] | None = None,
    external_report: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Validate the contract and return one deterministic readiness report."""
    repo_root = repo_root.resolve()
    contract = _load_object(contract_path)
    _validate_schema(contract, contract_schema_path)

    gate_ids = [gate['id'] for gate in contract['gates']]
    if len(gate_ids) != len(set(gate_ids)):
        raise ReadinessError('readiness gate ids must be unique')
    if set(gate_ids) != EXPECTED_GATE_IDS:
        raise ReadinessError(
            'readiness contract gate ids differ from the v1.0 scope: '
            f'missing={sorted(EXPECTED_GATE_IDS - set(gate_ids))}, '
            f'extra={sorted(set(gate_ids) - EXPECTED_GATE_IDS)}'
        )

    version_path = repo_root / contract['product_version_source']
    try:
        product_version = version_path.read_text(encoding='utf-8').strip()
    except OSError as exc:
        raise ReadinessError(
            f'cannot read product version {version_path}: {exc}') from exc
    product_tuple = _version_tuple(product_version)
    minimum_version = contract['minimum_release_candidate_version']
    minimum_tuple = _version_tuple(minimum_version)
    expected_tag = f'v{product_version}'
    available_tags = _git_tags(repo_root) if tags is None else tags
    minimum_version_met = product_tuple >= minimum_tuple
    tag_present = expected_tag in available_tags

    if external_report is None:
        checker = _external_checker()
        try:
            external_report = checker.validate_ledger(
                ledger_path,
                ledger_schema_path,
            )
        except checker.LedgerError as exc:
            raise ReadinessError(
                f'external first-map ledger invalid: {exc}') from exc

    gate_reports: list[dict[str, Any]] = []
    for gate in contract['gates']:
        blockers = list(gate['blockers'])
        missing = _missing_evidence(repo_root, gate['evidence'])
        blockers.extend(
            f'Missing tracked evidence: {path}' for path in missing)
        complete = gate['state'] == 'complete' and not missing
        detail = (
            'Tracked gate attestation and every evidence path are complete.'
            if complete
            else 'Tracked gate remains incomplete.'
        )

        if gate['id'] == 'external-adoption':
            complete = (
                gate['state'] == 'complete'
                and external_report['status'] == 'READY'
                and not missing
            )
            detail = (
                f"{external_report['accepted_validations']} / "
                f"{external_report['required_validations']} accepted "
                'independent validations.'
            )
            if external_report['status'] != 'READY':
                dynamic = (
                    f"{external_report['remaining_validations']} independent "
                    'first-map validation(s) remain.'
                )
                if dynamic not in blockers:
                    blockers.append(dynamic)

        if gate['id'] == 'release-publication':
            complete = (
                gate['state'] == 'complete'
                and minimum_version_met
                and tag_present
                and not missing
            )
            detail = (
                f'VERSION={product_version}; minimum={minimum_version}; '
                f'expected local tag={expected_tag}; '
                f'tag_present={str(tag_present).lower()}.'
            )
            if not minimum_version_met:
                blockers.append(
                    f'VERSION {product_version} is below the stable release '
                    f'candidate floor {minimum_version}.')
            if not tag_present:
                blockers.append(
                    f'Immutable release tag {expected_tag} is not present.')

        gate_reports.append({
            'id': gate['id'],
            'title': gate['title'],
            'status': 'COMPLETE' if complete else 'INCOMPLETE',
            'evidence': gate['evidence'],
            'blockers': blockers,
            'detail': detail,
        })

    complete_count = sum(
        gate['status'] == 'COMPLETE' for gate in gate_reports)
    report = {
        'schema_version': 1,
        'schema_uri': REPORT_SCHEMA_URI,
        'status': (
            'READY'
            if complete_count == len(gate_reports)
            else 'NOT_READY'
        ),
        'product_version': product_version,
        'target_version': contract['target_version'],
        'minimum_release_candidate_version': minimum_version,
        'summary': {
            'total': len(gate_reports),
            'complete': complete_count,
            'incomplete': len(gate_reports) - complete_count,
        },
        'release': {
            'expected_tag': expected_tag,
            'minimum_version_met': minimum_version_met,
            'tag_present': tag_present,
        },
        'external_first_map': external_report,
        'gates': gate_reports,
    }
    _validate_schema(report, report_schema_path)
    return report


def render_markdown(report: dict[str, Any]) -> str:
    """Render a reviewer-facing readiness summary."""
    lines = [
        '# lidarslam_ros2 v1.0 readiness',
        '',
        f"- Status: **{report['status']}**",
        f"- Product VERSION: `{report['product_version']}`",
        (
            '- Complete gates: '
            f"**{report['summary']['complete']} / "
            f"{report['summary']['total']}**"
        ),
        '',
        '| Gate | Status | Detail |',
        '| --- | --- | --- |',
    ]
    for gate in report['gates']:
        lines.append(
            f"| `{gate['id']}` | **{gate['status']}** | "
            f"{gate['detail']} |"
        )
    lines.extend(['', '## Remaining blockers', ''])
    incomplete = [
        gate for gate in report['gates']
        if gate['status'] == 'INCOMPLETE'
    ]
    if not incomplete:
        lines.append('- None.')
    else:
        for gate in incomplete:
            lines.append(f"### {gate['title']} (`{gate['id']}`)")
            lines.append('')
            lines.extend(f'- {blocker}' for blocker in gate['blockers'])
            lines.append('')
    return '\n'.join(lines).rstrip() + '\n'


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract', type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument(
        '--contract-schema',
        type=Path,
        default=DEFAULT_CONTRACT_SCHEMA,
    )
    parser.add_argument(
        '--report-schema',
        type=Path,
        default=DEFAULT_REPORT_SCHEMA,
    )
    parser.add_argument('--ledger', type=Path, default=DEFAULT_LEDGER)
    parser.add_argument(
        '--ledger-schema',
        type=Path,
        default=DEFAULT_LEDGER_SCHEMA,
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print JSON instead of Markdown.',
    )
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--output-markdown', type=Path)
    parser.add_argument(
        '--require-complete',
        action='store_true',
        help='Exit 1 unless every v1.0 readiness gate is complete.',
    )
    return parser.parse_args(argv)


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        report = evaluate_readiness(
            contract_path=args.contract,
            contract_schema_path=args.contract_schema,
            report_schema_path=args.report_schema,
            ledger_path=args.ledger,
            ledger_schema_path=args.ledger_schema,
        )
    except ReadinessError as exc:
        print(f'v1 readiness audit invalid: {exc}', file=sys.stderr)
        return 2

    json_text = json.dumps(report, indent=2, sort_keys=True) + '\n'
    markdown_text = render_markdown(report)
    if args.output_json:
        _write(args.output_json, json_text)
    if args.output_markdown:
        _write(args.output_markdown, markdown_text)
    print(json_text if args.json else markdown_text, end='')
    if args.require_complete and report['status'] != 'READY':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
