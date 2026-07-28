"""Regression tests for the machine-readable first-map documentation gate."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
VALIDATOR = REPO_ROOT / 'scripts' / 'validate_first_map_docs.py'
CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'first-map-v1.json'
VALIDATION_FORM = (
    REPO_ROOT / '.github' / 'ISSUE_TEMPLATE' / 'first-map-validation.yml'
)
VALIDATION_LEDGER = (
    REPO_ROOT / 'docs' / 'evidence' / 'independent-first-map-validations.json'
)
VALIDATION_LEDGER_DOC = (
    REPO_ROOT / 'docs' / 'evidence' / 'independent-first-map-validations.md'
)


def _run(contract: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(VALIDATOR), '--contract', str(contract)],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def test_tracked_first_map_contract_passes():
    completed = _run(CONTRACT)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert 'PASS: first-map documentation contract' in completed.stdout


def test_contract_rejects_a_fourth_beginner_entrypoint(tmp_path):
    value = json.loads(CONTRACT.read_text(encoding='utf-8'))
    value['official_entrypoints'].append(
        {
            'id': 'unsupported-fourth-path',
            'command': 'ros2 launch research experiment.launch.py',
            'documentation': ['README.md'],
            'probes': [],
        }
    )
    invalid_contract = tmp_path / 'invalid-first-map.json'
    invalid_contract.write_text(json.dumps(value), encoding='utf-8')

    completed = _run(invalid_contract)
    assert completed.returncode == 1
    assert 'must contain exactly docker-demo, own-bag, and source-demo' in (
        completed.stdout
    )


def test_malformed_contract_is_reported_without_a_traceback(tmp_path):
    value = json.loads(CONTRACT.read_text(encoding='utf-8'))
    value['supported_source_platforms'] = [None]
    value['successful_run_artifacts'] = [{'unexpected': 'object'}]
    malformed_contract = tmp_path / 'malformed-first-map.json'
    malformed_contract.write_text(json.dumps(value), encoding='utf-8')

    completed = _run(malformed_contract)
    assert completed.returncode == 1
    assert 'supported_source_platforms item 0 must be an object' in completed.stdout
    assert 'successful_run_artifacts must be' in completed.stdout
    assert 'Traceback' not in completed.stderr


def test_ci_and_release_bundle_enforce_the_contract():
    main_workflow = (
        REPO_ROOT / '.github' / 'workflows' / 'main.yml'
    ).read_text(encoding='utf-8')
    release_workflow = (
        REPO_ROOT / '.github' / 'workflows' / 'release.yml'
    ).read_text(encoding='utf-8')

    for workflow in (main_workflow, release_workflow):
        assert 'python3 scripts/validate_first_map_docs.py' in workflow
        assert (
            'graph_based_slam/test/test_first_map_docs_contract.py'
            in workflow
        )
        assert (
            'graph_based_slam/test/test_first_map_validation_collector.py'
            in workflow
        )
    assert 'cp docs/contracts/*.json release_bundle/docs/contracts/' in (
        release_workflow
    )
    assert 'docs/evidence/independent-first-map-validations.md' in (
        release_workflow
    )
    assert 'docs/evidence/independent-first-map-validations.json' in (
        release_workflow
    )
    assert (
        'configs/first_map_validation_answers.example.json '
        'release_bundle/configs/'
        in release_workflow
    )
    assert (
        'scripts/collect_first_map_validation.py release_bundle/scripts/'
        in release_workflow
    )


def test_external_validation_gate_starts_at_zero_with_structured_intake():
    ledger = json.loads(VALIDATION_LEDGER.read_text(encoding='utf-8'))
    ledger_doc = VALIDATION_LEDGER_DOC.read_text(encoding='utf-8')
    issue_form = VALIDATION_FORM.read_text(encoding='utf-8')

    assert ledger['schema_version'] == 1
    assert ledger['gate_id'] == 'v1-independent-first-map'
    assert ledger['target_accepted_validations'] == 3
    assert ledger['accepted_count'] == len(ledger['validations']) == 0
    assert 'Status: **0 / 3 accepted validations**.' in ledger_doc
    assert '| _No accepted validations yet_ |' in ledger_doc
    assert 'Maintainer-operated Docker, source, CI and real-data runs do not count' in (
        ledger_doc
    )

    for field_id in (
        'independence',
        'entrypoint',
        'revision',
        'starting_document',
        'environment',
        'commands',
        'first_attempt_result',
        'elapsed_time',
        'evidence',
        'onboarding_findings',
        'publication',
    ):
        assert f'id: {field_id}' in issue_form


def test_external_validation_kit_is_versioned_and_privacy_bounded():
    contract = json.loads(CONTRACT.read_text(encoding='utf-8'))
    kit = contract['external_validation_kit']

    assert (REPO_ROOT / kit['answers_template']).is_file()
    assert (REPO_ROOT / kit['collector']).is_file()
    assert (REPO_ROOT / kit['report_schema']).is_file()
    assert kit['report_files'] == [
        'independent_first_map_validation.json',
        'independent_first_map_validation.md',
    ]
    assert kit['privacy_boundary'] == [
        'no pointcloud geometry',
        'no raw logs',
        'no absolute local paths',
    ]
    assert '--require-eligible' in kit['command']
