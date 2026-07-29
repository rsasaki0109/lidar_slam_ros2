"""Tests for the fail-closed v1.0 readiness audit."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_v1_readiness.py'
SPEC = importlib.util.spec_from_file_location('v1_readiness', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
READINESS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(READINESS)


def test_tracked_contract_reports_exact_open_product_gates():
    report = READINESS.evaluate_readiness()

    assert report['status'] == 'NOT_READY'
    assert report['product_version'] == '0.7.0'
    assert report['summary'] == {
        'total': 10,
        'complete': 6,
        'incomplete': 4,
    }
    incomplete = {
        gate['id']
        for gate in report['gates']
        if gate['status'] == 'INCOMPLETE'
    }
    assert incomplete == {
        'distribution',
        'reliability',
        'external-adoption',
        'release-publication',
    }
    assert report['external_first_map']['accepted_validations'] == 0
    assert report['release'] == {
        'expected_tag': 'v0.7.0',
        'minimum_version_met': False,
        'tag_present': False,
    }


def test_require_complete_exits_one_for_tracked_state():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--require-complete', '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    assert json.loads(result.stdout)['status'] == 'NOT_READY'
    assert result.stderr == ''


def test_every_complete_gate_can_produce_ready_report(tmp_path):
    contract = json.loads(
        READINESS.DEFAULT_CONTRACT.read_text(encoding='utf-8'))
    for gate in contract['gates']:
        gate['state'] = 'complete'
        gate['blockers'] = []
        for evidence in gate['evidence']:
            path = tmp_path / evidence
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
    (tmp_path / 'VERSION').write_text('0.9.0\n', encoding='utf-8')
    contract_path = tmp_path / 'v1-readiness.json'
    contract_path.write_text(
        json.dumps(contract),
        encoding='utf-8',
    )
    external = {
        'schema_version': 1,
        'status': 'READY',
        'required_validations': 3,
        'accepted_validations': 3,
        'remaining_validations': 0,
        'distinct_reporters': 3,
        'documentation_path_counts': {
            'docker-first-map': 1,
            'source-quickstart': 1,
            'own-bag': 1,
        },
        'validation_ids': ['a', 'b', 'c'],
    }

    report = READINESS.evaluate_readiness(
        repo_root=tmp_path,
        contract_path=contract_path,
        tags={'v0.9.0'},
        external_report=external,
    )

    assert report['status'] == 'READY'
    assert report['summary']['complete'] == 10
    assert all(gate['blockers'] == [] for gate in report['gates'])


def test_evidence_cannot_escape_repository(tmp_path):
    contract = json.loads(
        READINESS.DEFAULT_CONTRACT.read_text(encoding='utf-8'))
    contract['gates'][0]['evidence'] = ['../outside']
    contract_path = tmp_path / 'v1-readiness.json'
    contract_path.write_text(json.dumps(contract), encoding='utf-8')
    (tmp_path / 'VERSION').write_text('0.9.0\n', encoding='utf-8')

    with pytest.raises(
        READINESS.ReadinessError,
        match='evidence path escapes repository',
    ):
        READINESS.evaluate_readiness(
            repo_root=tmp_path,
            contract_path=contract_path,
            tags={'v0.9.0'},
            external_report={'status': 'READY'},
        )
