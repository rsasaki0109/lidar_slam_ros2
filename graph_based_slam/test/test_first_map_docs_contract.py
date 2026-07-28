"""Regression tests for the machine-readable first-map documentation gate."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
VALIDATOR = REPO_ROOT / 'scripts' / 'validate_first_map_docs.py'
CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'first-map-v1.json'


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
    assert 'cp docs/contracts/*.json release_bundle/docs/contracts/' in (
        release_workflow
    )
