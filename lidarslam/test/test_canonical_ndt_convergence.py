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
#    in the documentation and/or other materials provided with the
#    distribution.
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

"""Regression tests for the canonical ndt_omp convergence contract."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import jsonschema
import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_canonical_ndt_convergence.py'
CONTRACT = ROOT / 'docs' / 'contracts' / 'canonical-ndt-convergence-v1.json'
REPORT_SCHEMA = (
    ROOT
    / 'docs'
    / 'schemas'
    / 'canonical-ndt-convergence-readiness-v1.schema.json'
)
SPEC = importlib.util.spec_from_file_location(
    'canonical_ndt_convergence', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


def _contract() -> dict:
    return json.loads(CONTRACT.read_text(encoding='utf-8'))


def _write_contract(tmp_path: Path, payload: dict) -> Path:
    path = tmp_path / 'contract.json'
    path.write_text(json.dumps(payload), encoding='utf-8')
    return path


def test_checked_in_bundle_is_artifact_ready_and_schema_valid():
    report = CHECKER.evaluate()

    assert report['status'] == 'ARTIFACTS_READY'
    assert report['mode'] == 'artifacts'
    assert report['authority'] == {
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }
    assert all(
        item['status'] == 'PASS'
        for item in report['checks']
        if not item['id'].startswith('upstream-checkout-')
        and item['id'] != 'upstream-patch-applies'
    )
    assert sum(
        item['status'] == 'NOT_CHECKED' for item in report['checks']) == 3
    schema = json.loads(REPORT_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator(schema).validate(report)


def test_parent_transition_covers_both_direct_consumers_exactly():
    report = CHECKER.evaluate()
    checks = {item['id']: item for item in report['checks']}

    assert checks['consumer-graph_based_slam/CMakeLists.txt'] == {
        'id': 'consumer-graph_based_slam/CMakeLists.txt',
        'status': 'PASS',
        'detail': (
            'current=7, deleted=7, canonical-added=7, '
            'virtual-fork-remaining=0'
        ),
    }
    assert checks['consumer-graph_based_slam/package.xml']['status'] == 'PASS'
    assert checks['consumer-scanmatcher/CMakeLists.txt']['status'] == 'PASS'
    assert checks['consumer-scanmatcher/package.xml']['status'] == 'PASS'
    assert checks['parent-api-spelling-transition']['status'] == 'PASS'


def test_exact_clean_upstream_checkout_promotes_local_review_status(
    monkeypatch,
    tmp_path: Path,
):
    checkout = tmp_path / 'upstream'
    checkout.mkdir()
    real_apply = CHECKER._apply_check

    def fake_git_text(repo, *arguments):
        assert repo == checkout
        if arguments == ('rev-parse', 'HEAD'):
            return _contract()['upstream']['base_commit']
        assert arguments == (
            'status', '--porcelain', '--untracked-files=all')
        return ''

    def fake_apply(repo, patch):
        if repo == checkout:
            return True, 'patch applies without modifying the checkout'
        return real_apply(repo, patch)

    monkeypatch.setattr(CHECKER, '_git_text', fake_git_text)
    monkeypatch.setattr(CHECKER, '_apply_check', fake_apply)

    report = CHECKER.evaluate(upstream_checkout=checkout)

    assert report['status'] == 'READY_FOR_UPSTREAM_REVIEW'
    assert report['upstream_checkout'] == {
        'inspected': True,
        'commit': _contract()['upstream']['base_commit'],
        'clean': True,
        'patch_applies': True,
    }
    assert str(checkout) not in json.dumps(report)


@pytest.mark.parametrize('failure', ('commit', 'dirty', 'apply'))
def test_upstream_checkout_failures_block_review(
    monkeypatch,
    tmp_path: Path,
    failure: str,
):
    checkout = tmp_path / 'upstream'
    checkout.mkdir()
    expected = _contract()['upstream']['base_commit']
    real_apply = CHECKER._apply_check

    def fake_git_text(repo, *arguments):
        assert repo == checkout
        if arguments == ('rev-parse', 'HEAD'):
            return '0' * 40 if failure == 'commit' else expected
        return ' M changed.cpp' if failure == 'dirty' else ''

    def fake_apply(repo, patch):
        if repo == checkout:
            return (
                (False, 'injected apply failure') if failure == 'apply'
                else (True, 'patch applies without modifying the checkout')
            )
        return real_apply(repo, patch)

    monkeypatch.setattr(CHECKER, '_git_text', fake_git_text)
    monkeypatch.setattr(CHECKER, '_apply_check', fake_apply)

    report = CHECKER.evaluate(upstream_checkout=checkout)

    assert report['status'] == 'BLOCKED'
    assert any(item['status'] == 'FAIL' for item in report['checks'])


def test_artifact_hash_drift_fails_closed(tmp_path: Path):
    payload = _contract()
    payload['upstream']['patch']['sha256'] = '0' * 64

    report = CHECKER.evaluate(contract_path=_write_contract(tmp_path, payload))

    assert report['status'] == 'BLOCKED'
    checks = {item['id']: item for item in report['checks']}
    assert checks['upstream-patch-sha256']['status'] == 'FAIL'


def test_consumer_inventory_drift_fails_closed(tmp_path: Path):
    payload = _contract()
    payload['parent_transition']['consumer_replacements'][0][
        'before_count'] = 6

    report = CHECKER.evaluate(contract_path=_write_contract(tmp_path, payload))

    assert report['status'] == 'BLOCKED'
    checks = {item['id']: item for item in report['checks']}
    assert checks['consumer-graph_based_slam/CMakeLists.txt']['status'] == (
        'FAIL')


def test_remote_write_authority_is_rejected_by_schema(tmp_path: Path):
    payload = _contract()
    payload['authority']['github_writes_authorized'] = True

    with pytest.raises(
        CHECKER.ConvergenceError,
        match='authority.github_writes_authorized',
    ):
        CHECKER.evaluate(contract_path=_write_contract(tmp_path, payload))


def test_cli_json_is_path_private_and_strict_mode_requires_checkout():
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--json',
            '--require-ready-for-upstream-review',
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    report = json.loads(result.stdout)
    assert report['status'] == 'ARTIFACTS_READY'
    assert '/home/' not in result.stdout
    assert '/tmp/' not in result.stdout
    assert result.stderr == ''


def test_output_file_is_create_only(tmp_path: Path):
    output = tmp_path / 'readiness.json'
    first = CHECKER.main(['--json', '--output-json', str(output)])
    before = output.read_bytes()
    second = CHECKER.main(['--json', '--output-json', str(output)])

    assert first == 0
    assert second == 2
    assert output.read_bytes() == before
