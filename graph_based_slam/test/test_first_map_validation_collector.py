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

"""Regression tests for privacy-bounded independent first-map evidence."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[2]
COLLECTOR = REPO_ROOT / 'scripts' / 'collect_first_map_validation.py'
FIRST_MAP_CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'first-map-v1.json'
REPORT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'independent-first-map-validation-v1.schema.json'
)


def _answers() -> dict:
    return {
        'schema_version': 1,
        'entrypoint_id': 'own-bag',
        'tested_identity': 'a' * 40,
        'starting_document': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/getting-started/'
        ),
        'environment': {
            'ros_distro': 'jazzy',
            'os': 'Ubuntu 24.04',
            'architecture': 'amd64',
            'cpu_label': 'redacted 8-core CPU',
            'ram_gib': 16,
            'docker_version': None,
        },
        'commands': [
            'lidarslam-map run /data/redacted-bag '
            '--output-dir output/redacted-map',
        ],
        'first_attempt': {
            'result': 'verified_map',
            'elapsed': '12 minutes',
            'findings': ['No blocking findings.'],
        },
        'attestations': {
            'independent_tester': True,
            'docs_only_start': True,
            'first_attempt_preserved': True,
            'commands_redacted': True,
        },
        'public_consent': True,
    }


def _write_answers(tmp_path: Path, value: dict | list) -> Path:
    path = tmp_path / 'answers.json'
    path.write_text(json.dumps(value), encoding='utf-8')
    return path


def _write_success_run(tmp_path: Path) -> Path:
    run_dir = tmp_path / 'private-site-map'
    contract = json.loads(FIRST_MAP_CONTRACT.read_text(encoding='utf-8'))
    for relative in contract['successful_run_artifacts']:
        path = run_dir / relative.rstrip('/')
        if relative.endswith('/'):
            path.mkdir(parents=True, exist_ok=True)
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f'fixture for {relative}\n', encoding='utf-8')

    manifest = {
        'schema_version': 2,
        'status': 'succeeded',
        'lifecycle': {
            'stage': 'complete',
            'runner_exit_code': 0,
        },
        'software': {
            'product_version': '0.6.0',
            'git_commit': 'a' * 40,
            'git_dirty': False,
            'ros_distro': 'jazzy',
        },
        'profile': {'id': 'rko_lio_graph_public_path'},
        'output': {
            'finalized': True,
            'diagnosis_status': 'success',
        },
    }
    (run_dir / 'run_manifest.json').write_text(
        json.dumps(manifest),
        encoding='utf-8',
    )
    (run_dir / 'verify_autoware_map.log').write_text(
        'PASS: 8 | WARN: 0 | FAIL: 0\n'
        'RESULT: PASS -- map is Autoware-compatible\n',
        encoding='utf-8',
    )
    (run_dir / 'autoware_map_diagnosis.json').write_text(
        json.dumps({'status': 'success'}),
        encoding='utf-8',
    )
    return run_dir


def _run(
    answers: Path,
    output_dir: Path,
    run_dir: Path | None = None,
    require_eligible: bool = False,
) -> subprocess.CompletedProcess[str]:
    command = [
        sys.executable,
        str(COLLECTOR),
        '--answers',
        str(answers),
        '--output-dir',
        str(output_dir),
    ]
    if run_dir is not None:
        command.extend(['--run-dir', str(run_dir)])
    if require_eligible:
        command.append('--require-eligible')
    return subprocess.run(
        command,
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def test_collector_writes_schema_valid_eligible_redacted_evidence(
    tmp_path: Path,
):
    answers = _write_answers(tmp_path, _answers())
    run_dir = _write_success_run(tmp_path)
    output_dir = tmp_path / 'submission'

    completed = _run(answers, output_dir, run_dir, require_eligible=True)

    assert completed.returncode == 0, completed.stdout + completed.stderr
    report = json.loads(
        (output_dir / 'independent_first_map_validation.json').read_text(
            encoding='utf-8'
        )
    )
    schema = json.loads(REPORT_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate(report, schema)
    assert report['acceptance_status'] == 'eligible'
    assert len(report['checks']) == 14
    assert all(check['passed'] for check in report['checks'])
    assert report['privacy'] == {
        'geometry_included': False,
        'absolute_local_paths_included': False,
        'raw_logs_included': False,
    }
    serialized = json.dumps(report)
    assert str(tmp_path) not in serialized
    assert 'private-site-map' not in serialized
    assert all(
        item['sha256'] is not None
        for item in report['run_evidence']['artifacts']
        if item['kind'] == 'file'
    )


def test_incomplete_attempt_still_writes_noneligible_report(tmp_path: Path):
    answers_value = _answers()
    answers_value['first_attempt']['result'] = 'workflow_incomplete'
    answers_value['first_attempt']['elapsed'] = 'not reached'
    answers_value['first_attempt']['findings'] = ['Workflow stopped early.']
    answers = _write_answers(tmp_path, answers_value)

    completed = _run(answers, tmp_path / 'submission')

    assert completed.returncode == 0
    report = json.loads(
        (
            tmp_path
            / 'submission'
            / 'independent_first_map_validation.json'
        ).read_text(encoding='utf-8')
    )
    assert report['acceptance_status'] == 'not_eligible'
    assert report['run_evidence']['run_directory_supplied'] is False
    assert not next(
        check for check in report['checks']
        if check['id'] == 'first_attempt_verified_map'
    )['passed']

    strict = _run(
        answers,
        tmp_path / 'strict-submission',
        require_eligible=True,
    )
    assert strict.returncode == 1


def test_source_revision_must_match_terminal_manifest(tmp_path: Path):
    answers_value = _answers()
    answers_value['tested_identity'] = 'b' * 40
    answers = _write_answers(tmp_path, answers_value)
    run_dir = _write_success_run(tmp_path)

    completed = _run(answers, tmp_path / 'submission', run_dir)

    assert completed.returncode == 0
    report = json.loads(
        (
            tmp_path
            / 'submission'
            / 'independent_first_map_validation.json'
        ).read_text(encoding='utf-8')
    )
    assert report['acceptance_status'] == 'not_eligible'
    identity_check = next(
        check for check in report['checks']
        if check['id'] == 'tested_identity_bound_to_entrypoint'
    )
    assert not identity_check['passed']
    assert 'manifest_git_commit=' + 'a' * 40 in identity_check['observed']


def test_malformed_answers_fail_without_partial_report(tmp_path: Path):
    answers = _write_answers(tmp_path, [])
    output_dir = tmp_path / 'submission'

    completed = _run(answers, output_dir)

    assert completed.returncode == 2
    assert 'answers root must be an object' in completed.stderr
    assert 'Traceback' not in completed.stderr
    assert not output_dir.exists()
