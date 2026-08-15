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

"""Tests for the live package-manager release audit."""

from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_package_manager_release_readiness.py'
SPEC = importlib.util.spec_from_file_location(
    'package_manager_release_readiness',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
READINESS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(READINESS)


def _successful_run() -> dict:
    return {
        'id': 123,
        'html_url': 'https://github.com/example/actions/runs/123',
        'display_title': READINESS.expected_run_name('0.9.0'),
        'event': 'workflow_dispatch',
        'status': 'completed',
        'conclusion': 'success',
        'workflow_path': (
            '.github/workflows/package-manager-install-upgrade.yml'
        ),
        'head_sha': 'a' * 40,
        'jobs': [
            {
                'name': 'humble clean-install',
                'status': 'completed',
                'conclusion': 'success',
            },
            {
                'name': 'jazzy clean-install',
                'status': 'completed',
                'conclusion': 'success',
            },
        ],
    }


def test_exact_successful_main_channel_matrix_is_ready():
    report = READINESS.evaluate_readiness(
        version='0.9.0',
        snapshot={
            'inspected': True,
            'errors': [],
            'runs': [_successful_run()],
        },
    )

    assert report['status'] == 'READY'
    assert report['selected_run']['id'] == 123
    assert report['actions'] == []
    assert all(check['status'] == 'PASS' for check in report['checks'])


def test_no_matching_run_is_not_run():
    report = READINESS.evaluate_readiness(
        version='0.9.0',
        snapshot={'inspected': True, 'errors': [], 'runs': []},
    )

    assert report['status'] == 'NOT_RUN'
    assert report['selected_run'] is None
    assert report['actions']


def test_api_failure_is_blocked_not_not_run():
    report = READINESS.evaluate_readiness(
        version='0.9.0',
        snapshot={
            'inspected': False,
            'errors': ['rate limited'],
            'runs': [],
        },
    )

    assert report['status'] == 'BLOCKED'
    assert report['remote']['inspected'] is False


def test_github_token_is_scoped_to_api_requests(monkeypatch):
    """The optional token is sent only to bounded GitHub API requests."""
    requests = []

    class FakeHeaders:
        def get(self, _name):
            return None

    class FakeResponse:
        status = 200
        headers = FakeHeaders()

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self, _limit):
            return b'{}'

    def fake_urlopen(request, timeout):
        assert timeout == 20
        requests.append(request)
        return FakeResponse()

    monkeypatch.setenv('GITHUB_TOKEN', 'read-only-test-token')
    monkeypatch.setattr(READINESS.urllib.request, 'urlopen', fake_urlopen)

    READINESS._request_json('https://api.github.com/repos/owner/repo')
    READINESS._request_json(
        'https://raw.githubusercontent.com/owner/repo/main/file')

    assert requests[0].get_header('Authorization') == (
        'Bearer read-only-test-token'
    )
    assert requests[1].get_header('Authorization') is None


def test_both_named_distros_are_required():
    run = _successful_run()
    run['jobs'][1]['conclusion'] = 'failure'

    report = READINESS.evaluate_readiness(
        version='0.9.0',
        snapshot={'inspected': True, 'errors': [], 'runs': [run]},
    )

    assert report['status'] == 'NOT_RUN'
    failed = {
        check['id']
        for check in report['checks']
        if check['status'] == 'FAIL'
    }
    assert failed == {'jazzy-main-clean-install'}


def test_workflow_run_name_exposes_exact_audit_identity():
    workflow = (
        ROOT / '.github' / 'workflows'
        / 'package-manager-install-upgrade.yml'
    ).read_text(encoding='utf-8')

    assert (
        'run-name: package-manager / ${{ inputs.source_ref }} / '
        '${{ inputs.target_version }} / ${{ inputs.target_channel }} / '
        '${{ inputs.mode }}'
    ) in workflow
