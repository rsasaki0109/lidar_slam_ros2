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

"""Tests for the read-only ndt_omp_ros2 initial-release preflight."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_ndt_omp_release_readiness.py'
SPEC = importlib.util.spec_from_file_location('ndt_release_readiness', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
PREFLIGHT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREFLIGHT)


def _local(ready: bool = True):
    return {
        'ready': ready,
        'gitlink_commit': PREFLIGHT.EXPECTED_COMMIT,
        'head_commit': PREFLIGHT.EXPECTED_COMMIT,
        'package_version': PREFLIGHT.EXPECTED_VERSION,
        'checks': [{
            'id': 'fixture',
            'status': 'PASS' if ready else 'FAIL',
            'detail': 'fixture local candidate state',
        }],
    }


def _pull_request(
    distro: str,
    *,
    response_pending: bool = False,
    state: str = 'open',
    merged: bool = False,
):
    number = PREFLIGHT.ROSDISTRO_PULL_REQUESTS[distro]
    review_url = (
        f'https://github.com/ros/rosdistro/pull/{number}'
        '#pullrequestreview-1'
    ) if response_pending else None
    return {
        'number': number,
        'url': f'https://github.com/ros/rosdistro/pull/{number}',
        'state': state,
        'merged': merged,
        'mergeable': True if state == 'open' else None,
        'head_sha': 'a' * 40,
        'updated_at': '2026-08-12T00:00:00Z',
        'latest_actionable_review': {
            'url': review_url,
            'author': 'reviewer' if response_pending else None,
            'created_at': (
                '2026-08-04T00:00:00Z' if response_pending else None
            ),
        },
        'response_pending': response_pending,
    }


def _remote(
    *,
    tag: bool,
    release_repo: bool,
    humble: bool,
    jazzy: bool,
    pending: tuple[str, ...] = (),
    closed: tuple[str, ...] = (),
):
    return {
        'errors': [],
        'origin_branch_commit': PREFLIGHT.EXPECTED_COMMIT,
        'source_tag_present': tag,
        'release_repository_present': release_repo,
        'rosdistro': {'humble': humble, 'jazzy': jazzy},
        'pull_requests': {
            distro: _pull_request(
                distro,
                response_pending=distro in pending,
                state='closed' if distro in closed else 'open',
            )
            for distro in PREFLIGHT.DISTROS
        },
    }


def test_tracked_candidate_is_locally_ready_and_schema_valid():
    report = PREFLIGHT.evaluate_readiness(offline=True)

    assert report['status'] == 'LOCAL_READY'
    assert report['local']['ready'] is True
    assert report['local']['gitlink_commit'] == PREFLIGHT.EXPECTED_COMMIT
    assert report['local']['head_commit'] == PREFLIGHT.EXPECTED_COMMIT
    assert report['local']['package_version'] == '0.1.0'
    assert report['schema_version'] == 2
    assert all(
        item['status'] == 'PASS' for item in report['local']['checks'])


def test_initially_absent_remote_artifacts_are_ready_to_tag():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=False,
            release_repo=False,
            humble=False,
            jazzy=False,
        ),
    )

    assert report['status'] == 'READY_TO_TAG'


def test_partial_publication_is_in_progress():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=True,
            jazzy=False,
        ),
    )

    assert report['status'] == 'IN_PROGRESS'
    assert any(
        'PR #52950 (jazzy)' in action
        and 'do not recreate the source tag or rerun Bloom' in action
        for action in report['actions']
    )
    assert not any(
        action.startswith('Bloom-release') for action in report['actions'])


def test_generated_prs_are_waited_on_without_repeating_bloom():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
        ),
    )

    assert report['status'] == 'IN_PROGRESS'
    assert len(report['actions']) == 2
    assert 'PR #52949 (humble)' in report['actions'][0]
    assert 'PR #52950 (jazzy)' in report['actions'][1]
    assert all('rerun Bloom' in action for action in report['actions'])


def test_unanswered_human_review_requires_response_instead_of_waiting():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
            pending=('humble', 'jazzy'),
        ),
    )

    assert report['status'] == 'REVIEW_REQUIRED'
    assert len(report['actions']) == 2
    assert all('unanswered human review' in item for item in report['actions'])
    assert all('upstream lineage' in item for item in report['actions'])
    assert all('collision-free convergence plan' in item
               for item in report['actions'])
    assert not any('Wait for' in item for item in report['actions'])


def test_closed_unmerged_generated_pr_blocks_replacement_release_state():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
            closed=('jazzy',),
        ),
    )

    assert report['status'] == 'BLOCKED'
    assert any(
        'PR #52950 (jazzy) closed without merge' in item
        for item in report['actions']
    )


def test_complete_publication_is_released():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=True,
            jazzy=True,
        ),
    )

    assert report['status'] == 'RELEASED'


def test_remote_error_and_branch_drift_fail_closed():
    failed_query = _remote(
        tag=False,
        release_repo=False,
        humble=False,
        jazzy=False,
    )
    failed_query['errors'] = ['GitHub returned HTTP 503']
    drifted = _remote(
        tag=False,
        release_repo=False,
        humble=False,
        jazzy=False,
    )
    drifted['origin_branch_commit'] = '0' * 40

    assert PREFLIGHT.evaluate_readiness(
        local=_local(), remote=failed_query)['status'] == 'BLOCKED'
    assert PREFLIGHT.evaluate_readiness(
        local=_local(), remote=drifted)['status'] == 'BLOCKED'


def test_explicit_404_is_absent_without_hiding_other_remote_state(monkeypatch):
    def fake_request(url):
        if url.endswith('/git/ref/heads/humble'):
            return 200, json.dumps({
                'object': {'sha': PREFLIGHT.EXPECTED_COMMIT},
            })
        if '/git/ref/tags/' in url or url.endswith(
                '/ndt_omp_ros2-release'):
            return 404, ''
        return 200, 'repositories:\n  another_package:\n'

    monkeypatch.setattr(PREFLIGHT, '_request_text', fake_request)

    remote = PREFLIGHT.inspect_remote()
    report = PREFLIGHT.evaluate_readiness(local=_local(), remote=remote)

    assert remote['errors'] == []
    assert remote['source_tag_present'] is False
    assert remote['release_repository_present'] is False
    assert remote['rosdistro'] == {'humble': False, 'jazzy': False}
    assert all(
        item['state'] is None
        for item in remote['pull_requests'].values()
    )
    assert report['status'] == 'READY_TO_TAG'


def test_pull_request_inspection_detects_question_and_author_response(
        monkeypatch):
    author_replied = False

    def fake_request_json(url):
        if url.endswith('/pulls/52950'):
            return {
                'user': {'login': 'rsasaki0109', 'type': 'User'},
                'state': 'open',
                'merged': False,
                'mergeable': True,
                'head': {'sha': 'b' * 40},
                'updated_at': '2026-08-07T00:00:00Z',
                'html_url': 'https://github.com/ros/rosdistro/pull/52950',
            }
        if '/reviews?' in url:
            return [{
                'user': {'login': 'reviewer', 'type': 'User'},
                'body': 'How does this relate to ndt_omp?',
                'state': 'COMMENTED',
                'submitted_at': '2026-08-04T00:00:00Z',
                'html_url': (
                    'https://github.com/ros/rosdistro/pull/52950'
                    '#pullrequestreview-1'
                ),
            }]
        if '/issues/' in url and author_replied:
            return [{
                'user': {'login': 'rsasaki0109', 'type': 'User'},
                'body': 'Thanks; here is the convergence plan.',
                'created_at': '2026-08-05T00:00:00Z',
                'html_url': (
                    'https://github.com/ros/rosdistro/pull/52950'
                    '#issuecomment-1'
                ),
            }]
        return []

    monkeypatch.setattr(PREFLIGHT, '_request_json', fake_request_json)

    pending = PREFLIGHT._inspect_pull_request('jazzy')
    assert pending['response_pending'] is True
    assert pending['latest_actionable_review']['author'] == 'reviewer'

    author_replied = True
    answered = PREFLIGHT._inspect_pull_request('jazzy')
    assert answered['response_pending'] is False


def test_missing_candidate_path_fails_closed(tmp_path):
    report = PREFLIGHT.evaluate_readiness(
        repo_root=tmp_path,
        offline=True,
    )

    assert report['status'] == 'BLOCKED'
    assert report['local']['ready'] is False


def test_offline_strict_gate_refuses_ready_to_tag(tmp_path):
    output = tmp_path / 'report.json'
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--offline',
            '--require-ready-to-tag',
            '--json',
            '--output-json',
            str(output),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    assert json.loads(result.stdout)['status'] == 'LOCAL_READY'
    assert json.loads(output.read_text(encoding='utf-8'))['status'] == (
        'LOCAL_READY'
    )
