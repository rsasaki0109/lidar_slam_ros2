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
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
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

"""Tests for the fail-closed issue-triage application packet."""

from __future__ import annotations

import copy
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'prepare_issue_triage_application.py'
PROPOSAL_PATH = (
    ROOT
    / 'docs'
    / 'evidence'
    / 'growth'
    / 'open-issue-triage-proposal-2026-08-11.json'
)
PROPOSAL_SCHEMA_PATH = (
    ROOT / 'docs' / 'schemas' / 'issue-triage-proposal-v1.schema.json'
)
PACKET_SCHEMA_PATH = (
    ROOT
    / 'docs'
    / 'schemas'
    / 'issue-triage-application-packet-v1.schema.json'
)
SPEC = importlib.util.spec_from_file_location(
    'prepare_issue_triage_application',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
APPLICATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(APPLICATION)


def _load(path: Path) -> dict:
    """Load one tracked JSON object."""
    payload = json.loads(path.read_text(encoding='utf-8'))
    assert isinstance(payload, dict)
    return payload


def _proposal() -> dict:
    """Return the tracked proposal."""
    return _load(PROPOSAL_PATH)


def _proposal_schema() -> dict:
    """Return the tracked proposal schema."""
    return _load(PROPOSAL_SCHEMA_PATH)


def _packet_schema() -> dict:
    """Return the tracked packet schema."""
    return _load(PACKET_SCHEMA_PATH)


def _packet(*, issue_number=None, live_status='NOT_RUN') -> dict:
    """Build one packet from tracked sources."""
    return APPLICATION.build_packet(
        _proposal(),
        _proposal_schema(),
        _packet_schema(),
        issue_number=issue_number,
        live_status=live_status,
    )


def _action(packet: dict, issue_number: int) -> dict:
    """Return one issue action from a packet."""
    return next(
        item for item in packet['actions']
        if item['issue_number'] == issue_number
    )


def _live_snapshot(proposal: dict) -> tuple[list[dict], list[str]]:
    """Build the exact API-shaped fields retained by the proposal."""
    issues = [{
        'issue_number': item['issue_number'],
        'title': item['title'],
        'updated_at': item['observed']['updated_at'],
        'labels': list(item['observed']['labels']),
    } for item in proposal['issues']]
    return issues, list(proposal['label_catalog'])


def test_complete_packet_is_schema_valid_ordered_and_unauthorized():
    """All 29 rows become deterministic review actions without authority."""
    packet = _packet()

    APPLICATION.validate_packet(packet, _packet_schema())
    assert packet['status'] == 'PREPARED_NOT_AUTHORIZED'
    assert packet['selection'] == {
        'requested_issue_number': None,
        'proposal_issue_count': 29,
        'selected_action_count': 29,
    }
    assert packet['summary'] == {
        'close_action_count': 23,
        'reproduction_request_count': 4,
        'dependency_review_count': 9,
        'monitor_only_count': 1,
    }
    assert [item['issue_number'] for item in packet['actions'][:6]] == [
        69, 422, 64, 104, 106, 124,
    ]
    assert [item['issue_number'] for item in packet['actions'][6:]] == sorted(
        item['issue_number'] for item in packet['actions'][6:]
    )
    assert packet['authority'] == {
        'github_requests': 'NONE',
        'github_writes_authorized': False,
        'issue_comments_authorized': False,
        'issue_label_changes_authorized': False,
        'issue_state_changes_authorized': False,
        'local_files_written': False,
        'remote_mutations_performed': False,
    }


def test_issue_69_is_a_reviewed_keep_open_help_request():
    """The crash-safety issue is not accidentally closed."""
    action = _action(_packet(), 69)

    assert action['content_status'] == 'READY_FOR_MAINTAINER_REVIEW'
    assert action['proposed_changes'] == {
        'labels_to_add': ['help wanted'],
        'comment_required': True,
        'close_issue': False,
        'state_reason': None,
    }
    assert len(action['evidence']) == 6
    assert all(
        item['kind'] == 'repository_file' and len(item['sha256']) == 64
        for item in action['evidence']
    )
    response = action['public_response_draft']
    assert 'What we found:' in response
    assert 'Current status: **keeping this issue open**.' in response
    assert 'privacy-safe synthetic points' in response
    assert 'public Draft PR #427' in response
    assert 'passes Humble and Jazzy CI' in response
    assert 'No named release contains the fix yet' in response
    assert '`vg_size_for_map`' in response
    assert '`vg_size_for_input`' in response
    assert 'only a workaround' in response
    assert 'original 2 GB bag is not required' in response
    assert 'fix remains local' not in response
    assert 'lacks supported public CI' not in response


def test_issue_422_remains_monitor_only():
    """Independent validation is preserved without a support intervention."""
    action = _action(_packet(), 422)

    assert action['content_status'] == 'MONITOR_ONLY'
    assert action['proposed_changes'] == {
        'labels_to_add': [],
        'comment_required': False,
        'close_issue': False,
        'state_reason': None,
    }


def test_dependency_rows_are_explicitly_blocked_for_review():
    """Starter dependencies cannot be flattened into a blind close batch."""
    actions = [
        item for item in _packet()['actions']
        if item['content_status'] == 'DEPENDENCY_REVIEW_REQUIRED'
    ]

    assert [item['issue_number'] for item in actions] == [
        106, 98, 102, 105, 108, 111, 112, 115, 122,
    ]
    assert all(item['prerequisites']['dependency_ids'] for item in actions)


def test_single_issue_selection_is_bounded_and_unknown_issue_fails():
    """A maintainer can review one known issue but cannot invent a row."""
    packet = _packet(issue_number=104)

    assert packet['selection']['selected_action_count'] == 1
    assert [item['issue_number'] for item in packet['actions']] == [104]
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='is not in the proposal',
    ):
        _packet(issue_number=999)


def test_response_drafts_are_issue_specific_and_contain_no_mutation_command():
    """Each draft cites its rationale, next step, and immutable evidence."""
    packet = _packet()
    rendered = APPLICATION.render_packet(packet)

    for action in packet['actions']:
        response = action['public_response_draft']
        assert action['rationale'] in response
        assert action['response_summary'] in response
        assert 'Evidence reviewed:' in response
    assert 'review aid' in rendered
    assert 'does not authorize posting, labeling, or closing' in rendered
    assert 'gh issue' not in rendered
    assert 'PATCH ' not in rendered


def test_external_and_repository_evidence_stay_inside_bounded_sources():
    """Only tracked files and a clean github.com repository URL are accepted."""
    action = _action(_packet(), 118)

    assert action['evidence'][0] == {
        'kind': 'external_url',
        'url': 'https://github.com/rsasaki0109/lidar_localization_ros2',
    }
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='outside the GitHub boundary',
    ):
        APPLICATION._evidence_record('https://example.com/evidence')
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='unavailable',
    ):
        APPLICATION._evidence_record('docs/missing-evidence.md')


def test_packet_retains_no_raw_github_content_or_identity():
    """Live reads do not become a durable raw issue archive."""
    packet = _packet()

    assert packet['privacy'] == {
        'author_identities_retained': False,
        'comment_bodies_requested': False,
        'issue_bodies_retained': False,
        'raw_github_records_written': False,
    }
    serialized = json.dumps(packet, sort_keys=True)
    assert '"author"' not in serialized
    assert '"comments"' not in serialized


@pytest.mark.parametrize(
    ('mutate', 'message'),
    [
        (
            lambda packet: packet['actions'][1].update(order=1),
            'order must be consecutive',
        ),
        (
            lambda packet: packet['summary'].update(close_action_count=0),
            'summary is inconsistent',
        ),
        (
            lambda packet: (
                packet['actions'][0]['proposed_changes'].update(
                    close_issue=True
                ),
                packet['summary'].update(close_action_count=24),
            ),
            'close decision is inconsistent',
        ),
        (
            lambda packet: packet['actions'][0].update(
                public_response_draft='generic response'
            ),
            'response draft is not issue-specific',
        ),
        (
            lambda packet: packet['actions'][0]['evidence'][0].update(
                sha256='0' * 64
            ),
            'evidence has drifted',
        ),
        (
            lambda packet: packet['authority'].update(
                github_requests='GET_ONLY'
            ),
            'request mode is inconsistent',
        ),
    ],
)
def test_semantic_validator_rejects_packet_drift(mutate, message):
    """Schema-shaped but internally inconsistent packets fail closed."""
    packet = copy.deepcopy(_packet())
    mutate(packet)

    with pytest.raises(APPLICATION.ApplicationPacketError, match=message):
        APPLICATION.validate_packet(packet, _packet_schema())


def test_source_proposal_and_generator_hashes_are_rechecked():
    """A packet cannot detach itself from either implementation input."""
    packet = copy.deepcopy(_packet())
    packet['source']['proposal_canonical_sha256'] = '0' * 64
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='source proposal hash has drifted',
    ):
        APPLICATION.validate_packet(packet, _packet_schema())

    packet = copy.deepcopy(_packet())
    packet['source']['generator_sha256'] = '0' * 64
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='source generator hash has drifted',
    ):
        APPLICATION.validate_packet(packet, _packet_schema())


def test_schema_refuses_any_claimed_write_authority():
    """No Boolean edit can turn this review artifact into write approval."""
    packet = copy.deepcopy(_packet())
    packet['authority']['github_writes_authorized'] = True

    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='packet schema failed',
    ):
        APPLICATION.validate_packet(packet, _packet_schema())


def test_live_main_uses_get_only_snapshot_and_emits_pass(monkeypatch, capsys):
    """An exact live snapshot changes only the request audit mode."""
    proposal = _proposal()
    live_issues, live_labels = _live_snapshot(proposal)
    calls = []

    def fake_fetch(repository):
        calls.append(repository)
        return live_issues, live_labels

    monkeypatch.setattr(
        APPLICATION.PROPOSAL_CHECKER,
        'fetch_live_snapshot',
        fake_fetch,
    )

    assert APPLICATION.main(['--live', '--issue', '69', '--json']) == 0
    packet = json.loads(capsys.readouterr().out)
    assert calls == ['rsasaki0109/lidar_slam_ros2']
    assert packet['live_check'] == {'performed': True, 'status': 'PASS'}
    assert packet['authority']['github_requests'] == 'GET_ONLY'
    assert packet['authority']['remote_mutations_performed'] is False


def test_live_drift_returns_failure_without_a_false_packet(monkeypatch, capsys):
    """Any open-issue drift prevents packet output."""
    proposal = _proposal()
    live_issues, live_labels = _live_snapshot(proposal)
    live_issues.pop()
    monkeypatch.setattr(
        APPLICATION.PROPOSAL_CHECKER,
        'fetch_live_snapshot',
        lambda repository: (live_issues, live_labels),
    )

    assert APPLICATION.main(['--live', '--json']) == 1
    captured = capsys.readouterr()
    assert captured.out == ''
    assert 'open-issue set drifted' in captured.err


def test_cli_prints_json_and_human_review_cards_without_output_option():
    """The CLI is stdout-only and has no file-writing mode."""
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--issue', '69', '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['actions'][0]['issue_number'] == 69

    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--issue', '422'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert 'MONITOR_ONLY' in result.stdout
    assert 'mutation command' in result.stdout

    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--output', 'packet.json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert 'unrecognized arguments: --output' in result.stderr


def test_generator_contains_no_remote_write_adapter():
    """The generator delegates only the existing GET-only live snapshot."""
    source = SCRIPT.read_text(encoding='utf-8')

    assert 'fetch_live_snapshot' in source
    assert "'POST'" not in source
    assert "'PATCH'" not in source
    assert "'PUT'" not in source
    assert "'DELETE'" not in source
    assert 'subprocess' not in source


def test_application_packet_is_registered_and_bundled():
    """Ctest and release evidence include the exact review implementation."""
    cmake = (ROOT / 'lidarslam' / 'CMakeLists.txt').read_text(
        encoding='utf-8'
    )
    builder = (ROOT / 'scripts' / 'build_release_bundle.py').read_text(
        encoding='utf-8'
    )

    assert 'test_prepare_issue_triage_application' in cmake
    assert "'scripts/prepare_issue_triage_application.py'" in builder
