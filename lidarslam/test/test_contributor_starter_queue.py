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
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Regression tests for the bounded local contributor starter queue."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import jsonschema

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'contributor_starter_queue.py'
QUEUE_PATH = (
    ROOT / 'docs' / 'contracts' / 'contributor-starter-queue-v1.json'
)
SCHEMA_PATH = (
    ROOT / 'docs' / 'schemas' / 'contributor-starter-queue-v1.schema.json'
)
SPEC = importlib.util.spec_from_file_location(
    'contributor_starter_queue', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


def _load(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding='utf-8'))
    assert isinstance(payload, dict)
    return payload


def _queue() -> dict:
    return copy.deepcopy(_load(QUEUE_PATH))


def _schema() -> dict:
    return copy.deepcopy(_load(SCHEMA_PATH))


def _task(payload: dict, task_id: str) -> dict:
    return next(item for item in payload['tasks'] if item['id'] == task_id)


def _copy_scoped_files(payload: dict, destination: Path) -> None:
    paths = {
        path
        for task in payload['tasks']
        for path in task['allowed_paths']
    }
    for path in paths:
        target = destination / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((ROOT / path).read_bytes())


def test_checked_in_queue_is_schema_valid_and_ready_local_only():
    """The checked-in queue keeps five actionable local-only tasks."""
    queue, report = CHECKER.evaluate()

    jsonschema.Draft7Validator(_schema()).validate(queue)
    assert report['status'] == 'QUEUE_READY_LOCAL_ONLY'
    assert report['task_count'] == 5
    assert report['ready_task_ids'] == list(CHECKER.EXPECTED_TASK_IDS)
    assert report['stale_tasks'] == []
    assert report['remote_duplicate_audit'] == {
        'checked_at': '2026-08-12T13:39:03+09:00',
        'open_pull_request_count': 1,
        'matching_task_pull_requests': 0,
        'recheck_before_publication': True,
    }
    assert report['authority'] == {
        'github_writes_authorized': False,
        'issues_published': False,
        'remote_mutations_performed': False,
    }


def test_all_tasks_are_bounded_to_thirty_minutes_and_exact_paths():
    """Every task stays within the advertised beginner-sized boundary."""
    payload = _queue()

    assert [task['id'] for task in payload['tasks']] == [
        'starter-C1',
        'starter-C2',
        'starter-C3',
        'starter-C4',
        'starter-C5',
    ]
    assert max(task['estimate_minutes'] for task in payload['tasks']) == 30
    assert all(task['estimate_minutes'] <= 30 for task in payload['tasks'])
    assert all(task['allowed_paths'] == sorted(task['allowed_paths'])
               for task in payload['tasks'])
    assert all(not task['private_data_required'] for task in payload['tasks'])
    assert all(not task['hardware_required'] for task in payload['tasks'])


def test_rendered_task_is_copy_ready_but_keeps_coordination_gate():
    """Rendered issue text is complete and clearly remains unpublished."""
    payload = _queue()

    for task in payload['tasks']:
        body = CHECKER.render_task_markdown(task)
        assert body.startswith(f"# {task['title']}\n")
        assert 'prepared locally but is not a published GitHub issue' in body
        assert '## Allowed files' in body
        assert '## Acceptance' in body
        assert '## Non-goals' in body
        assert '## Focused checks' in body
        assert 'No private data or hardware is required.' in body
        for path in task['allowed_paths']:
            assert f'`{path}`' in body
        for label in task['labels']:
            assert f'`{label}`' in body


def test_arbitrary_command_cannot_replace_an_allowlisted_profile():
    """JSON data cannot inject a shell command into the verifier."""
    payload = _queue()
    task = _task(payload, 'starter-C1')
    task['focused_checks'][0]['argv'] = ['bash', '-c', 'touch unexpected']

    with pytest.raises(
        CHECKER.QueueError,
        match='focused checks do not match profile',
    ):
        CHECKER.validate_queue(payload, _schema())


def test_remote_write_authority_is_rejected_by_schema():
    """The local contract cannot grant itself GitHub write authority."""
    payload = _queue()
    payload['authority']['github_writes_authorized'] = True

    with pytest.raises(
        CHECKER.QueueError,
        match='authority.github_writes_authorized',
    ):
        CHECKER.validate_queue(payload, _schema())


def test_open_pull_request_duplicate_fails_closed():
    """A matching open PR prevents the queue from reporting ready."""
    payload = _queue()
    payload['remote_duplicate_audit']['task_matches'][0][
        'matching_open_pull_requests'
    ] = [999]

    with pytest.raises(
        CHECKER.QueueError,
        match='open pull request duplicates require review',
    ):
        CHECKER.validate_queue(payload, _schema())


def test_duplicate_audit_must_cover_every_task_in_order():
    """The remote duplicate audit cannot omit or shuffle a task."""
    payload = _queue()
    matches = payload['remote_duplicate_audit']['task_matches']
    matches[0], matches[1] = matches[1], matches[0]

    with pytest.raises(
        CHECKER.QueueError,
        match='duplicate audit must cover the ordered task set',
    ):
        CHECKER.validate_queue(payload, _schema())


def test_missing_scoped_file_fails_closed(tmp_path: Path):
    """A missing file invalidates its starter scope."""
    payload = _queue()
    _copy_scoped_files(payload, tmp_path)
    (tmp_path / 'docs' / 'getting-started.md').unlink()

    with pytest.raises(
        CHECKER.QueueError,
        match='allowed path is not a regular file',
    ):
        CHECKER.validate_queue(payload, _schema(), tmp_path)


def test_probe_cannot_escape_task_path_scope():
    """Known-gap probes can inspect only files a task may modify."""
    payload = _queue()
    _task(payload, 'starter-C1')['gap_probes'][0]['path'] = 'README.md'

    with pytest.raises(
        CHECKER.QueueError,
        match='gap probe is outside its path scope',
    ):
        CHECKER.validate_queue(payload, _schema())


def test_docs_task_becomes_stale_when_planned_marker_appears(tmp_path: Path):
    """An already-present docs card takes its task out of the ready set."""
    payload = _queue()
    _copy_scoped_files(payload, tmp_path)
    getting_started = tmp_path / 'docs' / 'getting-started.md'
    getting_started.write_text(
        getting_started.read_text(encoding='utf-8')
        + '\n### Recover g2o dependency failures\n',
        encoding='utf-8',
    )

    report = CHECKER.validate_queue(payload, _schema(), tmp_path)

    assert report['status'] == 'QUEUE_STALE_LOCAL_ONLY'
    assert report['ready_task_ids'] == [
        'starter-C2',
        'starter-C3',
        'starter-C4',
        'starter-C5',
    ]
    assert report['stale_tasks'] == [{
        'id': 'starter-C1',
        'status': 'STALE',
        'reasons': ['the planned g2o recovery card marker already exists'],
    }]


def test_japanese_version_task_becomes_stale_when_marker_appears(
    tmp_path: Path,
):
    """A completed Japanese version card requires queue reassessment."""
    payload = _queue()
    _copy_scoped_files(payload, tmp_path)
    source = tmp_path / 'docs' / 'getting-started-ja.md'
    source.write_text(
        source.read_text(encoding='utf-8')
        + '\nlidarslam-map --version\n',
        encoding='utf-8',
    )

    report = CHECKER.validate_queue(payload, _schema(), tmp_path)

    assert report['status'] == 'QUEUE_STALE_LOCAL_ONLY'
    assert report['stale_tasks'][0]['id'] == 'starter-C5'
    assert report['stale_tasks'][0]['reasons'] == [
        'the planned Japanese version-identity marker already exists',
    ]


def test_japanese_recovery_card_keeps_empty_frame_action():
    """The completed Japanese card retains an actionable frame repair."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert 'header.frame_id' in source
    assert 'publisherの`header.frame_id`を修正してから再確認' in source
    assert 'viewerのframe名を推測して先に進めません' in source


def test_japanese_recovery_card_explains_pointcloud_topic_selection():
    """The Japanese card makes the topic placeholder copy-ready."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert 'ros2 topic list -t' in source
    assert '[sensor_msgs/msg/PointCloud2]' in source
    assert '`<POINTCLOUD_TOPIC>`をその名前に置き換えます' in source
    assert 'PointCloud2の行がない場合は' in source


def test_japanese_recovery_card_explains_tf_frame_substitution():
    """The Japanese card connects the TF placeholders to observed frames."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert 'check 2で出た空でないframe名を`<POINTCLOUD_FRAME>`に入れます' in source
    assert '<TF_TARGET_FRAME>`にはruntimeまたはviewerが基準にするtarget frame' in source
    assert '`livox_frame`なら、`<POINTCLOUD_FRAME>`だけを' in source
    assert 'viewerでframe名を推測したり' in source
    assert '同じ実際の' in source
    assert 'frame名で再確認します' in source


def test_japanese_recovery_card_explains_headless_preview_options():
    """The Japanese card makes local browser recovery copy-ready."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert 'ブラウザが自動で' in source
    assert 'ヘッドレス環境では' in source
    assert 'self-contained HTML' in source
    assert '`--no-open`を付けるとブラウザを起動せず' in source
    assert 'lidarslam-map view /path/to/output' in source
    assert '--viewer browser' in source
    assert '--no-open' in source
    assert '--preview-dir /path/to/preview' in source
    assert (
        '`HTML: /path/to/output/preview/mid360_robot_3d_map_preview.html`'
        in source
    )
    assert 'サニタイズ済みのsupport' in source


def test_japanese_recovery_card_explains_session_history_and_recovery():
    """The Japanese card makes retained session recovery copy-ready."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '保存したsessionを探して復旧する' in source
    assert 'lidarslam-map sessions' in source
    assert '--status action_required' in source
    assert '--viewer none' in source
    assert '--json' in source
    assert '`Details:`と`Next:`' in source
    assert '`Next:`の行に表示された実際のcommandをそのまま実行' in source
    assert 'map_verify' in source
    assert 'autoware_map_diagnosis.md' in source
    assert 'session履歴、catalog、診断、previewはlocal-only' in source
    assert 'preview\nHTMLをGitHub issueへuploadせず' in source


def test_japanese_recovery_card_explains_privacy_first_support_handoff():
    """The Japanese card makes support sharing reviewable and bounded."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '支援へ共有する前に確認する' in source
    assert 'lidarslam-map support /path/to/session_bundle --json' in source
    assert 'read-onlyで、ZIPを作成せず' in source
    assert 'README.txt' in source
    assert 'issue-body.md' in source
    assert 'support-report.json' in source
    assert 'map geometry、bag、raw log、parameter内容' in source
    assert 'credentialのようなcommand値は含まれません' in source
    assert '`--first-map`はreceipt-boundのPASSを再検証' in source
    assert 'review済みのfirst-map receiptだけ' in source
    assert '`--first-map --json`のhandoff JSONは' in source
    assert '公開添付には使いません' in source


def test_japanese_recovery_card_explains_independent_first_map_handoff():
    """The Japanese card keeps independent validation privacy-bounded."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '独立validator向けのissue form' in source
    assert 'canonical\nindependent-validation issue form' in source
    assert '自分で実行したrunの' in source
    assert 'private pathをredact' in source
    assert 'first-map receiptだけを添付します' in source
    assert 'map、bag、preview、\nraw log、trajectory、parameter' in source
    assert (
        '`--first-map --json`のhandoff JSONとlocal receipt pathはpublic '
        'attachmentではありません' in source
    )
    assert 'maintainerのlive step-by-step guidanceは' in source
    assert '[Independent First-map Validation](external-first-map-validation.md)' in source


def test_stale_task_cannot_be_rendered_copy_ready():
    """Stale task details are not emitted as a copy-ready issue body."""
    report = {
        'stale_tasks': [{
            'id': 'starter-C2',
            'status': 'STALE',
            'reasons': ['planned marker already exists'],
        }],
    }

    with pytest.raises(
        CHECKER.QueueError,
        match='starter-C2 is stale and must be reviewed before rendering',
    ):
        CHECKER._require_render_ready(report, 'starter-C2')


def test_docs_verifier_uses_temp_site_and_never_contract_shell(
    monkeypatch,
):
    """Docs verification writes only to a temporary site directory."""
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, '', '')

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)
    task = _task(_queue(), 'starter-C1')

    report = CHECKER.verify_task(task)

    assert report['status'] == 'FOCUSED_CHECKS_PASSED'
    assert report['workspace_artifacts_written'] is False
    assert len(calls) == 1
    command, kwargs = calls[0]
    assert command[:6] == [
        sys.executable, '-m', 'mkdocs', 'build', '--strict', '--site-dir'
    ]
    assert Path(command[6]).name == 'site'
    assert ROOT not in Path(command[6]).parents
    assert kwargs['env']['PYTHONDONTWRITEBYTECODE'] == '1'


def test_c5_verifier_uses_the_fixed_docs_profile(monkeypatch):
    """C5 verification follows the fixed built-in docs profile."""
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, '', '')

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)
    task = _task(_queue(), 'starter-C5')

    report = CHECKER.verify_task(task)

    assert report['status'] == 'FOCUSED_CHECKS_PASSED'
    assert len(calls) == 1
    command, kwargs = calls[0]
    assert command[:6] == [
        sys.executable, '-m', 'mkdocs', 'build', '--strict', '--site-dir'
    ]
    assert Path(command[6]).name == 'site'
    assert ROOT not in Path(command[6]).parents
    assert kwargs['env']['PYTEST_ADDOPTS'] == '-p no:cacheprovider'


def test_code_verifier_stops_after_first_failure(monkeypatch):
    """A failed focused check prevents later checks from running."""
    calls = []

    def fake_run(command, **kwargs):
        calls.append(command)
        return subprocess.CompletedProcess(command, 1, '', '')

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)
    task = _task(_queue(), 'starter-C5')

    report = CHECKER.verify_task(task)

    assert report['status'] == 'FOCUSED_CHECKS_FAILED'
    assert len(calls) == 1
    assert report['checks'][0]['returncode'] == 1


def test_default_cli_json_is_path_private_and_no_write():
    """Default machine output contains no private path or write claim."""
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    payload = json.loads(result.stdout)
    assert payload['status'] == 'QUEUE_READY_LOCAL_ONLY'
    assert payload['ready_task_ids'] == list(CHECKER.EXPECTED_TASK_IDS)
    assert payload['stale_tasks'] == []
    assert payload['authority']['github_writes_authorized'] is False
    assert '/home/' not in result.stdout
    assert '/tmp/' not in result.stdout
    assert result.stderr == ''


def test_list_cli_names_every_task_and_publication_boundary():
    """The list view keeps all five tasks behind the publication gate."""
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--list'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert 'PREPARED_NOT_PUBLISHED' in result.stdout
    assert 'Wait for a published issue' in result.stdout
    for task_id in CHECKER.EXPECTED_TASK_IDS:
        assert task_id in result.stdout
    assert result.stderr == ''
