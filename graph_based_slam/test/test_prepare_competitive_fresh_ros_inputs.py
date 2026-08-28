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

"""Contract tests for the fresh ROS input preparation checkpoint."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'prepare_competitive_fresh_ros_inputs.py'
SPEC = importlib.util.spec_from_file_location('prepare_fresh_ros_inputs', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True) + '\n', encoding='utf-8')


def _make_root(tmp_path: Path, raw: bytes = b'raw-bag') -> tuple[Path, dict]:
    root = tmp_path / 'managed'
    slot = root / 'slots' / 'exp14'
    slot.mkdir(parents=True)
    raw_path = slot / 'source' / 'exp14.bag'
    raw_path.parent.mkdir()
    raw_path.write_bytes(raw)
    plan_sha = '1' * 64
    _write_json(root / '.freeze-root.json', {
        'schema_version': 1,
        'marker_kind': 'competitive_fresh_holdout_managed_root',
        'plan_sha256': plan_sha,
        'selection_receipt_sha256': '2' * 64,
    })
    manifest = {
        'schema_version': 1,
        'manifest_kind': 'competitive_fresh_holdout',
        'status': 'downloaded_hashed',
        'sequence': 'exp14',
        'source': {'plan_sha256': plan_sha},
        'raw_bag': {
            'path': 'slots/exp14/source/exp14.bag',
            'bytes': len(raw),
            'sha256': MODULE.sha256_file(raw_path),
        },
        # Deliberately not created.  A successful preparation proves that this
        # opaque GT path was never opened.
        'ground_truth': {
            'path': 'slots/exp14/ground_truth/exp14.txt',
            'bytes': 123,
            'sha256': '3' * 64,
            'content_opened': False,
        },
    }
    manifest_path = slot / 'manifest.json'
    _write_json(manifest_path, manifest)
    return root, manifest


def _runtime() -> dict[str, str]:
    return {
        'rosbags_version': '0.11.0',
        'python_version': '3.12.3',
        'numpy_version': '2.0.0',
        'converter_executable': '/tmp/rosbags-convert',
        'converter_script_sha256': '4' * 64,
        'converter_help_sha256': '5' * 64,
        'comparator_script': 'scripts/compare_rosbag_semantic_inputs.py',
        'comparator_script_sha256': MODULE.sha256_file(MODULE.COMPARATOR),
    }


def _metadata() -> dict:
    return {
        'rosbag2_bagfile_information': {
            'topics_with_message_count': [
                {'topic_metadata': {'name': name, 'type': msg_type}}
                for name, msg_type, _role in MODULE.CANONICAL_TOPIC_CONTRACT
            ]
        }
    }


def _semantic_report() -> dict:
    return {
        'schema_version': 1,
        'all_topics_equal': True,
        'topics': [
            {
                'topic': name,
                'equal': True,
                'message_count_left': 1,
                'message_count_right': 1,
                'aggregate_sha256_left': 'a' * 64,
                'aggregate_sha256_right': 'a' * 64,
                'first_mismatch_index': None,
            }
            for name, _msg_type, _role in MODULE.CANONICAL_TOPIC_CONTRACT
        ],
    }


def _mock_tools(monkeypatch):
    calls = []

    def run(command, check, capture_output, text):
        del check, capture_output, text
        calls.append(command)
        if command[0] == MODULE.ROSBAGS_CONVERT:
            destination = Path(command[command.index('--dst') + 1])
            destination.mkdir()
            (destination / 'metadata.yaml').write_text(
                yaml.safe_dump(_metadata()), encoding='utf-8')
        else:
            assert command[1] == str(MODULE.COMPARATOR)
            output = Path(command[command.index('--output') + 1])
            output.write_text(json.dumps(_semantic_report()) + '\n', encoding='utf-8')
        return type('Completed', (), {'stdout': '', 'stderr': ''})()

    monkeypatch.setattr(MODULE, '_probe_runtime', lambda: _runtime())
    monkeypatch.setattr(MODULE.subprocess, 'run', run)
    return calls


def test_sequence_and_all_selection_are_manifest_scoped(tmp_path):
    root, _manifest = _make_root(tmp_path)
    assert MODULE._manifest_paths(root, ['exp14']) == ['exp14']
    assert MODULE._manifest_paths(root, None) == ['exp14']


def test_prepare_pins_commands_and_never_opens_gt(tmp_path, monkeypatch):
    root, _manifest = _make_root(tmp_path)
    calls = _mock_tools(monkeypatch)
    marker = MODULE._managed_marker(root)
    result = MODULE._prepare_slot(root, 'exp14', False, _runtime(), marker)
    slot = root / 'slots' / 'exp14'
    assert result['status'] == 'prepared'
    assert (slot / 'canonical_ros2').is_dir()
    assert (slot / 'semantic_equivalence.json').is_file()
    assert (slot / 'preparation_receipt.json').is_file()
    assert not (slot / 'ground_truth' / 'exp14.txt').exists()
    assert calls[0] == [
        'rosbags-convert', '--src', str(slot / 'source' / 'exp14.bag'),
        '--dst', str(slot / 'canonical_ros2.part'), '--dst-storage', 'sqlite3',
        '--compress', 'none', '--src-typestore', 'ros1_noetic',
        '--dst-typestore', 'copy']
    assert calls[1][1] == str(MODULE.COMPARATOR)
    assert calls[1][calls[1].index('--left') + 1].endswith('exp14.bag')
    assert calls[1][calls[1].index('--right') + 1].endswith('canonical_ros2.part')
    receipt = json.loads(
        (slot / 'preparation_receipt.json').read_text(encoding='utf-8'))
    assert receipt['runtime']['rosbags_version'] == '0.11.0'
    assert receipt['ground_truth_content_opened'] is False
    assert receipt['semantic_report_all_topics_equal'] is True


def test_resume_requires_receipt_identity_and_skips_verified_output(
        tmp_path, monkeypatch):
    root, _manifest = _make_root(tmp_path)
    _mock_tools(monkeypatch)
    marker = MODULE._managed_marker(root)
    MODULE._prepare_slot(root, 'exp14', False, _runtime(), marker)
    monkeypatch.setattr(MODULE.subprocess, 'run',
                        lambda *args, **kwargs: pytest.fail('resume executed tools'))
    result = MODULE._prepare_slot(root, 'exp14', True, _runtime(), marker)
    assert result['resumed'] is True
    receipt_path = root / 'slots' / 'exp14' / 'preparation_receipt.json'
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    receipt['plan_sha256'] = 'f' * 64
    receipt_path.write_text(json.dumps(receipt), encoding='utf-8')
    with pytest.raises(ValueError, match='plan identity'):
        MODULE._prepare_slot(root, 'exp14', True, _runtime(), marker)


def test_resume_publishes_verified_staging_receipt(tmp_path, monkeypatch):
    root, _manifest = _make_root(tmp_path)
    _mock_tools(monkeypatch)
    marker = MODULE._managed_marker(root)
    MODULE._prepare_slot(root, 'exp14', False, _runtime(), marker)
    slot = root / 'slots' / 'exp14'
    for final_name, staged_name in (
            ('canonical_ros2', 'canonical_ros2.part'),
            ('semantic_equivalence.json', 'semantic_equivalence.json.part'),
            ('preparation_receipt.json', 'preparation_receipt.json.part')):
        (slot / final_name).replace(slot / staged_name)
    monkeypatch.setattr(MODULE.subprocess, 'run',
                        lambda *args, **kwargs: pytest.fail('staging resume ran tools'))
    result = MODULE._prepare_slot(root, 'exp14', True, _runtime(), marker)
    assert result['resumed'] is True
    assert (slot / 'canonical_ros2').is_dir()
    assert (slot / 'preparation_receipt.json').is_file()


def test_resume_after_converter_crash_does_not_reconvert(tmp_path, monkeypatch):
    root, _manifest = _make_root(tmp_path)
    marker = MODULE._managed_marker(root)

    def converter_crash(command, check, capture_output, text):
        del check, capture_output, text
        assert command[0] == MODULE.ROSBAGS_CONVERT
        destination = Path(command[command.index('--dst') + 1])
        destination.mkdir()
        (destination / 'metadata.yaml').write_text(
            yaml.safe_dump(_metadata()), encoding='utf-8')
        raise RuntimeError('simulated converter crash')

    monkeypatch.setattr(MODULE.subprocess, 'run', converter_crash)
    with pytest.raises(RuntimeError, match='converter crash'):
        MODULE._prepare_slot(root, 'exp14', False, _runtime(), marker)
    assert (root / 'slots' / 'exp14' / 'canonical_ros2.part').is_dir()

    calls = _mock_tools(monkeypatch)
    result = MODULE._prepare_slot(root, 'exp14', True, _runtime(), marker)
    assert result['resumed'] is True
    assert all(command[0] != MODULE.ROSBAGS_CONVERT for command in calls)


def test_resume_after_comparator_crash_reuses_semantic_part(tmp_path, monkeypatch):
    root, _manifest = _make_root(tmp_path)
    marker = MODULE._managed_marker(root)

    def comparator_crash(command, check, capture_output, text):
        del check, capture_output, text
        if command[0] == MODULE.ROSBAGS_CONVERT:
            destination = Path(command[command.index('--dst') + 1])
            destination.mkdir()
            (destination / 'metadata.yaml').write_text(
                yaml.safe_dump(_metadata()), encoding='utf-8')
            return
        output = Path(command[command.index('--output') + 1])
        output.write_text(json.dumps(_semantic_report()) + '\n', encoding='utf-8')
        raise RuntimeError('simulated comparator crash')

    monkeypatch.setattr(MODULE.subprocess, 'run', comparator_crash)
    with pytest.raises(RuntimeError, match='comparator crash'):
        MODULE._prepare_slot(root, 'exp14', False, _runtime(), marker)
    slot = root / 'slots' / 'exp14'
    assert (slot / 'canonical_ros2.part').is_dir()
    assert (slot / 'semantic_equivalence.json.part').is_file()

    monkeypatch.setattr(MODULE.subprocess, 'run',
                        lambda *args, **kwargs: pytest.fail('resume reran a tool'))
    result = MODULE._prepare_slot(root, 'exp14', True, _runtime(), marker)
    assert result['resumed'] is True
    assert (slot / 'preparation_receipt.json').is_file()


def test_resume_after_first_publish_rename_completes_remaining_files(
        tmp_path, monkeypatch):
    root, _manifest = _make_root(tmp_path)
    _mock_tools(monkeypatch)
    marker = MODULE._managed_marker(root)
    MODULE._prepare_slot(root, 'exp14', False, _runtime(), marker)
    slot = root / 'slots' / 'exp14'
    (slot / 'preparation_receipt.json').replace(
        slot / 'preparation_receipt.json.part')
    (slot / 'semantic_equivalence.json').replace(
        slot / 'semantic_equivalence.json.part')
    monkeypatch.setattr(MODULE.subprocess, 'run',
                        lambda *args, **kwargs: pytest.fail('rename resume ran tools'))
    result = MODULE._prepare_slot(root, 'exp14', True, _runtime(), marker)
    assert result['resumed'] is True
    assert (slot / 'canonical_ros2').is_dir()
    assert (slot / 'semantic_equivalence.json').is_file()
    assert (slot / 'preparation_receipt.json').is_file()


def test_resume_after_second_publish_rename_completes_receipt(tmp_path, monkeypatch):
    root, _manifest = _make_root(tmp_path)
    _mock_tools(monkeypatch)
    marker = MODULE._managed_marker(root)
    MODULE._prepare_slot(root, 'exp14', False, _runtime(), marker)
    slot = root / 'slots' / 'exp14'
    (slot / 'preparation_receipt.json').replace(
        slot / 'preparation_receipt.json.part')
    monkeypatch.setattr(MODULE.subprocess, 'run',
                        lambda *args, **kwargs: pytest.fail('rename resume ran tools'))
    result = MODULE._prepare_slot(root, 'exp14', True, _runtime(), marker)
    assert result['resumed'] is True
    assert (slot / 'preparation_receipt.json').is_file()


def test_resume_rejects_raw_or_ros2_tamper(tmp_path, monkeypatch):
    root, _manifest = _make_root(tmp_path)
    _mock_tools(monkeypatch)
    marker = MODULE._managed_marker(root)
    MODULE._prepare_slot(root, 'exp14', False, _runtime(), marker)
    (root / 'slots' / 'exp14' / 'source' / 'exp14.bag').write_bytes(b'tampered')
    with pytest.raises(ValueError, match='raw_bag|raw-bag identity|SHA'):
        MODULE._prepare_slot(root, 'exp14', True, _runtime(), marker)


def test_converter_version_is_exactly_pinned(monkeypatch):
    def version(name):
        if name == 'rosbags':
            return '0.10.0'
        return '2.0.0'

    monkeypatch.setattr(MODULE.importlib.metadata, 'version', version)
    with pytest.raises(ValueError, match='0.11.0'):
        MODULE._probe_runtime()


def test_semantic_report_requires_all_canonical_topics(tmp_path):
    report_path = tmp_path / 'semantic.json'
    report = _semantic_report()
    report['topics'] = report['topics'][:-1]
    report_path.write_text(json.dumps(report), encoding='utf-8')
    with pytest.raises(ValueError, match='canonical topic contract'):
        MODULE._validate_semantic_report(report_path)


def test_manifest_path_traversal_fails_before_any_gt_access(tmp_path):
    root, manifest = _make_root(tmp_path)
    manifest['raw_bag']['path'] = '../outside.bag'
    _write_json(root / 'slots' / 'exp14' / 'manifest.json', manifest)
    with pytest.raises(ValueError, match='relative|escapes|must not contain'):
        MODULE._manifest_for(root, 'exp14')


def test_symlink_slot_is_rejected(tmp_path):
    root, _manifest = _make_root(tmp_path)
    slot = root / 'slots' / 'exp14'
    target = tmp_path / 'outside-slot'
    slot.rename(target)
    slot.symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError, match='unsafe'):
        MODULE._manifest_for(root, 'exp14')


def test_unmanaged_partial_output_requires_resume(tmp_path, monkeypatch):
    root, _manifest = _make_root(tmp_path)
    slot = root / 'slots' / 'exp14'
    (slot / 'canonical_ros2.part').mkdir()
    marker = MODULE._managed_marker(root)
    with pytest.raises(ValueError, match='use --resume'):
        MODULE._prepare_slot(root, 'exp14', False, _runtime(), marker)


def test_unmanaged_slot_entry_is_rejected(tmp_path):
    root, _manifest = _make_root(tmp_path)
    (root / 'slots' / 'exp14' / 'unexpected.txt').write_text('x', encoding='utf-8')
    marker = MODULE._managed_marker(root)
    with pytest.raises(ValueError, match='unmanaged slot entry'):
        MODULE._prepare_slot(root, 'exp14', False, _runtime(), marker)
