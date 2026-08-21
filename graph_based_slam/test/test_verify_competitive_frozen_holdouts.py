# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Contract tests for the read-only frozen-holdout deep verifier."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'verify_competitive_frozen_holdouts.py'
SPEC = importlib.util.spec_from_file_location('verify_frozen_holdouts', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True) + '\n', encoding='utf-8')


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


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


def _make_fixture(tmp_path: Path) -> tuple[Path, Path, Path]:
    """Build tiny, fully-hashed artifacts without reading any real dataset."""
    root = tmp_path / 'managed'
    root.mkdir(parents=True)
    converter = tmp_path / 'rosbags-convert'
    converter.write_bytes(b'synthetic converter executable')
    revision = 'e' * 40
    calibration_bytes = b'calibration-fixture\n'
    calibration_sha = hashlib.sha256(calibration_bytes).hexdigest()
    calibration_path_for_blob = tmp_path / 'calibration-fixture.yaml'
    calibration_path_for_blob.write_bytes(calibration_bytes)
    calibration_blob = MODULE.git_blob_sha1(calibration_path_for_blob)
    selected = {}
    artifacts = {}
    for candidate_name, sequence in zip(
            MODULE.EXPECTED_CANDIDATES, MODULE.EXPECTED_SEQUENCES):
        slot = root / 'slots' / sequence
        raw_path = slot / 'source' / f'{sequence}.bag'
        gt_path = slot / 'ground_truth' / f'{sequence}.txt'
        calibration_path = slot / 'calibration' / 'files' / 'calib.yaml'
        raw_path.parent.mkdir(parents=True)
        gt_path.parent.mkdir(parents=True)
        calibration_path.parent.mkdir(parents=True)
        raw_path.write_bytes(f'raw-{sequence}\n'.encode())
        gt_path.write_bytes(f'opaque-ground-truth-{sequence}\n'.encode())
        calibration_path.write_bytes(calibration_bytes)
        ros2_path = slot / 'canonical_ros2'
        ros2_path.mkdir(parents=True)
        (ros2_path / 'metadata.yaml').write_text(
            yaml.safe_dump(_metadata()), encoding='utf-8')
        semantic_path = slot / 'semantic_equivalence.json'
        _write_json(semantic_path, _semantic_report())
        artifacts[sequence] = {
            'raw_path': raw_path,
            'gt_path': gt_path,
            'calibration_path': calibration_path,
            'ros2_path': ros2_path,
            'semantic_path': semantic_path,
        }
        selected[candidate_name] = {
            'dataset': f'synthetic_{sequence}',
            'sequence': sequence,
            'bag': {
                'expected_bytes': raw_path.stat().st_size,
                'lfs_sha256': _sha256(raw_path),
                'git_blob_oid': MODULE.git_blob_sha1(raw_path),
            },
            'ground_truth': {
                'expected_bytes': gt_path.stat().st_size,
                'git_blob_oid': MODULE.git_blob_sha1(gt_path),
                'sha256': _sha256(gt_path),
            },
            'calibration_archive_sha256': None,
            'input_manifest_sha256': None,
        }
    selection = {
        'schema_version': 1,
        'receipt_kind': MODULE.SELECTION_KIND,
        'status': 'selected_unopened',
        'official_source': {
            'repository': 'https://example.invalid/dataset',
            'revision': revision,
            'calibration': {
                'tree_path': 'calibration/calibration_files',
                'tree_oid': 'c' * 40,
                'relevant_files': {
                    'calib.yaml': {
                        'expected_bytes': len(calibration_bytes),
                        'git_blob_oid': calibration_blob,
                    }
                },
            },
        },
        'selection_decision': {
            'selected_candidates': selected,
        },
    }
    selection_path = tmp_path / 'selection.yaml'
    selection_path.write_text(yaml.safe_dump(selection, sort_keys=False),
                              encoding='utf-8')
    selection_sha = MODULE._sha256_file(selection_path, 'selection')
    plan_sha = '1' * 64
    _write_json(root / '.freeze-root.json', {
        'schema_version': 1,
        'marker_kind': 'competitive_fresh_holdout_managed_root',
        'plan_sha256': plan_sha,
        'selection_receipt_sha256': selection_sha,
        'destination_root': str(root),
    })
    comparator = ROOT / 'scripts' / 'compare_rosbag_semantic_inputs.py'
    runtime = {
        'rosbags_version': MODULE.ROSBAGS_VERSION,
        'python_version': '3.12.3',
        'numpy_version': '2.0.0',
        'converter_executable': str(converter),
        'converter_script_sha256': _sha256(converter),
        'converter_help_sha256': '5' * 64,
        'comparator_script': 'scripts/compare_rosbag_semantic_inputs.py',
        'comparator_script_sha256': _sha256(comparator),
    }
    for candidate_name, candidate in selected.items():
        sequence = candidate['sequence']
        slot = root / 'slots' / sequence
        item = artifacts[sequence]
        topics = MODULE._metadata_topics(item['ros2_path'])
        ros2_tree_sha = MODULE._safe_tree_sha256(
            item['ros2_path'], root, f'{sequence} canonical ROS 2')
        report, semantic_sha, report_sha = MODULE._semantic_identity(
            item['semantic_path'])
        required_contract = [
            {'name': name, 'type': msg_type, 'role': role}
            for name, msg_type, role in MODULE.CANONICAL_TOPIC_CONTRACT]
        canonical_identity = {
            'canonical_rosbag2_tree_sha256': ros2_tree_sha,
            'topics': topics,
            'required_topic_contract': required_contract,
            'semantic_equivalence_sha256': semantic_sha,
            'semantic_equivalence_hash_kind': MODULE.SEMANTIC_HASH_KIND,
        }
        calibration_entries = [{
            'logical_path': 'calibration/calibration_files/calib.yaml',
            'path': f'slots/{sequence}/calibration/files/calib.yaml',
            'bytes': item['calibration_path'].stat().st_size,
            'sha256': calibration_sha,
            'git_blob_sha1': calibration_blob,
        }]
        calibration = {
            'hash_kind': MODULE.CALIBRATION_HASH_KIND,
            'tree_path': 'calibration/calibration_files',
            'tree_oid': 'c' * 40,
            'archive_sha256': None,
            'files': calibration_entries,
            'sha256': MODULE.calibration_tree_sha256(calibration_entries),
        }
        raw_rel = f'slots/{sequence}/source/{sequence}.bag'
        gt_rel = f'slots/{sequence}/ground_truth/{sequence}.txt'
        base_manifest = {
            'schema_version': 1,
            'manifest_kind': 'competitive_fresh_holdout',
            'status': 'downloaded_hashed',
            'slot': candidate_name,
            'sequence': sequence,
            'dataset': candidate['dataset'],
            'source': {
                'revision': revision,
                'selection_receipt_sha256': selection_sha,
                'plan_sha256': plan_sha,
            },
            'hash_policy': {
                'ground_truth': 'opaque_file_sha256_only_no_parser',
                'calibration': MODULE.CALIBRATION_HASH_KIND,
                'input_manifest': MODULE.INPUT_HASH_KIND,
                'semantic_equivalence': MODULE.SEMANTIC_HASH_KIND,
            },
            'raw_bag': {
                'path': raw_rel,
                'bytes': item['raw_path'].stat().st_size,
                'sha256': _sha256(item['raw_path']),
            },
            'ground_truth': {
                'path': gt_rel,
                'bytes': item['gt_path'].stat().st_size,
                'sha256': _sha256(item['gt_path']),
                'content_opened': False,
            },
            'calibration': calibration,
            'canonical_rosbag2': None,
            'semantic_equivalence_sha256': None,
            'input_manifest_sha256': None,
            'blind_audit': {
                'ground_truth_content_opened': False,
                'trajectory_metrics_computed': False,
                'performance_data_recorded': False,
            },
        }
        input_payload = {
            'hash_kind': MODULE.INPUT_HASH_KIND,
            'raw_bag_sha256': base_manifest['raw_bag']['sha256'],
            'canonical_rosbag2_tree_sha256': ros2_tree_sha,
            'semantic_equivalence_sha256': semantic_sha,
            'required_topic_contract': required_contract,
        }
        final_manifest = json.loads(json.dumps(base_manifest))
        final_manifest['status'] = 'frozen_unopened'
        final_manifest['canonical_rosbag2'] = canonical_identity
        final_manifest['semantic_equivalence_sha256'] = semantic_sha
        final_manifest['input_manifest_sha256'] = MODULE.canonical_json_sha256(
            input_payload)
        final_manifest['input_manifest_hash_kind'] = MODULE.INPUT_HASH_KIND
        final_manifest['blind_audit'].update({
            'canonical_rosbag2_metadata_only': True,
            'semantic_report_payload_only': True,
        })
        manifest_path = slot / 'manifest.json'
        _write_json(manifest_path, final_manifest)
        converter_argv = [
            'rosbags-convert', '--src', str(item['raw_path']), '--dst',
            str(slot / 'canonical_ros2.part'), '--dst-storage', 'sqlite3',
            '--compress', 'none', '--src-typestore', 'ros1_noetic',
            '--dst-typestore', 'copy',
        ]
        report_part = slot / 'semantic_equivalence.json.part'
        comparator_argv = [
            sys.executable, str(comparator), '--left', str(item['raw_path']),
            '--right', str(slot / 'canonical_ros2.part'),
        ]
        for name, _msg_type, _role in MODULE.CANONICAL_TOPIC_CONTRACT:
            comparator_argv.extend(['--topic', name])
        comparator_argv.extend(['--output', str(report_part)])
        receipt = {
            'schema_version': 1,
            'receipt_kind': MODULE.PREPARATION_KIND,
            'status': 'prepared',
            'sequence': sequence,
            'plan_sha256': plan_sha,
            'manifest_sha256': MODULE.canonical_json_sha256(
                MODULE._base_manifest_from_final(final_manifest)),
            'raw_bag': final_manifest['raw_bag'],
            'ground_truth_content_opened': False,
            'runtime': runtime,
            'converter_argv': converter_argv,
            'comparator_argv': comparator_argv,
            'topics': topics,
            'ros2_tree_sha256': ros2_tree_sha,
            'semantic_report_sha256': report_sha,
            'semantic_equivalence_sha256': semantic_sha,
            'semantic_equivalence_hash_kind': MODULE.SEMANTIC_HASH_KIND,
            'semantic_report_all_topics_equal': report['all_topics_equal'],
        }
        _write_json(slot / 'preparation_receipt.json', receipt)
    return root, selection_path, tmp_path / 'deep-verification.json'


def _run(root: Path, selection: Path, output: Path) -> int:
    return MODULE.main([
        '--root', str(root), '--selection', str(selection), '--output', str(output)])


def test_three_slot_fixture_passes_and_keeps_gt_opaque(tmp_path, capsys):
    root, selection, output = _make_fixture(tmp_path)
    assert _run(root, selection, output) == 0
    result = json.loads(output.read_text(encoding='utf-8'))
    assert result['status'] == 'PASS'
    assert [row['sequence'] for row in result['slots']] == list(
        MODULE.EXPECTED_SEQUENCES)
    text = output.read_text(encoding='utf-8')
    assert 'opaque-ground-truth' not in text
    assert 'ground_truth/exp' not in text
    stdout = capsys.readouterr().out
    assert 'opaque-ground-truth' not in stdout
    assert 'ground_truth/exp' not in stdout


@pytest.mark.parametrize('tamper', ('raw', 'ground_truth', 'calibration',
                                    'semantic', 'receipt', 'path'))
def test_artifact_tampering_fails_without_gt_leak(tmp_path, tamper):
    root, selection, output = _make_fixture(tmp_path)
    slot = root / 'slots' / 'exp14'
    if tamper == 'raw':
        (slot / 'source' / 'exp14.bag').write_bytes(b'tampered')
    elif tamper == 'ground_truth':
        (slot / 'ground_truth' / 'exp14.txt').write_bytes(b'tampered')
    elif tamper == 'calibration':
        (slot / 'calibration' / 'files' / 'calib.yaml').write_bytes(b'tampered')
    elif tamper == 'semantic':
        report = json.loads(
            (slot / 'semantic_equivalence.json').read_text(encoding='utf-8'))
        report['all_topics_equal'] = False
        _write_json(slot / 'semantic_equivalence.json', report)
    elif tamper == 'receipt':
        receipt_path = slot / 'preparation_receipt.json'
        receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
        receipt['manifest_sha256'] = 'f' * 64
        _write_json(receipt_path, receipt)
    else:
        manifest_path = slot / 'manifest.json'
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        manifest['raw_bag']['path'] = '../outside.bag'
        _write_json(manifest_path, manifest)
    assert _run(root, selection, output) == 1
    text = output.read_text(encoding='utf-8')
    assert 'opaque-ground-truth' not in text
    assert 'ground_truth/exp' not in text


def test_symlink_tree_and_extra_slot_fail_closed(tmp_path):
    root, selection, output = _make_fixture(tmp_path)
    ros2 = root / 'slots' / 'exp14' / 'canonical_ros2'
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'metadata.yaml').write_text(yaml.safe_dump(_metadata()),
                                           encoding='utf-8')
    shutil.rmtree(ros2)
    os.symlink(outside, ros2)
    assert _run(root, selection, output) == 1

    root, selection, output = _make_fixture(tmp_path / 'extra')
    (root / 'slots' / 'exp99').mkdir()
    assert _run(root, selection, output) == 1


def test_marker_selection_and_plan_binding_fail_closed(tmp_path):
    root, selection, output = _make_fixture(tmp_path)
    marker_path = root / '.freeze-root.json'
    marker = json.loads(marker_path.read_text(encoding='utf-8'))
    marker['selection_receipt_sha256'] = 'f' * 64
    _write_json(marker_path, marker)
    assert _run(root, selection, output) == 1
