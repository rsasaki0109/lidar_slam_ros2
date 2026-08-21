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

"""Contract tests for the GT-blind M6a driver."""

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'run_competitive_gt_blind_benchmark',
    ROOT / 'scripts/run_competitive_gt_blind_benchmark.py')
RUNNER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RUNNER)


def test_schedule_is_explicit_27_attempt_product():
    attempts = RUNNER.schedule()
    assert len(attempts) == 27
    assert [(item['system'], item['slot'], item['repetition'])
            for item in attempts[:3]] == [
                ('ours', 'fresh_1', 1), ('ours', 'fresh_1', 2),
                ('ours', 'fresh_1', 3)]
    assert attempts[-1] == {
        'schedule_index': 27, 'system': 'fast_livo2',
        'slot': 'fresh_3', 'repetition': 3}
    assert RUNNER.schedule_sha(attempts) == RUNNER.schedule_sha()


def test_input_and_output_roots_must_not_overlap(tmp_path):
    input_root = tmp_path / 'inputs'
    input_root.mkdir()
    with pytest.raises(RUNNER.ContractError):
        RUNNER.assert_roots_are_disjoint(input_root, input_root / 'results')
    with pytest.raises(RUNNER.ContractError):
        nested = input_root / 'nested'
        nested.mkdir()
        RUNNER.assert_roots_are_disjoint(nested, input_root)


def test_gt_path_cannot_be_reached_by_mount_or_environment(tmp_path):
    gt = tmp_path / 'ground_truth.txt'
    gt.write_text('opaque fixture\n')
    bag = tmp_path / 'bag'
    bag.mkdir()
    item = {'gt_realpath': gt.resolve(), 'gt_device': gt.stat().st_dev,
            'gt_inode': gt.stat().st_ino}
    guard = RUNNER._guard_gt(item, [(bag, '/input/bag', 'ro')],
                             ['docker', 'run', '--network', 'none'],
                             {'BAG_PATH': '/input/bag'})
    assert guard['ground_truth_reachable'] is False
    with pytest.raises(RUNNER.ContractError):
        RUNNER._guard_gt(item, [(tmp_path, '/input', 'ro')],
                         ['docker', 'run'], {})
    with pytest.raises(RUNNER.ContractError):
        RUNNER._guard_gt(item, [(bag, '/input/bag', 'ro')],
                         ['docker', 'run'], {'INPUT': 'ground_truth.txt'})


def test_marker_mismatch_and_unowned_root_are_rejected(tmp_path):
    root = tmp_path / 'results'
    root.mkdir()
    identity = {'schedule_sha256': 'a' * 64}
    RUNNER.check_or_create_marker(root, identity, create=True)
    with pytest.raises(RUNNER.ContractError):
        RUNNER.check_or_create_marker(root, {'schedule_sha256': 'b' * 64}, create=False)
    other = tmp_path / 'other'
    other.mkdir()
    (other / 'stale.txt').write_text('stale\n')
    with pytest.raises(RUNNER.ContractError):
        RUNNER.check_or_create_marker(other, identity, create=False)


def test_image_digest_must_be_immutable():
    receipt = {'systems': {'ours': {'container': {
        'image_tag': 'ours:test', 'image_digest': 'not-a-digest'}}}}
    with pytest.raises(RUNNER.ContractError):
        RUNNER.image_ref_and_labels('ours', receipt, inspect=False)


def test_fast_command_has_raw_only_mount_and_semantic_identity():
    item = {
        'raw_path': Path('/managed/slots/exp14/source/exp14.bag'),
        'canonical_path': Path('/managed/slots/exp14/canonical_ros2'),
    }
    command, env = RUNNER.docker_command(
        'fast_livo2', item, 'fast:test@sha256:' + 'a' * 64,
        Path('/M6A_OUTPUT_PLACEHOLDER'),
        {'schedule_index': 1, 'system': 'fast_livo2',
         'slot': 'fresh_1', 'repetition': 1})
    command_text = '\0'.join(command + list(env.values()))
    assert '/input/raw_input.bag' in command_text
    assert '/input/canonical_ros2' not in command_text
    assert '/ground_truth' not in command_text
    assert '/calibration' not in command_text


def test_frozen_manifest_semantic_mismatch_is_rejected(tmp_path):
    root = tmp_path / 'managed'
    canonical = root / 'slots/exp14/canonical_ros2'
    source = root / 'slots/exp14/source'
    canonical.mkdir(parents=True)
    source.mkdir(parents=True)
    (canonical / 'metadata.yaml').write_text('metadata\n')
    raw = source / 'exp14.bag'
    raw.write_bytes(b'raw')
    gt = root / 'slots/exp14/ground_truth/exp14.txt'
    gt.parent.mkdir()
    gt.write_bytes(b'opaque')
    manifest = {
        'status': 'frozen_unopened',
        'raw_bag': {'sha256': 'a' * 64},
        'canonical_rosbag2': {
            'canonical_rosbag2_tree_sha256': 'b' * 64,
            'semantic_equivalence_sha256': 'c' * 64},
        'semantic_equivalence_sha256': 'c' * 64,
        'input_manifest_sha256': 'd' * 64,
        'calibration': {'sha256': 'e' * 64},
    }
    manifest_path = root / 'slots/exp14/manifest.json'
    manifest_path.write_text(json.dumps(manifest))
    identity = {
        'raw_bag_path': 'slots/exp14/source/exp14.bag',
        'ground_truth_path': 'slots/exp14/ground_truth/exp14.txt',
        'manifest_path': 'slots/exp14/manifest.json',
        'manifest_file_sha256': RUNNER.sha256_file(manifest_path),
        'raw_bag_sha256': 'a' * 64, 'raw_bag_bytes': 3,
        'canonical_rosbag2_tree_sha256': 'b' * 64,
        'semantic_equivalence_sha256': 'c' * 64,
        'input_manifest_sha256': 'd' * 64,
        'calibration_archive_sha256': 'e' * 64,
    }
    profile = {'datasets': {'fresh_holdout_slots': {
        'fresh_1': {'status': 'frozen_unopened', 'sequence': 'exp14',
                    'frozen_identity': identity}}}}
    manifest['semantic_equivalence_sha256'] = 'f' * 64
    manifest['canonical_rosbag2']['semantic_equivalence_sha256'] = 'f' * 64
    manifest_path.write_text(json.dumps(manifest))
    identity['manifest_file_sha256'] = RUNNER.sha256_file(manifest_path)
    with pytest.raises(RUNNER.ContractError, match='semantic'):
        RUNNER.resolve_slot(root, profile, 'fresh_1')


def test_nonzero_timeout_is_retained_as_immutable_failed_attempt(tmp_path, monkeypatch):
    output = tmp_path / 'results'
    output.mkdir()
    item = {
        'schedule_index': 1, 'system': 'glim_cpu', 'slot': 'fresh_1',
        'sequence': 'exp14', 'repetition': 1,
        'image_digest': 'sha256:' + 'a' * 64,
        'execution_identity': {'revision': 'b' * 40},
        'input': {'semantic_equivalence_sha256': 'c' * 64},
        'profile_canonical_sha256': 'd' * 64,
        'execution_receipt_file_sha256': 'e' * 64,
        'selection_receipt_file_sha256': 'f' * 64,
        'argv': ['docker', 'run', '-v', '/M6A_OUTPUT_PLACEHOLDER:/out'],
        'env': {'OMP_NUM_THREADS': '8'},
        'mounts': [{'source': '/managed/bag', 'target': '/input/bag', 'mode': 'ro'}],
        'gt_blind_guard': {'ground_truth_reachable': False,
                           'gt_device': 1, 'gt_inode': 2,
                           'mount_sources': ['/managed/bag']},
    }

    # The driver catches subprocess.TimeoutExpired; use that exact exception
    # to exercise the formal timeout path rather than a generic OS failure.
    def expired(*_args, **_kwargs):
        raise RUNNER.subprocess.TimeoutExpired(['docker', 'run'], 1)

    monkeypatch.setattr(RUNNER.subprocess, 'run', expired)
    report = RUNNER.run_attempt(item, output, 1.0)
    final = output / 'attempt_001'
    assert final.is_dir() and not (output / 'attempt_001.part').exists()
    assert report['execution']['timed_out'] is True
    assert report['completion']['complete'] is False
    assert json.loads((final / 'attempt.json').read_text())['execution']['exit_status'] == 124


def test_nonzero_and_missing_output_are_retained_as_failed_attempt(tmp_path, monkeypatch):
    output = tmp_path / 'results'
    output.mkdir()
    item = {
        'schedule_index': 2, 'system': 'ours', 'slot': 'fresh_1',
        'sequence': 'exp14', 'repetition': 2,
        'image_digest': 'sha256:' + 'a' * 64,
        'execution_identity': {'revision': 'b' * 40},
        'input': {'semantic_equivalence_sha256': 'c' * 64},
        'profile_canonical_sha256': 'd' * 64,
        'execution_receipt_file_sha256': 'e' * 64,
        'selection_receipt_file_sha256': 'f' * 64,
        'argv': ['docker', 'run', '-v', '/M6A_OUTPUT_PLACEHOLDER:/out'],
        'env': {'OMP_NUM_THREADS': '8'},
        'mounts': [{'source': '/managed/bag', 'target': '/input/bag', 'mode': 'ro'}],
        'gt_blind_guard': {'ground_truth_reachable': False,
                           'gt_device': 1, 'gt_inode': 2,
                           'mount_sources': ['/managed/bag']},
    }

    class Completed:
        returncode = 17

    monkeypatch.setattr(RUNNER.subprocess, 'run', lambda *_args, **_kwargs: Completed())
    report = RUNNER.run_attempt(item, output, 1.0)
    final = output / 'attempt_002'
    assert final.is_dir() and report['completion']['complete'] is False
    assert json.loads((final / 'attempt.json').read_text())['execution']['exit_status'] == 17


def test_existing_attempt_cannot_be_overwritten(tmp_path):
    output = tmp_path / 'results'
    output.mkdir()
    (output / 'attempt_001').mkdir()
    item = {'schedule_index': 1, 'system': 'ours', 'slot': 'fresh_1',
            'sequence': 'exp14', 'repetition': 1}
    with pytest.raises(RUNNER.ContractError, match='already exists'):
        RUNNER.run_attempt(item, output, 1.0)


def test_expected_output_contract_is_fail_closed():
    assert RUNNER.expected_outputs('ours', Path('/out')) == [Path('/out/traj_raw.tum')]
    assert RUNNER.expected_outputs('glim_cpu', Path('/out')) == [
        Path('/out/dump/traj_lidar.txt')]
    assert RUNNER.expected_outputs('fast_livo2', Path('/out')) == [Path('/out/odometry.csv')]


def test_gt_blind_wrappers_require_explicit_mode():
    for name in ('ours_container_gt_blind_run.sh',
                 'glim_container_gt_blind_run.sh',
                 'fast_livo2_container_gt_blind_run.sh'):
        text = (ROOT / 'scripts' / name).read_text()
        if name.startswith('ours_'):
            assert '--gt-blind' in text
            assert 'BAG_PATH is required' in text
        else:
            assert 'GT_BLIND' in text
            assert 'GT_BLIND=1 is required' in text
