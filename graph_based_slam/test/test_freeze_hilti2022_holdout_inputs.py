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


"""Tests for strict competitive holdout input freezing."""

import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'freeze_hilti2022_holdout_inputs.py'
SPEC = importlib.util.spec_from_file_location('freeze_holdout', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _fixture(tmp_path: Path, raw: bytes = b'raw-bag'):
    dataset = tmp_path / 'dataset'
    ros2 = dataset / 'exp99_ros2'
    ros2.mkdir(parents=True)
    slug = 'exp99_fixture'
    (dataset / f'{slug}.bag').write_bytes(raw)
    (dataset / f'{slug}_gt.txt').write_text('1 0 0 0 0 0 0 1\n')
    (dataset / 'calibration_files.zip').write_bytes(b'calibration')
    metadata = {'rosbag2_bagfile_information': {
        'storage_identifier': 'sqlite3', 'duration': {'nanoseconds': 20},
        'starting_time': {'nanoseconds_since_epoch': 10}, 'message_count': 2,
        'topics_with_message_count': [
            {'topic_metadata': {'name': '/hesai/pandar'}},
            {'topic_metadata': {'name': '/alphasense/imu'}},
        ]}}
    (ros2 / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
    (ros2 / 'bag.db3').write_bytes(b'converted')
    profile = {'competitive_slam_profile': {
        'name': 'fixture', 'datasets': {'holdout_slots': {'holdout_1': {
            'sequence': 'exp99', 'dataset': 'fixture',
            'bag_url': f'https://example.test/{slug}.bag',
            'ground_truth_url': f'https://example.test/{slug}.txt',
            'bag_expected_bytes': len(raw)}}}}}
    profile_path = tmp_path / 'profile.yaml'
    profile_path.write_text(yaml.safe_dump(profile))
    return dataset, profile_path


def test_manifest_freezes_both_bag_representations_and_metadata(tmp_path):
    dataset, profile = _fixture(tmp_path)
    manifest = MODULE.build_manifest(profile, dataset, 'exp99')
    assert manifest['status'] == 'frozen'
    assert len(manifest['hashes']['raw_rosbag1_sha256']) == 64
    assert len(manifest['hashes']['canonical_rosbag2_tree_sha256']) == 64
    assert manifest['canonical_rosbag2']['message_count'] == 2
    assert manifest['canonical_rosbag2']['topics'] == [
        '/alphasense/imu', '/hesai/pandar']


def test_incomplete_raw_bag_is_rejected_before_hashing(tmp_path):
    dataset, profile = _fixture(tmp_path, raw=b'full-input')
    (dataset / 'exp99_fixture.bag').write_bytes(b'partial')
    with pytest.raises(ValueError, match='byte count mismatch'):
        MODULE.build_manifest(profile, dataset, 'exp99')


def test_deleted_raw_bag_can_only_use_valid_preserved_hash(tmp_path):
    dataset, profile = _fixture(tmp_path)
    raw = dataset / 'exp99_fixture.bag'
    digest = MODULE.sha256_file(raw)
    raw.unlink()
    (dataset / 'exp99_raw_bag.sha256').write_text(digest + '\n')
    manifest = MODULE.build_manifest(profile, dataset, 'exp99')
    assert manifest['hashes']['raw_rosbag1_sha256'] == digest

    (dataset / 'exp99_raw_bag.sha256').write_text('not-a-hash\n')
    with pytest.raises(ValueError, match='invalid raw bag SHA-256'):
        MODULE.build_manifest(profile, dataset, 'exp99')


def test_profile_pinned_ground_truth_hash_drift_is_rejected(tmp_path):
    dataset, profile = _fixture(tmp_path)
    document = yaml.safe_load(profile.read_text())
    document['competitive_slam_profile']['datasets']['holdout_slots'][
        'holdout_1']['ground_truth_sha256'] = '0' * 64
    profile.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='ground_truth_sha256 differs'):
        MODULE.build_manifest(profile, dataset, 'exp99')
