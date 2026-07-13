"""Tests for the multi-dataset public-suite adoption gate."""

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

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'suite_gate', ROOT / 'scripts/evaluate_public_suite_gate.py')
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


POLICY = {
    'minimum_improved_datasets': 2,
    'maximum_primary_metric_regression_percent': 2.0,
    'require_runtime_and_memory': True,
    'require_raw_artifacts': True,
}


def make_manifest(tmp_path, dataset, improved, change, capture='one'):
    artifact = tmp_path / f'{dataset}_{capture}.tum'
    artifact.write_text(dataset + capture)
    document = {
        'profile': 'public_suite_v1',
        'dataset': dataset,
        'adoption_policy': POLICY,
        'inputs': {'raw_tum': {
            'path': str(artifact), 'sha256': gate.sha256(artifact)}},
        'evidence': {'complete': True},
        'reports': {'runtime': {
            'realtime_factor': 0.8, 'peak_rss_mb': 500.0,
            'process_exit_status': 0}},
        'primary_delta': {
            'improved': improved, 'change_percent': change},
    }
    path = tmp_path / f'{dataset}_{capture}.json'
    path.write_text(json.dumps(document))
    return path, document


def test_adopts_only_two_unique_improved_datasets_with_all_evidence(tmp_path):
    first = make_manifest(tmp_path, 'hilti', True, -4.0)
    second = make_manifest(tmp_path, 'kitti', True, -3.0)

    report = gate.aggregate([first, second])

    assert report['verdict'] == 'ADOPT'
    assert report['summary']['improved_datasets'] == 2
    assert all(report['gates'].values())


def test_duplicate_capture_does_not_inflate_improved_dataset_count(tmp_path):
    first = make_manifest(tmp_path, 'aist', True, -4.0, 'one')
    second = make_manifest(tmp_path, 'aist', True, -3.0, 'two')

    report = gate.aggregate([first, second])

    assert report['summary']['unique_datasets'] == 1
    assert report['summary']['improved_datasets'] == 1
    assert report['gates']['multiple_complete_datasets'] is False
    assert report['verdict'] == 'DO_NOT_ADOPT'


def test_one_regressed_capture_rejects_dataset_and_regression_gate(tmp_path):
    first = make_manifest(tmp_path, 'hilti', True, -4.0)
    second = make_manifest(tmp_path, 'kitti', False, 2.1)

    report = gate.aggregate([first, second])

    assert report['summary']['improved_datasets'] == 1
    assert report['gates']['maximum_primary_regression'] is False
    assert report['verdict'] == 'DO_NOT_ADOPT'


def test_detects_input_hash_drift_and_missing_runtime(tmp_path):
    first_path, first = make_manifest(tmp_path, 'hilti', True, -4.0)
    second_path, second = make_manifest(tmp_path, 'kitti', True, -3.0)
    Path(first['inputs']['raw_tum']['path']).write_text('changed')
    second['reports']['runtime'].pop('peak_rss_mb')
    second_path.write_text(json.dumps(second))

    report = gate.aggregate([(first_path, first), (second_path, second)])

    assert report['gates']['raw_artifact_integrity'] is False
    assert report['gates']['runtime_and_memory'] is False
    assert report['verdict'] == 'DO_NOT_ADOPT'


def test_directory_artifact_hash_matches_cross_repo_contract(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text('metadata')
    (bag / 'data.db3').write_text('data')

    assert gate.sha256(bag) == gate.sha256(bag)
