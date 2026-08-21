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


"""Contract tests for the frozen GLIM and FAST-LIVO2 competition profile."""

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[2]
PROFILE_PATH = (
    ROOT / 'configs' / 'slam_benchmark_profiles' / 'competitive_slam_v1.yaml'
)
HILTI_RKO_PATH = ROOT / 'configs' / 'hilti2022' / 'rko_lio_hilti2022_pandar.yaml'
EXECUTION_RECEIPT_PATH = ROOT / 'configs' / 'slam_benchmark_profiles' / (
    'competitive_execution_selection_2026-08.yaml')
EXECUTION_CHECKER_PATH = ROOT / 'scripts' / 'check_competitive_execution_selection.py'
_CHECKER_SPEC = importlib.util.spec_from_file_location(
    'competitive_execution_selection_checker', EXECUTION_CHECKER_PATH)
assert _CHECKER_SPEC.loader is not None
_CHECKER = importlib.util.module_from_spec(_CHECKER_SPEC)
_CHECKER_SPEC.loader.exec_module(_CHECKER)


def _profile():
    document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    return document['competitive_slam_profile']


def test_rivals_are_pinned_to_full_git_revisions():
    profile = _profile()
    for rival in profile['rivals'].values():
        assert len(rival['revision']) == 40
        int(rival['revision'], 16)
    assert len(profile['rivals']['glim']['ros2_revision']) == 40
    int(profile['rivals']['glim']['ros2_revision'], 16)
    assert profile['rivals']['glim']['revision'] != profile['rivals']['glim'][
        'annotated_tag_object']


def test_win_policy_requires_three_real_repeated_holdouts():
    profile = _profile()
    policy = profile['win_policy']
    assert profile['repetitions'] == 3
    assert policy['minimum_assigned_holdouts'] >= 3
    assert policy['minimum_holdout_wins'] >= 3
    assert policy['minimum_primary_improvement_percent'] == 3.0
    assert policy['maximum_realtime_factor'] <= 1.0
    assert policy['maximum_peak_rss_ratio_to_rival'] <= 1.2
    assert policy['maximum_mapping_regression_percent'] <= 2.0
    assert policy['maximum_visual_colour_regression_percent'] <= 2.0


def test_seen_datasets_cannot_silently_become_holdouts():
    profile = _profile()
    datasets = profile['datasets']
    seen = set(datasets['bringup']) | set(datasets['development'])
    seen |= set(datasets['regression_only'])
    holdouts = datasets['holdout_slots']
    assert not seen.intersection(holdouts)
    assert len({slot['dataset'] for slot in holdouts.values()}) == len(holdouts)
    assert all(
        slot['status'] in {'assigned_inputs_pending_hash', 'frozen'}
        for slot in holdouts.values()
    )
    assert all(slot['bag_expected_bytes'] > 0 for slot in holdouts.values())
    assert all(slot['bag_url'].startswith('https://') for slot in holdouts.values())
    assert all(
        slot['ground_truth_url'].startswith('https://')
        for slot in holdouts.values()
    )
    assert all(len(slot['ground_truth_sha256']) == 64 for slot in holdouts.values())
    assert all(
        len(slot['calibration_archive_sha256']) == 64
        for slot in holdouts.values()
    )
    assert all(slot['status'] == 'frozen' for slot in holdouts.values())
    for slot in holdouts.values():
        for key in ('raw_rosbag1_sha256', 'canonical_rosbag2_tree_sha256',
                    'input_manifest_sha256', 'semantic_equivalence_sha256'):
            assert len(slot[key]) == 64
    assert profile['phase_gates']['before_algorithm_tuning'][
        'require_all_holdout_slots_assigned'
    ] is True
    assert profile['phase_gates']['before_algorithm_tuning'][
        'require_all_holdout_inputs_frozen'
    ] is True


def test_fresh_v2_slots_are_deep_verified_but_unscored():
    profile = _profile()
    slots = profile['datasets']['fresh_holdout_slots']
    assert len(slots) >= 3
    assert all(slot['status'] == 'frozen_unopened' for slot in slots.values())
    assert [slot['sequence'] for slot in slots.values()] == ['exp14', 'exp16', 'exp18']
    assert all(len(slot['selection_receipt_sha256']) == 64 for slot in slots.values())
    assert all(len(slot['input_manifest_sha256']) == 64 for slot in slots.values())
    assert all(len(slot['ground_truth_sha256']) == 64 for slot in slots.values())
    assert all(len(slot['calibration_archive_sha256']) == 64 for slot in slots.values())
    receipt_path = ROOT / slots['fresh_1']['selection_receipt_path']
    assert receipt_path.is_file()
    receipt = yaml.safe_load(receipt_path.read_text(encoding='utf-8'))
    assert receipt['status'] == 'frozen_unopened'
    assert receipt['selection_decision']['no_performance_data_used'] is True
    assert receipt['selection_decision']['no_ground_truth_content_opened'] is True
    exposed = profile['datasets']['holdout_slots']
    assert not set(slots).intersection(exposed)


def test_execution_selection_receipt_is_registered_and_ready_preflight():
    profile = _profile()
    policy = profile['evidence_gate_v2']
    assert policy['require_execution_selection_receipt'] is True
    path = ROOT / policy['execution_selection_receipt_path']
    assert path == EXECUTION_RECEIPT_PATH
    assert hashlib.sha256(path.read_bytes()).hexdigest() == (
        policy['execution_selection_receipt_sha256'])
    receipt = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert receipt['receipt_kind'] == 'competitive_execution_selection'
    assert receipt['status'] == 'ready'
    assert receipt['systems']['ours']['repository']['revision_status'] == 'pinned'
    assert receipt['systems']['ours']['repository']['worktree_dirty'] is False
    assert receipt['systems']['ours']['repository']['revision'] == (
        '866f733677e92ecb08d67126e463da99dd140d46')
    assert receipt['common_identity']['machine_fingerprint']['status'] == 'ready'
    assert receipt['common_identity']['thread_policy']['status'] == 'ready'
    assert receipt['common_identity']['thread_policy']['cpu_affinity'] == list(range(8))
    assert receipt['common_identity']['thread_policy']['canonical_sha256'] == (
        '262d656a7c382cd27696b3150215ae563a014796de69718754858bcb814ba993')
    enforcement = receipt['common_identity']['thread_policy']['enforcement']
    assert 'taskset' in enforcement['cpu_affinity']
    assert 'docker_cpuset' in enforcement['cpu_affinity']
    assert enforcement['required_before_run'] is True


def test_m6a5_memory_contract_and_campaign_lineage_are_bound():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    contract = receipt['m6a5_memory_contract']
    assert contract['measurement_version'] == 'm6a5-cgroup-v2-memory-v1'
    assert contract['measurement_scope'] == 'container_cgroup_v2'
    assert contract['comparative_rss_field'] == 'container_cgroup_peak_bytes'
    assert contract['docker_client_comparable'] is False
    assert contract['known_allocation_peak_delta_bytes'] > 100 * 1024 * 1024
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['m6a5_memory_contract_and_lineage']['pass'] is True
    assert result['status'] == 'PASS'


def test_m6a5_memory_contract_tamper_is_fail_closed():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    receipt['m6a5_memory_contract']['docker_client_comparable'] = True
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['m6a5_memory_contract_and_lineage']['pass'] is False
    assert any('docker_client_comparable' in item for item in result['errors'])


def test_m6a7_process_rss_contract_and_audit_are_bound():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    contract = receipt['m6a7_process_rss_contract']
    assert contract['status'] == 'PASS'
    assert contract['primary_metric'] == 'aggregate_process_tree_peak_rss_bytes'
    assert contract['primary_metric_definition'] == (
        'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted')
    assert contract['memory_max'] == 'max'
    assert contract['docker_client_comparable'] is False
    assert contract['schedule'] == {
        'order': 'AB_BA_alternating', 'pairs': 20, 'runs': 40,
        'all_complete': True}
    assert contract['blind_scope'] == {
        'ground_truth_content_opened': False, 'scorer_invoked': False,
        'campaign4_started': False}
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['m6a7_process_rss_contract']['pass'] is True
    assert result['status'] == 'PASS'


def test_m6a7_process_rss_contract_tamper_is_fail_closed():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    receipt['m6a7_process_rss_contract']['memory_max'] = '4g'
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['m6a7_process_rss_contract']['pass'] is False
    assert any('memory_max' in item for item in result['errors'])


def test_observed_identity_is_complete_after_external_freeze():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['machine_fingerprint']['pass'] is True
    assert result['checks']['thread_policy_complete']['pass'] is True
    assert result['checks']['system_ours']['evidence']['revision_status'] == 'pinned'
    assert result['status'] == 'PASS'
    assert result['pass'] is True


def test_external_container_config_is_bound_to_immutable_image():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    config = receipt['systems']['fast_livo2']['configs'][1]
    assert config['path_kind'] == 'external_container_absolute_path'
    assert config['hash_kind'] == 'external_container_file_sha256'
    assert config['status'] == 'observed'
    assert config['container_image_digest'] == receipt['systems']['fast_livo2'][
        'container']['image_digest']
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['system_fast_livo2']['pass'] is True

    tampered = copy.deepcopy(receipt)
    tampered['systems']['fast_livo2']['configs'][1][
        'container_image_digest'] = 'sha256:' + '0' * 64
    result = _CHECKER.evaluate(tampered, profile_document)
    assert result['checks']['system_fast_livo2']['pass'] is False
    assert any('container_image_digest does not match image' in item
               for item in result['errors'])


def test_canonical_profile_hash_excludes_only_registered_receipt_sha():
    document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    original = copy.deepcopy(document)
    expected = _CHECKER.canonical_profile_sha256(document)
    assert document == original
    document['competitive_slam_profile']['evidence_gate_v2'][
        'execution_selection_receipt_sha256'] = '0' * 64
    assert _CHECKER.canonical_profile_sha256(document) == expected


def test_canonical_profile_hash_changes_for_non_receipt_mutations():
    document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    expected = _CHECKER.canonical_profile_sha256(document)
    mutations = []
    renamed = copy.deepcopy(document)
    renamed['competitive_slam_profile']['name'] += '-mutated'
    mutations.append(renamed)
    policy_changed = copy.deepcopy(document)
    policy_changed['competitive_slam_profile']['evidence_gate_v2'][
        'repetitions'] += 1
    mutations.append(policy_changed)
    dataset_changed = copy.deepcopy(document)
    dataset_changed['competitive_slam_profile']['datasets']['holdout_slots'][
        'holdout_1']['sequence'] += '-mutated'
    mutations.append(dataset_changed)
    assert all(_CHECKER.canonical_profile_sha256(item) != expected
               for item in mutations)
    assert _CHECKER.PROFILE_CANONICAL_HASH_KIND == 'canonical_profile_sha256_v1'


def test_execution_preflight_pending_observation_remains_incomplete():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    receipt['status'] = 'pending'
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['status'] == 'INCOMPLETE'
    assert result['pass'] is False
    assert result['checks']['receipt_path_and_sha256']['pass'] is True
    assert result['checks']['receipt_status_ready']['pass'] is False


def test_execution_preflight_rejects_receipt_hash_mismatch():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    mutated = copy.deepcopy(profile_document)
    mutated['competitive_slam_profile']['evidence_gate_v2'][
        'execution_selection_receipt_sha256'] = '0' * 64
    result = _CHECKER.evaluate(receipt, mutated)
    assert result['status'] == 'INVALID'
    assert result['pass'] is False
    assert any('receipt SHA' in item for item in result['errors'])


def _mark_system_ready(receipt, system):
    item = receipt['systems'][system]
    item['repository']['revision_status'] = 'ready'
    item['repository']['worktree_dirty'] = False
    for key in ('tracked_diff_sha256', 'untracked_content_sha256',
                'clean_provenance_sha256'):
        item['repository'][key] = 'a' * 64
    item['container']['status'] = 'ready'
    item['container']['image_digest'] = 'sha256:' + 'b' * 64
    item['toolchain']['status'] = 'ready'
    item['toolchain']['fingerprint'] = 'c' * 64


def test_execution_preflight_ready_status_but_dirty_ours_is_not_ready():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    receipt['status'] = 'ready'
    _mark_system_ready(receipt, 'ours')
    receipt['systems']['ours']['repository']['worktree_dirty'] = True
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['system_ours']['pass'] is False
    assert any('worktree must be clean' in item for item in result['errors'])


def test_execution_preflight_pending_container_and_toolchain_cannot_pass():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    _mark_system_ready(receipt, 'glim')
    receipt['systems']['glim']['container']['status'] = 'pending_build'
    receipt['systems']['glim']['toolchain']['status'] = 'pending_build'
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['system_glim']['pass'] is False
    assert any('glim.container.status' in item for item in result['errors'])
    assert any('glim.toolchain.status' in item for item in result['errors'])


def test_execution_preflight_system_diagnostics_are_independent():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    _mark_system_ready(receipt, 'glim')
    receipt['systems']['glim']['repository']['worktree_dirty'] = False
    # The checked-in receipt now has real ours observations; make this system
    # explicitly unresolved so the per-system diagnostic remains meaningful.
    receipt['systems']['ours']['container']['status'] = 'pending_build'
    receipt['systems']['ours']['toolchain']['status'] = 'pending_build'
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['system_glim']['pass'] is True
    assert result['checks']['system_ours']['pass'] is False
    assert result['checks']['all_systems_pinned_and_resolved']['evidence']['per_system'][
        'glim'] is True


def test_execution_preflight_composite_scorer_mismatch_is_invalid():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    receipt['common_identity']['scorer']['canonical_fingerprint'] = '0' * 64
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['status'] == 'INVALID'
    assert result['checks']['scorer_files_and_fingerprint']['pass'] is False
    assert any('canonical scorer file payload' in item
               for item in result['errors'])


def test_execution_preflight_cli_emits_ready_json_yaml_identity(tmp_path):
    json_path = tmp_path / 'preflight.json'
    yaml_path = tmp_path / 'preflight.yaml'
    completed = subprocess.run([
        'python3', str(EXECUTION_CHECKER_PATH),
        '--receipt', str(EXECUTION_RECEIPT_PATH),
        '--profile', str(PROFILE_PATH),
        '--output', str(json_path),
        '--yaml-output', str(yaml_path),
    ], check=False, capture_output=True, text=True)
    assert completed.returncode == 0
    json_result = json.loads(json_path.read_text(encoding='utf-8'))
    yaml_result = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    assert json_result['status'] == 'PASS'
    assert yaml_result['status'] == json_result['status']
    assert json_result['pass'] is True
    for key in ('profile_sha256', 'execution_receipt_sha256',
                'canonical_scorer_fingerprint',
                'thread_policy_canonical_sha256'):
        assert key in json_result['identity']


def test_comparison_modalities_are_not_mixed_between_rivals():
    tracks = _profile()['tracks']
    assert tracks['glim_cpu_lidar_imu']['required_modalities'] == ['lidar', 'imu']
    assert tracks['fast_livo2_lidar_imu_visual']['required_modalities'] == [
        'lidar', 'imu', 'monocular_camera'
    ]


def test_replay_pacing_is_not_mislabeled_as_processing_rtf():
    runtime = _profile()['runtime_policy']
    assert runtime['gate_metric'] == 'runtime.processing_realtime_factor'
    assert runtime['replay_wall_realtime_factor_is_diagnostic_only'] is True
    assert runtime['maximum_trajectory_end_gap_seconds'] <= 0.25
    aggregation = _profile()['repetition_aggregation']
    assert aggregation['processing_realtime_factor'] == 'median'
    assert aggregation['peak_rss'] == 'maximum'
    assert aggregation['completion_and_failures'] == 'worst_case'
    assert aggregation['mapping_quality'] == 'worst_case'


def test_cross_ros_comparison_requires_semantic_sensor_identity():
    execution = _profile()['execution_contract']
    assert execution['same_sensor_messages'] is True
    assert execution['cross_ros_representation_policy'] == (
        'canonical_deserialized_message_digest')
    assert execution['require_cross_ros_topic_counts_equal'] is True
    assert execution['require_cross_ros_record_timestamps_equal'] is True
    assert execution['require_cross_ros_all_sensor_fields_equal'] is True
    assert execution['ignored_cross_ros_transport_fields'] == [
        'std_msgs/Header.seq']


def test_competition_rko_config_explicitly_disables_relocalization_scope():
    config = yaml.safe_load(HILTI_RKO_PATH.read_text(encoding='utf-8'))
    assert config['enable_kidnap_relocalization'] is False
    assert config['reset_on_registration_failure'] is False
    assert config['relocalize_after_scan_gap'] is False
