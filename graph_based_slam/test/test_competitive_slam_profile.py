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

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
PROFILE_PATH = (
    ROOT / 'configs' / 'slam_benchmark_profiles' / 'competitive_slam_v1.yaml'
)
HILTI_RKO_PATH = ROOT / 'configs' / 'hilti2022' / 'rko_lio_hilti2022_pandar.yaml'


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
    assert len({(slot['dataset'], slot['sequence'])
                for slot in holdouts.values()}) == len(holdouts)
    assert all(
        slot['status'] in {'assigned_inputs_pending_hash', 'frozen'}
        for slot in holdouts.values()
    )
    assert all(slot.get('bag_expected_bytes',
                        slot.get('archive_expected_bytes', 0)) > 0
               for slot in holdouts.values())
    assert all(slot.get('bag_url', slot.get('archive_url', '')).startswith('https://')
               for slot in holdouts.values())
    assert all(len(slot['ground_truth_sha256']) == 64 for slot in holdouts.values())
    for slot in holdouts.values():
        if slot['status'] == 'frozen':
            for key in ('raw_rosbag1_sha256', 'canonical_rosbag2_tree_sha256',
                        'input_manifest_sha256', 'semantic_equivalence_sha256'):
                assert len(slot[key]) == 64
    assert profile['phase_gates']['before_algorithm_tuning'][
        'require_all_holdout_slots_assigned'
    ] is True
    assert profile['phase_gates']['before_algorithm_tuning'][
        'require_all_holdout_inputs_frozen'
    ] is True


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
    assert runtime['fast_livo2_processing_probe_rate'] > 1.0
    assert runtime['fast_livo2_processing_probe_repetitions'] == 3
    assert runtime['fast_livo2_processing_probe_fixed_drain_seconds'] >= 0.0
    assert runtime['fast_livo2_processing_probe_max_ape_drift_percent'] <= 1.0
    aggregation = _profile()['repetition_aggregation']
    assert aggregation['processing_realtime_factor'] == 'median'
    assert aggregation['peak_rss'] == 'maximum'
    assert aggregation['completion_and_failures'] == 'worst_case'
    assert aggregation['mapping_quality'] == 'worst_case'
    assert 'runtime.processing_realtime_factor' in (
        _profile()['required_metrics']['common'])
    assert 'runtime.realtime_factor' not in (
        _profile()['required_metrics']['common'])


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
