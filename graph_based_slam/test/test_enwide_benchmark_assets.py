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

"""Validate the preregistered ENWIDE SOTA benchmark assets."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[2]
PROFILE = (
    ROOT / 'configs' / 'slam_benchmark_profiles'
    / 'degenerate_lio_sota_v1.yaml'
)
PROFILE_V2 = (
    ROOT / 'configs' / 'slam_benchmark_profiles'
    / 'degenerate_lio_sota_v2.yaml'
)
DOWNLOADER = ROOT / 'scripts' / 'download_enwide.sh'
RUNNER = ROOT / 'scripts' / 'run_enwide_sota_benchmark.sh'
RKO_CONFIG = (
    ROOT / 'configs' / 'enwide' / 'rko_lio_os0_degenerate_sota_v1.yaml'
)
RKO_INTENSITY_V2 = (
    ROOT / 'configs' / 'enwide'
    / 'rko_lio_os0_intensity_exploratory_v2.yaml'
)
RKO_INTENSITY_ALIAS_V3 = (
    ROOT / 'configs' / 'enwide'
    / 'rko_lio_os0_intensity_alias_v3.yaml'
)
RKO_INTENSITY_ALIAS_V4 = (
    ROOT / 'configs' / 'enwide'
    / 'rko_lio_os0_intensity_alias_v4.yaml'
)
RKO_INTENSITY_PEARSON_V5 = (
    ROOT / 'configs' / 'enwide'
    / 'rko_lio_os0_intensity_pearson_v5.yaml'
)
RKO_ORIENTED_GRID_V6 = (
    ROOT / 'configs' / 'enwide'
    / 'rko_lio_os0_oriented_grid_v6.yaml'
)
RKO_PHOTOMETRIC_V7 = (
    ROOT / 'configs' / 'enwide'
    / 'rko_lio_os0_photometric_v7.yaml'
)
RKO_OPEN_GROUND_V8 = (
    ROOT / 'configs' / 'enwide'
    / 'rko_lio_os0_open_ground_v8.yaml'
)


def _profile():
    return yaml.safe_load(PROFILE.read_text())['degenerate_lio_sota_profile']


def test_enwide_profile_preregisters_no_external_velocity_track():
    profile = _profile()
    assert profile['status'] == 'preregistered_report_only'
    assert profile['claim_policy']['sota_claim_allowed'] is False
    assert profile['track']['required_modalities'] == [
        'lidar_geometry', 'lidar_intensity', 'imu',
    ]
    assert set(profile['track']['forbidden_modalities']) == {
        'radar', 'wheel_odometry', 'gnss', 'camera',
    }
    assert profile['track']['maximum_interpolation_time_gap_s'] == 0.11
    assert profile['execution_contract']['parameter_policy'] == (
        'one_frozen_parameter_set'
    )
    assert (
        profile['execution_contract']['minimum_matched_ground_truth_fraction']
        == 0.98
    )
    assert profile['execution_contract']['candidate_config'] == str(
        RKO_CONFIG.relative_to(ROOT)
    )


def test_enwide_profile_freezes_tunnel_inputs_and_published_reference():
    datasets = _profile()['datasets']
    assert set(datasets) == {'enwide_tunnel_s', 'enwide_tunnel_d'}
    assert datasets['enwide_tunnel_s']['expected_bag_bytes'] == 14983936757
    assert datasets['enwide_tunnel_d']['expected_bag_bytes'] == 7485669675
    assert datasets['enwide_tunnel_s']['published_coin_lio'] == {
        'ate_rmse_m': 0.743,
        'rte_percent': 1.60,
    }
    assert datasets['enwide_tunnel_d']['published_coin_lio'] == {
        'ate_rmse_m': 0.487,
        'rte_percent': 1.59,
    }
    assert all(not dataset['tuning_allowed'] for dataset in datasets.values())


def test_enwide_rival_revisions_are_full_commit_hashes():
    for rival in _profile()['rivals'].values():
        revision = rival['revision']
        assert len(revision) == 40
        int(revision, 16)


def test_enwide_downloader_help_is_offline_and_documents_safe_modes():
    completed = subprocess.run(
        ['bash', str(DOWNLOADER), '--help'],
        check=True,
        capture_output=True,
        text=True,
    )
    assert '--metadata-only' in completed.stdout
    assert '--convert' in completed.stdout
    assert 'tunnel_s|tunnel_d|all' in completed.stdout
    text = DOWNLOADER.read_text()
    assert '.ros2-convert.XXXXXX' in text
    assert "'version': importlib.metadata.version('rosbags')" in text


def test_enwide_rko_config_uses_official_os0_extrinsic_and_no_external_velocity():
    config = yaml.safe_load(RKO_CONFIG.read_text())
    assert config['extrinsic_lidar2base_quat_xyzw_xyz'] == [
        0.0, 0.0, 1.0, 0.0, -0.006253, 0.011775, 0.028525,
    ]
    assert config['lidar_timestamps.force_relative'] is True
    assert config['intensity_constraint'] is True
    assert config['radar_velocity_fusion'] is False
    assert config['radar_velocity_continuous_fusion'] is False


def test_enwide_exploratory_v2_only_adds_existing_intensity_gate():
    baseline = yaml.safe_load(RKO_CONFIG.read_text())
    candidate = yaml.safe_load(RKO_INTENSITY_V2.read_text())
    added = {
        key: value for key, value in candidate.items()
        if key not in baseline
    }
    assert added == {
        'intensity_disagreement_gate': True,
        'intensity_disagreement_min_mps': 0.2,
        'intensity_disagreement_min_scans': 3,
        'intensity_disagreement_weight': 1.0,
        'intensity_min_peak_margin': 0.0,
        'intensity_peak_exclusion_radius_bins': 1,
    }
    assert {
        key: value for key, value in candidate.items()
        if key in baseline
    } == baseline


def test_enwide_alias_v3_only_changes_preregistered_peak_margin():
    exploratory = yaml.safe_load(RKO_INTENSITY_V2.read_text())
    alias_aware = yaml.safe_load(RKO_INTENSITY_ALIAS_V3.read_text())

    assert alias_aware['intensity_min_peak_margin'] == 0.005
    assert {
        key: value for key, value in alias_aware.items()
        if key != 'intensity_min_peak_margin'
    } == {
        key: value for key, value in exploratory.items()
        if key != 'intensity_min_peak_margin'
    }


def test_enwide_alias_v4_only_changes_schema_v2_peak_margin():
    exploratory = yaml.safe_load(RKO_INTENSITY_V2.read_text())
    alias_aware = yaml.safe_load(RKO_INTENSITY_ALIAS_V4.read_text())

    assert alias_aware['intensity_min_peak_margin'] == 0.004
    assert {
        key: value for key, value in alias_aware.items()
        if key != 'intensity_min_peak_margin'
    } == {
        key: value for key, value in exploratory.items()
        if key != 'intensity_min_peak_margin'
    }


def test_enwide_pearson_v5_keeps_exploratory_v2_parameters():
    exploratory = yaml.safe_load(RKO_INTENSITY_V2.read_text())
    pearson = yaml.safe_load(RKO_INTENSITY_PEARSON_V5.read_text())

    assert pearson == exploratory


def test_enwide_oriented_grid_v6_only_adds_grid_adapter_parameters():
    pearson = yaml.safe_load(RKO_INTENSITY_PEARSON_V5.read_text())
    grid = yaml.safe_load(RKO_ORIENTED_GRID_V6.read_text())
    added = {
        key: value for key, value in grid.items()
        if key not in pearson
    }

    assert added == {
        'intensity_oriented_grid': True,
        'intensity_grid_half_width_m': 5.0,
        'intensity_grid_max_lateral_shift_m': 0.5,
        'intensity_grid_height_weight': 0.25,
    }
    assert {
        key: value for key, value in grid.items()
        if key in pearson
    } == pearson


def test_enwide_runner_exposes_only_dataset_output_and_repetition_options():
    completed = subprocess.run(
        ['bash', str(RUNNER), '--help'],
        check=True,
        capture_output=True,
        text=True,
    )
    assert '--sequence-dir' in completed.stdout
    assert '--output-dir' in completed.stdout
    assert '--runs' in completed.stdout
    for forbidden_option in (
            '--lidar-topic', '--imu-topic', '--rko-param', '--segment-length'):
        assert forbidden_option not in completed.stdout
    text = RUNNER.read_text()
    assert 'add71ee46322a09139fa95e187d2816ed2c36295' in text
    assert '--completion-end-margin-secs 1.0' in text
    assert '--max-time-gap 0.11' in text
    assert 'warning: position-only scoring failed' in text
    assert '--skip-map-save' in text
    assert "'sota_claim_allowed': False" in text
    assert 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"' in text


def test_enwide_photometric_v7_uses_the_os_sensor_extrinsic_and_beam_model():
    config = yaml.safe_load(RKO_PHOTOMETRIC_V7.read_text())
    # Clouds are published in os_sensor, whose axes match os_imu.
    assert config['extrinsic_lidar2base_quat_xyzw_xyz'] == [
        0.0, 0.0, 0.0, 1.0, -0.006253, 0.011775, -0.007645,
    ]
    assert config['photometric'] is True
    assert config['photometric_scale'] == 0.003
    assert config['photometric_channel'] == 'intensity'
    assert len(config['photometric_model.altitudes_deg']) == 128
    assert len(config['photometric_model.pixel_shift_by_row']) == 128
    assert config['photometric_model.columns'] == 1024
    assert config['photometric_model.beam_offset_mm'] == 27.67
    assert config['photometric_model.cloud_to_lidar_z_m'] == 0.03617
    assert len(config['photometric_image.line_highpass']) == 33
    assert len(config['photometric_image.line_lowpass']) == 33
    assert config['photometric_image.masks'] == [0, 84, 70, 44, 953, 84, 70, 44]
    # No external velocity and none of the inactive v1-v6 degeneracy features.
    assert config['radar_velocity_fusion'] is False
    assert config['radar_velocity_continuous_fusion'] is False
    assert not any(key.startswith(('degeneracy_', 'intensity_')) for key in config)


def test_enwide_open_ground_v8_adds_only_the_open_ground_options_to_v7():
    v7 = yaml.safe_load(RKO_PHOTOMETRIC_V7.read_text())
    v8 = yaml.safe_load(RKO_OPEN_GROUND_V8.read_text())
    added = {
        'bump_image_registration': True,
        'velocity_window_sec': 0.3,
        'skip_registration_after_gap_sec': 0.15,
    }
    assert {key: v8.pop(key) for key in added} == added
    assert v8 == v7


def test_enwide_profile_v2_changes_only_the_candidate():
    v1 = _profile()
    v2 = yaml.safe_load(PROFILE_V2.read_text())['degenerate_lio_sota_profile']
    assert v2['name'] == 'degenerate_lio_sota_v2'
    assert v2['claim_policy']['sota_claim_allowed'] is False
    for section in ('track', 'win_policy', 'rivals', 'required_metrics'):
        assert v2[section] == v1[section], section
    contract = v2['execution_contract']
    assert contract['candidate_config'] == str(
        RKO_PHOTOMETRIC_V7.relative_to(ROOT)
    )
    assert contract['candidate_rko_lio_revision'] == (
        'e9441b33fda002be082ef7f20b0375ee0b700a48'
    )
    datasets = v2['datasets']
    assert datasets['enwide_tunnel_d']['used_for_candidate_development'] is True
    assert datasets['enwide_tunnel_s']['used_for_candidate_development'] is False


def test_enwide_runner_selects_the_frozen_candidate_by_profile():
    text = RUNNER.read_text()
    assert '--profile NAME' in text
    assert 'configs/enwide/rko_lio_os0_photometric_v7.yaml' in text
    assert 'e9441b33fda002be082ef7f20b0375ee0b700a48' in text
    assert 'PROFILE_NAME=degenerate_lio_sota_v1' in text
